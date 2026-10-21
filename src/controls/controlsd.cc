// controlsd: K7 횡제어 프로세스(100 Hz). 로직은 ControlsTick(controls_tick.h)에 있고, 이 파일은
// 공유 메모리 입출력, 파라미터 파일 감시, 학습 상태 파일 쓰기, 1초 통계만 맡는다.
#include "common/background_writer.h"
#include "common/build_version.h"
#include "common/ipc_channels.h"
#include "common/utils_file.h"
#include "common/utils_process.h"
#include "common/utils_time.h"
#include "controls/controls_tick.h"
#include "controls/lateral_nn_select.h"

#include <signal.h>
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace {

volatile sig_atomic_t g_stop = 0;

// 채널이 열릴 때까지 1초마다 다시 연다. size는 LatestChannel이면 내용 크기, CanQueue면 슬롯 수다.
template <class Channel>
bool open_when_ready(Channel *channel, const char *topic, size_t size, bool create) {
    while (!g_stop) {
        if (channel->open(topic, size, create)) return true;
        std::fprintf(stderr, "controlsd: waiting for %s\n", topic);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return false;
}

void fill_send_batch(CanBatch *batch, const std::vector<CanFrame> &frames) {
    fill_can_batch(batch, frames.data(), frames.size(), [](IpcCanFrame *dst, const CanFrame &src) {
        dst->address = src.address;
        dst->src = src.bus;
        dst->data_len = src.length;
        std::copy_n(src.data.begin(), src.length, dst->data);
    });
}

// 한 틱이 한 일(1초 창에서 가장 느린 틱의 것을 남긴다): 새 모델로 플래너, torqued 적합(4 Hz), 학습 저장(paramsd 60초·
// torqued 12초마다 직렬화), 파라미터·학습 토크 모델 재적용. 아무것도 아니면 plain이다.
enum TickWork : unsigned { kWorkModel = 1U, kWorkFit = 2U, kWorkSave = 4U, kWorkParams = 8U };

std::string work_names(unsigned work) {
    static constexpr const char *kNames[] = {"model", "fit", "save", "params"};
    std::string names;
    for (unsigned i = 0; i < 4; ++i)
        if (work & (1U << i)) names += (names.empty() ? "" : "+") + std::string(kNames[i]);
    return names.empty() ? "plain" : names;
}

// 제어 스레드가 지금까지 빼앗긴(비자발) 문맥 전환 수. 1초에 한 번 읽는다.
long involuntary_switches() {
    struct rusage usage{};
    getrusage(RUSAGE_THREAD, &usage);
    return usage.ru_nivcsw;
}

// 1초 창의 루프 통계. 창이 끝나면 한 줄로 찍고 비운다.
struct TickStats {
    unsigned can_frames = 0;
    unsigned generated_frames = 0;
    unsigned publish_errors = 0;
    unsigned send_queue_full = 0;
    unsigned stale_can_batches = 0;
    unsigned ticks = 0;
    unsigned misses = 0;
    double work_sum_us = 0.0;
    double work_max_us = 0.0;
    // 예정 시각보다 늦게 깨어난 최댓값. 실시간 우선순위가 필요한지는 이 꼬리로 판단한다(커널이 다른 일로 늦게
    // 깨우면 일 자체가 짧아도 LKAS 프레임이 밀린다).
    double wake_max_us = 0.0;
    unsigned slowest_work = 0;  // work_max_us 틱이 한 일(TickWork)

    void account(double work_us, bool missed, double wake_late_us, unsigned work) {
        work_sum_us += work_us;
        if (work_us > work_max_us) {
            work_max_us = work_us;
            slowest_work = work;
        }
        wake_max_us = std::max(wake_max_us, wake_late_us);
        ++ticks;
        if (missed) ++misses;
    }

    // switches는 창 동안 제어 스레드가 선점된 횟수다. 한 틱의 계산은 수십 µs인데 work_max가 ms라면 그 틱이 한 일
    // (slow=)과 선점(nivcsw=)으로 가린다.
    void log(double window_s, unsigned long long tx_depth, unsigned long long rx_depth, unsigned param_generation,
             long switches, const ControlsTick &tick) {
        const LateralControlResult &result = tick.result();
        const PandaGateOutput &panda = tick.panda();
        const LateralTarget &target = tick.target();
        const VehicleCanState &vehicle = tick.vehicle();
        const float road_bank_lat_accel = tick.controller().road_bank_lat_accel();
        const AdaptiveCruiseOutput &adaptive_cruise = tick.adaptive_cruise();
        const AdaptiveCruiseInput &cruise_input = tick.cruise_input();  // 크루즈가 받은 앞차(추적이 있으면 그것)
        std::fprintf(
            stderr,
            "controlsd: hz=%.3f work_avg_us=%.1f work_max_us=%.1f wake_max_us=%.1f "
            "misses=%u can=%u generated=%u errors=%u txFull=%u rxStale=%u "
            "queue=%llu/%llu params=%u "
            "engaged=%u active=%u "
            "panda=%u/%u plan=%u mpc=%u desire=%d "
            "torque=%d/%d driver=%d angle=%.2f "
            "curve=%.6f/%.6f curveVm=%.6f curveYaw=%.6f lat=%.2f bank=%.2f long=%.2f "
            "error=%.6f pathY=%.3f "
            "laneC=%.3f laneW=%.2f lane=%.2f/%.2f "
            "lprob=%.2f/%.2f/%.2f lstd=%.2f/%.2f "
            "cluster=%.1f wheel=%.1f cruise=%u max=%.1f cmd=%.1f target=%.1f "
            "lead=%u/%.1f/%.1f button=%d pedal=%d/%d block=%s slow=%s nivcsw=%ld\n",
            ticks / window_s, work_sum_us / std::max(1U, ticks), work_max_us, wake_max_us, misses, can_frames,
            generated_frames, publish_errors, send_queue_full, stale_can_batches, tx_depth, rx_depth, param_generation,
            result.engaged ? 1 : 0, result.active ? 1 : 0, panda.ready ? 1 : 0, panda.controls_allowed ? 1 : 0,
            target.valid ? 1 : 0, target.mpc_solution_valid ? 1 : 0, static_cast<int>(target.desire),
            result.desired_torque, result.apply_torque, vehicle.driver_torque, vehicle.steering_angle_deg,
            result.desired_curvature, result.actual_curvature, result.actual_curvature_vm, result.actual_curvature_yaw,
            vehicle.lat_accel_mps2, road_bank_lat_accel, vehicle.long_accel_mps2, result.curvature_error,
            target.target_y_m, 0.5 * (target.lane_left_y_m + target.lane_right_y_m), target.lane_width_m,
            target.lane_left_y_m, target.lane_right_y_m, target.lane_left_prob, target.lane_right_prob,
            target.lane_d_prob, target.lane_left_std, target.lane_right_std, result.cluster_speed_kph,
            result.control_speed_kph, adaptive_cruise.active ? 1U : 0U, adaptive_cruise.maximum_speed_kph,
            adaptive_cruise.commanded_speed_kph, adaptive_cruise.target_speed_kph, adaptive_cruise.lead_valid ? 1U : 0U,
            cruise_input.vision_lead_distance_m, cruise_input.vision_lead_relative_speed_mps,
            adaptive_cruise.command_button, vehicle.gas, vehicle.driver_override,
            block_reason_name(result.active_block), work_names(slowest_work).c_str(), switches);
        *this = TickStats{};
    }
};

}  // namespace

int main() {
    install_stop_signal_handlers(&g_stop);
    log_build_version("controlsd");

    try {
        CanQueue can_sub;
        LatestChannel model_sub;
        LatestChannel panda_state_sub;
        CanQueue sendcan_pub;
        LatestChannel control_state_pub;
        LatestChannel learner_state_pub;
        LatestChannel localization_sub;
        bool localization_open = false;
        double next_localization_open_s = 0.0;
        if (!open_when_ready(&can_sub, kCanTopic, kCanQueueSlots, true) ||
            !open_when_ready(&model_sub, kModelStateTopic, sizeof(ModelState), false) ||
            !open_when_ready(&panda_state_sub, kPandaStateTopic, sizeof(PandaState), true) ||
            !open_when_ready(&sendcan_pub, kSendCanTopic, kCanQueueSlots, true) ||
            !open_when_ready(&control_state_pub, kControlStateTopic, sizeof(ControlState), true) ||
            !open_when_ready(&learner_state_pub, kLearnerStateTopic, sizeof(LearnerState), true)) {
            return 0;
        }
        sendcan_pub.reset();

        const bool force_engaged = env_flag("EDGEPILOT_FORCE_ENGAGED", false);
        const ControlParamPaths param_paths{param_path("steering.json"), param_path("adaptive_cruise.json")};
        ControlParams params;
        std::string error;
        if (!load_control_params(param_paths, &params, &error)) throw std::runtime_error(error);
        std::fprintf(stderr, "controlsd: params steering=%s adaptive=%s %s\n", param_paths.steering.c_str(),
                     param_paths.cruise.c_str(), control_params_summary(params).c_str());
        // paramsd·torqued. 사전값은 시작 때 파라미터로 고정한다. 복원이 거부된 저장은 상류처럼
        // 지운다(torqued는 깨진 캐시만, 튜닝이 바뀐 캐시는 둔다). 실패한 쓰기는 learner_store가 다시 쓴다.
        BackgroundWriter learner_store("controlsd: learner write");
        const std::string vehicle_learn_path = param_path("live_parameters.json");
        const std::string torque_learn_path = param_path("live_torque_parameters.bin");
        const std::string vehicle_learn_json = read_text_file(vehicle_learn_path);
        const std::string torque_learn_cache = read_text_file(torque_learn_path);
        ControlsTick tick(params, force_engaged, vehicle_learn_json, torque_learn_cache);
        const LateralLearners &learners = tick.learners();
        if (learners.vehicle_restore_rejected()) learner_store.remove(vehicle_learn_path);
        if (learners.torque_restore_status() == TorqueRestore::Corrupt) learner_store.remove(torque_learn_path);
        std::fprintf(stderr, "controlsd: learners paramsd=%s torqued=%s use_vehicle=%u use_torque=%u\n",
                     learners.vehicle_restored()  ? "restored"
                     : vehicle_learn_json.empty() ? "fresh"
                                                  : "rejected",
                     learners.torque_restore_status() == TorqueRestore::Restored        ? "restored"
                     : learners.torque_restore_status() == TorqueRestore::KeyMismatch   ? "key_mismatch"
                     : learners.torque_restore_status() == TorqueRestore::Corrupt       ? "corrupt"
                     : learners.torque_restore_status() == TorqueRestore::SourceChanged ? "source_changed"
                                                                                        : "fresh",
                     params.steering.use_live_vehicle_params ? 1U : 0U,
                     params.steering.use_live_torque_params ? 1U : 0U);
        // 학습한 토크 모델(docs/lateral-nn.md): 고른 모델을 읽고, 웹 콘솔에서 바꾸면 갈아 끼운다. 없거나 못 읽으면 선형
        // 피드포워드만 쓴다(nn_feedforward를 켜도).
        LateralNnSelector nn_selector;
        if (const auto nn = nn_selector.load_now()) {
            tick.set_lateral_nn(nn);
            std::fprintf(stderr, "controlsd: lateral nn %s (%s, %zu inputs) nn_feedforward=%u\n",
                         nn_selector.loaded_name().c_str(), nn->name().c_str(), nn->input_count(),
                         params.steering.nn_feedforward ? 1U : 0U);
        } else {
            std::fprintf(stderr, "controlsd: lateral nn off: %s\n", nn_selector.last_error().c_str());
        }
        TickStats stats;
        long switches_at_log = involuntary_switches();
        uint64_t model_seq = 0;
        uint64_t panda_state_seq = 0;

        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();
        auto next_tick = start;
        auto log_start = start;
        ControlParamsWatcher param_watcher(param_paths, start);
        // 틱마다 0으로 채우지 않게 루프 밖에 둔다(CAN 묶음 21.5 KB: pop과 fill_send_batch는 count까지 채운다,
        // ModelState 3.7 KB: 새 스냅샷일 때만 복사한다)
        CanBatch can_batch, send_batch;
        ModelState model;
        PandaState panda_state;

        while (!g_stop) {
            const auto scheduled = next_tick;  // 이번 틱이 깨어나야 했던 시각
            next_tick += std::chrono::milliseconds(10);
            const auto work_start = Clock::now();
            const double now_s = std::chrono::duration<double>(work_start - start).count();
            unsigned work = 0;  // 이번 틱이 한 일(TickWork)

            if (const auto reloaded = param_watcher.poll(work_start, params)) {
                params = *reloaded;
                tick.apply_params(params);
                work |= kWorkParams;
            }
            std::shared_ptr<const LateralNnModel> next_nn;
            if (nn_selector.poll(work_start, &next_nn)) {
                tick.set_lateral_nn(next_nn);
                work |= kWorkParams;
            }

            // 신선도는 꺼낸 직후의 시각으로 잰다(pandad 송신 스레드와 같다). 틱 시작 시각이면 그 뒤에 들어온
            // 묶음이 미래 시각이라 버려졌다.
            while (can_sub.pop(&can_batch)) {
                if (!tick.on_can_batch(can_batch, monotonic_now_ns(), now_s)) {
                    ++stats.stale_can_batches;
                    continue;
                }
                stats.can_frames += can_batch.count;
            }
            // seq가 그대로면 복사하지 않는다(read_new, 기다리지 않음)
            if (model_sub.read_new(&model_seq, &model, sizeof(model), 0)) {
                tick.on_model(model);
                work |= kWorkModel;
            }
            if (panda_state_sub.read_new(&panda_state_seq, &panda_state, sizeof(panda_state), 0))
                tick.on_panda(panda_state);
            // locationd가 없어도 제어는 그대로라 붙을 때까지 1초마다 다시 열어 본다.
            if (!localization_open && now_s >= next_localization_open_s) {
                localization_open = localization_sub.open(kLocalizationStateTopic, sizeof(LocalizationState), false);
                next_localization_open_s = now_s + 1.0;
            }
            LocalizationRead localization;
            localization.open = localization_open;
            if (localization_open)
                localization.read = localization_sub.read(&localization.state, sizeof(localization.state));
            // IPC를 읽는 동안 새 모델/Panda 상태가 발행될 수 있으므로 locationd 나이와 freshness 판정에는 공유 상태를
            // 다 읽은 직후의 시간을 사용한다.
            const uint64_t read_ns = monotonic_now_ns();
            localization.read_ns = read_ns;
            tick.on_localization(localization, now_s);
            ControlState control_state = tick.step(now_s, read_ns);
            control_state.timestamp_ns = monotonic_now_ns();
            if (!control_state_pub.publish(&control_state, sizeof(control_state))) {
                ++stats.publish_errors;
            }

            const LateralControlResult &result = tick.result();
            if (result.should_send && !result.frames.empty()) {
                fill_send_batch(&send_batch, result.frames);
                if (!sendcan_pub.push(send_batch)) {
                    ++stats.publish_errors;
                    ++stats.send_queue_full;
                } else {
                    stats.generated_frames += static_cast<unsigned>(result.frames.size());
                }
            }

            // 송신 뒤에 둬서 적합·직렬화 틱이 CAN 송신을 늦추지 않게 한다.
            LearnerOutputs learned = tick.update_learners(now_s);
            if (learned.vehicle_json) learner_store.write(vehicle_learn_path, std::move(*learned.vehicle_json));
            if (learned.torque_cache) learner_store.write(torque_learn_path, std::move(*learned.torque_cache));
            if (learners.torque_published()) work |= kWorkFit;
            if (learners.vehicle_persist_due() || learners.torque_persist_due()) work |= kWorkSave;
            if (learned.learner_state) {
                learned.learner_state->timestamp_ns = monotonic_now_ns();
                if (!learner_state_pub.publish(&*learned.learner_state, sizeof(LearnerState))) ++stats.publish_errors;
            }

            const auto work_end = Clock::now();
            stats.account(
                std::chrono::duration<double, std::micro>(work_end - work_start).count(), work_end > next_tick,
                std::max(0.0, std::chrono::duration<double, std::micro>(work_start - scheduled).count()), work);
            if (work_end - log_start >= std::chrono::seconds(1)) {
                const long switches = involuntary_switches();
                stats.log(std::chrono::duration<double>(work_end - log_start).count(),
                          static_cast<unsigned long long>(sendcan_pub.depth()),
                          static_cast<unsigned long long>(can_sub.depth()), param_watcher.generation(),
                          switches - switches_at_log, tick);
                switches_at_log = switches;
                log_start = work_end;
            }

            // tick을 넘겼으면 밀린 만큼 따라잡지 않고 현재 시각으로 재동기화한다.
            // next_tick을 과거에 둔 채로 두면 다음 몇 번의 반복이 sleep 없이 연속
            // 실행되어 LKAS frame과 counter가 한꺼번에 몰려 나간다.
            const auto tick_end = Clock::now();
            if (next_tick > tick_end) {
                std::this_thread::sleep_until(next_tick);
            } else {
                next_tick = tick_end;
            }
        }
        std::fprintf(stderr, "controlsd: stopping\n");
        // 마지막 저장 뒤 배운 것(paramsd 최대 60초, torqued 12초)을 남기고 쓰기가 끝날 때까지 기다린다
        LearnerOutputs last = tick.final_learner_outputs();
        if (last.vehicle_json) learner_store.write(vehicle_learn_path, std::move(*last.vehicle_json));
        if (last.torque_cache) learner_store.write(torque_learn_path, std::move(*last.torque_cache));
        learner_store.flush();
        return 0;
    } catch (const std::exception &error) {
        std::fprintf(stderr, "controlsd error: %s\n", error.what());
        return 1;
    }
}
