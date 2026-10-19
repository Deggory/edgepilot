#include "common/app_config.h"
#include "common/build_version.h"
#include "common/device_settings.h"
#include "common/ipc_channels.h"
#include "common/ipc_messages.h"
#include "common/model_output.h"
#include "common/utils_process.h"
#include "common/utils_time.h"
#include "maixcam2/maix_cmm.h"
#include "model/calibration_service.h"
#include "model/flow_worker.h"
#include "model/lead_tracker.h"
#include "model/model_state_fill.h"
#include "model/replay_source.h"
#include "model/supercombo_model.h"

#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;

// EDGEPILOT_RAW_DUMP: replay 중 모델 raw 출력을 SCODMP1로 남긴다.
// tests/gtest_model_output가 이 포맷을 읽어 보드 출력과 호스트
// 기준을 프레임 단위로 비교할 수 있다(모델 교체 검증용).
class RawOutputDump {
public:
    explicit RawOutputDump(const char *path) {
        if (!path || !path[0]) return;
        file_ = std::fopen(path, "wb");
        if (!file_) {
            std::fprintf(stderr, "modeld: cannot open raw dump %s\n", path);
            return;
        }
        std::fwrite("SCODMP1\0", 1, 8, file_);
        const uint32_t placeholder = 0;
        std::fwrite(&placeholder, sizeof(placeholder), 1, file_);  // 원시 float 값
        std::fwrite(&placeholder, sizeof(placeholder), 1, file_);  // frames
    }

    ~RawOutputDump() {
        if (!file_) return;
        std::fseek(file_, 8, SEEK_SET);
        std::fwrite(&raw_size_, sizeof(raw_size_), 1, file_);
        std::fwrite(&frames_, sizeof(frames_), 1, file_);
        std::fclose(file_);
        std::fprintf(stderr, "modeld: raw dump wrote %u frames x %u floats\n", frames_, raw_size_);
    }

    void append(const std::vector<float> &raw) {
        if (!file_) return;
        raw_size_ = static_cast<uint32_t>(raw.size());  // 늘 모델 출력 하나(2576)
        std::fwrite(raw.data(), sizeof(float), raw.size(), file_);
        ++frames_;
    }

private:
    std::FILE *file_ = nullptr;
    uint32_t raw_size_ = 0;
    uint32_t frames_ = 0;
};

// 1초 창 통계. 두 루프(라이브·재생)가 같은 시계로 fps를 센다. 벽시계가 아니라 monotonic_now_ns라
// NTP가 시각을 옮겨도 창 길이와 전체 fps가 흔들리지 않는다.
struct RateWindow {
    uint64_t start_ns = monotonic_now_ns();
    uint64_t last_ns = start_ns;
    unsigned last_processed = 0;
    unsigned last_errors = 0;

    // 1초가 지났으면 창을 닫고 true. window_us는 닫힌 창의 길이.
    bool close_if_due(uint64_t *window_us) {
        const uint64_t now_ns = monotonic_now_ns();
        const uint64_t elapsed_us = (now_ns - last_ns) / 1000ULL;
        if (elapsed_us < 1000000ULL) return false;
        *window_us = elapsed_us;
        last_ns = now_ns;
        return true;
    }

    double total_fps(unsigned processed) const {
        const uint64_t since_start_us = (monotonic_now_ns() - start_ns) / 1000ULL;
        return since_start_us > 0 ? processed * 1000000.0 / since_start_us : 0.0;
    }
};

// controlsd 스냅샷에서 모델 입력용 자차 속도와 desire, 광류 보정기의 직진 조건(곡률·깜빡이), 결합 상태(보정
// 출처 설정을 해제까지 미룬다)를 읽는다. controlsd가 아직 없으면 열릴 때까지 마지막 값을 유지한다. 스냅샷이
// kStaleNs보다 오래되면(controlsd가 죽거나 멈춤) 차 상태를 낡은 것으로 보고 결합도 풀린 것으로 본다: 마지막
// 속도로 광류 보정의 직진 조건이 계속 열려 있지 않게 한다.
class EgoStateReader {
public:
    static constexpr uint64_t kStaleNs = 1'000'000'000ULL;

    void poll() {
        if (!open_) open_ = sub_.open(kControlStateTopic, sizeof(ControlState), false);
        if (!open_) return;
        ControlState control_state;
        const bool read = sub_.read(&control_state, sizeof(control_state));
        if (read) last_stamp_ns_ = control_state.timestamp_ns;
        if (!timestamp_fresh_ns(last_stamp_ns_, monotonic_now_ns(), kStaleNs)) {
            gate_.vehicle_fresh = false;
            engaged_ = false;
            return;
        }
        if (!read) return;
        const float ego_speed_kph =
            control_state.ego_speed_kph > 0.0f ? control_state.ego_speed_kph : control_state.cluster_speed_kph;
        v_ego_ = std::max(0.0f, ego_speed_kph / 3.6f);
        desire_ = static_cast<int>(control_state.desire);
        gate_.v_ego = v_ego_;
        gate_.curvature = control_state.actual_curvature;
        gate_.blinker = control_state.left_blinker != 0 || control_state.right_blinker != 0;
        gate_.desire = control_state.desire != 0;
        gate_.vehicle_fresh = control_state.vehicle_fresh != 0;
        engaged_ = control_state.engaged != 0;
    }
    float v_ego() const { return v_ego_; }
    bool vehicle_fresh() const { return gate_.vehicle_fresh; }
    int desire() const { return desire_; }
    bool engaged() const { return engaged_; }
    const FlowGateInput &flow_gate() const { return gate_; }

private:
    LatestChannel sub_;
    bool open_ = false;
    uint64_t last_stamp_ns_ = 0;
    float v_ego_ = 0.0f;
    int desire_ = 0;
    bool engaged_ = false;
    FlowGateInput gate_;
};

// 웹 기기 설정(display.json). modeld에서는 이것만 파일을 읽어(1초에 한 번 stat) 카메라 장착과 보정 출처가 같은
// 값을 받는다. 재생은 보드 설정을 읽지 않고 EDGEPILOT_CAMERA_OFFSET_M·EDGEPILOT_CAMERA_HEIGHT_M(기본 0, 모델
// 높이)만 쓰며, 보정 출처는 포즈다.
class ModeldSettings {
public:
    explicit ModeldSettings(bool live) : live_(live) {
        if (live_) {
            file_.poll(monotonic_now_ns(), &settings_);
        } else {
            settings_.camera_offset_m = env_setting("EDGEPILOT_CAMERA_OFFSET_M", &DeviceSettings::camera_offset_m);
            settings_.camera_height_m = env_setting("EDGEPILOT_CAMERA_HEIGHT_M", &DeviceSettings::camera_height_m);
        }
        log();
    }

    // 프레임마다 한 번
    void poll() {
        if (live_ && file_.poll(monotonic_now_ns(), &settings_)) log();
    }
    const DeviceSettings &get() const { return settings_; }

private:
    static float env_setting(const char *name, float DeviceSettings::*member) {
        const DeviceSettings defaults;
        const char *text = std::getenv(name);
        if (!text || !*text) return defaults.*member;
        char *end = nullptr;
        const float value = std::strtof(text, &end);
        if (*end != '\0' || !std::isfinite(value)) throw std::runtime_error(std::string(name) + " is not a number");
        return clamp_device_setting(member, value);
    }
    void log() const {
        std::fprintf(stderr, "\nmodeld: settings camera offset=%+.3f m height=%.2f m, calibration source %s (%s)\n",
                     settings_.camera_offset_m, settings_.camera_height_m,
                     settings_.calibration_use_flow ? "flow" : "pose", live_ ? "display.json" : "replay env");
    }

    bool live_ = false;
    DeviceSettingsFile file_;
    DeviceSettings settings_;
};

// 카메라 장착(기기 설정 camera_offset_m·camera_height_m). 바뀐 오프셋과 높이는 초당 kRateMps로 옮긴다: 모델은 t−4·t
// 두 프레임과 4.8초 특징 이력을 보므로 한 번에 바꾸면 차가 옆으로 순간 이동한 것처럼 보여 pose·보정·locationd·경로가
// 튄다(보정 워프도 같은 이유로 천천히 옮긴다, model/calibration_source). 높이도 오프셋이 있으면 워프의 옆 이동을
// 바꾸므로 같이 옮긴다(예전에는 높이만 한 번에 바꿨다). 시작할 때는 첫 프레임부터 그대로 쓴다.
class CameraMount {
public:
    static constexpr float kRateMps = 0.1f;
    static constexpr float kFrameS = 0.05f;

    explicit CameraMount(const DeviceSettings &start)
        : offset_m_(start.camera_offset_m), height_m_(start.camera_height_m) {}

    // 매 프레임 실행 직전. 이번 프레임 워프에 쓴 값이 offset_m()·height_m()이다.
    void apply(SupercomboModel &model, const DeviceSettings &settings) {
        const float step = kRateMps * kFrameS;
        offset_m_ += std::clamp(settings.camera_offset_m - offset_m_, -step, step);
        height_m_ += std::clamp(settings.camera_height_m - height_m_, -step, step);
        model.set_camera_mount(offset_m_, height_m_);
    }
    float offset_m() const { return offset_m_; }
    float height_m() const { return height_m_; }

private:
    float offset_m_ = 0.0f;
    float height_m_ = 0.0f;
};

// 앞차 쪽 입출력
struct LeadIo {
    LeadFlowState flow{};                                     // 광류 작업이 마지막으로 낸 확대율
    LeadFlowInput *next = nullptr;                            // 광류 작업이 다음 카메라 프레임에서 따라갈 앞차(채운다)
    LeadTracker *tracker = nullptr;                           // 앞차 추적기(ModelState.lead_track)
    float ego_mps = std::numeric_limits<float>::quiet_NaN();  // 자차 속도, 모르면 NaN(추적 시작에만 쓴다)
};

// engaged는 controlsd 결합 상태, use_flow는 보정 출처 설정. 워프에 쓴 보정(출처와 상관없이)이 calibration,
// 출처와 포즈 보정기 자신의 추정이 calibration_source, 광류 상태가 flow_calibration이다.
void publish_output(LatestChannel &model_pub, SupercomboModel &model, const ParsedModelOutput &parsed,
                    CalibrationService &calibration, const CameraMount &mount, uint64_t frame_id,
                    uint64_t capture_timestamp_ns, float model_ms, float v_ego, bool engaged, bool use_flow,
                    const LeadIo &lead) {
    calibration.update(parsed, v_ego, engaged, use_flow);
    float input_rpy[3];
    calibration.input_rpy(input_rpy);
    model.set_input_calibration(input_rpy);

    const ProjectionState projection = calibration.projection();

    ModelState state =
        fill_model_state(parsed, projection, calibration.snapshot(), frame_id, capture_timestamp_ns, model_ms);
    state.camera_offset_m = mount.offset_m();
    state.camera_height_m = mount.height_m();
    state.flow_calibration = calibration.flow_state();
    state.calibration_source = calibration.source_state();
    state.lead_flow = lead.flow;
    if (lead.tracker) {  // 모델 앞차 거리와 확대율을 섞는다(비전 크루즈가 쓴다)
        LeadTrackInput in;
        in.t_s = static_cast<double>(capture_timestamp_ns) * 1e-9;
        in.lead_valid = lead_usable(state.lead);
        in.distance_m = state.lead.x;
        in.relative_speed_mps = state.lead.velocity - lead.ego_mps;
        in.flow_valid = state.lead_flow.valid != 0;
        in.flow_frame_id = state.lead_flow.frame_id;
        in.flow_rate = state.lead_flow.expansion_rate;
        lead.tracker->update(in);
        state.lead_track = lead.tracker->state();
    }
    if (lead.next) {  // 광류 작업이 다음 카메라 프레임에서 따라갈 앞차
        lead.next->valid = state.lead.valid != 0;
        lead.next->probability = state.lead.probability;
        lead.next->x = state.lead.x;
        lead.next->y = state.lead.y;
        std::copy(input_rpy, input_rpy + 3, lead.next->rpy);
        lead.next->camera_height_m = mount.height_m();
        lead.next->camera_offset_m = mount.offset_m();
    }
    model_pub.publish(&state, sizeof(state));
}

int run_replay(const AppConfig &config, LatestChannel &model_pub) {
    ReplayNv12Source source(config.replay_nv12_path);
    // 재생 소스 해상도에 맞춘 기본 워프. GDC 워프도 이 크기로 만들어져야
    // 라이브와 같은 경로를 탄다.
    AppConfig replay_config = config;
    replay_config.nv12_width = source.width();
    replay_config.nv12_height = source.height();
    replay_config.set_warp_source(source.width(), source.height());
    std::fprintf(stderr, "modeld replay input format=NV12 frames=%u file=%s\n", source.frame_count(),
                 config.replay_nv12_path.c_str());

    SupercomboModel model(config.axmodel_path.c_str(), replay_config);
    const ModeldSettings settings(false);
    CalibrationService calibration(config, settings.get().calibration_use_flow);
    float initial_rpy[3] = {};
    calibration.input_rpy(initial_rpy);
    model.set_input_calibration(initial_rpy);
    CameraMount mount(settings.get());

    Nv12Frame frame;
    std::vector<float> raw;
    RawOutputDump raw_dump(std::getenv("EDGEPILOT_RAW_DUMP"));
    unsigned processed = 0;
    unsigned errors = 0;
    RateWindow window;

    while (!g_stop && source.read(frame)) {
        mount.apply(model, settings.get());
        const uint64_t t0 = monotonic_now_ns();
        const bool ok = model.run_frame_nv12(frame.data.data(), raw);
        const uint64_t t1 = monotonic_now_ns();
        if (ok) {
            raw_dump.append(raw);
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            // 재생은 처리 시각으로 찍으므로 앞차 추적(시간 간격이 필요하다)은 돌리지 않는다
            const LeadIo lead;
            publish_output(model_pub, model, parsed, calibration, mount, processed, monotonic_now_ns(), model_ms, 0.0f,
                           false, settings.get().calibration_use_flow, lead);
            ++processed;
        } else {
            ++errors;
        }

        uint64_t window_us = 0;
        if (window.close_if_due(&window_us)) {
            std::fprintf(stderr, "modeld replay: frames=%u/%u fps=%.2f errors=%u          \r", processed,
                         source.frame_count(), window.total_fps(processed), errors);
            std::fflush(stderr);
        }
    }

    std::fprintf(stderr, "\nmodeld replay done frames=%u errors=%u fps=%.2f\n", processed, errors,
                 window.total_fps(processed));
    return processed > 0 && errors == 0 ? 0 : 1;
}

// 광류 작업이 앞차 점을 고를 때 읽는 링 슬롯의 CPU 매핑(캐시 없이). camerad가 다시 떠서 물리 주소가 바뀌면 다시
// 매핑한다.
struct RingSlotMaps {
    unsigned long long phys[kFrameSlots] = {};
    uint8_t *virt[kFrameSlots] = {};
    size_t size = 0;

    void attach(FrameRing &ring, unsigned slot) {
        const unsigned long long p = ring.slot_phys(slot);
        if (p == phys[slot] && virt[slot]) return;
        if (virt[slot]) cmm_unmap(virt[slot], size);
        size = ring.frame_bytes();
        virt[slot] = cmm_map(p, size);
        phys[slot] = p;
        ring.attach_slot(slot, virt[slot]);
    }
    ~RingSlotMaps() {
        for (unsigned i = 0; i < kFrameSlots; ++i) cmm_unmap(virt[i], size);
    }
};

int run_live(const AppConfig &config, LatestChannel &model_pub, LatestChannel &record_frame_pub) {
    LatestChannel frame_sub;
    FrameRing frame_ring;
    if (!frame_sub.open(kRoadAiFrameTopic, sizeof(RoadAiFrame), true))
        throw std::runtime_error("open roadAiFrame ipc failed");

    while (!g_stop && !frame_ring.open(false)) {
        std::fprintf(stderr, "modeld: waiting for road ai frame ring\n");
        usleep(500000);
    }
    if (!frame_ring.valid()) return 1;

    SupercomboModel model(config.axmodel_path.c_str(), config);
    ModeldSettings settings(true);
    CalibrationService calibration(config, settings.get().calibration_use_flow);
    float initial_rpy[3] = {};
    calibration.input_rpy(initial_rpy);
    model.set_input_calibration(initial_rpy);
    CameraMount mount(settings.get());
    EgoStateReader ego;
    // 링 슬롯의 CPU 매핑(광류 작업의 앞차 점 고르기). 광류 작업보다 먼저 선언해 나중에 없앤다.
    RingSlotMaps slot_maps;
    // 광류 작업(보정 창과 앞차 확대율). 링 슬롯을 IVE가 직접 읽으므로 frame_ring보다 먼저 없어져야 한다(뒤에
    // 선언). IVE를 쓸 수 없으면 끄고 계속 간다(저장해 둔 보정 추정은 보정 서비스가 그대로 쓴다). 보정을 워프에
    // 쓸지는 웹 기기 설정이고, 앞차 확대율은 ModelState.lead_flow에 기록하고 앞차 추적에 넣는다.
    std::unique_ptr<FlowWorker> flow;
    LeadFlowInput last_lead;   // 직전 모델 출력의 앞차(다음 카메라 프레임에서 따라간다)
    LeadTracker lead_tracker;  // ModelState.lead_track
    if (config.flow_calibration) {
        try {
            const FlowIntrinsics k{config.input_warp_fx, config.input_warp_fy, config.input_warp_cx,
                                   config.input_warp_cy};
            flow =
                std::make_unique<FlowWorker>(k, static_cast<int>(frame_ring.width()),
                                             static_cast<int>(frame_ring.height()), &calibration.stored_flow_blocks());
            // 보정 서비스가 상태를 읽고 블록을 저장하고 초기화를 보낸다. 워커는 calibration보다 늦게 선언돼 먼저
            // 없어지지만, 그 뒤에는 update를 부르지 않고 서비스의 소멸자도 측정원을 건드리지 않는다.
            calibration.attach_flow(flow.get());
            std::fprintf(stderr, "modeld: flow calibration and lead flow on (VPP/IVE pyramid, IVE LK)\n");
        } catch (const std::exception &e) {
            std::fprintf(stderr, "modeld: flow calibration and lead flow off: %s\n", e.what());
        }
    }
    std::vector<float> raw;
    uint64_t last_frame_seq = 0;
    unsigned processed = 0;
    unsigned errors = 0;
    unsigned missed = 0;
    unsigned frame_sync_failures = 0;
    uint64_t last_frame_id = 0;
    bool have_last_frame_id = false;
    RateWindow window;

    std::fprintf(stderr, "modeld: live frame ring slots=%u frame=%ux%u, every frame\n", frame_ring.slot_count(),
                 frame_ring.width(), frame_ring.height());

    while (!g_stop) {
        RoadAiFrame meta;
        if (!frame_sub.read_new(&last_frame_seq, &meta, sizeof(meta), 1000)) {
            std::fprintf(stderr, "modeld: waiting for roadAiFrame\n");
            continue;
        }

        if (have_last_frame_id && meta.frame_id > last_frame_id + 1)
            missed += static_cast<unsigned>(meta.frame_id - last_frame_id - 1);
        have_last_frame_id = true;
        last_frame_id = meta.frame_id;

        // GDC가 링 슬롯(CMM)을 직접 읽는다(복사 없음). camerad가 슬롯을 다시 잡는 동안은 물리 주소가 0이다.
        const unsigned long long slot_phys = frame_ring.slot_phys(meta.slot);
        uint64_t slot_seq = 0;
        if (slot_phys == 0 || !frame_ring.read_begin(meta.slot, meta.frame_id, &slot_seq)) {
            ++frame_sync_failures;
            ++errors;
            continue;
        }
        settings.poll();
        ego.poll();
        model.set_desire(ego.desire());
        mount.apply(model, settings.get());
        if (flow) {
            // Y 평면이 슬롯 앞에 있다(NV12). IVE가 복사를 마친 뒤 슬롯이 그대로인지 seqlock으로 본다.
            slot_maps.attach(frame_ring, meta.slot);  // 슬롯마다 한 번 매핑한다
            FlowFrame job;
            job.y_phys = slot_phys;
            job.y_virt = slot_maps.virt[meta.slot];
            job.stride = static_cast<int>(meta.width);
            job.frame_id = meta.frame_id;
            job.t_s = static_cast<double>(meta.timestamp_ns) * 1e-9;
            job.gate = ego.flow_gate();
            job.lead = last_lead;
            job.still_valid = [&frame_ring, slot = meta.slot, id = meta.frame_id, slot_seq] {
                return frame_ring.read_still_valid(slot, id, slot_seq);
            };
            flow->push(std::move(job));
        }

        // 녹화기(recordd)가 모델이 본 바로 그 프레임을 따라가도록 알린다.
        record_frame_pub.publish(&meta, sizeof(meta));

        const uint64_t t0 = monotonic_now_ns();
        // GDC가 읽는 동안 슬롯이 덮어써졌으면 모델 상태(이미지·특징 큐)에 넣기 전에 버린다.
        const bool ok = model.run_frame_phys(
            slot_phys, raw, [&] { return frame_ring.read_still_valid(meta.slot, meta.frame_id, slot_seq); });
        const uint64_t t1 = monotonic_now_ns();
        if (ok) {
            ParsedModelOutput parsed = ModelOutputParser::parse(raw);
            const float model_ms = static_cast<float>((t1 - t0) / 1000000.0);
            LeadIo lead;
            lead.flow = flow ? flow->lead_state() : LeadFlowState{};
            lead.next = &last_lead;
            lead.tracker = &lead_tracker;
            if (ego.vehicle_fresh()) lead.ego_mps = ego.v_ego();
            publish_output(model_pub, model, parsed, calibration, mount, meta.frame_id, meta.timestamp_ns, model_ms,
                           ego.v_ego(), ego.engaged(), settings.get().calibration_use_flow, lead);
            ++processed;
        } else {
            ++errors;
        }

        uint64_t window_us = 0;
        if (window.close_if_due(&window_us)) {
            std::fprintf(stderr, "modeld: fps=%.2f frames=%u missed=%u sync=%u errors=%u(+%u) last_ms=%.2f          \r",
                         (processed - window.last_processed) * 1000000.0 / window_us, processed, missed,
                         frame_sync_failures, errors, errors - window.last_errors, ok ? (t1 - t0) / 1000000.0 : 0.0);
            std::fflush(stderr);
            window.last_processed = processed;
            window.last_errors = errors;
        }
    }

    std::fprintf(stderr, "\nmodeld done frames=%u missed=%u sync=%u errors=%u fps=%.2f\n", processed, missed,
                 frame_sync_failures, errors, window.total_fps(processed));
    return processed > 0 && errors == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char *argv[]) {
    install_stop_signal_handlers(&g_stop);
    log_build_version("modeld");

    try {
        AppConfig config = AppConfig::from_env(argc, argv);
        LatestChannel model_pub;
        LatestChannel record_frame_pub;
        if (!model_pub.open(kModelStateTopic, sizeof(ModelState), true))
            throw std::runtime_error("open modelState ipc failed");
        if (!record_frame_pub.open(kRecordFrameTopic, sizeof(RoadAiFrame), true))
            throw std::runtime_error("open recordFrame ipc failed");

        if (config.replay_enabled()) return run_replay(config, model_pub);
        return run_live(config, model_pub, record_frame_pub);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "modeld error: %s\n", e.what());
        std::fprintf(stderr, "%s\n", AppConfig::usage(argc > 0 ? argv[0] : "modeld").c_str());
        return 1;
    }
}
