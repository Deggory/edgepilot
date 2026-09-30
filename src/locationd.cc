/* locationd: 보드 IMU·모델 pose·제어 상태로 자세 칼만 필터(상류 locationd)와 조향 지연
 * 추정(상류 lagd)을 돌려 /edgepilot_localization에 발행한다. 본체는 LocalizationPipeline이고
 * 이 파일은 공유 메모리 입출력과 lagd 저장만 맡는다.
 *
 * 지금은 관찰 전용이다: controlsd는 이 출력을 쓰지 않는다. 입력이 없으면(IMU 없음 등)
 * 발행하지 않고 기다린다. lagd 추정은 params/live_delay.json에 60초마다와 종료 때 저장하고,
 * 시작 때 초기값(steering.json steer_actuator_delay)이 같으면 이어서 쓴다. */
#include "ipc_channels.h"
#include "ipc_messages.h"
#include "localization_pipeline.h"
#include "utils_json.h"
#include "utils_process.h"
#include "utils_time.h"

#include <signal.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

namespace {

volatile sig_atomic_t g_stop = 0;

constexpr uint64_t kPersistIntervalNs = 60'000'000'000ULL;
constexpr uint64_t kLogIntervalNs = 10'000'000'000ULL;

std::string read_file(const std::string &path)
{
    std::ifstream file(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

bool write_file_atomic(const std::string &path, const std::string &content)
{
    const std::string tmp = path + ".tmp";
    {
        std::ofstream file(tmp, std::ios::binary | std::ios::trunc);
        if (!(file << content)) return false;
    }
    return std::rename(tmp.c_str(), path.c_str()) == 0;
}

void open_optional(LatestChannel &channel, bool *opened, const char *name, size_t size)
{
    if (!*opened) *opened = channel.open(name, size, false);
}

}  // namespace

int main()
{
    install_stop_signal_handlers(&g_stop);

    LateralLagConfig lag_config;
    float actuator_delay = 0.0f;
    if (parse_json_float_value(read_file(param_path("steering.json")), "steer_actuator_delay", &actuator_delay) &&
        actuator_delay > 0.0f && actuator_delay < 1.0f)
        lag_config.initial_lag = actuator_delay;
    LocalizationPipeline pipeline(lag_config);

    const std::string lag_path = param_path("live_delay.json");
    const std::string lag_cache = read_file(lag_path);
    const bool restored = !lag_cache.empty() && pipeline.lag().restore(lag_cache);
    if (!lag_cache.empty() && !restored) std::remove(lag_path.c_str());
    pipeline.set_lag_restored(restored);
    {
        const LateralLagOutput o = pipeline.lag().output();
        std::fprintf(stderr, "locationd: initial_lag=%.3f restored=%d lateral_delay=%.3f blocks=%d\n",
                     lag_config.initial_lag, restored ? 1 : 0, o.lateral_delay, o.valid_blocks);
    }

    LatestChannel pub;
    if (!pub.open(kLocalizationStateTopic, sizeof(LocalizationState), true)) {
        std::fprintf(stderr, "locationd: cannot open %s\n", kLocalizationStateTopic);
        return 1;
    }
    LatestChannel imu_sub, model_sub, control_sub;
    bool imu_open = false, model_open = false, control_open = false;
    uint64_t imu_seq = 0, model_seq = 0, control_seq = 0;
    uint64_t next_persist_ns = monotonic_now_ns() + kPersistIntervalNs;
    uint64_t next_log_ns = monotonic_now_ns() + kLogIntervalNs;
    unsigned long long batches = 0, published = 0;
    std::string last_saved = lag_cache;
    LocalizationState last{};
    LocationInputCounters last_counters;
    const auto d = [](uint64_t now_count, uint64_t before) {
        return static_cast<unsigned long long>(now_count - before);
    };

    auto persist = [&]() {
        const std::string json = pipeline.lag().cache_json();
        if (json == last_saved) return;
        if (write_file_atomic(lag_path, json)) last_saved = json;
        else std::fprintf(stderr, "locationd: cannot write %s\n", lag_path.c_str());
    };

    while (!g_stop) {
        open_optional(imu_sub, &imu_open, kImuTopic, sizeof(ImuBatch));
        open_optional(model_sub, &model_open, kModelStateTopic, sizeof(ModelState));
        open_optional(control_sub, &control_open, kControlStateTopic, sizeof(ControlState));
        if (!imu_open) {
            usleep(200000);
            continue;
        }
        // 제어 상태는 100 Hz라 5 ms마다 본다(몇 개 놓쳐도 lagd는 0.1초 안의 값과 짝짓는다).
        ControlState control;
        if (control_open && control_sub.read_new(&control_seq, &control, sizeof(control), 0))
            pipeline.on_control(control);
        ModelState model;
        if (model_open && model_sub.read_new(&model_seq, &model, sizeof(model), 0)) pipeline.on_model(model);
        ImuBatch batch;
        if (!imu_sub.read_new(&imu_seq, &batch, sizeof(batch), 5)) continue;
        ++batches;
        LocalizationState out{};
        if (pipeline.on_imu(batch, static_cast<double>(monotonic_now_ns()) * 1e-9, &out)) {
            if (pub.publish(&out, sizeof(out))) ++published;
            last = out;
        }

        const uint64_t now = monotonic_now_ns();
        if (now >= next_persist_ns) {
            next_persist_ns = now + kPersistIntervalNs;
            persist();
        }
        if (now >= next_log_ns) {
            next_log_ns = now + kLogIntervalNs;
            std::fprintf(stderr,
                         "locationd: batches=%llu published=%llu flags=0x%x inputs=0x%x yaw=%.4f roll=%.2fdeg "
                         "pitch=%.2fdeg v=%.1f lag status=%u delay=%.3f est=%.3f+-%.3f blocks=%d points=%u\n",
                         batches, published, last.flags, last.input_flags, last.angular_velocity_calib[2],
                         last.orientation_calib[0] * 57.29578f, last.orientation_calib[1] * 57.29578f,
                         last.velocity_device[0], last.lag_status, last.lateral_delay_s, last.lag_estimate_s,
                         last.lag_estimate_std_s, last.lag_valid_blocks, last.lag_points);
            // 지난 로그 이후 입력별 받음/거부(사유별)
            const LocationInputCounters &c = pipeline.estimator().counters();
            const LocationInputCounters &p = last_counters;
            std::fprintf(stderr,
                         "locationd: inputs accel ok=%llu ts=%llu sanity=%llu filter=%llu | gyro ok=%llu ts=%llu "
                         "sanity=%llu cross=%llu filter=%llu | camera ok=%llu ts=%llu sanity=%llu filter=%llu "
                         "speed_guard=%llu\n",
                         d(c.accel_ok, p.accel_ok), d(c.accel_timestamp, p.accel_timestamp),
                         d(c.accel_sanity, p.accel_sanity), d(c.accel_filter, p.accel_filter),
                         d(c.gyro_ok, p.gyro_ok), d(c.gyro_timestamp, p.gyro_timestamp),
                         d(c.gyro_sanity, p.gyro_sanity), d(c.gyro_cross_check, p.gyro_cross_check),
                         d(c.gyro_filter, p.gyro_filter), d(c.camera_ok, p.camera_ok),
                         d(c.camera_timestamp, p.camera_timestamp), d(c.camera_sanity, p.camera_sanity),
                         d(c.camera_filter, p.camera_filter), d(c.camera_speed_guard, p.camera_speed_guard));
            last_counters = c;
        }
    }
    persist();
    std::fprintf(stderr, "locationd: done\n");
    return 0;
}
