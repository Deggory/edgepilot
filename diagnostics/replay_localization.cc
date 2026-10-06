/* 녹화 경로를 locationd 본체(LocalizationPipeline: 자세 칼만 필터 + lagd)에 기록 순서대로
 * 흘린다. 보드의 locationd와 같은 코드다.
 *
 * 사용: replay_localization <out.csv> <events.bin...>
 * CSV(IMU 묶음마다): t, 속도, CAN 요레이트, locationd 보정 요레이트·표준편차, 롤·피치, 플래그,
 *   lagd 값. 끝에 요레이트 비교와 lagd 결과를 출력한다. (CAN 횡가속 LatAccel은 부호가 반대이고
 *   배율이 맞지 않아 롤 비교 기준으로 쓰지 않는다: 2026-09-27 경로에서 요레이트·속도 대비 −0.35배) */
#include "event_log_reader.h"
#include "ipc_messages.h"
#include "localization_pipeline.h"
#include "recorded_model_state.h"
#include "recorded_vehicle_can.h"
#include "recording_format.h"
#include "vehicle_can.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

struct Stats {
    double n = 0, sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
    void add(double x, double y)
    {
        n += 1; sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
    }
    double corr() const
    {
        const double cx = sxx - sx * sx / n, cy = syy - sy * sy / n;
        return (sxy - sx * sy / n) / std::sqrt(cx * cy);
    }
    double slope() const { return (sxy - sx * sy / n) / (sxx - sx * sx / n); }  // y ≈ slope·x
    double mean_diff() const { return (sy - sx) / n; }
};

}  // namespace

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <out.csv> <events.bin...>\n", argv[0]);
        return 2;
    }
    std::FILE *csv = std::fopen(argv[1], "w");
    if (!csv) return 1;
    std::fprintf(csv, "t,v,can_yaw_right,loc_yaw_right,loc_yaw_std,loc_roll,loc_pitch,flags,loc_vx,"
                      "lag_status,lateral_delay,lag_estimate,lag_std,lag_blocks,lag_points,input_flags\n");

    LocalizationPipeline pipeline;
    VehicleCanState vehicle{};
    long outputs = 0;
    Stats yaw_stats;
    std::vector<char> buf;

    for (int f = 2; f < argc; ++f) {
        EventLogReader reader(argv[f]);
        if (!reader.ok()) continue;
        EventRecordHeader rh{};
        while (reader.next(&rh, &buf)) {
            const double record_s = static_cast<double>(rh.timestamp_ns) * 1e-9;
            const auto type = static_cast<RecordType>(rh.type);
            if (type == RecordType::CanRx) {
                apply_recorded_can(buf.data(), rh.payload_size, record_s, &vehicle);
            } else if (type == RecordType::ModelState) {
                ModelState ms{};
                if (decode_recorded_model_state(buf.data(), rh.payload_size, reader.version(), &ms)) pipeline.on_model(ms);
            } else if (type == RecordType::ControlState) {
                ControlState cs{};
                std::memcpy(&cs, buf.data(), std::min(sizeof(cs), buf.size()));
                pipeline.on_control(cs);
            } else if (type == RecordType::Imu) {
                ImuBatch batch{};
                std::memcpy(&batch, buf.data(), std::min(sizeof(batch), buf.size()));
                LocalizationState o{};
                if (!pipeline.on_imu(batch, record_s, &o)) continue;
                ++outputs;
                const double v = (vehicle.wheel_speed_rl_kph + vehicle.wheel_speed_rr_kph) / 7.2;
                // K7 ESP12 요레이트는 제어 관례와 반대(왼쪽 양수)
                const double can_yaw = -vehicle.yaw_rate_rad_s;
                std::fprintf(csv, "%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%.3f,%u,%.3f,%.3f,%.3f,%d,%u,%u\n",
                             o.timestamp_ns * 1e-9, v, can_yaw, o.angular_velocity_calib[2],
                             o.angular_velocity_calib_std[2], o.orientation_calib[0], o.orientation_calib[1], o.flags,
                             o.velocity_device[0], o.lag_status, o.lateral_delay_s, o.lag_estimate_s,
                             o.lag_estimate_std_s, o.lag_valid_blocks, o.lag_points, o.input_flags);
                const uint32_t ok = kLocalizationFilterValid | kLocalizationInputsOk | kLocalizationSensorsOk |
                                    kLocalizationPosenetOk;
                if ((o.flags & ok) == ok && v > 5.0 && vehicle.yaw_rate_valid)
                    yaw_stats.add(can_yaw, o.angular_velocity_calib[2]);
            }
        }
    }
    std::fclose(csv);
    std::printf("outputs %ld\n", outputs);
    std::printf("yaw rate  locationd vs CAN (n=%.0f): corr %.4f  slope %.4f  mean diff %.5f rad/s\n", yaw_stats.n,
                yaw_stats.corr(), yaw_stats.slope(), yaw_stats.mean_diff());
    const LocationInputCounters &c = pipeline.estimator().counters();
    std::printf("inputs: accel ok %llu rejected %llu | gyro ok %llu cross %llu other %llu | camera ok %llu "
                "ts %llu sanity %llu filter %llu speed_guard %llu\n",
                (unsigned long long)c.accel_ok, (unsigned long long)(c.accel_timestamp + c.accel_sanity + c.accel_filter),
                (unsigned long long)c.gyro_ok, (unsigned long long)c.gyro_cross_check,
                (unsigned long long)(c.gyro_timestamp + c.gyro_sanity + c.gyro_filter), (unsigned long long)c.camera_ok,
                (unsigned long long)c.camera_timestamp, (unsigned long long)c.camera_sanity,
                (unsigned long long)c.camera_filter, (unsigned long long)c.camera_speed_guard);
    const LateralLagOutput o = pipeline.lag().output();
    std::printf("lagd: valid blocks %d, status %u, lateral_delay %.3f s, estimate %.3f +- %.3f\n", o.valid_blocks,
                static_cast<unsigned>(o.status), o.lateral_delay, o.estimate, o.estimate_std);
    return 0;
}
