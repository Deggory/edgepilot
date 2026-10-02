/* locationd(PoseKalman·LocationEstimator)와 lagd(LateralLagEstimator) 이식 검사.
 * lagd의 상관·신뢰도는 upstream 식을 그대로 옮긴 파이썬(FFT 방식)과 같은 입력에서 대조한다. */
#include "lateral_lag.h"
#include "localization_pipeline.h"
#include "ipc_messages.h"
#include "location_estimator.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

// 파이썬 upstream 이식(lagd.py actuator_delay, FFT 마스크 정규화 상관)으로 구한 값.
// 같은 식의 입력: 0.33 s 늦고 0.9배인 응답 + 13 rad/s 잡음, 7초마다 한 구간씩 마스크.
TEST(Localization, LagdMatchesUpstreamCorrelation)
{
    const int n = 1200;
    std::vector<double> des(n), act(n);
    std::vector<bool> mask(n);
    for (int i = 0; i < n; ++i) {
        const double t = i * 0.05, td = t - 0.33;
        des[i] = std::sin(0.7 * t) + 0.5 * std::sin(1.9 * t + 1.0);
        act[i] = 0.9 * (std::sin(0.7 * td) + 0.5 * std::sin(1.9 * td + 1.0)) + 0.05 * std::sin(13.0 * t);
        mask[i] = std::fmod(std::floor(t / 7.0), 3.0) != 0.0;
    }
    auto e = LateralLagEstimator::masked_smooth(des, mask);
    auto a = LateralLagEstimator::masked_smooth(act, mask);
    for (auto *v : {&e, &a})
        for (double &x : *v)
            if (!std::isfinite(x)) x = 0.0;
    const auto r = LateralLagEstimator::actuator_delay(e, a, mask, 0.05, 0.15, 0.65);
    EXPECT_NEAR(r.lag, 0.330126928, 1e-6);
    EXPECT_NEAR(r.corr, 0.998941686, 1e-6);
    EXPECT_NEAR(r.confidence, 0.6, 1e-9);
}

// 20 Hz로 60초 넘게 조건을 만족하는 주행을 넣으면 블록이 차고 진짜 지연을 낸다. 두 사인파의
// 합은 상관 봉우리가 넓어 upstream 신뢰도가 0.6이라, 블록 로직을 보려고 기준만 0.5로 낮춘다
// (신뢰도 계산 자체는 위 검사가 upstream과 대조한다).
TEST(Localization, LagdEstimatesDelayFromDriving)
{
    LateralLagConfig config;
    config.initial_lag = 0.34;
    config.min_confidence = 0.5;
    LateralLagEstimator lag(config);
    const double true_lag = 0.28, v = 20.0;
    auto kappa = [](double t) { return 0.002 * std::sin(0.9 * t) + 0.001 * std::sin(2.3 * t + 0.5); };
    int estimates = 0;
    for (int i = 0; i < 20 * 400; ++i) {
        const double t = 100.0 + i * 0.05;
        LateralLagInput in;
        in.t = t;
        in.lat_active = true;
        in.v_ego = v;
        in.desired_curvature = kappa(t);
        in.yaw_rate = kappa(t - true_lag) * v;
        in.pose_valid = in.calib_valid = true;
        lag.update_points(in);
        if (i % 5 == 0 && lag.update_estimate()) ++estimates;
    }
    const LateralLagOutput out = lag.output();
    EXPECT_GE(out.valid_blocks, 5) << "추정 " << estimates << "개";
    EXPECT_EQ(out.status, LateralLagStatus::Estimated);
    EXPECT_NEAR(out.lateral_delay, true_lag, 0.02);
    // 저장한 추정으로 다시 시작하면 블록과 값을 이어 간다
    LateralLagEstimator again(config);
    ASSERT_TRUE(again.restore(lag.cache_json()));
    EXPECT_EQ(again.output().valid_blocks, out.valid_blocks);
    EXPECT_NEAR(again.output().lateral_delay, out.estimate, 1e-5);
}

TEST(Localization, LagdIgnoresSteeringOverrideAndLowSpeed)
{
    LateralLagEstimator lag;
    for (int i = 0; i < 20 * 200; ++i) {
        const double t = 50.0 + i * 0.05;
        LateralLagInput in;
        in.t = t;
        in.lat_active = true;
        in.steering_pressed = (i / 100) % 2 == 0;  // 5초마다 번갈아 핸들을 잡는다
        in.v_ego = 5.0;                             // 저속
        in.desired_curvature = 0.002 * std::sin(t);
        in.yaw_rate = in.desired_curvature * in.v_ego;
        in.pose_valid = in.calib_valid = true;
        lag.update_points(in);
        if (i % 5 == 0) lag.update_estimate();
    }
    EXPECT_EQ(lag.output().valid_blocks, 0) << "저속이면 쓰지 않는다";
    EXPECT_NEAR(lag.output().lateral_delay, 0.34, 1e-9) << "추정 전에는 초기값";
}

// 정차: 기울어진 보드의 가속도(중력)와 자이로 바이어스를 넣으면 roll·pitch와 바이어스가 수렴한다.
TEST(Localization, PoseKalmanConvergesAtRest)
{
    LocationEstimator loc;
    const double pitch = 3.0 * kPi / 180.0, roll = -1.5 * kPi / 180.0;
    const double g = 9.81;
    // 기기 좌표계 중력 측정: device_from_ned · (0,0,−g)
    const double ax = g * std::sin(pitch), ay = -g * std::cos(pitch) * std::sin(roll),
                 az = -g * std::cos(pitch) * std::cos(roll);
    const double bias_z = 0.02;  // rad/s, 기기 z
    // 칩 좌표로 되돌린다: device = (−z, x, −y) → chip = (y_dev, −z_dev, −x_dev)
    const float accel_chip[3] = {static_cast<float>(ay), static_cast<float>(-az), static_cast<float>(-ax)};
    const float gyro_chip[3] = {0.0f, static_cast<float>(-bias_z), 0.0f};
    const float zero[3] = {0, 0, 0}, small[3] = {0.05f, 0.05f, 0.05f};
    const double calib[3] = {0, 0, 0};
    loc.handle_calibration(calib);
    loc.set_imu_extrinsic(calib);  // 칩 축만 본다
    for (int i = 0; i < 104 * 60; ++i) {
        const double t = 10.0 + i / 104.0;
        loc.handle_car_speed(t, 0.0, true);  // 정차(CAN 차속 0)
        loc.handle_imu(t, accel_chip, gyro_chip);
        if (i % 5 == 0) loc.handle_camera_odometry(t + 0.02, zero, zero, small, small);
    }
    const LivePoseEstimate p = loc.estimate(10.0 + 60.0);
    EXPECT_TRUE(p.filter_valid && p.inputs_ok && p.sensors_ok && p.posenet_ok);
    EXPECT_NEAR(p.orientation_ned[0], roll, 0.2 * kPi / 180.0);
    EXPECT_NEAR(p.orientation_ned[1], pitch, 0.2 * kPi / 180.0);
    EXPECT_NEAR(loc.filter().x()[PoseKalman::kGyroBias + 2], bias_z, 0.003);
    EXPECT_NEAR(p.angular_velocity_device[2], 0.0, 0.003);
}

// 일정한 오른쪽 선회: 자이로와 카메라 회전이 같은 요레이트를 말하면 각속도가 그 값이다.
TEST(Localization, PoseKalmanTracksYawRate)
{
    LocationEstimator loc;
    const double w = 0.1, v = 15.0, g = 9.81;
    const float gyro_chip[3] = {0.0f, static_cast<float>(-w), 0.0f};  // 기기 z(아래) = −칩 y
    // 원심: 기기 y 가속 = w·v(ω×v의 y), 중력 z = −g
    const float accel_chip[3] = {static_cast<float>(w * v), static_cast<float>(g), 0.0f};
    const float trans[3] = {static_cast<float>(v), 0, 0};
    const float rot[3] = {0, 0, static_cast<float>(w)};
    const float stds[3] = {0.1f, 0.1f, 0.1f};
    const double calib[3] = {0, 0, 0};
    loc.handle_calibration(calib);
    loc.set_imu_extrinsic(calib);
    for (int i = 0; i < 104 * 30; ++i) {
        const double t = 20.0 + i / 104.0;
        loc.handle_car_speed(t, v, true);
        loc.handle_imu(t, accel_chip, gyro_chip);
        if (i % 5 == 0) loc.handle_camera_odometry(t + 0.05, trans, rot, stds, stds);
    }
    const LivePoseEstimate p = loc.estimate(50.0);
    EXPECT_NEAR(p.angular_velocity_device[2], w, 0.005);
    EXPECT_NEAR(p.velocity_device[0], v, 0.3);
    const double calib_rpy[3] = {0, 0, 0};
    EXPECT_NEAR(calibrate_pose(p, calib_rpy).angular_velocity[2], w, 0.005) << "보정 좌표계 요레이트";
}

/* 카메라는 IMU 묶음(~100 ms) 앞에 몰려 들어가 마지막 프레임 하나가 자이로 ~10표본을 가린다.
 * 상류(도착 순서, 프레임당 ~5표본)처럼 나쁜 프레임 하나로는 inputs_ok가 내려가지 않고, 연달아
 * 나쁘면 내려간다. */
TEST(Localization, GyroCrossCheckToleratesOneBadCameraFrame)
{
    const double w = 0.1, v = 15.0, g = 9.81;
    const float gyro_chip[3] = {0.0f, static_cast<float>(-w), 0.0f};
    const float accel_chip[3] = {static_cast<float>(w * v), static_cast<float>(g), 0.0f};
    const float trans[3] = {static_cast<float>(v), 0, 0};
    const float rot[3] = {0, 0, static_cast<float>(w)};
    const float stds[3] = {0.1f, 0.1f, 0.1f};
    const float bad_rot[3] = {0, 0, static_cast<float>(w - 0.5)};  // 급회전 중 모델이 틀린 요레이트
    const float tight[3] = {0.001f, 0.001f, 0.001f};  // ×10 → 교차검사 한계 0.3 rad/s
    const double calib[3] = {0, 0, 0};
    for (const int bad_frames : {1, 2}) {
        SCOPED_TRACE(bad_frames);
        LocationEstimator loc;
        loc.handle_calibration(calib);
        loc.set_imu_extrinsic(calib);
        int i = 0;
        auto imu = [&](int n) {
            for (int k = 0; k < n; ++k, ++i) {
                const double t = 20.0 + i / 104.0;
                loc.handle_car_speed(t, v, true);
                loc.handle_imu(t, accel_chip, gyro_chip);
            }
        };
        for (; i < 104 * 20;) {
            const double t = 20.0 + i / 104.0;
            if (i % 5 == 0) loc.handle_camera_odometry(t + 0.1, trans, rot, stds, stds);
            imu(1);
        }
        ASSERT_TRUE(loc.estimate(20.0 + i / 104.0).inputs_ok);
        for (int f = 0; f < bad_frames; ++f) {
            loc.handle_camera_odometry(20.0 + i / 104.0 + 0.1, trans, bad_rot, stds, tight);
            imu(11);  // 묶음 하나가 이 프레임 뒤에 통째로 들어온다
        }
        EXPECT_GE(loc.counters().gyro_cross_check, 11u * bad_frames) << "가린 자이로는 버린다";
        const bool gyro_invalid = (loc.invalid_service_mask() & kLocalizationInvalidGyro) != 0;
        EXPECT_EQ(gyro_invalid, bad_frames == 2);
    }
}

// 늦게 온 관측을 되감아 넣으면, 처음부터 시간순으로 넣은 것과 결과가 같다.
TEST(Localization, PoseKalmanRewindMatchesInOrder)
{
    using K = PoseKalman::Kind;
    PoseKalman in_order, rewound;
    struct Obs {
        double t;
        K kind;
        PoseKalman::Vec3 z;
    };
    std::vector<Obs> obs;
    for (int i = 0; i < 60; ++i) {
        const double t = 1.0 + i * 0.01;
        obs.push_back({t, K::Gyro, {0.001 * i, -0.002, 0.05}});
        obs.push_back({t, K::Accel, {0.1, 0.2, -9.8}});
    }
    const Obs late = {1.25, K::CameraRotation, {0.0, 0.0, 0.06}};
    std::vector<Obs> sorted = obs;
    sorted.push_back(late);
    std::stable_sort(sorted.begin(), sorted.end(), [](const Obs &a, const Obs &b) { return a.t < b.t; });
    for (const Obs &o : sorted) in_order.predict_and_observe(o.t, o.kind, o.z, PoseKalman::default_noise(o.kind));
    for (const Obs &o : obs) rewound.predict_and_observe(o.t, o.kind, o.z, PoseKalman::default_noise(o.kind));
    ASSERT_TRUE(rewound.predict_and_observe(late.t, late.kind, late.z, PoseKalman::default_noise(late.kind)));
    for (int i = 0; i < PoseKalman::kN; ++i) EXPECT_NEAR(rewound.x()[i], in_order.x()[i], 1e-9) << "상태 " << i;
    EXPECT_FALSE(rewound.predict_and_observe(0.2, K::Gyro, {0, 0, 0}, PoseKalman::default_noise(K::Gyro)))
        << "0.8초보다 오래된 관측은 거부";
}

/* 파이프라인: 조향 지연 lag만큼 늦게 따라오는 선회를 IMU 묶음·모델 상태·제어 상태로 넣는다.
 * 모델 상태는 캡처보다 늦게, IMU는 묶음으로 도착한다(보드와 같은 순서). */
struct PipelineRun {
    LocalizationState last{};
    LateralLagOutput lag{};
    int outputs = 0;
};

PipelineRun run_pipeline(double batch_s, double true_lag, double seconds)
{
    LateralLagConfig config;
    config.min_confidence = 0.5;  // 합성 사인파의 신뢰도(LagdEstimatesDelayFromDriving 참고)
    LocalizationPipeline pipeline(config);
    const double v = 20.0, g = 9.81, t0 = 100.0;
    auto kappa = [](double t) { return 0.002 * std::sin(0.9 * t) + 0.001 * std::sin(2.3 * t + 0.5); };
    auto yaw = [&](double t) { return kappa(t - true_lag) * v; };
    PipelineRun run;
    ImuBatch batch{};
    double next_model = t0, next_control = t0, batch_start = t0;
    for (int i = 0; i < static_cast<int>(seconds * 104); ++i) {
        const double t = t0 + i / 104.0;
        while (next_control <= t) {
            ControlState cs{};
            cs.timestamp_ns = static_cast<uint64_t>(next_control * 1e9);
            cs.active = 1;
            cs.vehicle_fresh = 1;
            cs.ego_speed_kph = static_cast<float>(v * 3.6);
            cs.desired_curvature = static_cast<float>(kappa(next_control));
            pipeline.on_control(cs);
            next_control += 0.01;
        }
        // 캡처 0.07초 뒤에 도착하는 모델 상태
        while (next_model + 0.07 <= t) {
            ModelState ms{};
            ms.capture_timestamp_ns = static_cast<uint64_t>(next_model * 1e9);
            ms.calibration.status = 1;
            ms.pose.valid = 1;
            ms.pose.trans[0] = static_cast<float>(v);
            ms.pose.rot[2] = static_cast<float>(yaw(next_model - LocationEstimator::kCamOdoPoseDelay));
            for (int k = 0; k < 3; ++k) ms.pose.trans_std[k] = ms.pose.rot_std[k] = 0.1f;
            pipeline.on_model(ms);
            next_model += 0.05;
        }
        ImuSample &s = batch.samples[batch.count++];
        s.timestamp_ns = static_cast<uint64_t>(t * 1e9);
        const double w = yaw(t);
        // 칩 축: 기기 z(아래) = −칩 y, 기기 y = 칩 x. 원심 w·v, 중력 −g(기기 z)
        s.gyro_rad_s[1] = static_cast<float>(-w);
        s.accel_mps2[0] = static_cast<float>(w * v);
        s.accel_mps2[1] = static_cast<float>(g);
        if (t - batch_start >= batch_s - 1e-9 || batch.count == kImuBatchMaxSamples) {
            if (pipeline.on_imu(batch, t + 0.002, &run.last)) ++run.outputs;
            batch.count = 0;
            batch_start = t;
        }
    }
    run.lag = pipeline.lag().output();
    return run;
}

TEST(Localization, PipelineOutputsPoseAndLagIndependentOfImuBatching)
{
    for (const double batch_s : {0.05, 0.1}) {
        const PipelineRun run = run_pipeline(batch_s, 0.3, 360.0);
        SCOPED_TRACE(batch_s);
        const uint32_t ok = kLocalizationFilterValid | kLocalizationInputsOk | kLocalizationSensorsOk |
                            kLocalizationPosenetOk | kLocalizationCalibValid;
        EXPECT_EQ(run.last.flags & ok, ok);
        // 샘플 간격(9.6 ms)으로 묶음이 조금 길어진다: 묶음마다 한 번 발행
        EXPECT_GT(run.outputs, 360.0 / batch_s * 0.8);
        EXPECT_LE(run.outputs, 360.0 / batch_s);
        EXPECT_NEAR(run.last.velocity_device[0], 20.0, 0.3);
        EXPECT_GE(run.lag.valid_blocks, 5);
        // 묶음 주기와 무관하게 점은 20 Hz 격자라 지연이 같아야 한다
        EXPECT_NEAR(run.lag.lateral_delay, 0.3, 0.03);
        EXPECT_EQ(run.last.lag_valid_blocks, run.lag.valid_blocks);
        EXPECT_EQ(run.last.input_flags, 0U) << "맞는 카메라는 차속 가드에 걸리지 않는다";
    }
}

/* 정차 중 모델이 가짜 움직임(2026-09-29 책상: 전진 15 m/s, 요 −0.25 rad/s)을 내도, CAN 차속이
 * 0이면 카메라를 쓰지 않아 자세가 IMU 중력 기울기에 머물고 입력도 정상으로 남는다.
 * 차속을 모를 때(부팅 직후)도 카메라를 쓰지 않는다. */
TEST(Localization, CameraSpeedGuardKeepsPoseWhenStopped)
{
    const float accel[3] = {0.16f, 9.76f, 0.08f}, gyro[3] = {0.0396f, 0.0157f, -0.0222f};  // 보드 책상 실측
    const float rot_std[3] = {0.0009f, 0.0009f, 0.0009f}, trans_std[3] = {0.05f, 0.05f, 0.05f};
    const double calib[3] = {0, 0, 0};
    // speed_known_from: 이 시각부터 CAN 차속이 들어온다(부팅 직후에는 없다)
    auto run = [&](double speed_known_from, double *roll, double *pitch, bool *inputs_ok, uint64_t *guarded) {
        LocationEstimator loc;
        loc.handle_calibration(calib);
        loc.set_imu_extrinsic(calib);
        for (int i = 0; i < 104 * 240; ++i) {
            const double t = 10.0 + i / 104.0;
            const bool fake = t >= 70.0 && t < 190.0;
            loc.handle_car_speed(t, 0.0, t >= speed_known_from);
            loc.handle_imu(t, accel, gyro);
            if (i % 5 == 0) {
                const float trans[3] = {fake ? 15.0f : 0.0f, 0, 0};
                const float rot[3] = {fake ? 0.015f : 0.0f, fake ? 0.029f : 0.0f, fake ? -0.25f : 0.0f};
                loc.handle_camera_odometry(t + 0.1, trans, rot, trans_std, rot_std);
            }
        }
        const LivePoseEstimate p = loc.estimate(10.0 + 240.0);
        *roll = p.orientation_ned[0] * 180.0 / kPi;
        *pitch = p.orientation_ned[1] * 180.0 / kPi;
        *inputs_ok = p.inputs_ok;
        *guarded = loc.counters().camera_speed_guard;
    };
    // 칩 가속도의 중력 기울기: 기기 (−z, x, −y) = (−0.08, 0.16, −9.76)
    const double true_roll = std::atan2(-0.16, 9.76) * 180.0 / kPi, true_pitch = std::asin(-0.08 / 9.81) * 180.0 / kPi;
    double roll, pitch;
    bool inputs_ok;
    uint64_t guarded;
    run(0.0, &roll, &pitch, &inputs_ok, &guarded);
    EXPECT_NEAR(roll, true_roll, 0.5);
    EXPECT_NEAR(pitch, true_pitch, 0.5);
    EXPECT_TRUE(inputs_ok);
    EXPECT_NEAR(static_cast<double>(guarded), 120.0 * 104.0 / 5.0, 5.0);  // 가짜 구간의 카메라 관측 전부
    /* 부팅: 가짜 움직임이 나오는 동안 차속을 모르다가(10~130 s) 이후 CAN 차속 0이 들어온다.
     * 모르는 동안 카메라를 쓰지 않아 자이로가 버려지지 않고, 차속이 오면 정상 기울기로 돌아온다
     * (예전에는 모르면 가드가 없어 책상처럼 틀어지고 inputs_ok가 7분 가까이 떨어졌다). */
    run(130.0, &roll, &pitch, &inputs_ok, &guarded);
    EXPECT_NEAR(roll, true_roll, 0.5);
    EXPECT_NEAR(pitch, true_pitch, 0.5);
    EXPECT_TRUE(inputs_ok);
    EXPECT_GE(static_cast<double>(guarded), 120.0 * 104.0 / 5.0) << "모르는 동안의 관측은 쓰지 않는다";
}

/* 외부 회전: 칩이 카메라 대비 앞 2.2°·오른쪽 1.0° 기운 보드를 수평에 두면, 원시 가속도는 그만큼
 * 기운 중력을 보이지만 기본 외부 회전을 적용한 추정은 수평이다. */
TEST(Localization, ImuExtrinsicLevelsTiltedChip)
{
    const double r = LocationEstimator::kDefaultImuExtrinsicRpy[0], p = LocationEstimator::kDefaultImuExtrinsicRpy[1];
    const double g = 9.81;
    // 기기(수평) 중력 측정 (0,0,−g)을 IMU 좌표로: imu = device_from_imuᵀ · device
    const double cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p);
    // euler_rotate(r,p,0)ᵀ의 셋째 열 × (−g)
    const double ix = -g * (-sp), iy = -g * (cp * sr), iz = -g * (cp * cr);
    // 기기 → 칩: chip = (y, −z, −x)
    const float accel_chip[3] = {static_cast<float>(iy), static_cast<float>(-iz), static_cast<float>(-ix)};
    const float zero[3] = {0, 0, 0}, small[3] = {0.05f, 0.05f, 0.05f};
    const double calib[3] = {0, 0, 0};
    for (const bool apply : {true, false}) {
        LocationEstimator loc;
        loc.handle_calibration(calib);
        if (!apply) loc.set_imu_extrinsic(calib);
        for (int i = 0; i < 104 * 60; ++i) {
            const double t = 10.0 + i / 104.0;
            loc.handle_car_speed(t, 0.0, true);
            loc.handle_imu(t, accel_chip, zero);
            if (i % 5 == 0) loc.handle_camera_odometry(t + 0.02, zero, zero, small, small);
        }
        const LivePoseEstimate e = loc.estimate(70.0);
        SCOPED_TRACE(apply);
        if (apply) {
            EXPECT_NEAR(e.orientation_ned[0], 0.0, 0.1 * kPi / 180.0);
            EXPECT_NEAR(e.orientation_ned[1], 0.0, 0.1 * kPi / 180.0);
        } else {
            EXPECT_NEAR(e.orientation_ned[0], r, 0.15 * kPi / 180.0) << "보정 없으면 칩 기울기가 그대로";
            EXPECT_NEAR(e.orientation_ned[1], p, 0.15 * kPi / 180.0);
        }
    }
}

}  // namespace
