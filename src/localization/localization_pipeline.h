#ifndef LOCALIZATION_PIPELINE_H
#define LOCALIZATION_PIPELINE_H

/* locationd 프로세스의 본체: IMU 묶음·모델 상태·제어 상태를 받아 자세 칼만 필터와 lagd를
 * 돌리고 LocalizationState를 만든다. 하드웨어·공유 메모리와 무관해 녹화 재생(diagnostics/
 * replay_localization)과 검사가 같은 코드를 쓴다.
 *
 * 순서: 카메라 주행거리는 캡처 0.1초 전 시각의 관측이라 IMU보다 늦게 도착한다. 도착 즉시
 * 넣으면 필터가 매번 되감으므로, 대기열에 두었다가 IMU 샘플 사이의 제자리에 끼워 넣는다.
 *
 * lagd 입력: IMU 샘플을 넣는 도중 20 Hz 격자마다 그 시각의 요레이트를 꺼내고, 목표 곡률·조향
 * 상태도 같은 시각의 제어 상태를 이력에서 찾아 짝짓는다(묶음 도착 때 최신 값을 쓰면 묶음
 * 지연만큼 지연이 크게 나오고, 묶음 주기로 넣으면 점 간격이 dt와 달라진다). */

#include "common/ipc_messages.h"
#include "localization/lateral_lag.h"
#include "localization/location_estimator.h"

#include <deque>

class LocalizationPipeline {
public:
    explicit LocalizationPipeline(const LateralLagConfig &lag_config = LateralLagConfig());

    // 제어 상태(100 Hz). 차속과 lagd 입력 이력.
    void on_control(const ControlState &control);
    // 모델 상태(20 Hz). 캘리브레이션은 바로 반영하고 카메라 주행거리는 대기열에 넣는다.
    void on_model(const ModelState &model);
    /* IMU 묶음. 샘플과 그 사이의 카메라 주행거리를 시각 순서로 넣고, 마지막 샘플 시각의
     * 추정을 out에 채운다. now_s는 센서 신선도 판정용 현재 시각. 샘플이 없으면 false. */
    bool on_imu(const ImuBatch &batch, double now_s, LocalizationState *out);

    LateralLagEstimator &lag() { return lag_; }
    const LocationEstimator &estimator() const { return estimator_; }
    void set_lag_restored(bool restored) { lag_restored_ = restored; }

    // |driver_torque| 이 값을 넘으면 핸들 조작(상류 Hyundai STEER_THRESHOLD)
    static constexpr int kSteeringPressedTorque = 150;

private:
    struct CameraOdometry {
        double capture_s;
        float trans[3], rot[3], trans_std[3], rot_std[3];
    };
    struct ControlSample {
        double t;
        bool active, pressed, saturated;
        double desired_curvature, v_ego;
    };
    void flush_camera_until(double t);
    void feed_lag(double t);
    const ControlSample *control_at(double t) const;

    LocationEstimator estimator_;
    LateralLagEstimator lag_;
    std::deque<CameraOdometry> pending_camera_;
    std::deque<ControlSample> control_history_;
    double calib_rpy_[3] = {0, 0, 0};
    bool calib_valid_ = false;
    bool lag_restored_ = false;
    unsigned lag_points_since_estimate_ = 0;
    double next_lag_t_ = 0.0;
};

#endif
