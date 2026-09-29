#include "localization_pipeline.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace {

constexpr size_t kMaxPendingCamera = 40;      // 2초(IMU가 멈추면 오래된 것부터 버린다)
constexpr double kControlHistoryS = 1.0;
constexpr double kMaxControlGapS = 0.1;       // 이보다 먼 제어 상태와는 짝짓지 않는다
constexpr unsigned kLagPointsPerEstimate = 5;  // 20 Hz 점, 4 Hz 추정(상류 lagd)

}  // namespace

LocalizationPipeline::LocalizationPipeline(const LateralLagConfig &lag_config) : lag_(lag_config) {}

void LocalizationPipeline::on_control(const ControlState &control)
{
    const double v = std::isfinite(control.ego_speed_kph) ? control.ego_speed_kph / 3.6 : 0.0;
    estimator_.handle_car_speed(v);
    ControlSample s;
    s.t = static_cast<double>(control.timestamp_ns) * 1e-9;
    s.active = control.active != 0;
    s.pressed = std::abs(control.driver_torque) > kSteeringPressedTorque;
    s.saturated = std::fabs(control.normalized_output) >= 0.999f;
    s.desired_curvature = std::isfinite(control.desired_curvature) ? control.desired_curvature : 0.0;
    s.v_ego = v;
    if (!control_history_.empty() && s.t < control_history_.back().t) control_history_.clear();
    control_history_.push_back(s);
    while (control_history_.size() > 1 && s.t - control_history_.front().t > kControlHistoryS)
        control_history_.pop_front();
}

void LocalizationPipeline::on_model(const ModelState &model)
{
    calib_rpy_[0] = model.calibration.roll;
    calib_rpy_[1] = model.calibration.pitch;
    calib_rpy_[2] = model.calibration.yaw;
    calib_valid_ = model.calibration.status == 1;
    estimator_.handle_calibration(calib_rpy_);
    if (!model.pose.valid || model.capture_timestamp_ns == 0) return;
    CameraOdometry c;
    c.capture_s = static_cast<double>(model.capture_timestamp_ns) * 1e-9;
    std::copy(model.pose.trans, model.pose.trans + 3, c.trans);
    std::copy(model.pose.rot, model.pose.rot + 3, c.rot);
    std::copy(model.pose.trans_std, model.pose.trans_std + 3, c.trans_std);
    std::copy(model.pose.rot_std, model.pose.rot_std + 3, c.rot_std);
    pending_camera_.push_back(c);
    while (pending_camera_.size() > kMaxPendingCamera) pending_camera_.pop_front();
}

void LocalizationPipeline::flush_camera_until(double t)
{
    while (!pending_camera_.empty() &&
           pending_camera_.front().capture_s - LocationEstimator::kCamOdoPoseDelay <= t) {
        const CameraOdometry &c = pending_camera_.front();
        estimator_.handle_camera_odometry(c.capture_s, c.trans, c.rot, c.trans_std, c.rot_std);
        pending_camera_.pop_front();
    }
}

const LocalizationPipeline::ControlSample *LocalizationPipeline::control_at(double t) const
{
    // t 이전의 가장 늦은 제어 상태(그 시각에 내려진 명령)
    const ControlSample *best = nullptr;
    for (const ControlSample &s : control_history_) {
        if (s.t > t) break;
        best = &s;
    }
    return best && t - best->t <= kMaxControlGapS ? best : nullptr;
}

void LocalizationPipeline::feed_lag(double t)
{
    const ControlSample *c = control_at(t);
    if (!c) return;
    const LivePoseEstimate pose = estimator_.estimate(t);
    const CalibratedPose cal = calibrate_pose(pose, calib_rpy_);
    LateralLagInput in;
    in.t = t;
    in.lat_active = c->active;
    in.steering_pressed = c->pressed;
    in.saturated = c->saturated;
    in.desired_curvature = c->desired_curvature;
    in.v_ego = c->v_ego;
    in.yaw_rate = cal.angular_velocity[2];
    in.yaw_rate_std = cal.angular_velocity_std[2];
    in.pose_valid = pose.filter_valid && pose.inputs_ok && pose.sensors_ok && pose.posenet_ok;
    in.calib_valid = calib_valid_;
    lag_.update_points(in);
    if (++lag_points_since_estimate_ >= kLagPointsPerEstimate) {
        lag_points_since_estimate_ = 0;
        lag_.update_estimate();
    }
}

bool LocalizationPipeline::on_imu(const ImuBatch &batch, double now_s, LocalizationState *out)
{
    const uint32_t count = std::min<uint32_t>(batch.count, kImuBatchMaxSamples);
    if (count == 0) return false;
    const double dt = lag_.config().dt;
    double last_t = 0.0;
    for (uint32_t i = 0; i < count; ++i) {
        const ImuSample &s = batch.samples[i];
        const double t = static_cast<double>(s.timestamp_ns) * 1e-9;
        flush_camera_until(t);
        estimator_.handle_imu(t, s.accel_mps2, s.gyro_rad_s);
        last_t = t;
        /* lagd 점은 IMU 묶음 주기와 무관하게 dt 격자로 넣는다(상관의 지연은 점 간격이 dt라고
         * 가정한다). 멈췄다가 다시 오면 격자를 새로 잡는다. */
        if (next_lag_t_ <= 0.0 || t - next_lag_t_ > 1.0 || t < next_lag_t_ - 2.0 * dt) next_lag_t_ = t;
        if (t >= next_lag_t_) {
            feed_lag(t);
            next_lag_t_ += dt;
        }
    }

    const LivePoseEstimate pose = estimator_.estimate(now_s);
    const CalibratedPose cal = calibrate_pose(pose, calib_rpy_);

    LocalizationState &o = *out;
    o = LocalizationState{};
    o.timestamp_ns = static_cast<uint64_t>(std::llround(last_t * 1e9));
    o.flags = (pose.filter_valid ? kLocalizationFilterValid : 0U) | (pose.inputs_ok ? kLocalizationInputsOk : 0U) |
              (pose.sensors_ok ? kLocalizationSensorsOk : 0U) | (pose.posenet_ok ? kLocalizationPosenetOk : 0U) |
              (calib_valid_ ? kLocalizationCalibValid : 0U) | (lag_restored_ ? kLocalizationLagRestored : 0U);
    for (int i = 0; i < 3; ++i) {
        o.orientation_calib[i] = static_cast<float>(cal.orientation[i]);
        o.orientation_std[i] = static_cast<float>(pose.orientation_ned_std[i]);
        o.angular_velocity_calib[i] = static_cast<float>(cal.angular_velocity[i]);
        o.angular_velocity_calib_std[i] = static_cast<float>(cal.angular_velocity_std[i]);
        o.velocity_device[i] = static_cast<float>(pose.velocity_device[i]);
        o.velocity_device_std[i] = static_cast<float>(pose.velocity_device_std[i]);
        o.acceleration_calib[i] = static_cast<float>(cal.acceleration[i]);
    }
    const LateralLagOutput lag = lag_.output();
    o.lag_status = static_cast<uint32_t>(lag.status);
    o.lateral_delay_s = static_cast<float>(lag.lateral_delay);
    o.lag_estimate_s = static_cast<float>(lag.estimate);
    o.lag_estimate_std_s = static_cast<float>(lag.estimate_std);
    o.lag_valid_blocks = lag.valid_blocks;
    o.lag_cal_perc = lag.cal_perc;
    o.lag_points = static_cast<uint32_t>(lag_.okay_points());
    return true;
}
