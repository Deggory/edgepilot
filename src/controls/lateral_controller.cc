#include "controls/lateral_controller.h"

#include "common/model_output.h"
#include "common/utils_math.h"
#include "common/utils_time.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPandaEngageGraceS = 1.0;
// 운전자가 넘겨받은 큰 회전에서 조향을 다시 켜는 각도(carrotpilot lat_suspend_control의 resume_angle)
constexpr float kDriverReleaseAngleDeg = 15.0f;
// 운전자가 넘겨받은 정지라도 핸들이 고장 각도(85도)보다 이만큼 아래이고 계획 곡률이 지금 핸들 곡률의 ±25% 안(같은 쪽)인
// 채로 1초 가면 다시 조향한다. 램프처럼 운전자가 85도를 잠깐 넘긴 뒤 커브를 붙잡고 있는 경우다. 교차로를 빠져나오며
// 펴는 핸들은 계획(직진)과 어긋나 그대로 쉰다: 2026-10-03 2:54 탈출은 계획이 직진인데 핸들이 77도인 채 다시 켜져
// 최대 토크로 핸들을 밀었다. 0.3초면 탈출 중 잠깐 멈춘 핸들에서도 켜졌다(녹화 15개 재생: 0.3초 26번 중 25번이 교차로
// 탈출, 1초는 9번 중 8번이고 기존보다 0.5~2.2초 일찍. 2026-10-05 16:46 램프는 두 경우 다 켜져 7~10초 일찍 돕는다).
constexpr float kHeldTurnResumeMarginDeg = 5.0f;
constexpr float kHeldTurnPlanTolerance = 0.25f;
constexpr int kHeldTurnResumeFrames = 100;

// 학습한 토크 모델 피드포워드를 섞는 무게의 틱당 변화(100 Hz에서 1초 램프)
constexpr float kNnBlendStep = 0.01f;
// 학습 데이터는 3 m/s 위만 썼다(nnlc csvdata.usable_mask). 그 아래에서는 선형 피드포워드로 돌아간다.
constexpr float kNnMinSpeedMps = 3.0f;
// 모델이 읽는 가장 먼 시각에 계획 나이 보정을 더해도 계획 격자 끝(model_t_idx(kLateralControlN - 1) = 2.5 s) 안이다
static_assert(kLateralNnLatestS + kMaxPlanAgeCompS <= 10.0f * (kLateralControlN - 1) * (kLateralControlN - 1) /
                                                              ((kTrajectorySize - 1.0f) * (kTrajectorySize - 1.0f)) +
                                                          1e-4f,
              "lateral NN inputs must stay inside the plan");

// 곡률 상한: 횡가속 ±kMaxLateralAccel(학습 롤만큼 이동, 속도 하한 kMinCurvatureSpeedMps)과 ±kMaxCurvature.
// clip_curvature와 학습한 토크 모델의 입력 궤적이 같이 쓴다.
float limit_curvature(float speed_mps, float curvature, float roll_rad) {
    const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
    const float roll_lat_accel = roll_rad * 9.81f;
    curvature = clamp_float(curvature, (-kMaxLateralAccel + roll_lat_accel) / (speed * speed),
                            (kMaxLateralAccel + roll_lat_accel) / (speed * speed));
    return clamp_float(curvature, -kMaxCurvature, kMaxCurvature);
}

float interp_lateral(float x, const float *values) {
    if (x <= 0.0f) return values[0];
    for (int i = 1; i < kLateralControlN; ++i) {
        const float high_x = model_t_idx(i);
        if (x <= high_x) {
            const float low_x = model_t_idx(i - 1);
            const float p = (x - low_x) / (high_x - low_x);
            return values[i - 1] + p * (values[i] - values[i - 1]);
        }
    }
    return values[kLateralControlN - 1];
}

// SteeringParams의 토크 제한을 CAN 계층 표현으로 옮긴다.
HyundaiSteeringLimits hyundai_limits(const SteeringParams &params) {
    HyundaiSteeringLimits out;
    out.steer_max = params.steer_max;
    out.steer_delta_up = params.steer_delta_up;
    out.steer_delta_down = params.steer_delta_down;
    out.steer_driver_allowance = params.steer_driver_allowance;
    out.steer_driver_multiplier = params.steer_driver_multiplier;
    out.steer_driver_factor = params.steer_driver_factor;
    return out;
}

}  // namespace

LateralController::LateralController(LateralControllerConfig config) : config_(config), clock_(monotonic_now_ns) {}

// 제어 상태를 유지한 채 런타임 파라미터를 즉시 교체한다.
void LateralController::update_params(const SteeringParams &steering_params) {
    config_.steering_params = steering_params;
}

void LateralController::set_live_params(const LiveLateralParams &live, bool vehicle_valid, bool calibrated) {
    live_ = live;
    live_vehicle_valid_ = vehicle_valid;
    live_calibrated_ = calibrated;
}

void LateralController::set_live_delay(float delay_s, bool valid) {
    live_delay_s_ = delay_s;
    live_delay_valid_ = valid && std::isfinite(delay_s);
}

bool LateralController::live_delay_in_use() const {
    return config_.steering_params.use_live_delay && live_delay_valid_;
}

float LateralController::plan_delay_s() const {
    // lagd 지연 범위(lateral_lag.h min_lag~max_lag)
    return live_delay_in_use() ? clamp_float(live_delay_s_, 0.15f, 0.65f)
                               : config_.steering_params.steer_actuator_delay;
}

LiveLateralParams LateralController::live_params() const {
    LiveLateralParams live = live_;
    live.use_vehicle = live_.use_vehicle && config_.steering_params.use_live_vehicle_params;
    live.use_torque = live_.use_torque && config_.steering_params.use_live_torque_params;
    return live;
}

// 차량 버튼/상태와 lane path를 바탕으로 LKAS 제어 결과와 CAN frame을 만든다.
LateralControlResult LateralController::update(const LateralPath &path, const LateralTarget &target,
                                               const VehicleCanState &vehicle_state, double now_s, int frame,
                                               bool panda_ready, bool panda_controls_allowed) {
    const bool engage_requested =
        !config_.force_engaged && !engaged_ && vehicle_state.clu_button == 0 && last_button_ == kCruiseButtonSet;
    update_button_state(vehicle_state.clu_button, now_s);
    const bool logical_engaged = config_.force_engaged || engaged_;

    LateralControlResult result;
    result.engaged = logical_engaged;
    const LateralPath gated_path = debounce_path(path, now_s);
    result.path_usable = gated_path.usable_for_steering;
    result.left_lane = path.left_valid;
    result.right_lane = path.right_valid;
    result.seeds_ready = seed_frames_ready(vehicle_state);
    result.vehicle_fresh = vehicle_state_fresh(vehicle_state, now_s,
                                               static_cast<double>(config_.timing.vehicle_state_timeout_ms) / 1000.0);
    result.cluster_speed_kph = cluster_speed_kph(vehicle_state);
    const float raw_speed_kph =
        vehicle_speed_kph(vehicle_state, now_s, static_cast<double>(config_.timing.vehicle_state_timeout_ms) / 1000.0);
    result.control_speed_kph = static_cast<float>(v_ego_filter_.update(raw_speed_kph / 3.6) * 3.6);
    const float speed_mps = result.control_speed_kph / 3.6f;
    const LiveLateralParams live = live_params();
    const float plan_age = plan_age_s(target);
    // 노이즈가 있는 운전자 조향 토크를 openpilot 방식으로 거른다
    const bool steering_pressed =
        steering_pressed_.update(vehicle_state.driver_torque, config_.steering_params.steering_pressed_threshold);
    result.steering_pressed = steering_pressed;
    if (steering_pressed) last_steering_pressed_s_ = now_s;
    const bool wheels_stopped = vehicle_standstill(vehicle_state);
    result.steer_fault = vehicle_state.steer_fault_temporary;
    const bool steer_fault_alert = update_steer_fault_alert(result.steer_fault, steering_pressed, wheels_stopped);
    result.active_block =
        active_block_reason(gated_path, target, vehicle_state, result.seeds_ready, result.vehicle_fresh, panda_ready,
                            panda_controls_allowed, result.control_speed_kph, plan_age, steer_fault_alert);
    const BlockKind kind = block_kind(result.active_block);
    resolve_engagement(kind, logical_engaged, engage_requested, now_s, &result);
    // 가용성 대기 중에는 토크만 0으로 하고 steer_req/MDPS 속도 스푸프는
    // 유지한다 — 매 정차마다 끊기면 MDPS/클러스터가 천이 경보를 낸다.
    // 결함/해제는 즉시 끊는다. plan 무효는 engage는 거부하되 steer_req는 잡아둔다.
    steer_availability_hold_ =
        logical_engaged && !result.active &&
        (kind == BlockKind::Availability || result.active_block == BlockReason::LateralPlanInvalid);
    // MDPS가 고장을 알리는 동안은 요청을 내리지 않는다(상류 latActive=false면 요청 비트도 0)
    const bool steer_request = (result.active || steer_availability_hold_) && !result.steer_fault;
    result.cut_steer_temp = update_cut_steer_state(steer_request, vehicle_state.steering_angle_deg);
    const bool plan_matches = large_angle_hold_ && driver_took_wheel_ &&
                              plan_matches_wheel(target, speed_mps, plan_age, vehicle_state.steering_angle_deg, live);
    result.large_angle_hold =
        update_large_angle_hold(steer_request, vehicle_state.steering_angle_deg, steering_pressed, plan_matches);
    result.large_angle_hold_by_driver = result.large_angle_hold && driver_took_wheel_;
    // 상류 latActive: 정차(최소 조향 속도·0.3 m/s 이하 또는 바퀴 정지)와 MDPS 일시 고장에서는 조향하지 않는다.
    // 큰 조향각에서 요청을 끈 동안도 활성이라도 조향을 쉰다.
    const bool standstill =
        !(speed_mps > std::max(config_.steering_params.min_steer_speed_mps, kStandstillSteerSpeedMps)) ||
        wheels_stopped;
    const bool steering = result.active && !result.large_angle_hold && !result.steer_fault && !standstill;
    result.lat_active = steering;
    const bool yaw_rate_valid =
        signal_time_fresh(vehicle_state.esp12_time_s, now_s,
                          static_cast<double>(config_.timing.vehicle_state_timeout_ms) / 1000.0) &&
        vehicle_state.yaw_rate_valid;
    road_bank_.update(0.01f, speed_mps, vehicle_state.lat_accel_mps2, vehicle_state.lat_accel_valid,
                      vehicle_state.yaw_rate_rad_s, yaw_rate_valid);

    bool curvature_limited = false;
    float plan_curvature = 0.0f;
    result.desired_curvature =
        clip_curvature(speed_mps, prev_desired_curvature_,
                       requested_curvature(target, vehicle_state, speed_mps, plan_age, steering, steering_pressed,
                                           yaw_rate_valid, live, &plan_curvature),
                       live.use_vehicle ? live.roll_rad : 0.0f, &curvature_limited);
    prev_desired_curvature_ = result.desired_curvature;
    const float nn_feedforward =
        lateral_nn_feedforward(target, vehicle_state, speed_mps, plan_age, steering, steering_pressed, live);

    if (steering) {
        steer(vehicle_state, speed_mps, now_s, steering_pressed, yaw_rate_valid, curvature_limited, plan_curvature,
              live, nn_feedforward, &result);
    } else {
        // 상류 LaC.reset(): 포화 시간을 지운다(PID는 토크 제어기가 비활성 틱에 지운다)
        sat_time_ = 0.0f;
        // 0을 넘기면 커브 중 engage 시 지연 버퍼가 0-setpoint로 P를 튀게 한다
        torque_controller_.update(false, speed_mps, result.desired_curvature, vehicle_state.steering_angle_deg, false,
                                  steer_limited_by_safety_, config_.steering_params, plan_delay_s(),
                                  vehicle_state.yaw_rate_rad_s, yaw_rate_valid, road_bank_.lat_accel(), live);
        copy_torque_state(&result);
        result.desired_torque = 0;
        result.apply_torque = 0;
    }

    // 결합 중이거나 해제 직후(inactive_release_ms) 동안은 토크 0으로 계속 보내 MDPS가 부드럽게 놓게
    // 한다.
    result.should_send = result.seeds_ready &&
                         (result.active || result.engaged ||
                          now_s - last_disengage_s_ < static_cast<double>(config_.timing.inactive_release_ms) / 1000.0);
    if (result.should_send) {
        result.frames = build_frames(vehicle_state, result, frame);
    } else {
        lkas11_counter_valid_ = false;
    }

    steer_req_sent_ = result.should_send && steer_request && !result.large_angle_hold && !result.cut_steer_temp;
    last_torque_ = result.apply_torque;
    // 상류는 컨트롤러 출력(CC.actuators.torque)과 보낸 토크(actuatorsOutput.torque)를 정규화해 비교한다
    const float steer_max = static_cast<float>(config_.steering_params.steer_max);
    steer_limited_by_safety_ =
        std::fabs(result.normalized_output * steer_max - static_cast<float>(result.apply_torque)) >
        kSteerLimitedBySafetyThreshold * steer_max;
    if (!result.active) {
        last_torque_ = 0;
        steer_limited_by_safety_ = false;
    }
    return result;
}

// path 복귀 디바운스: 무효는 즉시 반영, 무효를 겪은 뒤의 복귀는 0.5s
// 연속 유효를 요구한다(경계 깜빡임 차단). 최초 유효는 바로 통과.
LateralPath LateralController::debounce_path(const LateralPath &path, double now_s) {
    if (!path.usable_for_steering) {
        path_valid_since_s_ = -1.0;
        path_usable_debounced_ = false;
        path_seen_invalid_ = true;
    } else if (!path_usable_debounced_) {
        if (path_valid_since_s_ < 0.0) path_valid_since_s_ = now_s;
        if (!path_seen_invalid_ || now_s - path_valid_since_s_ >= 0.5) path_usable_debounced_ = true;
    }
    LateralPath gated_path = path;
    gated_path.usable_for_steering = path_usable_debounced_;
    if (!path_usable_debounced_) gated_path.invalid_reason = "path_invalid";
    return gated_path;
}

// plan 나이: 근거 프레임 캡처 시각부터 지금까지. lag 보상과 staleness
// gate가 함께 쓴다. 타임스탬프가 없으면(테스트, 초기값) 0으로 둔다.
float LateralController::plan_age_s(const LateralTarget &target) const {
    float plan_age_s = 0.0f;
    if (target.valid && target.capture_timestamp_ns != 0) {
        const uint64_t control_now_ns = clock_();
        if (control_now_ns > target.capture_timestamp_ns) {
            plan_age_s = static_cast<float>(static_cast<double>(control_now_ns - target.capture_timestamp_ns) * 1e-9);
        }
    }
    return plan_age_s;
}

// 결합과 Panda 유예를 풀고 제어 상태를 처음으로 돌린다(해제·engage 거부·CANCEL 공통).
void LateralController::disengage(double now_s) {
    panda_engage_pending_ = false;
    engaged_ = false;
    reset_control_state();
    last_disengage_s_ = now_s;
}

// 이번 틱의 차단 사유(kind)로 해제·engage 거부·Panda 유예를 정하고, result의 engaged,
// soft_disabling, engage_rejected, active를 채운다.
void LateralController::resolve_engagement(BlockKind kind, bool logical_engaged, bool engage_requested, double now_s,
                                           LateralControlResult *result) {
    // SoftDisable: engage 중이면 3초간 조향을 유지하며 경고하고, 그 뒤에는 Hard처럼 해제한다.
    bool soft_disable_expired = false;
    if (logical_engaged && !engage_requested && kind == BlockKind::SoftDisable) {
        if (soft_disable_start_s_ < 0.0) soft_disable_start_s_ = now_s;
        soft_disable_expired = now_s - soft_disable_start_s_ >= kSoftDisableS;
        result->soft_disabling = !soft_disable_expired;
    } else {
        soft_disable_start_s_ = -1.0;
    }
    if (logical_engaged && (kind == BlockKind::Hard || soft_disable_expired)) {
        disengage(now_s);
        result->engaged = config_.force_engaged;
    }

    if (engage_requested) {
        if (kind == BlockKind::Transient) {
            panda_engage_pending_ = true;
            panda_engage_pending_s_ = now_s;
        } else {
            panda_engage_pending_ = false;
        }
    }

    if (engage_requested && (kind == BlockKind::Hard || kind == BlockKind::Reject || kind == BlockKind::SoftDisable)) {
        // 차량/컨트롤러의 정적 gate는 실제 engage 요청 실패로 처리한다.
        disengage(now_s);
        result->engaged = config_.force_engaged;
        result->engage_rejected = true;
    }

    if (panda_engage_pending_) {
        const bool panda_waiting = kind == BlockKind::Transient;
        const bool grace_elapsed = now_s - panda_engage_pending_s_ >= kPandaEngageGraceS;
        if (kind == BlockKind::None || kind == BlockKind::Availability) {
            // Panda 허가가 도착했고 나머지는 가용성 상태뿐이면 engage를 유지한다.
            panda_engage_pending_ = false;
        } else if (!panda_waiting || grace_elapsed) {
            // 정적 실패를 저장했다가 gate가 해소되면 조용히 engage하지 않는다. Panda에는
            // 짧은 비동기 health handshake 유예만 허용하며, 완료되지 않으면 실제 차단
            // 사유를 한 번 보고한다.
            disengage(now_s);
            result->engaged = config_.force_engaged;
            result->engage_rejected = true;
        }
    }
    result->active = kind == BlockKind::None || result->soft_disabling;
}

// 클립 전 목표 곡률. 상류 controlsd: 활성이면 plan, 비활성이면 실제 곡률을 클립에 넣는다. 재활성 때
// 목표가 실제 곡률에서 한계 안으로 출발하고, 비활성 중의 잘못된 plan 값이 넘어오지 않는다.
float LateralController::requested_curvature(const LateralTarget &target, const VehicleCanState &vehicle_state,
                                             float speed_mps, float plan_age_s, bool steering, bool steering_pressed,
                                             bool yaw_rate_valid, const LiveLateralParams &live,
                                             float *plan_curvature) {
    const SteeringParams &control_params = config_.steering_params;
    float requested_curvature = steering ? lag_adjusted_curvature(target, speed_mps, plan_age_s, plan_delay_s())
                                         : torque_controller_.estimate_actual_curvature(
                                               speed_mps, vehicle_state.steering_angle_deg, control_params,
                                               vehicle_state.yaw_rate_rad_s, yaw_rate_valid, live);
    *plan_curvature = requested_curvature;
    // 운전자가 핸들을 잡지 않았을 때는 목표를 avoid_lkas_fault_hold_angle_deg(80도) 조향각이 내는
    // 곡률 안으로 묶는다(같은 차량 모델, 학습 SR·강성·오프셋·롤). 컨트롤러가 스스로 85도를 넘겨
    // 0.89초 뒤 토크가 빠지고 핸들이 풀렸다 다시 잡는 반복 대신, 그 각도에서 토크를 끊김 없이
    // 유지한다. 운전자가 조향 중이면 묶지 않는다: 교차로에서 더 감는 운전자를 80도 쪽으로 밀지
    // 않고, 85도 위에서는 시간 규칙이 그대로 적용된다. 횡가속 한계가 더 좁은 약 35 km/h 위에서는
    // 걸리지 않는다.
    float lo = 0.0f, hi = 0.0f;
    if (std::isfinite(requested_curvature) && hold_angle_bounds(speed_mps, steering, steering_pressed, live, &lo, &hi))
        requested_curvature = std::clamp(requested_curvature, lo, hi);
    return requested_curvature;
}

bool LateralController::hold_angle_bounds(float speed_mps, bool steering, bool steering_pressed,
                                          const LiveLateralParams &live, float *lo, float *hi) {
    const SteeringParams &control_params = config_.steering_params;
    const float hold_angle_deg = control_params.avoid_lkas_fault_hold_angle_deg;
    if (!steering || steering_pressed || !control_params.avoid_lkas_fault_enabled || !(hold_angle_deg > 0.0f) ||
        !std::isfinite(speed_mps))
        return false;
    const float left = torque_controller_.curvature_at_angle(speed_mps, hold_angle_deg, control_params, live);
    const float right = torque_controller_.curvature_at_angle(speed_mps, -hold_angle_deg, control_params, live);
    *lo = std::min(left, right);
    *hi = std::max(left, right);
    return true;
}

// 토크 제어기의 진단값을 결과에 옮긴다.
void LateralController::copy_torque_state(LateralControlResult *result) const {
    result->actual_curvature = torque_controller_.actual_curvature();
    result->actual_curvature_vm = torque_controller_.actual_curvature_vm();
    result->actual_curvature_yaw = torque_controller_.actual_curvature_yaw();
    result->curvature_error = result->desired_curvature - result->actual_curvature;
    result->normalized_output = torque_controller_.normalized_output();
    result->feedforward = torque_controller_.feedforward();
    result->nn_feedforward_used = torque_controller_.nn_feedforward_used();
}

void LateralController::set_lateral_nn(std::shared_ptr<const LateralNnModel> model) {
    lateral_nn_ = model && model->loaded() ? std::move(model) : nullptr;
    nn_inputs_.reset();
    nn_weight_ = 0.0f;
}

// 학습한 토크 모델의 입력은 목표 궤적이다(학습은 실제 궤적, docs/lateral-nn.md).
// - la(δ): 계획 곡률(MPC 궤적을 계획 나이만큼 앞으로 보간) × v². PID가 따르는 요청과 같은 범위로 묶는다: 운전자가
//   핸들을 잡지 않았을 때의 80도 유지 한계(hold_angle_bounds), 횡가속·곡률 상한(limit_curvature). 계획이 NaN이면 그
//   틱은 모델도 NaN이라 선형 피드포워드를 쓴다. 조향각(δ): 그 곡률을 차량 모델로 뒤집은 각도(학습 SR·강성·영점).
// - roll: 편경사 추정 / g(학습 데이터와 같은 추정기). long: ESP12 종가속.
float LateralController::lateral_nn_feedforward(const LateralTarget &target, const VehicleCanState &vehicle_state,
                                                float speed_mps, float plan_age_s, bool steering, bool steering_pressed,
                                                const LiveLateralParams &live) {
    if (!lateral_nn_) return NAN;
    const float speed = std::isfinite(speed_mps) ? std::max(speed_mps, 0.0f) : 0.0f;
    const float age = clamp_float(plan_age_s, 0.0f, kMaxPlanAgeCompS);
    const float roll_rad = live.use_vehicle ? live.roll_rad : 0.0f;
    float hold_lo = -kMaxCurvature, hold_hi = kMaxCurvature;
    hold_angle_bounds(speed, steering, steering_pressed, live, &hold_lo, &hold_hi);
    const auto curvature_at = [&](float delta_s) {
        const float curvature = interp_lateral(delta_s + age, target.curvatures);
        if (!std::isfinite(curvature)) return NAN;
        return limit_curvature(speed, std::clamp(curvature, hold_lo, hold_hi), roll_rad);
    };
    const float speed_sq = speed * speed;
    const TorqueController::AngleMap angle = torque_controller_.angle_map(speed, config_.steering_params, live);
    const auto la_at = [&](float delta_s) { return curvature_at(delta_s) * speed_sq; };
    const auto angle_at = [&](float delta_s) { return angle(curvature_at(delta_s)); };
    nn_inputs_.push(la_at(0.0f), angle_at(0.0f));
    // 켜고 끌 때, 그리고 조향을 시작할 때 피드포워드가 건너뛰지 않게 1초에 걸쳐 섞는다(설정은 주행 중에도 바뀐다)
    const bool wanted = steering && config_.steering_params.nn_feedforward && target.valid && speed > kNnMinSpeedMps;
    nn_weight_ = clamp_float(nn_weight_ + (wanted ? kNnBlendStep : -kNnBlendStep), 0.0f, 1.0f);
    if (!steering || !target.valid || nn_weight_ <= 0.0f) return NAN;
    return nn_inputs_.evaluate(*lateral_nn_, speed, road_bank_.lat_accel() / 9.81f, vehicle_state.long_accel_mps2,
                               la_at, angle_at);
}

// 조향 중: 토크를 계산하고 85도 고장 회피와 Panda 한계를 적용한 뒤 포화를 본다.
void LateralController::steer(const VehicleCanState &vehicle_state, float speed_mps, double now_s,
                              bool steering_pressed, bool yaw_rate_valid, bool curvature_limited, float plan_curvature,
                              const LiveLateralParams &live, float nn_feedforward, LateralControlResult *result) {
    const SteeringParams &control_params = config_.steering_params;
    // 85도 위에서도 컨트롤러 토크를 그대로 내되, 요청을 끄는 프레임(avoid_lkas_fault_max_frames,
    // update_large_angle_hold)에 0에 닿도록 크기를 steer_delta_down × 남은 프레임으로 묶는다.
    // 하강 레이트 한계로 내려와도 늦지 않는 만큼만 남기는 것이라, 최대 토크로 85도를 넘으면
    // 약 0.35초는 그대로 두고 0.55초에 걸쳐 내린다(2026-10-03까지는 0.69초 선형 램프로 0).
    // 요청은 토크가 0일 때 꺼지므로 어시스트가 빠지는 "탁"이 없다. 적분기는 85도 위에서 얼린다.
    const bool above_fault_angle =
        control_params.avoid_lkas_fault_enabled &&
        std::fabs(vehicle_state.steering_angle_deg) >= control_params.avoid_lkas_fault_max_angle_deg;
    int torque_cap = control_params.steer_max;
    if (result->large_angle_hold) {
        torque_cap = 0;
    } else if (fault_angle_frames_ > 0) {
        torque_cap =
            std::min(torque_cap, control_params.steer_delta_down *
                                     std::max(0, control_params.avoid_lkas_fault_max_frames - fault_angle_frames_));
    }
    const int raw_torque = torque_controller_.update(
        true, speed_mps, result->desired_curvature, vehicle_state.steering_angle_deg, steering_pressed,
        steer_limited_by_safety_ || above_fault_angle, control_params, plan_delay_s(), vehicle_state.yaw_rate_rad_s,
        yaw_rate_valid, road_bank_.lat_accel(), live, nn_feedforward, nn_weight_);
    result->desired_torque = std::clamp(raw_torque, -torque_cap, torque_cap);
    copy_torque_state(result);
    result->apply_torque = apply_hyundai_steer_torque_limits(
        result->desired_torque, last_torque_, vehicle_state.driver_torque, hyundai_limits(control_params));
    result->steer_saturated =
        update_saturation(*result, plan_curvature, speed_mps, now_s, steering_pressed, curvature_limited);
}

// openpilot LatControl._check_saturation + selfdrived steerSaturated. 출력이 한계에
// 붙었거나 곡률이 횡가속 한계에 잘렸는데, 안전 한계(토크 레이트)나 운전자 때문이
// 아닐 때 0.4초 누적되면 포화다. 커브에서 plan 목표 횡가속(클립 전, 상류는 모델의
// desiredCurvature)이 실제의 1.2배를 넘고 1 m/s² 이상이며 최근 2초 안에 핸들을 잡지
// 않았으면 경고한다.
bool LateralController::update_saturation(const LateralControlResult &result, float plan_curvature, float speed_mps,
                                          double now_s, bool steering_pressed, bool curvature_limited) {
    constexpr float kDt = 0.01f;
    const bool saturated_now = std::fabs(result.normalized_output) > 1.0f - 1e-3f || curvature_limited;
    if (saturated_now && speed_mps > kSatCheckMinSpeedMps && !steer_limited_by_safety_ && !steering_pressed)
        sat_time_ += kDt;
    else
        sat_time_ -= kDt;
    sat_time_ = clamp_float(sat_time_, 0.0f, kSteerLimitTimerS);
    const bool lac_saturated = sat_time_ > kSteerLimitTimerS - 1e-3f;
    const bool recent_steer_pressed = now_s - last_steering_pressed_s_ < 2.0;
    const float clipped_speed = std::max(speed_mps, 0.3f);
    const float desired_lat_accel = plan_curvature * clipped_speed * clipped_speed;
    const float actual_lat_accel = result.actual_curvature * clipped_speed * clipped_speed;
    const bool undershooting = std::fabs(desired_lat_accel) / std::fabs(1e-3f + actual_lat_accel) > 1.2f;
    const bool turning = std::fabs(desired_lat_accel) > 1.0f;
    return !recent_steer_pressed && undershooting && turning && lac_saturated;
}

// CLU 버튼 edge로 engage/disengage 상태를 갱신한다.
void LateralController::update_button_state(int button, double now_s) {
    if (button == last_button_) return;
    if (button == kCruiseButtonCancel) {
        disengage(now_s);
    } else if (button == 0 && last_button_ == kCruiseButtonSet) {
        engaged_ = true;
    }
    last_button_ = button;
}

// avoid_lkas_fault를 끈 경우: 상류 hyundai carcontroller의 common_fault_avoidance(MAX_ANGLE, MAX_ANGLE_FRAMES,
// MAX_ANGLE_CONSECUTIVE_FRAMES). 요청을 낸 채 85도 위에 머문 프레임을 세어 89를 넘으면 2프레임 요청을 끄고
// (ToiFlt를 켜고 토크는 그대로) 다시 센다. K7은 끈 요청을 85도 위에서 다시 켜면 곧 고장을 내므로 기본값은
// 켠 쪽(update_large_angle_hold)이다.
bool LateralController::update_cut_steer_state(bool steer_request, float steering_angle_deg) {
    const SteeringParams &params = config_.steering_params;
    if (params.avoid_lkas_fault_enabled) {
        angle_limit_counter_ = 0;
        return false;
    }
    const bool fault_condition = std::fabs(steering_angle_deg) >= params.avoid_lkas_fault_max_angle_deg;
    angle_limit_counter_ = steer_request && fault_condition ? angle_limit_counter_ + 1 : 0;
    const bool cut = angle_limit_counter_ > params.avoid_lkas_fault_max_frames;
    if (angle_limit_counter_ >= params.avoid_lkas_fault_max_frames + params.avoid_lkas_fault_cut_frames)
        angle_limit_counter_ = 0;
    return cut;
}

// 상류 car_events: MDPS 일시 고장이 운전자가 핸들을 잡은 채 시작했으면 경고하지 않고, 운전자가 놓은 지
// 1.5초가 안 됐거나 정차면 조용한 경고(그 고장 동안 유지), 그 밖에는 steerTempUnavailable이다. 어느 쪽이든
// 고장 동안 조향은 쉰다.
bool LateralController::update_steer_fault_alert(bool steer_fault, bool steering_pressed, bool standstill) {
    steering_unpressed_frames_ =
        steering_pressed ? 0 : std::min(steering_unpressed_frames_ + 1, kSilentSteerWarningFrames);
    bool alert = false;
    if (steer_fault) {
        if (steering_pressed && (!last_steer_fault_ || no_steer_warning_)) {
            no_steer_warning_ = true;
        } else {
            no_steer_warning_ = false;
            if (silent_steer_warning_ || standstill || steering_unpressed_frames_ < kSilentSteerWarningFrames)
                silent_steer_warning_ = true;
            else
                alert = true;
        }
    } else {
        no_steer_warning_ = false;
        silent_steer_warning_ = false;
    }
    last_steer_fault_ = steer_fault;
    return alert;
}

// 큰 조향각 고장 회피. K7 MDPS는 steer 요청이 켜진 채 |조향각|이 85도 위에 0.98~1.12초 머물면
// 토크와 무관하게 ToiFlt+FailStat을 내고, 85도 아래로 와야 푼다(2026-09-18 실측). 요청을 내는
// 동안(active, 또는 정차 같은 가용성 대기) 85도 위 체류를 세어 avoid_lkas_fault_max_frames에 닿으면
// 요청을 끈다. 그 프레임까지 토크는 update가 0으로 내려 둔다. openpilot식 2프레임 컷은 85도 위에서
// 요청을 다시 켜는 순간 3~14 ms 뒤 고장을 냈고(2026-10-03, 4번 중 4번), 정차 대기 중에는 세지도
// 않아 1초 뒤 고장이 났다. 그래서 85도 위에서는 요청을 새로 켜지 않는다(결합이나 차단 해제가
// 85도 위에서 일어나도 같다).
// 끈 요청은 85도 아래로 오면 다시 켠다. 다만 그 체류 중 운전자가 핸들을 돌렸으면(넘겨받은 회전) 핸들이
// kDriverReleaseAngleDeg(15도) 아래로 오고 손을 떼거나(carrotpilot lat_suspend_control과 같은 해제 조건), 80도
// 아래에서 계획이 지금 핸들과 같은 커브를 1초 가리킬 때까지 끈 채로 둔다. 회전을 빠져나오며 핸들을 펴는 운전자를
// 시스템이 미리 밀지 않고(2026-10-03 2:54 탈출에서 최대 213), 램프에서 85도를 잠깐 넘긴 뒤 커브를 붙잡은 운전자에게는
// 커브가 끝날 때까지 손을 놓게 하지 않는다(2026-10-05 16:46 램프는 14초 동안 토크 0).
bool LateralController::update_large_angle_hold(bool steer_requested, float steering_angle_deg, bool steering_pressed,
                                                bool plan_matches_wheel) {
    const SteeringParams &params = config_.steering_params;
    const float angle = std::fabs(steering_angle_deg);
    const bool above = params.avoid_lkas_fault_enabled && angle >= params.avoid_lkas_fault_max_angle_deg;
    if (above && steer_requested && steering_pressed) driver_took_wheel_ = true;
    if (!above) {
        fault_angle_frames_ = 0;
        if (large_angle_hold_ && driver_took_wheel_) {
            const bool on_planned_turn =
                angle < params.avoid_lkas_fault_max_angle_deg - kHeldTurnResumeMarginDeg && plan_matches_wheel;
            held_turn_frames_ = on_planned_turn ? held_turn_frames_ + 1 : 0;
            const bool centered = angle < kDriverReleaseAngleDeg && !steering_pressed;
            if (!centered && held_turn_frames_ < kHeldTurnResumeFrames) return true;
        }
        large_angle_hold_ = false;
        driver_took_wheel_ = false;
        held_turn_frames_ = 0;
        return false;
    }
    held_turn_frames_ = 0;
    if (!steer_requested) {
        fault_angle_frames_ = 0;
        return large_angle_hold_;
    }
    if (!steer_req_sent_) {
        large_angle_hold_ = true;
    } else if (!large_angle_hold_ && ++fault_angle_frames_ >= params.avoid_lkas_fault_max_frames) {
        large_angle_hold_ = true;
    }
    return large_angle_hold_;
}

// 제어 내부 상태를 초기값으로 되돌린다. 토크 PID는 해제 뒤 첫 비활성 틱에 토크 제어기가 지운다.
void LateralController::reset_control_state() {
    last_torque_ = 0;
    steer_limited_by_safety_ = false;
    fault_angle_frames_ = 0;
    large_angle_hold_ = false;
    driver_took_wheel_ = false;
    held_turn_frames_ = 0;
    angle_limit_counter_ = 0;
}

bool LateralController::plan_matches_wheel(const LateralTarget &target, float speed_mps, float plan_age_s,
                                           float steering_angle_deg, const LiveLateralParams &live) {
    if (!target.valid || !std::isfinite(speed_mps) || !std::isfinite(steering_angle_deg)) return false;
    const float plan = lag_adjusted_curvature(target, speed_mps, plan_age_s, plan_delay_s());
    const float wheel =
        torque_controller_.curvature_at_angle(speed_mps, steering_angle_deg, config_.steering_params, live);
    return std::isfinite(plan) && plan * wheel > 0.0f &&
           std::fabs(plan - wheel) <= kHeldTurnPlanTolerance * std::fabs(wheel);
}

// active를 막는 현재 gate reason을 계산한다.
BlockReason LateralController::active_block_reason(const LateralPath &path, const LateralTarget &target,
                                                   const VehicleCanState &vehicle_state, bool seeds_ready,
                                                   bool vehicle_fresh, bool panda_ready, bool panda_controls_allowed,
                                                   float speed_kph, float plan_age_s, bool steer_fault_alert) const {
    // 순서 규칙: 데이터 유효성 -> 차량 결함(hard disengage) -> 핸드셰이크 ->
    // 가용성 대기. 결함이 뒤로 밀리면 앞선 일시적 사유가 결함을 가리고, 그
    // 사이 engage가 유예되어 톤만 울렸다가 해제된다.
    if (!config_.force_engaged && !engaged_) return BlockReason::NotEngaged;
    if (!config_.steering_params.enabled) return BlockReason::ControllerDisabled;
    if (!seeds_ready) return BlockReason::SeedsMissing;
    if (!vehicle_fresh) return BlockReason::VehicleStateStale;
    if (!std::isfinite(speed_kph)) return BlockReason::SpeedInvalid;
    if (vehicle_state.door_open) return BlockReason::DoorOpen;
    if (vehicle_state.seatbelt_unlatched) return BlockReason::SeatbeltUnlatched;
    if (vehicle_state.esp_disabled) return BlockReason::EspDisabled;
    if (vehicle_state.park_brake) return BlockReason::ParkBrake;
    if (vehicle_state.brake_error) return BlockReason::BrakeError;
    if (vehicle_state.gear != kGearDrive) return BlockReason::GearNotDrive;
    // Panda 핸드셰이크는 차량 결함 뒤에 온다. 앞에 두면 시동 직후 health가
    // 도착하기 전의 engage 요청이 panda_not_ready(일시적)로 분류되어 유예되고,
    // 안전벨트/기어 같은 하드 결함이 가려진 채 engage 톤이 울린 뒤 해제된다.
    if (!panda_ready) return BlockReason::PandaNotReady;
    if (!panda_controls_allowed) return BlockReason::PandaControlsOff;
    if (!path.usable_for_steering) {
        // 정지에서는 plan이 원래 짧아 path 무효가 정상이다. 오류가 아니라
        // 대기로 보고한다. 이 속도 밑은 min_steer_speed로 토크도 0이다.
        if (speed_kph / 3.6f < config_.steering_params.min_steer_speed_mps) return BlockReason::Stopped;
        return BlockReason::PathInvalid;
    }
    if (!target.valid || !target.mpc_solution_valid) return BlockReason::LateralPlanInvalid;
    // 모델 경로 gate는 모델 발행 시각만 본다. 플랜은 근거 프레임 캡처 시각으로도 막아, 늦게 발행된
    // 낡은 프레임의 플랜도 쓰지 않는다.
    if (target.capture_timestamp_ns != 0 &&
        plan_age_s > static_cast<float>(config_.timing.model_timeout_ms) / 1000.0f) {
        return BlockReason::LateralPlanStale;
    }
    // openpilot steerTempUnavailable, calibrationIncomplete/Recalibrating/Invalid, paramsdTemporaryError.
    // SoftDisable이라 맨 끝에 둔다. MDPS 고장 동안 조향을 쉬는 것은 이 사유와 따로 update가 정한다(조용한
    // 경고일 때도 쉰다). paramsd는 쓰는 중이고 보정이 끝났을 때만 본다(상류 sm.seen·cal_status 검사).
    if (steer_fault_alert) return BlockReason::SteerTempUnavailable;
    if (calibration_status_ == 0) return BlockReason::CalibrationIncomplete;
    if (calibration_status_ == 3) return BlockReason::CalibrationRecalibrating;
    if (calibration_status_ != 1) return BlockReason::CalibrationInvalid;
    if (live_params().use_vehicle && !live_vehicle_valid_ && live_calibrated_) return BlockReason::ParamsdInvalid;
    return BlockReason::None;
}

float lag_adjusted_curvature(const LateralTarget &target, float speed_mps, float plan_age_s,
                             float steer_actuator_delay_s) {
    // plan은 카메라 캡처 시점 기준이므로 소비 시점까지의 실측 나이를 actuator
    // delay에 더해 보간한다. 부수 효과로 desired curvature가 20Hz 계단 대신
    // 매 tick plan 위를 따라 전진한다.
    const float delay = std::max(0.01f, steer_actuator_delay_s) + clamp_float(plan_age_s, 0.0f, kMaxPlanAgeCompS);
    const float current_curvature = target.curvatures[0];
    const float psi = delay < kMinStableDelayS
                          ? delay / kMinStableDelayS * interp_lateral(kMinStableDelayS, target.psis)
                          : interp_lateral(delay, target.psis);
    // openpilot drive_helpers.MIN_SPEED. 하한이 낮으면 psi/(v*delay)가 정지
    // 부근에서 발산해 작은 plan 오차가 곡률 상한까지 증폭된다.
    const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
    const float curvature_from_psi = psi / (speed * delay);
    return current_curvature + 2.0f * (curvature_from_psi - current_curvature);
}

float clip_curvature(float speed_mps, float prev_curvature, float new_curvature, float roll_rad, bool *limited) {
    if (limited) *limited = false;
    // 바퀴 속도가 끊기면 NaN이다. 그대로 두면 clamp_float가 -kMaxCurvature를 낸다.
    if (!std::isfinite(speed_mps) || !std::isfinite(new_curvature)) return prev_curvature;
    const float speed = std::max(speed_mps, kMinCurvatureSpeedMps);
    // ISO 횡저크 한계를 직전 출력 기준 틱당 변화율로 건다. 플랜 노드 기준 창이던
    // v0.9.4와 달리 틱간 계단을 실제로 막아, 변화율 제한된 와이어가 예산을
    // 노이즈에 쓰지 않는다.
    const float max_curvature_rate = kMaxLateralJerk / (speed * speed);
    const float rate_limited = clamp_float(new_curvature, prev_curvature - max_curvature_rate * kCurvatureRateWindowS,
                                           prev_curvature + max_curvature_rate * kCurvatureRateWindowS);
    const float curvature = limit_curvature(speed_mps, rate_limited, roll_rad);
    if (limited) *limited = curvature != rate_limited;
    return curvature;
}

float lag_adjusted_desired_curvature(const LateralTarget &target, float speed_mps, float plan_age_s,
                                     float steer_actuator_delay_s, float prev_curvature, float roll_rad) {
    if (!target.valid) return 0.0f;
    return clip_curvature(speed_mps, prev_curvature,
                          lag_adjusted_curvature(target, speed_mps, plan_age_s, steer_actuator_delay_s), roll_rad);
}

// 최종 송신 frame 묶음을 만든다.
std::vector<CanFrame> LateralController::build_frames(const VehicleCanState &vehicle_state,
                                                      const LateralControlResult &result, int frame) {
    HyundaiLkasCommand command;
    command.apply_steer = result.apply_torque;
    command.steer_req = (result.active || steer_availability_hold_) && !result.steer_fault && !result.large_angle_hold;
    command.cut_steer_temp = result.cut_steer_temp;
    // 클러스터 LKAS 표시(양쪽 차선 보임): 3 작동, 4 대기. 클러스터는 sys_state 천이마다 부저를
    // 울리므로 active(정차 대기 등 가용성)가 아니라 engaged를 따라 enable/disable에서만 울린다.
    // steer_req는 별도 비트로 매 프레임 정확히 나간다.
    command.sys_state = result.engaged ? 3 : 4;
    command.sys_warning = false;
    command.left_lane = result.left_lane;
    command.right_lane = result.right_lane;
    command.lkas_msg_count = next_lkas11_counter(vehicle_state);
    command.ldws_fix = false;  // 이 K7은 LDWS 전용이지만 LdwsOpt_USM 3은 효과가 없었다(2026-09-24)

    // should_send면 세 seed가 다 있다(seed_frames_ready).
    return build_lateral_can_frames(decode_lkas11(vehicle_state.lkas11_seed), decode_clu11(vehicle_state.clu11_seed),
                                    vehicle_state.mdps12_seed, command, config_.steering_params.mdps_speed_spoof_kph,
                                    result.active || steer_availability_hold_, vehicle_state.speed_unit_mph, frame);
}

int LateralController::next_lkas11_counter(const VehicleCanState &vehicle_state) {
    if (!lkas11_counter_valid_) {
        const HyundaiLkas11Values seed = decode_lkas11(vehicle_state.lkas11_seed);
        lkas11_counter_ = (seed.msg_count + 1) & 0xf;
        lkas11_counter_valid_ = true;
    }
    const int counter = lkas11_counter_ & 0xf;
    lkas11_counter_ = (lkas11_counter_ + 1) & 0xf;
    return counter;
}
