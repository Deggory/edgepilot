#include "controls/control_params.h"

#include "common/utils_json.h"

#include <stdexcept>
#include <string>

float SteeringParams::center_to_front_m() const { return wheelbase_m * center_to_front_ratio; }

namespace {

constexpr JsonBoolField<SteeringParams> kSteeringBools[] = {
    {"enabled", &SteeringParams::enabled},
    {"avoid_lkas_fault_enabled", &SteeringParams::avoid_lkas_fault_enabled},
    {"live_bank_compensation", &SteeringParams::live_bank_compensation},
    {"use_live_vehicle_params", &SteeringParams::use_live_vehicle_params},
    {"use_live_torque_params", &SteeringParams::use_live_torque_params},
    {"use_live_delay", &SteeringParams::use_live_delay},
    {"nn_feedforward", &SteeringParams::nn_feedforward},
    {"use_locationd_learner_inputs", &SteeringParams::use_locationd_learner_inputs},
};
constexpr JsonIntField<SteeringParams> kSteeringInts[] = {
    {"steering_pressed_threshold", 0, 500, &SteeringParams::steering_pressed_threshold},
    // 범위는 웹 콘솔 대조용이다. 값은 아래 read_path_mode가 먼저 검사한다(반올림·클램프하지 않음).
    {"path_mode", 0, 2, &SteeringParams::path_mode},
};
constexpr JsonFloatField<SteeringParams> kSteeringFloats[] = {
    {"torque_lat_accel_factor", 0.5f, 5.0f, &SteeringParams::torque_lat_accel_factor},
    {"torque_kp", 0.0f, 10.0f, &SteeringParams::torque_kp},
    {"torque_ki", 0.0f, 2.0f, &SteeringParams::torque_ki},
    {"torque_friction", 0.0f, 0.3f, &SteeringParams::torque_friction},
    {"steering_angle_deadzone_deg", 0.0f, 3.0f, &SteeringParams::steering_angle_deadzone_deg},
    {"torque_straight_kp_scale", 0.3f, 1.0f, &SteeringParams::torque_straight_kp_scale},
    {"steer_ratio", 8.0f, 25.0f, &SteeringParams::steer_ratio},
    {"tire_stiffness_factor", 0.2f, 2.0f, &SteeringParams::tire_stiffness_factor},
    {"steer_actuator_delay", 0.01f, 1.0f, &SteeringParams::steer_actuator_delay},
    // 고장 각도(avoid_lkas_fault_max_angle_deg, 85도) 아래에서 멈춰야 뜻이 있다
    {"avoid_lkas_fault_hold_angle_deg", 0.0f, 84.0f, &SteeringParams::avoid_lkas_fault_hold_angle_deg},
    {"angle_offset_deg", -10.0f, 10.0f, &SteeringParams::angle_offset_deg},
    {"torque_lat_accel_offset", -1.0f, 1.0f, &SteeringParams::torque_lat_accel_offset},
    {"mass_kg", 1000.0f, 2600.0f, &SteeringParams::mass_kg},
    {"wheelbase_m", 2.0f, 3.5f, &SteeringParams::wheelbase_m},
    {"center_to_front_ratio", 0.2f, 0.7f, &SteeringParams::center_to_front_ratio},
    {"path_offset_m", -1.0f, 1.0f, &SteeringParams::path_offset_m},
    {"lane_path_weight", 0.5f, 10.0f, &SteeringParams::lane_path_weight},
    {"min_steer_speed_mps", 0.0f, 5.0f, &SteeringParams::min_steer_speed_mps},
    {"lane_change_min_speed_kph", 0.0f, 80.0f, &SteeringParams::lane_change_min_speed_kph},
};

// 경로 모드는 고르는 값이라 0, 1, 2 말고는 거부한다(다른 정수 항목처럼 클램프하면 7이 Lanebow가 된다).
// 2026-10-15 전 파일의 laneless_mode는 path_mode로 읽는다: 보드의 params/는 배포가 덮어쓰지 않는다.
void read_path_mode(const std::string &text, SteeringParams *params) {
    float mode = 0.0f;
    const bool has_mode = parse_json_float_value(text, "path_mode", &mode);
    bool laneless = false;
    const bool has_laneless = parse_json_bool_value(text, "laneless_mode", &laneless);
    if (has_mode && has_laneless)
        throw std::runtime_error("laneless_mode was replaced by path_mode; keep only path_mode");
    if (has_mode && !(mode == 0.0f || mode == 1.0f || mode == 2.0f))
        throw std::runtime_error("path_mode must be 0 (lane), 1 (laneless) or 2 (lanebow)");
    if (has_laneless) params->path_mode = static_cast<int>(laneless ? PathMode::Laneless : PathMode::Lane);
}

}  // namespace

bool load_steering_params_json(const std::string &path, SteeringParams *params, std::string *error) {
    if (!params) return false;
    return load_json_param_file(
        path,
        [params](const std::string &text) {
            // 2026-09-24 이전 raw 표기. 조용히 기본값으로 떨어지지 않게 거부한다.
            for (const char *key : {"torque_max_lat_accel_raw", "torque_kp_raw", "torque_kf_raw", "torque_ki_raw",
                                    "torque_friction_raw"}) {
                if (text.find(std::string("\"") + key + "\"") != std::string::npos)
                    throw std::runtime_error(std::string(key) +
                                             " was replaced by torque_lat_accel_factor/"
                                             "torque_kp/torque_ki/torque_friction");
            }
            read_path_mode(text, params);
            parse_json_fields(text, kSteeringBools, params);
            parse_json_fields(text, kSteeringInts, params);
            parse_json_fields(text, kSteeringFloats, params);
        },
        error);
}

std::string first_steering_param_difference(const SteeringParams &a, const SteeringParams &b) {
    for (const auto &field : kSteeringBools)
        if (a.*field.member != b.*field.member) return field.key;
    for (const auto &field : kSteeringInts)
        if (a.*field.member != b.*field.member) return field.key;
    for (const auto &field : kSteeringFloats)
        if (a.*field.member != b.*field.member) return field.key;
    return "";
}
