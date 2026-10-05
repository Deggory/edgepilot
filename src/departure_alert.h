#pragma once

#include <cstdint>

enum class DepartureAlertType : uint32_t {
  none = 0,
  lead_departed = 1,
  green_light = 2,
};

const char *departure_alert_name(DepartureAlertType type);

struct DepartureAlertInput {
  double now_s = 0.0;
  bool vehicle_valid = false;
  int gear = 0;
  float speed_mps = 0.0f;
  bool gas_pressed = false;

  bool lead_updated = false;
  bool lead_valid = false;
  float lead_distance_m = 0.0f;
  float lead_relative_speed_mps = 0.0f;

  bool model_updated = false;
  bool model_valid = false;
  float plan_distance_m = 0.0f;
  float gas_press_prob = 0.0f;  // 모델이 본 "운전자가 2초 뒤 가속 페달을 밟고 있을" 확률
  bool turn_signal_on = false;  // 좌·우 방향지시등(비상등 포함)
};

struct DepartureAlertOutput {
  DepartureAlertType type = DepartureAlertType::none;
  uint32_t event_id = 0;
  bool lead_armed = false;
  bool green_light_armed = false;
};

class DepartureAlertDetector {
public:
  DepartureAlertOutput update(const DepartureAlertInput &input);

private:
  void reset_cycle();
  void reset_lead();
  void reset_green_light();
  void trigger(DepartureAlertType type, double now_s);

  bool consumed_ = false;
  double stopped_since_s_ = -1.0;

  double lead_seen_since_s_ = -1.0;
  double lead_depart_candidate_since_s_ = -1.0;
  float lead_baseline_distance_m_ = 0.0f;
  bool lead_armed_ = false;
  double lead_last_seen_s_ = -1.0;     // vision 앞차가 마지막으로 보인 시각(짧은 끊김을 넘긴다)
  double close_lead_seen_s_ = -1.0;    // 가까운 앞차가 마지막으로 보인 시각
  float stop_lead_min_m_ = 0.0f;       // 이번 정차에서 본 가까운 앞차의 최소 거리
  bool close_lead_moved_ = false;      // 그 앞차가 최소 거리에서 멀어졌다

  double short_plan_since_s_ = -1.0;  // 이번 정차에서 plan이 짧게 이어지기 시작한 시각
  int gas_press_frames_ = 0;           // 무장 뒤 가속 확률이 기준을 넘은 연속 모델 프레임
  double green_light_candidate_since_s_ = -1.0;
  bool green_light_armed_ = false;

  DepartureAlertType active_type_ = DepartureAlertType::none;
  double active_until_s_ = -1.0;
  uint32_t event_id_ = 0;
};
