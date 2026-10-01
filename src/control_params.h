#pragma once

#include <string>

/* controlsd가 함께 읽는 런타임 파라미터. params/steering.json과
 * params/driving.json이 각각의 출처다. CAN 계층은 이 헤더에 의존하지만
 * 반대 방향은 없다 — 토크 제한 변환은 hyundai_can.h가 제공한다. */

/* 기본값은 params/steering.json(K7 YG HEV 실차 검증값)과 일치시킨다. 로드 실패는
 * controlsd가 throw하므로 이 값은 파일 폴백이 아니라, JSON에 키가 빠졌을 때와
 * 호스트 체크·리플레이 도구가 실제로 쓰는 값이다. */
struct SteeringParams {
  bool enabled = true;

  /* panda의 hyundai safety가 같은 숫자를 강제한다(safety_hyundai.h: MAX_STEER 384,
   * MAX_RATE_UP 3, MAX_RATE_DOWN 7, DRIVER_TORQUE_ALLOWANCE 50, DRIVER_TORQUE_FACTOR 2).
   * 올리면 panda가 프레임을 거부하고 내리면 순정보다 약해지기만 하므로 런타임에서
   * 읽지 않는다. steer_driver_factor는 panda에 대응 항이 없어 1이 아니면 우리 계산만
   * 어긋난다. 바꾸려면 panda 펌웨어와 함께 바꾸고 여기를 고친다. */
  int steer_max = 384;
  int steer_delta_up = 3;
  int steer_delta_down = 7;
  int steer_driver_allowance = 50;
  int steer_driver_multiplier = 2;
  int steer_driver_factor = 1;
  int steering_pressed_threshold = 150;

  /* openpilot 토크 튜닝 그대로(횡가속도 공간). 토크 = (FF + P + I) / 배율 + 마찰.
   * 배율은 MaixCAM2 torqued 학습값(2026-09-27, 6769점 TLS 2.75; K230 때 2.65~2.95). */
  float torque_lat_accel_factor = 2.75f;
  // KP_INTERP의 30 m/s 끝점. 나머지 점은 상류 고정값이다.
  float torque_kp = 0.8f;
  float torque_ki = 0.15f;
  float torque_friction = 0.083f;  // 토크 공간, torqued 학습값(2026-09-27)
  bool torque_use_angle = true;
  int torque_output_sign = -1;

  // paramsd 학습값(2026-09-27): 조향비 14.72, 타이어 강성 0.83.
  float steer_ratio = 14.72f;
  float tire_stiffness_factor = 0.83f;
  float steer_actuator_delay = 0.34f;
  bool avoid_lkas_fault_enabled = true;
  float avoid_lkas_fault_max_angle_deg = 85.0f;
  int avoid_lkas_fault_max_frames = 89;
  /* steer request를 끊는 프레임 수. K7에서 2프레임 컷은 MDPS fault 타이머를
   * 되돌리지 못한다(2026-09-21 실측: 폴트 중 2프레임 컷 34회 모두 1.5초 유지).
   * 길이를 바꿔 되돌아가는 지점을 찾기 위한 실측 파라미터다. */
  int avoid_lkas_fault_cut_frames = 2;
  float angle_offset_deg = -1.57f;  // paramsd 학습 평균(2026-09-27; K230 때 −1.5~−1.6°)
  /* openpilot latAccelOffset(m/s^2). 상수 횡가속 편향을 FF에서 뺀다.
   * +y=오른쪽 관례라 양수 = 우측 쏠림 보정. fit 도구 출력을 그대로 넣는다. */
  float torque_lat_accel_offset = -0.06f;  // torqued 학습값(2026-09-27)
  /* ESP12 실측으로 추정한 도로 편경사(뱅크)를 FF에서 실시간 보정한다.
   * 켜면 상수 offset이 커버 못 하는 커브별 편경사까지 잡는다. */
  bool live_bank_compensation = true;
  /* paramsd·torqued 학습값 사용. 끄면 학습기는 계산·기록만 한다. 차량 값을 켜면
   * 롤 보정이 live_bank_compensation을 대신한다. */
  bool use_live_vehicle_params = false;
  bool use_live_torque_params = false;
  /* lagd(locationd) 조향 지연 사용. 켜고 lagd가 확정(5블록)되면 경로 지연(목표 곡률을 읽는
   * 시점)에 추정값을 쓴다. 토크 컨트롤러·torqued는 steer_actuator_delay를 그대로 쓴다(1단계). */
  bool use_live_delay = false;
  /* paramsd·torqued 입력을 상류처럼 locationd 요레이트·롤로 받는다. 끄면 ESP12 요레이트(자체 바이어스
   * 추정)와 ESP12 횡가속 롤. locationd가 없거나 낡으면 ESP12로 돌아간다. */
  bool use_locationd_learner_inputs = true;
  float mass_kg = 1816.0f;
  float wheelbase_m = 2.855f;
  float center_to_front_ratio = 0.4f;
  float steer_ratio_rear = 0.0f;
  float path_offset_m = 0.0f;
  float min_steer_speed_mps = 1.0f;

  float center_to_front_m() const;
};

struct DrivingParams {
  int model_timeout_ms = 250;
  int vehicle_state_timeout_ms = 500;
  int inactive_release_ms = 3000;
  float mdps_speed_spoof_kph = 60.0f;
  float lane_change_min_speed_kph = 30.0f;
  bool laneless_mode = false;
};

// params/steering.json을 읽어 SteeringParams에 반영한다.
bool load_steering_params_json(const std::string &path,
                               SteeringParams *params,
                               std::string *error);

// params/driving.json을 읽어 DrivingParams에 반영한다.
bool load_driving_params_json(const std::string &path,
                              DrivingParams *params,
                              std::string *error);
