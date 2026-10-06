#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "car/can_frame.h"

struct HyundaiSteeringLimits {
  int steer_max = 384;
  int steer_delta_up = 3;
  int steer_delta_down = 7;
  int steer_driver_allowance = 50;
  int steer_driver_multiplier = 2;
  int steer_driver_factor = 1;
};

int apply_hyundai_steer_torque_limits(int desired_torque, int last_torque, int driver_torque,
                                      const HyundaiSteeringLimits &limits = HyundaiSteeringLimits{});

float mdps_speed_for_lkas(float cluster_speed_raw, bool lkas_active, bool is_mph,
                          float spoof_speed_kph = 60.0f);

struct HyundaiLkas11Values {
  int ldws_active_mode = 0;
  int ldws_sys_state = 0;
  int sys_warning = 0;
  int left_lane_depart = 0;
  int right_lane_depart = 0;
  int hba_lamp = 0;
  int fcw_bas_req = 0;
  int steer_torque = 0;
  bool steer_req = false;
  bool toi_fault = false;
  int hba_sys_state = 0;
  int fcw_opt = 0;
  int hba_opt = 0;
  int msg_count = 0;
  int fcw_sys_state = 0;
  int fcw_collision_warning = 0;
  int fusion_state = 0;
  int unknown1 = 0;
  int fcw_opt_usm = 0;
  int ldws_opt_usm = 0;
  int unknown2 = 0;
};

struct HyundaiClu11Values {
  int cruise_sw_state = 0;
  int cruise_sw_main = 0;
  int sld_main_sw = 0;
  int parity_bit = 0;
  float speed_decimal = 0.0f;
  float speed = 0.0f;
  bool speed_unit_mph = false;
  int detent_out = 0;
  int rheostat_level = 0;
  int clu_info = 0;
  int amp_info = 0;
  int alive_count = 0;
};

struct HyundaiLkasCommand {
  int apply_steer = 0;
  bool steer_req = false;
  bool cut_steer_temp = false;
  bool sys_warning = false;
  int sys_state = 0;
  bool left_lane = false;
  bool right_lane = false;
  int left_lane_depart = 0;
  int right_lane_depart = 0;
  int lkas_msg_count = 0;
  bool ldws_fix = false;
};

struct HyundaiCluCommand {
  int button = 0;
  float speed = 0.0f;
  int frame = 0;
};


HyundaiLkas11Values decode_lkas11(const std::array<uint8_t, 8> &data);
HyundaiClu11Values decode_clu11(const std::array<uint8_t, 4> &data);

uint8_t hyundai_lkas11_checksum(const std::array<uint8_t, 8> &data);
CanFrame create_lkas11_frame(const HyundaiLkas11Values &seed, const HyundaiLkasCommand &command,
                             uint8_t bus);
CanFrame create_clu11_frame(const HyundaiClu11Values &seed, const HyundaiCluCommand &command,
                            uint8_t bus);
// 최신 MDPS12 seed에서 openpilot create_mdps12와 같은 오류 회피 frame을 만든다.
CanFrame create_mdps12_frame(const std::array<uint8_t, 8> &seed, int frame);

/* K7 커뮤니티 하네스(MDPS가 버스 1)의 횡제어 프레임을 송신 순서대로: LKAS11(파워트레인 버스),
 * LKAS11(MDPS 버스), 홀수 프레임이면 MDPS가 보는 CLU11 속도 바꿔치기(MDPS 버스), MDPS12 오류 회피
 * 프레임(카메라 버스). */
std::vector<CanFrame> build_lateral_can_frames(const HyundaiLkas11Values &lkas_seed,
                                               const HyundaiClu11Values &clu_seed,
                                               const std::array<uint8_t, 8> &mdps12_seed,
                                               const HyundaiLkasCommand &lkas_command,
                                               float mdps_speed_spoof_kph, bool lkas_active,
                                               bool is_mph, int frame);

// 비전 크루즈의 버튼 펄스: 운전자 CLU11 seed에 버튼만 바꿔 파워트레인 버스로 보낸다.
CanFrame create_cruise_button_frame(const HyundaiClu11Values &clu_seed, int button, int frame);
