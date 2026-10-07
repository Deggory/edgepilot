#include "planning/desire_helper.h"

#include "car/vehicle_can.h"

#include <algorithm>
#include <cstdlib>

namespace {

constexpr double kDtModel = 0.05;  // 모델 프레임 주기(20 Hz)

}  // namespace

/* openpilot desire_helper(0.9.4 차선선 페이드 포함)와 같다. 꺼지는 조건은 조향 비활성과 10초
 * 초과뿐이다. 예전에는 포크에서 온 두 가지가 더 있었다: 출력 0.8 이상이 0.5초 이어지면
 * 차선 변경 취소(차선이 차를 붙잡아 운전자와 싸우는 바로 그 순간 취소됐다), 속도별로 느린
 * 차선선 페이드(60 km/h에서 2.5초). 2026-09-27 실차에서 운전자가 핸들을 한참 잡아야 해서
 * 둘 다 upstream으로 되돌렸다. */
void DesireHelper::update(const VehicleCanState &vehicle, float v_ego, bool active,
                          const DesireModelInputs &model) {
  const bool one_blinker = vehicle.left_blinker != vehicle.right_blinker;
  const bool below_speed = v_ego < params_.lane_change_min_speed_mps;
  int direction_now = direction_;
  if (vehicle.left_blinker) direction_now = -1;
  if (vehicle.right_blinker) direction_now = 1;

  /* 깜빡이 쪽이 도로 경계(경계는 또렷하고 그 너머 차선선은 없음)면 차선 변경을 시작하지 않는다
   * (dragonpilot의 도로 경계 판정). 막힌 동안 깜빡이를 꺼진 것으로 두므로, 경계가 사라지면 깜빡이를
   * 켠 채로도 그 틱에 시작한다. */
  const double left_edge_prob = std::clamp(1.0 - model.road_edge_stds[0], 0.0, 1.0);
  const double right_edge_prob = std::clamp(1.0 - model.road_edge_stds[1], 0.0, 1.0);
  const double left_nearside_prob = model.lane_probabilities[0];
  const double right_nearside_prob = model.lane_probabilities[3];
  const int road_edge = right_edge_prob > 0.35 && right_nearside_prob < 0.2 &&
                                left_nearside_prob >= right_nearside_prob
      ? 1
      : left_edge_prob > 0.35 && left_nearside_prob < 0.2 &&
                                right_nearside_prob >= left_nearside_prob
          ? -1 : 0;
  const int lane_direction = vehicle.left_blinker ? -1 : vehicle.right_blinker ? 1 : 2;
  const bool road_edge_blocked = lane_change_state_ == LaneChangeState::Off && road_edge == lane_direction;

  if (road_edge_blocked) {
    direction_ = 0;
  } else if (!active || lane_change_timer_ > 10.0) {
    lane_change_state_ = LaneChangeState::Off;
    direction_ = 0;
  } else {
    const bool steering_pressed =
        std::abs(vehicle.driver_torque) > params_.steering_pressed_threshold;
    const bool torque_applied = steering_pressed &&
        ((vehicle.driver_torque > 0 && direction_ == -1) ||
         (vehicle.driver_torque < 0 && direction_ == 1));
    const bool blindspot_detected =
        (vehicle.left_blindspot && direction_ == -1) ||
        (vehicle.right_blindspot && direction_ == 1);
    if (lane_change_state_ == LaneChangeState::Off && one_blinker && !previous_one_blinker_ &&
        !below_speed) {
      lane_change_state_ = LaneChangeState::PreLaneChange;
      direction_ = direction_now;
      lane_change_lane_prob_ = 1.0;
    } else if (lane_change_state_ == LaneChangeState::PreLaneChange) {
      if (!one_blinker || below_speed) {
        lane_change_state_ = LaneChangeState::Off;
      } else if (!blindspot_detected && torque_applied) {
        lane_change_state_ = LaneChangeState::Starting;
      }
    } else if (lane_change_state_ == LaneChangeState::Starting) {
      // 0.5초에 걸쳐 차선선을 뺀다(openpilot "fade out over .5s").
      lane_change_lane_prob_ = std::max(0.0, lane_change_lane_prob_ - 2.0 * kDtModel);
      if (model.lane_change_prob < 0.02 && lane_change_lane_prob_ < 0.01)
        lane_change_state_ = LaneChangeState::Finishing;
    } else if (lane_change_state_ == LaneChangeState::Finishing) {
      // 복구 0.5초 (openpilot 기본 1.0초). 변경 직후 새 차선 적응을 당긴다.
      lane_change_lane_prob_ = std::min(1.0, lane_change_lane_prob_ + 2.0 * kDtModel);
      if (lane_change_lane_prob_ > 0.99) {
        lane_change_state_ = one_blinker ? LaneChangeState::PreLaneChange : LaneChangeState::Off;
        if (!one_blinker) direction_ = 0;
      }
    }
  }

  const bool changing = lane_change_state_ == LaneChangeState::Starting ||
                        lane_change_state_ == LaneChangeState::Finishing;
  lane_change_timer_ = changing ? lane_change_timer_ + kDtModel : 0.0;
  previous_one_blinker_ = road_edge_blocked ? false : one_blinker;
  desire_ = changing && direction_ == -1 ? Desire::LaneChangeLeft
      : changing && direction_ == 1 ? Desire::LaneChangeRight : Desire::None;

  /* 회전 desire(실험, DrivingParams::turn_desire): 차선 변경 속도 미만 + 깜빡이 하나 + 결합 중 +
   * 차선 변경이 진행 중이 아님(상태 0). 빠를 때 켠 깜빡이도 그 속도 아래까지 켜져 있으면 회전으로
   * 본다. 회전 차로로 차선을 바꾸거나 미리 깜빡이를 켜고 감속해 도는 순서가 흔하다(2026-10-03 실차:
   * 회전 6번 중 3번이 30 km/h 위에서 켰다). 변경 대기(1)는 감속하면 0이 되고, 변경을 마친 뒤
   * 깜빡이가 남으면 대기(1)로 돌아갔다가 감속하면 0이 된다. 진행 중인 변경(2, 3)이 먼저다.
   * 차선 변경 뒤 깜빡이를 켠 채 정체로 감속해도 회전 의도가 들어간다는 뜻이라 운전자가 바로잡는다.
   * 모델 desire 입력은 rising edge 펄스이고 5초(100틱) 뒤 빠지므로 2.5초마다 한 번 내렸다
   * 다시 올린다(깜빡이를 끄면 modeld가 이력에서 지운다). */
  turn_desire_active_ = params_.turn_desire_enabled && active && one_blinker && below_speed &&
                        lane_change_state_ == LaneChangeState::Off;
  turn_desire_direction_ = turn_desire_active_ ? (vehicle.left_blinker ? 1 : 2) : 0;
  if (turn_desire_active_) {
    const bool on = turn_desire_ticks_ % kTurnRepulseTicks < kTurnRepulseTicks / 2;
    desire_ = on ? static_cast<Desire>(turn_desire_direction_) : Desire::None;
    ++turn_desire_ticks_;
  } else {
    turn_desire_ticks_ = 0;
  }
}
