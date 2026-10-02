/* 횡 플래너(LateralPlanner): laneless는 openpilot 메인의 get_curvature_from_plan과, 차선 변경은
 * 상류 desire_helper와 대조한다. 실험용 회전 desire와 path_offset_m의 적용 범위도 본다. MPC 자체의
 * 최적성은 gtest_lateral_mpc가 본다. */
#include "control_params.h"
#include "ipc_messages.h"
#include "lateral_controller.h"
#include "lateral_planner.h"
#include "model_output.h"
#include "vehicle_can.h"

#include <gtest/gtest.h>
#include <vector>

namespace {

/* laneless 모드는 openpilot 메인의 get_curvature_from_plan이다: 모델 plan의 yaw·yaw rate만 쓰고
 * plan 위치·경로 오프셋·MPC를 쓰지 않는다. */
TEST(LateralPlanner, LanelessUsesPlanYawLikeUpstream) {
  SteeringParams steering;
  steering.path_offset_m = -0.3f;
  DrivingParams driving;
  driving.laneless_mode = true;
  LateralPlanner planner(steering, driving);
  const float v = 20.0f;
  auto model_for = [&](float kappa, float lateral_offset) {
    ModelState ms{};
    ms.valid = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
      const float t = model_t_idx(i);
      ms.model_t[i] = t;
      ms.plan[i] = {v * t, lateral_offset, 0.0f};
      ms.plan_yaw[i] = kappa * v * t;
      ms.plan_yaw_rate[i] = kappa * v;
    }
    return ms;
  };
  VehicleCanState vehicle{};
  // 일정 곡률 plan: 2·ψ(t)/(v·t) − ψ̇/v = 2κ − κ = κ
  LateralTarget turn = planner.update(model_for(0.002f, 0.0f), vehicle, v, 0.0f, true);
  ASSERT_TRUE(turn.valid);
  ASSERT_TRUE(turn.laneless_mode);
  ASSERT_TRUE(turn.mpc_solution_valid);
  EXPECT_NEAR(lag_adjusted_curvature(turn, v, 0.0f, 0.34f), 0.002f, 1e-5f) << "일정 곡률 plan은 그 곡률";
  EXPECT_NEAR(lag_adjusted_curvature(turn, v, 0.1f, 0.34f), 0.002f, 1e-5f) << "plan 나이와 무관";
  // yaw가 0이면 plan이 옆으로 0.5 m 떨어져 있고 오프셋이 −0.3이어도 목표는 직진이다
  LateralTarget straight = planner.update(model_for(0.0f, 0.5f), vehicle, v, 0.0f, true);
  EXPECT_NEAR(lag_adjusted_curvature(straight, v, 0.05f, 0.34f), 0.0f, 1e-7f)
      << "위치와 경로 오프셋은 laneless 곡률에 들어가지 않는다";
}

/* 차선 변경은 openpilot desire_helper와 같다: 깜빡이 + 그 방향 핸들 토크로 시작하고, 모델이
 * 끝났다고 할 때(lane_change_prob < 0.02)나 10초·비활성으로만 끝난다. 차선선은 0.5초에 뺀다. */
TEST(LateralPlanner, LaneChangeFollowsUpstreamDesireHelper) {
  SteeringParams steering;
  DrivingParams driving;
  LateralPlanner planner(steering, driving);
  const float v = 20.0f;
  ModelState ms{};
  ms.valid = 1;
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float t = model_t_idx(i);
    ms.model_t[i] = t;
    ms.lane_t[i] = t;
    ms.plan[i] = {v * t, 0.0f, 0.0f};
  }
  ms.desire_state[0] = 1.0f;
  VehicleCanState vehicle{};
  LateralTarget r = planner.update(ms, vehicle, v, 0.0f, true);
  ASSERT_EQ(r.desire, 0);
  vehicle.left_blinker = true;
  r = planner.update(ms, vehicle, v, 0.0f, true);
  EXPECT_EQ(r.desire, 0) << "깜빡이만으로는 시작하지 않는다";
  vehicle.driver_torque = 300;  // 왼쪽으로 민다
  r = planner.update(ms, vehicle, v, 0.0f, true);
  EXPECT_EQ(r.desire, 3) << "깜빡이 방향으로 밀면 laneChangeLeft";
  // 모델이 차선 변경 중이라고 하는 동안(prob 높음)은 핸들을 놓아도 3초 넘게 이어진다.
  vehicle.driver_torque = 0;
  ms.desire_state[0] = 0.1f;
  ms.desire_state[3] = 0.9f;
  for (int i = 0; i < 60; ++i) r = planner.update(ms, vehicle, v, 0.0f, true);
  EXPECT_EQ(r.desire, 3) << "출력이 커져도 취소하지 않는다(예전 출력 0.8 취소 제거)";
  // 모델이 끝났다고 하면 마무리(차선선을 0.5초에 되살림) 동안은 openpilot 0.9.4 DESIRES처럼
  // laneChangeLeft를 유지하고, 끝나면 내린다.
  ms.desire_state[0] = 1.0f;
  ms.desire_state[3] = 0.0f;
  vehicle.left_blinker = false;
  r = planner.update(ms, vehicle, v, 0.0f, true);
  EXPECT_EQ(r.desire, 3) << "마무리 단계";
  for (int i = 0; i < 12; ++i) r = planner.update(ms, vehicle, v, 0.0f, true);
  EXPECT_EQ(r.desire, 0) << "차선선을 되살리면 끝난다";
}

/* 회전 desire(실험): 저속 + 깜빡이 + 결합 중이면 turnLeft/turnRight를 2.5초마다 다시 올리고,
 * 그동안 차선 모드라도 모델 경로를 따른다. 스위치·깜빡이·속도·비활성 어느 것이든 풀리면 끝. */
TEST(LateralPlanner, TurnDesireRepulsesAtLowSpeedWithBlinker) {
  SteeringParams steering;
  DrivingParams driving;
  driving.turn_desire = true;
  LateralPlanner planner(steering, driving);
  ModelState ms{};
  ms.valid = 1;
  const float v = 5.0f;  // 18 km/h < 30
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float t = model_t_idx(i);
    ms.model_t[i] = t;
    ms.lane_t[i] = t;
    ms.plan[i] = {v * t, 0.0f, 0.0f};
  }
  for (int l = 0; l < 4; ++l) ms.lane_probabilities[l] = 0.9f;  // 차선이 뚜렷해도
  ms.desire_state[0] = 1.0f;
  VehicleCanState vehicle{};
  vehicle.left_blinker = true;
  std::vector<int> seq;
  bool model_path = true;
  for (int i = 0; i < 120; ++i) {  // 6초
    const LateralTarget r = planner.update(ms, vehicle, v, 0.0f, true);
    seq.push_back(r.desire);
    model_path = model_path && r.laneless_mode;
  }
  EXPECT_EQ(seq[0], 1) << "깜빡이를 켜자마자 turnLeft";
  EXPECT_EQ(seq[24], 1);
  EXPECT_EQ(seq[25], 0) << "1.25초 뒤 내려 다음 rising edge를 만든다";
  EXPECT_EQ(seq[50], 1) << "2.5초마다 다시 올린다";
  EXPECT_EQ(seq[100], 1);
  EXPECT_TRUE(model_path) << "회전 desire 동안은 모델 경로";

  vehicle.left_blinker = false;
  vehicle.right_blinker = true;
  EXPECT_EQ(planner.update(ms, vehicle, v, 0.0f, true).desire, 2) << "오른쪽은 turnRight";
  EXPECT_EQ(planner.update(ms, vehicle, 12.0f, 0.0f, true).desire, 0) << "차선 변경 속도 이상이면 아님";
  EXPECT_EQ(planner.update(ms, vehicle, v, 0.0f, false).desire, 0) << "비활성이면 아님";
  vehicle.right_blinker = false;
  EXPECT_EQ(planner.update(ms, vehicle, v, 0.0f, true).desire, 0) << "깜빡이를 끄면 끝";

  DrivingParams off;
  LateralPlanner plain(steering, off);
  vehicle.left_blinker = true;
  EXPECT_EQ(plain.update(ms, vehicle, v, 0.0f, true).desire, 0) << "스위치가 꺼져 있으면 openpilot과 같다";

  /* 빠를 때(35 km/h) 켠 깜빡이는 차선 변경 대기다. 감속해 25 km/h가 돼도 회전으로 바꾸지 않고,
   * 깜빡이를 다시 켜면(그 속도 미만) 회전이다. */
  LateralPlanner slowing(steering, driving);
  vehicle = VehicleCanState{};
  vehicle.left_blinker = true;
  EXPECT_EQ(slowing.update(ms, vehicle, 35.0f / 3.6f, 0.0f, true).desire, 0) << "차선 변경 대기(넛지 전)";
  bool turned = false;
  for (int i = 0; i < 40; ++i) turned = turned || slowing.update(ms, vehicle, 25.0f / 3.6f, 0.0f, true).desire != 0;
  EXPECT_FALSE(turned) << "차선 변경 의도로 켠 깜빡이는 감속해도 회전이 아니다";
  vehicle.left_blinker = false;
  slowing.update(ms, vehicle, 25.0f / 3.6f, 0.0f, true);
  vehicle.left_blinker = true;
  EXPECT_EQ(slowing.update(ms, vehicle, 25.0f / 3.6f, 0.0f, true).desire, 1) << "저속에서 새로 켜면 회전";
}

/* path_offset_m은 차선 중심에만 적용된다. 차선이 없어 모델 경로로 넘어가면(교차로) 적용하지
 * 않는다: 차 기준인 모델 경로에 더하면 위치 고정점 없이 차가 오프셋 쪽으로 계속 밀린다. */
TEST(LateralPlanner, PathOffsetOnlyShiftsLanePath) {
  SteeringParams steering;
  steering.path_offset_m = -0.3f;
  DrivingParams driving;
  LateralPlanner planner(steering, driving);
  const float v = 15.0f;
  auto model_for = [&](float lane_prob) {
    ModelState ms{};
    ms.valid = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
      const float t = model_t_idx(i);
      ms.model_t[i] = t;
      ms.lane_t[i] = t;
      ms.plan[i] = {v * t, 0.0f, 0.0f};
      ms.lanes[1][i] = {v * t, -1.75f, 0.0f};
      ms.lanes[2][i] = {v * t, 1.75f, 0.0f};
    }
    ms.lane_probabilities[1] = ms.lane_probabilities[2] = lane_prob;
    ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
    ms.desire_state[0] = 1.0f;
    return ms;
  };
  VehicleCanState vehicle{};
  LateralTarget r;
  for (int i = 0; i < 100; ++i) r = planner.update(model_for(0.99f), vehicle, v, 0.0f, true);
  ASSERT_FALSE(r.laneless_mode);
  EXPECT_NEAR(r.target_y_m, -0.3f, 0.03f) << "차선이 보이면 차선 중심에서 오프셋만큼";
  for (int i = 0; i < 100; ++i) r = planner.update(model_for(0.0f), vehicle, v, 0.0f, true);
  ASSERT_TRUE(r.laneless_mode);
  EXPECT_NEAR(r.target_y_m, 0.0f, 0.01f) << "모델 경로로 넘어가면 오프셋을 더하지 않는다";
}

}  // namespace
