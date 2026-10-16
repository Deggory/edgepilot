// 횡 플래너(LateralPlanner): laneless는 openpilot 메인의 get_curvature_from_plan과, 차선 변경은
// 상류 desire_helper와 대조한다. path_offset_m의 적용 범위도 본다. MPC 자체의 최적성은
// gtest_lateral_mpc가 본다.
#include "car/vehicle_can.h"
#include "common/ipc_messages.h"
#include "common/model_output.h"
#include "controls/control_params.h"
#include "controls/lateral_controller.h"
#include "planning/lateral_planner.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace {

// laneless 모드는 openpilot 메인의 get_curvature_from_plan이다: 모델 plan의 yaw·yaw rate만 쓰고
// plan 위치·경로 오프셋·MPC를 쓰지 않는다.
TEST(LateralPlanner, LanelessUsesPlanYawLikeUpstream) {
    SteeringParams steering;
    steering.path_offset_m = -0.3f;
    steering.path_mode = static_cast<int>(PathMode::Laneless);
    LateralPlanner planner(steering);
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
    LateralTarget turn = planner.update(model_for(0.002f, 0.0f), nullptr, vehicle, v, 0.0f, true, false);
    ASSERT_TRUE(turn.valid);
    ASSERT_TRUE(turn.laneless_mode);
    ASSERT_TRUE(turn.mpc_solution_valid);
    EXPECT_NEAR(lag_adjusted_curvature(turn, v, 0.0f, 0.34f), 0.002f, 1e-5f) << "일정 곡률 plan은 그 곡률";
    EXPECT_NEAR(lag_adjusted_curvature(turn, v, 0.1f, 0.34f), 0.002f, 1e-5f) << "plan 나이와 무관";
    // yaw가 0이면 plan이 옆으로 0.5 m 떨어져 있고 오프셋이 −0.3이어도 목표는 직진이다
    LateralTarget straight = planner.update(model_for(0.0f, 0.5f), nullptr, vehicle, v, 0.0f, true, false);
    EXPECT_NEAR(lag_adjusted_curvature(straight, v, 0.05f, 0.34f), 0.0f, 1e-7f)
        << "위치와 경로 오프셋은 laneless 곡률에 들어가지 않는다";
}

// 차선 변경은 openpilot desire_helper와 같다: 깜빡이 + 그 방향 핸들 토크로 시작하고, 모델이
// 끝났다고 할 때(lane_change_prob < 0.02)나 10초·비활성으로만 끝난다. 차선선은 0.5초에 뺀다.
TEST(LateralPlanner, LaneChangeFollowsUpstreamDesireHelper) {
    SteeringParams steering;
    LateralPlanner planner(steering);
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
    LateralTarget r = planner.update(ms, nullptr, vehicle, v, 0.0f, true, false);
    ASSERT_EQ(r.desire, 0);
    vehicle.left_blinker = true;
    r = planner.update(ms, nullptr, vehicle, v, 0.0f, true, false);
    EXPECT_EQ(r.desire, 0) << "깜빡이만으로는 시작하지 않는다";
    EXPECT_EQ(r.lane_change_state, 1) << "HUD: 핸들을 밀기를 기다린다";
    EXPECT_EQ(r.lane_change_direction, -1);
    vehicle.driver_torque = 300;  // 왼쪽으로 민다(컨트롤러가 거른 운전자 조향)
    r = planner.update(ms, nullptr, vehicle, v, 0.0f, true, true);
    EXPECT_EQ(r.desire, 3) << "깜빡이 방향으로 밀면 laneChangeLeft";
    EXPECT_EQ(r.lane_change_state, 2) << "HUD: 변경 중";
    // 모델이 차선 변경 중이라고 하는 동안(prob 높음)은 핸들을 놓아도 3초 넘게 이어진다.
    vehicle.driver_torque = 0;
    ms.desire_state[0] = 0.1f;
    ms.desire_state[3] = 0.9f;
    for (int i = 0; i < 60; ++i) r = planner.update(ms, nullptr, vehicle, v, 0.0f, true, false);
    EXPECT_EQ(r.desire, 3) << "출력이 커져도 취소하지 않는다(예전 출력 0.8 취소 제거)";
    // 모델이 끝났다고 하면 마무리(차선선을 0.5초에 되살림) 동안은 openpilot 0.9.4 DESIRES처럼
    // laneChangeLeft를 유지하고, 끝나면 내린다.
    ms.desire_state[0] = 1.0f;
    ms.desire_state[3] = 0.0f;
    vehicle.left_blinker = false;
    r = planner.update(ms, nullptr, vehicle, v, 0.0f, true, false);
    EXPECT_EQ(r.desire, 3) << "마무리 단계";
    for (int i = 0; i < 12; ++i) r = planner.update(ms, nullptr, vehicle, v, 0.0f, true, false);
    EXPECT_EQ(r.desire, 0) << "차선선을 되살리면 끝난다";
    EXPECT_EQ(r.lane_change_state, 0);
}

// 차선 변경 속도 아래에서 켠 깜빡이는 desire를 주지 않는다(상류 desire_helper). 빠를 때 시작한 차선 변경은
// 그 속도 아래로 감속해도 이어진다.
TEST(LateralPlanner, SlowBlinkerGivesNoDesire) {
    SteeringParams steering;
    LateralPlanner planner(steering);
    ModelState ms{};
    ms.valid = 1;
    const float v = 5.0f;  // 18 km/h < 30
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float t = model_t_idx(i);
        ms.model_t[i] = t;
        ms.lane_t[i] = t;
        ms.plan[i] = {v * t, 0.0f, 0.0f};
    }
    for (int l = 0; l < 4; ++l) ms.lane_probabilities[l] = 0.9f;
    ms.desire_state[0] = 1.0f;
    VehicleCanState vehicle{};
    vehicle.left_blinker = true;
    for (int i = 0; i < 60; ++i) {  // 3초
        const LateralTarget r = planner.update(ms, nullptr, vehicle, v, 0.0f, true, false);
        ASSERT_EQ(r.desire, 0) << i;
        ASSERT_EQ(r.lane_change_state, 0) << i;
    }

    LateralPlanner changing(steering);
    vehicle = VehicleCanState{};
    vehicle.left_blinker = true;
    changing.update(ms, nullptr, vehicle, 35.0f / 3.6f, 0.0f, true, false);
    vehicle.driver_torque = 300;  // 왼쪽 넛지(컨트롤러가 거른 운전자 조향)
    EXPECT_EQ(changing.update(ms, nullptr, vehicle, 35.0f / 3.6f, 0.0f, true, true).desire, 3) << "차선 변경 시작";
    vehicle.driver_torque = 0;
    EXPECT_EQ(changing.update(ms, nullptr, vehicle, 25.0f / 3.6f, 0.0f, true, false).desire, 3)
        << "시작한 변경은 감속해도 이어진다";
}

// path_offset_m은 차선 중심에만 적용된다. 차선이 없어 모델 경로로 넘어가면(교차로) 적용하지
// 않는다: 차 기준인 모델 경로에 더하면 위치 고정점 없이 차가 오프셋 쪽으로 계속 밀린다.
TEST(LateralPlanner, PathOffsetOnlyShiftsLanePath) {
    SteeringParams steering;
    steering.path_offset_m = -0.3f;
    LateralPlanner planner(steering);
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
    for (int i = 0; i < 100; ++i) r = planner.update(model_for(0.99f), nullptr, vehicle, v, 0.0f, true, false);
    ASSERT_FALSE(r.laneless_mode);
    EXPECT_NEAR(r.target_y_m, -0.3f, 0.03f) << "차선이 보이면 차선 중심에서 오프셋만큼";
    for (int i = 0; i < 100; ++i) r = planner.update(model_for(0.0f), nullptr, vehicle, v, 0.0f, true, false);
    ASSERT_TRUE(r.laneless_mode);
    EXPECT_NEAR(r.target_y_m, 0.0f, 0.01f) << "모델 경로로 넘어가면 오프셋을 더하지 않는다";
}

// Lane 모드에서 차선이 안 보이면(교차로) laneless 모드와 같은 openpilot 메인 계산으로 목표를 낸다:
// plan 위치가 직진이어도 plan yaw가 곡률 κ를 말하면 목표는 κ다. 차선을 잃는 순간에는 한 프레임에
// 바꾸지 않고 0.5초에 걸쳐 섞는다.
TEST(LateralPlanner, LaneModeFallbackMatchesLaneless) {
    const float v = 6.0f;
    const float kappa = 0.02f;
    auto model_for = [&](float lane_prob) {
        ModelState ms{};
        ms.valid = 1;
        for (int i = 0; i < kTrajectorySize; ++i) {
            const float t = model_t_idx(i);
            ms.model_t[i] = t;
            ms.lane_t[i] = t;
            ms.plan[i] = {v * t, 0.0f, 0.0f};  // 위치는 직진
            ms.plan_yaw[i] = kappa * v * t;    // yaw는 κ로 회전
            ms.plan_yaw_rate[i] = kappa * v;
            ms.lanes[1][i] = {v * t, -1.75f, 0.0f};
            ms.lanes[2][i] = {v * t, 1.75f, 0.0f};
        }
        ms.lane_probabilities[1] = ms.lane_probabilities[2] = lane_prob;
        ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
        ms.desire_state[0] = 1.0f;
        return ms;
    };
    SteeringParams steering;
    LateralPlanner lane_mode(steering);
    steering.path_mode = static_cast<int>(PathMode::Laneless);
    LateralPlanner laneless(steering);
    VehicleCanState vehicle{};
    LateralTarget r;
    for (int i = 0; i < 60; ++i) r = lane_mode.update(model_for(0.99f), nullptr, vehicle, v, 0.0f, true, false);
    ASSERT_FALSE(r.laneless_mode);
    const float with_lanes = lag_adjusted_curvature(r, v, 0.0f, 0.42f);
    EXPECT_LT(std::fabs(with_lanes), 0.3f * kappa) << "차선이 보이면 차선(직진)을 따른다";

    std::vector<float> handoff;
    for (int i = 0; i < 40; ++i) {
        r = lane_mode.update(model_for(0.0f), nullptr, vehicle, v, 0.0f, true, false);
        handoff.push_back(lag_adjusted_curvature(r, v, 0.0f, 0.42f));
    }
    ASSERT_TRUE(r.laneless_mode) << "차선을 잃으면 모델 경로 구간";
    const LateralTarget ref = laneless.update(model_for(0.0f), nullptr, vehicle, v, 0.0f, true, false);
    EXPECT_NEAR(handoff.back(), lag_adjusted_curvature(ref, v, 0.0f, 0.42f), 1e-6f)
        << "자리 잡은 뒤에는 laneless 모드와 같은 목표";
    EXPECT_NEAR(handoff.back(), kappa, 1e-4f);
    float max_step = 0.0f;
    for (size_t i = 1; i < handoff.size(); ++i) max_step = std::max(max_step, std::fabs(handoff[i] - handoff[i - 1]));
    EXPECT_LT(max_step, 0.25f * kappa) << "인계는 한 프레임에 넘어가지 않는다";
}

// lane_path_weight는 Lane 모드 MPC가 차선 중심에서 벗어난 위치를 되돌리는 세기다. 차가 차선 중심에서
// 0.3 m 오른쪽에 있으면 가중치 3이 1보다 왼쪽(음) 곡률을 더 크게 요구한다.
TEST(LateralPlanner, LanePathWeightStrengthensCentering) {
    const float v = 20.0f;
    auto model_for = [&]() {
        ModelState ms{};
        ms.valid = 1;
        for (int i = 0; i < kTrajectorySize; ++i) {
            const float t = model_t_idx(i);
            ms.model_t[i] = t;
            ms.lane_t[i] = t;
            ms.plan[i] = {v * t, 0.0f, 0.0f};
            ms.lanes[1][i] = {v * t, -2.05f, 0.0f};  // 차가 차선 중심보다 0.3 m 오른쪽
            ms.lanes[2][i] = {v * t, 1.45f, 0.0f};
        }
        ms.lane_probabilities[1] = ms.lane_probabilities[2] = 0.99f;
        ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
        ms.desire_state[0] = 1.0f;
        return ms;
    };
    auto desired = [&](float weight) {
        SteeringParams steering;
        steering.lane_path_weight = weight;
        LateralPlanner planner(steering);
        VehicleCanState vehicle{};
        LateralTarget r;
        for (int i = 0; i < 40; ++i) r = planner.update(model_for(), nullptr, vehicle, v, 0.0f, true, false);
        EXPECT_FALSE(r.laneless_mode);
        return lag_adjusted_curvature(r, v, 0.0f, 0.42f);
    };
    const float upstream = desired(1.0f);
    const float stronger = desired(3.0f);
    EXPECT_LT(upstream, 0.0f) << "차선 중심(왼쪽)으로 돌아간다";
    EXPECT_LT(stronger, upstream * 1.2f) << "가중치 3은 더 강하게 되돌린다";
}

// Lanebow 시험용 프레임: X_IDXS 위의 supercombo 선(확률 0.99), 직진 plan, 증가하는 촬영 시각(시간 필터가 이어진다).
ModelState lanebow_model(int frame, float v, float sc_left, float sc_right, float sc_prob = 0.99f, float kappa = 0.0f) {
    ModelState ms{};
    ms.valid = 1;
    ms.capture_timestamp_ns = 1'000'000'000ULL + static_cast<uint64_t>(frame) * 50'000'000ULL;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float t = model_t_idx(i);
        const float x = model_x_idx(i);
        ms.model_t[i] = t;
        ms.lane_t[i] = x / v;
        ms.plan[i] = {v * t, 0.0f, 0.0f};
        ms.plan_yaw[i] = kappa * v * t;
        ms.plan_yaw_rate[i] = kappa * v;
        ms.lanes[1][i] = {x, sc_left, 1.2f};
        ms.lanes[2][i] = {x, sc_right, 1.2f};
    }
    ms.lane_probabilities[1] = ms.lane_probabilities[2] = sc_prob;
    ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
    ms.desire_state[0] = 1.0f;
    return ms;
}

// 같은 프레임의 LD 결과: 자차 선 둘(없애려면 NaN), 36 m까지, 신뢰도 0.9
LaneState lanebow_lane(int frame, float left, float right) {
    LaneState lane;
    lane.frame_id = static_cast<uint64_t>(frame);
    lane.capture_timestamp_ns = 1'000'000'000ULL + static_cast<uint64_t>(frame) * 50'000'000ULL;
    lane.valid = 1;
    for (const auto &[slot, y] :
         {std::pair<int8_t, float>{kLaneSlotEgoLeft, left}, std::pair<int8_t, float>{kLaneSlotEgoRight, right}}) {
        if (!std::isfinite(y)) continue;
        LaneRoadLine &road = lane.road[lane.road_count++];
        road.kind = kLaneRoadKindLine;
        road.slot = slot;
        road.confidence = 0.9f;
        for (unsigned i = 0; i < kLaneRoadPoints && model_x_idx(static_cast<int>(i)) <= 36.0f; ++i) road.y[i] = y;
    }
    return lane;
}

// Lanebow는 Lane 모드와 같은 융합을 LD 차선으로 한다: supercombo는 차가 가운데라 하고 LD는 차로 중심이 0.3 m
// 오른쪽이라 하면, Lanebow는 오른쪽으로(양의 곡률) 가고 Lane 모드는 가만히 있다.
TEST(LateralPlanner, LanebowFollowsTheLdLanes) {
    const float v = 20.0f;
    SteeringParams steering;
    LateralPlanner lane_mode(steering);
    steering.path_mode = static_cast<int>(PathMode::Lanebow);
    LateralPlanner lanebow(steering);
    VehicleCanState vehicle{};
    LateralTarget lane_r, bow_r;
    for (int k = 0; k < 60; ++k) {
        const ModelState model = lanebow_model(k, v, -1.75f, 1.75f);
        const LaneState lane = lanebow_lane(k, -1.45f, 2.05f);
        lane_r = lane_mode.update(model, &lane, vehicle, v, 0.0f, true, false);
        bow_r = lanebow.update(model, &lane, vehicle, v, 0.0f, true, false);
    }
    ASSERT_FALSE(bow_r.laneless_mode);
    EXPECT_EQ(bow_r.lane_frame_id, 59u) << "이번 프레임에 쓴 LD";
    EXPECT_EQ(lane_r.lane_frame_id, 0u) << "Lane 모드는 LD를 쓰지 않는다";
    EXPECT_NEAR(bow_r.lane_left_y_m, -1.45f, 0.01f) << "관측값도 LD";
    EXPECT_GT(lag_adjusted_curvature(bow_r, v, 0.0f, 0.42f), 1e-4f) << "LD 차로 중심(오른쪽)으로";
    EXPECT_NEAR(lag_adjusted_curvature(lane_r, v, 0.0f, 0.42f), 0.0f, 2e-5f) << "Lane 모드는 supercombo 차선";
}

// LD가 없으면(플러그인 없음, 낡음) Lanebow는 차선을 잃은 Lane 모드처럼 모델 경로로 가고, 자리 잡으면 Laneless와 같다.
TEST(LateralPlanner, LanebowWithoutLdMatchesLaneless) {
    const float v = 6.0f, kappa = 0.02f;
    SteeringParams steering;
    steering.path_mode = static_cast<int>(PathMode::Lanebow);
    LateralPlanner lanebow(steering);
    steering.path_mode = static_cast<int>(PathMode::Laneless);
    LateralPlanner laneless(steering);
    VehicleCanState vehicle{};
    LateralTarget bow_r, ref;
    for (int k = 0; k < 40; ++k) {
        const ModelState model = lanebow_model(k, v, -1.75f, 1.75f, 0.99f, kappa);
        bow_r = lanebow.update(model, nullptr, vehicle, v, 0.0f, true, false);
        ref = laneless.update(model, nullptr, vehicle, v, 0.0f, true, false);
    }
    ASSERT_TRUE(bow_r.laneless_mode) << "supercombo 차선이 뚜렷해도 LD가 없으면 모델 경로";
    EXPECT_NEAR(lag_adjusted_curvature(bow_r, v, 0.0f, 0.42f), lag_adjusted_curvature(ref, v, 0.0f, 0.42f), 1e-6f);
}

// LD 선이 한쪽만 보여도 그 선과 차로 폭으로 Lanebow를 이어 간다.
TEST(LateralPlanner, LanebowKeepsGoingOnOneLine) {
    const float v = 20.0f;
    SteeringParams steering;
    steering.path_mode = static_cast<int>(PathMode::Lanebow);
    LateralPlanner planner(steering);
    VehicleCanState vehicle{};
    LateralTarget r;
    for (int k = 0; k < 60; ++k) {
        const LaneState lane = lanebow_lane(k, -1.75f, 1.75f);
        r = planner.update(lanebow_model(k, v, -1.75f, 1.75f), &lane, vehicle, v, 0.0f, true, false);
    }
    for (int k = 60; k < 120; ++k) {
        const LaneState lane = lanebow_lane(k, -1.75f, std::numeric_limits<float>::quiet_NaN());
        r = planner.update(lanebow_model(k, v, -1.75f, 1.75f), &lane, vehicle, v, 0.0f, true, false);
        ASSERT_FALSE(r.laneless_mode) << "frame " << k;
    }
    // 한 선이면 Lane 모드처럼 그 선에서 차로 폭의 절반: 양쪽이 안 보이는 동안 폭은 속도별 기본 폭으로 옮겨 간다
    EXPECT_NEAR(r.target_y_m, -1.75f + 0.5f * r.lane_width_m, 0.02f);
    EXPECT_GT(r.lane_width_m, 3.2f);
}

// LD 선이 뚜렷하면 supercombo 선이 LD 시야 너머에서 벌어져도(먼 곳 오차) Lanebow는 차선을 놓지 않는다. 시야 너머를
// 선마다 다른 모양으로 이으면 먼 폭이 커져 폭 검사가 확률을 깎아 laneless로 넘어갔다(2026-10-15 리허설).
TEST(LateralPlanner, LanebowIgnoresFarSupercomboDivergence) {
    const float v = 22.0f;
    SteeringParams steering;
    steering.path_mode = static_cast<int>(PathMode::Lanebow);
    LateralPlanner planner(steering);
    VehicleCanState vehicle{};
    for (int k = 0; k < 80; ++k) {
        ModelState model = lanebow_model(k, v, -1.75f, 1.75f);
        model.lane_probabilities[2] = 0.9f;  // 오른선이 먼 곳에서 오른쪽으로 벌어진다
        for (int i = 0; i < kTrajectorySize; ++i) {
            const float x = model_x_idx(i);
            if (x > 40.0f) model.lanes[2][i].y += 0.002f * (x - 40.0f) * (x - 40.0f);
        }
        const LaneState lane = lanebow_lane(k, -1.75f, 1.75f);
        const LateralTarget r = planner.update(model, &lane, vehicle, v, 0.0f, true, false);
        if (k >= 20) ASSERT_FALSE(r.laneless_mode) << "frame " << k;
    }
}

// 실행 중에 Lane과 Lanebow를 오가도 두 출처가 늘 데워져 있어 목표 곡률이 크게 튀지 않는다.
TEST(LateralPlanner, LanebowSwitchIsSmooth) {
    const float v = 20.0f;
    SteeringParams steering;
    LateralPlanner planner(steering);
    VehicleCanState vehicle{};
    std::vector<float> curvature;
    for (int k = 0; k < 120; ++k) {
        if (k == 40 || k == 80) {
            steering.path_mode = static_cast<int>(k == 40 ? PathMode::Lanebow : PathMode::Lane);
            planner.update_params(steering);
        }
        const LaneState lane = lanebow_lane(k, -1.70f, 1.80f);  // LD는 차로 중심이 5 cm 오른쪽
        const LateralTarget r =
            planner.update(lanebow_model(k, v, -1.75f, 1.75f), &lane, vehicle, v, 0.0f, true, false);
        if (k < 20) continue;  // 처음 0.2초는 유효 확률이 차오르는 동안 모델 경로다
        ASSERT_FALSE(r.laneless_mode) << k;
        curvature.push_back(lag_adjusted_curvature(r, v, 0.0f, 0.42f));
    }
    float max_step = 0.0f;
    for (size_t i = 1; i < curvature.size(); ++i)
        max_step = std::max(max_step, std::fabs(curvature[i] - curvature[i - 1]));
    // 두 출처의 차이(5 cm)만큼만 바뀐다: 계단이 횡가속 0.1 m/s²(20 m/s에서 곡률 2.5e-4) 아래
    EXPECT_LT(max_step * v * v, 0.1f);
}

// Lanebow 차선 변경: 변경하는 동안 LD 자차 선이 밀려 가다 선을 밟고(버림), 넘어가면 이름이 바뀐다(옛 왼쪽 옆 선이
// 새 왼선). 변경 중에는 desire_helper가 차선 확률을 빼고, 끝난 뒤 새 차로의 선으로 튀지 않고 돌아온다.
TEST(LateralPlanner, LanebowLaneChangeSwapsSlotsQuietly) {
    const float v = 20.0f;
    SteeringParams steering;
    steering.path_mode = static_cast<int>(PathMode::Lanebow);
    LateralPlanner planner(steering);
    VehicleCanState vehicle{};
    int k = 0;
    auto step = [&](float offset, float lane_change_prob, bool pushing) {
        ModelState model = lanebow_model(k, v, -1.75f, 1.75f);
        model.desire_state[0] = 1.0f - lane_change_prob;
        model.desire_state[3] = lane_change_prob;
        // 차가 왼쪽으로 offset만큼 갔다: 선은 오른쪽으로 밀린다. 왼선이 차를 넘으면 옆 차로의 선이 새 왼선이다.
        float left = -1.75f + offset, right = 1.75f + offset;
        if (left > 0.0f) {
            right = left;
            left -= 3.5f;
        }
        if (std::fabs(left) < 0.3f || std::fabs(right) < 0.3f) left = right = std::numeric_limits<float>::quiet_NaN();
        const LaneState lane = lanebow_lane(k, left, right);
        ++k;
        return planner.update(model, &lane, vehicle, v, 0.0f, true, pushing);
    };
    LateralTarget r;
    for (int i = 0; i < 60; ++i) r = step(0.0f, 0.0f, false);
    ASSERT_FALSE(r.laneless_mode);
    vehicle.left_blinker = true;
    step(0.0f, 0.0f, false);
    vehicle.driver_torque = 300;
    r = step(0.0f, 0.0f, true);
    ASSERT_EQ(r.desire, 3) << "차선 변경 시작";
    vehicle.driver_torque = 0;
    for (int i = 1; i <= 60; ++i) r = step(3.5f * i / 60.0f, 0.9f, false);
    vehicle.left_blinker = false;
    std::vector<float> after;
    for (int i = 0; i < 60; ++i) {
        r = step(3.5f, 0.0f, false);
        const float curvature = lag_adjusted_curvature(r, v, 0.0f, 0.42f);
        ASSERT_TRUE(std::isfinite(curvature)) << i;
        after.push_back(curvature);
    }
    EXPECT_EQ(r.lane_change_state, 0);
    EXPECT_FALSE(r.laneless_mode) << "새 차로의 LD 선으로 돌아왔다";
    float max_step = 0.0f;
    for (size_t i = 1; i < after.size(); ++i) max_step = std::max(max_step, std::fabs(after[i] - after[i - 1]));
    EXPECT_LT(max_step * v * v, 0.1f) << "끝난 뒤 횡가속 계단 0.1 m/s² 아래";
    EXPECT_NEAR(after.back(), 0.0f, 5e-5f) << "새 차로 가운데";
}

// 정차 부근에서는 plan이 몇 m로 붕괴한다. LD가 있어도 목표는 유한하다.
TEST(LateralPlanner, LanebowStaysFiniteWhenThePlanCollapses) {
    SteeringParams steering;
    steering.path_mode = static_cast<int>(PathMode::Lanebow);
    LateralPlanner planner(steering);
    VehicleCanState vehicle{};
    for (int k = 0; k < 40; ++k) {
        ModelState model = lanebow_model(k, 0.3f, -1.75f, 1.75f);
        for (int i = 0; i < kTrajectorySize; ++i) {
            model.plan[i] = {std::min(0.3f * model_t_idx(i), 2.5f), 0.0f, 0.0f};
            model.lane_t[i] = model_x_idx(i) <= 2.5f ? model.lane_t[i] : std::numeric_limits<float>::quiet_NaN();
        }
        const LaneState lane = lanebow_lane(k, -1.6f, 1.9f);
        const LateralTarget r = planner.update(model, &lane, vehicle, 0.3f, 0.0f, true, false);
        ASSERT_TRUE(r.valid);
        for (int i = 0; i < kLateralControlN; ++i) {
            ASSERT_TRUE(std::isfinite(r.psis[i]) && std::isfinite(r.curvatures[i])) << k << " " << i;
        }
    }
}

}  // namespace
