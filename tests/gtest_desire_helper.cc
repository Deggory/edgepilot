// 차선 변경 상태 머신(DesireHelper, openpilot desire_helper 이식): 깜빡이 다음 운전자가 깜빡이 쪽으로
// 핸들을 밀어야 변경이 시작되고, 차선선을 0.5초에 빼고 되돌린다. 사각지대면 기다리고, 10초가 넘으면 끈다.
// 도로 경계 쪽으로는 시작하지 않다가 경계가 사라지면 시작한다. 깜빡이를 켠 회전은 차선 변경이 아니고, 깜빡이
// 한 번에 한 번만 바꾸며, 반대로 밀면 취소한다. 모델 프레임(20 Hz)마다 부른다.
#include "car/vehicle_can.h"
#include "planning/desire_helper.h"

#include <gtest/gtest.h>

#include <cstdlib>

namespace {

constexpr float kHighwaySpeedMps = 20.0f;

// 차선선 넷이 다 보이고 도로 경계는 흐린(경계 아님) 모델 프레임.
DesireModelInputs open_road(double lane_change_prob = 0.0) {
    DesireModelInputs model;
    model.lane_change_prob = lane_change_prob;
    model.lane_probabilities = {0.8, 0.9, 0.9, 0.8};
    model.road_edge_stds = {1.0, 1.0};
    return model;
}

VehicleCanState with_blinker(int side, int driver_torque = 0) {
    VehicleCanState vehicle;
    vehicle.left_blinker = side < 0;
    vehicle.right_blinker = side > 0;
    vehicle.driver_torque = driver_torque;
    return vehicle;
}

// 컨트롤러가 거른 운전자 조향(steering pressed)은 토크가 기준(150)을 넘는 것으로 대신한다.
void run(DesireHelper *helper, int frames, const VehicleCanState &vehicle, const DesireModelInputs &model,
         float v_ego = kHighwaySpeedMps, bool lat_active = true) {
    for (int i = 0; i < frames; ++i)
        helper->update(vehicle, v_ego, lat_active, std::abs(vehicle.driver_torque) > 150, model);
}

// 왼쪽 차선 변경을 시작해 끝까지 마친다(깜빡이는 켠 채).
void change_left(DesireHelper *helper) {
    run(helper, 1, with_blinker(-1, 300), open_road(0.9));
    run(helper, 10, with_blinker(-1), open_road(0.9));
    run(helper, 1, with_blinker(-1), open_road(0.0));
    run(helper, 10, with_blinker(-1), open_road(0.0));
}

TEST(DesireHelper, LaneChangeStartsWhenTheDriverPushesTowardTheBlinker) {
    DesireHelper helper;
    run(&helper, 1, with_blinker(-1), open_road());
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange);
    ASSERT_EQ(helper.direction(), -1);
    ASSERT_EQ(helper.desire(), Desire::None) << "핸들을 밀기 전에는 모델 desire가 없다";

    run(&helper, 5, with_blinker(-1, -300), open_road());
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange) << "반대로 민 토크는 무시한다";

    run(&helper, 1, with_blinker(-1, 300), open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting);
    ASSERT_EQ(helper.desire(), Desire::LaneChangeLeft);
    ASSERT_TRUE(helper.changing_lanes());

    run(&helper, 10, with_blinker(-1), open_road(0.9));
    ASSERT_NEAR(helper.lane_change_lane_prob(), 0.0, 1e-9) << "차선선을 0.5초에 뺀다";
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting) << "모델이 변경 중이라 하면 머문다";

    run(&helper, 1, with_blinker(0), open_road(0.0));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Finishing);
    run(&helper, 10, with_blinker(0), open_road(0.0));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "0.5초에 되돌리고 깜빡이가 없으면 끝";
    ASSERT_EQ(helper.direction(), 0);
    ASSERT_EQ(helper.desire(), Desire::None);
}

TEST(DesireHelper, BlindSpotHoldsTheStart) {
    DesireHelper helper;
    VehicleCanState vehicle = with_blinker(1);
    run(&helper, 1, vehicle, open_road());
    vehicle.driver_torque = -300;
    vehicle.right_blindspot = true;
    run(&helper, 20, vehicle, open_road());
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange) << "사각지대 차가 있으면 기다린다";
    vehicle.right_blindspot = false;
    run(&helper, 1, vehicle, open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting);
    ASSERT_EQ(helper.desire(), Desire::LaneChangeRight);
}

TEST(DesireHelper, ChangeTurnsOffAfterTenSeconds) {
    DesireHelper helper;
    run(&helper, 1, with_blinker(-1), open_road());
    run(&helper, 1, with_blinker(-1, 300), open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting);
    run(&helper, 190, with_blinker(-1), open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting) << "9.5초에는 아직 기다린다";
    run(&helper, 20, with_blinker(-1), open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "10초가 넘으면 끈다";
    ASSERT_EQ(helper.desire(), Desire::None);
}

TEST(DesireHelper, RoadEdgeBlocksUntilItClears) {
    DesireHelper helper;
    DesireModelInputs edge = open_road();
    edge.road_edge_stds = {1.0, 0.2};                // 오른쪽 경계가 또렷하다
    edge.lane_probabilities = {0.8, 0.9, 0.9, 0.1};  // 그 너머 차선선은 없다
    run(&helper, 10, with_blinker(1), edge);
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "도로 경계 쪽으로는 시작하지 않는다";
    run(&helper, 1, with_blinker(-1), edge);
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange) << "반대쪽은 막지 않는다";

    DesireHelper cleared;
    run(&cleared, 10, with_blinker(1), edge);
    run(&cleared, 1, with_blinker(1), open_road());
    ASSERT_EQ(cleared.lane_change_state(), LaneChangeState::PreLaneChange)
        << "경계가 사라지면 깜빡이를 다시 켜지 않아도 시작한다";
}

// 깜빡이 한 번에 차선 변경은 한 번이다: 끝난 뒤 깜빡이를 켜 둔 채 다시 밀어도 시작하지 않고(램프·회전 출구에서
// 4.5초마다 다시 시작했다), 깜빡이를 껐다 켜면 다시 할 수 있다.
TEST(DesireHelper, OneLaneChangePerBlinker) {
    DesireHelper helper;
    run(&helper, 1, with_blinker(-1), open_road());
    change_left(&helper);
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "깜빡이가 켜져 있어도 기다리지 않는다";
    run(&helper, 10, with_blinker(-1, 300), open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "같은 깜빡이로는 다시 바꾸지 않는다";
    ASSERT_EQ(helper.desire(), Desire::None);
    run(&helper, 1, with_blinker(0), open_road());
    run(&helper, 1, with_blinker(-1), open_road());
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange) << "깜빡이를 다시 켜면 기다린다";
    run(&helper, 1, with_blinker(-1, 300), open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting);

    // 변경 중 깜빡이를 껐다 같은 쪽으로 다시 켜면 한 번 더 바꾸라는 신호라 마친 뒤 기다린다
    DesireHelper again;
    run(&again, 1, with_blinker(-1), open_road());
    run(&again, 1, with_blinker(-1, 300), open_road(0.9));
    run(&again, 5, with_blinker(-1), open_road(0.9));
    run(&again, 2, with_blinker(0), open_road(0.9));
    run(&again, 5, with_blinker(-1), open_road(0.9));
    run(&again, 1, with_blinker(-1), open_road(0.0));
    run(&again, 10, with_blinker(-1), open_road(0.0));
    ASSERT_EQ(again.lane_change_state(), LaneChangeState::PreLaneChange);
    run(&again, 1, with_blinker(-1, 300), open_road(0.9));
    ASSERT_EQ(again.lane_change_state(), LaneChangeState::Starting);

    // 끝나 가는 동안 반대쪽 깜빡이로 바꾸면(꺼진 틈 없이) 그쪽 변경은 새 신호라 기다린다
    DesireHelper switching;
    run(&switching, 1, with_blinker(-1), open_road());
    run(&switching, 1, with_blinker(-1, 300), open_road(0.9));
    run(&switching, 10, with_blinker(-1), open_road(0.9));
    run(&switching, 1, with_blinker(1), open_road(0.0));
    run(&switching, 10, with_blinker(1), open_road(0.0));
    ASSERT_EQ(switching.lane_change_state(), LaneChangeState::PreLaneChange);
    ASSERT_EQ(switching.direction(), 1);
    run(&switching, 1, with_blinker(1, -300), open_road(0.9));
    ASSERT_EQ(switching.lane_change_state(), LaneChangeState::Starting);
    ASSERT_EQ(switching.desire(), Desire::LaneChangeRight);
}

// 깜빡이를 켜고 교차로를 돌면(핸들 45도 이상) 차선 변경이 아니라 회전이다: 대기 중이면 끄고, 변경 중이면
// 그만두며, 모델에는 회전 desire를 주고, 그 깜빡이로는 차선 변경을 다시 시작하지 않는다. 회전하려고 감속하는
// 동안은 회전 쪽으로 밀어도 차선 변경을 시작하지 않는다.
TEST(DesireHelper, TurnWithTheBlinkerIsNotALaneChange) {
    constexpr float kCitySpeedMps = 45.0f / 3.6f;
    DesireHelper turning;
    run(&turning, 1, with_blinker(-1), open_road(), kCitySpeedMps);
    VehicleCanState wheel = with_blinker(-1, 300);
    wheel.steering_angle_deg = 60.0f;
    run(&turning, 1, wheel, open_road(0.9), kCitySpeedMps);
    ASSERT_EQ(turning.lane_change_state(), LaneChangeState::Off) << "핸들을 크게 돌리면 회전이다";
    ASSERT_EQ(turning.desire(), Desire::TurnLeft);
    run(&turning, 20, with_blinker(-1, 300), open_road(0.9), kCitySpeedMps);
    ASSERT_EQ(turning.lane_change_state(), LaneChangeState::Off) << "회전 뒤 켜 둔 깜빡이로는 시작하지 않는다";
    ASSERT_EQ(turning.desire(), Desire::None) << "핸들이 돌아오면 회전 desire를 거둔다";

    DesireHelper changing;
    run(&changing, 1, with_blinker(-1), open_road(), kCitySpeedMps);
    run(&changing, 1, with_blinker(-1, 300), open_road(0.9), kCitySpeedMps);
    ASSERT_EQ(changing.lane_change_state(), LaneChangeState::Starting);
    run(&changing, 1, wheel, open_road(0.9), kCitySpeedMps);
    ASSERT_EQ(changing.lane_change_state(), LaneChangeState::Off) << "변경 중 크게 돌리면 그만둔다";

    // 1 m/s²로 줄이는 중(1초 넘게)에 회전 쪽으로 밀면 기다리고, 감속을 멈추면 시작한다
    DesireHelper slowing;
    float v = kCitySpeedMps;
    run(&slowing, 25, with_blinker(-1), open_road(), v);
    for (int i = 0; i < 20; ++i) {
        v -= 0.05f;
        run(&slowing, 1, with_blinker(-1), open_road(), v);
    }
    for (int i = 0; i < 20; ++i) {
        v -= 0.05f;
        run(&slowing, 1, with_blinker(-1, 300), open_road(0.9), v);
        ASSERT_EQ(slowing.lane_change_state(), LaneChangeState::PreLaneChange) << i;
    }
    run(&slowing, 25, with_blinker(-1), open_road(0.9), v);
    run(&slowing, 1, with_blinker(-1, 300), open_road(0.9), v);
    ASSERT_EQ(slowing.lane_change_state(), LaneChangeState::Starting) << "일정한 속도면 시작한다";
}

// 변경 중 운전자가 반대쪽으로 0.2초 밀면 취소하고, 그 깜빡이로는 다시 시작하지 않는다. 잠깐 민 것은 무시한다.
TEST(DesireHelper, PushingBackCancelsTheChange) {
    DesireHelper helper;
    run(&helper, 1, with_blinker(-1), open_road());
    run(&helper, 1, with_blinker(-1, 300), open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting);
    run(&helper, 3, with_blinker(-1, -300), open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Starting) << "0.15초는 무시한다";
    run(&helper, 1, with_blinker(-1), open_road(0.9));
    run(&helper, 4, with_blinker(-1, -300), open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "반대로 0.2초 밀면 취소";
    ASSERT_EQ(helper.desire(), Desire::None);
    run(&helper, 10, with_blinker(-1, 300), open_road(0.9));
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "같은 깜빡이로는 다시 시작하지 않는다";
}

// 상류처럼 조향하지 않으면(latActive 거짓: 정차, 큰 조향각 정지) 끄고, 조향이 돌아오면 켜 둔 깜빡이를 새로 켠
// 것으로 본다. 기다리는 동안 깜빡이를 반대로 바꾸면 방향도 바뀐다.
TEST(DesireHelper, FollowsLateralActiveAndTheBlinkerSide) {
    DesireHelper helper;
    run(&helper, 1, with_blinker(-1), open_road());
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange);
    run(&helper, 1, with_blinker(1), open_road());
    ASSERT_EQ(helper.direction(), 1) << "기다리는 동안 방향을 다시 읽는다";
    run(&helper, 5, with_blinker(1), open_road(), kHighwaySpeedMps, false);
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::Off) << "조향하지 않으면 끈다";
    run(&helper, 1, with_blinker(1), open_road());
    ASSERT_EQ(helper.lane_change_state(), LaneChangeState::PreLaneChange) << "조향이 돌아오면 켜 둔 깜빡이로 기다린다";
}

// 회전 desire는 깜빡이 쪽으로 45도 넘게 돌릴 때 주고, 핸들이 15도 아래로 돌아올 때까지 이어진다(도중에 깜빡이를
// 꺼도). 모델 입력은 바뀐 틱의 펄스라 2.5초마다 한 틱 내렸다 다시 준다. 깜빡이 반대쪽으로 돈 것은 회전 desire가
// 아니다(차선 변경만 막는다).
TEST(DesireHelper, TurnDesireFollowsTheWheel) {
    DesireHelper helper;
    VehicleCanState vehicle = with_blinker(1);
    run(&helper, 1, vehicle, open_road(), 5.0f);
    vehicle.steering_angle_deg = -30.0f;
    run(&helper, 5, vehicle, open_road(), 5.0f);
    ASSERT_FALSE(helper.turning()) << "45도 아래는 아직 회전이 아니다";
    vehicle.steering_angle_deg = -120.0f;
    run(&helper, 1, vehicle, open_road(), 5.0f);
    ASSERT_TRUE(helper.turning());
    ASSERT_EQ(helper.turn_direction(), 1);
    ASSERT_EQ(helper.desire(), Desire::TurnRight);
    int dropped = 0;
    for (int i = 0; i < 99; ++i) {
        run(&helper, 1, vehicle, open_road(), 5.0f);
        if (helper.desire() == Desire::None)
            ++dropped;
        else
            ASSERT_EQ(helper.desire(), Desire::TurnRight) << i;
    }
    ASSERT_EQ(dropped, 2) << "5초 동안 2.5초마다 한 틱 내렸다 다시 준다";
    vehicle.right_blinker = false;
    vehicle.steering_angle_deg = -40.0f;
    run(&helper, 5, vehicle, open_road(), 5.0f);
    ASSERT_TRUE(helper.turning()) << "깜빡이가 꺼져도 회전을 마칠 때까지";
    vehicle.steering_angle_deg = -10.0f;
    run(&helper, 1, vehicle, open_road(), 5.0f);
    ASSERT_FALSE(helper.turning());
    ASSERT_EQ(helper.desire(), Desire::None);

    DesireHelper opposite;
    VehicleCanState wrong = with_blinker(1);
    wrong.steering_angle_deg = 90.0f;  // 오른쪽 깜빡이, 왼쪽으로 돈다
    run(&opposite, 3, wrong, open_road(), 5.0f);
    ASSERT_FALSE(opposite.turning());
    ASSERT_EQ(opposite.desire(), Desire::None);
}

}  // namespace
