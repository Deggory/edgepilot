#pragma once

// LateralController의 틱 입력 둘: LateralPlanner(횡방향 MPC)가 만드는 횡방향 계획(LateralTarget, 생산자는 controlsd의
// MPC 하나뿐이다)과 모델 plan의 조향 가용성(LateralPath).

#include <cstdint>

constexpr int kLateralControlN = 17;

// 차선 변경 단계(상류 LaneChangeState, planning/desire_helper가 정한다). 정수 값은 ControlState·HUD 플래그에 그대로
// 쓰인다.
enum class LaneChangeState : int {
    Off = 0,
    PreLaneChange = 1,  // 깜빡이를 켰고 운전자가 핸들을 밀기를 기다린다
    Starting = 2,       // 변경 중: 차선선을 빼며 모델 desire를 준다
    Finishing = 3,      // 끝나 가는 중: 차선선을 되돌린다
};

// 모델 desire 입력(0.9.4·master DESIRES).
enum class Desire : int {
    None = 0,
    TurnLeft = 1,
    TurnRight = 2,
    LaneChangeLeft = 3,
    LaneChangeRight = 4,
};

// 모델 plan을 조향 가용성 gate로만 환산한 것(control_holds.h path_from_model_state). 목표 곡률은 MPC(LateralTarget)가
// 내므로 경로 기하는 없다. 쓸 수 없으면 모델이 낡았거나(model_lost) 경로가 짧다.
struct LateralPath {
    int point_count = 0;
    float reach_m = 0.0f;
    bool usable_for_steering = false;
    // 모델 상태가 낡았다(경로 기하와 별개로 입력이 끊긴 것이다). 시작할 때의 빈 상태(시각 0)도 여기에 든다.
    bool model_lost = false;
};

struct LateralTarget {
    bool valid = false;
    // 이 계획의 근거가 된 카메라 프레임 캡처 시각(monotonic_now_ns 기준).
    // 컨트롤러가 lag 보상과 staleness gate에 쓴다. 0이면 미상(테스트 등)으로
    // 취급해 나이 보정을 건너뛴다.
    uint64_t capture_timestamp_ns = 0;
    bool mpc_solution_valid = false;
    bool laneless_mode = false;
    float target_y_m = 0.0f;
    Desire desire = Desire::None;
    // 차선 변경 단계와 방향(-1 왼쪽, 1 오른쪽). HUD 표시용.
    LaneChangeState lane_change_state = LaneChangeState::Off;
    int lane_change_direction = 0;
    // 경로 노드의 방향각(rad)과 곡률(1/m). [0]이 지금이다.
    float psis[kLateralControlN] = {};
    float curvatures[kLateralControlN] = {};
    // 차선 관측값(로그 전용). 모델 좌표계(+y=오른쪽), 오프셋 미적용.
    float lane_left_y_m = 0.0f;
    float lane_right_y_m = 0.0f;
    float lane_width_m = 0.0f;
    float lane_left_prob = 0.0f;
    float lane_right_prob = 0.0f;
    float lane_left_std = 0.0f;
    float lane_right_std = 0.0f;
    float lane_d_prob = 0.0f;
};
