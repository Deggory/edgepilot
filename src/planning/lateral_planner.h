#pragma once

// 횡방향 계획. 모델 출력의 차선 확률·폭(상류 lateral_planner)과 횡방향 MPC로 목표 곡률(LateralTarget)을 만든다.
// Laneless 모드와 Lane 모드의 모델 경로 구간은 plan의 방향각을 쓰고, 차선 변경은 desire_helper가 정한다.
// LateralTarget을 만드는 곳은 이것뿐이다.

#include "controls/lateral_target.h"

#include <memory>

struct ModelState;
struct SteeringParams;
struct VehicleCanState;

// 플래너가 컨트롤러에서 받는 값(직전 틱). 상류 lateral_planner·desire_helper가 carState·carControl에서 읽는 것들이다.
struct LateralPlannerInput {
    float v_ego = 0.0f;               // 거른 vEgo(m/s)
    float measured_curvature = 0.0f;  // 실제 곡률(1/m)
    bool lat_active = false;          // 지금 조향하는가(상류 latActive). 차선 변경 판단(DesireHelper)에 쓴다
    bool steering_pressed = false;    // 컨트롤러가 거른 운전자 조향. 차선 변경 판단에 쓴다
};

class LateralPlanner {
public:
    explicit LateralPlanner(const SteeringParams &params);
    ~LateralPlanner();

    LateralPlanner(const LateralPlanner &) = delete;
    LateralPlanner &operator=(const LateralPlanner &) = delete;

    void update_params(const SteeringParams &params);
    // 차선선 시간 필터의 새 측정 가중(planning/lane_line_filter). 재생 도구가 비교할 때 바꾼다. 1이면 끈다.
    void set_lane_filter_weight(double weight);

    LateralTarget update(const ModelState &model, const VehicleCanState &vehicle, const LateralPlannerInput &in);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
