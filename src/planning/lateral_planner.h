#pragma once

#include "controls/lateral_target.h"

#include <memory>

struct ModelState;
struct SteeringParams;
struct VehicleCanState;

class LateralPlanner {
public:
    explicit LateralPlanner(const SteeringParams &params);
    ~LateralPlanner();

    LateralPlanner(const LateralPlanner &) = delete;
    LateralPlanner &operator=(const LateralPlanner &) = delete;

    void update_params(const SteeringParams &params);
    // 차선선 시간 필터의 새 측정 가중(planning/lane_line_filter). 재생 도구가 비교할 때 바꾼다. 1이면 끈다.
    void set_lane_filter_weight(double weight);

    // lat_active: 지금 조향하는가(상류 latActive), steering_pressed: 컨트롤러가 거른 운전자 조향. 둘 다 차선 변경
    // 판단(DesireHelper)에 쓴다.
    LateralTarget update(const ModelState &model, const VehicleCanState &vehicle, float v_ego, float measured_curvature,
                         bool lat_active, bool steering_pressed);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
