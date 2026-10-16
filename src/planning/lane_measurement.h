#ifndef PLANNING_LANE_MEASUREMENT_H
#define PLANNING_LANE_MEASUREMENT_H

// 한 출처(supercombo 차선 또는 차선선 플러그인 LD)의 이번 프레임 자차 좌우 차선. 횡 플래너의 차선 코어
// (lateral_planner.cc LanePlanner)가 이것으로 유효 확률·차로 폭·차선 중심 경로를 만든다. 선은 출처가 이미 시간
// 필터에 거른 값이고 모든 점이 유한하다.

#include "common/model_output.h"  // kTrajectorySize

#include <array>

struct LaneMeasurement {
    std::array<double, kTrajectorySize> t{};  // plan이 각 점의 거리에 닿는 시각(ModelState.lane_t). NaN이면 plan 밖
    std::array<double, kTrajectorySize> x{};  // 점의 거리(X_IDXS)
    std::array<double, kTrajectorySize> left{}, right{};  // 횡위치(오른쪽 +)
    double left_prob = 0.0, right_prob = 0.0;             // 보정 전 확률
    double left_std = 0.0, right_std = 0.0;
    int width_index = 0;  // 차로 폭을 배울 점(supercombo는 0, LD는 측정이 덮은 가까운 점)
};

#endif
