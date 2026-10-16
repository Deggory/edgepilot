#ifndef LANE_LINE_FILTER_H
#define LANE_LINE_FILTER_H

// 차선선 시간 필터. 모델 차선은 프레임마다 흔들리고(2026-10-10 밤 주행: 27 m에서 1.3 cm, 75 m에서 5.5 cm),
// 그 흔들림이 조향 지령 잔떨림의 대부분이다. 직전 프레임에 거른 선을 그 사이 자차가 움직인 만큼(속도와
// 실측 곡률의 원호) 옮겨 지금 차체 좌표로 예측하고, 새 측정과 섞는다. 도로에 고정된 선은 차가 움직여도
// 예측이 맞으므로 차의 움직임에는 늦지 않고, 모델이 제 추정을 고치는 것만 천천히 따라간다(같은 거리의
// 점끼리 섞으면 차로 유지 피드백이 늦어 폐루프 재생에서 차선 중앙 오차가 두 배였다). 가까운 점이 예측에서
// 크게 벗어나면(차로 변경으로 선의 이름이 바뀜, 선이 다시 잡힘) 측정에서 다시 시작한다. 하드웨어와 무관하다.
// docs/lane-line-filter.md

#include "common/model_output.h"  // kTrajectorySize

#include <array>

// 새 측정의 가중. 재생 비교(docs/lane-line-filter.md)에서 조향 지령 잔떨림을 45% 줄이고 토크 레이트 한계에
// 걸리는 시간을 절반으로 줄이면서, 폐루프 차선 중앙 오차는 0.2 cm 안으로 늘었다. 0.33은 잔떨림을 60% 줄이고
// 오차가 0.4 cm 늘었다.
constexpr double kLaneLineFilterWeight = 0.5;

class LaneLineFilter {
public:
    // 가중 1이면 거르지 않는다(재생 도구의 --lane-filter)
    void set_measurement_weight(double weight) { weight_ = weight; }

    // x는 점의 거리(앞이 +, 증가), y는 측정 횡위치(오른쪽이 +), t_s는 촬영 시각, v_mps는 자차 속도,
    // curvature는 실측 곡률(오른쪽이 +). 거른 횡위치를 out에 쓴다(y와 같은 배열이어도 된다).
    void update(double t_s, const float *x, const float *y, double v_mps, double curvature, float *out);
    // update와 같지만 앞쪽 count개 점만 쓴다(LD처럼 시야가 짧은 선). out의 나머지는 NaN이다. count개 안에
    // 유한하지 않은 값이 있으면 update처럼 측정을 그대로 내고 다시 시작한다.
    void update_prefix(double t_s, const float *x, const float *y, int count, double v_mps, double curvature,
                       float *out);
    // 직전에 거른 선을 t_s까지 자차 움직임(속도와 곡률의 원호)만큼 옮겨 x[0..count)에서 읽는다. 옮기며 짧아진 앞
    // 끝(간 거리만큼)은 마지막 구간을 곧게 늘리고, 그보다 먼 점은 NaN이다. 거른 선이 없거나 t_s가 그 선보다 앞서면 false.
    bool predict(double t_s, const float *x, int count, double v_mps, double curvature, float *out) const;
    bool has_line() const { return have_; }
    double time_s() const { return t_s_; }

private:
    double weight_ = kLaneLineFilterWeight;
    bool have_ = false;
    double t_s_ = 0.0;
    int count_ = 0;                                  // 거른 선의 점 개수
    std::array<double, kTrajectorySize> x_{}, y_{};  // 직전 프레임 차체 좌표의 거른 선
};

#endif
