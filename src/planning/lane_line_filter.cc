#include "planning/lane_line_filter.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kMaxGapS = 0.2;         // 프레임 사이가 이보다 길면 다시 시작한다
constexpr double kRestartReachM = 20.0;  // 다시 시작을 판단하는 가까운 점의 범위
constexpr double kRestartM = 0.5;        // 그 점들이 예측에서 평균 이만큼 벗어나면 다시 시작한다

using Line = std::array<double, kTrajectorySize>;

// px[j] <= x <= px[j + 1]인 x에서 선을 읽는다. x에 더 가까운 쪽 이웃을 더한 세 점의 2차(라그랑주) 보간이다.
// 직선 보간은 굽은 선의 현을 읽어 프레임마다 선을 커브 안쪽으로 당기고(점 간격 6 m, 곡률 0.002에서 수 mm),
// 필터가 그것을 쌓는다. 점이 겹치면 직선 보간이다.
double interpolate(const Line &px, const Line &py, int j, double x) {
    const double span = px[j + 1] - px[j];
    if (!(span > 1e-6)) return py[j + 1];
    const bool left = j > 0 && (x - px[j] < px[j + 1] - x || j + 2 >= kTrajectorySize);
    const int a = left ? j - 1 : j;
    if (a + 2 >= kTrajectorySize || !(px[a + 1] - px[a] > 1e-6) || !(px[a + 2] - px[a + 1] > 1e-6))
        return py[j] + (x - px[j]) / span * (py[j + 1] - py[j]);
    const double x0 = px[a], x1 = px[a + 1], x2 = px[a + 2];
    return py[a] * (x - x1) * (x - x2) / ((x0 - x1) * (x0 - x2)) +
           py[a + 1] * (x - x0) * (x - x2) / ((x1 - x0) * (x1 - x2)) +
           py[a + 2] * (x - x0) * (x - x1) / ((x2 - x0) * (x2 - x1));
}

}  // namespace

void LaneLineFilter::update(double t_s, const float *x, const float *y, double speed_mps, double curvature,
                            float *out) {
    const double dt = t_s - t_s_;
    bool restart = !have_ || !(dt > 0.0) || dt > kMaxGapS || weight_ >= 1.0;
    Line predicted{};
    if (!restart) {
        // 직전 선을 지금 차체 좌표로 옮긴다. 자차는 거리 d를 곡률 k의 원호로 갔다(방향 theta = k·d).
        const double d = std::max(0.0, speed_mps) * dt;
        const double theta = curvature * d;
        const bool straight = std::fabs(theta) < 1e-9;
        const double dx = straight ? d : d * std::sin(theta) / theta;
        const double dy = straight ? 0.5 * d * theta : d * (1.0 - std::cos(theta)) / theta;
        const double c = std::cos(theta), s = std::sin(theta);
        Line px{}, py{};
        for (int j = 0; j < kTrajectorySize; ++j) {
            const double qx = x_[j] - dx, qy = y_[j] - dy;
            px[j] = c * qx + s * qy;
            py[j] = -s * qx + c * qy;
            if (j > 0) px[j] = std::max(px[j], px[j - 1]);
        }
        // 측정 점의 거리에서 예측을 읽는다. 예측이 닿지 않는 먼 끝은 측정 그대로다.
        int j = 0;
        double error = 0.0;
        int near = 0;
        for (int i = 0; i < kTrajectorySize; ++i) {
            if (x[i] > px[kTrajectorySize - 1]) {
                predicted[i] = y[i];
            } else if (x[i] <= px[0]) {
                predicted[i] = py[0];
            } else {
                while (j + 1 < kTrajectorySize - 1 && px[j + 1] < x[i]) ++j;
                predicted[i] = interpolate(px, py, j, x[i]);
            }
            if (x[i] <= kRestartReachM) {
                error += std::fabs(y[i] - predicted[i]);
                ++near;
            }
        }
        restart = near > 0 && error / near > kRestartM;
    }
    for (int i = 0; i < kTrajectorySize; ++i) {
        const double value = restart ? y[i] : predicted[i] + weight_ * (y[i] - predicted[i]);
        x_[i] = x[i];
        y_[i] = value;
        out[i] = static_cast<float>(value);
    }
    t_s_ = t_s;
    have_ = true;
}
