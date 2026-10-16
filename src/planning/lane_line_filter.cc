#include "planning/lane_line_filter.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kMaxGapS = 0.2;         // 프레임 사이가 이보다 길면 다시 시작한다
constexpr double kRestartReachM = 20.0;  // 다시 시작을 판단하는 가까운 점의 범위
constexpr double kRestartM = 0.5;        // 그 점들이 예측에서 평균 이만큼 벗어나면 다시 시작한다

using Line = std::array<double, kTrajectorySize>;

// px[j] <= x <= px[j + 1]인 x에서 선(앞쪽 n개 점)을 읽는다. x에 더 가까운 쪽 이웃을 더한 세 점의 2차(라그랑주)
// 보간이다. 직선 보간은 굽은 선의 현을 읽어 프레임마다 선을 커브 안쪽으로 당기고(점 간격 6 m, 곡률 0.002에서
// 수 mm), 필터가 그것을 쌓는다. 점이 겹치면 직선 보간이다.
double interpolate(const Line &px, const Line &py, int n, int j, double x) {
    const double span = px[j + 1] - px[j];
    if (!(span > 1e-6)) return py[j + 1];
    const bool left = j > 0 && (x - px[j] < px[j + 1] - x || j + 2 >= n);
    const int a = left ? j - 1 : j;
    if (a + 2 >= n || !(px[a + 1] - px[a] > 1e-6) || !(px[a + 2] - px[a + 1] > 1e-6))
        return py[j] + (x - px[j]) / span * (py[j + 1] - py[j]);
    const double x0 = px[a], x1 = px[a + 1], x2 = px[a + 2];
    return py[a] * (x - x1) * (x - x2) / ((x0 - x1) * (x0 - x2)) +
           py[a + 1] * (x - x0) * (x - x2) / ((x1 - x0) * (x1 - x2)) +
           py[a + 2] * (x - x0) * (x - x1) / ((x2 - x0) * (x2 - x1));
}

// 직전 차체 좌표의 선(앞쪽 n개 점)을 지금 차체 좌표로 옮긴다. 자차는 거리 d = v·dt를 곡률 k의 원호로 갔다
// (방향 theta = k·d).
void move_by_ego_arc(const Line &x, const Line &y, int n, double dt, double v_mps, double curvature, Line *px,
                     Line *py) {
    const double d = std::max(0.0, v_mps) * dt;
    const double theta = curvature * d;
    const bool straight = std::fabs(theta) < 1e-9;
    const double dx = straight ? d : d * std::sin(theta) / theta;
    const double dy = straight ? 0.5 * d * theta : d * (1.0 - std::cos(theta)) / theta;
    const double c = std::cos(theta), s = std::sin(theta);
    for (int j = 0; j < n; ++j) {
        const double qx = x[j] - dx, qy = y[j] - dy;
        (*px)[j] = c * qx + s * qy;
        (*py)[j] = -s * qx + c * qy;
        if (j > 0) (*px)[j] = std::max((*px)[j], (*px)[j - 1]);
    }
}

// 옮긴 선(px, py의 앞쪽 n개 점)을 x에서 읽는다. j는 x가 커지는 순서로 읽을 때 이어 쓰는 구간 위치다. 선이 닿지
// 않는 먼 점은 beyond, 가장 가까운 점보다 가까운 점은 그 점의 값이다.
double read_moved(const Line &px, const Line &py, int n, int *j, double x, double beyond) {
    if (x > px[n - 1]) return beyond;
    if (x <= px[0]) return py[0];
    while (*j + 1 < n - 1 && px[*j + 1] < x) ++*j;
    return interpolate(px, py, n, *j, x);
}

}  // namespace

void LaneLineFilter::update(double t_s, const float *x, const float *y, double v_mps, double curvature, float *out) {
    update_prefix(t_s, x, y, kTrajectorySize, v_mps, curvature, out);
}

void LaneLineFilter::update_prefix(double t_s, const float *x, const float *y, int count, double v_mps,
                                   double curvature, float *out) {
    count = std::clamp(count, 0, kTrajectorySize);
    for (int i = count; i < kTrajectorySize; ++i) out[i] = std::numeric_limits<float>::quiet_NaN();
    for (int i = 0; i < count; ++i) {
        if (!std::isfinite(x[i]) || !std::isfinite(y[i])) {
            std::copy(y, y + count, out);
            have_ = false;
            return;
        }
    }
    if (count == 0) {
        have_ = false;
        return;
    }
    const double dt = t_s - t_s_;
    bool restart = !have_ || !(dt > 0.0) || dt > kMaxGapS || weight_ >= 1.0;
    Line predicted{};
    if (!restart) {
        Line px{}, py{};
        move_by_ego_arc(x_, y_, count_, dt, v_mps, curvature, &px, &py);
        // 측정 점의 거리에서 예측을 읽는다. 예측이 닿지 않는 먼 끝은 측정 그대로다.
        int j = 0;
        double error = 0.0;
        int near = 0;
        for (int i = 0; i < count; ++i) {
            predicted[i] = read_moved(px, py, count_, &j, x[i], y[i]);
            if (x[i] <= kRestartReachM) {
                error += std::fabs(y[i] - predicted[i]);
                ++near;
            }
        }
        restart = near > 0 && error / near > kRestartM;
    }
    for (int i = 0; i < count; ++i) {
        const double value = restart ? y[i] : predicted[i] + weight_ * (y[i] - predicted[i]);
        x_[i] = x[i];
        y_[i] = value;
        out[i] = static_cast<float>(value);
    }
    count_ = count;
    t_s_ = t_s;
    have_ = true;
}

bool LaneLineFilter::predict(double t_s, const float *x, int count, double v_mps, double curvature, float *out) const {
    const double dt = t_s - t_s_;
    if (!have_ || count_ < 1 || !(dt >= 0.0)) return false;
    Line px{}, py{};
    move_by_ego_arc(x_, y_, count_, dt, v_mps, curvature, &px, &py);
    // 옮긴 선은 자차가 간 거리만큼 앞이 짧아진다. 그만큼은 마지막 구간을 곧게 늘려 측정이 닿은 거리를 그대로 둔다
    // (늘리지 않으면 LD가 같은 프레임인지 한 프레임 전인지에 따라 시야 끝이 한 격자씩 오갔다).
    const int n = count_;
    const double travel = std::max(0.0, v_mps) * dt;
    const double span = n >= 2 ? px[n - 1] - px[n - 2] : 0.0;
    const double slope = span > 1e-6 ? (py[n - 1] - py[n - 2]) / span : 0.0;
    int j = 0;
    for (int i = 0; i < std::clamp(count, 0, kTrajectorySize); ++i) {
        if (x[i] > px[n - 1]) {
            out[i] = x[i] <= px[n - 1] + travel + 1e-6 ? static_cast<float>(py[n - 1] + slope * (x[i] - px[n - 1]))
                                                       : std::numeric_limits<float>::quiet_NaN();
            continue;
        }
        out[i] = static_cast<float>(read_moved(px, py, n, &j, x[i], std::numeric_limits<double>::quiet_NaN()));
    }
    return true;
}
