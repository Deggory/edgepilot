#include "planning/lanebow_source.h"

#include "common/utils_math.h"

#include <algorithm>
#include <cmath>

namespace {

double ramp(double value, double from, double to) { return std::clamp((value - from) / (to - from), 0.0, 1.0); }

// 앞에서부터 이어진 유한한 점의 개수
int finite_prefix(const float *y, int count) {
    int n = 0;
    while (n < count && std::isfinite(y[n])) ++n;
    return n;
}

// LD 시야 너머를 이을 모양(격자 X_IDXS 위의 횡위치). 두 선이 같이 쓴다: supercombo 두 선을 확률 비율로 섞은 모양을 더
// 확실한 쪽의 확률만큼, 나머지는 plan(앞 거리 x에서의 횡위치). 선마다 오프셋을 따로 맞추므로 모양만 쓰인다.
std::array<double, kTrajectorySize> extension_shape(const ModelState &model) {
    std::array<double, kTrajectorySize> plan_x{}, plan_y{};
    for (int i = 0; i < kTrajectorySize; ++i) {
        plan_x[i] = model.plan[i].x;
        plan_y[i] = model.plan[i].y;
    }
    make_monotonic(plan_x.data(), plan_x.size());
    const double left = std::clamp(static_cast<double>(model.lane_probabilities[1]), 0.0, 1.0);
    const double right = std::clamp(static_cast<double>(model.lane_probabilities[2]), 0.0, 1.0);
    const double lines = std::max(left, right);
    const double w_left = left + right > 0.0 ? lines * left / (left + right) : 0.0;
    const double w_right = left + right > 0.0 ? lines * right / (left + right) : 0.0;
    std::array<double, kTrajectorySize> shape{};
    for (int i = 0; i < kTrajectorySize; ++i)
        shape[i] = w_left * model.lanes[1][i].y + w_right * model.lanes[2][i].y +
                   (1.0 - lines) * interp(model_x_idx_double(i), plan_x.data(), plan_y.data(), plan_x.size());
    return shape;
}

}  // namespace

void LanebowSource::set_filter_weight(double weight) {
    for (Side &side : side_) side.filter.set_measurement_weight(weight);
}

void LanebowSource::take_frame(const LaneState &lane, double v_ego, double curvature) {
    const double t_s = static_cast<double>(lane.capture_timestamp_ns) * 1e-9;
    const LaneRoadLine *lines[2] = {lane_ego_road_line(lane, kLaneSlotEgoLeft),
                                    lane_ego_road_line(lane, kLaneSlotEgoRight)};
    int count[2] = {0, 0};
    bool ok[2];
    for (int s = 0; s < 2; ++s) {
        if (lines[s]) count[s] = finite_prefix(lines[s]->y, kLaneRoadPoints);
        // 형상 검사: 왼선은 차의 왼쪽, 오른선은 오른쪽(x = 0)이고 kMaxEgoOffsetM 안. 둘 다 있으면 폭 kMinWidthM 이상,
        // kCrossCheckM까지 엇갈리지 않아야 한다. 선 밟기(차선 변경 중), 슬롯 뒤바뀜, 옆 차로 선, 정지선·횡단보도 끝
        // 오인을 버린다.
        const float y0 = count[s] > 0 ? lines[s]->y[0] : 0.0f;
        ok[s] = count[s] > 0 && (s == 0 ? y0 < 0.0f : y0 > 0.0f) && std::fabs(y0) <= kMaxEgoOffsetM;
    }
    if (ok[0] && ok[1]) {
        bool good = lines[1]->y[0] - lines[0]->y[0] >= kMinWidthM;
        for (int i = 0; good && i < std::min(count[0], count[1]) && model_x_idx(i) <= kCrossCheckM; ++i)
            good = lines[0]->y[i] < lines[1]->y[i];
        if (!good) ok[0] = ok[1] = false;
    }
    float x[kLaneRoadPoints];
    for (unsigned i = 0; i < kLaneRoadPoints; ++i) x[i] = model_x_idx(static_cast<int>(i));
    for (int s = 0; s < 2; ++s) {
        if (!ok[s]) continue;  // 받지 못한 선: 확률은 kHoldS 동안 그대로, 선은 예측
        Side &side = side_[s];
        float filtered[kTrajectorySize];
        side.filter.update_prefix(t_s, x, lines[s]->y, count[s], v_ego, curvature, filtered);
        side.seen_t_s = t_s;
        side.seen_frame = lane.frame_id;
        side.seen_prob = ramp(lines[s]->confidence, kConfidence0, kConfidence1) *
                         ramp(model_x_idx(count[s] - 1), kReach0M, kReach1M);
    }
}

bool LanebowSource::line_at(const Side &side, double t_s, double v_ego, double curvature,
                            const std::array<double, kTrajectorySize> &shape,
                            std::array<double, kTrajectorySize> *out) const {
    if (!side.filter.has_line() || t_s - side.filter.time_s() > kPredictS) return false;
    float x[kLaneRoadPoints], y[kLaneRoadPoints];
    for (unsigned i = 0; i < kLaneRoadPoints; ++i) x[i] = model_x_idx(static_cast<int>(i));
    if (!side.filter.predict(t_s, x, kLaneRoadPoints, v_ego, curvature, y)) return false;
    const int n = finite_prefix(y, kLaneRoadPoints);
    if (n < 2) return false;

    // 시야 끝 kBlendM 안의 평균 차이로 공통 모양에 맞추고, 그 안에서 LD에서 이은 모양으로 서서히 바꾼다
    const double reach = model_x_idx(n - 1);
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < n; ++i) {
        if (model_x_idx(i) < reach - kBlendM) continue;
        sum += y[i] - shape[i];
        ++count;
    }
    const double offset = sum / count;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const double joined = shape[i] + offset;
        if (i >= n) {
            (*out)[i] = joined;
        } else if (reach > kBlendM && model_x_idx(i) > reach - kBlendM) {
            const double w = (model_x_idx(i) - (reach - kBlendM)) / kBlendM;
            (*out)[i] = (1.0 - w) * y[i] + w * joined;
        } else {
            (*out)[i] = y[i];
        }
    }
    return true;
}

LaneMeasurement LanebowSource::measure(const LaneState *lane, const ModelState &model, double v_ego, double curvature) {
    const double t_s = static_cast<double>(model.capture_timestamp_ns) * 1e-9;
    // 선을 붙잡는 시간은 LD 시각으로 잰다(LD는 보통 한 프레임 늦다). LD가 오지 않으면 모델 시각으로 잰다.
    double hold_t = t_s;
    if (lane && lane->valid) {
        if (!have_frame_ || lane->frame_id != taken_frame_) take_frame(*lane, v_ego, curvature);
        taken_frame_ = lane->frame_id;
        have_frame_ = true;
        hold_t = static_cast<double>(lane->capture_timestamp_ns) * 1e-9;
    }
    LaneMeasurement m;
    for (int i = 0; i < kTrajectorySize; ++i) {
        m.t[i] = model.lane_t[i];
        m.x[i] = model_x_idx(i);
    }
    m.width_index = kWidthIndex;
    const std::array<double, kTrajectorySize> shape = extension_shape(model);
    bool have[2];
    lane_frame_id_ = 0;
    lane_age_s_ = -1.0;
    for (int s = 0; s < 2; ++s) {
        const Side &side = side_[s];
        double prob = hold_t - side.seen_t_s <= kHoldS + 1e-6 ? side.seen_prob : 0.0;
        have[s] = line_at(side, t_s, v_ego, curvature, shape, s == 0 ? &m.left : &m.right);
        if (!have[s]) prob = 0.0;
        (s == 0 ? m.left_prob : m.right_prob) = prob;
        if (prob > 0.0 && side.seen_frame >= lane_frame_id_) {
            lane_frame_id_ = side.seen_frame;
            lane_age_s_ = t_s - side.seen_t_s;
        }
    }
    // 없는 선의 자리 표시(확률 0이라 차선 코어는 경로에 쓰지 않는다). 한쪽만 없으면 남은 선을 kDefaultWidthM만큼 옮겨
    // 폭 검사가 엉뚱한 supercombo 선을 보지 않게 하고, 둘 다 없으면 supercombo 선을 둔다.
    if (have[0] && !have[1]) {
        for (int i = 0; i < kTrajectorySize; ++i) m.right[i] = m.left[i] + kDefaultWidthM;
    } else if (!have[0] && have[1]) {
        for (int i = 0; i < kTrajectorySize; ++i) m.left[i] = m.right[i] - kDefaultWidthM;
    } else if (!have[0] && !have[1]) {
        for (int i = 0; i < kTrajectorySize; ++i) {
            m.left[i] = model.lanes[1][i].y;
            m.right[i] = model.lanes[2][i].y;
        }
    }
    return m;
}
