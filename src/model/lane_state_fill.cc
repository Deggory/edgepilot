#include "model/lane_state_fill.h"

#include "common/projection.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

static_assert(kLaneSlotEgoLeft == EP_LANE_EGO_LEFT && kLaneSlotEgoRight == EP_LANE_EGO_RIGHT,
              "LaneState slots are the plugin ABI's");

namespace {

// 두 점이 이보다 멀면(앞 방향) 그 사이를 잇지 않는다. 점선 사이 빈 구간은 잇되, 선이 끊긴 곳은 남긴다.
constexpr float kMaxGapM = 8.0f;
// 가까운 쪽 늘리기: 가장 가까운 측정이 이 안이어야 하고, 이 범위의 점에 2차식을 맞춘다.
constexpr float kNearStartM = 8.0f;
constexpr float kNearFitToM = 20.0f;
constexpr int kNearFitMinPoints = 4;
// 늘린 선의 2차 계수 상한(y = a + b x + c x²에서 c = 곡률/2): 곡률 0.02(반지름 50 m, 램프)
constexpr float kNearMaxQuadratic = 0.01f;
// 노면 높이로 쓸 수 있는 값(카메라 아래 m)과 그 값을 보고 광선을 맞히는 앞쪽 격자 수: 도로면 블록 격자(48 m)와 그
// 너머 한 칸(X_IDXS[17] = 54 m)까지. 99 m까지 보면 오르막 끝의 높이가 0.3 m 아래로 내려가 선 전체가 평평한 면으로
// 바뀌었다(10-12·10-15 주행 프레임의 26~28 %, 31.7 m에서 약 0.5 m).
constexpr float kMinSurfaceZ = 0.3f;
constexpr float kMaxSurfaceZ = 10.0f;
constexpr int kSurfacePoints = kLaneRoadPoints + 1;

int16_t quarter_pixel(float v) {
    const long q = std::lround(v * 4.0f);
    return static_cast<int16_t>(
        std::clamp<long>(q, std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::max()));
}

void copy_points(const EpLanePoint *from, uint32_t count, LanePointState *to, uint32_t *to_count) {
    const uint32_t n = std::min<uint32_t>(count, kLaneMaxPoints);
    for (uint32_t i = 0; i < n; ++i) {
        to[i].x4 = quarter_pixel(from[i].x);
        to[i].y4 = quarter_pixel(from[i].y);
        to[i].width4 = quarter_pixel(from[i].width);
        to[i].edge_x4 = quarter_pixel(from[i].edge_x);
    }
    *to_count = n;
}

// 기기 좌표(앞, 오른쪽, 아래)의 광선을 보정 좌표로 돌린다: calib = R^T device(R = device_from_calib).
void calib_ray(const LaneCamera &camera, float u, float v, float *calib) {
    const float device[3] = {1.0f, (u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy};
    float rot[9];
    rotation_from_rpy(camera.roll, camera.pitch, camera.yaw, rot);
    for (int i = 0; i < 3; ++i)
        calib[i] = rot[0 * 3 + i] * device[0] + rot[1 * 3 + i] * device[1] + rot[2 * 3 + i] * device[2];
}

bool usable_surface(const std::array<float, kTrajectorySize> &z) {
    for (int i = 0; i < kSurfacePoints; ++i)
        if (!std::isfinite(z[i]) || z[i] < kMinSurfaceZ || z[i] > kMaxSurfaceZ) return false;
    return true;
}

// 선마다 노면 높이 출처: 같은 선의 supercombo 높이, 못 쓰면 자차 두 선의 평균, 그것도 못 쓰면 평평한 면(flat).
const float *surface_for(const LaneRoadProfile &profile, int8_t kind, int8_t slot,
                         const std::array<float, kTrajectorySize> &mean, bool mean_ok,
                         const std::array<float, kTrajectorySize> &flat) {
    const std::array<float, kTrajectorySize> *own = nullptr;
    if (kind == kLaneRoadKindLine) {
        switch (slot) {
            case EP_LANE_EGO_LEFT:
            case EP_LANE_LEFT_LANE_RIGHT:
                own = &profile.lane_z[1];
                break;
            case EP_LANE_EGO_RIGHT:
            case EP_LANE_RIGHT_LANE_LEFT:
                own = &profile.lane_z[2];
                break;
            case EP_LANE_LEFT_NEXT:
                own = &profile.lane_z[0];
                break;
            default:
                own = &profile.lane_z[3];
                break;
        }
    } else {
        own = (slot == EP_BOUNDARY_NEAR_LEFT || slot == EP_BOUNDARY_FAR_LEFT) ? &profile.edge_z[0] : &profile.edge_z[1];
    }
    if (usable_surface(*own)) return own->data();
    return mean_ok ? mean.data() : flat.data();
}

struct RoadPoint {
    float x = 0.0f;  // 앞(m)
    float y = 0.0f;  // 오른쪽(m)
};

// 도로면 점들(x 순)을 격자 X_IDXS[0..kLaneRoadPoints)에서 읽어 out에. 8 m 넘게 빈 곳과 덮지 않는 곳은 NaN.
// 가장 가까운 점이 kNearStartM 안이고 거기서 처음 덮은 격자까지 끊김 없이 이어지면, 그보다 가까운 격자는 끊김 전의
// 가까운 점(kNearFitToM까지)에 맞춘 2차식으로 늘린다.
void sample_road(const std::vector<RoadPoint> &road, LaneRoadLine *out) {
    for (unsigned k = 0; k < kLaneRoadPoints; ++k) {
        const float d = model_x_idx(static_cast<int>(k));
        for (size_t i = 1; i < road.size(); ++i) {
            const RoadPoint &a = road[i - 1], &b = road[i];
            if (a.x <= d && d <= b.x && b.x - a.x < kMaxGapM) {
                out->y[k] = b.x > a.x ? a.y + (b.y - a.y) * (d - a.x) / (b.x - a.x) : a.y;
                if (out->first < 0) out->first = static_cast<int8_t>(k);
                out->last = static_cast<int8_t>(k);
                break;
            }
        }
    }
    if (out->first <= 0 || road.empty() || road.front().x > kNearStartM) return;
    // 가까운 점들이 끊긴 조각이면(처음 덮은 격자 전에 kMaxGapM 넘는 빈 곳) 그 사이를 지어내지 않는다
    const float first_x = model_x_idx(out->first);
    size_t fit_end = road.size();
    for (size_t i = 1; i < road.size(); ++i) {
        if (road[i].x - road[i - 1].x < kMaxGapM) continue;
        if (road[i - 1].x < first_x) return;
        fit_end = i;
        break;
    }

    // 끊김 전 20 m까지의 점에 최소 제곱 2차식(정규 방정식). 곡률이 너무 크면 직선으로 맞춘다.
    double s[5] = {}, t[3] = {};
    int n = 0;
    for (size_t i = 0; i < fit_end; ++i) {
        const RoadPoint &p = road[i];
        if (p.x > kNearFitToM) break;
        double xp = 1.0;
        for (int j = 0; j < 5; ++j) {
            s[j] += xp;
            if (j < 3) t[j] += xp * p.y;
            xp *= p.x;
        }
        ++n;
    }
    if (n < kNearFitMinPoints) return;
    double a = 0.0, b = 0.0, c = 0.0;
    const double det =
        s[0] * (s[2] * s[4] - s[3] * s[3]) - s[1] * (s[1] * s[4] - s[3] * s[2]) + s[2] * (s[1] * s[3] - s[2] * s[2]);
    if (std::fabs(det) > 1e-9) {
        a = (t[0] * (s[2] * s[4] - s[3] * s[3]) - s[1] * (t[1] * s[4] - s[3] * t[2]) +
             s[2] * (t[1] * s[3] - s[2] * t[2])) /
            det;
        b = (s[0] * (t[1] * s[4] - s[3] * t[2]) - t[0] * (s[1] * s[4] - s[3] * s[2]) +
             s[2] * (s[1] * t[2] - t[1] * s[2])) /
            det;
        c = (s[0] * (s[2] * t[2] - t[1] * s[3]) - s[1] * (s[1] * t[2] - t[1] * s[2]) +
             t[0] * (s[1] * s[3] - s[2] * s[2])) /
            det;
    }
    if (!(std::fabs(det) > 1e-9) || !(std::fabs(c) <= kNearMaxQuadratic)) {
        const double line_det = s[0] * s[2] - s[1] * s[1];
        if (!(std::fabs(line_det) > 1e-9)) return;
        a = (t[0] * s[2] - s[1] * t[1]) / line_det;
        b = (s[0] * t[1] - s[1] * t[0]) / line_det;
        c = 0.0;
    }
    for (int k = 0; k < out->first; ++k) {
        const double x = model_x_idx(k);
        out->y[k] = static_cast<float>(a + b * x + c * x * x);
    }
}

// 선 하나(차선선은 안쪽 경계 edge_x, 도로 경계는 점)를 노면으로 옮겨 격자에서 읽는다.
void project_line(const EpLanePoint *points, uint32_t count, bool use_edge, const LaneCamera &camera,
                  const float *surface_x, const float *surface_z, LaneRoadLine *out) {
    std::vector<RoadPoint> road;
    road.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        const float u = use_edge ? points[i].edge_x : points[i].x;
        RoadPoint p;
        if (lane_pixel_to_surface(camera, surface_x, surface_z, kSurfacePoints, u, points[i].y, &p.x, &p.y))
            road.push_back(p);
    }
    std::sort(road.begin(), road.end(), [](const RoadPoint &a, const RoadPoint &b) { return a.x < b.x; });
    sample_road(road, out);
}

}  // namespace

LaneRoadProfile lane_road_profile(const ParsedModelOutput &parsed) {
    LaneRoadProfile profile;
    for (int i = 0; i < kTrajectorySize; ++i) {
        profile.x[i] = model_x_idx(i);
        for (int l = 0; l < 4; ++l) profile.lane_z[l][i] = parsed.lanes[l].points[i].z;
        for (int e = 0; e < 2; ++e) profile.edge_z[e][i] = parsed.road_edges[e].points[i].z;
    }
    return profile;
}

bool lane_pixel_to_surface(const LaneCamera &camera, const float *surface_x, const float *surface_z, int count, float u,
                           float v, float *x, float *y) {
    if (camera.fx <= 0.0f || camera.fy <= 0.0f || camera.height_m <= 0.0f || count < 2) return false;
    float calib[3];
    calib_ray(camera, u, v, calib);
    if (calib[0] <= 1e-4f || calib[2] <= 1e-4f) return false;
    // 앞 거리 d에서 광선의 높이(아래 +)는 d·calib_z/calib_x다. 노면 높이를 뺀 차이는 가까이서 음수이고, 노면 점
    // 사이에서는 둘 다 직선이라 처음 0이 되는 구간에서 정확히 풀린다.
    const float slope = calib[2] / calib[0];
    float prev_x = surface_x[0], prev_gap = surface_x[0] * slope - surface_z[0];
    if (!(prev_gap < 0.0f)) return false;
    float d = -1.0f;
    for (int i = 1; i < count; ++i) {
        const float gap = surface_x[i] * slope - surface_z[i];
        if (gap >= 0.0f) {
            d = prev_x + (-prev_gap) * (surface_x[i] - prev_x) / (gap - prev_gap);
            break;
        }
        prev_x = surface_x[i];
        prev_gap = gap;
    }
    if (!(d > 0.0f)) return false;
    const float s = d / calib[0];
    *x = d;
    // 모델 점은 가상 카메라(오프셋만큼 오른쪽) 기준이다. 워프는 도로면에 맞춘 변환이라 높이 z인 점을 offset·z/h만큼
    // 옮긴다(model_input_transform: 평평한 면에서는 offset 그대로).
    *y = calib[1] * s - camera.offset_m * (calib[2] * s) / camera.height_m;
    return true;
}

void fill_lane_state(const EpLaneResult &result, const LaneCamera &camera, const LaneRoadProfile &profile,
                     int image_width, int image_height, LaneState *out) {
    out->image_width = static_cast<uint16_t>(std::clamp(image_width, 0, 65535));
    out->image_height = static_cast<uint16_t>(std::clamp(image_height, 0, 65535));
    out->line_count = std::min<uint32_t>(result.line_count, kLaneMaxLines);
    for (uint32_t i = 0; i < out->line_count; ++i) {
        const EpLaneLine &from = result.lines[i];
        LaneLineState &to = out->lines[i];
        to.slot = static_cast<int8_t>(from.slot);
        to.shape = static_cast<int8_t>(from.shape);
        to.double_line = static_cast<int8_t>(from.double_line);
        to.color = static_cast<int8_t>(from.color);
        to.confidence = from.confidence;
        copy_points(from.points, from.point_count, to.points, &to.point_count);
    }
    out->boundary_count = std::min<uint32_t>(result.boundary_count, kLaneMaxBoundaries);
    for (uint32_t i = 0; i < out->boundary_count; ++i) {
        const EpLaneBoundary &from = result.boundaries[i];
        LaneBoundaryState &to = out->boundaries[i];
        to.slot = static_cast<int8_t>(from.slot);
        to.type = static_cast<int8_t>(from.type);
        to.confidence = from.confidence;
        copy_points(from.points, from.point_count, to.points, &to.point_count);
    }
    out->vanishing_x = result.vanishing_x;
    out->vanishing_y = result.vanishing_y;
    out->vanishing_confidence = result.vanishing_confidence;

    // 도로면 블록: 선이 앞, 경계가 뒤
    std::array<float, kTrajectorySize> mean{}, flat{};
    for (int i = 0; i < kTrajectorySize; ++i) mean[i] = 0.5f * (profile.lane_z[1][i] + profile.lane_z[2][i]);
    const bool mean_ok = usable_surface(mean);
    flat.fill(camera.height_m);
    out->road_count = 0;
    for (uint32_t i = 0; i < out->line_count; ++i) {
        const EpLaneLine &from = result.lines[i];
        LaneRoadLine &road = out->road[out->road_count++];
        road = LaneRoadLine();
        road.kind = kLaneRoadKindLine;
        road.index = static_cast<int8_t>(i);
        road.slot = out->lines[i].slot;
        road.type = out->lines[i].shape;
        road.double_line = out->lines[i].double_line;
        road.color = out->lines[i].color;
        road.confidence = out->lines[i].confidence;
        project_line(from.points, std::min<uint32_t>(from.point_count, EP_LANE_MAX_POINTS), true, camera,
                     profile.x.data(), surface_for(profile, road.kind, road.slot, mean, mean_ok, flat), &road);
    }
    for (uint32_t i = 0; i < out->boundary_count; ++i) {
        const EpLaneBoundary &from = result.boundaries[i];
        LaneRoadLine &road = out->road[out->road_count++];
        road = LaneRoadLine();
        road.kind = kLaneRoadKindBoundary;
        road.index = static_cast<int8_t>(i);
        road.slot = out->boundaries[i].slot;
        road.type = out->boundaries[i].type;
        road.confidence = out->boundaries[i].confidence;
        project_line(from.points, std::min<uint32_t>(from.point_count, EP_LANE_MAX_POINTS), false, camera,
                     profile.x.data(), surface_for(profile, road.kind, road.slot, mean, mean_ok, flat), &road);
    }
}
