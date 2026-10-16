#include "hud/hud_draw.h"

#include "common/utils_math.h"

#include <algorithm>
#include <cmath>

// HUD의 Lanebow 장면(docs/lanebow.md): 차선선 플러그인(LD)이 본 선을 실제 모양(실선·점선·보츠)과 실제 색으로, 도로
// 경계를 가는 선으로 그린다. 겹선도 한 줄로 그린다(두 줄로 그리면 카메라의 도색 두 줄과 함께 네 줄로 보였다). LD 점은
// 이미 프레임 화소라 보정·투영 없이 화면으로 옮긴다(frame_pixel_to_screen). LD로 조향하는 동안의 무지개 경로는 모델
// 경로와 같은 띠를 LD 차로 가운데로 옮겨 그린다(lanebow_center_path, hud_scene.cc).

namespace hud_draw {
namespace {

// lane_plugin_abi.h의 값(HUD는 플러그인 헤더를 보지 않는다)
constexpr int kShapeSolid = 0, kShapeDashed = 1, kShapeBotts = 2;
constexpr int kDoubleDashed = 2, kDoubleBotts = 3;
constexpr int kColorWhite = 0, kColorYellow = 1, kColorBlue = 2;
constexpr int kBoundaryCurb = 3, kBoundaryCones = 5;

constexpr int kMaxRibbonPoints = 34;  // 띠 하나 = 꼭짓점 2n개, 캔버스 한도 72 안
// 선 반폭: LD가 잰 도색 반폭의 kPaintWidthScale배를 [kMinHalfWidthPx, kMaxHalfWidthPx]로 자른다. 잰 폭 그대로는
// supercombo 선(반폭 2~5 cm 띠)보다 두세 배 굵었고, 가까운 점선은 번진 폭까지 재 반폭이 27 px까지 나왔다.
constexpr float kPaintWidthScale = 0.6f;
constexpr float kMinHalfWidthPx = 1.0f;
constexpr float kMaxHalfWidthPx = 5.0f;
constexpr float kBoundaryHalfWidthPx = 1.1f;
// 점선 무늬: 카메라 높이(kModelHeight)의 평평한 도로를 가정한 거리로 dash 4 m, 간격 5 m(실제 도색 길이와 같지 않은
// 표시용)
constexpr float kDashOnM = 4.0f;
constexpr float kDashPeriodM = 9.0f;

struct ScreenLine {
    HudPoint p[kLaneMaxPoints];
    float half[kLaneMaxPoints];  // 반폭(화면 px)
    float v[kLaneMaxPoints];     // 원래 프레임 행(점선 거리용)
    int n = 0;
};

// LaneState 점(1/4 화소, 도색 가운데) → 화면. 반폭은 fixed_half_px가 있으면 그 값, 없으면 LD가 잰 도색 폭을 같은
// 투영으로 옮긴 것.
ScreenLine to_screen(const LanePointState *points, uint32_t count, const LaneState &lane, const HudCanvas &canvas,
                     float fixed_half_px = 0.0f) {
    ScreenLine line;
    if (lane.image_width == 0 || lane.image_height == 0) return line;
    auto screen = [&](float u, float v, float *x, float *y) {
        frame_pixel_to_screen(u, v, lane.image_width, lane.image_height, canvas.width(), canvas.height(), x, y);
    };
    for (uint32_t i = 0; i < std::min<uint32_t>(count, kLaneMaxPoints); ++i) {
        const LanePointState &pt = points[i];
        const float u = pt.x4 / 4.0f, v = pt.y4 / 4.0f;
        float x = 0.0f, y = 0.0f, paint_edge_x = 0.0f, unused = 0.0f;
        screen(u, v, &x, &y);
        float half = fixed_half_px;
        if (!(half > 0.0f)) {
            screen(u + 0.5f * pt.width4 / 4.0f, v, &paint_edge_x, &unused);
            half = std::clamp(kPaintWidthScale * (paint_edge_x - x), kMinHalfWidthPx, kMaxHalfWidthPx);
        }
        line.p[line.n] = {x, y};
        line.half[line.n] = half;
        line.v[line.n] = v;
        ++line.n;
    }
    return line;
}

// 점들을 잇는 띠(점마다 반폭). 점이 많으면 고르게 솎는다.
void stroke(HudCanvas &canvas, const HudPoint *p, const float *half, int n, uint32_t color) {
    if (n < 2) return;
    HudPoint pts[kMaxRibbonPoints];
    float hw[kMaxRibbonPoints];
    int m = 0;
    for (int k = 0; k < kMaxRibbonPoints && k < n; ++k) {
        const int i = n <= kMaxRibbonPoints ? k : static_cast<int>(std::lround(k * (n - 1.0) / (kMaxRibbonPoints - 1)));
        if (m && std::fabs(p[i].x - pts[m - 1].x) < 0.05f && std::fabs(p[i].y - pts[m - 1].y) < 0.05f) continue;
        pts[m] = p[i];
        hw[m] = half[i];
        ++m;
    }
    if (m < 2) return;
    HudPoint polygon[2 * kMaxRibbonPoints];
    for (int i = 0; i < m; ++i) {
        const HudPoint &a = pts[std::max(0, i - 1)], &b = pts[std::min(m - 1, i + 1)];
        float tx = b.x - a.x, ty = b.y - a.y;
        const float len = std::max(1e-3f, std::sqrt(tx * tx + ty * ty));
        tx /= len;
        ty /= len;
        polygon[i] = {pts[i].x - ty * hw[i], pts[i].y + tx * hw[i]};
        polygon[2 * m - 1 - i] = {pts[i].x + ty * hw[i], pts[i].y - tx * hw[i]};
    }
    canvas.fill_polygon(polygon, 2 * m, color);
}

// 평평한 도로를 가정한 행 v의 앞 거리(점선 무늬용). 지평선 위면 큰 값.
float flat_distance(float v, float horizon_v, float fy) {
    return v > horizon_v + 1.0f ? fy * kModelHeight / (v - horizon_v) : 1e3f;
}

// 선의 무늬(kShape*). 겹선은 그 무늬(겹 실선·점선·보츠)의 한 줄로 그린다(두 줄로 그리면 카메라의 도색 두 줄과 함께
// 네 줄로 보였다).
int line_pattern(const LaneLineState &state) {
    if (state.double_line == kDoubleDashed) return kShapeDashed;
    if (state.double_line == kDoubleBotts) return kShapeBotts;
    return state.double_line ? kShapeSolid : state.shape;
}

// 무늬대로 그린다: 실선은 띠 하나, 점선은 이어진 점 묶음마다 띠, 보츠는 점마다 작은 마름모.
void stroke_pattern(HudCanvas &canvas, const ScreenLine &line, int pattern, uint32_t color, const LaneState &lane) {
    if (pattern == kShapeBotts) {
        for (int i = 0; i < line.n; i += 3) {
            const float r = std::max(1.5f, line.half[i] * 1.4f);
            const HudPoint &c = line.p[i];
            const HudPoint diamond[] = {{c.x, c.y - r}, {c.x + r, c.y}, {c.x, c.y + r}, {c.x - r, c.y}};
            canvas.fill_polygon(diamond, 4, color);
        }
        return;
    }
    if (pattern != kShapeDashed) {
        stroke(canvas, line.p, line.half, line.n, color);
        return;
    }
    const float h = static_cast<float>(lane.image_height);
    const float horizon = lane.vanishing_confidence > 0.3f ? lane.vanishing_y : 0.5f * h;
    const float fy = kCameraFy * h / static_cast<float>(kDefaultSensorHeight);
    int start = -1;
    for (int i = 0; i <= line.n; ++i) {
        const bool on = i < line.n && std::fmod(flat_distance(line.v[i], horizon, fy), kDashPeriodM) < kDashOnM;
        if (on && start < 0) start = i;
        if (!on && start >= 0) {
            // 묶음이 한 점뿐이면 다음 점까지 이어 짧은 조각으로
            const int end = std::min(line.n, std::max(i, start + 2));
            stroke(canvas, line.p + start, line.half + start, end - start, color);
            start = -1;
        }
    }
}

uint32_t paint_color(int color, float confidence) {
    const float alpha = 0.55f + 0.4f * std::clamp(confidence, 0.0f, 1.0f);
    const uint32_t base = color == kColorWhite    ? hud_argb(255, 245, 245, 245)
                          : color == kColorYellow ? hud_argb(255, 255, 204, 0)
                          : color == kColorBlue   ? kBlue
                                                  : hud_argb(255, 175, 180, 188);
    return hud_fade(base, alpha);
}

uint32_t boundary_color(int type) {
    if (type == kBoundaryCones) return hud_fade(kAmber, 0.8f);
    if (type == kBoundaryCurb) return hud_fade(hud_argb(255, 255, 120, 90), 0.75f);
    return hud_fade(hud_argb(255, 200, 204, 212), 0.6f);
}

}  // namespace

bool lanebow_center_path(const LaneState &lane, const std::array<ModelPoint, kTrajectorySize> &plan,
                         std::array<ModelPoint, kTrajectorySize> *out) {
    const LaneRoadLine *left = lane_ego_road_line(lane, kLaneSlotEgoLeft);
    const LaneRoadLine *right = lane_ego_road_line(lane, kLaneSlotEgoRight);
    if (!left || !right) return false;
    // plan의 앞 거리와 횡위치(앞 거리가 줄어드는 점은 평평하게)
    float plan_x[kTrajectorySize], plan_y[kTrajectorySize];
    for (int i = 0; i < kTrajectorySize; ++i) {
        plan_x[i] = std::isfinite(plan[i].x) ? plan[i].x : (i ? plan_x[i - 1] : 0.0f);
        plan_y[i] = std::isfinite(plan[i].y) ? plan[i].y : (i ? plan_y[i - 1] : 0.0f);
    }
    make_monotonic(plan_x, kTrajectorySize);
    // LD 두 선이 다 있는 격자에서 LD 차로 가운데 - plan
    float xs[kLaneRoadPoints], ds[kLaneRoadPoints];
    size_t n = 0;
    for (unsigned i = 0; i < kLaneRoadPoints; ++i) {
        if (!std::isfinite(left->y[i]) || !std::isfinite(right->y[i]) || right->y[i] <= left->y[i]) continue;
        xs[n] = model_x_idx(static_cast<int>(i));
        ds[n] = 0.5f * (left->y[i] + right->y[i]) - interp(xs[n], plan_x, plan_y, kTrajectorySize);
        ++n;
    }
    if (n < 2) return false;
    for (int i = 0; i < kTrajectorySize; ++i) {
        (*out)[i] = plan[i];
        (*out)[i].y = plan[i].y + interp(plan[i].x, xs, ds, n);
    }
    return true;
}

void draw_lanebow_lines(HudCanvas &canvas, const LaneState &lane) {
    for (uint32_t i = 0; i < std::min<uint32_t>(lane.boundary_count, kLaneMaxBoundaries); ++i) {
        const LaneBoundaryState &b = lane.boundaries[i];
        const ScreenLine line = to_screen(b.points, b.point_count, lane, canvas, kBoundaryHalfWidthPx);
        stroke(canvas, line.p, line.half, line.n, boundary_color(b.type));
    }
    for (uint32_t i = 0; i < std::min<uint32_t>(lane.line_count, kLaneMaxLines); ++i) {
        const LaneLineState &state = lane.lines[i];
        const ScreenLine line = to_screen(state.points, state.point_count, lane, canvas);
        stroke_pattern(canvas, line, line_pattern(state), paint_color(state.color, state.confidence), lane);
    }
}

}  // namespace hud_draw
