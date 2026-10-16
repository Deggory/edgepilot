#include "hud/hud_draw.h"

#include <array>
#include <iterator>
#include <optional>

// HUD의 도로 장면: 모델이 본 도로 경계·차선선·경로를 거리에 따라 흐려지는 띠로, vision 앞차를
// 위험도 색의 갈매기표로, 차 위치를 자기 차선 안 눈금으로 그린다. 모델 좌표를 투영해 화면에 놓는다.

namespace hud_draw {
namespace {

constexpr float kMinDrawDistance = 10.0f;
constexpr float kMaxDrawDistance = 100.0f;
constexpr float kPathHalfWidth = 0.9f;
constexpr float kLaneHalfWidthMin = 0.02f;
constexpr float kLaneHalfWidthPerProbability = 0.03f;
constexpr float kEdgeHalfWidth = 0.04f;
constexpr float kLaneMinProbability = 0.05f;
constexpr float kEdgeMinConfidence = 0.05f;
constexpr float kFarAlpha = 0.12f;  // 띠의 먼 끝 알파 비

constexpr int kLeadTimeIndex = 0;
constexpr float kLeadRiskDistanceM = 40.0f;
constexpr float kLeadRiskClosingMps = 10.0f;
constexpr float kLeadClosingLabelKph = -3.0f;

// 가깝거나 빠르게 다가올수록 1
float lead_risk(const LeadInfo &lead) {
    const float distance = 1.0f - lead.distance_m / kLeadRiskDistanceM;
    const float closing = -lead.relative_speed_kph / 3.6f / kLeadRiskClosingMps;
    return std::clamp(std::clamp(distance, 0.0f, 1.0f) + std::clamp(closing, 0.0f, 1.0f), 0.0f, 1.0f);
}

// 모델 좌표는 180° 뒤집힌 화면 기준이라 투영한 뒤 뒤집는다.
std::optional<HudPoint> project(const HudCanvas &canvas, const ProjectionState &projection, float x, float y, float z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return std::nullopt;
    float px = 0.0f, py = 0.0f;
    if (!project_point_subpixel(projection, x, y, z, canvas.width(), canvas.height(), &px, &py)) return std::nullopt;
    return HudPoint{static_cast<float>(canvas.width() - 1) - px, static_cast<float>(canvas.height() - 1) - py};
}

// 궤적 양쪽을 투영한 띠를 가까운 쪽 알파에서 먼 쪽으로 흐리게 칠한다. 접히거나(감기는 방향이
// 바뀜) 뒤로 가는 구간에서 멈춘다. stops가 있으면 색은 가까운 stops[0]에서 먼 끝 색까지 바뀐다(무지개).
void draw_ribbon(HudCanvas &canvas, const std::array<ModelPoint, kTrajectorySize> &points, float half_width,
                 float z_offset, float max_distance, uint32_t color, const ProjectionState &projection,
                 const uint32_t *stops = nullptr, int stop_count = 0) {
    HudPoint left[kTrajectorySize], right[kTrajectorySize];
    int n = 0;
    float previous_x = -1.0f, winding = 0.0f;
    for (const ModelPoint &point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            if (n) break;
            continue;
        }
        if (point.x > max_distance) break;
        if (point.x < 0.5f) continue;
        if (n && point.x <= previous_x) break;
        const auto l = project(canvas, projection, point.x, point.y - half_width, point.z + z_offset);
        const auto r = project(canvas, projection, point.x, point.y + half_width, point.z + z_offset);
        if (!l || !r) {
            if (n) break;
            continue;
        }
        if (n) {
            // 사각형 (이전 왼쪽, 지금 왼쪽, 지금 오른쪽, 이전 오른쪽)의 부호 넓이
            const HudPoint quad[] = {left[n - 1], *l, *r, right[n - 1]};
            float area = 0.0f;
            for (int i = 0; i < 4; ++i) area += quad[i].x * quad[(i + 1) % 4].y - quad[(i + 1) % 4].x * quad[i].y;
            if (std::fabs(area) < 1.0f || (winding != 0.0f && area * winding <= 0.0f)) break;
            if (winding == 0.0f) winding = area;
        }
        left[n] = *l;
        right[n] = *r;
        previous_x = point.x;
        ++n;
    }
    if (n < 2) return;

    HudPoint polygon[2 * kTrajectorySize];
    for (int i = 0; i < n; ++i) {
        polygon[i] = left[i];
        polygon[2 * n - 1 - i] = right[i];
    }
    const float near_y = std::max(left[0].y, right[0].y);
    const float far_y = std::min(left[n - 1].y, right[n - 1].y);
    // 색 띠는 화면에 보이는 부분에 펼친다(가까운 끝은 보닛 아래 화면 밖이라 그대로 두면 첫 색이 보이지 않는다).
    // 흐림은 다른 띠와 같은 높이로 재 경로 모드가 바뀌어도 띠의 알파가 같다.
    const HudColorRamp ramp{stops, stop_count, std::min(near_y, static_cast<float>(canvas.height() - 1)), far_y};
    canvas.fill_polygon_faded(polygon, 2 * n, color, near_y, far_y, kFarAlpha, stops ? &ramp : nullptr);
}

// 조향 출력이 한계에 가까운 정도(0..1, 출력 70%에서 0, 100%에서 1). 경로 띠가 그만큼 주황으로 기운다.
float path_strain(const HudState &hud) {
    return std::clamp((std::fabs(hud.normalized_output) - 0.7f) / 0.3f, 0.0f, 1.0f);
}

uint32_t path_color(const HudState &hud) {
    if (!hud.controller_engaged) return hud_argb(120, 255, 255, 255);
    if (!steering_now(hud)) return hud_fade(kGray, 0.65f);
    const uint32_t mode = hud.laneless_mode ? kBlue : hud.lanebow_mode ? kViolet : kGreen;
    return hud_fade(mix(mode, kAmber, path_strain(hud)), 0.75f);
}

void draw_lead_marker(HudCanvas &canvas, const LeadInfo &lead, const ProjectionState &projection) {
    const auto at = project(canvas, projection, lead.point.x, lead.point.y, kModelHeight);
    if (!at) return;
    const float size = std::clamp(15.0f - lead.distance_m * 0.06f, 8.0f, 15.0f);
    const float cx = std::clamp(at->x, size + 6.0f, static_cast<float>(canvas.width()) - size - 6.0f);
    const float cy = std::clamp(at->y + size, size + 6.0f, static_cast<float>(canvas.height()) - size - 30.0f);
    const float risk = lead_risk(lead);
    const uint32_t fill = risk > 0.75f ? kRed : mix(kText, kAmber, risk / 0.75f);

    auto chevron = [&](float grow, uint32_t color) {
        const HudPoint points[] = {{cx, cy - size - grow},
                                   {cx + size * 1.2f + grow, cy + size * 0.6f + grow},
                                   {cx, cy + size * 0.15f + grow * 0.5f},
                                   {cx - size * 1.2f - grow, cy + size * 0.6f + grow}};
        canvas.fill_polygon(points, 4, color);
    };
    chevron(3.0f, kShadow);
    chevron(0.0f, hud_fade(fill, 0.75f + 0.25f * lead.probability));
    canvas.text(static_cast<int>(cx), static_cast<int>(cy + size * 0.6f) + 8, lead_text(lead), kHudCaptionFont, kText,
                HudAlign::center, true);
}

}  // namespace

LeadInfo lead_info(const HudState &hud, const ParsedModelOutput &output) {
    LeadInfo info;
    info.vision =
        output.valid && output.leads.primary(kLeadTimeIndex, kLeadProbabilityThreshold, &info.point, &info.probability);
    info.radar = hud.radar_lead_valid && std::isfinite(hud.radar_lead_distance_m) && hud.radar_lead_distance_m > 0.0f;
    if (info.radar) {
        info.distance_m = hud.radar_lead_distance_m;
        info.relative_speed_kph = hud.radar_lead_relative_speed_mps * 3.6f;
    } else if (info.vision) {
        info.distance_m = std::max(0.0f, info.point.x - kRadarToCameraDistanceM);
        info.relative_speed_kph = (info.point.velocity - hud.ego_speed_kph / 3.6f) * 3.6f;
    }
    return info;
}

std::string lead_text(const LeadInfo &lead) {
    if (lead.relative_speed_kph <= kLeadClosingLabelKph)
        return format_text("%.0f m  %.0f km/h", lead.distance_m, lead.relative_speed_kph);
    return format_text("%.0f m", lead.distance_m);
}

void draw_scene(HudCanvas &canvas, const ParsedModelOutput &output, const ProjectionState &projection,
                const HudState &hud, const LeadInfo &lead, const LaneState *lane) {
    if (!output.valid) return;
    const float max_distance = output.plan.valid
                                   ? std::clamp(output.plan.points.back().x, kMinDrawDistance, kMaxDrawDistance)
                                   : kMaxDrawDistance;

    // Lanebow: LD 선이 supercombo 차선·경계를 대신하고, LD로 조향하는 동안 경로는 무지개다(출력이 한계에 가까우면
    // 다른 경로 띠처럼 주황으로 기운다). 무지개 띠는 모델 경로 띠와 같은 점·폭으로 LD 차로 가운데에 놓는다. 자차 두
    // 선이 없거나 차선을 바꾸는 중이면(플래너가 차선을 빼고 plan을 따른다) plan 그대로다.
    if (lane) {
        if (output.plan.valid) {
            const bool rainbow = lanebow_steering(hud) && steering_now(hud);
            uint32_t stops[std::size(kRainbow)];
            for (size_t i = 0; i < std::size(kRainbow); ++i) stops[i] = mix(kRainbow[i], kAmber, path_strain(hud));
            std::array<ModelPoint, kTrajectorySize> center;
            const bool centered =
                rainbow && hud.lane_change != 2 && lanebow_center_path(*lane, output.plan.points, &center);
            draw_ribbon(canvas, centered ? center : output.plan.points, kPathHalfWidth, kModelHeight, max_distance,
                        rainbow ? hud_fade(kText, 0.75f) : path_color(hud), projection, rainbow ? stops : nullptr,
                        static_cast<int>(std::size(stops)));
        }
        draw_lanebow_lines(canvas, *lane);
        if (lead.vision) draw_lead_marker(canvas, lead, projection);
        return;
    }
    if (!hud.laneless_mode) {
        for (const ParsedRoadEdge &edge : output.road_edges) {
            const float confidence = std::clamp(1.0f - edge.std, 0.0f, 1.0f);
            if (!edge.valid || confidence < kEdgeMinConfidence) continue;
            draw_ribbon(canvas, edge.points, kEdgeHalfWidth, 0.0f, max_distance, hud_fade(kRed, 0.7f * confidence),
                        projection);
        }
        for (const ParsedLaneLine &lane : output.lanes) {
            if (!lane.valid || lane.probability < kLaneMinProbability) continue;
            const float half_width = std::max(kLaneHalfWidthMin, kLaneHalfWidthPerProbability * lane.probability);
            draw_ribbon(canvas, lane.points, half_width, 0.0f, max_distance,
                        hud_fade(kText, 0.25f + 0.65f * lane.probability), projection);
        }
    }
    if (output.plan.valid)
        draw_ribbon(canvas, output.plan.points, kPathHalfWidth, kModelHeight, max_distance, path_color(hud),
                    projection);
    if (lead.vision) draw_lead_marker(canvas, lead, projection);
}

// 차선 안 위치: 가까운 거리(kLaneRulerIndex)에서 자기 차선 양쪽 선(l, r)을 잇는 가는 눈금을 길 위에
// 그리고, 그 거리의 축척대로 차선 중앙 눈금(선 위)과 차(선 아래, 위를 향한 삼각형, 조향 색)를
// 버니어처럼 위아래로 놓는다. 차는 차선 중앙 오프셋(x=0, 0.5초 평활)만큼 중앙에서 비켜 있고 아래에
// cm를 쓴다. 오프셋이 있을 때만(두 선이 다 확실할 때).
void draw_lane_position(HudCanvas &canvas, const ModelPoint &l, const ModelPoint &r, const ProjectionState &projection,
                        float lane_center_offset_m, uint32_t car_color) {
    if (!std::isfinite(lane_center_offset_m)) return;
    const float width_m = r.y - l.y;
    const auto left = project(canvas, projection, l.x, l.y, l.z), right = project(canvas, projection, r.x, r.y, r.z);
    if (!left || !right || width_m < 2.0f || right->x <= left->x) return;
    const uint32_t ink = hud_fade(kText, 0.9f);
    const HudPoint rule[] = {
        {left->x, left->y - 1.0f}, {right->x, right->y - 1.0f}, {right->x, right->y + 1.0f}, {left->x, left->y + 1.0f}};
    canvas.fill_polygon(rule, 4, hud_fade(kText, 0.45f));
    // 눈금은 화소에 맞춘 사각형(가늘어 둥글릴 필요가 없고, 다각형보다 싸다)
    auto tick = [&](float x, float y, int w, int top, int h) {
        canvas.fill_rect(static_cast<int>(std::lround(x)) - w / 2, static_cast<int>(std::lround(y)) + top, w, h, ink);
    };
    tick(left->x, left->y, 3, -8, 16);
    tick(right->x, right->y, 3, -8, 16);
    const HudPoint center{(left->x + right->x) / 2.0f, (left->y + right->y) / 2.0f};
    tick(center.x, center.y, 2, -10, 9);
    // + = 차선 중앙이 오른쪽 = 차가 왼쪽
    const float car_x = center.x - lane_center_offset_m * (right->x - left->x) / width_m;
    const HudPoint car[] = {
        {car_x, center.y + 3.0f}, {car_x + 7.0f, center.y + 13.0f}, {car_x - 7.0f, center.y + 13.0f}};
    canvas.fill_polygon(car, 3, car_color);
    const long cm = std::lround(std::fabs(lane_center_offset_m) * 100.0f);
    canvas.text(static_cast<int>(car_x), static_cast<int>(center.y) + 16,
                cm == 0 ? std::string("0 cm") : format_text("%ld cm %s", cm, lane_center_offset_m > 0.0f ? "L" : "R"),
                kHudCaptionFont, kText, HudAlign::center, true);
}

}  // namespace hud_draw
