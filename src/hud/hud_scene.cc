#include "hud/hud_draw.h"

#include <array>
#include <optional>

/* HUD의 도로 장면: 모델이 본 도로 경계·차선선·경로를 거리에 따라 흐려지는 띠로, vision 앞차를
 * 위험도 색의 갈매기표로, 차 위치를 자기 차선 안 눈금으로 그린다. 모델 좌표를 투영해 화면에 놓는다. */

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
constexpr int kLaneRulerIndex = 6;  // 차선 안 위치 눈금을 놓는 궤적 점(6.75 m, 보닛 바로 위)

constexpr int kLeadTimeIndex = 0;
constexpr float kLeadRiskDistanceM = 40.0f;
constexpr float kLeadRiskClosingMps = 10.0f;
constexpr float kLeadClosingLabelKph = -3.0f;

// 가깝거나 빠르게 다가올수록 1
float lead_risk(const LeadInfo &lead)
{
    const float distance = 1.0f - lead.distance_m / kLeadRiskDistanceM;
    const float closing = -lead.relative_speed_kph / 3.6f / kLeadRiskClosingMps;
    return std::clamp(std::clamp(distance, 0.0f, 1.0f) + std::clamp(closing, 0.0f, 1.0f), 0.0f, 1.0f);
}

/* 모델 좌표는 180° 뒤집힌 화면 기준이라 투영한 뒤 뒤집는다. */
std::optional<HudPoint> project(const HudCanvas &canvas, const ProjectionState &projection,
                                float x, float y, float z)
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return std::nullopt;
    float px = 0.0f, py = 0.0f;
    if (!project_point_subpixel(projection, x, y, z, canvas.width(), canvas.height(), &px, &py))
        return std::nullopt;
    return HudPoint{static_cast<float>(canvas.width() - 1) - px, static_cast<float>(canvas.height() - 1) - py};
}

/* 궤적 양쪽을 투영한 띠를 가까운 쪽 알파에서 먼 쪽으로 흐리게 칠한다. 접히거나(감기는 방향이
 * 바뀜) 뒤로 가는 구간에서 멈춘다. */
void draw_ribbon(HudCanvas &canvas, const std::array<ModelPoint, kTrajectorySize> &points,
                 float half_width, float z_offset, float max_distance, uint32_t color,
                 const ProjectionState &projection)
{
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
            for (int i = 0; i < 4; ++i)
                area += quad[i].x * quad[(i + 1) % 4].y - quad[(i + 1) % 4].x * quad[i].y;
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
    canvas.fill_polygon_faded(polygon, 2 * n, color, near_y, far_y, kFarAlpha);
}

uint32_t path_color(const HudState &hud)
{
    if (!hud.controller_engaged) return hud_argb(120, 255, 255, 255);
    if (!steering_now(hud)) return hud_fade(kGray, 0.65f);
    // 출력이 한계에 가까워지면 주황으로 기운다.
    const float strain = (std::fabs(hud.normalized_output) - 0.7f) / 0.3f;
    return hud_fade(mix(hud.laneless_mode ? kBlue : kGreen, kAmber, strain), 0.75f);
}

void draw_lead_marker(HudCanvas &canvas, const LeadInfo &lead, const ProjectionState &projection)
{
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
    canvas.text(static_cast<int>(cx), static_cast<int>(cy + size * 0.6f) + 8, lead_text(lead),
                kHudCaptionFont, kText, HudAlign::center, true);
}

}  // namespace

LeadInfo lead_info(const HudState &hud, const ParsedModelOutput &output)
{
    LeadInfo info;
    info.vision = output.valid && output.leads.primary(kLeadTimeIndex, kLeadProbabilityThreshold,
                                                       &info.point, &info.probability);
    info.radar = hud.radar_lead_valid && std::isfinite(hud.radar_lead_distance_m) &&
                 hud.radar_lead_distance_m > 0.0f;
    if (info.radar) {
        info.distance_m = hud.radar_lead_distance_m;
        info.relative_speed_kph = hud.radar_lead_relative_speed_mps * 3.6f;
    } else if (info.vision) {
        info.distance_m = std::max(0.0f, info.point.x - kRadarToCameraDistanceM);
        info.relative_speed_kph = (info.point.velocity - hud.ego_speed_kph / 3.6f) * 3.6f;
    }
    return info;
}

std::string lead_text(const LeadInfo &lead)
{
    if (lead.relative_speed_kph <= kLeadClosingLabelKph)
        return format_text("%.0f m  %.0f km/h", lead.distance_m, lead.relative_speed_kph);
    return format_text("%.0f m", lead.distance_m);
}

void draw_scene(HudCanvas &canvas, const ParsedModelOutput &output,
                const ProjectionState &projection, const HudState &hud, const LeadInfo &lead)
{
    if (!output.valid) return;
    const float max_distance = output.plan.valid
        ? std::clamp(output.plan.points.back().x, kMinDrawDistance, kMaxDrawDistance)
        : kMaxDrawDistance;

    if (!hud.laneless_mode) {
        for (const ParsedRoadEdge &edge : output.road_edges) {
            const float confidence = std::clamp(1.0f - edge.std, 0.0f, 1.0f);
            if (!edge.valid || confidence < kEdgeMinConfidence) continue;
            draw_ribbon(canvas, edge.points, kEdgeHalfWidth, 0.0f, max_distance,
                        hud_fade(kRed, 0.7f * confidence), projection);
        }
        for (const ParsedLaneLine &lane : output.lanes) {
            if (!lane.valid || lane.probability < kLaneMinProbability) continue;
            const float half_width = std::max(kLaneHalfWidthMin, kLaneHalfWidthPerProbability * lane.probability);
            draw_ribbon(canvas, lane.points, half_width, 0.0f, max_distance,
                        hud_fade(kText, 0.25f + 0.65f * lane.probability), projection);
        }
    }
    if (output.plan.valid)
        draw_ribbon(canvas, output.plan.points, kPathHalfWidth, kModelHeight, max_distance,
                    path_color(hud), projection);
    if (lead.vision) draw_lead_marker(canvas, lead, projection);
}

/* 차선 안 위치: 가까운 거리(kLaneRulerIndex)에서 자기 차선 양쪽 선을 잇는 가는 눈금을 길 위에
 * 그리고, 그 거리의 축척대로 차선 중앙 눈금(선 위)과 차(선 아래, 위를 향한 삼각형, 조향 색)를
 * 버니어처럼 위아래로 놓는다. 차는 모델이 본 차선 중앙 오프셋(x=0, 0.5초 평활)만큼 중앙에서 비켜
 * 있고 아래에 cm를 쓴다. 두 선이 다 확실할 때만. */
void draw_lane_position(HudCanvas &canvas, const ParsedModelOutput &output,
                        const ProjectionState &projection, float lane_center_offset_m, uint32_t car_color)
{
    if (!output.valid || !std::isfinite(lane_center_offset_m)) return;
    const ModelPoint &l = output.lanes[1].points[kLaneRulerIndex], &r = output.lanes[2].points[kLaneRulerIndex];
    const float width_m = r.y - l.y;
    const auto left = project(canvas, projection, l.x, l.y, l.z), right = project(canvas, projection, r.x, r.y, r.z);
    if (!left || !right || width_m < 2.0f || right->x <= left->x) return;
    const uint32_t ink = hud_fade(kText, 0.9f);
    const HudPoint rule[] = {{left->x, left->y - 1.0f}, {right->x, right->y - 1.0f},
                             {right->x, right->y + 1.0f}, {left->x, left->y + 1.0f}};
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
    const HudPoint car[] = {{car_x, center.y + 3.0f}, {car_x + 7.0f, center.y + 13.0f}, {car_x - 7.0f, center.y + 13.0f}};
    canvas.fill_polygon(car, 3, car_color);
    const long cm = std::lround(std::fabs(lane_center_offset_m) * 100.0f);
    canvas.text(static_cast<int>(car_x), static_cast<int>(center.y) + 16,
                cm == 0 ? std::string("0 cm") : format_text("%ld cm %s", cm, lane_center_offset_m > 0.0f ? "L" : "R"),
                kHudCaptionFont, kText, HudAlign::center, true);
}

}  // namespace hud_draw
