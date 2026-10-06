#include "overlay_renderer.h"

#include "calibration_online.h"
#include "can_frame.h"
#include "lateral_lag.h"
#include "overlay_canvas.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <optional>
#include <string>

/* MaixCAM2 HUD(640x480). 주행 화면에는 필요한 것만 둔다: 상태 테두리, 현재 속도와 그 양옆
 * 깜빡이·비상등(노랑), 설정 속도(비전 크루즈가 낮게 잡으면 SET도)와 기어, 조향 모드와 하고 있는
 * 조작(회전, 차선 변경), 경로·차선, 앞차, 토크 바(운전자 토크 눈금 포함), 알림, 오토 홀드(속도
 * 아래 큰 배지). 아래 두 모서리에는 TPMS와 카메라 보정을, 그 위에는 보드 상태와 주행에 쓰는
 * 학습값을 좌우 짝으로 늘 두고, 오른쪽 위 상태 알약은 녹화와 와이파이를 보이며 누르면 네트워크
 * 카드를 연다. panda·저장 공간은 문제가 있을 때 칩으로도 띄우고(온도는 보드 상태 카드가 색으로),
 * 다른 카드에 없는 수치 진단은 웹 기기 설정의 HUD 진단을 켜면 왼쪽 카드에 모은다. 색·간격·글꼴
 * 크기는 아래 토큰 한 곳에서 정한다. 크기는 2.4" 패널(333 ppi)을 차 안에서 읽을 만큼(2026-10-04
 * 실차에서 한 번 키움). */

namespace {

// ---- 디자인 토큰 ----

constexpr uint32_t kText = hud_argb(255, 255, 255, 255);
constexpr uint32_t kTextSecondary = hud_argb(235, 190, 196, 205);
constexpr uint32_t kCard = hud_argb(150, 12, 14, 18);
constexpr uint32_t kCardStrong = hud_argb(205, 12, 14, 18);
constexpr uint32_t kTrack = hud_argb(90, 255, 255, 255);
constexpr uint32_t kShadow = hud_argb(110, 0, 0, 0);
constexpr uint32_t kCarBody = hud_argb(70, 255, 255, 255);
constexpr uint32_t kDriverTorque = hud_argb(255, 90, 200, 250);  // 토크 바의 운전자 토크 눈금
constexpr uint32_t kSignalHousing = hud_argb(255, 58, 62, 70);    // 신호등 아이콘 몸체
constexpr uint32_t kGreen = hud_argb(255, 48, 209, 88);
constexpr uint32_t kYellow = hud_argb(255, 255, 204, 0);  // 깜빡이·비상등
constexpr uint32_t kBlue = hud_argb(255, 64, 156, 255);
constexpr uint32_t kGray = hud_argb(255, 150, 156, 165);
constexpr uint32_t kAmber = hud_argb(255, 255, 176, 32);
constexpr uint32_t kRed = hud_argb(255, 255, 69, 58);

constexpr int kMargin = 14;
constexpr int kGap = 8;
constexpr float kRadius = 10.0f;
constexpr int kBorder = 4;
constexpr int kCardPad = 10;
constexpr int kChipH = 30;
constexpr int kChipPadX = 12;
constexpr int kDot = 10;
constexpr int kTopCardH = 80;   // 왼쪽 위 설정 속도·기어 카드
constexpr int kSetCardW = 90;
constexpr int kGearCardW = 56;
constexpr int kNetworkCardMinW = 200;
constexpr int kTorqueBarW = 240;
constexpr int kTorqueBarH = 6;
constexpr int kAlertH = 76;
constexpr int kAlertMinW = 320;
constexpr int kRowH = 19;  // 카드 안 이름·값 줄 간격
// 아래 모서리 카드(TPMS, 카메라 보정)와 보정 위 학습 카드. 모서리 카드는 위가 알림 카드, 아래가
// 토크 바와 맞는다. 알림 카드는 둘 사이(640 − 2·(14 + 126 + 8) = 344)에 들어간다.
constexpr int kCornerCardW = 126;
constexpr int kCornerCardH = kAlertH + kGap + kTorqueBarH;
constexpr int kStatusTouchW = 190;  // 상태 알약을 누른 것으로 보는 오른쪽 위 영역(손가락만큼 넉넉하게)
constexpr int kStatusTouchH = 76;
constexpr int kLeftColumnTouchW = 250;  // 왼쪽 열을 누른 것으로 보는 폭(진단 카드 대부분)

// ---- 장면 상수 ----

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

constexpr float kRadarToCameraDistanceM = 1.52f;
constexpr float kLeadProbabilityThreshold = 0.5f;
constexpr int kLeadTimeIndex = 0;
constexpr float kLeadRiskDistanceM = 40.0f;
constexpr float kLeadRiskClosingMps = 10.0f;
constexpr float kLeadClosingLabelKph = -3.0f;

constexpr int kTurnLitSteps = 15;
constexpr int kTurnChevronStartStep[] = {0, 4, 8};
constexpr float kTurnChevronAlpha[] = {0.35f, 0.65f, 1.0f};

constexpr int kWeakWifiDbm = -75;
constexpr float kWarmTempC = 70.0f;
constexpr float kHotTempC = 80.0f;
constexpr float kHighUsagePercent = 90.0f;  // 보드 상태 카드: CPU·메모리·저장 공간이 이만큼 차면 주황
constexpr float kTpmsLowBar = 2.2f;
constexpr float kTpmsHighBar = 2.8f;
constexpr float kTpmsLowPsi = 32.0f;
constexpr float kTpmsHighPsi = 45.0f;

template <typename... Args>
std::string format_text(const char *format, Args... args)
{
    char text[96];
    std::snprintf(text, sizeof(text), format, args...);
    return text;
}

uint32_t mix(uint32_t a, uint32_t b, float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    uint32_t out = 0;
    for (int shift = 0; shift < 32; shift += 8) {
        const float ca = static_cast<float>((a >> shift) & 0xff);
        const float cb = static_cast<float>((b >> shift) & 0xff);
        out |= static_cast<uint32_t>(std::lround(ca + (cb - ca) * t)) << shift;
    }
    return out;
}

// 줄 배치 기준 글리프: 대문자 H, 숫자만 있는 글꼴은 0.
const HudGlyph &cap_glyph(const HudFont &font)
{
    const HudGlyph *cap = font.glyph('H');
    return cap && cap->h ? *cap : *font.glyph('0');
}

// 대문자 윗선을 cap_top에 두는 줄 위 y.
int cap_line(const HudFont &font, int cap_top) { return cap_top - cap_glyph(font).top; }

// 대문자 아랫선(기준선)을 baseline에 두는 줄 위 y. 크기가 다른 글꼴을 한 줄에 맞출 때 쓴다.
int base_line(const HudFont &font, int baseline)
{
    return cap_line(font, baseline - cap_glyph(font).h);
}

// 대문자 높이를 box_h 안 가운데에 두는 줄 위 y.
int centered_line_top(const HudFont &font, int box_y, int box_h)
{
    return cap_line(font, box_y + (box_h - cap_glyph(font).h) / 2);
}

// ---- 상태 → 색·문구 ----

// 지금 조향하는가: 활성이고 85도 위에서 쉬는 중이 아니다.
bool steering_now(const OverlayHudState &hud) { return hud.controller_active && !hud.steer_paused; }

// 화면 테두리와 모드 점의 색. 결합 전이면 0(테두리 없음). 결합했지만 조향을 쉬면 회색.
uint32_t state_color(const OverlayHudState &hud)
{
    if (hud.steering_fault || hud.panda_faults != 0 || hud.soft_disabling) return kRed;
    if (!hud.services_healthy) return kAmber;
    if (steering_now(hud)) return hud.laneless_mode ? kBlue : kGreen;
    return hud.controller_engaged ? kGray : 0;
}

std::string mode_text(const OverlayHudState &hud)
{
    if (hud.controller_active)
        return hud.laneless_mode ? "LANELESS" : "LANE";
    if (!hud.controller_engaged) return hud.controller_enabled ? "READY" : "OFF";
    if (hud.active_block[0] == '\0') return "READY";
    if (const char *label = engage_block_label(hud.active_block)) return label;
    std::string fallback = hud.active_block;
    std::replace(fallback.begin(), fallback.end(), '_', ' ');
    return fallback;
}

const char *gear_text(int gear)
{
    switch (gear) {
    case kGearPark: return "P";
    case kGearDrive: return "D";
    case kGearNeutral: return "N";
    case kGearReverse: return "R";
    case kGearSport: return "S";
    default: return "-";
    }
}

// 기어 카드의 색: 후진은 주황, 주차·중립은 흐리게.
uint32_t gear_color(int gear)
{
    if (gear == kGearReverse) return kAmber;
    return gear == kGearDrive || gear == kGearSport ? kText : kTextSecondary;
}

bool speed_valid(float kph) { return std::isfinite(kph) && kph > 0.0f; }

// ---- 앞차 ----

struct LeadInfo {
    bool vision = false;
    bool radar = false;
    ParsedLeadPoint point{};
    float probability = 0.0f;
    float distance_m = 0.0f;
    float relative_speed_kph = 0.0f;
};

LeadInfo lead_info(const OverlayHudState &hud, const ParsedModelOutput &output)
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

// 가깝거나 빠르게 다가올수록 1
float lead_risk(const LeadInfo &lead)
{
    const float distance = 1.0f - lead.distance_m / kLeadRiskDistanceM;
    const float closing = -lead.relative_speed_kph / 3.6f / kLeadRiskClosingMps;
    return std::clamp(std::clamp(distance, 0.0f, 1.0f) + std::clamp(closing, 0.0f, 1.0f), 0.0f, 1.0f);
}

std::string lead_text(const LeadInfo &lead)
{
    if (lead.relative_speed_kph <= kLeadClosingLabelKph)
        return format_text("%.0f m  %.0f km/h", lead.distance_m, lead.relative_speed_kph);
    return format_text("%.0f m", lead.distance_m);
}

// ---- TPMS ----

// 공기압 하나의 색: 낮으면 주황, 높으면 빨강, 값이 없으면 0.
uint32_t tire_color(float pressure, bool bar)
{
    if (!std::isfinite(pressure) || pressure <= 0.0f) return 0;
    if (pressure > (bar ? kTpmsHighBar : kTpmsHighPsi)) return kRed;
    return pressure < (bar ? kTpmsLowBar : kTpmsLowPsi) ? kAmber : kText;
}

// ---- 알림 ----

uint32_t alert_color(HudAlertLevel level)
{
    switch (level) {
    case HudAlertLevel::proceed: return kGreen;
    case HudAlertLevel::caution: return kAmber;
    case HudAlertLevel::critical: return kRed;
    case HudAlertLevel::notice: break;
    }
    return kText;
}

// ---- 공통 부품 ----

// 칩 앞 표시: 색 점 또는 방향 화살표.
enum class ChipMark { dot, left, right };

// 방향 화살표: 가운데 (cx, cy), 폭 w, 높이 h인 삼각형. direction은 -1(왼쪽) 또는 1.
void draw_arrow(OverlayCanvas &canvas, float cx, float cy, float w, float h, int direction, uint32_t color)
{
    const float tip = cx + direction * w / 2.0f, back = cx - direction * w / 2.0f;
    const HudPoint points[] = {{tip, cy}, {back, cy - h / 2.0f}, {back, cy + h / 2.0f}};
    canvas.fill_polygon(points, 3, color);
}

/* 신호 대기: 한국식 가로 신호등(빨강·노랑·초록 중 빨강만 켜짐), 글 없이 아이콘만. 오른쪽 끝
 * right, 위 y, 높이 h. */
void draw_traffic_light(OverlayCanvas &canvas, int right, int y, int h)
{
    const int lamp = h - 4, w = 3 * lamp + 8, x = right - w;
    canvas.fill_round_rect(x, y, w, h, h / 2.0f, kSignalHousing);
    const uint32_t lamps[] = {kRed, hud_fade(kAmber, 0.25f), hud_fade(kGreen, 0.25f)};
    for (int i = 0; i < 3; ++i)
        canvas.fill_round_rect(x + 2 + i * (lamp + 2), y + 2, lamp, lamp, lamp / 2.0f, lamps[i]);
}

// 둥근 칩: 앞 표시와 글. x는 align 기준점. 그린 폭을 돌려준다.
int chip(OverlayCanvas &canvas, int x, int y, const std::string &text, uint32_t color,
         HudAlign align = HudAlign::left, ChipMark mark = ChipMark::dot)
{
    constexpr int kArrowW = 10;
    const int mark_w = mark == ChipMark::dot ? kDot : kArrowW;
    const int w = 2 * kChipPadX + mark_w + 6 + kHudBodyFont.width(text);
    const int left = align == HudAlign::right ? x - w : align == HudAlign::center ? x - w / 2 : x;
    canvas.fill_round_rect(left, y, w, kChipH, kChipH / 2.0f, kCard);
    const int mark_x = left + kChipPadX;
    if (mark == ChipMark::dot)
        canvas.fill_round_rect(mark_x, y + (kChipH - kDot) / 2, kDot, kDot, kDot / 2.0f, color);
    else
        draw_arrow(canvas, mark_x + kArrowW / 2.0f, y + kChipH / 2.0f, kArrowW, 13.0f,
                   mark == ChipMark::left ? -1 : 1, color);
    canvas.text(mark_x + mark_w + 6, centered_line_top(kHudBodyFont, y, kChipH), text, kHudBodyFont, kText);
    return w;
}

// ---- 장면(모델 출력) ----

/* 모델 좌표는 180° 뒤집힌 화면 기준이라 투영한 뒤 뒤집는다. */
std::optional<HudPoint> project(const OverlayCanvas &canvas, const ProjectionState &projection,
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
void draw_ribbon(OverlayCanvas &canvas, const std::array<ModelPoint, kTrajectorySize> &points,
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

uint32_t path_color(const OverlayHudState &hud)
{
    if (!hud.controller_engaged) return hud_argb(120, 255, 255, 255);
    if (!steering_now(hud)) return hud_fade(kGray, 0.65f);
    // 출력이 한계에 가까워지면 주황으로 기운다.
    const float strain = (std::fabs(hud.normalized_output) - 0.7f) / 0.3f;
    return hud_fade(mix(hud.laneless_mode ? kBlue : kGreen, kAmber, strain), 0.75f);
}

void draw_lead_marker(OverlayCanvas &canvas, const LeadInfo &lead, const ProjectionState &projection)
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

void draw_scene(OverlayCanvas &canvas, const ParsedModelOutput &output,
                const ProjectionState &projection, const OverlayHudState &hud, const LeadInfo &lead)
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
void draw_lane_position(OverlayCanvas &canvas, const ParsedModelOutput &output,
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

// ---- HUD ----

void draw_border(OverlayCanvas &canvas, uint32_t color)
{
    if (!color) return;
    const int w = canvas.width(), h = canvas.height();
    canvas.fill_rect(0, 0, w, kBorder, color);
    canvas.fill_rect(0, h - kBorder, w, kBorder, color);
    canvas.fill_rect(0, kBorder, kBorder, h - 2 * kBorder, color);
    canvas.fill_rect(w - kBorder, kBorder, kBorder, h - 2 * kBorder, color);
}

// 현재 속도 아래 "km/h" 줄의 위 y.
int speed_unit_line() { return kMargin + kHudSpeedFont.glyph('0')->h + 4; }

// 가운데 현재 속도와 그 양옆 깜빡이 화살표. 화살표 기준으로 쓸 숫자 폭을 돌려준다.
int draw_speed(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    const std::string speed = format_text("%.0f", std::max(0.0f, hud.cluster_speed_kph));
    const int center = canvas.width() / 2;
    const int top = kMargin - kHudSpeedFont.glyph('0')->top;
    const int width = canvas.text(center, top, speed, kHudSpeedFont, kText, HudAlign::center, true);
    canvas.text(center, speed_unit_line(), "km/h", kHudCaptionFont, kTextSecondary, HudAlign::center, true);
    return width;
}

/* 오토 홀드: 현재 속도 바로 아래 가운데의 큰 배지. 차 계기판처럼 초록 글씨로, 정차해 브레이크를
 * 잡고 있는 동안 멀리서도 보이게. */
void draw_brake_hold(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    if (!hud.brake_hold) return;
    constexpr int kH = 40, kPadX = 20;
    const char *text = "AUTO HOLD";
    const int w = 2 * kPadX + kHudTitleFont.width(text);
    const int y = speed_unit_line() + kHudCaptionFont.ascent + kGap;
    canvas.fill_round_rect((canvas.width() - w) / 2, y, w, kH, kH / 2.0f, kCardStrong);
    canvas.text(canvas.width() / 2, centered_line_top(kHudTitleFont, y, kH), text, kHudTitleFont, kGreen,
                HudAlign::center);
}

/* 현재 속도 양옆의 깜빡이 화살표(노랑, 비상등이면 양쪽). 세 개가 차례로 켜진다. 숫자에서 18 px
 * 띄우되, 세 자리 속도에서도 왼쪽 위 카드에 닿지 않게 안쪽으로 당긴다(오른쪽은 대칭). */
void draw_turn_signals(OverlayCanvas &canvas, const OverlayHudState &hud, int speed_width)
{
    if (!hud.left_blinker && !hud.right_blinker) return;
    const int step = std::clamp(hud.turn_signal_step, 0, kTurnSignalSteps - 1);
    if (step >= kTurnLitSteps) return;
    const float center_y = kMargin + kHudSpeedFont.glyph('0')->h / 2.0f;
    constexpr float kHalfH = 18.0f, kWidth = 18.0f, kThickness = 9.0f, kStep = 17.0f;
    constexpr float kReach = 2.0f * kStep + kThickness + kWidth;  // 안쪽 끝에서 바깥 화살표 끝까지
    constexpr float kCardsRight = kMargin + kSetCardW + kGap + kGearCardW + kGap;
    const float offset = std::min(speed_width / 2.0f + 18.0f, canvas.width() / 2.0f - kCardsRight - kReach);

    auto side = [&](bool active, float direction) {
        if (!active) return;
        const float inner = canvas.width() / 2.0f + direction * offset;
        for (int i = 0; i < 3 && step >= kTurnChevronStartStep[i]; ++i) {
            const float x = inner + direction * i * kStep;
            const HudPoint points[] = {{x, center_y - kHalfH},
                                       {x + direction * kThickness, center_y - kHalfH},
                                       {x + direction * (kThickness + kWidth), center_y},
                                       {x + direction * kThickness, center_y + kHalfH},
                                       {x, center_y + kHalfH},
                                       {x + direction * kWidth, center_y}};
            canvas.fill_polygon(points, 6, hud_fade(kYellow, kTurnChevronAlpha[i]));
        }
    };
    side(hud.left_blinker, -1.0f);
    side(hud.right_blinker, 1.0f);
}

/* 왼쪽 위 값 카드 하나: 위에 작은 이름, 그 아래 큰 값. sub가 있으면 큰 값을 이름 바로 아래로
 * 올리고 카드 아래쪽에 sub를 작게 쓴다. */
void top_card(OverlayCanvas &canvas, int x, int w, const char *title, const std::string &value, uint32_t color,
              const std::string &sub = {}, uint32_t sub_color = kTextSecondary)
{
    const int y = kMargin, header_bottom = y + kCardPad + cap_glyph(kHudCaptionFont).h;
    canvas.fill_round_rect(x, y, w, kTopCardH, kRadius, kCard);
    canvas.text(x + w / 2, cap_line(kHudCaptionFont, y + kCardPad), title, kHudCaptionFont, kTextSecondary,
                HudAlign::center);
    if (sub.empty()) {
        canvas.text(x + w / 2, centered_line_top(kHudValueFont, header_bottom, y + kTopCardH - 4 - header_bottom),
                    value, kHudValueFont, color, HudAlign::center);
        return;
    }
    canvas.text(x + w / 2, cap_line(kHudValueFont, header_bottom + 7), value, kHudValueFont, color, HudAlign::center);
    canvas.text(x + w / 2, base_line(kHudBodyFont, y + kTopCardH - kCardPad), sub, kHudBodyFont, sub_color,
                HudAlign::center);
}

/* 왼쪽 위: 설정 속도 카드와 기어 카드, 그 아래 조향 모드 칩. 비전 크루즈가 설정보다 낮게 잡고
 * 있으면 그 속도를 설정 속도 카드 아래쪽에 SET으로. 다음 칩이 올 y를 돌려준다. */
int draw_cruise(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    const int x = kMargin, y = kMargin;
    const bool max_valid = speed_valid(hud.cruise_max_speed_kph);
    const bool limited = max_valid && speed_valid(hud.cruise_command_speed_kph) &&
                         hud.cruise_command_speed_kph < hud.cruise_max_speed_kph - 2.0f;
    top_card(canvas, x, kSetCardW, "MAX", max_valid ? format_text("%.0f", hud.cruise_max_speed_kph) : "-",
             max_valid && hud.cruise_active ? kText : kTextSecondary,
             limited ? format_text("SET %.0f", hud.cruise_command_speed_kph) : std::string(), kAmber);
    // 차 상태가 없으면 기어도 모른다(gear 0은 P와 같다)
    top_card(canvas, x + kSetCardW + kGap, kGearCardW, "GEAR", hud.vehicle_fresh ? gear_text(hud.gear) : "-",
             hud.vehicle_fresh ? gear_color(hud.gear) : kTextSecondary);

    const int chip_y = y + kTopCardH + kGap;
    chip(canvas, x, chip_y, mode_text(hud), state_color(hud) ? state_color(hud) : kGray);
    return chip_y + kChipH + kGap;
}

constexpr float kWifiRadius = 20.5f;  // 와이파이 부채의 바깥 반지름(아이콘 폭 = 2·r·sin45°)

/* 와이파이 부채: 아래 꼭짓점의 점과 그 위 호 셋(45°~135°). 세기만큼 점부터 켜고 나머지 호는
 * 흐리게, 가장 약하면 점만 주황. 무선이 아니면(dbm 0) 다 켠다. (cx, apex)는 꼭짓점. */
void draw_wifi_icon(OverlayCanvas &canvas, float cx, float apex, int dbm)
{
    constexpr int kSegments = 8;
    constexpr float kBands[3][2] = {{7.0f, 9.5f}, {12.5f, 15.0f}, {18.0f, kWifiRadius}};
    const int level = dbm == 0 || dbm >= -55 ? 4 : dbm >= -65 ? 3 : dbm > kWeakWifiDbm ? 2 : 1;
    const uint32_t lit = level <= 1 ? kAmber : kText;
    canvas.fill_round_rect(cx - 2.5f, apex - 5.0f, 5.0f, 5.0f, 2.5f, lit);
    for (int band = 0; band < 3; ++band) {
        HudPoint points[2 * (kSegments + 1)];
        for (int i = 0; i <= kSegments; ++i) {
            const float angle = (0.25f + 0.5f * static_cast<float>(i) / kSegments) * 3.14159265f;
            const float c = std::cos(angle), s = std::sin(angle);
            points[i] = {cx + kBands[band][1] * c, apex - kBands[band][1] * s};
            points[2 * kSegments + 1 - i] = {cx + kBands[band][0] * c, apex - kBands[band][0] * s};
        }
        canvas.fill_polygon(points, 2 * (kSegments + 1), band + 1 < level ? lit : kTrack);
    }
}

/* 오른쪽 위 상태 알약: 녹화 중이면 빨간 점과 REC, 그리고 와이파이 부채(끊기면 OFFLINE).
 * 누르면 네트워크 카드가 열린다(hud_status_touch). 다음 줄 y를 돌려준다. */
int draw_status_pill(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    constexpr int kWifiW = 30, kItemGap = 12;  // kWifiW ≈ 2·kWifiRadius·sin45°
    const int y = kMargin;
    const int rec_w = hud.recording ? kDot + 6 + kHudBodyFont.width("REC") + kItemGap : 0;
    const int network_w = hud.network_connected ? kWifiW : kHudBodyFont.width("OFFLINE");
    const int w = 2 * kChipPadX + rec_w + network_w;
    int x = canvas.width() - kMargin - w;
    canvas.fill_round_rect(x, y, w, kChipH, kChipH / 2.0f, kCard);
    x += kChipPadX;
    const int line = centered_line_top(kHudBodyFont, y, kChipH);
    if (hud.recording) {
        canvas.fill_round_rect(x, y + (kChipH - kDot) / 2, kDot, kDot, kDot / 2.0f, kRed);
        canvas.text(x + kDot + 6, line, "REC", kHudBodyFont, kText);
        x += rec_w;
    }
    // 부채 높이(바깥 반지름)를 알약 높이 가운데에
    if (hud.network_connected)
        draw_wifi_icon(canvas, x + kWifiW / 2.0f, y + (kChipH + kWifiRadius) / 2.0f, hud.wifi_signal_dbm);
    else canvas.text(x, line, "OFFLINE", kHudBodyFont, kAmber);
    return y + kChipH + kGap;
}

/* 상태 알약을 눌러 연 네트워크 카드: 머리글에 신호 세기, 아래에 와이파이 이름, 주소,
 * 인터페이스(끊겼으면 "Not connected"). 와이파이 밖의 링크(USB 가상 이더넷)가 있으면 맨 아래에
 * 작게. 다음 줄 y를 돌려준다. */
int draw_network_card(OverlayCanvas &canvas, int y, const OverlayHudState &hud)
{
    const char *title = "WI-FI";
    const bool online = hud.network_connected, known_dbm = online && hud.wifi_signal_dbm != 0;
    const std::string status = !online ? "OFFLINE" : known_dbm ? format_text("%d dBm", hud.wifi_signal_dbm) : "";
    const bool weak = !online || (known_dbm && hud.wifi_signal_dbm <= kWeakWifiDbm);
    const std::string wired =
        hud.wired_interface[0] != '\0' ? format_text("%s  %s", hud.wired_interface, hud.wired_ipv4) : "";
    struct Line {
        const char *text;
        const HudFont &font;
        uint32_t color;
        int advance;  // 앞 줄 기준선에서 이 줄 기준선까지
    };
    const Line lines[] = {{online ? hud.network_ssid : "Not connected", kHudBodyFont, online ? kText : kTextSecondary, 25},
                          {online ? hud.network_ipv4 : "", kHudBodyFont, kText, 23},
                          {online ? hud.network_interface : "", kHudCaptionFont, kTextSecondary, 19},
                          {wired.c_str(), kHudCaptionFont, kTextSecondary, 19}};
    const int header_baseline = kCardPad + cap_glyph(kHudCaptionFont).h;
    int w = kHudCaptionFont.width(title) + 2 * kGap + kHudCaptionFont.width(status);
    int h = header_baseline + kCardPad;
    for (const Line &line : lines) {
        if (line.text[0] == '\0') continue;
        w = std::max(w, line.font.width(line.text));
        h += line.advance;
    }
    w = std::max(w + 2 * kCardPad, kNetworkCardMinW);
    const int x = canvas.width() - kMargin - w;
    canvas.fill_round_rect(x, y, w, h, kRadius, kCardStrong);
    int baseline = y + header_baseline;
    canvas.text(x + kCardPad, base_line(kHudCaptionFont, baseline), title, kHudCaptionFont, kTextSecondary);
    canvas.text(x + w - kCardPad, base_line(kHudCaptionFont, baseline), status, kHudCaptionFont,
                weak ? kAmber : kTextSecondary, HudAlign::right);
    for (const Line &line : lines) {
        if (line.text[0] == '\0') continue;
        baseline += line.advance;
        canvas.text(x + kCardPad, base_line(line.font, baseline), line.text, line.font, line.color);
    }
    return y + h + kGap;
}

/* 오른쪽 위: 상태 알약, 열려 있으면 네트워크 카드, 그 아래 문제 칩(panda, 저장 공간, 레이더 앞차)과
 * 신호 대기 신호등. CPU 온도는 보드 상태 카드가 색으로 알린다. */
void draw_status(OverlayCanvas &canvas, const OverlayHudState &hud, const LeadInfo &lead)
{
    const int right = canvas.width() - kMargin;
    int y = draw_status_pill(canvas, hud);
    if (hud.network_card) y = draw_network_card(canvas, y, hud);
    auto warn = [&](const std::string &text, uint32_t color) {
        chip(canvas, right, y, text, color, HudAlign::right);
        y += kChipH + kGap;
    };
    if (!hud.panda_connected || !hud.panda_healthy) warn("PANDA", kAmber);
    if (hud.storage_full) warn("STORAGE FULL", kAmber);
    if (lead.radar && !lead.vision) warn(format_text("LEAD %s", lead_text(lead).c_str()), kText);
    if (hud.green_light_alert_armed) draw_traffic_light(canvas, right, y, kChipH);
}

/* 오른쪽 열 카드(TPMS, 카메라 보정, 학습값)의 틀: 머리글 왼쪽에 이름, 오른쪽에 단위나 상태. */
void corner_card(OverlayCanvas &canvas, int x, int y, int h, const char *title, const std::string &status,
                 uint32_t status_color)
{
    canvas.fill_round_rect(x, y, kCornerCardW, h, kRadius, kCard);
    const int line = cap_line(kHudCaptionFont, y + kCardPad);
    canvas.text(x + kCardPad, line, title, kHudCaptionFont, kTextSecondary);
    canvas.text(x + kCornerCardW - kCardPad, line, status, kHudCaptionFont, status_color, HudAlign::right);
}

// 카드 아래쪽에 쌓는 줄: 왼쪽에 작은 이름, 오른쪽 정렬한 값.
struct CardRow {
    const char *name;
    std::string value;
    uint32_t color;
};

/* 머리글 아래 줄 rows개를 담는 카드 높이: 머리글 대문자, 줄 사이와 같은 9 px 틈, 값 대문자,
 * 그다음 줄마다 kRowH. 줄 셋이면 모서리 카드 높이와 같다. */
constexpr int rows_card_h(int rows)
{
    return 2 * kCardPad + 32 + (rows - 1) * kRowH;
}
static_assert(rows_card_h(3) == kCornerCardH, "CAL rows fill the corner card");

// 아래가 bottom인 카드에 줄을 아래부터 맞춰 쓴다.
template <size_t N>
void card_rows(OverlayCanvas &canvas, int x, int bottom, const CardRow (&rows)[N])
{
    for (size_t i = 0; i < N; ++i) {
        const int baseline = bottom - kCardPad - static_cast<int>(N - 1 - i) * kRowH;
        canvas.text(x + kCardPad, base_line(kHudCaptionFont, baseline), rows[i].name, kHudCaptionFont,
                    kTextSecondary);
        canvas.text(x + kCornerCardW - kCardPad, base_line(kHudBodyFont, baseline), rows[i].value, kHudBodyFont,
                    rows[i].color, HudAlign::right);
    }
}

/* 왼쪽 아래 TPMS: 위에서 본 차의 바퀴 넷과 그 옆 공기압. 낮으면 주황, 높으면 빨강이고, 차가
 * TPMS 경고를 내면 머리글도 빨강. 값이 없으면 "--". */
void draw_tpms(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    const int x = kMargin, y = canvas.height() - kMargin - kCornerCardH;
    const bool bar = hud.tpms_unit == 2;
    corner_card(canvas, x, y, kCornerCardH, "TPMS", bar ? "bar" : "psi", hud.tpms_warning ? kRed : kTextSecondary);

    // 정수 좌표라 둥근 사각형이 곧은 행 지름길을 탄다.
    constexpr int kBodyW = 26, kBodyH = 50, kTireW = 6, kTireH = 13, kTireInset = 8;
    const int body_x = x + (kCornerCardW - kBodyW) / 2, body_y = y + kCornerCardH - kCardPad - kBodyH;
    canvas.fill_round_rect(body_x, body_y, kBodyW, kBodyH, 9, kCarBody);
    canvas.fill_round_rect(body_x + 4, body_y + 11, kBodyW - 8, 9, 2, kShadow);  // 앞유리

    const float pressures[] = {hud.tpms_pressure_fl, hud.tpms_pressure_fr, hud.tpms_pressure_rl,
                               hud.tpms_pressure_rr};
    for (int i = 0; i < 4; ++i) {
        const bool right = i % 2 != 0, rear = i >= 2;
        const float pressure = hud.tpms_valid ? pressures[i] : 0.0f;
        const uint32_t color = tire_color(pressure, bar);
        const int tire_x = right ? body_x + kBodyW - 1 : body_x - kTireW + 1;
        const int tire_y = rear ? body_y + kBodyH - kTireInset - kTireH : body_y + kTireInset;
        canvas.fill_round_rect(tire_x, tire_y, kTireW, kTireH, 2, color ? color : kTrack);
        canvas.text(right ? tire_x + kTireW + 6 : tire_x - 6, centered_line_top(kHudBodyFont, tire_y, kTireH),
                    color ? format_text(bar ? "%.1f" : "%.0f", pressure) : "--", kHudBodyFont,
                    color ? color : kTextSecondary, right ? HudAlign::left : HudAlign::right);
    }
}

/* 오른쪽 아래 카메라 보정: 머리글에 상태(보정 중이면 진행률, 다시 보정 중이면 이름이 RECAL),
 * 아래에 roll·pitch·yaw. */
void draw_calibration(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    const int x = canvas.width() - kMargin - kCornerCardW, y = canvas.height() - kMargin - kCornerCardH;
    const int percent = std::clamp(hud.calibration_valid_blocks * 100 / OnlineCalibrator::kInputsNeeded, 0, 100);
    const char *title = "CAL";
    std::string status = "--";
    uint32_t color = kTextSecondary;
    if (hud.calibration_available) {
        switch (static_cast<CalibrationStatus>(hud.calibration_status)) {
        case CalibrationStatus::Calibrated: status = "OK"; break;
        case CalibrationStatus::Invalid: status = "INVALID"; color = kRed; break;
        case CalibrationStatus::Recalibrating:
            title = "RECAL";
            status = format_text("%d%%", percent);
            color = kAmber;
            break;
        default: status = format_text("%d%%", percent); color = kAmber; break;
        }
    }
    corner_card(canvas, x, y, kCornerCardH, title, status, color);
    auto angle = [&](float deg) { return hud.calibration_available ? format_text("%.2f\xb0", deg) : "--"; };
    const uint32_t value_color = hud.calibration_available ? kText : kTextSecondary;
    const CardRow rows[] = {{"ROLL", angle(hud.calibration_roll_deg), value_color},
                            {"PITCH", angle(hud.calibration_pitch_deg), value_color},
                            {"YAW", angle(hud.calibration_yaw_deg), value_color}};
    card_rows(canvas, x, y + kCornerCardH, rows);
}

/* 카메라 보정 위: 주행에 쓰는 학습값(조향비, 조향각 영점, 횡가속 토크 계수, 조향 지연). 제어가
 * 지금 그 값을 쓰면 흰색, 아직 쓰지 않으면(제어는 파라미터를 쓴다) 흐리게. 지연은 쓰는 동안
 * 경로에 넣는 값, 아니면 lagd가 추정 중인 값. */
void draw_learned(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    constexpr int kH = rows_card_h(4);
    const int x = canvas.width() - kMargin - kCornerCardW;
    const int y = canvas.height() - kMargin - kCornerCardH - kGap - kH;
    corner_card(canvas, x, y, kH, "LEARNED", "", kTextSecondary);
    const bool fresh = hud.learner_fresh;
    auto value = [&](const char *format, float v) { return fresh ? format_text(format, v) : std::string("--"); };
    auto color = [&](bool used) { return fresh && used ? kText : kTextSecondary; };
    std::string delay = "--";
    if (hud.delay_learned) delay = format_text("%.2f s", hud.lateral_delay_s);
    else if (hud.lag_blocks > 0) delay = format_text("%.2f s", hud.lag_estimate_s);
    const CardRow rows[] = {{"SR", value("%.2f", hud.steer_ratio), color(hud.vehicle_learned)},
                            {"OFFSET", value("%.2f\xb0", hud.angle_offset_fast_deg), color(hud.vehicle_learned)},
                            {"TORQUE", value("%.2f", hud.torque_factor_filtered), color(hud.torque_learned)},
                            {"DELAY", delay, color(hud.delay_learned)}};
    card_rows(canvas, x, y + kH, rows);
}

/* TPMS 위: 보드 상태(CPU 온도, CPU 사용률, 메모리, 저장 공간). 오른쪽 학습 카드와 짝이다. 온도는
 * 70°C부터 주황·80°C부터 빨강, 나머지는 90%부터(저장 공간은 녹화를 멈췄으면 바로) 주황. */
void draw_system(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    constexpr int kH = rows_card_h(4);
    const int x = kMargin, y = canvas.height() - kMargin - kCornerCardH - kGap - kH;
    corner_card(canvas, x, y, kH, "SYSTEM", "", kTextSecondary);
    auto usage = [](float percent, bool warn) { return warn || percent >= kHighUsagePercent ? kAmber : kText; };
    const bool temp_known = hud.cpu_temp_c > 0.0f;
    const uint32_t temp_color = !temp_known ? kTextSecondary
                              : hud.cpu_temp_c >= kHotTempC ? kRed
                              : hud.cpu_temp_c >= kWarmTempC ? kAmber : kText;
    const CardRow rows[] = {
        {"TEMP", temp_known ? format_text("%.0f\xb0" "C", hud.cpu_temp_c) : "--", temp_color},
        {"CPU", format_text("%.0f%%", hud.cpu_percent), usage(hud.cpu_percent, false)},
        {"RAM", format_text("%.0f%%", hud.memory_percent), usage(hud.memory_percent, false)},
        {"DISK", format_text("%.0f%%", hud.storage_percent), usage(hud.storage_percent, hud.storage_full)}};
    card_rows(canvas, x, y + kH, rows);
}

/* 아래 가장자리 토크 바: 보낸 토크를 가운데에서 회전 쪽으로 채우고, 운전자 토크를 같은 눈금에
 * 하늘색 세로 눈금으로 얹는다. 시스템과 운전자가 서로 미는지 바로 보인다. + = 왼쪽. */
void draw_torque_bar(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    if (!hud.controller_engaged) return;
    constexpr float kDriverTickMin = 0.05f;  // 이보다 작은 운전자 토크(손을 얹은 정도)는 그리지 않는다
    const float cx = canvas.width() / 2.0f, half = kTorqueBarW / 2.0f;
    const float y = static_cast<float>(canvas.height() - kMargin - kTorqueBarH);
    const bool steering = steering_now(hud);
    canvas.fill_round_rect(cx - half, y, kTorqueBarW, kTorqueBarH, kTorqueBarH / 2.0f,
                           steering ? kTrack : hud_fade(kTrack, 0.5f));
    const float fraction = steering ? std::clamp(hud.steer_torque_fraction, -1.0f, 1.0f) : 0.0f;
    const float length = std::fabs(fraction) * half;
    if (length >= 1.0f) {
        // openpilot mici 토크 바처럼 75%를 넘으면 흰색에서 주황으로. + = 왼쪽 조향은 왼쪽으로 찬다.
        const uint32_t color = mix(kText, kAmber, (std::fabs(fraction) - 0.75f) * 4.0f);
        canvas.fill_round_rect(fraction > 0.0f ? cx - length : cx, y, length, kTorqueBarH, kTorqueBarH / 2.0f, color);
    }
    const float driver = std::clamp(hud.driver_torque_fraction, -1.0f, 1.0f);
    if (std::fabs(driver) >= kDriverTickMin)
        canvas.fill_round_rect(cx - driver * half - 1.5f, y - 5.0f, 3.0f, kTorqueBarH + 10.0f, 1.5f, kDriverTorque);
}

// 아래 가운데 알림 카드(hud_select_alert). 아래 모서리 카드 사이에 들어간다.
void draw_alert(OverlayCanvas &canvas, const HudAlert &alert)
{
    if (alert.empty()) return;
    const int max_w = canvas.width() - 2 * (kMargin + kCornerCardW + kGap);
    const int text_w = std::max(kHudTitleFont.width(alert.title), kHudBodyFont.width(alert.detail));
    const int w = std::clamp(text_w + 56, kAlertMinW, max_w);
    const int x = (canvas.width() - w) / 2, center = canvas.width() / 2;
    const int y = canvas.height() - kMargin - kTorqueBarH - kGap - kAlertH;
    constexpr int kInset = 16;  // 제목 윗선·설명 기준선과 색 막대 끝
    canvas.fill_round_rect(x, y, w, kAlertH, kRadius + 4, kCardStrong);
    canvas.fill_round_rect(x + 12, y + kInset, 4, kAlertH - 2 * kInset, 2.0f, alert_color(alert.level));
    const int title_w = canvas.text(center, cap_line(kHudTitleFont, y + kInset), alert.title, kHudTitleFont, kText,
                                    HudAlign::center);
    if (alert.arrow)
        draw_arrow(canvas, center + alert.arrow * (title_w / 2.0f + 18.0f),
                   y + kInset + cap_glyph(kHudTitleFont).h / 2.0f, 14.0f, 16.0f, alert.arrow, kText);
    canvas.text(center, base_line(kHudBodyFont, y + kAlertH - kInset), alert.detail, kHudBodyFont, kTextSecondary,
                HudAlign::center);
}

// 왼쪽 열: 조향 중에 하고 있는 조작(차선 변경, 교차로 회전). 다음 칩이 올 y를 돌려준다.
int draw_maneuver(OverlayCanvas &canvas, int y, const OverlayHudState &hud)
{
    if (!steering_now(hud)) return y;
    const char *text = nullptr;
    int direction = 0;
    if (hud.lane_change == 2) {
        text = "CHANGING LANES";
        direction = hud.lane_change_direction;
    } else if (hud.turn_direction) {
        text = hud.turn_direction < 0 ? "TURN LEFT" : "TURN RIGHT";
        direction = hud.turn_direction;
    }
    if (!text) return y;
    chip(canvas, kMargin, y, text, state_color(hud), HudAlign::left, direction < 0 ? ChipMark::left : ChipMark::right);
    return y + kChipH + kGap;
}

/* 웹 기기 설정의 HUD 진단: 다른 카드에 없는 수치만 모은다. 처리 속도, 조향 토크, 학습기 상세
 * (paramsd 강성과 평균 영점, torqued 원시 추정과 진행률, lagd 블록). 보드 상태·기어·크루즈·연결·
 * 제어가 쓰는 학습값·보정·TPMS·네트워크는 각자 카드나 칩에 있다. 아직 유효하지 않은 학습기 줄은
 * 주황. */
void draw_debug_card(OverlayCanvas &canvas, int y, const OverlayHudState &hud)
{
    struct Line {
        std::string text;
        uint32_t color = kTextSecondary;
    };
    const int lag_needed = LateralLagConfig{}.min_valid_block_count;
    const bool learner = hud.learner_fresh;
    const Line lines[] = {
        {format_text("AI %.1f  CAM %.1f  HUD %.1f FPS", hud.model_fps, hud.preview_fps, hud.overlay_fps)},
        {format_text("ANGLE %.0f  DES %d  APPLY %d  DRV %d", hud.steering_angle_deg, hud.desired_torque,
                     hud.apply_torque, hud.driver_torque)},
        learner ? Line{format_text("STIFF %.2f  AVG OFFSET %.2f\xb0", hud.stiffness, hud.angle_offset_deg),
                       hud.params_valid ? kTextSecondary : kAmber}
                : Line{"STIFF --  AVG OFFSET --"},
        learner ? Line{format_text("TORQUE RAW %.2f  FRIC %.2f  OFS %.2f  %d%%", hud.torque_factor,
                                   hud.torque_friction, hud.torque_offset, hud.torque_cal_percent),
                       hud.torque_valid ? kTextSecondary : kAmber}
                : Line{"TORQUE RAW --"},
        {hud.lag_blocks >= 0 ? format_text("LAGD %d/%d BLOCKS", std::min(hud.lag_blocks, lag_needed), lag_needed)
                             : std::string("LAGD --"),
         hud.lag_blocks >= 0 && hud.lag_blocks < lag_needed ? kAmber : kTextSecondary},
    };
    constexpr int kLineH = 20;
    const int count = static_cast<int>(std::size(lines));
    int w = 0;
    for (const Line &line : lines) w = std::max(w, kHudCaptionFont.width(line.text));
    // 첫 줄 대문자 윗선과 끝 줄 대문자 아랫선에서 카드 끝까지 kCardPad
    const int h = 2 * kCardPad + (count - 1) * kLineH + cap_glyph(kHudCaptionFont).h;
    canvas.fill_round_rect(kMargin, y, w + 2 * kCardPad, h, kRadius, kCard);
    for (int i = 0; i < count; ++i)
        canvas.text(kMargin + kCardPad, cap_line(kHudCaptionFont, y + kCardPad + i * kLineH), lines[i].text,
                    kHudCaptionFont, lines[i].color);
}

}  // namespace

void OverlayRenderer::draw(const OverlayTarget &target, const ParsedModelOutput &output,
                           const ProjectionState &projection, const OverlayHudState &hud)
{
    const uint32_t buffer_h = target.orientation.transpose ? target.width : target.height;
    BufferDamage *damage = nullptr;
    for (BufferDamage &known : damage_)
        if (known.map == target.map && known.width == target.width && known.rows.size() == buffer_h)
            damage = &known;
    const bool known = damage != nullptr;
    if (!known) {
        damage = &damage_[next_slot_++ % damage_.size()];
        *damage = BufferDamage{target.map, target.width, std::vector<uint16_t>(buffer_h, 0)};
    }
    OverlayCanvas canvas(target.map, static_cast<int>(target.width), static_cast<int>(target.height),
                         static_cast<int>(target.stride), coverage_, damage->rows, target.orientation);
    canvas.clear(known);
    last_damage_ = &damage->rows;
    last_tile_shift_ = canvas.tile_shift();
    const LeadInfo lead = lead_info(hud, output);
    draw_scene(canvas, output, projection, hud, lead);
    draw_lane_position(canvas, output, projection, hud.lane_center_offset_m,
                       steering_now(hud) ? state_color(hud) : kText);
    draw_border(canvas, state_color(hud));
    draw_turn_signals(canvas, hud, draw_speed(canvas, hud));
    draw_brake_hold(canvas, hud);
    const int left_y = draw_maneuver(canvas, draw_cruise(canvas, hud), hud);
    draw_system(canvas, hud);
    // 진단 카드는 보드 상태 카드 뒤에: 조작 칩까지 있어 길어지면 위에 겹쳐 보인다
    if (hud.debug_overlay) draw_debug_card(canvas, left_y, hud);
    // 학습 카드는 오른쪽 열 칩보다 먼저: 칩이 많아 겹치면 경고가 위에 보인다
    draw_learned(canvas, hud);
    draw_status(canvas, hud, lead);
    draw_tpms(canvas, hud);
    draw_calibration(canvas, hud);
    draw_torque_bar(canvas, hud);
    draw_alert(canvas, hud_select_alert(hud, output.valid));
}

bool hud_status_touch(int x, int y, int width)
{
    return x >= width - kStatusTouchW && y < kStatusTouchH;
}

bool hud_left_column_touch(int x, int y, int height)
{
    return x < kLeftColumnTouchW && y < height - kMargin - kCornerCardH;
}
