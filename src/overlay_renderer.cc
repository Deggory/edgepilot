#include "overlay_renderer.h"

#include "calibration_online.h"
#include "overlay_canvas.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <optional>
#include <string>

/* MaixCAM2 HUD(640x480). 주행 화면에는 필요한 것만 둔다: 상태 테두리, 현재·설정 속도, 조향
 * 모드, 경로·차선, 앞차, 토크 바, 알림. 아래 두 모서리에는 TPMS와 카메라 보정을 늘 두고,
 * 오른쪽 위 상태 알약은 녹화와 와이파이를 보이며 누르면 네트워크 카드를 연다. 온도·panda·
 * 저장 공간은 문제가 있을 때만 칩으로 띄우고, 그 밖의 수치 진단은 웹 기기 설정의 HUD 진단을
 * 켜면 왼쪽 카드에 모은다. 색·간격·글꼴 크기는 아래 토큰 한 곳에서 정한다. */

namespace {

// ---- 디자인 토큰 ----

constexpr uint32_t kText = hud_argb(255, 255, 255, 255);
constexpr uint32_t kTextSecondary = hud_argb(235, 190, 196, 205);
constexpr uint32_t kCard = hud_argb(150, 12, 14, 18);
constexpr uint32_t kCardStrong = hud_argb(205, 12, 14, 18);
constexpr uint32_t kTrack = hud_argb(90, 255, 255, 255);
constexpr uint32_t kShadow = hud_argb(110, 0, 0, 0);
constexpr uint32_t kCarBody = hud_argb(70, 255, 255, 255);
constexpr uint32_t kGreen = hud_argb(255, 48, 209, 88);
constexpr uint32_t kBlue = hud_argb(255, 64, 156, 255);
constexpr uint32_t kGray = hud_argb(255, 150, 156, 165);
constexpr uint32_t kAmber = hud_argb(255, 255, 176, 32);
constexpr uint32_t kRed = hud_argb(255, 255, 69, 58);

constexpr int kMargin = 14;
constexpr int kGap = 8;
constexpr float kRadius = 10.0f;
constexpr int kBorder = 4;
constexpr int kCardPad = 10;
constexpr int kChipH = 26;
constexpr int kChipPadX = 10;
constexpr int kDot = 8;
constexpr int kSetCardW = 84;
constexpr int kSetCardH = 70;
constexpr int kNetworkCardMinW = 180;
constexpr int kTorqueBarW = 220;
constexpr int kTorqueBarH = 6;
constexpr int kAlertH = 68;
constexpr int kAlertMinW = 340;
// 아래 모서리 카드(TPMS, 카메라 보정). 위는 알림 카드, 아래는 토크 바와 맞춘다.
constexpr int kCornerCardW = 116;
constexpr int kCornerCardH = kAlertH + kGap + kTorqueBarH;
constexpr int kStatusTouchW = 170;  // 상태 알약을 누른 것으로 보는 오른쪽 위 영역(손가락만큼 넉넉하게)
constexpr int kStatusTouchH = 70;

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

// 화면 테두리와 모드 점의 색. 결합 전이면 0(테두리 없음).
uint32_t state_color(const OverlayHudState &hud)
{
    if (hud.steering_fault || hud.panda_faults != 0 || hud.soft_disabling) return kRed;
    if (!hud.services_healthy) return kAmber;
    if (hud.controller_active) return hud.laneless_mode ? kBlue : kGreen;
    return hud.controller_engaged ? kGray : 0;
}

std::string mode_text(const OverlayHudState &hud)
{
    if (hud.controller_active)
        return hud.lateral_mode_available && hud.laneless_mode ? "LANELESS" : "LANE";
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
    case 0: return "P";
    case 5: return "D";
    case 6: return "N";
    case 7: return "R";
    case 8: return "S";
    default: return "-";
    }
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

    bool any() const { return radar || vision; }
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

// ---- 알림: 우선순위는 engage 거부 > 해제 예고 > 조향 결함 > panda 결함 > 조향 한계 >
// 서비스 대기 > 출발 감지 ----

struct Alert {
    std::string title;
    std::string detail;
    uint32_t color = 0;

    bool empty() const { return title.empty(); }
};

Alert select_alert(const OverlayHudState &hud, bool model_ok)
{
    const std::string links = format_text("MODEL %s    CONTROL %s    PANDA %s", model_ok ? "OK" : "--",
                                          hud.vehicle_fresh ? "OK" : "--",
                                          hud.panda_connected ? "OK" : "--");
    if (hud.engage_reject_label[0] != '\0') return {"UNABLE TO ENGAGE", hud.engage_reject_label, kAmber};
    if (hud.soft_disabling) {
        const char *label = engage_block_label(hud.active_block);
        return {"TAKE CONTROL", label ? label : "Disengaging", kRed};
    }
    if (hud.steering_fault) return {"STEERING FAULT", links, kRed};
    if (hud.panda_faults != 0) return {"PANDA FAULT", links, kRed};
    if (hud.steer_saturated) return {"TAKE CONTROL", "Turn exceeds steering limit", kAmber};
    if (!hud.services_healthy) return {"WAITING FOR SERVICES", links, kAmber};
    if (hud.departure_alert_type == DepartureAlertType::lead_departed)
        return {"LEAD VEHICLE MOVING", "Check the road and proceed", kGreen};
    if (hud.departure_alert_type == DepartureAlertType::green_light)
        return {"GREEN LIGHT", "Check the road and proceed", kGreen};
    return {};
}

// ---- 공통 부품 ----

// 둥근 칩: 색 점(선택)과 글. x는 align 기준점. 그린 폭을 돌려준다.
int chip(OverlayCanvas &canvas, int x, int y, const std::string &text, uint32_t dot,
         HudAlign align = HudAlign::left, uint32_t fill = kCard)
{
    const int dot_w = dot ? kDot + 6 : 0;
    const int w = 2 * kChipPadX + dot_w + kHudBodyFont.width(text);
    const int left = align == HudAlign::right ? x - w : align == HudAlign::center ? x - w / 2 : x;
    canvas.fill_round_rect(left, y, w, kChipH, kChipH / 2.0f, fill);
    if (dot) canvas.fill_round_rect(left + kChipPadX, y + (kChipH - kDot) / 2, kDot, kDot, kDot / 2.0f, dot);
    canvas.text(left + kChipPadX + dot_w, centered_line_top(kHudBodyFont, y, kChipH), text, kHudBodyFont, kText);
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
    if (!hud.controller_active) return hud_fade(kGray, 0.65f);
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

// 가운데 현재 속도와 그 양옆 깜빡이 화살표. 화살표 기준으로 쓸 숫자 폭을 돌려준다.
int draw_speed(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    const std::string speed = format_text("%.0f", std::max(0.0f, hud.cluster_speed_kph));
    const int center = canvas.width() / 2;
    const HudGlyph *digit = kHudSpeedFont.glyph('0');
    const int top = kMargin - digit->top;
    const int width = canvas.text(center, top, speed, kHudSpeedFont, kText, HudAlign::center, true);
    canvas.text(center, kMargin + digit->h + 4, "km/h", kHudCaptionFont, kTextSecondary, HudAlign::center, true);
    return width;
}

void draw_turn_signals(OverlayCanvas &canvas, const OverlayHudState &hud, int speed_width)
{
    if (!hud.left_blinker && !hud.right_blinker) return;
    const int step = std::clamp(hud.turn_signal_step, 0, kTurnSignalSteps - 1);
    if (step >= kTurnLitSteps) return;
    const float center_y = kMargin + kHudSpeedFont.glyph('0')->h / 2.0f;
    constexpr float kHalfH = 14.0f, kWidth = 14.0f, kThickness = 7.0f, kStep = 13.0f;

    auto side = [&](bool active, float direction) {
        if (!active) return;
        const float inner = canvas.width() / 2.0f + direction * (speed_width / 2.0f + 18.0f);
        for (int i = 0; i < 3 && step >= kTurnChevronStartStep[i]; ++i) {
            const float x = inner + direction * i * kStep;
            const HudPoint points[] = {{x, center_y - kHalfH},
                                       {x + direction * kThickness, center_y - kHalfH},
                                       {x + direction * (kThickness + kWidth), center_y},
                                       {x + direction * kThickness, center_y + kHalfH},
                                       {x, center_y + kHalfH},
                                       {x + direction * kWidth, center_y}};
            canvas.fill_polygon(points, 6, hud_fade(kGreen, kTurnChevronAlpha[i]));
        }
    };
    side(hud.left_blinker, -1.0f);
    side(hud.right_blinker, 1.0f);
}

/* 왼쪽 위: 설정 속도 카드와 조향 모드 칩. 비전 크루즈가 설정보다 낮게 잡고 있으면 그 속도를
 * 카드 아래에 작게. 다음 칩이 올 y를 돌려준다. */
int draw_cruise(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    const int x = kMargin, y = kMargin;
    canvas.fill_round_rect(x, y, kSetCardW, kSetCardH, kRadius, kCard);
    canvas.text(x + kSetCardW / 2, y + 7, "MAX", kHudCaptionFont, kTextSecondary, HudAlign::center);
    const bool max_valid = speed_valid(hud.cruise_max_speed_kph);
    canvas.text(x + kSetCardW / 2, centered_line_top(kHudValueFont, y + 22, kSetCardH - 28),
                max_valid ? format_text("%.0f", hud.cruise_max_speed_kph) : "-",
                kHudValueFont, max_valid && hud.cruise_active ? kText : kTextSecondary, HudAlign::center);

    int next_y = y + kSetCardH + kGap;
    chip(canvas, x, next_y, mode_text(hud), state_color(hud) ? state_color(hud) : kGray);
    next_y += kChipH + kGap;
    if (max_valid && speed_valid(hud.cruise_command_speed_kph) &&
        hud.cruise_command_speed_kph < hud.cruise_max_speed_kph - 2.0f) {
        chip(canvas, x, next_y, format_text("SET %.0f", hud.cruise_command_speed_kph), kAmber);
        next_y += kChipH + kGap;
    }
    return next_y;
}

// 와이파이 막대 넷(left, bottom 기준). 약하면 주황. 무선이 아니면(dbm 0) 다 켠다.
void draw_signal_bars(OverlayCanvas &canvas, float left, float bottom, int dbm)
{
    const int level = dbm == 0 || dbm >= -55 ? 4 : dbm >= -65 ? 3 : dbm > kWeakWifiDbm ? 2 : 1;
    const uint32_t lit = level <= 1 ? kAmber : kText;
    for (int i = 0; i < 4; ++i) {
        const float h = 5.0f + 3.0f * i;
        canvas.fill_round_rect(left + 6.0f * i, bottom - h, 4.0f, h, 2.0f, i < level ? lit : kTrack);
    }
}

/* 오른쪽 위 상태 알약: 녹화 중이면 빨간 점과 REC, 그리고 와이파이 막대(끊기면 OFFLINE).
 * 누르면 네트워크 카드가 열린다(hud_status_touch). 다음 줄 y를 돌려준다. */
int draw_status_pill(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    constexpr int kBarsW = 22, kItemGap = 12;
    const int y = kMargin;
    const int rec_w = hud.recording ? kDot + 6 + kHudBodyFont.width("REC") + kItemGap : 0;
    const int network_w = hud.network_connected ? kBarsW : kHudBodyFont.width("OFFLINE");
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
    if (hud.network_connected) draw_signal_bars(canvas, x, y + 20.0f, hud.wifi_signal_dbm);
    else canvas.text(x, line, "OFFLINE", kHudBodyFont, kAmber);
    return y + kChipH + kGap;
}

/* 상태 알약을 눌러 연 네트워크 카드: 머리글에 신호 세기, 아래에 와이파이 이름, 주소,
 * 인터페이스. 다음 줄 y를 돌려준다. */
int draw_network_card(OverlayCanvas &canvas, int y, const OverlayHudState &hud)
{
    const bool wifi = std::strncmp(hud.network_interface, "wlan", 4) == 0;
    const char *title = wifi ? "WI-FI" : "NETWORK";
    const std::string status = !hud.network_connected ? "OFFLINE"
                             : wifi ? format_text("%d dBm", hud.wifi_signal_dbm) : "";
    const bool weak = !hud.network_connected || (wifi && hud.wifi_signal_dbm <= kWeakWifiDbm);
    struct Line {
        const char *text;
        const HudFont &font;
        uint32_t color;
        int advance;  // 앞 줄 기준선에서 이 줄 기준선까지
    };
    const Line lines[] = {{hud.network_ssid, kHudBodyFont, kText, 22},
                          {hud.network_ipv4, kHudBodyFont, kText, 20},
                          {hud.network_interface, kHudCaptionFont, kTextSecondary, 17}};
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

/* 오른쪽 위: 상태 알약, 열려 있으면 네트워크 카드, 그 아래 문제 칩(온도, panda, 저장 공간,
 * 레이더 앞차, 신호 대기). */
void draw_status(OverlayCanvas &canvas, const OverlayHudState &hud, const LeadInfo &lead)
{
    const int right = canvas.width() - kMargin;
    int y = draw_status_pill(canvas, hud);
    if (hud.network_card) y = draw_network_card(canvas, y, hud);
    auto warn = [&](const std::string &text, uint32_t color) {
        chip(canvas, right, y, text, color, HudAlign::right);
        y += kChipH + kGap;
    };
    if (hud.cpu_temp_c >= kWarmTempC) warn(format_text("%.0f\xb0" "C", hud.cpu_temp_c), hud.cpu_temp_c >= kHotTempC ? kRed : kAmber);
    if (!hud.panda_connected || !hud.panda_healthy) warn("PANDA", kAmber);
    if (hud.storage_full) warn("STORAGE FULL", kAmber);
    if (lead.radar && !lead.vision) warn(format_text("LEAD %s", lead_text(lead).c_str()), kText);
    if (hud.green_light_alert_armed) warn("WAITING FOR GREEN", kRed);
}

/* 아래 모서리 카드(TPMS, 카메라 보정)의 틀: 머리글 왼쪽에 이름, 오른쪽에 단위나 상태. */
void corner_card(OverlayCanvas &canvas, int x, int y, const char *title, const std::string &status,
                 uint32_t status_color)
{
    canvas.fill_round_rect(x, y, kCornerCardW, kCornerCardH, kRadius, kCard);
    const int line = cap_line(kHudCaptionFont, y + kCardPad);
    canvas.text(x + kCardPad, line, title, kHudCaptionFont, kTextSecondary);
    canvas.text(x + kCornerCardW - kCardPad, line, status, kHudCaptionFont, status_color, HudAlign::right);
}

/* 왼쪽 아래 TPMS: 위에서 본 차의 바퀴 넷과 그 옆 공기압. 낮으면 주황, 높으면 빨강이고, 차가
 * TPMS 경고를 내면 머리글도 빨강. 값이 없으면 "--". */
void draw_tpms(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    const int x = kMargin, y = canvas.height() - kMargin - kCornerCardH;
    const bool bar = hud.tpms_unit == 2;
    corner_card(canvas, x, y, "TPMS", bar ? "bar" : "psi", hud.tpms_warning ? kRed : kTextSecondary);

    // 정수 좌표라 둥근 사각형이 곧은 행 지름길을 탄다.
    constexpr int kBodyW = 24, kBodyH = 46, kTireW = 5, kTireH = 12, kTireInset = 7;
    const int body_x = x + (kCornerCardW - kBodyW) / 2, body_y = y + kCornerCardH - kCardPad - kBodyH;
    canvas.fill_round_rect(body_x, body_y, kBodyW, kBodyH, 8, kCarBody);
    canvas.fill_round_rect(body_x + 4, body_y + 10, kBodyW - 8, 8, 2, kShadow);  // 앞유리

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

/* 오른쪽 아래 카메라 보정: 머리글에 상태(보정 중이면 진행률), 아래에 roll·pitch·yaw. */
void draw_calibration(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    const int x = canvas.width() - kMargin - kCornerCardW, y = canvas.height() - kMargin - kCornerCardH;
    const int percent = std::clamp(hud.calibration_valid_blocks * 100 / OnlineCalibrator::kInputsNeeded, 0, 100);
    std::string status = "--";
    uint32_t color = kTextSecondary;
    if (hud.calibration_available) {
        switch (static_cast<CalibrationStatus>(hud.calibration_status)) {
        case CalibrationStatus::Calibrated: status = "OK"; break;
        case CalibrationStatus::Invalid: status = "INVALID"; color = kRed; break;
        case CalibrationStatus::Recalibrating: status = format_text("RECAL %d%%", percent); color = kAmber; break;
        default: status = format_text("%d%%", percent); color = kAmber; break;
        }
    }
    corner_card(canvas, x, y, "CAL", status, color);

    constexpr int kRowH = 17;
    const struct { const char *name; float deg; } rows[] = {
        {"ROLL", hud.calibration_roll_deg}, {"PITCH", hud.calibration_pitch_deg}, {"YAW", hud.calibration_yaw_deg}};
    for (int i = 0; i < 3; ++i) {
        const int baseline = y + kCornerCardH - kCardPad - (2 - i) * kRowH;
        canvas.text(x + kCardPad, base_line(kHudCaptionFont, baseline), rows[i].name, kHudCaptionFont,
                    kTextSecondary);
        canvas.text(x + kCornerCardW - kCardPad, base_line(kHudBodyFont, baseline),
                    hud.calibration_available ? format_text("%.2f\xb0", rows[i].deg) : "--", kHudBodyFont,
                    hud.calibration_available ? kText : kTextSecondary, HudAlign::right);
    }
}

void draw_torque_bar(OverlayCanvas &canvas, const OverlayHudState &hud)
{
    if (!hud.controller_engaged) return;
    const float cx = canvas.width() / 2.0f;
    const float y = static_cast<float>(canvas.height() - kMargin - kTorqueBarH);
    canvas.fill_round_rect(cx - kTorqueBarW / 2.0f, y, kTorqueBarW, kTorqueBarH, kTorqueBarH / 2.0f,
                           hud.controller_active ? kTrack : hud_fade(kTrack, 0.5f));
    const float fraction = hud.controller_active ? std::clamp(hud.steer_torque_fraction, -1.0f, 1.0f) : 0.0f;
    const float length = std::fabs(fraction) * kTorqueBarW / 2.0f;
    if (length < 1.0f) return;
    // openpilot mici 토크 바처럼 75%를 넘으면 흰색에서 주황으로. + = 왼쪽 조향은 왼쪽으로 찬다.
    const uint32_t color = mix(kText, kAmber, (std::fabs(fraction) - 0.75f) * 4.0f);
    canvas.fill_round_rect(fraction > 0.0f ? cx - length : cx, y, length, kTorqueBarH, kTorqueBarH / 2.0f, color);
}

// 아래 가운데 알림 카드. 아래 모서리 카드 사이에 들어간다.
void draw_alert(OverlayCanvas &canvas, const Alert &alert)
{
    if (alert.empty()) return;
    const int max_w = canvas.width() - 2 * (kMargin + kCornerCardW + kGap);
    const int text_w = std::max(kHudTitleFont.width(alert.title), kHudBodyFont.width(alert.detail));
    const int w = std::clamp(text_w + 56, kAlertMinW, max_w);
    const int x = (canvas.width() - w) / 2, center = canvas.width() / 2;
    const int y = canvas.height() - kMargin - kTorqueBarH - kGap - kAlertH;
    canvas.fill_round_rect(x, y, w, kAlertH, kRadius + 4, kCardStrong);
    canvas.fill_round_rect(x + 12, y + 14, 4, kAlertH - 28, 2.0f, alert.color);
    canvas.text(center, cap_line(kHudTitleFont, y + 15), alert.title, kHudTitleFont, kText, HudAlign::center);
    canvas.text(center, base_line(kHudBodyFont, y + kAlertH - 15), alert.detail, kHudBodyFont, kTextSecondary,
                HudAlign::center);
}

// 왼쪽 열의 오토 홀드 칩. 다음 칩이 올 y를 돌려준다.
int draw_brake_hold(OverlayCanvas &canvas, int y, const OverlayHudState &hud)
{
    if (!hud.brake_hold) return y;
    chip(canvas, kMargin, y, "AUTO HOLD", kGreen);
    return y + kChipH + kGap;
}

// 웹 기기 설정의 HUD 진단: 예전 패널의 수치를 한 카드에(보정·TPMS·네트워크는 따로 있다).
void draw_debug_card(OverlayCanvas &canvas, int y, const OverlayHudState &hud)
{
    const std::string lines[] = {
        format_text("AI %.1f  CAM %.1f  HUD %.1f FPS", hud.model_fps, hud.preview_fps, hud.overlay_fps),
        format_text("CPU %.0f%%  %.0f\xb0" "C  MEM %.0f%%  DISK %.0f%%", hud.cpu_percent, hud.cpu_temp_c,
                    hud.memory_percent, hud.storage_percent),
        format_text("ANGLE %.0f  DES %d  APPLY %d  DRV %d", hud.steering_angle_deg, hud.desired_torque,
                    hud.apply_torque, hud.driver_torque),
        format_text("GEAR %s  CRUISE %s  PANDA %s  CAR %s", gear_text(hud.gear), hud.cruise_active ? "ON" : "OFF",
                    hud.panda_connected && hud.panda_healthy ? "OK" : "--", hud.vehicle_fresh ? "OK" : "--"),
    };
    constexpr int kLineH = 17;
    const int count = static_cast<int>(std::size(lines));
    int w = 0;
    for (const std::string &line : lines) w = std::max(w, kHudCaptionFont.width(line));
    canvas.fill_round_rect(kMargin, y, w + 20, count * kLineH + 12, kRadius, kCard);
    for (int i = 0; i < count; ++i)
        canvas.text(kMargin + 10, y + 6 + i * kLineH, lines[i], kHudCaptionFont, kTextSecondary);
}

}  // namespace

void OverlayRenderer::draw(const OverlayTarget &target, const ParsedModelOutput &output,
                           const ProjectionState &projection, const OverlayHudState &hud)
{
    BufferDamage *damage = nullptr;
    for (BufferDamage &known : damage_)
        if (known.map == target.map && known.width == target.width && known.rows.size() == target.height)
            damage = &known;
    const bool known = damage != nullptr;
    if (!known) {
        damage = &damage_[next_slot_++ % damage_.size()];
        *damage = BufferDamage{target.map, target.width, std::vector<uint16_t>(target.height, 0)};
    }
    OverlayCanvas canvas(target.map, static_cast<int>(target.width), static_cast<int>(target.height),
                         static_cast<int>(target.stride), coverage_, damage->rows);
    canvas.clear(known);
    const LeadInfo lead = lead_info(hud, output);
    draw_scene(canvas, output, projection, hud, lead);
    draw_border(canvas, state_color(hud));
    draw_turn_signals(canvas, hud, draw_speed(canvas, hud));
    const int left_y = draw_brake_hold(canvas, draw_cruise(canvas, hud), hud);
    if (hud.debug_overlay) draw_debug_card(canvas, left_y, hud);
    draw_status(canvas, hud, lead);
    draw_tpms(canvas, hud);
    draw_calibration(canvas, hud);
    draw_torque_bar(canvas, hud);
    draw_alert(canvas, select_alert(hud, output.valid));
}

bool hud_status_touch(int x, int y, int width)
{
    return x >= width - kStatusTouchW && y < kStatusTouchH;
}
