#include "hud/hud_draw.h"

#include "model/calibration_online.h"
#include "localization/lateral_lag.h"

#include <iterator>

/* HUD의 카드: 아래 두 모서리의 TPMS와 카메라 보정, 그 위 보드 상태와 학습값(늘 있다), 상태 알약을
 * 눌러 여는 네트워크 카드, 웹 기기 설정의 HUD 진단 카드. */

namespace hud_draw {
namespace {

constexpr float kWarmTempC = 70.0f;
constexpr float kHotTempC = 80.0f;
constexpr float kHighUsagePercent = 90.0f;  // 보드 상태 카드: CPU·메모리·저장 공간이 이만큼 차면 주황
constexpr float kTpmsLowBar = 2.2f;
constexpr float kTpmsHighBar = 2.8f;
constexpr float kTpmsLowPsi = 32.0f;
constexpr float kTpmsHighPsi = 45.0f;

// 공기압 하나의 색: 낮으면 주황, 높으면 빨강, 값이 없으면 0.
uint32_t tire_color(float pressure, bool bar)
{
    if (!std::isfinite(pressure) || pressure <= 0.0f) return 0;
    if (pressure > (bar ? kTpmsHighBar : kTpmsHighPsi)) return kRed;
    return pressure < (bar ? kTpmsLowBar : kTpmsLowPsi) ? kAmber : kText;
}

/* 오른쪽 열 카드(TPMS, 카메라 보정, 학습값)의 틀: 머리글 왼쪽에 이름, 오른쪽에 단위나 상태. */
void corner_card(HudCanvas &canvas, int x, int y, int h, const char *title, const std::string &status,
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
void card_rows(HudCanvas &canvas, int x, int bottom, const CardRow (&rows)[N])
{
    for (size_t i = 0; i < N; ++i) {
        const int baseline = bottom - kCardPad - static_cast<int>(N - 1 - i) * kRowH;
        canvas.text(x + kCardPad, base_line(kHudCaptionFont, baseline), rows[i].name, kHudCaptionFont,
                    kTextSecondary);
        canvas.text(x + kCornerCardW - kCardPad, base_line(kHudBodyFont, baseline), rows[i].value, kHudBodyFont,
                    rows[i].color, HudAlign::right);
    }
}

}  // namespace

/* 상태 알약을 눌러 연 네트워크 카드: 머리글에 신호 세기, 아래에 와이파이 이름, 주소,
 * 인터페이스(끊겼으면 "Not connected"). 와이파이 밖의 링크(USB 가상 이더넷)가 있으면 맨 아래에
 * 작게. 다음 줄 y를 돌려준다. */
int draw_network_card(HudCanvas &canvas, int y, const HudState &hud)
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

/* 왼쪽 아래 TPMS: 위에서 본 차의 바퀴 넷과 그 옆 공기압. 낮으면 주황, 높으면 빨강이고, 차가
 * TPMS 경고를 내면 머리글도 빨강. 값이 없으면 "--". */
void draw_tpms(HudCanvas &canvas, const HudState &hud)
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
void draw_calibration(HudCanvas &canvas, const HudState &hud)
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
void draw_learned(HudCanvas &canvas, const HudState &hud)
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
void draw_system(HudCanvas &canvas, const HudState &hud)
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

/* 웹 기기 설정의 HUD 진단: 다른 카드에 없는 수치만 모은다. 처리 속도, 조향 토크, 학습기 상세
 * (paramsd 강성과 평균 영점, torqued 원시 추정과 진행률, lagd 블록). 보드 상태·기어·크루즈·연결·
 * 제어가 쓰는 학습값·보정·TPMS·네트워크는 각자 카드나 칩에 있다. 아직 유효하지 않은 학습기 줄은
 * 주황. */
void draw_debug_card(HudCanvas &canvas, int y, const HudState &hud)
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

}  // namespace hud_draw
