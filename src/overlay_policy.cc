#include "overlay_policy.h"

#include "overlay_renderer.h"

#include <cmath>
#include <cstdio>

namespace {

constexpr uint64_t kRejectLabelNs = 3000000000ULL;
constexpr uint64_t kTurnSignalStepNs = 50000000ULL;
constexpr uint64_t kNetworkCardNs = 10000000000ULL;

const char *engage_block_text(const char *block)
{
    if (!block || block[0] == '\0') return "NOT READY";
    const char *label = engage_block_label(block);
    return label ? label : block;
}

static_assert(static_cast<int>(OverlaySound::unable) == static_cast<int>(OverlayAlert::unable) &&
                  static_cast<int>(OverlaySound::signal_changed) == static_cast<int>(OverlayAlert::signal_changed),
              "control event sounds share OverlayAlert's values");

}  // namespace

bool OverlayAlertEvents::baseline(const ControlState &control)
{
    const auto counter_reset = [](uint32_t current, uint32_t previous) {
        return previous != 0 && current < previous;
    };
    const bool counters_reset =
        initialized_ &&
        (counter_reset(control.engage_event_id, last_engage_) ||
         counter_reset(control.disengage_event_id, last_disengage_) ||
         counter_reset(control.engage_reject_event_id, last_reject_) ||
         counter_reset(control.departure_alert_event_id, last_departure_));
    if (initialized_ && !counters_reset) return false;
    last_engage_ = control.engage_event_id;
    last_disengage_ = control.disengage_event_id;
    last_reject_ = control.engage_reject_event_id;
    last_departure_ = control.departure_alert_event_id;
    initialized_ = true;
    return true;
}

OverlayAlertEvents::Decision OverlayAlertEvents::update(const ControlState &control,
                                                        DepartureAlertType departure_type)
{
    Decision decision;
    if (baseline(control)) return decision;
    if (control.engage_reject_event_id != 0 && control.engage_reject_event_id != last_reject_) {
        last_reject_ = control.engage_reject_event_id;
        return {OverlayAlert::unable, last_reject_};
    }
    if (control.engage_event_id != 0 && control.engage_event_id != last_engage_) {
        last_engage_ = control.engage_event_id;
        return {OverlayAlert::engage, last_engage_};
    }
    if (control.disengage_event_id != 0 && control.disengage_event_id != last_disengage_) {
        last_disengage_ = control.disengage_event_id;
        return {OverlayAlert::disengage, last_disengage_};
    }
    if (departure_type != DepartureAlertType::none && control.departure_alert_event_id != 0 &&
        control.departure_alert_event_id != last_departure_) {
        last_departure_ = control.departure_alert_event_id;
        return {OverlayAlert::signal_changed, last_departure_};
    }
    return decision;
}

const char *overlay_sound_name(OverlaySound sound)
{
    switch (sound) {
    case OverlaySound::unable: return "unable";
    case OverlaySound::engage: return "engage";
    case OverlaySound::disengage: return "disengage";
    case OverlaySound::signal_changed: return "signal_changed";
    case OverlaySound::take_control: return "take_control";
    case OverlaySound::unavailable: return "unavailable";
    case OverlaySound::none: break;
    }
    return "none";
}

AlertSoundId overlay_sound_id(OverlaySound sound)
{
    switch (sound) {
    case OverlaySound::unable: return AlertSoundId::unable;
    case OverlaySound::engage: return AlertSoundId::engage;
    case OverlaySound::disengage: return AlertSoundId::disengage;
    case OverlaySound::signal_changed: return AlertSoundId::signal_changed;
    case OverlaySound::take_control: return AlertSoundId::unable;
    case OverlaySound::unavailable: return AlertSoundId::unavailable;
    case OverlaySound::none: break;
    }
    return AlertSoundId::count;
}

OverlaySoundDecision OverlayAlertPolicy::update(const ControlState &control, bool control_fresh,
                                                const PandaState &panda, bool panda_fresh, uint64_t now_ns,
                                                OverlayHudState *hud)
{
    OverlaySoundDecision decision;
    OverlayAlertEvents::Decision event;
    if (control_fresh) event = events_.update(control, hud->departure_alert_type);
    if (event.alert != OverlayAlert::none) {
        decision = {static_cast<OverlaySound>(event.alert), event.event_id};
        if (event.alert == OverlayAlert::unable) {
            std::snprintf(hud->engage_reject_label, sizeof(hud->engage_reject_label), "%s",
                          engage_block_text(control.engage_reject_block));
            reject_label_until_ns_ = now_ns + kRejectLabelNs;
        }
    }
    if (now_ns >= reject_label_until_ns_) hud->engage_reject_label[0] = '\0';

    // 해제 예고·조향 한계: 다른 알림이 울린 프레임에는 이전 값을 두어 다음 프레임에 울린다.
    if (decision.sound == OverlaySound::none) {
        const bool soft = hud->soft_disabling, saturated = hud->steer_saturated;
        const bool rising = (soft && !previous_soft_disabling_) || (saturated && !previous_steer_saturated_);
        previous_soft_disabling_ = soft;
        previous_steer_saturated_ = saturated;
        if (rising) decision.sound = OverlaySound::take_control;
    }

    const bool panda_unavailable =
        panda.timestamp_ns != 0 &&
        (!panda_fresh || !hud->panda_connected || !hud->panda_healthy || panda.faults != 0);
    const bool unavailable = !control_fresh || panda_unavailable || control.steering_fault != 0;
    if (!availability_initialized_) {
        availability_initialized_ = true;
    } else if (unavailable && !previous_unavailable_ && decision.sound == OverlaySound::none) {
        decision.sound = OverlaySound::unavailable;
    }
    previous_unavailable_ = unavailable;
    return decision;
}

bool TurnSignalClock::update(uint64_t now_ns, OverlayHudState *hud)
{
    if (hud->left_blinker != left_ || hud->right_blinker != right_) {
        left_ = hud->left_blinker;
        right_ = hud->right_blinker;
        start_ns_ = now_ns;
    }
    const bool blinking = hud->left_blinker || hud->right_blinker;
    const int step = blinking
        ? static_cast<int>(((now_ns - start_ns_) / kTurnSignalStepNs) % kTurnSignalSteps)
        : 0;
    const bool changed = step != hud->turn_signal_step;
    hud->turn_signal_step = step;
    return changed && blinking;
}

const char *HudTouch::tap(int x, int y, int width, int height, uint64_t now_ns, OverlayHudState *hud)
{
    if (hud_status_touch(x, y, width)) {
        hud->network_card = !hud->network_card;
        network_card_until_ns_ = now_ns + kNetworkCardNs;
        return "network card";
    }
    if (hud_left_column_touch(x, y, height)) {
        hud->debug_overlay = !hud->debug_overlay;
        return "debug card";
    }
    hud->network_card = false;
    return "close";
}

void HudTouch::expire(uint64_t now_ns, OverlayHudState *hud) const
{
    if (now_ns >= network_card_until_ns_) hud->network_card = false;
}

float smooth_lane_center_offset(float smoothed, float raw)
{
    constexpr float kAlpha = 0.1f;
    if (!std::isfinite(raw)) return raw;
    return std::isfinite(smoothed) ? smoothed + kAlpha * (raw - smoothed) : raw;
}
