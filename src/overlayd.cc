#include "alert_sound.h"
#include "app_config.h"
#include "utils_process.h"
#include "utils_time.h"
#include "overlay_policy.h"
#include "overlay_state.h"
#include "ipc_channels.h"
#include "ipc_messages.h"
#include "overlay_renderer.h"
#include "device_settings.h"
#include "projection.h"
#include "system_monitor.h"
#include "maix_display.h"
#include "maix_touch.h"

#include <linux/videodev2.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

/* MaixCAM2 HUD. LCD의 VO 하드웨어 레이어 둘을 쓴다: 카메라 프레임 링의 최신 프레임은
 * 비디오 레이어 0(IVPS로 가운데 4:3을 640x480에), HUD는 렌더러가 640x480 화면 좌표로
 * 세로 패널 방향 그림판에 그려 그래픽 레이어 fb0로 옮기고, 합성은 하드웨어가 한다. 알림은
 * 화면 카드와 로그에 더해 보드 스피커로 알림음을 낸다(alert_sound.h). 터치스크린은 오른쪽
 * 위 상태 알약(네트워크 카드)과 왼쪽 열(진단 카드)을 켜고 끄는 데 쓴다. */

namespace {

volatile sig_atomic_t g_stop = 0;
/* 책상 확인용: SIGUSR1을 받을 때마다 알림음을 차례로 하나씩 낸다(pkill -USR1 overlayd). */
volatile sig_atomic_t g_test_sound_requests = 0;

void on_test_sound_signal(int) { ++g_test_sound_requests; }

constexpr int kOutW = MaixDisplay::kWidth;
constexpr int kOutH = MaixDisplay::kHeight;
constexpr uint64_t kStateFreshNs = 2000000000ULL;
// 오버레이(HUD 그림, CPU)는 상태가 바뀔 때 모델과 같은 20 Hz로 다시 그린다.
// 상한을 50 ms가 아닌 45 ms로 두어 모델 출력 도착이 5 ms 루프만큼 흔들려도 프레임을
// 건너뛰지 않는다. 영상은 하드웨어 레이어라 카메라 프레임마다 올린다.
constexpr uint64_t kOverlayIntervalNs = 45000000ULL;
constexpr useconds_t kPollUs = 5000;

struct StageStats {
    uint64_t total_ns = 0;
    uint32_t count = 0;

    void add(uint64_t elapsed_ns)
    {
        total_ns += elapsed_ns;
        ++count;
    }

    double avg_ms_and_reset()
    {
        const double avg = count > 0
            ? static_cast<double>(total_ns) / static_cast<double>(count) / 1000000.0
            : 0.0;
        total_ns = 0;
        count = 0;
        return avg;
    }
};

class OverlayDisplay {
public:
    explicit OverlayDisplay(const AppConfig &config)
        : profile_(config.profile)
    {
        // 차선 투영도 모델 워프와 같은 카메라 파라미터를 쓴다(EDGEPILOT_CAMERA_INTRINSICS).
        projection_set_camera_intrinsics(config.camera_fx, config.camera_fy, config.camera_cx,
                                         config.camera_cy);
        default_projection_ = make_projection_state(config.manual_roll,
                                                    config.manual_pitch,
                                                    config.manual_yaw);
    }

    int run()
    {
        if (!model_state_.open(kModelStateTopic)) throw std::runtime_error("open modelState ipc failed");
        if (!panda_state_.open(kPandaStateTopic)) throw std::runtime_error("open pandaState ipc failed");
        if (!control_state_.open(kControlStateTopic)) throw std::runtime_error("open controlState ipc failed");
        if (!manager_state_.open(kManagerStateTopic)) throw std::runtime_error("open managerState ipc failed");
        if (!frame_sub_.open(kRoadAiFrameTopic, sizeof(RoadAiFrame), true))
            throw std::runtime_error("open roadAiFrame ipc failed");
        if (!record_state_.open(kRecordStateTopic)) throw std::runtime_error("open recordState ipc failed");
        if (!learner_state_.open(kLearnerStateTopic)) throw std::runtime_error("open learnerState ipc failed");
        if (!localization_state_.open(kLocalizationStateTopic))
            throw std::runtime_error("open localization ipc failed");
        if (!touch_.open()) std::fprintf(stderr, "overlayd: no touchscreen; tap toggles disabled\n");

        uint64_t window_start = monotonic_now_ns();
        while (!g_stop) {
            const uint64_t loop_start = monotonic_now_ns();
            pending_redraw_ = update_model() || pending_redraw_;
            pending_redraw_ = update_aux_state() || pending_redraw_;
            pending_redraw_ = turn_signal_clock_.update(loop_start, &hud_) || pending_redraw_;
            pending_redraw_ = update_touch(loop_start) || pending_redraw_;
            play_test_sound();
            apply_device_settings(loop_start);
            update_preview();
            if (pending_redraw_ && loop_start - last_overlay_draw_ns_ >= kOverlayIntervalNs) {
                pending_redraw_ = false;
                last_overlay_draw_ns_ = loop_start;
                draw_overlay();
            }

            const uint64_t now = monotonic_now_ns();
            if (now - window_start >= 1000000000ULL) {
                const double seconds = (now - window_start) / 1e9;
                hud_.preview_fps = static_cast<float>(preview_frames_ / seconds);
                hud_.overlay_fps = static_cast<float>(overlay_frames_ / seconds);
                hud_.model_fps = static_cast<float>(model_updates_ / seconds);
                system_monitor_.sample(&hud_);
                refresh_hud_state();
                pending_redraw_ = true;
                char profile_text[64] = "";
                if (profile_)
                    std::snprintf(profile_text, sizeof(profile_text), " draw=%.2fms video=%.2fms",
                                  overlay_stats_.avg_ms_and_reset(), present_stats_.avg_ms_and_reset());
                std::fprintf(stderr,
                             "overlay: preview=%.2f model=%.2f overlay=%.2f%s cpu=%.1f%% mem=%.1f%% temp=%.1fC errors=%u          \r",
                             hud_.preview_fps, hud_.model_fps, hud_.overlay_fps, profile_text,
                             hud_.cpu_percent, hud_.memory_percent, hud_.cpu_temp_c, errors_);
                std::fflush(stderr);
                preview_frames_ = overlay_frames_ = model_updates_ = 0;
                window_start = now;
            }

            usleep(kPollUs);
        }
        return 0;
    }

private:

    /* 최신 카메라 프레임을 화면 레이어 0에 올린다. 영상은 가운데 4:3을 IVPS로 잘라
     * 비율을 지키고, 투영(projection.cc)도 같은 영역을 640x480 화면에 담는다. */
    bool update_preview()
    {
        RoadAiFrame meta;
        if (!frame_sub_.read_new(&last_frame_seq_, &meta, sizeof(meta), 0)) return false;
        if (!frame_ring_.valid() && !frame_ring_.open(false)) return false;
        if (meta.slot >= frame_ring_.slot_count()) {
            ++errors_;
            return false;
        }
        const uint64_t start = profile_ ? monotonic_now_ns() : 0;
        const unsigned long long phys = frame_ring_.slot_phys(meta.slot);
        /* IVPS가 CMM 슬롯을 직접 읽는다(복사 없음). 읽는 동안 camerad가 슬롯을
         * 덮어썼으면 화면에 올리기 전에 버린다. */
        uint64_t seq = 0;
        const bool shown = phys != 0 &&
            frame_ring_.read_begin(meta.slot, meta.frame_id, &seq) &&
            display_.show_video_phys(phys, static_cast<int>(meta.width), static_cast<int>(meta.height),
                                     meta.format == V4L2_PIX_FMT_NV21, [&] {
                                         return frame_ring_.read_still_valid(meta.slot, meta.frame_id, seq);
                                     });
        if (!shown) {
            ++errors_;
            return false;
        }
        if (profile_) present_stats_.add(monotonic_now_ns() - start);
        ++preview_frames_;
        return true;
    }

    /* HUD를 세로 패널 방향 그림판에 화면 좌표(640x480)로 그리고(캔버스가 transpose), 바뀐 칸만
     * 그래픽 레이어 fb0로 옮긴다. 회전 하드웨어(TDP)를 거치지 않아 그린 화소가 그대로 나간다. */
    void draw_overlay()
    {
        const uint64_t draw_start = profile_ ? monotonic_now_ns() : 0;
        const MaixDisplay::OverlayBuffer buffer = display_.begin_overlay();
        if (!buffer.pixels) {
            ++errors_;
            return;
        }
        const OverlayTarget target{buffer.pixels, static_cast<uint32_t>(kOutW), static_cast<uint32_t>(kOutH),
                                   static_cast<uint32_t>(buffer.stride),
                                   HudOrientation{true, buffer.flip_x, buffer.flip_y}};
        overlay_.draw(target, have_model_state_ ? latest_output_ : ParsedModelOutput{},
                      have_model_state_ ? latest_projection_ : default_projection_, hud_);
        if (!display_.end_overlay(overlay_.last_damage().data(), overlay_.last_tile_shift())) ++errors_;
        if (profile_) overlay_stats_.add(monotonic_now_ns() - draw_start);
        ++overlay_frames_;
    }

    /* 웹 기기 설정: HUD 진단 카드의 기본값. 그 값이 바뀔 때만 적용해, 다른 설정(소리·밝기)을 바꿔도
     * 터치로 켜고 끈 진단 카드가 되돌아가지 않는다. 알림음 크기는 바뀌면 적용하고 확인음을 한 번
     * 낸다(시작 때 읽은 값은 소리 없이).
     * 카메라 장착 오프셋은 modeld가 그 프레임에 쓴 값을 ModelState로 받는다(projection_from_model_state). */
    void apply_device_settings(uint64_t now_ns)
    {
        if (!device_settings_file_.poll(now_ns, &device_settings_)) return;
        if (!device_settings_read_ || device_settings_.hud_debug != applied_hud_debug_) {
            hud_.debug_overlay = applied_hud_debug_ = device_settings_.hud_debug;
            pending_redraw_ = true;
        }
        const float percent = device_settings_.alert_volume_percent;
        if (sound_.enabled() && std::isfinite(percent) && std::fabs(percent - sound_.volume_percent()) > 0.5f) {
            sound_.set_volume_percent(percent);
            std::fprintf(stderr, "\noverlayd: alert volume %.0f%%\n", sound_.volume_percent());
            if (device_settings_read_) sound_.play(AlertSoundId::engage);  // 바꾼 크기를 들려준다
        }
        device_settings_read_ = true;
    }

    bool update_model()
    {
        if (!model_state_.poll()) return false;
        const ModelState &model = model_state_.latest();
        ++model_updates_;
        have_model_state_ = model.valid != 0 && fresh(model.model_timestamp_ns, monotonic_now_ns());
        latest_output_ = parsed_from_model_state(model);
        latest_projection_ = projection_from_model_state(model);
        const float lane_center = have_model_state_ ? lane_center_offset_m(latest_output_)
                                                    : std::numeric_limits<float>::quiet_NaN();
        hud_.lane_center_offset_m = smooth_lane_center_offset(hud_.lane_center_offset_m, lane_center);
        return true;
    }

    /* 새 panda/control/manager/record/학습기/locationd 스냅샷이 있으면 true. 모델이 멈춰도
     * 속도·토스트가 제어 상태를 따라가도록 재그리기 트리거가 된다. */
    bool update_aux_state()
    {
        bool changed = panda_state_.poll();
        changed = control_state_.poll() || changed;
        changed = manager_state_.poll() || changed;
        changed = record_state_.poll() || changed;
        changed = learner_state_.poll() || changed;
        changed = localization_state_.poll() || changed;
        refresh_hud_state();
        return changed;
    }

    /* 터치로 네트워크·진단 카드를 여닫고(HudTouch) 탭은 로그에 남긴다. 화면이 바뀌면 true. */
    bool update_touch(uint64_t now_ns)
    {
        const bool was_open = hud_.network_card, was_debug = hud_.debug_overlay;
        int x = 0, y = 0;
        if (touch_.poll_tap(&x, &y)) {
            const char *action = hud_touch_.tap(x, y, kOutW, kOutH, now_ns, &hud_);
            std::fprintf(stderr, "\noverlayd: tap x=%d y=%d %s\n", x, y, action);
        }
        hud_touch_.expire(now_ns, &hud_);
        return hud_.network_card != was_open || hud_.debug_overlay != was_debug;
    }

    static bool fresh(uint64_t timestamp_ns, uint64_t now)
    {
        return timestamp_fresh_ns(timestamp_ns, now, kStateFreshNs);
    }

    struct Freshness {
        bool model = false;
        bool panda = false;
        bool control = false;
        bool manager = false;
        bool record = false;
        bool learner = false;
        bool localization = false;
    };

    Freshness freshness(uint64_t now) const
    {
        return {fresh(model_state_.latest().model_timestamp_ns, now),
                fresh(panda_state_.latest().timestamp_ns, now),
                fresh(control_state_.latest().timestamp_ns, now),
                fresh(manager_state_.latest().timestamp_ns, now),
                fresh(record_state_.latest().timestamp_ns, now),
                fresh(learner_state_.latest().timestamp_ns, now),
                fresh(localization_state_.latest().timestamp_ns, now)};
    }

    /* 최신 스냅샷을 HUD 상태로 옮기고, 이 프레임의 알림음을 낸다(OverlayAlertPolicy). */
    void refresh_hud_state()
    {
        const uint64_t now = monotonic_now_ns();
        const Freshness f = freshness(now);
        have_model_state_ = model_state_.latest().valid != 0 && f.model;
        hud_apply_panda_state(panda_state_.latest(), f.panda, &hud_);
        hud_apply_control_state(control_state_.latest(), f.control, &hud_);
        hud_apply_model_state(model_state_.latest(), f.model, &hud_);
        hud_apply_manager_state(manager_state_.latest(), f.manager, have_model_state_, &hud_);
        hud_apply_record_state(record_state_.latest(), f.record, &hud_);
        hud_apply_learner_state(learner_state_.latest(), f.learner, &hud_);
        hud_apply_localization_state(localization_state_.latest(), f.localization, &hud_);
        play_alert(alert_policy_.update(control_state_.latest(), f.control, panda_state_.latest(), f.panda, now,
                                        &hud_));
    }

    void play_test_sound()
    {
        static constexpr AlertSoundId kOrder[] = {
            AlertSoundId::engage, AlertSoundId::disengage, AlertSoundId::unable,
            AlertSoundId::signal_changed, AlertSoundId::unavailable,
        };
        const int requests = g_test_sound_requests;
        if (requests == test_sounds_played_) return;
        test_sounds_played_ = requests;
        const AlertSoundId id = kOrder[static_cast<size_t>(requests - 1) % std::size(kOrder)];
        sound_.play(id);
        std::fprintf(stderr, "\noverlayd: test sound %s\n", alert_sound_name(id));
    }

    void play_alert(const OverlaySoundDecision &decision)
    {
        if (decision.sound == OverlaySound::none) return;
        sound_.play(overlay_sound_id(decision.sound));
        switch (decision.sound) {
        case OverlaySound::unable:
            std::fprintf(stderr, "overlayd: alert=unable event=%u block=%s\n", decision.event_id,
                         control_state_.latest().engage_reject_block);
            break;
        case OverlaySound::take_control:
            std::fprintf(stderr, "overlayd: alert=take_control soft_disable=%d steer_saturated=%d block=%s\n",
                         hud_.soft_disabling ? 1 : 0, hud_.steer_saturated ? 1 : 0, hud_.active_block);
            break;
        case OverlaySound::unavailable:
            std::fprintf(stderr, "overlayd: alert=unavailable\n");
            break;
        default:
            std::fprintf(stderr, "overlayd: alert=%s event=%u\n", overlay_sound_name(decision.sound),
                         decision.event_id);
            break;
        }
    }

    OverlayRenderer overlay_;
    AlertSound sound_;
    DeviceSettingsFile device_settings_file_;
    DeviceSettings device_settings_;
    bool device_settings_read_ = false;
    bool applied_hud_debug_ = false;  // 마지막으로 적용한 웹 설정의 HUD 진단
    int test_sounds_played_ = 0;
    bool profile_ = false;

    Subscription<ModelState> model_state_;
    Subscription<PandaState> panda_state_;
    Subscription<ControlState> control_state_;
    Subscription<ManagerState> manager_state_;
    Subscription<RecordState> record_state_;
    Subscription<LearnerState> learner_state_;
    Subscription<LocalizationState> localization_state_;
    LatestChannel frame_sub_;
    MaixDisplay display_;
    MaixTouch touch_;
    FrameRing frame_ring_;
    uint64_t last_frame_seq_ = 0;
    uint64_t last_overlay_draw_ns_ = 0;

    ParsedModelOutput latest_output_ {};
    ProjectionState latest_projection_ {};
    ProjectionState default_projection_ {};
    bool have_model_state_ = false;
    bool pending_redraw_ = true;
    unsigned errors_ = 0;

    unsigned preview_frames_ = 0;
    unsigned overlay_frames_ = 0;
    unsigned model_updates_ = 0;

    StageStats overlay_stats_;
    StageStats present_stats_;
    SystemMonitor system_monitor_;
    OverlayAlertPolicy alert_policy_;
    TurnSignalClock turn_signal_clock_;
    HudTouch hud_touch_;
    OverlayHudState hud_;
};

} // namespace

int main()
{
    install_stop_signal_handlers(&g_stop);
    signal(SIGUSR1, on_test_sound_signal);

    try {
        AppConfig config = AppConfig::from_env_defaults();
        OverlayDisplay app(config);
        return app.run();
    } catch (const std::exception &e) {
        std::fprintf(stderr, "overlayd error: %s\n", e.what());
        return 1;
    }
}
