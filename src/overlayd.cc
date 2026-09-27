#include "alert_sound.h"
#include "app_config.h"
#include "utils_process.h"
#include "utils_time.h"
#include "overlay_state.h"
#include "ipc_channels.h"
#include "ipc_messages.h"
#include "overlay_renderer.h"
#include "projection.h"
#include "system_monitor.h"
#include "maix_display.h"
#include "utils_json.h"

#include <linux/videodev2.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/core.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iterator>
#include <thread>
#include <cstdio>
#include <stdexcept>
#include <string>

/* MaixCAM2 HUD. LCD의 VO 하드웨어 두 레이어를 쓴다: 카메라 프레임 링의 최신
 * 프레임은 레이어 0(IVPS로 가운데 4:3을 640x480에), HUD 렌더러가 640x480 네이티브
 * 배치로 그린 BGRA는 레이어 1에 올리고 합성은 하드웨어가 한다. 알림은 화면 토스트와
 * 로그에 더해 K230 부저와 같은 멜로디를 보드 스피커로 낸다(alert_sound.h). */

namespace {

volatile sig_atomic_t g_stop = 0;
/* 책상 확인용: SIGUSR1을 받을 때마다 알림음을 차례로 하나씩 낸다(pkill -USR1 overlayd). */
volatile sig_atomic_t g_test_sound_requests = 0;

void on_test_sound_signal(int) { ++g_test_sound_requests; }

constexpr int kOutW = MaixDisplay::kWidth;
constexpr int kOutH = MaixDisplay::kHeight;
constexpr uint64_t kStateFreshNs = 2000000000ULL;
// 오버레이(HUD 그림, OpenCV CPU)는 상태가 바뀔 때 모델과 같은 20 Hz로 다시 그린다.
// 상한을 50 ms가 아닌 45 ms로 두어 모델 출력 도착이 5 ms 루프만큼 흔들려도 프레임을
// 건너뛰지 않는다. 영상은 하드웨어 레이어라 카메라 프레임마다 올린다.
constexpr uint64_t kOverlayIntervalNs = 45000000ULL;
constexpr useconds_t kPollUs = 5000;
constexpr uint64_t kEngageAlertNs = 3000000000ULL;
constexpr uint64_t kTurnSignalStepNs = 50000000ULL;

std::string executable_dir()
{
    char path[512];
    const ssize_t length = readlink("/proc/self/exe", path, sizeof(path) - 1);
    if (length <= 0) return ".";
    path[length] = '\0';
    const std::string full(path);
    const size_t slash = full.rfind('/');
    return slash == std::string::npos ? "." : full.substr(0, slash);
}

const char *engage_block_text(const char *block)
{
    if (!block || block[0] == '\0') return "NOT READY";
    const char *label = engage_block_label(block);
    return label ? label : block;
}

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

/* 웹 기기 설정의 알림음 크기(params/display.json의 alert_volume_percent)를 1초마다 보고
 * 바뀌면 적용한 뒤 확인음을 한 번 낸다. SD가 녹화로 바쁠 때 stat/open이 몇 초씩 막힐 수
 * 있어 화면 루프가 아닌 자기 스레드에서 읽는다. 값이 없으면 시작 크기(EDGEPILOT_ALERT_VOLUME)를
 * 그대로 둔다. */
class SoundSettingsWatcher {
public:
    explicit SoundSettingsWatcher(AlertSound *sound)
        : sound_(sound), path_(param_path("display.json"))
    {
        if (sound_->enabled()) thread_ = std::thread(&SoundSettingsWatcher::loop, this);
    }
    ~SoundSettingsWatcher()
    {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
    }
    SoundSettingsWatcher(const SoundSettingsWatcher &) = delete;
    SoundSettingsWatcher &operator=(const SoundSettingsWatcher &) = delete;

private:
    void loop()
    {
        bool first = true;
        struct timespec last_mtime = {};
        while (!stop_) {
            struct stat st {};
            if (stat(path_.c_str(), &st) == 0 &&
                (first || st.st_mtim.tv_sec != last_mtime.tv_sec ||
                 st.st_mtim.tv_nsec != last_mtime.tv_nsec)) {
                last_mtime = st.st_mtim;
                std::ifstream file(path_);
                const std::string text((std::istreambuf_iterator<char>(file)),
                                       std::istreambuf_iterator<char>());
                float percent = 0.0f;
                if (parse_json_float_value(text, "alert_volume_percent", &percent) &&
                    std::fabs(percent - sound_->volume_percent()) > 0.5f) {
                    sound_->set_volume_percent(percent);
                    std::fprintf(stderr, "\noverlayd: alert volume %.0f%%\n", sound_->volume_percent());
                    if (!first) sound_->play(AlertSoundId::engage);  // 바꾼 크기를 들려준다
                }
                first = false;
            }
            for (int i = 0; i < 10 && !stop_; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    AlertSound *sound_;
    std::string path_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

class OverlayDisplay {
public:
    explicit OverlayDisplay(const AppConfig &config)
        : profile_(config.profile)
    {
        // OpenCV 스레드 풀은 대기 중에도 코어를 돌려서 modeld와 CPU를 다툰다.
        cv::setNumThreads(0);
        // 차선 투영도 모델 워프와 같은 카메라 파라미터를 쓴다(EDGEPILOT_CAMERA_INTRINSICS).
        projection_set_camera_intrinsics(config.camera_fx, config.camera_fy, config.camera_cx,
                                         config.camera_cy);
        default_projection_ = make_projection_state(config.manual_roll,
                                                    config.manual_pitch,
                                                    config.manual_yaw);
        overlay_.load_assets(executable_dir() + "/assets/ui");
    }

    int run()
    {
        if (!model_state_sub_.open(kModelStateTopic, sizeof(ModelState), true))
            throw std::runtime_error("open modelState ipc failed");
        if (!panda_state_sub_.open(kPandaStateTopic, sizeof(PandaState), true))
            throw std::runtime_error("open pandaState ipc failed");
        if (!control_state_sub_.open(kControlStateTopic, sizeof(ControlState), true))
            throw std::runtime_error("open controlState ipc failed");
        if (!manager_state_sub_.open(kManagerStateTopic, sizeof(ManagerState), true))
            throw std::runtime_error("open managerState ipc failed");
        if (!frame_sub_.open(kRoadAiFrameTopic, sizeof(RoadAiFrame), true))
            throw std::runtime_error("open roadAiFrame ipc failed");

        uint64_t window_start = monotonic_now_ns();
        while (!g_stop) {
            const uint64_t loop_start = monotonic_now_ns();
            pending_redraw_ = update_model() || pending_redraw_;
            pending_redraw_ = update_aux_state() || pending_redraw_;
            pending_redraw_ = update_turn_signal(loop_start) || pending_redraw_;
            play_test_sound();
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

    /* HUD를 화면 레이어 1의 CMM 블록(640x480 BGRA)에 바로 그리고 복사 없이 올린다.
     * 가로로 누르지 않는 640 폭 배치라 글꼴이 네이티브 픽셀에 정수 배율로 맞는다. */
    void draw_overlay()
    {
        const uint64_t draw_start = profile_ ? monotonic_now_ns() : 0;
        uint8_t *buffer = display_.begin_overlay();
        if (!buffer) {
            ++errors_;
            return;
        }
        const OverlayTarget target{buffer, static_cast<uint32_t>(kOutW), static_cast<uint32_t>(kOutH),
                                   static_cast<uint32_t>(kOutW * 4)};
        overlay_.draw(target, have_model_state_ ? latest_output_ : ParsedModelOutput{},
                      have_model_state_ ? latest_projection_ : default_projection_, hud_, false);
        if (!display_.end_overlay()) ++errors_;
        if (profile_) overlay_stats_.add(monotonic_now_ns() - draw_start);
        ++overlay_frames_;
    }

    /* 새 스냅샷이면 저장하고 true. */
    template <typename State>
    static bool poll(LatestChannel &channel, State *state, uint64_t *seq)
    {
        State candidate;
        uint64_t candidate_seq = *seq;
        if (!channel.read(&candidate, sizeof(candidate), &candidate_seq) || candidate_seq == *seq)
            return false;
        *state = candidate;
        *seq = candidate_seq;
        return true;
    }

    bool update_model()
    {
        if (!poll(model_state_sub_, &latest_model_state_, &latest_model_seq_)) return false;
        ++model_updates_;
        have_model_state_ = latest_model_state_.valid != 0 &&
            fresh(latest_model_state_.model_timestamp_ns, monotonic_now_ns());
        latest_output_ = parsed_from_model_state(latest_model_state_);
        latest_projection_ = projection_from_model_state(latest_model_state_);
        return true;
    }

    /* 새 panda/control/manager 스냅샷이 있으면 true. 모델이 멈춰도 속도·토스트가
     * 제어 상태를 따라가도록 재그리기 트리거가 된다. */
    bool update_aux_state()
    {
        bool changed = poll(panda_state_sub_, &latest_panda_state_, &latest_panda_seq_);
        changed = poll(control_state_sub_, &latest_control_state_, &latest_control_seq_) || changed;
        changed = poll(manager_state_sub_, &latest_manager_state_, &latest_manager_seq_) || changed;
        refresh_hud_state();
        return changed;
    }

    /* 깜빡이 단계는 켜진 시각 기준으로 나간다. 단계가 바뀌면 true. */
    bool update_turn_signal(uint64_t now_ns)
    {
        if (hud_.left_blinker != previous_left_blinker_ ||
            hud_.right_blinker != previous_right_blinker_) {
            previous_left_blinker_ = hud_.left_blinker;
            previous_right_blinker_ = hud_.right_blinker;
            turn_signal_start_ns_ = now_ns;
        }
        const bool blinking = hud_.left_blinker || hud_.right_blinker;
        const int step = blinking
            ? static_cast<int>(((now_ns - turn_signal_start_ns_) / kTurnSignalStepNs) %
                               kTurnSignalSteps)
            : 0;
        const bool changed = step != hud_.turn_signal_step;
        hud_.turn_signal_step = step;
        return changed && blinking;
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
    };

    Freshness freshness(uint64_t now) const
    {
        return {fresh(latest_model_state_.model_timestamp_ns, now),
                fresh(latest_panda_state_.timestamp_ns, now),
                fresh(latest_control_state_.timestamp_ns, now),
                fresh(latest_manager_state_.timestamp_ns, now)};
    }

    /* 최신 스냅샷을 HUD 상태로 옮기고, control 이벤트 카운터로 토스트·알림을 낸다. */
    void refresh_hud_state()
    {
        const uint64_t now = monotonic_now_ns();
        const Freshness f = freshness(now);
        have_model_state_ = latest_model_state_.valid != 0 && f.model;
        hud_apply_panda_state(latest_panda_state_, f.panda, &hud_);
        hud_apply_control_state(latest_control_state_, f.control, &hud_);
        hud_apply_model_state(latest_model_state_, f.model, &hud_);
        hud_apply_manager_state(latest_manager_state_, f.manager, have_model_state_, &hud_);
        process_alert_events(f, now);
    }

    void play_test_sound()
    {
        static constexpr AlertSoundId kOrder[] = {
            AlertSoundId::engage, AlertSoundId::disengage, AlertSoundId::unable,
            AlertSoundId::signal_changed, AlertSoundId::unavailable,
        };
        static constexpr const char *kNames[] = {
            "engage", "disengage", "unable", "signal_changed", "unavailable",
        };
        const int requests = g_test_sound_requests;
        if (requests == test_sounds_played_) return;
        test_sounds_played_ = requests;
        const size_t i = static_cast<size_t>(requests - 1) % std::size(kOrder);
        sound_.play(kOrder[i]);
        std::fprintf(stderr, "\noverlayd: test sound %s\n", kNames[i]);
    }

    /* 고른 알림을 소리 내고 기록한다. engage 거부는 토스트도 띄운다. 알렸으면 true. */
    bool play_alert(const OverlayAlertEvents::Decision &decision, uint64_t now)
    {
        static constexpr const char *kAlertNames[] = {
            "none", "unable", "engage", "disengage", "signal_changed",
        };
        static constexpr AlertSoundId kSounds[] = {
            AlertSoundId::count, AlertSoundId::unable, AlertSoundId::engage,
            AlertSoundId::disengage, AlertSoundId::signal_changed,
        };
        if (decision.alert == OverlayAlert::none) return false;
        sound_.play(kSounds[static_cast<int>(decision.alert)]);
        if (decision.alert == OverlayAlert::unable) {
            const ControlState &c = latest_control_state_;
            std::snprintf(hud_.engage_alert_message, sizeof(hud_.engage_alert_message),
                          "UNABLE TO ENGAGE: %s", engage_block_text(c.engage_reject_block));
            engage_alert_until_ns_ = now + kEngageAlertNs;
            std::fprintf(stderr, "overlayd: alert=unable event=%u block=%s\n",
                         decision.event_id, c.engage_reject_block);
            return true;
        }
        std::fprintf(stderr, "overlayd: alert=%s event=%u\n",
                     kAlertNames[static_cast<int>(decision.alert)], decision.event_id);
        return true;
    }

    /* 가용 → 불가용 천이에만 울린다. active 천이는 정차 부근 path 깜빡임마다
     * 울리므로 소리내지 않는다. 같은 프레임에 다른 알림이 울렸으면 생략. */
    void play_availability_alert(const Freshness &f, bool suppressed)
    {
        const bool panda_unavailable =
            latest_panda_state_.timestamp_ns != 0 &&
            (!f.panda || !hud_.panda_connected || !hud_.panda_healthy ||
             latest_panda_state_.faults != 0);
        const bool unavailable =
            !f.control || panda_unavailable || latest_control_state_.steering_fault != 0;
        if (!alert_state_initialized_) {
            alert_state_initialized_ = true;
        } else if (unavailable && !previous_unavailable_ && !suppressed) {
            sound_.play(AlertSoundId::unavailable);
            std::fprintf(stderr, "overlayd: alert=unavailable\n");
        }
        previous_unavailable_ = unavailable;
    }

    void process_alert_events(const Freshness &f, uint64_t now)
    {
        OverlayAlertEvents::Decision decision;
        if (f.control) decision = alert_events_.update(latest_control_state_, hud_.departure_alert_type);
        bool played = play_alert(decision, now);
        if (now >= engage_alert_until_ns_) hud_.engage_alert_message[0] = '\0';
        played = play_take_control_alert(played) || played;
        play_availability_alert(f, played);
    }

    /* 해제 예고와 조향 한계 경고는 켜지는 순간 한 번 울린다. 해제 자체는 disengage
     * 이벤트가 따로 울린다. 같은 프레임에 다른 알림이 울렸으면 다음 프레임으로 미룬다. */
    bool play_take_control_alert(bool suppressed)
    {
        const bool soft = hud_.soft_disabling, saturated = hud_.steer_saturated;
        const bool rising = (soft && !previous_soft_disabling_) ||
                            (saturated && !previous_steer_saturated_);
        if (suppressed) return false;
        previous_soft_disabling_ = soft;
        previous_steer_saturated_ = saturated;
        if (!rising) return false;
        sound_.play(AlertSoundId::unable);
        std::fprintf(stderr, "overlayd: alert=take_control soft_disable=%d steer_saturated=%d block=%s\n",
                     soft ? 1 : 0, saturated ? 1 : 0, hud_.active_block);
        return true;
    }

    OverlayRenderer overlay_;
    AlertSound sound_;
    SoundSettingsWatcher sound_settings_{&sound_};  // sound_ 뒤에 선언해야 먼저 멈춘다
    bool previous_soft_disabling_ = false;
    bool previous_steer_saturated_ = false;
    int test_sounds_played_ = 0;
    bool profile_ = false;

    LatestChannel model_state_sub_;
    LatestChannel panda_state_sub_;
    LatestChannel control_state_sub_;
    LatestChannel manager_state_sub_;
    LatestChannel frame_sub_;
    MaixDisplay display_;
    FrameRing frame_ring_;
    uint64_t last_frame_seq_ = 0;
    uint64_t last_overlay_draw_ns_ = 0;

    uint64_t latest_model_seq_ = 0;
    uint64_t latest_panda_seq_ = 0;
    uint64_t latest_control_seq_ = 0;
    uint64_t latest_manager_seq_ = 0;
    ModelState latest_model_state_ {};
    PandaState latest_panda_state_ {};
    ControlState latest_control_state_ {};
    ManagerState latest_manager_state_ {};
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
    OverlayAlertEvents alert_events_;
    bool alert_state_initialized_ = false;
    bool previous_unavailable_ = false;
    uint64_t engage_alert_until_ns_ = 0;
    bool previous_left_blinker_ = false;
    bool previous_right_blinker_ = false;
    uint64_t turn_signal_start_ns_ = 0;
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
