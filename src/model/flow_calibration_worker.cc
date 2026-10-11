#include "model/flow_calibration_worker.h"

#include "maixcam2/maix_ive_flow.h"

#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>

namespace {

/* 칸 후보의 문턱(최댓값 대비). 밤에는 불빛이 최댓값을 차지해 1%로는 37~46점만 남는 구간이 있었다
 * (2026-10 보드 재생). 칸마다 하나만 고르므로 문턱을 낮춰도 점은 고르게 퍼진다. */
constexpr double kCornerQuality = 0.002;
constexpr size_t kQueueDepth = 3;
constexpr double kRadToDeg = 57.29577951308232;

double now_ms()
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double thread_cpu_ms()
{
    timespec ts{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) * 1e3 + static_cast<double>(ts.tv_nsec) / 1e6;
}

} // namespace

FlowWindowTracker::FlowWindowTracker(const FlowIntrinsics &k, int width, int height)
    : k_(k), ive_(std::make_unique<IveFlow>(width, height)), start_(2 * kFlowMaxPoints), cur_(2 * kFlowMaxPoints),
      alive_(kFlowMaxPoints), step_in_(2 * kFlowMaxPoints), step_out_(2 * kFlowMaxPoints), step_ok_(kFlowMaxPoints)
{
    live_.reserve(kFlowMaxPoints);
    fit_a_.reserve(2 * kFlowMaxPoints);
    fit_b_.reserve(2 * kFlowMaxPoints);
}

FlowWindowTracker::~FlowWindowTracker() = default;

const char *FlowWindowTracker::pyramid_engine() const
{
    return ive_->pyramid_engine();
}

void FlowWindowTracker::start_window(uint64_t frame_id)
{
    (void)frame_id;
    const int16_t *resp = ive_->corner_response();
    n_ = resp ? pick_flow_corners(resp, ive_->width() / 2, IveFlow::kBandRows, 2, IveFlow::kBandTop, kCornerQuality,
                                  kFlowMaxPoints, start_.data())
              : 0;
    std::copy(start_.begin(), start_.begin() + 2 * n_, cur_.begin());
    std::fill(alive_.begin(), alive_.begin() + n_, 1);
    steps_ = 0;
    in_window_ = n_ >= kFlowMinTracks;
}

void FlowWindowTracker::process(unsigned long long y_phys, int stride, uint64_t frame_id, const FlowGateInput &gate,
                                const std::function<bool()> &still_valid)
{
    auto lose = [this] {
        if (in_window_) ++stats_.windows_lost;
        in_window_ = false;
    };
    if (!flow_gate_open(gate)) {  // 곧고 빠르지 않으면 IVE를 쓰지 않는다
        lose();
        have_last_ = false;
        return;
    }
    const double wall0 = now_ms(), cpu0 = thread_cpu_ms();
    const bool consecutive = have_last_ && frame_id == last_frame_ + 1;
    if (!ive_->push(y_phys, stride) || (still_valid && !still_valid())) {
        lose();
        have_last_ = false;
        return;
    }
    const std::function<bool()> prev_valid = consecutive ? last_valid_ : std::function<bool()>();
    have_last_ = true;
    last_frame_ = frame_id;
    last_valid_ = still_valid;
    ++stats_.frames;
    if (in_window_ && !consecutive) lose();  // 이동이 8 px 한계를 넘을 수 있다

    if (in_window_) {
        // 살아 있는 점만 모아 따라간다
        live_.clear();
        for (int i = 0; i < n_; ++i) {
            if (!alive_[i]) continue;
            const size_t m = live_.size();
            step_in_[2 * m] = cur_[2 * i];
            step_in_[2 * m + 1] = cur_[2 * i + 1];
            live_.push_back(i);
        }
        const int m = static_cast<int>(live_.size());
        // 원본 단은 링 슬롯을 그대로 읽었으므로 LK 동안 두 슬롯이 덮어써지지 않았어야 한다
        if (!ive_->track(step_in_.data(), m, step_out_.data(), step_ok_.data()) || (prev_valid && !prev_valid()) ||
            (still_valid && !still_valid())) {
            lose();
        } else {
            int alive = 0;
            for (int j = 0; j < m; ++j) {
                const int i = live_[j];
                if (step_ok_[j]) {
                    cur_[2 * i] = step_out_[2 * j];
                    cur_[2 * i + 1] = step_out_[2 * j + 1];
                    ++alive;
                } else {
                    alive_[i] = 0;
                }
            }
            ++steps_;
            if (alive < kFlowMinTracks) {
                lose();
            } else if (steps_ >= kFlowWindowSteps) {
                fit_a_.clear();
                fit_b_.clear();
                for (int i = 0; i < n_; ++i) {
                    if (!alive_[i]) continue;
                    fit_a_.push_back(start_[2 * i]);
                    fit_a_.push_back(start_[2 * i + 1]);
                    fit_b_.push_back(cur_[2 * i]);
                    fit_b_.push_back(cur_[2 * i + 1]);
                }
                const FlowSolution s = solve_flow_direction(k_, fit_a_.data(), fit_b_.data(), alive);
                if (s.valid) {
                    calibrator_.add(s, alive, frame_id);
                    ++stats_.windows_done;
                } else {
                    calibrator_.note_rejected(s);
                    ++stats_.rejected;
                }
                in_window_ = false;
            }
        }
    }
    // 창이 없으면(방금 끝났어도) 이 프레임에서 새 창을 연다
    if (!in_window_) start_window(frame_id);
    stats_.wall_ms += now_ms() - wall0;
    stats_.cpu_ms += thread_cpu_ms() - cpu0;
}

void FlowWindowTracker::reset_calibration()
{
    calibrator_.reset();
    in_window_ = false;
    have_last_ = false;
    last_valid_ = nullptr;
}

FlowCalibrationWorker::FlowCalibrationWorker(const FlowIntrinsics &k, int width, int height,
                                             const FlowBlocks *restore)
    : tracker_(k, width, height)
{
    if (restore && !restore->yaw.empty()) {
        if (tracker_.restore(*restore))
            std::fprintf(stderr, "modeld: flow calibration restored %zu blocks\n", restore->yaw.size());
        else
            std::fprintf(stderr, "modeld: flow calibration ignored stored blocks (invalid)\n");
    }
    state_ = tracker_.calibrator().state();
    blocks_ = tracker_.calibrator().blocks();
    thread_ = std::thread(&FlowCalibrationWorker::run, this);
}

void FlowCalibrationWorker::request_reset()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        reset_requested_ = true;
    }
    cv_.notify_one();
}

bool FlowCalibrationWorker::take_blocks(FlowBlocks *out)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!blocks_changed_) return false;
    *out = blocks_;
    blocks_changed_ = false;
    return true;
}

FlowCalibrationWorker::~FlowCalibrationWorker()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_one();
    if (thread_.joinable()) thread_.join();
}

void FlowCalibrationWorker::push(Frame frame)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= kQueueDepth) {
            queue_.pop_front();
            ++dropped_;
        }
        queue_.push_back(std::move(frame));
    }
    cv_.notify_one();
}

FlowCalibrationState FlowCalibrationWorker::state(uint32_t *resets_applied) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (resets_applied) *resets_applied = resets_applied_;
    return state_;
}

void FlowCalibrationWorker::run()
{
    // modeld는 nice -15로 돈다. 이 스레드는 IVE를 기다리는 시간이 대부분이라 보통 우선순위면 된다.
    setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 0);
    uint32_t logged_blocks = tracker_.calibrator().state().blocks;
    uint32_t logged_mount_changes = tracker_.calibrator().state().mount_changes;
    for (;;) {
        Frame f;
        bool reset = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return stop_ || reset_requested_ || !queue_.empty(); });
            if (stop_) return;
            reset = reset_requested_;
            reset_requested_ = false;
            if (!reset) {
                f = std::move(queue_.front());
                queue_.pop_front();
            }
        }
        if (reset) {
            tracker_.reset_calibration();
            const FlowCalibrationState st = tracker_.calibrator().state();
            logged_blocks = st.blocks;
            logged_mount_changes = st.mount_changes;
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = st;
            blocks_ = FlowBlocks{};
            blocks_changed_ = false;  // 저장 파일은 보정 서비스가 지운다
            ++resets_applied_;
            std::fprintf(stderr, "\nflowcal: reset, collecting from scratch\n");
            continue;
        }
        tracker_.process(f.y_phys, f.stride, f.frame_id, f.gate, f.still_valid);
        const FlowCalibrationState st = tracker_.calibrator().state();
        const bool block_done = st.blocks != logged_blocks;
        uint64_t dropped = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = st;
            dropped = dropped_;
            if (block_done) {
                blocks_ = tracker_.calibrator().blocks();
                blocks_changed_ = true;
            }
        }
        if (!block_done) continue;
        logged_blocks = st.blocks;
        if (st.mount_changes != logged_mount_changes) {
            logged_mount_changes = st.mount_changes;
            std::fprintf(stderr, "\nflowcal: mount change detected (%u so far), kept the last %u blocks\n",
                         st.mount_changes, st.estimate_blocks);
        }
        const FlowWindowTracker::Stats &s = tracker_.stats();
        const double frames = static_cast<double>(std::max<uint64_t>(s.frames, 1));
        std::fprintf(stderr,
                     "\nflowcal: block %u windows %u yaw %+.3f pitch %+.3f deg (sd %.3f/%.3f, %u blocks, %s) | "
                     "last yaw %+.3f pitch %+.3f rot %.3f deg tracks %u inliers %u parallax %.1f px | frames %llu "
                     "done %llu lost %llu rejected %llu dropped %llu | %.1f ms wall %.2f ms cpu per frame, pyramid %s\n",
                     st.blocks, st.windows, st.yaw * kRadToDeg, st.pitch * kRadToDeg, st.sd_yaw * kRadToDeg,
                     st.sd_pitch * kRadToDeg, st.estimate_blocks, st.status == 2 ? "estimate" : "collecting",
                     st.window_yaw * kRadToDeg, st.window_pitch * kRadToDeg, st.window_rotation * kRadToDeg,
                     st.window_tracks, st.window_inliers, st.window_parallax, static_cast<unsigned long long>(s.frames),
                     static_cast<unsigned long long>(s.windows_done), static_cast<unsigned long long>(s.windows_lost),
                     static_cast<unsigned long long>(s.rejected), static_cast<unsigned long long>(dropped),
                     s.wall_ms / frames, s.cpu_ms / frames, tracker_.pyramid_engine());
    }
}
