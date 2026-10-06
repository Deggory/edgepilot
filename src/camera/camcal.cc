#include "hud/alert_sound.h"
#include "maixcam2/maix_camera.h"
#include "maixcam2/maix_cmm.h"
#include "maixcam2/maix_display.h"
#include "common/utils_process.h"
#include "common/utils_time.h"

#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <signal.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

/* 카메라 내부 파라미터 측정용 스틸 캡처(K230 camcal의 k230_snapshot을 옮긴 것).
 *
 * 런타임과 같은 MaixCamera 경로(센서 2560x1440 전체 화각을 크롭 없이 줄임, 보드
 * cam_flip/mirror, AI-ISP 설정)로 1920x1080을 받아, LCD에는 전체 화면을 원본 비율로
 * (위아래 검은 띠) 보여 주고, Func 키·화면 터치·터미널 Enter를 누르면 그 프레임을 무손실
 * PNG로 저장한다. 저장은 별도 스레드가 하므로 미리보기는 멈추지 않고, 파일이 디스크에
 * 쓰이면 확인음이 난다. 카메라를 독점하므로 edgepilot 런타임을 먼저 멈춘다.
 *
 * 환경: CAMCAL_DIR(기본 /root/camcal/snapshots), CAMCAL_WIDTH/HEIGHT(1920/1080),
 * CAMCAL_FORMAT(png|jpg), CAMCAL_MAX_SHUTTER_US(기본 10000: 손떨림 번짐을 줄인다),
 * CAMCAL_INPUTS(기본 /dev/input/event0,/dev/input/event1). */

namespace {

volatile sig_atomic_t g_stop = 0;

constexpr int kSlots = 3;
constexpr int kBarH = 60;  // 640x360 미리보기 위아래 띠

struct Shot {
    std::vector<uint8_t> nv12;
    int index = 0;
};

int next_index(const std::string &dir)
{
    int best = 0;
    if (DIR *d = opendir(dir.c_str())) {
        while (dirent *e = readdir(d)) {
            int n = 0;
            if (std::sscanf(e->d_name, "snap_%d_", &n) == 1 && n > best) best = n;
        }
        closedir(d);
    }
    return best + 1;
}

void make_dirs(const std::string &path)
{
    for (size_t i = 1; i <= path.size(); ++i)
        if (i == path.size() || path[i] == '/') mkdir(path.substr(0, i).c_str(), 0755);
}

std::string timestamp()
{
    char buf[32];
    const std::time_t t = std::time(nullptr);
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
    return buf;
}

class Saver {
public:
    Saver(std::string dir, int width, int height, bool png, AlertSound &sound)
        : dir_(std::move(dir)), width_(width), height_(height), png_(png), sound_(sound),
          thread_(&Saver::loop, this) {}
    ~Saver()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_one();
        thread_.join();
    }

    bool busy() const { return busy_; }
    void submit(Shot shot)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_ = std::move(shot);
            has_pending_ = true;
            busy_ = true;
        }
        cv_.notify_one();
    }

    std::atomic<int> saved{0};
    std::atomic<int> failed{0};
    std::atomic<double> last_sharp{0.0};
    std::atomic<bool> changed{false};

private:
    void loop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (true) {
            cv_.wait(lock, [this] { return has_pending_ || stop_; });
            if (!has_pending_) return;
            Shot shot = std::move(pending_);
            has_pending_ = false;
            lock.unlock();
            save(shot);
            busy_ = false;
            changed = true;
            lock.lock();
        }
    }

    void save(const Shot &shot)
    {
        const uint64_t start = monotonic_now_ns();
        cv::Mat yuv(height_ * 3 / 2, width_, CV_8UC1, const_cast<uint8_t *>(shot.nv12.data()));
        const cv::Mat y = yuv.rowRange(0, height_);
        cv::Mat lap;
        cv::Laplacian(y, lap, CV_64F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(lap, mean, stddev);
        const double sharp = stddev[0] * stddev[0];
        cv::Mat bgr;
        cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
        char name[64];
        std::snprintf(name, sizeof(name), "snap_%04d_%s.%s", shot.index, timestamp().c_str(),
                      png_ ? "png" : "jpg");
        const std::string path = dir_ + "/" + name;
        const std::vector<int> params = png_
            ? std::vector<int>{cv::IMWRITE_PNG_COMPRESSION, 1}
            : std::vector<int>{cv::IMWRITE_JPEG_QUALITY, 95};
        bool ok = false;
        try {
            ok = cv::imwrite(path, bgr, params);
        } catch (const cv::Exception &e) {
            std::fprintf(stderr, "\ncamcal: imwrite: %s\n", e.what());
        }
        if (ok) ok = [&] {  // 확인음은 파일이 디스크에 있을 때 낸다
            const int fd = ::open(path.c_str(), O_RDONLY);
            if (fd < 0) return false;
            const bool synced = fsync(fd) == 0;
            ::close(fd);
            return synced;
        }();
        const double ms = (monotonic_now_ns() - start) / 1e6;
        if (ok) {
            ++saved;
            last_sharp = sharp;
            sound_.play(AlertSoundId::signal_changed);
            std::fprintf(stderr, "\ncamcal: %d saved %s sharp=%.0f write=%.0fms\n", saved.load(),
                         path.c_str(), sharp, ms);
        } else {
            ++failed;
            sound_.play(AlertSoundId::unable);
            std::fprintf(stderr, "\ncamcal: save failed %s\n", path.c_str());
        }
    }

    std::string dir_;
    int width_, height_;
    bool png_;
    AlertSound &sound_;
    std::mutex mutex_;
    std::condition_variable cv_;
    Shot pending_;
    bool has_pending_ = false;
    bool stop_ = false;
    std::atomic<bool> busy_{false};
    std::thread thread_;
};

/* Func 키(gpio_keys)와 터치스크린의 누름, 터미널의 Enter/스페이스를 셔터로, q를 종료로. */
class Inputs {
public:
    explicit Inputs(const std::string &devices)
    {
        std::stringstream list(devices);
        std::string path;
        while (std::getline(list, path, ',')) {
            if (path.empty()) continue;
            const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd < 0) {
                std::fprintf(stderr, "camcal: cannot open %s: %s\n", path.c_str(), std::strerror(errno));
                continue;
            }
            fds_.push_back(fd);
            std::fprintf(stderr, "camcal: shutter input %s\n", path.c_str());
        }
        if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &saved_tty_) == 0) {
            termios raw = saved_tty_;
            raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            tty_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
        }
    }
    ~Inputs()
    {
        for (int fd : fds_) ::close(fd);
        if (tty_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_tty_);
    }

    // 누른 횟수(셔터)를 돌려준다. q면 quit을 켠다.
    int poll(bool *quit)
    {
        int presses = 0;
        for (int fd : fds_) {
            input_event ev;
            while (::read(fd, &ev, sizeof(ev)) == static_cast<ssize_t>(sizeof(ev)))
                if (ev.type == EV_KEY && ev.value == 1) ++presses;
        }
        if (tty_) {
            char c;
            while (::read(STDIN_FILENO, &c, 1) == 1) {
                if (c == '\n' || c == ' ') ++presses;
                if (c == 'q' || c == 'Q') *quit = true;
            }
        }
        return presses;
    }

private:
    std::vector<int> fds_;
    termios saved_tty_{};
    bool tty_ = false;
};

void draw_osd(MaixDisplay &display, int width, int height, int saved, double sharp, bool saving,
              bool flash)
{
    const MaixDisplay::OverlayBuffer buffer = display.begin_overlay();
    if (!buffer.pixels) return;
    // 가로 화면 좌표로 그린 뒤 세로 패널 방향 그림판으로 옮긴다(transpose, 보드 방향만큼 뒤집기).
    cv::Mat osd(MaixDisplay::kHeight, MaixDisplay::kWidth, CV_8UC4, cv::Scalar(0, 0, 0, 0));
    const cv::Scalar white(255, 255, 255, 255), yellow(0, 220, 255, 255);
    char top[96], bottom[96];
    std::snprintf(top, sizeof(top), "CAMCAL %dx%d   saved %d", width, height, saved);
    if (saving)
        std::snprintf(bottom, sizeof(bottom), "SAVING...");
    else if (saved > 0)
        std::snprintf(bottom, sizeof(bottom), "FUNC: shoot   last sharp %.0f", sharp);
    else
        std::snprintf(bottom, sizeof(bottom), "FUNC: shoot");
    cv::putText(osd, top, cv::Point(12, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, white, 2, cv::LINE_8);
    cv::putText(osd, bottom, cv::Point(12, MaixDisplay::kHeight - 20), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                saving ? yellow : white, 2, cv::LINE_8);
    if (flash)
        cv::rectangle(osd, cv::Rect(0, kBarH, MaixDisplay::kWidth, MaixDisplay::kHeight - 2 * kBarH),
                      white, 8);
    cv::Mat panel(MaixDisplay::kWidth, MaixDisplay::kHeight, CV_8UC4, buffer.pixels, buffer.stride);
    cv::transpose(osd, panel);
    if (buffer.flip_x) cv::flip(panel, panel, 1);
    if (buffer.flip_y) cv::flip(panel, panel, 0);
    display.end_overlay();
}

} // namespace

int main()
{
    install_stop_signal_handlers(&g_stop);

    try {
        const int width = static_cast<int>(env_float("CAMCAL_WIDTH", 1920));
        const int height = static_cast<int>(env_float("CAMCAL_HEIGHT", 1080));
        const std::string dir = env_string("CAMCAL_DIR", "/root/camcal/snapshots");
        const bool png = env_string("CAMCAL_FORMAT", "png") != "jpg";
        // MaixCamera가 읽는 최대 노출. 손에 들고 찍으므로 런타임(33 ms)보다 짧게 둔다.
        setenv("EDGEPILOT_MAX_SHUTTER_US", env_string("CAMCAL_MAX_SHUTTER_US", "10000").c_str(), 1);
        make_dirs(dir);
        int index = next_index(dir);

        MaixCamera camera(width, height, 20, true);
        MaixDisplay display;
        display.set_letterbox(true);
        AlertSound sound;
        Saver saver(dir, width, height, png, sound);
        Inputs inputs(env_string("CAMCAL_INPUTS", "/dev/input/event0,/dev/input/event1"));

        const size_t frame_bytes = static_cast<size_t>(width) * height * 3 / 2;
        std::vector<CmmBlock> slots(kSlots);
        for (auto &slot : slots)
            if (!cmm_alloc(&slot, frame_bytes, "camcal")) throw std::runtime_error("CMM alloc failed");

        std::fprintf(stderr, "camcal: %dx%d NV12 -> %s (%s). Func/touch/Enter = shoot, q = quit\n",
                     width, height, dir.c_str(), png ? "png" : "jpg");

        uint64_t frames = 0, errors = 0, window_frames = 0;
        uint64_t window_start = monotonic_now_ns(), flash_until = 0;
        bool redraw = true, flashing = false, was_busy = false;
        while (!g_stop) {
            CmmBlock &slot = slots[frames % kSlots];
            if (!camera.read_to(slot.phys)) {
                ++errors;
                continue;
            }
            ++frames;
            ++window_frames;
            if (!display.show_video_phys(slot.phys, width, height, false, [] { return true; })) ++errors;

            bool quit = false;
            const int presses = inputs.poll(&quit);
            if (quit) break;
            const uint64_t now = monotonic_now_ns();
            if (presses > 0) {
                if (saver.busy()) {
                    sound.play(AlertSoundId::unable);
                    std::fprintf(stderr, "\ncamcal: still saving the previous shot, ignored\n");
                } else {
                    // CMM 슬롯은 캐시하지 않는 메모리라 한 번에 복사해 두고 저장 스레드에 넘긴다.
                    Shot shot;
                    shot.nv12.assign(slot.virt, slot.virt + frame_bytes);
                    shot.index = index++;
                    saver.submit(std::move(shot));
                    flash_until = now + 150000000ULL;
                    flashing = true;
                    redraw = true;
                }
            }
            if (flashing && now >= flash_until) {
                flashing = false;
                redraw = true;
            }
            if (saver.changed.exchange(false) || saver.busy() != was_busy) redraw = true;
            if (redraw) {
                was_busy = saver.busy();
                draw_osd(display, width, height, saver.saved, saver.last_sharp, was_busy, flashing);
                redraw = false;
            }

            if (now - window_start >= 1000000000ULL) {
                std::fprintf(stderr, "camcal: fps=%.1f saved=%d failed=%d errors=%llu          \r",
                             window_frames * 1e9 / (now - window_start), saver.saved.load(),
                             saver.failed.load(), static_cast<unsigned long long>(errors));
                window_start = now;
                window_frames = 0;
            }
        }
        std::fprintf(stderr, "\ncamcal: done, saved %d\n", saver.saved.load());
        for (auto &slot : slots) cmm_free(&slot);
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "camcal error: %s\n", e.what());
        return 1;
    }
}
