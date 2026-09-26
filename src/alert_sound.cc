#include "alert_sound.h"

#include "utils_process.h"

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

extern char **environ;

namespace {

constexpr int kRate = 48000;
constexpr int kChannels = 2;
constexpr double kFadeMs = 4.0;  // 음 경계의 딸깍 소리를 없앤다
constexpr int kChunkFrames = kRate / 50;  // 20 ms씩 흘린다

struct Tone {
    unsigned hz;  // 0이면 쉼
    unsigned ms;
};

/* K230 piezo_buzzer.c의 시퀀스 그대로(음높이 Hz, 길이 ms). AlertSoundId 순서. */
const std::vector<Tone> kMelodies[] = {
    // unable: 거부
    {{740, 160}, {0, 30}, {523, 200}, {0, 30}, {740, 400}},
    // engage: 상승 아르페지오
    {{262, 45}, {0, 12}, {392, 45}, {0, 12}, {523, 45}, {0, 12}, {784, 45}, {0, 12}, {1047, 240}},
    // disengage: 하강
    {{784, 70}, {0, 15}, {659, 70}, {0, 15}, {523, 70}, {0, 15}, {440, 80}, {0, 15}, {392, 190}},
    // signal_changed: 출발 알림
    {{392, 65}, {0, 15}, {523, 65}, {0, 15}, {659, 65}, {0, 15}, {784, 180}},
    // unavailable: 경고
    {{330, 80}, {0, 25}, {262, 80}, {0, 25}, {330, 80}, {0, 25}, {262, 80}, {0, 60}, {220, 210}},
};
static_assert(sizeof(kMelodies) / sizeof(kMelodies[0]) == static_cast<size_t>(AlertSoundId::count),
              "one melody per AlertSoundId");

/* 사인파에 약한 2·3배음을 섞어 작은 스피커에서도 음높이가 또렷하게 들리게 한다. */
std::vector<int16_t> render(const std::vector<Tone> &melody, double gain)
{
    std::vector<int16_t> pcm;
    for (const Tone &tone : melody) {
        const int n = kRate * static_cast<int>(tone.ms) / 1000;
        const int fade = std::min(n / 2, static_cast<int>(kRate * kFadeMs / 1000.0));
        for (int i = 0; i < n; ++i) {
            double v = 0.0;
            if (tone.hz) {
                const double phase = 2.0 * M_PI * tone.hz * i / kRate;
                v = (std::sin(phase) + 0.25 * std::sin(2 * phase) + 0.1 * std::sin(3 * phase)) / 1.35;
                v *= std::min({1.0, i / static_cast<double>(fade), (n - 1 - i) / static_cast<double>(fade)});
            }
            const int16_t s = static_cast<int16_t>(std::lround(v * gain * 32767.0));
            pcm.push_back(s);  // L
            pcm.push_back(s);  // R
        }
    }
    return pcm;
}

bool write_all(int fd, const int16_t *data, size_t samples)
{
    const char *p = reinterpret_cast<const char *>(data);
    size_t left = samples * sizeof(int16_t);
    while (left > 0) {
        const ssize_t n = ::write(fd, p, left);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += n;
        left -= static_cast<size_t>(n);
    }
    return true;
}

} // namespace

AlertSound::AlertSound() : pcm_(env_string("K230_ALERT_PCM", "plughw:0,1"))
{
    if (!env_flag("K230_ALERT_SOUND", true)) return;
    if (access("/usr/bin/aplay", X_OK) != 0) {
        std::fprintf(stderr, "alert sound: /usr/bin/aplay not found, sounds off\n");
        return;
    }
    // aplay가 죽어 파이프가 끊겨도 프로세스가 SIGPIPE로 끝나지 않게 한다(write가 EPIPE를 돌려준다).
    signal(SIGPIPE, SIG_IGN);
    const double gain = std::clamp(env_float("K230_ALERT_VOLUME", 70.0f), 0.0f, 100.0f) / 100.0 * 0.6;
    for (const auto &melody : kMelodies) clips_.push_back(render(melody, gain));
    enabled_ = true;
    thread_ = std::thread(&AlertSound::loop, this);
}

AlertSound::~AlertSound()
{
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    stop_player();
}

void AlertSound::play(AlertSoundId id)
{
    if (enabled_ && id != AlertSoundId::count) pending_ = static_cast<int>(id);
}

bool AlertSound::start_player()
{
    int fds[2];
    if (pipe(fds) != 0) return false;
    // 파이프에 무음이 쌓이면 알림이 그만큼 늦게 나오므로 버퍼를 한 페이지로 줄인다(~21 ms).
    fcntl(fds[1], F_SETPIPE_SZ, 4096);
    fcntl(fds[1], F_SETFD, FD_CLOEXEC);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, fds[0], STDIN_FILENO);
    posix_spawn_file_actions_addclose(&actions, fds[0]);
    std::string device = "-D" + pcm_;
    // ALSA 버퍼 60 ms / 주기 20 ms: 지연을 짧게 두면서 스레드가 늦어도 끊기지 않을 만큼.
    char *argv[] = {const_cast<char *>("aplay"), const_cast<char *>("-q"), device.data(),
                    const_cast<char *>("-t"), const_cast<char *>("raw"), const_cast<char *>("-f"),
                    const_cast<char *>("S16_LE"), const_cast<char *>("-r"), const_cast<char *>("48000"),
                    const_cast<char *>("-c"), const_cast<char *>("2"), const_cast<char *>("-B"),
                    const_cast<char *>("60000"), const_cast<char *>("-F"), const_cast<char *>("20000"),
                    nullptr};
    pid_t pid = -1;
    const int rc = posix_spawn(&pid, "/usr/bin/aplay", &actions, nullptr, argv, environ);
    posix_spawn_file_actions_destroy(&actions);
    ::close(fds[0]);
    if (rc != 0) {
        ::close(fds[1]);
        return false;
    }
    player_ = pid;
    pipe_fd_ = fds[1];
    return true;
}

void AlertSound::stop_player()
{
    if (pipe_fd_ >= 0) ::close(pipe_fd_);  // aplay는 EOF로 끝난다
    pipe_fd_ = -1;
    if (player_ > 0) {
        kill(player_, SIGTERM);
        waitpid(player_, nullptr, 0);
    }
    player_ = -1;
}

void AlertSound::loop()
{
    const std::vector<int16_t> silence(kChunkFrames * kChannels, 0);
    const std::vector<int16_t> *clip = nullptr;
    size_t position = 0;
    while (!stop_) {
        if (pipe_fd_ < 0 && !start_player()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        const int next = pending_.exchange(-1);
        if (next >= 0) {  // 새 알림은 재생 중인 알림을 끊는다
            clip = &clips_[static_cast<size_t>(next)];
            position = 0;
        }
        const int16_t *data = silence.data();
        size_t samples = silence.size();
        if (clip) {
            samples = std::min(silence.size(), clip->size() - position);
            data = clip->data() + position;
            position += samples;
            if (position >= clip->size()) clip = nullptr;
        }
        // aplay가 실시간으로 읽으므로 이 write가 스레드의 박자를 맞춘다.
        if (!write_all(pipe_fd_, data, samples)) {
            std::fprintf(stderr, "alert sound: player exited, restarting\n");
            stop_player();
        }
    }
}
