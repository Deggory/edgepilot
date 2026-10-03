#include "alert_tones.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace {

/* 음 하나: 시작 시각, 음높이, 감쇠 시간 상수(진폭이 1/e가 되는 시간), 크기. 앞 음이 다 사라지기
 * 전에 다음 음이 겹쳐 울린다. */
struct Note {
    unsigned at_ms;
    double hz;
    unsigned decay_ms;
    double level;
};

/* 알림 하나: 음들과 밝기(2·3배음 비율). 거부·경고일수록 밝아 작은 스피커에서 또렷하다. */
struct Tone {
    const char *name;
    std::vector<Note> notes;
    double brightness;
};

constexpr double kE5 = 659.26, kF5 = 698.46, kG5 = 783.99, kA5 = 880.0, kB5 = 987.77, kD6 = 1174.66,
                 kG6 = 1567.98;

// AlertSoundId 순서.
const Tone kTones[] = {
    // 거부·조향 넘겨받기: 같은 음을 두 번 두드리고 4도 아래로
    {"unable", {{0, kA5, 60, 0.9}, {150, kA5, 60, 0.9}, {300, kE5, 140, 1.0}}, 0.55},
    // engage: 완전5도 위로
    {"engage", {{0, kG5, 140, 0.85}, {110, kD6, 200, 1.0}}, 0.35},
    // disengage: 완전5도 아래로
    {"disengage", {{0, kD6, 140, 0.9}, {110, kG5, 220, 1.0}}, 0.35},
    // 출발 알림: 밝게 세 음 위로
    {"signal_changed", {{0, kB5, 110, 0.75}, {75, kD6, 110, 0.8}, {150, kG6, 240, 1.0}}, 0.3},
    // 사용 불가 경고: 트라이톤을 두 번
    {"unavailable", {{0, kB5, 90, 1.0}, {160, kF5, 120, 1.0}, {360, kB5, 90, 1.0}, {520, kF5, 150, 1.0}}, 0.65},
};
static_assert(std::size(kTones) == static_cast<size_t>(AlertSoundId::count), "one tone per AlertSoundId");

constexpr double kAttackMs = 4.0;  // 반코사인으로 올려 딸깍 소리 없이 시작
constexpr double kTailMs = 10.0;   // 끝을 0으로 접는다
constexpr double kQuietDecays = 7.0;  // 감쇠 상수의 7배면 -60 dB

}  // namespace

const char *alert_sound_name(AlertSoundId id)
{
    return id < AlertSoundId::count ? kTones[static_cast<int>(id)].name : "none";
}

std::vector<int16_t> render_alert_tone(AlertSoundId id, int rate)
{
    if (id >= AlertSoundId::count) return {};
    const Tone &tone = kTones[static_cast<int>(id)];
    double end_ms = 0.0;
    for (const Note &note : tone.notes) end_ms = std::max(end_ms, note.at_ms + kQuietDecays * note.decay_ms);
    std::vector<double> mix(static_cast<size_t>(end_ms * rate / 1000.0), 0.0);
    for (const Note &note : tone.notes) {
        const size_t start = static_cast<size_t>(note.at_ms) * rate / 1000;
        for (size_t i = start; i < mix.size(); ++i) {
            const double ms = static_cast<double>(i - start) * 1000.0 / rate;
            const double attack = ms < kAttackMs ? 0.5 - 0.5 * std::cos(M_PI * ms / kAttackMs) : 1.0;
            const double decays = ms / note.decay_ms;
            const double phase = 2.0 * M_PI * note.hz * ms / 1000.0;
            mix[i] += note.level * attack *
                      (std::exp(-decays) * std::sin(phase) +
                       tone.brightness * (0.5 * std::exp(-2.0 * decays) * std::sin(2.0 * phase) +
                                          0.25 * std::exp(-3.0 * decays) * std::sin(3.0 * phase)));
        }
    }
    double peak = 0.0;
    for (double v : mix) peak = std::max(peak, std::fabs(v));
    const double gain = peak > 0.0 ? kAlertTonePeak * 32767.0 / peak : 0.0;
    const size_t tail = static_cast<size_t>(kTailMs * rate / 1000.0);
    std::vector<int16_t> pcm(mix.size());
    for (size_t i = 0; i < mix.size(); ++i) {
        const size_t left = mix.size() - i;
        const double fold = left < tail ? static_cast<double>(left - 1) / tail : 1.0;
        pcm[i] = static_cast<int16_t>(std::lround(mix[i] * gain * fold));
    }
    return pcm;
}
