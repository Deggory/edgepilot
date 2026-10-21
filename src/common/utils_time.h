#pragma once

// 런타임 공통 시계는 monotonic_now_ns 하나다(CLOCK_BOOTTIME). 프로세스 사이를
// 건너가는 타임스탬프는 전부 이 시계로 찍는다. 데몬 1초 통계 줄의 창(StatsWindow)도 여기 있다.

#include <time.h>
#include <cstdint>

inline uint64_t monotonic_now_ns() {
    timespec ts{};
#ifdef CLOCK_BOOTTIME
    clock_gettime(CLOCK_BOOTTIME, &ts);
#else
    clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

// 데몬 1초 통계 줄의 창. 루프가 셀 일마다 count()하고, close_if_due()가 창이 끝났으면 그 창의 길이(seconds)와
// 초당 개수(rate)를 남기고 새 창을 연다. monotonic_now_ns 시계라 NTP가 시각을 옮겨도 창 길이가 흔들리지 않는다.
class StatsWindow {
public:
    explicit StatsWindow(uint64_t now_ns, uint64_t length_ns = 1'000'000'000ULL)
        : start_ns_(now_ns), window_start_ns_(now_ns), length_ns_(length_ns) {}

    void count() { ++count_; }
    bool close_if_due(uint64_t now_ns) {
        if (now_ns - window_start_ns_ < length_ns_) return false;
        seconds_ = static_cast<double>(now_ns - window_start_ns_) / 1e9;
        rate_ = static_cast<double>(count_) / seconds_;
        count_ = 0;
        window_start_ns_ = now_ns;
        return true;
    }
    double seconds() const { return seconds_; }
    double rate() const { return rate_; }
    // 시작부터 지금까지의 초당 개수(끝 줄)
    double total_rate(uint64_t total, uint64_t now_ns) const {
        return now_ns > start_ns_ ? static_cast<double>(total) * 1e9 / static_cast<double>(now_ns - start_ns_) : 0.0;
    }

private:
    uint64_t start_ns_;
    uint64_t window_start_ns_;
    uint64_t length_ns_;
    uint64_t count_ = 0;
    double seconds_ = 0.0;
    double rate_ = 0.0;
};

// 상류 DT_CTRL·DT_MDL: controlsd 틱(100 Hz)과 모델 프레임(20 Hz)의 주기(s). float 계산에는 F 쪽을 쓴다(값을 바꾸지
// 않으려고 float 식은 float 상수로 둔다).
constexpr double kDtCtrl = 0.01;
constexpr double kDtMdl = 0.05;
constexpr float kDtCtrlF = 0.01f;
constexpr float kDtMdlF = 0.05f;

// ns 타임스탬프 freshness 공통 판정. 0(미설정)과 미래 타임스탬프를 모두 stale로 본다.
inline bool timestamp_fresh_ns(uint64_t timestamp_ns, uint64_t now_ns, uint64_t max_age_ns) {
    return timestamp_ns != 0 && now_ns >= timestamp_ns && now_ns - timestamp_ns <= max_age_ns;
}

// CAN 신호 freshness 공통 판정. 미수신(음수)과 미래 타임스탬프를 모두
// stale로 본다.
inline bool signal_time_fresh(double timestamp_s, double now_s, double timeout_s) {
    return timestamp_s >= 0.0 && now_s >= timestamp_s && now_s - timestamp_s <= timeout_s;
}
