#ifndef UTILS_TIME_H
#define UTILS_TIME_H

// 런타임 공통 시계는 monotonic_now_ns 하나다(CLOCK_BOOTTIME). 프로세스 사이를
// 건너가는 타임스탬프는 전부 이 시계로 찍는다.

#include <time.h>
#include <cstdint>

inline uint64_t monotonic_now_ns() {
    timespec ts{};
#ifdef CLOCK_BOOTTIME
    clock_gettime(CLOCK_BOOTTIME, &ts);
#else
    clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}

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

#endif
