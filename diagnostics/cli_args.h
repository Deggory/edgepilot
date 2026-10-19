#pragma once

// 진단 도구 공용 옵션 값 읽기. std::atof·atoi·sscanf는 "abc"나 "0.3s", 모자란 목록을 말없이 0이나 앞부분으로 읽어,
// 오타가 있어도 기본값 대신 0으로 돌았다. 값 전체가 유한한 수(목록이면 정한 개수)이고 [lo, hi] 안일 때만 담고 true를
// 돌려준다. 도구는 false면 사용법 오류로 끝낸다.

#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <limits>

inline bool parse_double_arg(const char *text, double *out, double lo = -std::numeric_limits<double>::max(),
                             double hi = std::numeric_limits<double>::max()) {
    if (text == nullptr || *text == '\0') return false;
    char *end = nullptr;
    errno = 0;
    const double value = std::strtod(text, &end);
    if (errno != 0 || *end != '\0' || !std::isfinite(value) || value < lo || value > hi) return false;
    *out = value;
    return true;
}

inline bool parse_float_arg(const char *text, float *out, double lo = -std::numeric_limits<float>::max(),
                            double hi = std::numeric_limits<float>::max()) {
    double value = 0.0;
    if (!parse_double_arg(text, &value, lo, hi)) return false;
    *out = static_cast<float>(value);
    return true;
}

inline bool parse_long_arg(const char *text, long *out, long lo = LONG_MIN, long hi = LONG_MAX) {
    if (text == nullptr || *text == '\0') return false;
    char *end = nullptr;
    errno = 0;
    const long value = std::strtol(text, &end, 10);
    if (errno != 0 || *end != '\0' || value < lo || value > hi) return false;
    *out = value;
    return true;
}

inline bool parse_int_arg(const char *text, int *out, int lo = INT_MIN, int hi = INT_MAX) {
    long value = 0;
    if (!parse_long_arg(text, &value, lo, hi)) return false;
    *out = static_cast<int>(value);
    return true;
}

// "a,b,c"처럼 쉼표로 나눈 유한한 수 정확히 n개(8개까지).
inline bool parse_float_list_arg(const char *text, float *out, int n) {
    constexpr int kMaxValues = 8;
    if (text == nullptr || n < 1 || n > kMaxValues) return false;
    float values[kMaxValues];
    const char *p = text;
    for (int i = 0; i < n; ++i) {
        char *end = nullptr;
        errno = 0;
        const double value = std::strtod(p, &end);
        if (end == p || errno != 0 || !std::isfinite(value) || std::fabs(value) > std::numeric_limits<float>::max())
            return false;
        values[i] = static_cast<float>(value);
        const bool last = i + 1 == n;
        if (*end != (last ? '\0' : ',')) return false;
        p = end + 1;
    }
    for (int i = 0; i < n; ++i) out[i] = values[i];
    return true;
}
