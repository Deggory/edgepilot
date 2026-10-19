#ifndef UTILS_MATH_H
#define UTILS_MATH_H

#include <algorithm>
#include <cmath>
#include <initializer_list>

inline float clamp_float(float value, float lo, float hi) {
    if (!std::isfinite(value)) return lo;
    return std::min(std::max(value, lo), hi);
}

inline int clamp_int(float value, int lo, int hi) {
    return std::min(std::max(static_cast<int>(std::lround(value)), lo), hi);
}

inline int clamp_int(int value, int lo, int hi) { return std::min(std::max(value, lo), hi); }

// openpilot interp: 강한 단조 증가 xp에 대한 구간 선형 보간. 범위 밖은 끝값, NaN이면 마지막 값. fp는 xp와 길이가
// 같다.
inline float interp(float x, std::initializer_list<float> xp, std::initializer_list<float> fp) {
    const auto xs = xp.begin();
    const auto fs = fp.begin();
    const size_t n = xp.size();
    if (x <= xs[0]) return fs[0];
    if (x >= xs[n - 1]) return fs[n - 1];
    for (size_t i = 1; i < n; ++i)
        if (x <= xs[i]) return fs[i - 1] + (x - xs[i - 1]) * (fs[i] - fs[i - 1]) / (xs[i] - xs[i - 1]);
    return fs[n - 1];
}

// row-major 3x3 곱. 결합 순서는 k 내부 누적이다.
inline void matmul3(const float *a, const float *b, float *out) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) sum += a[r * 3 + k] * b[k * 3 + c];
            out[r * 3 + c] = sum;
        }
    }
}

inline float deg_to_rad(float deg) { return deg * 0.017453292519943295f; }

inline float rad_to_deg(float rad) { return rad * 57.29577951308232f; }

#endif
