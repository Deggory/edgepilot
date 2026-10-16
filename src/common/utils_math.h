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

// openpilot interp: 단조 증가 xp에 대한 구간 선형 보간. 범위 밖은 끝값.
inline float interp(float x, std::initializer_list<float> xp, std::initializer_list<float> fp) {
    const auto xs = xp.begin();
    const auto fs = fp.begin();
    const size_t n = xp.size();
    if (n == 0 || fp.size() != n) return 0.0f;
    if (x <= xs[0]) return fs[0];
    if (x >= xs[n - 1]) return fs[n - 1];
    for (size_t idx = 1; idx < n; ++idx) {
        if (x <= xs[idx]) {
            const float low_x = xs[idx - 1];
            const float high_x = xs[idx];
            const float low_y = fs[idx - 1];
            const float high_y = fs[idx];
            if (high_x - low_x < 1e-6f && low_x - high_x < 1e-6f) return high_y;
            return low_y + (x - low_x) * (high_y - low_y) / (high_x - low_x);
        }
    }
    return fs[n - 1];
}

// 같은 보간을 배열로: 증가하는 xp[0..count)의 fp. 범위 밖은 끝값, 같은 x가 이어지면 뒤 값.
template <typename T>
T interp(T x, const T *xp, const T *fp, size_t count) {
    if (count == 0) return T(0);
    if (x <= xp[0]) return fp[0];
    for (size_t i = 1; i < count; ++i) {
        if (x <= xp[i]) {
            const T span = xp[i] - xp[i - 1];
            if (span <= T(0)) return fp[i];
            const T p = (x - xp[i - 1]) / span;
            return fp[i - 1] + p * (fp[i] - fp[i - 1]);
        }
    }
    return fp[count - 1];
}

// interp의 xp로 쓰도록 뒤로 가는 점을 앞 점에 맞춰 평평하게 한다(양자화된 plan은 정차 부근에서 먼 점이 뒤로 뛴다).
template <typename T>
void make_monotonic(T *xp, size_t count) {
    for (size_t i = 1; i < count; ++i)
        if (xp[i] < xp[i - 1]) xp[i] = xp[i - 1];
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
