#pragma once

// NV12(width x height)를 모델 입력 YUV6 배치로 푼다: [Y(짝행,짝열), Y(홀행,짝열), Y(짝행,홀열), Y(홀행,홀열), U, V]
// 여섯 평면, 각 width/2 x height/2. GDC 워프(maix_gdc_warp) 뒤 프레임마다 두 번(512x256) 돈다. 예전 루프는 출력
// 포인터 여섯 개가 서로 겹칠 수 있어 컴파일러가 벡터화하지 못했다(프레임마다 바이트 단위로 65,536번). 여기서는
// __restrict로 겹치지 않는다고 알리고, NEON이 있으면 16열씩 나눠 읽어(vld2q) 쓴다. 바이트 재배치라 NEON과 스칼라
// 경로의 출력이 같다(gtest_calibration Yuv6Pack).

#include <cstddef>
#include <cstdint>

#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif

inline void nv12_to_yuv6(const uint8_t *__restrict nv12, uint8_t *__restrict yuv6, int width, int height) {
    const size_t w = static_cast<size_t>(width), hw = w / 2, hh = static_cast<size_t>(height) / 2, plane = hw * hh;
    const uint8_t *uv_plane = nv12 + w * static_cast<size_t>(height);
    for (size_t r = 0; r < hh; ++r) {
        const uint8_t *even = nv12 + 2 * r * w;
        const uint8_t *odd = even + w;
        const uint8_t *uv = uv_plane + r * w;
        uint8_t *p0 = yuv6 + r * hw, *p1 = p0 + plane, *p2 = p1 + plane, *p3 = p2 + plane;
        uint8_t *pu = p3 + plane, *pv = pu + plane;
        size_t col = 0;
#if defined(__ARM_NEON)
        for (; col + 16 <= hw; col += 16) {
            const uint8x16x2_t e = vld2q_u8(even + 2 * col);
            const uint8x16x2_t o = vld2q_u8(odd + 2 * col);
            const uint8x16x2_t c = vld2q_u8(uv + 2 * col);
            vst1q_u8(p0 + col, e.val[0]);
            vst1q_u8(p1 + col, o.val[0]);
            vst1q_u8(p2 + col, e.val[1]);
            vst1q_u8(p3 + col, o.val[1]);
            vst1q_u8(pu + col, c.val[0]);
            vst1q_u8(pv + col, c.val[1]);
        }
#endif
        for (size_t x = 2 * col; x + 1 < w; x += 2) {  // NEON 16열에 남은 것(또는 NEON이 없을 때 전부)
            p0[x / 2] = even[x];
            p1[x / 2] = odd[x];
            p2[x / 2] = even[x + 1];
            p3[x / 2] = odd[x + 1];
            pu[x / 2] = uv[x];
            pv[x / 2] = uv[x + 1];
        }
    }
}
