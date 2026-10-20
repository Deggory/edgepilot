#ifndef MAIXCAM2_GDC_WARP_H
#define MAIXCAM2_GDC_WARP_H

// 모델 입력 워프를 AX620E IVPS GDC(AX_IVPS_Dewarp, 투시 변환)로 한다. CMM 블록의 소스 NV12(프레임 링 슬롯, 재생은
// source())에서 medmodel/sbigmodel 두 시점으로 512x256 NV12를 만들고 모델의 YUV6 배치(Y 4평면 + U + V, 128x256)로
// 풀어 준다. 행렬은 모델 픽셀 -> 소스 픽셀(ModelInputTransform::projection_matrix).
// 예전 CPU 워프(openpilot OpenCL과 같은 쌍선형)와 Y는 최대 1 LSB, U/V는 평균 0.4 LSB 차이였다(실측).
// AX_SYS_Init은 같은 프로세스의 AxEngineSession이 해 둔 뒤에 만든다.

#include <cstddef>
#include <cstdint>

class GdcWarp {
public:
    GdcWarp(int src_width, int src_height);
    ~GdcWarp();
    GdcWarp(const GdcWarp &) = delete;
    GdcWarp &operator=(const GdcWarp &) = delete;

    // CPU로 채울 소스 NV12 버퍼(src_width*src_height*3/2, 캐시 가능 CMM, 재생용). 처음 부를 때 잡는다.
    uint8_t *source();
    unsigned long long source_phys() const { return src_phys_; }

    // src_phys의 CMM 블록에서 두 시점을 워프해 각 YUV6(6*128*256 바이트)에 쓴다. source()면 캐시를 먼저 내보낸다.
    bool warp(unsigned long long src_phys, const float med_projection[9], const float sbig_projection[9],
              uint8_t *med_yuv6, uint8_t *sbig_yuv6);

private:
    bool dewarp(unsigned long long src_phys, const float projection[9], int view);

    int src_w_, src_h_;
    size_t src_bytes_;
    unsigned long long src_phys_ = 0;  // AX_U64
    uint8_t *src_virt_ = nullptr;
    // 두 시점의 NV12 출력을 한 블록에 이어 둔다(캐시 무효화를 프레임마다 같은 범위로 한 번)
    unsigned long long dst_phys_ = 0;
    uint8_t *dst_virt_ = nullptr;
};

#endif
