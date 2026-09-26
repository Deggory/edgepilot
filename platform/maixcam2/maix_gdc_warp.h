#ifndef MAIXCAM2_GDC_WARP_H
#define MAIXCAM2_GDC_WARP_H

/* 모델 입력 워프를 AX620E IVPS GDC(AX_IVPS_Dewarp, 투시 변환)로 한다. 소스 NV12를 CMM 버퍼(source())에 채우면 medmodel/sbigmodel
 * 두 시점으로 512x256 NV12를 만들고 모델의 YUV6 배치(Y 4평면 + U + V, 128x256)로
 * 풀어 준다. 행렬은 모델 픽셀 -> 소스 픽셀(ModelInputTransform::projection_matrix).
 * CPU 워프와 Y는 최대 1 LSB, U/V는 평균 0.4 LSB 차이다(실측).
 * AX_SYS_Init은 같은 프로세스의 AxEngineSession이 해 둔 뒤에 만든다. */

#include <cstddef>
#include <cstdint>

class GdcWarp {
public:
    GdcWarp(int src_width, int src_height);
    ~GdcWarp();
    GdcWarp(const GdcWarp &) = delete;
    GdcWarp &operator=(const GdcWarp &) = delete;

    // 다음 warp()가 읽을 소스 NV12 버퍼(src_width*src_height*3/2, 캐시 가능 CMM).
    uint8_t *source() { return src_virt_; }
    int src_width() const { return src_w_; }
    int src_height() const { return src_h_; }

    // 두 시점을 워프해 각 YUV6(6*128*256 바이트)에 쓴다.
    bool warp(const float med_projection[9], const float sbig_projection[9],
              uint8_t *med_yuv6, uint8_t *sbig_yuv6);
    // 소스를 복사하지 않고 다른 CMM 블록(예: 프레임 링 슬롯)에서 직접 읽는다.
    bool warp_phys(unsigned long long src_phys, const float med_projection[9],
                   const float sbig_projection[9], uint8_t *med_yuv6, uint8_t *sbig_yuv6);

private:
    void release();
    bool warp_one(unsigned long long src_phys, const float projection[9], int dst, uint8_t *yuv6);

    int src_w_, src_h_;
    size_t src_bytes_;
    unsigned long long src_phys_ = 0;  // AX_U64
    uint8_t *src_virt_ = nullptr;
    unsigned long long dst_phys_[2] = {};
    uint8_t *dst_virt_[2] = {};
};

#endif
