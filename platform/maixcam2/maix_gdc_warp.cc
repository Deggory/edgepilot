#include "maixcam2/maix_gdc_warp.h"

#include "ax_ivps_api.h"
#include "ax_sys_api.h"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace {

constexpr int kModelW = 512;
constexpr int kModelH = 256;
constexpr size_t kDstBytes = static_cast<size_t>(kModelW) * kModelH * 3 / 2;

AX_VIDEO_FRAME_T nv12_frame(uint64_t phys, uint8_t *virt, int w, int h)
{
    AX_VIDEO_FRAME_T f = {};
    f.u32Width = w;
    f.u32Height = h;
    f.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    f.u32PicStride[0] = f.u32PicStride[1] = w;
    f.u64PhyAddr[0] = phys;
    f.u64PhyAddr[1] = phys + static_cast<uint64_t>(w) * h;
    f.u64VirAddr[0] = reinterpret_cast<uintptr_t>(virt);
    f.u64VirAddr[1] = reinterpret_cast<uintptr_t>(virt) + static_cast<uint64_t>(w) * h;
    f.u32FrameSize = static_cast<AX_U32>(static_cast<size_t>(w) * h * 3 / 2);
    return f;
}

} // namespace

GdcWarp::GdcWarp(int src_width, int src_height)
    : src_w_(src_width), src_h_(src_height),
      src_bytes_(static_cast<size_t>(src_width) * src_height * 3 / 2)
{
    if (AX_IVPS_Init() != 0) throw std::runtime_error("AX_IVPS_Init failed");
    // 생성자가 던지면 소멸자가 불리지 않으므로 여기서 잡은 것을 풀고 다시 던진다.
    try {
        void *virt = nullptr;
        if (AX_SYS_MemAllocCached(&src_phys_, &virt, static_cast<AX_U32>(src_bytes_), 128,
                                  reinterpret_cast<const AX_S8 *>("gdc_src")) != 0)
            throw std::runtime_error("GDC source buffer alloc failed");
        src_virt_ = static_cast<uint8_t *>(virt);
        for (int i = 0; i < 2; ++i) {
            if (AX_SYS_MemAllocCached(&dst_phys_[i], &virt, static_cast<AX_U32>(kDstBytes), 128,
                                      reinterpret_cast<const AX_S8 *>("gdc_dst")) != 0)
                throw std::runtime_error("GDC output buffer alloc failed");
            dst_virt_[i] = static_cast<uint8_t *>(virt);
        }
    } catch (...) {
        release();
        throw;
    }
}

GdcWarp::~GdcWarp()
{
    release();
}

void GdcWarp::release()
{
    for (int i = 0; i < 2; ++i)
        if (dst_virt_[i]) AX_SYS_MemFree(dst_phys_[i], dst_virt_[i]);
    if (src_virt_) AX_SYS_MemFree(src_phys_, src_virt_);
    dst_virt_[0] = dst_virt_[1] = src_virt_ = nullptr;
    AX_IVPS_Deinit();
}

bool GdcWarp::warp_one(unsigned long long src_phys, const float projection[9], int dst, uint8_t *yuv6)
{
    AX_IVPS_DEWARP_ATTR_T attr = {};
    attr.eDewarpType = AX_IVPS_DEWARP_PERSPECTIVE;
    const double scale = projection[8] != 0.0f ? 1.0 / projection[8] : 1.0;
    for (int i = 0; i < 9; ++i)  // 소수 6자리 고정소수점
        attr.tPerspectiveAttr.nMatrix[i] = static_cast<AX_S64>(std::llround(projection[i] * scale * 1e6));
    AX_VIDEO_FRAME_T src = nv12_frame(src_phys, src_phys == src_phys_ ? src_virt_ : nullptr, src_w_, src_h_);
    AX_VIDEO_FRAME_T out = nv12_frame(dst_phys_[dst], dst_virt_[dst], kModelW, kModelH);
    if (AX_IVPS_Dewarp(&src, &out, &attr) != 0) return false;
    AX_SYS_MinvalidateCache(dst_phys_[dst], dst_virt_[dst], static_cast<AX_U32>(kDstBytes));

    // NV12 512x256 -> YUV6: [Y(짝행,짝열), Y(홀행,짝열), Y(짝행,홀열), Y(홀행,홀열), U, V]
    constexpr int hw = kModelW / 2, hh = kModelH / 2, plane = hw * hh;
    const uint8_t *y = dst_virt_[dst];
    const uint8_t *uv = y + kModelW * kModelH;
    for (int r = 0; r < hh; ++r) {
        const uint8_t *even = y + (2 * r) * kModelW;
        const uint8_t *odd = even + kModelW;
        uint8_t *p0 = yuv6 + r * hw, *p1 = p0 + plane, *p2 = p1 + plane, *p3 = p2 + plane;
        uint8_t *pu = p3 + plane, *pv = pu + plane;
        const uint8_t *c = uv + r * kModelW;
        for (int col = 0; col < hw; ++col) {
            p0[col] = even[2 * col];
            p1[col] = odd[2 * col];
            p2[col] = even[2 * col + 1];
            p3[col] = odd[2 * col + 1];
            pu[col] = c[2 * col];
            pv[col] = c[2 * col + 1];
        }
    }
    return true;
}

bool GdcWarp::warp(const float med_projection[9], const float sbig_projection[9],
                   uint8_t *med_yuv6, uint8_t *sbig_yuv6)
{
    AX_SYS_MflushCache(src_phys_, src_virt_, static_cast<AX_U32>(src_bytes_));
    return warp_one(src_phys_, med_projection, 0, med_yuv6) && warp_one(src_phys_, sbig_projection, 1, sbig_yuv6);
}

bool GdcWarp::warp_phys(unsigned long long src_phys, const float med_projection[9],
                        const float sbig_projection[9], uint8_t *med_yuv6, uint8_t *sbig_yuv6)
{
    return warp_one(src_phys, med_projection, 0, med_yuv6) && warp_one(src_phys, sbig_projection, 1, sbig_yuv6);
}
