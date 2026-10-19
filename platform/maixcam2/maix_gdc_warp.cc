#include "maixcam2/maix_gdc_warp.h"

#include "maixcam2/maix_frames.h"
#include "model/yuv6_pack.h"

#include <cmath>
#include <stdexcept>

namespace {

constexpr int kModelW = 512;
constexpr int kModelH = 256;
constexpr size_t kDstBytes = static_cast<size_t>(kModelW) * kModelH * 3 / 2;

// 가상 주소도 싣는다(CPU 주소를 아는 버퍼).
AX_VIDEO_FRAME_T mapped_nv12_frame(uint64_t phys, uint8_t *virt, int w, int h) {
    AX_VIDEO_FRAME_T f = nv12_frame(phys, w, h);
    f.u64VirAddr[0] = reinterpret_cast<uintptr_t>(virt);
    f.u64VirAddr[1] = reinterpret_cast<uintptr_t>(virt) + static_cast<uint64_t>(w) * h;
    return f;
}

}  // namespace

GdcWarp::GdcWarp(int src_width, int src_height)
    : src_w_(src_width), src_h_(src_height), src_bytes_(static_cast<size_t>(src_width) * src_height * 3 / 2) {
    if (AX_IVPS_Init() != 0) throw std::runtime_error("AX_IVPS_Init failed");
    for (int i = 0; i < 2; ++i) {
        void *virt = nullptr;
        if (AX_SYS_MemAllocCached(&dst_phys_[i], &virt, static_cast<AX_U32>(kDstBytes), 128,
                                  reinterpret_cast<const AX_S8 *>("gdc_dst")) != 0)
            throw std::runtime_error("GDC output buffer alloc failed");
        dst_virt_[i] = static_cast<uint8_t *>(virt);
    }
}

GdcWarp::~GdcWarp() {
    for (int i = 0; i < 2; ++i) AX_SYS_MemFree(dst_phys_[i], dst_virt_[i]);
    if (src_virt_) AX_SYS_MemFree(src_phys_, src_virt_);
    AX_IVPS_Deinit();
}

uint8_t *GdcWarp::source() {
    if (!src_virt_) {
        void *virt = nullptr;
        if (AX_SYS_MemAllocCached(&src_phys_, &virt, static_cast<AX_U32>(src_bytes_), 128,
                                  reinterpret_cast<const AX_S8 *>("gdc_src")) != 0)
            throw std::runtime_error("GDC source buffer alloc failed");
        src_virt_ = static_cast<uint8_t *>(virt);
    }
    return src_virt_;
}

bool GdcWarp::warp_one(unsigned long long src_phys, const float projection[9], int dst, uint8_t *yuv6) {
    AX_IVPS_DEWARP_ATTR_T attr = {};
    attr.eDewarpType = AX_IVPS_DEWARP_PERSPECTIVE;
    const double scale = projection[8] != 0.0f ? 1.0 / projection[8] : 1.0;
    for (int i = 0; i < 9; ++i)  // 소수 6자리 고정소수점
        attr.tPerspectiveAttr.nMatrix[i] = static_cast<AX_S64>(std::llround(projection[i] * scale * 1e6));
    AX_VIDEO_FRAME_T src = mapped_nv12_frame(src_phys, src_phys == src_phys_ ? src_virt_ : nullptr, src_w_, src_h_);
    AX_VIDEO_FRAME_T out = mapped_nv12_frame(dst_phys_[dst], dst_virt_[dst], kModelW, kModelH);
    if (AX_IVPS_Dewarp(&src, &out, &attr) != 0) return false;
    AX_SYS_MinvalidateCache(dst_phys_[dst], dst_virt_[dst], static_cast<AX_U32>(kDstBytes));

    nv12_to_yuv6(dst_virt_[dst], yuv6, kModelW, kModelH);  // NV12 512x256 -> YUV6
    return true;
}

bool GdcWarp::warp(unsigned long long src_phys, const float med_projection[9], const float sbig_projection[9],
                   uint8_t *med_yuv6, uint8_t *sbig_yuv6) {
    if (src_virt_ && src_phys == src_phys_) AX_SYS_MflushCache(src_phys_, src_virt_, static_cast<AX_U32>(src_bytes_));
    return warp_one(src_phys, med_projection, 0, med_yuv6) && warp_one(src_phys, sbig_projection, 1, sbig_yuv6);
}
