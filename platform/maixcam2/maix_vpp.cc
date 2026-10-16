#include "maixcam2/maix_vpp.h"

#include "ax_ivps_api.h"

namespace {

AX_VIDEO_FRAME_T frame(const VppNv12 &f, bool vu) {
    AX_VIDEO_FRAME_T out = {};
    out.u32Width = static_cast<AX_U32>(f.width);
    out.u32Height = static_cast<AX_U32>(f.height);
    out.enImgFormat = vu ? AX_FORMAT_YUV420_SEMIPLANAR_VU : AX_FORMAT_YUV420_SEMIPLANAR;
    out.u32PicStride[0] = out.u32PicStride[1] = static_cast<AX_U32>(f.width);
    out.u64PhyAddr[0] = f.phys;
    out.u64PhyAddr[1] = f.phys + static_cast<AX_U64>(f.width) * static_cast<AX_U64>(f.height);
    out.u32FrameSize = static_cast<AX_U32>(static_cast<uint64_t>(f.width) * static_cast<uint64_t>(f.height) * 3 / 2);
    return out;
}

}  // namespace

int vpp_crop_resize_nv12(const VppNv12 &src, int x, int y, int w, int h, const VppNv12 &dst) {
    if (((x | y | w | h | dst.width | dst.height) & 1) != 0 || w <= 0 || h <= 0 || x < 0 || y < 0 ||
        x + w > src.width || y + h > src.height)
        return -1;
    const AX_VIDEO_FRAME_T in = frame(src, src.chroma_vu);
    AX_VIDEO_FRAME_T out = frame(dst, false);
    AX_IVPS_RECT_T box = {};
    box.nX = static_cast<AX_S16>(x);
    box.nY = static_cast<AX_S16>(y);
    box.nW = static_cast<AX_U16>(w);
    box.nH = static_cast<AX_U16>(h);
    // 2026-10-15 보드 실측: VPP bilinear는 1280x288 -> 640x320에 1.1 ms이고 OpenCV INTER_LINEAR와 평균 0.6 LSB
    // 다르다. TDP는 이 crop-resize를 받지 않고(bInplace/rotate not support) 실패한 호출이 camerad의 IVPS
    // 파이프라인까지 멈춰 세웠으니 쓰지 않는다.
    AX_IVPS_CROP_RESIZE_ATTR_T attr = {};
    attr.eSclType = AX_IVPS_SCL_TYPE_BILINEAR;
    attr.eSclInput = AX_IVPS_SCL_INPUT_SHARE;
    attr.tAspectRatio.eMode = AX_IVPS_ASPECT_RATIO_STRETCH;
    std::lock_guard<std::mutex> lock(vpp_mutex());
    return AX_IVPS_CropResizeV2Vpp(&in, 1, &box, &out, &attr);
}
