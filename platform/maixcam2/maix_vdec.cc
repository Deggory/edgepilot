#include "maix_vdec.h"

#include "ax_buffer_tool.h"
#include "ax_ivps_api.h"
#include "ax_pool_type.h"
#include "ax_sys_api.h"
#include "ax_vdec_api.h"

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

namespace {

constexpr AX_VDEC_GRP kGroup = 0;
constexpr AX_U32 kFrameBuffers = 8;
constexpr AX_U32 kWidthAlign = 16;  // MSP 샘플 AX_VDEC_WIDTH_ALIGN

std::string hex(AX_S32 value)
{
    char text[16];
    std::snprintf(text, sizeof(text), "0x%x", static_cast<unsigned>(value));
    return text;
}

} // namespace

struct VideoDecoder::Impl {
    int width = 0;
    int height = 0;
    AX_POOL pool = AX_INVALID_POOLID;
    bool ivps = false, vdec = false, group = false, receiving = false;
};

VideoDecoder::VideoDecoder(Codec codec, int width, int height) : impl_(new Impl)
{
    Impl &m = *impl_;
    m.width = width;
    m.height = height;
    const AX_PAYLOAD_TYPE_E type = codec == Codec::HEVC ? PT_H265 : PT_H264;
    if (AX_SYS_Init() != 0) throw std::runtime_error("AX_SYS_Init failed");
    if (AX_IVPS_Init() != 0) throw std::runtime_error("AX_IVPS_Init failed");
    m.ivps = true;
    AX_VDEC_MOD_ATTR_T mod = {};
    mod.u32MaxGroupCount = 1;
    AX_S32 ret = AX_VDEC_Init(&mod);
    if (ret != 0) throw std::runtime_error("AX_VDEC_Init failed " + hex(ret));
    m.vdec = true;

    AX_VDEC_GRP_ATTR_T attr = {};
    attr.enCodecType = type;
    attr.enInputMode = AX_VDEC_INPUT_MODE_FRAME;
    attr.enLinkMode = AX_UNLINK_MODE;
    attr.enOutOrder = AX_VDEC_OUTPUT_ORDER_DISP;
    attr.u32PicWidth = width;
    attr.u32PicHeight = height;
    attr.u32StreamBufSize = static_cast<AX_U32>(width) * height * 3 / 2;
    attr.enVdecVbSource = AX_POOL_SOURCE_USER;
    attr.u32FrameBufCnt = kFrameBuffers;
    attr.s32DestroyTimeout = 0;
    ret = AX_VDEC_CreateGrp(kGroup, &attr);
    if (ret != 0) throw std::runtime_error("AX_VDEC_CreateGrp failed " + hex(ret));
    m.group = true;

    const AX_U32 stride = AX_COMM_ALIGN(width * 8, kWidthAlign * 8) / 8;
    AX_POOL_CONFIG_T pool = {};
    pool.MetaSize = 512;
    pool.BlkCnt = kFrameBuffers;
    pool.BlkSize = AX_VDEC_GetPicBufferSize(stride, AX_COMM_ALIGN(height, 16), type);
    pool.CacheMode = AX_POOL_CACHE_MODE_NONCACHE;
    std::strcpy(reinterpret_cast<char *>(pool.PartitionName), "anonymous");
    m.pool = AX_POOL_CreatePool(&pool);
    if (m.pool == AX_INVALID_POOLID) throw std::runtime_error("VDEC frame pool failed");
    ret = AX_VDEC_AttachPool(kGroup, m.pool);
    if (ret != 0) throw std::runtime_error("AX_VDEC_AttachPool failed " + hex(ret));

    AX_VDEC_GRP_PARAM_T param = {};
    param.enVdecMode = VIDEO_DEC_MODE_IPB;
    AX_VDEC_SetGrpParam(kGroup, &param);
    AX_VDEC_RECV_PIC_PARAM_T recv = {};
    recv.s32RecvPicNum = -1;
    ret = AX_VDEC_StartRecvStream(kGroup, &recv);
    if (ret != 0) throw std::runtime_error("AX_VDEC_StartRecvStream failed " + hex(ret));
    m.receiving = true;
}

VideoDecoder::~VideoDecoder()
{
    Impl &m = *impl_;
    if (m.receiving) AX_VDEC_StopRecvStream(kGroup);
    if (m.group) {
        AX_VDEC_DetachPool(kGroup);
        AX_VDEC_DestroyGrp(kGroup);
    }
    if (m.pool != AX_INVALID_POOLID) AX_POOL_DestroyPool(m.pool);
    if (m.vdec) AX_VDEC_Deinit();
    if (m.ivps) AX_IVPS_Deinit();
}

bool VideoDecoder::send(const uint8_t *data, size_t size, uint64_t pts, int timeout_ms)
{
    AX_VDEC_STREAM_T stream = {};
    stream.u64PTS = pts;
    stream.bEndOfFrame = AX_TRUE;
    stream.u32StreamPackLen = static_cast<AX_U32>(size);
    stream.pu8Addr = const_cast<AX_U8 *>(data);
    return AX_VDEC_SendStream(kGroup, &stream, timeout_ms) == 0;
}

bool VideoDecoder::receive_to(unsigned long long dst_phys, uint64_t *pts, int timeout_ms)
{
    Impl &m = *impl_;
    AX_VIDEO_FRAME_INFO_T frame = {};
    if (AX_VDEC_GetFrame(kGroup, &frame, timeout_ms) != 0) return false;
    AX_VIDEO_FRAME_T dst = {};
    dst.u32Width = m.width;
    dst.u32Height = m.height;
    dst.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    dst.u32PicStride[0] = dst.u32PicStride[1] = m.width;
    dst.u64PhyAddr[0] = dst_phys;
    dst.u64PhyAddr[1] = dst_phys + static_cast<AX_U64>(m.width) * m.height;
    dst.u32FrameSize = static_cast<AX_U32>(static_cast<size_t>(m.width) * m.height * 3 / 2);
    AX_VIDEO_FRAME_T src = frame.stVFrame;
    src.s16CropX = src.s16CropY = 0;
    src.s16CropWidth = static_cast<AX_S16>(m.width);   // 디코더 버퍼는 정렬로 더 클 수 있다
    src.s16CropHeight = static_cast<AX_S16>(m.height);
    AX_IVPS_CROP_RESIZE_ATTR_T resize = {};
    resize.eSclType = AX_IVPS_SCL_TYPE_AUTO;
    resize.eSclInput = AX_IVPS_SCL_INPUT_SHARE;
    resize.tAspectRatio.eMode = AX_IVPS_ASPECT_RATIO_STRETCH;
    const bool ok = AX_IVPS_CropResizeTdp(&src, &dst, &resize) == 0;
    if (pts) *pts = frame.stVFrame.u64PTS;
    AX_VDEC_ReleaseFrame(kGroup, &frame);
    return ok;
}
