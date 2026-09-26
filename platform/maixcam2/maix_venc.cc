#include "maix_venc.h"

#include "ax_ivps_api.h"
#include "ax_pool_type.h"
#include "ax_sys_api.h"
#include "ax_venc_api.h"

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {

constexpr VENC_CHN kChannel = 0;
constexpr AX_U32 kPoolBlocks = 6;           // 인코더 입력 FIFO(4) + 복사 중 1 + 여유 1
constexpr unsigned kPoolRebuildFailures = 20;  // camerad 재시작으로 풀이 무효가 되면 다시 만든다

AX_POOL make_pool(AX_U64 block_size)
{
    AX_POOL_CONFIG_T cfg = {};
    cfg.MetaSize = 512;
    cfg.BlkCnt = kPoolBlocks;
    cfg.BlkSize = block_size;
    cfg.CacheMode = AX_POOL_CACHE_MODE_NONCACHE;
    std::strcpy(reinterpret_cast<char *>(cfg.PartitionName), "anonymous");
    return AX_POOL_CreatePool(&cfg);
}

AX_VIDEO_FRAME_T nv12(int w, int h, AX_U64 phys)
{
    AX_VIDEO_FRAME_T f = {};
    f.u32Width = w;
    f.u32Height = h;
    f.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    f.u32PicStride[0] = f.u32PicStride[1] = w;
    f.u64PhyAddr[0] = phys;
    f.u64PhyAddr[1] = phys + static_cast<AX_U64>(w) * h;
    f.u32FrameSize = static_cast<AX_U32>(static_cast<size_t>(w) * h * 3 / 2);
    return f;
}

} // namespace

struct VideoEncoder::Impl {
    Codec codec = Codec::H264;
    int width = 0;
    int height = 0;
    AX_U64 frame_bytes = 0;
    AX_POOL pool = AX_INVALID_POOLID;
    unsigned copy_failures = 0;
    bool config_sent = false;
    std::vector<uint8_t> config;   // 파라미터 세트 (Annex B)
    std::vector<uint8_t> payload;  // 파라미터 세트를 뺀 프레임
    bool ivps = false, venc = false, channel = false, receiving = false;
};

VideoEncoder::VideoEncoder(Codec codec, int width, int height, int fps, unsigned bitrate) : impl_(new Impl)
{
    Impl &m = *impl_;
    m.codec = codec;
    m.width = width;
    m.height = height;
    m.frame_bytes = static_cast<AX_U64>(width) * height * 3 / 2;
    if (AX_SYS_Init() != 0) throw std::runtime_error("AX_SYS_Init failed");
    if (AX_IVPS_Init() != 0) throw std::runtime_error("AX_IVPS_Init failed");
    m.ivps = true;

    AX_VENC_MOD_ATTR_T mod = {};
    mod.enVencType = AX_VENC_VIDEO_ENCODER;
    mod.stModThdAttr.u32TotalThreadNum = 1;
    if (AX_VENC_Init(&mod) != 0) throw std::runtime_error("AX_VENC_Init failed");
    m.venc = true;

    AX_VENC_CHN_ATTR_T attr = {};
    attr.stVencAttr.enType = codec == Codec::HEVC ? PT_H265 : PT_H264;
    attr.stVencAttr.u32PicWidthSrc = width;
    attr.stVencAttr.u32PicHeightSrc = height;
    attr.stVencAttr.u32MaxPicWidth = width;
    attr.stVencAttr.u32MaxPicHeight = height;
    attr.stVencAttr.enLinkMode = AX_UNLINK_MODE;
    attr.stVencAttr.enMemSource = AX_MEMORY_SOURCE_CMM;
    attr.stVencAttr.u8InFifoDepth = 4;
    attr.stVencAttr.u8OutFifoDepth = 4;
    attr.stVencAttr.u32BufSize = static_cast<AX_U32>(m.frame_bytes);  // 한 프레임 크기면 충분
    attr.stRcAttr.s32FirstFrameStartQp = -1;
    attr.stRcAttr.stFrameRate.fSrcFrameRate = static_cast<AX_F32>(fps);
    attr.stRcAttr.stFrameRate.fDstFrameRate = static_cast<AX_F32>(fps);
    // CBR, 1초마다 IDR: 세그먼트를 1초 단위로 자를 수 있다.
    if (codec == Codec::HEVC) {
        attr.stVencAttr.enProfile = AX_VENC_HEVC_MAIN_PROFILE;
        attr.stVencAttr.enLevel = AX_VENC_HEVC_LEVEL_5_1;
        attr.stVencAttr.enTier = AX_VENC_HEVC_MAIN_TIER;
        attr.stRcAttr.enRcMode = AX_VENC_RC_MODE_H265CBR;
        AX_VENC_H265_CBR_T &cbr = attr.stRcAttr.stH265Cbr;
        cbr.u32Gop = static_cast<AX_U32>(fps);
        cbr.u32BitRate = bitrate / 1000;  // kbps
        cbr.u32MinQp = cbr.u32MinIQp = 10;
        cbr.u32MaxQp = cbr.u32MaxIQp = 51;
        cbr.u32MaxIprop = 40;
        cbr.u32MinIprop = 30;
        cbr.s32IntraQpDelta = -2;
    } else {
        attr.stVencAttr.enProfile = AX_VENC_H264_MAIN_PROFILE;
        attr.stVencAttr.enLevel = AX_VENC_H264_LEVEL_5_1;
        attr.stRcAttr.enRcMode = AX_VENC_RC_MODE_H264CBR;
        AX_VENC_H264_CBR_T &cbr = attr.stRcAttr.stH264Cbr;
        cbr.u32Gop = static_cast<AX_U32>(fps);
        cbr.u32BitRate = bitrate / 1000;  // kbps
        cbr.u32MinQp = cbr.u32MinIQp = 10;
        cbr.u32MaxQp = cbr.u32MaxIQp = 51;
        cbr.u32MaxIprop = 40;
        cbr.u32MinIprop = 10;
        cbr.s32IntraQpDelta = -2;
    }
    attr.stGopAttr.enGopMode = AX_VENC_GOPMODE_NORMALP;
    if (AX_VENC_CreateChn(kChannel, &attr) != 0) throw std::runtime_error("AX_VENC_CreateChn failed");
    m.channel = true;
    AX_VENC_RECV_PIC_PARAM_T recv = {};
    recv.s32RecvPicNum = -1;
    if (AX_VENC_StartRecvFrame(kChannel, &recv) != 0) throw std::runtime_error("AX_VENC_StartRecvFrame failed");
    m.receiving = true;
}

VideoEncoder::~VideoEncoder()
{
    Impl &m = *impl_;
    if (m.receiving) AX_VENC_StopRecvFrame(kChannel);
    if (m.channel) AX_VENC_DestroyChn(kChannel);
    if (m.venc) AX_VENC_Deinit();
    if (m.pool != AX_INVALID_POOLID) AX_POOL_DestroyPool(m.pool);
    if (m.ivps) AX_IVPS_Deinit();
}

bool VideoEncoder::submit(unsigned long long src_phys, uint64_t frame_id,
                      const std::function<bool()> &source_still_valid)
{
    Impl &m = *impl_;
    // 풀은 처음 쓸 때 만든다: camerad가 VI를 열며 AX 공용 풀을 다시 설정하면 그 전에
    // 만든 풀은 무효가 된다.
    if (m.pool == AX_INVALID_POOLID || m.copy_failures >= kPoolRebuildFailures) {
        if (m.pool != AX_INVALID_POOLID) AX_POOL_DestroyPool(m.pool);
        m.pool = make_pool(m.frame_bytes);
        m.copy_failures = 0;
        if (m.pool == AX_INVALID_POOLID) return false;
    }
    const AX_BLK block = AX_POOL_GetBlock(m.pool, m.frame_bytes, nullptr);
    if (block == AX_INVALID_BLOCKID) return false;  // 인코더가 입력을 다 쥐고 있다

    AX_VIDEO_FRAME_T src = nv12(m.width, m.height, src_phys);
    AX_VIDEO_FRAME_T dst = nv12(m.width, m.height, AX_POOL_Handle2PhysAddr(block));
    dst.u32BlkId[0] = block;
    AX_IVPS_CROP_RESIZE_ATTR_T resize = {};
    resize.eSclType = AX_IVPS_SCL_TYPE_AUTO;
    resize.eSclInput = AX_IVPS_SCL_INPUT_SHARE;
    resize.tAspectRatio.eMode = AX_IVPS_ASPECT_RATIO_STRETCH;
    const bool copied = AX_IVPS_CropResizeTdp(&src, &dst, &resize) == 0;
    m.copy_failures = copied ? 0 : m.copy_failures + 1;
    bool sent = false;
    if (copied && source_still_valid()) {
        AX_VIDEO_FRAME_INFO_T frame = {};
        frame.stVFrame = dst;
        frame.stVFrame.u64SeqNum = frame_id;
        frame.stVFrame.u64PTS = frame_id;
        frame.stVFrame.u64UserData = frame_id;
        sent = AX_VENC_SendFrame(kChannel, &frame, 0) == 0;
    }
    // 인코더가 받았으면 제 참조를 따로 잡는다. 우리 참조는 여기서 푼다.
    AX_POOL_ReleaseBlock(block);
    if (sent) ++submitted_;
    return sent;
}

unsigned VideoEncoder::drain(const std::function<void(const uint8_t *, size_t)> &on_config,
                         const std::function<void(const Packet &)> &on_packet, int timeout_ms)
{
    Impl &m = *impl_;
    unsigned count = 0;
    AX_VENC_STREAM_T stream;
    while (AX_VENC_GetStream(kChannel, &stream, count == 0 ? timeout_ms : 0) == 0) {
        const AX_VENC_PACK_T &pack = stream.stPack;
        bool keyframe = pack.enCodingType == AX_VENC_INTRA_FRAME;
        m.payload.clear();
        std::vector<uint8_t> config;
        for (AX_U32 i = 0; i < pack.u32NaluNum; ++i) {
            const AX_VENC_NALU_INFO_T &nalu = pack.stNaluInfo[i];
            const uint8_t *begin = pack.pu8Addr + nalu.u32NaluOffset;
            bool parameter_set, idr;
            if (m.codec == Codec::HEVC) {
                const AX_H265E_NALU_TYPE_E type = nalu.unNaluType.enH265EType;
                parameter_set = type == AX_H265E_NALU_VPS || type == AX_H265E_NALU_SPS ||
                                type == AX_H265E_NALU_PPS;
                idr = type == AX_H265E_NALU_IDRSLICE;
            } else {
                const AX_H264E_NALU_TYPE_E type = nalu.unNaluType.enH264EType;
                parameter_set = type == AX_H264E_NALU_SPS || type == AX_H264E_NALU_PPS;
                idr = type == AX_H264E_NALU_IDRSLICE;
            }
            if (parameter_set) {
                config.insert(config.end(), begin, begin + nalu.u32NaluLength);
            } else {
                if (idr) keyframe = true;
                m.payload.insert(m.payload.end(), begin, begin + nalu.u32NaluLength);
            }
        }
        if (pack.u32NaluNum == 0)  // NALU 정보가 없으면 통째로
            m.payload.assign(pack.pu8Addr, pack.pu8Addr + pack.u32Len);
        if (!config.empty() && config != m.config) {
            m.config = std::move(config);
            on_config(m.config.data(), m.config.size());
            m.config_sent = true;
        }
        const Packet packet{m.payload.data(), m.payload.size(), pack.u64SeqNum, keyframe};
        AX_VENC_ReleaseStream(kChannel, &stream);
        ++encoded_;
        ++count;
        if (m.config_sent && packet.size > 0) on_packet(packet);
    }
    return count;
}
