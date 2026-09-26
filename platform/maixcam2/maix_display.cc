#include "maix_display.h"

#include "ax_middleware.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

using namespace maix::middleware::maixcam2;

namespace {

/* MaixCDK DisplayAx::__config_vo_param(rotate=1)과 같다. 패널은 480x640 DSI. */
void config_vo(ax_vo_param_t *param, int width, int height)
{
    std::memset(param, 0, sizeof(*param));
    AX_VO_SYNC_INFO_T sync = {};
    sync.u16Vact = 640; sync.u16Vbb = 30; sync.u16Vfb = 30;
    sync.u16Hact = 480; sync.u16Hbb = 30; sync.u16Hfb = 30;
    sync.u16Hpw = 40; sync.u16Vpw = 11; sync.u32Pclk = 24750;
    sync.bIdv = AX_TRUE; sync.bIhs = AX_FALSE; sync.bIvs = AX_TRUE;
    SAMPLE_VO_CONFIG_S cfg = {};
    cfg.u32VDevNr = 1;
    cfg.stVoDev[0].u32VoDev = 0;
    cfg.stVoDev[0].enMode = AX_VO_MODE_OFFLINE;
    cfg.stVoDev[0].enVoIntfType = AX_VO_INTF_DSI;
    cfg.stVoDev[0].enIntfSync = AX_VO_OUTPUT_USER;
    cfg.stVoDev[0].enVoOutfmt = AX_VO_OUT_FMT_UNUSED;
    cfg.stVoDev[0].u32SyncIndex = 2;
    for (auto &layer : cfg.stVoLayer) {
        layer.bindVoDev[0] = layer.bindVoDev[1] = SAMPLE_VO_DEV_MAX;
        layer.enChnFrmFmt = AX_FORMAT_YUV420_SEMIPLANAR;
    }
    cfg.stGraphicLayer[0].u32FbNum = 1;
    cfg.stGraphicLayer[0].stFbConf[0].u32Index = 0;
    cfg.stGraphicLayer[0].stFbConf[0].u32Fmt = AX_FORMAT_ARGB8888;
    cfg.stGraphicLayer[0].stFbConf[0].u32ResoW = height;
    cfg.stGraphicLayer[0].stFbConf[0].u32ResoH = width;
    cfg.stVoLayer[0].stVoLayerAttr.stImageSize.u32Width = height;
    cfg.stVoLayer[0].stVoLayerAttr.stImageSize.u32Height = width;
    cfg.stVoLayer[0].stVoLayerAttr.enPixFmt = AX_FORMAT_YUV420_SEMIPLANAR;
    cfg.stVoLayer[0].stVoLayerAttr.u32DispatchMode = 1;
    cfg.stVoLayer[0].stVoLayerAttr.f32FrmRate = 60.0f;
    cfg.stVoLayer[0].u32ChnNr = 1;
    std::memcpy(&param->vo_cfg, &cfg, sizeof(cfg));
    std::memcpy(&param->sync_info, &sync, sizeof(sync));
}

AX_POOL make_pool(AX_U64 block_size, AX_U32 count, bool cached = false)
{
    AX_POOL_CONFIG_T cfg = {};
    cfg.MetaSize = 512;
    cfg.BlkCnt = count;
    cfg.BlkSize = block_size;
    cfg.CacheMode = cached ? AX_POOL_CACHE_MODE_CACHED : AX_POOL_CACHE_MODE_NONCACHE;
    std::strcpy(reinterpret_cast<char *>(cfg.PartitionName), "anonymous");
    return AX_POOL_CreatePool(&cfg);
}

/* 16:9 소스의 가운데를 화면 비율(kWidth:kHeight = 4:3)로 자른다. IVPS 정렬(폭 16,
 * 위치 2)을 맞춘다. app_config.h kPreviewAspect와 같은 영역이다. */
void center_crop(AX_VIDEO_FRAME_T *frame, int width, int height)
{
    int crop_w = height * MaixDisplay::kWidth / MaixDisplay::kHeight;
    if (crop_w > width) crop_w = width;
    crop_w &= ~15;
    frame->s16CropX = static_cast<AX_S16>(((width - crop_w) / 2) & ~1);
    frame->s16CropY = 0;
    frame->s16CropWidth = static_cast<AX_S16>(crop_w);
    frame->s16CropHeight = static_cast<AX_S16>(height);
}

/* /boot/board의 key=value (disp_flip 등). 없으면 기본값. */
int board_config(const char *key, int fallback)
{
    std::ifstream file("/boot/board");
    std::string line;
    const std::string want = std::string(key) + "=";
    while (std::getline(file, line))
        if (line.compare(0, want.size(), want) == 0) return std::atoi(line.c_str() + want.size());
    return fallback;
}

/* 백라이트는 PWM3(pwmchip0/pwm3). MaixCDK 화면이 닫히며 0으로 두므로 다시 켠다.
 * 밝기는 /boot/configs maix_backlight_value(%) x disp_max_backlight(%). */
void enable_backlight()
{
    int percent = 50;
    std::ifstream configs("/boot/configs");
    std::string line;
    while (std::getline(configs, line))
        if (line.rfind("maix_backlight_value=", 0) == 0) percent = std::atoi(line.c_str() + 21);
    const int max_percent = board_config("disp_max_backlight", 95);
    const std::string pwm = "/sys/class/pwm/pwmchip0/pwm3";
    if (!std::ifstream(pwm + "/period")) std::ofstream("/sys/class/pwm/pwmchip0/export") << 3;
    const long period = 100000;
    std::ofstream(pwm + "/period") << period;
    std::ofstream(pwm + "/duty_cycle") << period * percent * max_percent / 10000;
    std::ofstream(pwm + "/enable") << 1;
}

} // namespace

// VO 생성자도 AX SYS 초기화 뒤에 만든다.
struct MaixDisplay::Impl {
    SYS sys{false};
    std::unique_ptr<VO> video_ptr;
    std::unique_ptr<VO> osd_ptr;
    int video_ch = -1;
    int osd_ch = -1;
    AX_POOL dst_pool = AX_INVALID_POOLID;
    int errors = 0;
    AX_POOL osd_pool = AX_INVALID_POOLID;
    Frame *osd_frame = nullptr;
};

MaixDisplay::MaixDisplay() : impl_(new Impl)
{
    if (impl_->sys.init() != maix::err::ERR_NONE) throw std::runtime_error("AX SYS init failed");
    impl_->video_ptr.reset(new VO());
    impl_->osd_ptr.reset(new VO());
    VO &video_vo = *impl_->video_ptr;
    VO &osd_vo = *impl_->osd_ptr;
    const int flip = board_config("disp_flip", 1);
    const int mirror = board_config("disp_mirror", 0);
    ax_vo_param_t param;
    config_vo(&param, kWidth, kHeight);
    if (video_vo.init(&param) != maix::err::ERR_NONE) throw std::runtime_error("VO init failed");
    ax_vo_channel_param_t video = {kWidth, kHeight, AX_FORMAT_YUV420_SEMIPLANAR,
                                   AX_FORMAT_YUV420_SEMIPLANAR_VU, 60, 0, mirror, flip, 0, 90, -1, -1};
    impl_->video_ch = video_vo.get_unused_channel(0);
    if (impl_->video_ch < 0 || video_vo.add_channel(0, impl_->video_ch, &video) != maix::err::ERR_NONE)
        throw std::runtime_error("VO video channel failed");

    config_vo(&param, kWidth, kHeight);
    if (osd_vo.init(&param) != maix::err::ERR_NONE) throw std::runtime_error("VO OSD init failed");
    ax_vo_channel_param_t osd = {kWidth, kHeight, AX_FORMAT_BGRA8888,
                                 AX_FORMAT_YUV420_SEMIPLANAR_VU, 60, 0, mirror, flip, 0, 90, -1, -1};
    impl_->osd_ch = osd_vo.get_unused_channel(1);
    if (impl_->osd_ch < 0 || osd_vo.add_channel(1, impl_->osd_ch, &osd) != maix::err::ERR_NONE)
        throw std::runtime_error("VO OSD channel failed");

    enable_backlight();
}

MaixDisplay::~MaixDisplay()
{
    delete impl_->osd_frame;
    // VO가 마지막으로 받은 블록을 채널을 닫을 때 돌려주므로 풀은 그 뒤에 없앤다.
    if (impl_->osd_ptr) {
        if (impl_->osd_ch >= 0) impl_->osd_ptr->del_channel(1, impl_->osd_ch);
        impl_->osd_ptr->deinit();
    }
    if (impl_->video_ptr) {
        if (impl_->video_ch >= 0) impl_->video_ptr->del_channel(0, impl_->video_ch);
        impl_->video_ptr->deinit();
    }
    impl_->osd_ptr.reset();
    impl_->video_ptr.reset();
    if (impl_->osd_pool != AX_INVALID_POOLID) AX_POOL_DestroyPool(impl_->osd_pool);
    if (impl_->dst_pool != AX_INVALID_POOLID) AX_POOL_DestroyPool(impl_->dst_pool);
    impl_->sys.deinit();
}

bool MaixDisplay::show_video_phys(unsigned long long phys, int width, int height, bool nv21,
                                  const std::function<bool()> &source_still_valid)
{
    // 풀은 첫 프레임 때 만든다. camerad가 VI를 열며 AX 공용 풀을 다시 설정하므로
    // 그 전에 만든 풀은 무효가 될 수 있다.
    if (impl_->dst_pool == AX_INVALID_POOLID) {
        impl_->dst_pool = make_pool(kWidth * kHeight * 3 / 2, 3);
        if (impl_->dst_pool == AX_INVALID_POOLID) return false;
    }
    bool ok = false;
    Frame *out = nullptr;
    try {
        out = new Frame(impl_->dst_pool, kWidth, kHeight, nullptr, 0, AX_FORMAT_YUV420_SEMIPLANAR);
        AX_VIDEO_FRAME_T src = {}, dst = {};
        src.u32Width = width;
        src.u32Height = height;
        src.enImgFormat = nv21 ? AX_FORMAT_YUV420_SEMIPLANAR_VU : AX_FORMAT_YUV420_SEMIPLANAR;
        src.u32PicStride[0] = src.u32PicStride[1] = width;
        src.u64PhyAddr[0] = phys;
        src.u64PhyAddr[1] = phys + static_cast<AX_U64>(width) * height;
        src.u32FrameSize = static_cast<AX_U32>(static_cast<size_t>(width) * height * 3 / 2);
        if (!letterbox_) center_crop(&src, width, height);
        out->get_video_frame(&dst);
        AX_IVPS_CROP_RESIZE_ATTR_T attr = {};
        attr.eSclType = AX_IVPS_SCL_TYPE_AUTO;
        attr.eSclInput = AX_IVPS_SCL_INPUT_SHARE;
        attr.tAspectRatio.eMode = letterbox_ ? AX_IVPS_ASPECT_RATIO_AUTO : AX_IVPS_ASPECT_RATIO_STRETCH;
        attr.tAspectRatio.eAligns[0] = AX_IVPS_ASPECT_RATIO_HORIZONTAL_CENTER;
        attr.tAspectRatio.eAligns[1] = AX_IVPS_ASPECT_RATIO_VERTICAL_CENTER;
        attr.tAspectRatio.nBgColor = 0x000000;
        const int ivps = AX_IVPS_CropResizeTdp(&src, &dst, &attr);
        ok = ivps == 0 && source_still_valid() &&
             impl_->video_ptr->push(0, impl_->video_ch, out) == maix::err::ERR_NONE;
        if (ivps != 0 && impl_->errors++ % 100 == 0)
            std::fprintf(stderr, "display: IVPS crop-resize failed 0x%x (%d so far)\n", ivps, impl_->errors);
    } catch (const std::exception &e) {
        if (impl_->errors++ % 100 == 0)
            std::fprintf(stderr, "display: video frame failed: %s\n", e.what());
        ok = false;
    }
    delete out;
    return ok;
}

uint8_t *MaixDisplay::begin_overlay()
{
    if (impl_->osd_pool == AX_INVALID_POOLID) {
        impl_->osd_pool = make_pool(kWidth * kHeight * 4, 3, true);
        if (impl_->osd_pool == AX_INVALID_POOLID) return nullptr;
    }
    delete impl_->osd_frame;
    impl_->osd_frame = nullptr;
    try {
        impl_->osd_frame = new Frame(impl_->osd_pool, kWidth, kHeight, nullptr, 0, AX_FORMAT_BGRA8888);
    } catch (...) {
        return nullptr;
    }
    return static_cast<uint8_t *>(impl_->osd_frame->data);
}

bool MaixDisplay::end_overlay()
{
    Frame *frame = impl_->osd_frame;
    impl_->osd_frame = nullptr;
    if (!frame) return false;
    AX_VIDEO_FRAME_T info = {};
    frame->get_video_frame(&info);
    AX_SYS_MflushCache(info.u64PhyAddr[0], frame->data, static_cast<AX_U32>(kWidth * kHeight * 4));
    const bool ok = impl_->osd_ptr->push(1, impl_->osd_ch, frame) == maix::err::ERR_NONE;
    delete frame;
    return ok;
}
