#include "maix_camera.h"

#include "ax_middleware.hpp"
#include "maix_app.hpp"

#include <linux/videodev2.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

using namespace maix::middleware::maixcam2;

// VI 생성자는 AX SYS가 이미 초기화돼 있어야 하므로 SYS init 뒤에 만든다.
struct MaixCamera::Impl {
    SYS sys{false};
    std::unique_ptr<VI> vi_ptr;
    int channel = -1;
    VI &vi_ref() { return *vi_ptr; }
};

MaixCamera::MaixCamera(int width, int height, int fps, bool nv12)
    : impl_(new Impl), width_(width), height_(height), nv12_(nv12)
{
    if (impl_->sys.init() != maix::err::ERR_NONE) throw std::runtime_error("AX SYS init failed");
    impl_->vi_ptr.reset(new VI());
    VI &vi = impl_->vi_ref();
    auto sensor = vi.get_sensor_name();
    if (!sensor.first) throw std::runtime_error("camera sensor not found");

    // MaixCDK Camera::open와 같은 순서. 센서 모드는 이름으로 고른다(os04d10 2560x1440@30).
    int sensor_w = width, sensor_h = height, sensor_fps = fps;
    SAMPLE_VIN_PARAM_T vin = {};
    vin.eSysCase = vi.get_vi_case(const_cast<char *>(sensor.second.c_str()),
                                         sensor_w, sensor_h, sensor_fps);
    vin.eSysMode = COMMON_VIN_SENSOR;
    vin.eHdrMode = AX_SNS_LINEAR_MODE;
    vin.eLoadRawNode = LOAD_RAW_IFE;
    /* AI-ISP(NPU 한 코어를 쓰는 AI 노이즈 제거)는 늘 켠다. NPU 분할은 부팅 때
     * /boot/configs maix_npu_ai_isp=1로 정해지므로(install_autostart.sh가 켠다) 그 설정이
     * 없으면 여기서 멈춘다. modeld는 남은 코어 하나용(NPU1) 모델을 쓴다. */
    if (maix::app::get_sys_config_kv("npu", "ai_isp", "0") != "1")
        throw std::runtime_error("AI-ISP needs maix_npu_ai_isp=1 in /boot/configs (run install_autostart.sh, then reboot)");
    vin.bAiispEnable = AX_TRUE;
    COMMON_SYS_ARGS_T common = {}, priv = {};
    vi.config_sample_case(&vin, &common, &priv);
    if (vi.init() != maix::err::ERR_NONE) throw std::runtime_error("AX VI init failed");

    impl_->channel = vi.get_unused_channel();
    if (impl_->channel < 0) throw std::runtime_error("no free VI channel");
    const AX_IMG_FORMAT_E format = nv12 ? AX_FORMAT_YUV420_SEMIPLANAR : AX_FORMAT_YUV420_SEMIPLANAR_VU;
    // fit=2는 MaixCDK 기본(센서 전체 화각을 비율 유지로 채움). 16:9 센서라 크롭이 없다.
    if (vi.add_channel(impl_->channel, width, height, format, fps, 3, false, false, 2) !=
        maix::err::ERR_NONE)
        throw std::runtime_error("AX VI add_channel failed");

    /* 센서 자체를 요청 fps로 돌린다(AX_ISP_SetSnsAttr). ISP_RUN 스레드의 CPU는 센서
     * 프레임 수에 비례하므로 30 fps에서 버리던 프레임의 처리 비용이 없어지고, 모델은
     * 정확히 50 ms 간격의 프레임을 받는다. 자동 노출은 켜진 채로 둔다(MaixCDK set_fps는
     * 노출을 프레임 전체로 고정해 영상이 날아간다). 최대 노출은 33 ms로 묶어 30 fps
     * 때보다 움직임 번짐이 늘지 않게 한다(EDGEPILOT_MAX_SHUTTER_US). */
    AX_SNS_ATTR_T sns = {};
    if (AX_ISP_GetSnsAttr(0, &sns) == 0 && sns.fFrameRate != static_cast<AX_F32>(fps)) {
        sns.fFrameRate = static_cast<AX_F32>(fps);
        if (AX_ISP_SetSnsAttr(0, &sns) != 0) throw std::runtime_error("sensor frame rate change failed");
    }
    const char *shutter_env = std::getenv("EDGEPILOT_MAX_SHUTTER_US");
    const AX_U32 max_shutter = shutter_env ? static_cast<AX_U32>(std::atoi(shutter_env)) : 33333;
    AX_ISP_IQ_AE_PARAM_T ae = {};
    if (max_shutter > 0 && AX_ISP_IQ_GetAeParam(0, &ae) == 0 && ae.tAeAlgAuto.nMaxShutter > max_shutter) {
        ae.tAeAlgAuto.nMaxShutter = max_shutter;
        AX_ISP_IQ_SetAeParam(0, &ae);
    }
    /* os04d10 센서 라이브러리가 AE 갱신마다(프레임마다) 게인 표 경계 오류를 찍는다
     * ("value:253 beyond the range:(0, 55)", 화질 영향 없음). 초당 20줄이 SD에 쌓이고
     * 로그 데몬 CPU를 쓰므로 SENSOR 모듈(id 34)만 오류 수준 미만으로 거른다. */
    std::ofstream("/proc/ax_proc/logctl") << "ulog 34 2";
    std::fprintf(stderr, "camera: sensor %d fps, AE %s, max shutter %u us\n", fps,
                 ae.nEnable ? "auto" : "manual", ae.tAeAlgAuto.nMaxShutter);
}

MaixCamera::~MaixCamera()
{
    if (impl_->vi_ptr) {
        if (impl_->channel >= 0) impl_->vi_ptr->del_channel_all();
        impl_->vi_ptr->deinit();
        impl_->vi_ptr.reset();
    }
    impl_->sys.deinit();
}

uint32_t MaixCamera::fourcc() const
{
    return nv12_ ? V4L2_PIX_FMT_NV12 : V4L2_PIX_FMT_NV21;
}

bool MaixCamera::read_to(unsigned long long dst_phys, int timeout_ms)
{
    Frame *frame = impl_->vi_ptr->pop(impl_->channel, timeout_ms);
    if (!frame) return false;
    AX_VIDEO_FRAME_T src = {};
    frame->get_video_frame(&src);
    AX_VIDEO_FRAME_T dst = {};
    dst.u32Width = width_;
    dst.u32Height = height_;
    dst.enImgFormat = nv12_ ? AX_FORMAT_YUV420_SEMIPLANAR : AX_FORMAT_YUV420_SEMIPLANAR_VU;
    dst.u32PicStride[0] = dst.u32PicStride[1] = width_;
    dst.u64PhyAddr[0] = dst_phys;
    dst.u64PhyAddr[1] = dst_phys + static_cast<AX_U64>(width_) * height_;
    dst.u32FrameSize = static_cast<AX_U32>(static_cast<size_t>(width_) * height_ * 3 / 2);
    AX_IVPS_CROP_RESIZE_ATTR_T attr = {};
    attr.eSclType = AX_IVPS_SCL_TYPE_AUTO;
    attr.eSclInput = AX_IVPS_SCL_INPUT_SHARE;
    attr.tAspectRatio.eMode = AX_IVPS_ASPECT_RATIO_STRETCH;
    const bool ok = AX_IVPS_CropResizeTdp(&src, &dst, &attr) == 0;
    delete frame;
    return ok;
}
