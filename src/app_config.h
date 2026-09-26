#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <string>

constexpr unsigned kDefaultSensorWidth = 1920;
constexpr unsigned kDefaultSensorHeight = 1080;
/* AI 캡처 기본 해상도. MaixCAM2(ov_os04d10, 가로 화각 80.6°)에서 720p는
 * 13.2 px/°로 supercombo 학습 카메라(17.9 px/°)보다 거칠다. 1080p면 19.8 px/°
 * 지만 파이프·워프 비용을 아직 재지 않았다. */
constexpr unsigned kDefaultAiWidth = 1280;
constexpr unsigned kDefaultAiHeight = 720;
constexpr unsigned kDefaultModelWidth = 512;
constexpr unsigned kDefaultModelHeight = 256;
constexpr float kDefaultModelFx = 910.0f;
constexpr float kDefaultModelFy = 910.0f;
constexpr float kDefaultModelCx = 256.0f;
constexpr float kDefaultModelCy = 47.6f;
/* MaixCAM2(ov_os04d10) 카메라 내부 파라미터, 1920x1080 기준. 런타임과 같은 카메라
 * 경로(k230_camcal: 센서 전체 화각 축소, 보드 cam_flip/mirror)로 65" TV 11x6
 * 체스보드를 찍어 구했다(2026-09-26, 42장, 재투영 0.52 px, 1σ fx/fy ±2, cx ±2.6,
 * cy ±1.7; docs/camcal.md). 모든 출력 해상도가 센서 전체 화각의 축소라 비례 환산된다.
 * 왜곡(k1 -0.009, k2 0.013, k3 -0.021)은 평균 2.6 px라 워프는 핀홀로 둔다.
 * 이전 값(순정 카메라 앱으로 측정, 1132.3/1131.5/932.8/556.1)과 초점거리는 같고
 * cx가 7 px 달랐다. */
constexpr float kCameraFx = 1131.24f;
constexpr float kCameraFy = 1130.85f;
constexpr float kCameraCx = 940.13f;
constexpr float kCameraCy = 552.60f;
/* HUD 미리보기는 16:9 카메라 영상의 가운데를 이 비율(4:3, MaixCAM2 LCD)로 잘라
 * 늘림 없이 보여 준다. overlayd의 IVPS 크롭과 차선 투영(projection.cc)이 함께 쓴다.
 * 1920x1080 기준 크롭 폭 = 1080 * 4/3 = 1440. */
constexpr float kPreviewAspect = 4.0f / 3.0f;
constexpr float kPreviewCropWidth1080 = static_cast<float>(kDefaultSensorHeight) * kPreviewAspect;

constexpr float default_input_warp_fx(unsigned source_width)
{
    return kCameraFx * static_cast<float>(source_width) /
        static_cast<float>(kDefaultSensorWidth);
}
constexpr float default_input_warp_fy(unsigned source_height)
{
    return kCameraFy * static_cast<float>(source_height) /
        static_cast<float>(kDefaultSensorHeight);
}
constexpr float default_input_warp_cx(unsigned source_width)
{
    return kCameraCx * static_cast<float>(source_width) /
        static_cast<float>(kDefaultSensorWidth);
}
constexpr float default_input_warp_cy(unsigned source_height)
{
    return kCameraCy * static_cast<float>(source_height) /
        static_cast<float>(kDefaultSensorHeight);
}
constexpr float kDefaultInputWarpFx = default_input_warp_fx(kDefaultAiWidth);
constexpr float kDefaultInputWarpFy = default_input_warp_fy(kDefaultAiHeight);
constexpr float kDefaultInputWarpCx = default_input_warp_cx(kDefaultAiWidth);
constexpr float kDefaultInputWarpCy = default_input_warp_cy(kDefaultAiHeight);

struct AppConfig {
    std::string axmodel_path;

    unsigned nv12_width = kDefaultAiWidth;
    unsigned nv12_height = kDefaultAiHeight;
    unsigned nv12_crop_x = 0;
    unsigned nv12_crop_y = 0;
    unsigned nv12_crop_width = kDefaultSensorWidth;
    unsigned nv12_crop_height = kDefaultSensorHeight;
    unsigned max_frames = 0;

    std::string replay_nv12_path;

    bool calibration_auto = true;
    bool manual_calibration = false;
    float manual_roll = 0.0f;
    float manual_pitch = 0.0f;
    float manual_yaw = 0.0f;
    bool log_calibration = false;
    bool profile = false;

    /* 카메라 내부 파라미터(1920x1080 기준). SUPERCOMBO_CAMERA_INTRINSICS="fx,fy,cx,cy"로
     * 바꿀 수 있다(예: K230 녹화 리플레이에는 K230 카메라 값). input_warp_*는
     * 이 값을 캡처 해상도로 비례 환산한 것이다. */
    float camera_fx = kCameraFx;
    float camera_fy = kCameraFy;
    float camera_cx = kCameraCx;
    float camera_cy = kCameraCy;
    float input_warp_fx = kDefaultInputWarpFx;
    float input_warp_fy = kDefaultInputWarpFy;
    float input_warp_cx = kDefaultInputWarpCx;
    float input_warp_cy = kDefaultInputWarpCy;
    float input_warp_height = 1.22f;

    static AppConfig from_env(int argc, char *argv[]);
    static AppConfig from_env_defaults();
    // camera_*를 width x height 소스로 환산해 input_warp_*에 넣는다.
    void set_warp_source(unsigned width, unsigned height);
    static std::string usage(const char *program_name);

    bool replay_enabled() const { return !replay_nv12_path.empty(); }
};

#endif
