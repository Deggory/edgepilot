#include "common/projection.h"
#include "common/app_config.h"

#include <cmath>

void rotation_from_rpy(float roll, float pitch, float yaw, float *rot) {
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);

    rot[0] = cy * cp;
    rot[1] = cy * sp * sr - sy * cr;
    rot[2] = cy * sp * cr + sy * sr;
    rot[3] = sy * cp;
    rot[4] = sy * sp * sr + cy * cr;
    rot[5] = sy * sp * cr - cy * sr;
    rot[6] = -sp;
    rot[7] = cp * sr;
    rot[8] = cp * cr;
}

ProjectionState make_projection_state(float roll, float pitch, float yaw) {
    ProjectionState state{};
    state.roll = roll;
    state.pitch = pitch;
    state.yaw = yaw;

    float rot[9] = {};
    rotation_from_rpy(roll, pitch, yaw, rot);

    // 렌더러는 모델 화소를 180도 돌린다. 도로 축 뒤집기를 앞에 붙이면 어떤 RPY에서도 그 표시 변환이
    // 상쇄된다.
    float device_from_calib[9] = {};
    for (int col = 0; col < 3; ++col) {
        device_from_calib[0 * 3 + col] = rot[0 * 3 + col];
        device_from_calib[1 * 3 + col] = -rot[1 * 3 + col];
        device_from_calib[2 * 3 + col] = -rot[2 * 3 + col];
    }

    // view_from_device = [[0,1,0],[0,0,1],[1,0,0]]
    for (int col = 0; col < 3; ++col) {
        state.view_from_calib[0 * 3 + col] = device_from_calib[1 * 3 + col];
        state.view_from_calib[1 * 3 + col] = device_from_calib[2 * 3 + col];
        state.view_from_calib[2 * 3 + col] = device_from_calib[0 * 3 + col];
    }

    return state;
}

namespace {

struct CameraIntrinsics {
    float fx = kCameraFx;
    float fy = kCameraFy;
    float cx = kCameraCx;
    float cy = kCameraCy;
};
CameraIntrinsics g_camera;

}  // namespace

void projection_set_camera_intrinsics(float fx, float fy, float cx, float cy) { g_camera = {fx, fy, cx, cy}; }

bool project_point_subpixel(const ProjectionState &projection, float x_forward, float y_right, float z_down, int width,
                            int height, float *px, float *py) {
    if (x_forward < 0.5f || x_forward > 120.0f) return false;
    // 가상 카메라가 오프셋만큼 오른쪽이라 모델은 실제 점을 y − 오프셋으로 본다. 실제 위치로 되돌린다.
    y_right += projection.lateral_offset_m;

    const float *m = projection.view_from_calib;
    const float vx = m[0] * x_forward + m[1] * y_right + m[2] * z_down;
    const float vy = m[3] * x_forward + m[4] * y_right + m[5] * z_down;
    const float vz = m[6] * x_forward + m[7] * y_right + m[8] * z_down;
    if (vz <= 0.1f) return false;

    // 화면(가로, MaixCAM2 640x480)은 카메라 영상 가운데의 kPreviewAspect 영역(1080p 기준 폭
    // kPreviewCropWidth1080)을 담는다(overlayd가 IVPS로 같은 영역을 자른다). 세로 패널로 돌리는 것은
    // HudCanvas가 한다.
    const float screen_w = static_cast<float>(width);
    const float screen_h = static_cast<float>(height);
    const float sx = screen_w / kPreviewCropWidth1080;
    const float sy = screen_h / static_cast<float>(kDefaultSensorHeight);
    const float crop_x = (static_cast<float>(kDefaultSensorWidth) - kPreviewCropWidth1080) * 0.5f;
    const float fx = g_camera.fx * sx;
    const float fy = g_camera.fy * sy;
    const float calibrated_cx = (g_camera.cx - crop_x) * sx;
    const float calibrated_cy = g_camera.cy * sy;
    const float cx = screen_w - 1.0f - calibrated_cx;
    const float cy = screen_h - 1.0f - calibrated_cy;

    *px = fx * vx / vz + cx;
    *py = fy * vy / vz + cy;
    return *px > -200.0f && *px < static_cast<float>(width + 200) && *py > -200.0f &&
           *py < static_cast<float>(height + 200);
}

void frame_pixel_to_screen(float u, float v, int image_w, int image_h, int screen_w, int screen_h, float *x, float *y) {
    // 1080p 기준으로 옮긴 뒤 project_point_subpixel과 같은 가운데 자르기와 축척
    const float u1080 = u * static_cast<float>(kDefaultSensorWidth) / static_cast<float>(image_w);
    const float v1080 = v * static_cast<float>(kDefaultSensorHeight) / static_cast<float>(image_h);
    const float crop_x = (static_cast<float>(kDefaultSensorWidth) - kPreviewCropWidth1080) * 0.5f;
    *x = (u1080 - crop_x) * static_cast<float>(screen_w) / kPreviewCropWidth1080;
    *y = v1080 * static_cast<float>(screen_h) / static_cast<float>(kDefaultSensorHeight);
}
