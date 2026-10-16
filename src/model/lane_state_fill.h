#ifndef MODEL_LANE_STATE_FILL_H
#define MODEL_LANE_STATE_FILL_H

// 차선선 플러그인 결과(EpLaneResult)를 LaneState로 옮긴다. 점은 1/4 화소 정수로 두고, 선과 도로 경계는 노면으로
// 옮겨 거리 격자 X_IDXS의 가로 위치(LaneState.road)로도 남긴다. 노면은 같은 프레임 supercombo가 낸 노면 높이이고
// (보정이 어긋나도 supercombo 차선과 같은 거리에 놓인다), 쓸 수 없으면 카메라 높이의 평평한 면이다. 노면 좌표는
// ModelState 차선과 같은 가상 카메라 기준(앞 x, 오른쪽 y)이다.

#include "common/ipc_messages.h"
#include "common/model_output.h"
#include "model/lane_plugin_abi.h"

#include <array>

// 프레임 화소의 핀홀 카메라와 이 프레임 워프의 보정·장착.
struct LaneCamera {
    float fx = 0.0f, fy = 0.0f, cx = 0.0f, cy = 0.0f;  // 프레임 화소(modeld 워프와 같은 AppConfig input_warp_*)
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;       // 이 프레임 워프에 쓴 보정(rad, CalibrationService::input_rpy)
    float height_m = 0.0f;                             // 도로면에서 카메라까지(ModelState.camera_height_m)
    float offset_m = 0.0f;                             // 가상 카메라가 실제 카메라에서 오른쪽으로(camera_offset_m)
};

// supercombo가 같은 프레임에 낸 노면 높이(m, 아래 +, 보정 좌표; ModelState.lanes·road_edges의 z). 거리는 X_IDXS다.
// 차선 4개(먼 왼쪽, 자차 왼쪽, 자차 오른쪽, 먼 오른쪽)와 도로 가장자리 2개(왼쪽, 오른쪽).
struct LaneRoadProfile {
    std::array<float, kTrajectorySize> x{};
    std::array<std::array<float, kTrajectorySize>, 4> lane_z{};
    std::array<std::array<float, kTrajectorySize>, 2> edge_z{};
};

LaneRoadProfile lane_road_profile(const ParsedModelOutput &parsed);

// 화소 (u, v)의 광선이 노면(거리 surface_x[i]에서 높이 surface_z[i], 아래 +, 점 사이는 직선)과 처음 만나는 점: 앞 x,
// 오른쪽 y(m, 가상 카메라 기준). 노면의 마지막 점까지 만나지 않으면(지평선 위 등) false.
bool lane_pixel_to_surface(const LaneCamera &camera, const float *surface_x, const float *surface_z, int count, float u,
                           float v, float *x, float *y);

// 결과를 out의 선·경계·소실점·도로면 블록으로 옮긴다. frame_id·시각·valid·plugin_name은 부르는 쪽이 채운다.
void fill_lane_state(const EpLaneResult &result, const LaneCamera &camera, const LaneRoadProfile &profile,
                     int image_width, int image_height, LaneState *out);

#endif
