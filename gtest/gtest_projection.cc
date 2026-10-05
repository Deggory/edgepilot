/* project_point_subpixel: 화면 크기가 달라도(800x480, MaixCAM2 640x480) 같은 도로 점이 화면의
 * 같은 비율 위치에 놓여야 한다. 폭을 하드코딩하면 오버레이가 영상과 어긋난다. */
#include "app_config.h"
#include "ipc_messages.h"
#include "projection.h"

#include <cmath>
#include <cstdlib>

#include <gtest/gtest.h>

namespace {

struct Point {
  int x;
  int y;
};

Point project(const ProjectionState &p, float fwd, float left, float up, int w, int h) {
  Point out{};
  float x = 0.0f, y = 0.0f;
  EXPECT_TRUE(project_point_subpixel(p, fwd, left, up, w, h, &x, &y));
  out.x = static_cast<int>(std::round(x));
  out.y = static_cast<int>(std::round(y));
  return out;
}

TEST(Projection, SameRelativePositionAcrossLandscapeWidths) {
  const ProjectionState p = make_projection_state(0.0f, 0.02f, -0.01f);
  const float points[][3] = {{50.0f, 0.0f, 1.2f}, {20.0f, 1.8f, 0.0f}, {10.0f, -1.8f, 0.0f}};
  for (const auto &pt : points) {
    const Point wide = project(p, pt[0], pt[1], pt[2], 800, 480);
    const Point compact = project(p, pt[0], pt[1], pt[2], 640, 480);
    EXPECT_NEAR(compact.x, (wide.x + 0.5f) * 640.0f / 800.0f - 0.5f, 1.0f);
    EXPECT_NEAR(compact.y, wide.y, 1.0f);
  }
}

TEST(Projection, FollowsConfiguredIntrinsics) {
  // 초점거리가 길면(K230 카메라) 같은 옆 지점이 화면 가운데에 더 멀리 찍힌다.
  const ProjectionState p = make_projection_state(0.0f, 0.0f, 0.0f);
  const Point maix = project(p, 20.0f, 2.0f, 0.0f, 640, 480);
  projection_set_camera_intrinsics(1583.3981f, 1583.7622f, 954.9441f, 545.1774f);
  const Point k230 = project(p, 20.0f, 2.0f, 0.0f, 640, 480);
  projection_set_camera_intrinsics(kCameraFx, kCameraFy, kCameraCx, kCameraCy);
  const Point back = project(p, 20.0f, 2.0f, 0.0f, 640, 480);
  EXPECT_GT(std::abs(k230.x - 320), std::abs(maix.x - 320));
  EXPECT_EQ(back.x, maix.x);
  EXPECT_EQ(back.y, maix.y);
}

/* 카메라 장착 오프셋(set_camera_mount): 가상 카메라가 실제보다 오프셋만큼 오른쪽이라 모델은 실제 점
 * y를 y − 오프셋으로 본다(gtest_calibration_equivalence CameraMountShiftsGroundPlane). HUD는 모델 점을
 * 실제 위치에 그려야 한다. */
TEST(Projection, CameraOffsetDrawsModelPointsAtRealPosition) {
  const ProjectionState plain = make_projection_state(0.0f, 0.02f, -0.01f);
  ProjectionState shifted = plain;
  shifted.lateral_offset_m = 0.3f;
  for (const float fwd : {8.0f, 15.0f, 30.0f}) {
    const float y_model = 1.0f, z = 1.2f;  // 모델 좌표: 오른쪽·아래 양수
    const Point drawn = project(shifted, fwd, y_model, z, 640, 480);
    const Point real = project(plain, fwd, y_model + 0.3f, z, 640, 480);
    EXPECT_EQ(drawn.x, real.x) << fwd;
    EXPECT_EQ(drawn.y, real.y) << fwd;
  }
  // 오른쪽 점은 화면 오른쪽에 찍힌다(모델 y는 오른쪽 양수). 투영 좌표는 180° 돌아간
  // 버퍼라 표시할 때 뒤집는다(overlay_renderer project_display_point).
  auto shown_x = [&](float y) { return 640 - 1 - project(plain, 15.0f, y, 1.2f, 640, 480).x; };
  EXPECT_GT(shown_x(1.8f), 320);
  EXPECT_LT(shown_x(-1.8f), 320);
}

// HUD는 modeld가 그 프레임 워프에 쓴 오프셋(ModelState)으로 그린다
TEST(Projection, ModelStateCarriesCameraOffset) {
  ModelState ms;
  ms.camera_offset_m = 0.25f;
  EXPECT_FLOAT_EQ(projection_from_model_state(ms).lateral_offset_m, 0.25f);
}

}  // namespace
