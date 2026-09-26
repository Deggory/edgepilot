/* project_point: 화면 크기가 달라도(K230 800x480·세로 480x800, MaixCAM2 640x480) 같은
 * 도로 점이 화면의 같은 비율 위치에 놓여야 한다. 폭을 하드코딩하면 오버레이가 영상과
 * 어긋난다. */
#include "app_config.h"
#include "projection.h"

#include <cstdlib>

#include <gtest/gtest.h>

namespace {

struct Point {
  int x;
  int y;
};

Point project(const ProjectionState &p, float fwd, float left, float up, int w, int h) {
  Point out{};
  EXPECT_TRUE(project_point(p, fwd, left, up, w, h, &out.x, &out.y));
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

TEST(Projection, PortraitBufferMatchesLandscape) {
  const ProjectionState p = make_projection_state(0.0f, 0.02f, -0.01f);
  const Point land = project(p, 30.0f, 1.0f, 0.5f, 800, 480);
  const Point port = project(p, 30.0f, 1.0f, 0.5f, 480, 800);
  EXPECT_EQ(port.x, land.y);
  EXPECT_EQ(port.y, 800 - 1 - land.x);
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

}  // namespace
