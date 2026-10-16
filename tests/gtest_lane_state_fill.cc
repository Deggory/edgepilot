// 차선선 결과를 LaneState로(model/lane_state_fill): 화소 → 노면 투영이 정방향 투영의 역인지(평평한 면과 supercombo
// 노면 높이, 오프셋은 높이에 비례), 선의 안쪽 경계를 거리 격자에서 맞게 읽는지(덮지 않거나 멀리 끊긴 곳은 NaN, 끊긴
// 조각을 건너뛰지 않는 가까운 쪽 늘리기), 선마다 맞는 노면 높이를 쓰고 먼 오르막에서도 그것을 쓰는지, 점·종류를
// 1/4 화소 정수로 그대로 옮기는지 본다.
#include "common/projection.h"
#include "model/lane_state_fill.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

LaneCamera camera() {
    LaneCamera c;
    c.fx = 754.16f;
    c.fy = 753.90f;
    c.cx = 626.75f;
    c.cy = 368.40f;
    c.roll = 0.01f;
    c.pitch = -0.02f;
    c.yaw = 0.015f;
    c.height_m = 1.22f;
    c.offset_m = 0.05f;
    return c;
}

// 노면 점(가상 카메라 기준, 카메라 아래 z m)을 화소로: 시험용 정방향 투영(device = R calib, R = Rz Ry Rx). 워프는
// 도로면에 맞춘 변환이라 가상 카메라는 높이 z인 점을 offset·z/h만큼 옮겨 본다.
bool road_to_pixel(const LaneCamera &c, float x, float y, float z, float *u, float *v) {
    float rot[9];
    rotation_from_rpy(c.roll, c.pitch, c.yaw, rot);
    const float calib[3] = {x, y + c.offset_m * z / c.height_m, z};
    float device[3];
    for (int i = 0; i < 3; ++i)
        device[i] = rot[i * 3 + 0] * calib[0] + rot[i * 3 + 1] * calib[1] + rot[i * 3 + 2] * calib[2];
    if (device[0] <= 0.1f) return false;
    *u = c.fx * device[1] / device[0] + c.cx;
    *v = c.fy * device[2] / device[0] + c.cy;
    return true;
}

bool road_to_pixel(const LaneCamera &c, float x, float y, float *u, float *v) {
    return road_to_pixel(c, x, y, c.height_m, u, v);
}

// 노면 높이가 z = z0 + slope·x인 길(앞이 내리막이면 slope > 0)의 supercombo 높이 묶음
LaneRoadProfile sloped_profile(float z0, float slope) {
    LaneRoadProfile profile;
    for (int i = 0; i < kTrajectorySize; ++i) {
        profile.x[i] = model_x_idx(i);
        const float z = z0 + slope * profile.x[i];
        for (auto &lane : profile.lane_z) lane[i] = z;
        for (auto &edge : profile.edge_z) edge[i] = z;
    }
    return profile;
}

// road_y + curvature·x²/2에 놓인 차선선: 앞 x_from..x_to m를 2 m 간격으로, 안쪽 경계(edge_x)가 그 위치다. 노면은
// 카메라 아래 z0 + slope·x.
EpLaneLine line_at(const LaneCamera &c, int slot, float road_y, float x_from, float x_to, float curvature = 0.0f,
                   float z0 = -1.0f, float slope = 0.0f) {
    if (z0 < 0.0f) z0 = c.height_m;
    EpLaneLine line{};
    line.slot = slot;
    line.shape = EP_LANE_SOLID;
    line.color = EP_LANE_WHITE;
    line.confidence = 0.9f;
    for (float x = x_from; x <= x_to + 1e-3f && line.point_count < EP_LANE_MAX_POINTS; x += 2.0f) {
        float u = 0.0f, v = 0.0f;
        if (!road_to_pixel(c, x, road_y + 0.5f * curvature * x * x, z0 + slope * x, &u, &v)) {
            ADD_FAILURE() << "x=" << x << " 투영 실패";
            return {};
        }
        line.points[line.point_count++] = {u - 4.0f, v, 8.0f, u};
    }
    return line;
}

TEST(LaneStateFill, PixelToSurfaceInvertsTheProjection) {
    const LaneCamera c = camera();
    const LaneRoadProfile flat = sloped_profile(c.height_m, 0.0f);
    for (float x : {5.0f, 10.0f, 20.0f, 40.0f}) {
        for (float y : {-3.5f, -1.7f, 0.0f, 1.8f, 3.6f}) {
            float u = 0.0f, v = 0.0f, rx = 0.0f, ry = 0.0f;
            ASSERT_TRUE(road_to_pixel(c, x, y, &u, &v));
            ASSERT_TRUE(lane_pixel_to_surface(c, flat.x.data(), flat.lane_z[1].data(), kTrajectorySize, u, v, &rx, &ry))
                << x << ", " << y;
            EXPECT_NEAR(rx, x, 1e-3f * x) << "앞 거리";
            EXPECT_NEAR(ry, y, 1e-3f * x) << "가로 위치(가상 카메라 기준)";
        }
    }
    float rx = 0.0f, ry = 0.0f;
    EXPECT_FALSE(
        lane_pixel_to_surface(c, flat.x.data(), flat.lane_z[1].data(), kTrajectorySize, 640.0f, 100.0f, &rx, &ry))
        << "지평선 위는 노면에 닿지 않는다";
}

// 내리막·오르막에서 평평한 면 투영은 멀수록 어긋나고, 노면 높이 투영은 맞는다(오프셋도 높이에 비례해 맞는다).
TEST(LaneStateFill, SurfaceProjectionFollowsTheRoadHeight) {
    const LaneCamera c = camera();
    const LaneRoadProfile flat = sloped_profile(c.height_m, 0.0f);
    for (const float slope : {0.012f, -0.01f}) {
        const LaneRoadProfile profile = sloped_profile(c.height_m, slope);
        for (float x : {5.0f, 15.0f, 30.0f, 45.0f}) {
            for (float y : {-3.5f, -1.7f, 1.8f}) {
                float u = 0.0f, v = 0.0f, rx = 0.0f, ry = 0.0f;
                ASSERT_TRUE(road_to_pixel(c, x, y, c.height_m + slope * x, &u, &v));
                ASSERT_TRUE(lane_pixel_to_surface(c, profile.x.data(), profile.lane_z[1].data(), kTrajectorySize, u, v,
                                                  &rx, &ry));
                EXPECT_NEAR(rx, x, 0.01f) << "slope " << slope;
                EXPECT_NEAR(ry, y, 0.01f) << "slope " << slope;
                if (x >= 30.0f && y < -3.0f) {
                    float fx = 0.0f, fy = 0.0f;
                    ASSERT_TRUE(lane_pixel_to_surface(c, flat.x.data(), flat.lane_z[1].data(), kTrajectorySize, u, v,
                                                      &fx, &fy));
                    EXPECT_GT(std::fabs(fy - y), 0.1f) << "평평한 면 투영은 30 m 넘어 10 cm 넘게 어긋난다";
                }
            }
        }
    }
}

TEST(LaneStateFill, RoadBlockReadsTheLinesOnTheGrid) {
    const LaneCamera c = camera();
    EpLaneResult result{};
    result.line_count = 3;
    result.lines[0] = line_at(c, EP_LANE_EGO_LEFT, -1.7f, 4.0f, 40.0f);
    result.lines[1] = line_at(c, EP_LANE_EGO_RIGHT, 1.8f, 4.0f, 12.0f);    // 12 m까지만 보인다
    result.lines[2] = line_at(c, EP_LANE_LEFT_NEXT, -5.2f, 10.0f, 40.0f);  // 10 m부터 보인다
    result.boundary_count = 1;
    result.boundaries[0].slot = EP_BOUNDARY_NEAR_RIGHT;
    result.boundaries[0].type = EP_BOUNDARY_GUARDRAIL;
    result.boundaries[0].confidence = 0.6f;
    const EpLaneLine rail = line_at(c, 0, 5.4f, 6.0f, 30.0f);
    for (uint32_t i = 0; i < rail.point_count; ++i) {
        result.boundaries[0].points[i] = rail.points[i];
        result.boundaries[0].points[i].x = rail.points[i].edge_x;  // 경계는 점 그대로를 쓴다
    }
    result.boundaries[0].point_count = rail.point_count;
    LaneState state;
    fill_lane_state(result, c, sloped_profile(c.height_m, 0.0f), 1280, 720, &state);

    ASSERT_EQ(state.road_count, 4u) << "선 셋, 경계 하나";
    const LaneRoadLine &left = state.road[0];
    EXPECT_EQ(left.kind, kLaneRoadKindLine);
    EXPECT_EQ(left.slot, EP_LANE_EGO_LEFT);
    EXPECT_EQ(left.type, EP_LANE_SOLID);
    EXPECT_FLOAT_EQ(left.confidence, 0.9f);
    // X_IDXS: 3.0, 4.69, …, 36.75, 42.19 m. 4 m부터 덮으므로 처음 덮은 점은 5번(4.69 m), 마지막은 14번(36.75 m)
    EXPECT_EQ(left.first, 5);
    EXPECT_EQ(left.last, 14);
    for (unsigned k = 0; k < kLaneRoadPoints; ++k) {
        if (k <= 14)
            EXPECT_NEAR(left.y[k], -1.7f, 0.01f) << "grid " << k << " (0~4번은 가까운 쪽 늘리기)";
        else
            EXPECT_TRUE(std::isnan(left.y[k])) << "grid " << k;
    }
    const LaneRoadLine &right = state.road[1];
    EXPECT_EQ(right.last, 8) << "12 m";
    EXPECT_NEAR(right.y[0], 1.8f, 0.01f);
    EXPECT_TRUE(std::isnan(right.y[9])) << "선이 덮지 않는 15 m는 NaN";
    const LaneRoadLine &next = state.road[2];
    EXPECT_EQ(next.first, 8) << "10 m부터 보이면 첫 점은 12 m";
    EXPECT_TRUE(std::isnan(next.y[0])) << "가장 가까운 측정이 8 m 밖이면 늘리지 않는다";
    const LaneRoadLine &guardrail = state.road[3];
    EXPECT_EQ(guardrail.kind, kLaneRoadKindBoundary);
    EXPECT_EQ(guardrail.index, 0);
    EXPECT_EQ(guardrail.slot, EP_BOUNDARY_NEAR_RIGHT);
    EXPECT_EQ(guardrail.type, EP_BOUNDARY_GUARDRAIL);
    EXPECT_NEAR(guardrail.y[10], 5.4f, 0.01f) << "18.75 m";

    // 점 사이가 8 m 넘게 비면 잇지 않는다
    EpLaneResult gap{};
    gap.line_count = 1;
    gap.lines[0] = line_at(c, EP_LANE_EGO_LEFT, -1.7f, 6.0f, 8.0f);
    const EpLaneLine far = line_at(c, EP_LANE_EGO_LEFT, -1.7f, 22.0f, 28.0f);
    for (uint32_t i = 0; i < far.point_count; ++i) gap.lines[0].points[gap.lines[0].point_count++] = far.points[i];
    LaneState gapped;
    fill_lane_state(gap, c, sloped_profile(c.height_m, 0.0f), 1280, 720, &gapped);
    EXPECT_TRUE(std::isnan(gapped.road[0].y[9])) << "8 m와 22 m 사이의 15.2 m";
    EXPECT_TRUE(std::isnan(gapped.road[0].y[10])) << "18.75 m";
    EXPECT_NEAR(gapped.road[0].y[11], -1.7f, 0.01f) << "22~28 m 조각의 22.7 m는 남는다";
    EXPECT_NEAR(gapped.road[0].y[12], -1.7f, 0.01f) << "27 m";
}

// 굽은 선의 가까운 쪽은 4~20 m 점에 맞춘 2차식으로 x = 0까지 늘린다.
TEST(LaneStateFill, NearExtensionFollowsACurve) {
    const LaneCamera c = camera();
    for (const float curvature : {0.004f, -0.006f}) {
        EpLaneResult result{};
        result.line_count = 1;
        result.lines[0] = line_at(c, EP_LANE_EGO_RIGHT, 1.75f, 4.5f, 30.0f, curvature);
        LaneState state;
        fill_lane_state(result, c, sloped_profile(c.height_m, 0.0f), 1280, 720, &state);
        for (int k = 0; k <= 4; ++k) {
            const float x = model_x_idx(k);
            EXPECT_NEAR(state.road[0].y[k], 1.75f + 0.5f * curvature * x * x, 0.01f) << "x " << x;
        }
    }
}

// 선마다 같은 선의 supercombo 높이를 쓰고, 못 쓰면 자차 두 선의 평균, 그것도 없으면 평평한 면이다.
TEST(LaneStateFill, EachLineUsesItsOwnHeight) {
    const LaneCamera c = camera();
    LaneRoadProfile profile = sloped_profile(c.height_m, 0.0f);
    for (int i = 0; i < kTrajectorySize; ++i)
        profile.lane_z[2][i] = c.height_m + 0.012f * profile.x[i];  // 오른쪽만 내리막
    EpLaneResult result{};
    result.line_count = 2;
    result.lines[0] = line_at(c, EP_LANE_EGO_LEFT, -1.7f, 4.0f, 40.0f);
    result.lines[1] = line_at(c, EP_LANE_EGO_RIGHT, 1.8f, 4.0f, 40.0f, 0.0f, c.height_m, 0.012f);
    LaneState state;
    fill_lane_state(result, c, profile, 1280, 720, &state);
    EXPECT_NEAR(state.road[0].y[13], -1.7f, 0.01f) << "31.7 m, 왼쪽은 평평한 높이";
    EXPECT_NEAR(state.road[1].y[13], 1.8f, 0.01f) << "31.7 m, 오른쪽은 제 높이";

    // 먼 오른쪽 선의 높이가 없으면 자차 두 선 평균을 쓴다
    LaneRoadProfile broken = sloped_profile(c.height_m, 0.012f);
    for (float &z : broken.lane_z[3]) z = std::numeric_limits<float>::quiet_NaN();
    EpLaneResult next{};
    next.line_count = 1;
    next.lines[0] = line_at(c, EP_LANE_RIGHT_NEXT, 5.3f, 4.0f, 40.0f, 0.0f, c.height_m, 0.012f);
    LaneState with_mean;
    fill_lane_state(next, c, broken, 1280, 720, &with_mean);
    EXPECT_NEAR(with_mean.road[0].y[13], 5.3f, 0.01f);

    // 먼 오르막(54 m 너머에서 높이가 0.3 m 아래)도 제 높이를 쓴다: 도로면 블록이 쓰지 않는 먼 높이로 선 전체를 평평한
    // 면으로 돌리지 않는다
    const LaneRoadProfile uphill = sloped_profile(c.height_m, -0.01f);
    EpLaneResult rising{};
    rising.line_count = 1;
    rising.lines[0] = line_at(c, EP_LANE_EGO_RIGHT, 1.8f, 4.0f, 40.0f, 0.0f, c.height_m, -0.01f);
    LaneState on_hill;
    fill_lane_state(rising, c, uphill, 1280, 720, &on_hill);
    EXPECT_NEAR(on_hill.road[0].y[13], 1.8f, 0.01f) << "31.7 m";
}

// 가까운 점 둘(7, 8.4 m)이 처음 덮은 격자(18.75 m)와 8 m 넘게 끊겨 있으면 그 사이를 지어내지 않는다.
TEST(LaneStateFill, NearExtensionDoesNotBridgeAGap) {
    const LaneCamera c = camera();
    EpLaneResult result{};
    result.line_count = 1;
    EpLaneLine &line = result.lines[0];
    line = line_at(c, EP_LANE_EGO_LEFT, -1.7f, 7.0f, 7.0f);
    for (float x : {8.4f, 16.8f, 18.2f, 19.6f, 21.0f, 25.0f}) {
        const EpLaneLine one = line_at(c, EP_LANE_EGO_LEFT, -1.7f, x, x);
        line.points[line.point_count++] = one.points[0];
    }
    LaneState state;
    fill_lane_state(result, c, sloped_profile(c.height_m, 0.0f), 1280, 720, &state);
    EXPECT_EQ(state.road[0].first, 10) << "18.75 m";
    for (int k = 0; k < 10; ++k) EXPECT_TRUE(std::isnan(state.road[0].y[k])) << "grid " << k;
}

TEST(LaneStateFill, UnfilledStateHasNoRoadLines) {
    const LaneState state;
    EXPECT_EQ(state.road_count, 0u);
    for (const LaneRoadLine &road : state.road) {
        EXPECT_EQ(road.first, -1);
        for (float y : road.y) EXPECT_TRUE(std::isnan(y)) << "valid = 0으로 나가는 상태도 NaN이다";
    }
}

TEST(LaneStateFill, CopiesPointsAsQuarterPixels) {
    EpLaneResult result{};
    result.line_count = 1;
    EpLaneLine &line = result.lines[0];
    line.slot = EP_LANE_RIGHT_LANE_LEFT;
    line.shape = EP_LANE_DASHED;
    line.double_line = EP_LANE_DOUBLE_SOLID;
    line.color = EP_LANE_YELLOW;
    line.confidence = 0.75f;
    line.point_count = 2;
    line.points[0] = {100.3f, 575.0f, 12.6f, 106.6f};
    line.points[1] = {-20.0f, 290.0f, 3.0f, -18.5f};
    result.boundary_count = 1;
    result.boundaries[0].slot = EP_BOUNDARY_FAR_RIGHT;
    result.boundaries[0].type = EP_BOUNDARY_CONES;
    result.boundaries[0].confidence = 0.4f;
    result.boundaries[0].point_count = 1;
    result.boundaries[0].points[0] = {1200.0f, 500.0f, 0.0f, 1200.0f};
    result.vanishing_x = 640.0f;
    result.vanishing_y = 300.0f;
    result.vanishing_confidence = 0.95f;

    LaneState state;
    fill_lane_state(result, camera(), sloped_profile(1.22f, 0.0f), 1280, 720, &state);
    EXPECT_EQ(state.image_width, 1280);
    EXPECT_EQ(state.image_height, 720);
    ASSERT_EQ(state.line_count, 1u);
    EXPECT_EQ(state.lines[0].slot, EP_LANE_RIGHT_LANE_LEFT);
    EXPECT_EQ(state.lines[0].shape, EP_LANE_DASHED);
    EXPECT_EQ(state.lines[0].double_line, EP_LANE_DOUBLE_SOLID);
    EXPECT_EQ(state.lines[0].color, EP_LANE_YELLOW);
    EXPECT_FLOAT_EQ(state.lines[0].confidence, 0.75f);
    ASSERT_EQ(state.lines[0].point_count, 2u);
    // 1/4 화소로 반올림
    EXPECT_EQ(state.lines[0].points[0].x4, 401);
    EXPECT_EQ(state.lines[0].points[0].y4, 2300);
    EXPECT_EQ(state.lines[0].points[0].width4, 50);
    EXPECT_EQ(state.lines[0].points[0].edge_x4, 426);
    EXPECT_EQ(state.lines[0].points[1].x4, -80) << "프레임 밖으로 외삽한 점도 남긴다";
    ASSERT_EQ(state.boundary_count, 1u);
    EXPECT_EQ(state.boundaries[0].type, EP_BOUNDARY_CONES);
    EXPECT_EQ(state.boundaries[0].points[0].x4, 4800);
    EXPECT_FLOAT_EQ(state.vanishing_confidence, 0.95f);
}

}  // namespace
