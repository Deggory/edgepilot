/* modeld가 채우고(fill_model_state, compute_lane_t) overlayd가 되돌리는(parsed_from_model_state)
 * ModelState. 왕복에서 남는 값과 빠지는 값(lead는 t=0 하나, road_transform 없음), 보정 상태 칸, 그리고
 * lane_t: 일정 속도 plan이면 거리/속도, 짧은 plan이면 끝 뒤가 NaN, 뒤로 뛰는 knot에도 단조. */
#include "model/model_state_fill.h"
#include "hud/overlay_state.h"

#include <gtest/gtest.h>
#include <cmath>

namespace {

ParsedModelOutput distinct_output() {
  ParsedModelOutput out;
  out.valid = true;
  out.plan.valid = true;
  out.plan.best_index = 0;
  out.plan.probability = 0.8f;
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float t = model_t_idx(i);
    out.plan.points[i] = {15.0f * t, 0.01f * i, -0.002f * i};
    out.plan.yaw[i] = 0.001f * i;
    out.plan.yaw_rate[i] = -0.0005f * i;
    for (int lane = 0; lane < 4; ++lane)
      out.lanes[lane].points[i] = {model_x_idx(i), -5.4f + 3.6f * lane + 0.001f * i, 1.2f};
    for (int edge = 0; edge < 2; ++edge)
      out.road_edges[edge].points[i] = {model_x_idx(i), edge == 0 ? -7.0f : 7.0f, 1.2f};
  }
  for (int lane = 0; lane < 4; ++lane) {
    out.lanes[lane].valid = true;
    out.lanes[lane].probability = 0.2f + 0.2f * lane;
    out.lanes[lane].std = 0.1f + 0.05f * lane;
  }
  for (int edge = 0; edge < 2; ++edge) {
    out.road_edges[edge].valid = true;
    out.road_edges[edge].std = 0.3f + edge;
  }
  for (int i = 0; i < kDesireLen; ++i) out.meta.desire_state[i] = 0.1f * i;
  for (int i = 0; i < kMetaPressHorizons; ++i) {
    out.meta.gas_press[i] = 0.05f * i;
    out.meta.brake_press[i] = 0.9f - 0.1f * i;
  }
  out.leads.valid = true;
  out.leads.global_probabilities = {0.7f, 0.6f, 0.5f};
  for (int t = 0; t < kLeadMhpSelection; ++t)
    out.leads.predictions[t].points[0] = {30.0f + t, 0.5f, 12.0f, -0.5f};
  out.has_pose = true;
  for (int i = 0; i < 3; ++i) {
    out.pose.trans[i] = 1.0f + i;
    out.pose.rot[i] = 0.01f * i;
    out.pose.trans_std[i] = 0.1f;
    out.pose.rot_std[i] = 0.02f;
    out.pose.road_trans[i] = 9.0f;
  }
  return out;
}

TEST(ModelState, RoundTripKeepsWhatTheHudDraws) {
  const ParsedModelOutput in = distinct_output();
  OnlineCalibrator::Snapshot calibration;
  calibration.status = CalibrationStatus::Calibrated;
  calibration.valid_blocks = 7;
  calibration.spread[1] = 0.002f;
  const ProjectionState projection = make_projection_state(0.01f, -0.02f, 0.03f);
  ModelState state;
  fill_model_state(state, in, projection, calibration, 42, 123456789ULL, 15.5f);

  ASSERT_EQ(state.frame_id, 42);
  ASSERT_EQ(state.capture_timestamp_ns, 123456789ULL);
  ASSERT_NE(state.model_timestamp_ns, 0) << "발행 시각은 채울 때의 monotonic_now_ns";
  ASSERT_EQ(state.calibration.status, 1);
  ASSERT_EQ(state.calibration.valid_blocks, 7);
  ASSERT_FLOAT_EQ(state.calibration.pitch, -0.02f) << "보정 rpy는 투영(워프에 쓴 값)에서 온다";
  ASSERT_FLOAT_EQ(state.calibration.spread[1], 0.002f);

  const ParsedModelOutput out = parsed_from_model_state(state);
  ASSERT_TRUE(out.valid);
  ASSERT_FLOAT_EQ(out.plan.probability, in.plan.probability);
  for (int i = 0; i < kTrajectorySize; ++i) {
    ASSERT_FLOAT_EQ(out.plan.points[i].x, in.plan.points[i].x);
    ASSERT_FLOAT_EQ(out.plan.points[i].y, in.plan.points[i].y);
    ASSERT_FLOAT_EQ(out.plan.yaw[i], in.plan.yaw[i]);
    ASSERT_FLOAT_EQ(out.plan.yaw_rate[i], in.plan.yaw_rate[i]);
    for (int lane = 0; lane < 4; ++lane) ASSERT_FLOAT_EQ(out.lanes[lane].points[i].y, in.lanes[lane].points[i].y);
    for (int edge = 0; edge < 2; ++edge)
      ASSERT_FLOAT_EQ(out.road_edges[edge].points[i].y, in.road_edges[edge].points[i].y);
  }
  for (int lane = 0; lane < 4; ++lane) {
    ASSERT_FLOAT_EQ(out.lanes[lane].probability, in.lanes[lane].probability);
    ASSERT_FLOAT_EQ(out.lanes[lane].std, in.lanes[lane].std);
  }
  ASSERT_FLOAT_EQ(out.road_edges[1].std, in.road_edges[1].std);
  ASSERT_FLOAT_EQ(out.meta.desire_state[5], in.meta.desire_state[5]);
  ASSERT_FLOAT_EQ(out.meta.gas_press[1], in.meta.gas_press[1]);
  ASSERT_FLOAT_EQ(out.meta.brake_press[4], in.meta.brake_press[4]);
  ASSERT_TRUE(out.has_pose);
  ASSERT_FLOAT_EQ(out.pose.trans[2], in.pose.trans[2]);

  // ModelState가 싣지 않는 값
  ASSERT_FLOAT_EQ(out.leads.global_probabilities[0], 0.7f);
  ASSERT_FLOAT_EQ(out.leads.predictions[0].points[0].x, 30.0f) << "lead는 t=0 하나만 싣는다";
  ASSERT_FLOAT_EQ(out.leads.global_probabilities[1], 0.0f);
  ASSERT_FLOAT_EQ(out.pose.road_trans[0], 0.0f) << "road_transform은 ModelState에 없다";
}

TEST(ModelState, LaneTimeFollowsThePlan) {
  ParsedPlan plan;
  for (int i = 0; i < kTrajectorySize; ++i) plan.points[i].x = 25.0f * model_t_idx(i);  // 25 m/s
  float lane_t[kTrajectorySize];
  compute_lane_t(plan, lane_t);
  ASSERT_FLOAT_EQ(lane_t[0], 0.0f);
  for (int i = 1; i < kTrajectorySize; ++i)
    ASSERT_NEAR(lane_t[i], model_x_idx(i) / 25.0f, 1e-4f) << "일정 속도면 거리/속도, 점 " << i;

  ParsedPlan stopping;  // 10초에 30 m에서 멈춘다
  for (int i = 0; i < kTrajectorySize; ++i)
    stopping.points[i].x = 30.0f * std::sqrt(model_t_idx(i) / 10.0f);
  compute_lane_t(stopping, lane_t);
  int finite = 0;
  for (int i = 0; i < kTrajectorySize; ++i) finite += std::isfinite(lane_t[i]) ? 1 : 0;
  ASSERT_LT(finite, kTrajectorySize);
  ASSERT_FLOAT_EQ(lane_t[finite - 1], model_t_idx(kTrajectorySize - 1)) << "plan 끝을 넘는 첫 점은 마지막 시각";
  for (int i = finite; i < kTrajectorySize; ++i) ASSERT_TRUE(std::isnan(lane_t[i])) << "그 뒤는 NaN";

  ParsedPlan jumpy = plan;  // 정차 부근 양자화: 먼 knot이 뒤로 뛴다
  jumpy.points[12].x = jumpy.points[10].x - 0.5f;
  jumpy.points[20].x = jumpy.points[18].x - 2.0f;
  compute_lane_t(jumpy, lane_t);
  for (int i = 1; i < kTrajectorySize && std::isfinite(lane_t[i]); ++i)
    ASSERT_GE(lane_t[i], lane_t[i - 1]) << "lane_t는 단조다, 점 " << i;
}

}  // namespace
