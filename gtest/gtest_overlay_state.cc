/* overlay_state: 공유 상태 스냅샷 → HUD 상태 매핑. 보드 없이, OpenCV 없이 돈다. */
#include "control_block.h"
#include "control_params.h"
#include "overlay_state.h"

#include <gtest/gtest.h>
#include <cmath>
#include <cstring>

namespace {

TEST(OverlayState, ControlStateMapping) {
  ControlState c;
  c.enabled = 1;
  c.engaged = 1;
  c.cluster_speed_kph = 63.5f;
  c.hud_flags = kHudFlagLaneless;
  c.departure_alert_type = static_cast<uint32_t>(DepartureAlertType::green_light);
  c.apply_torque = -120;
  std::snprintf(c.active_block, sizeof(c.active_block), "%s", "not_engaged");

  OverlayHudState hud;
  hud_apply_control_state(c, true, &hud);
  // 신선한 제어 스냅샷은 필드 그대로 옮긴다
  ASSERT_TRUE(hud.controller_enabled);
  ASSERT_TRUE(hud.controller_engaged);
  ASSERT_TRUE(hud.laneless_mode);
  ASSERT_EQ(hud.cluster_speed_kph, 63.5f);
  ASSERT_EQ(hud.apply_torque, -120);
  ASSERT_EQ(hud.departure_alert_type, DepartureAlertType::green_light);
  ASSERT_STREQ(hud.active_block, "not_engaged");
  hud_apply_control_state(c, false, &hud);
  // 낡은 스냅샷은 HUD를 비우고 control_stale로 표시한다
  ASSERT_FALSE(hud.controller_enabled);
  ASSERT_FALSE(hud.laneless_mode);
  ASSERT_EQ(hud.cluster_speed_kph, 0.0f);
  ASSERT_EQ(hud.apply_torque, 0);
  ASSERT_EQ(hud.departure_alert_type, DepartureAlertType::none);
  ASSERT_STREQ(hud.active_block, "control_stale");
  // engage 차단 라벨은 아는 사유에만 있다
  ASSERT_NE(engage_block_label("panda_not_ready"), nullptr);
  ASSERT_STREQ(engage_block_label("panda_not_ready"), "PANDA NOT READY");
  ASSERT_EQ(engage_block_label("no_such_reason"), nullptr);
  ASSERT_EQ(engage_block_label(""), nullptr);
  for (const BlockReasonRow &row : kBlockReasons) {
    if (row.reason == BlockReason::None) continue;
    const char *label = engage_block_label(row.name);
    // 모든 차단 사유가 제 라벨로 HUD에 뜬다
    ASSERT_NE(label, nullptr);
    ASSERT_NE(label[0], '\0');
    ASSERT_STREQ(label, row.label);
  }
}

TEST(OverlayState, ManeuverFlagsMapping) {
  ControlState c;
  c.hud_flags = kHudFlagLaneChangePending | kHudFlagLaneChangeRight | kHudFlagTurnLeft;
  c.driver_torque = -96;
  OverlayHudState hud;
  hud_apply_control_state(c, true, &hud);
  // 차선 변경 대기(오른쪽)와 좌회전 desire, 운전자 토크 눈금
  ASSERT_EQ(hud.lane_change, 1);
  ASSERT_EQ(hud.lane_change_direction, 1);
  ASSERT_EQ(hud.turn_direction, -1);
  ASSERT_FALSE(hud.steer_paused);
  ASSERT_NEAR(hud.driver_torque_fraction, -96.0f / SteeringParams{}.steer_max, 1e-6f);
  c.hud_flags = kHudFlagLaneChanging | kHudFlagSteerPaused | kHudFlagSteerPausedByDriver | kHudFlagTurnRight;
  hud_apply_control_state(c, true, &hud);
  ASSERT_EQ(hud.lane_change, 2);
  ASSERT_EQ(hud.lane_change_direction, -1);
  ASSERT_TRUE(hud.steer_paused);
  ASSERT_TRUE(hud.steer_paused_by_driver);
  ASSERT_EQ(hud.turn_direction, 1);
  hud_apply_control_state(c, false, &hud);
  // 낡은 스냅샷은 조작 표시를 끈다
  ASSERT_EQ(hud.lane_change, 0);
  ASSERT_FALSE(hud.steer_paused);
  ASSERT_EQ(hud.turn_direction, 0);
  ASSERT_EQ(hud.driver_torque_fraction, 0.0f);
}

TEST(OverlayState, LearnerAndLaneMapping) {
  LearnerState learner;
  learner.flags = kLearnerSteerRatioValid | kLearnerStiffnessValid | kLearnerOffsetAverageValid;
  learner.steer_ratio = 15.43f;
  learner.lat_accel_factor_raw = 3.45f;
  learner.cal_perc = 74;
  learner.plan_delay_s = 0.42f;
  OverlayHudState hud;
  hud_apply_learner_state(learner, true, &hud);
  // paramsd 세 값이 다 유효해야 유효, torqued는 따로
  ASSERT_TRUE(hud.learner_fresh);
  ASSERT_TRUE(hud.params_valid);
  ASSERT_FALSE(hud.torque_valid);
  ASSERT_EQ(hud.steer_ratio, 15.43f);
  ASSERT_EQ(hud.torque_factor, 3.45f);
  ASSERT_EQ(hud.torque_cal_percent, 74);
  learner.flags &= ~kLearnerStiffnessValid;
  hud_apply_learner_state(learner, true, &hud);
  ASSERT_FALSE(hud.params_valid);

  // 학습 카드: 제어가 쓰는 형태의 값(빠른 영점, 필터한 토크 계수)과 쓰는지
  ASSERT_FALSE(hud.vehicle_learned || hud.torque_learned || hud.delay_learned);
  learner.flags |= kLearnerUseVehicle | kLearnerUseTorque | kLearnerUseDelay;
  learner.angle_offset_deg = -1.58f;
  learner.lat_accel_factor = 2.31f;
  hud_apply_learner_state(learner, true, &hud);
  ASSERT_TRUE(hud.vehicle_learned && hud.torque_learned && hud.delay_learned);
  ASSERT_EQ(hud.angle_offset_fast_deg, -1.58f);
  ASSERT_EQ(hud.torque_factor_filtered, 2.31f);
  hud_apply_learner_state(learner, false, &hud);
  ASSERT_FALSE(hud.learner_fresh);
  ASSERT_FALSE(hud.vehicle_learned || hud.torque_learned || hud.delay_learned) << "멈춘 controlsd의 값은 쓰는 값이 아니다";

  LocalizationState localization;
  localization.lag_valid_blocks = 3;
  hud_apply_localization_state(localization, true, &hud);
  ASSERT_EQ(hud.lag_blocks, 3);
  hud_apply_localization_state(localization, false, &hud);
  ASSERT_EQ(hud.lag_blocks, -1) << "locationd가 멈추면 lagd 없음";

  ParsedModelOutput output;
  output.valid = true;
  output.lanes[1].valid = output.lanes[2].valid = true;
  output.lanes[1].probability = output.lanes[2].probability = 0.9f;
  output.lanes[1].points[0].y = -1.70f;  // 왼쪽 선
  output.lanes[2].points[0].y = 1.86f;   // 오른쪽 선
  // 차선 중앙이 0.08 m 오른쪽 = 차가 왼쪽에 있다(lane_bias.py offset과 같은 부호)
  ASSERT_NEAR(lane_center_offset_m(output), 0.08f, 1e-6f);
  output.lanes[2].probability = 0.3f;
  ASSERT_TRUE(std::isnan(lane_center_offset_m(output))) << "한쪽 선이 불확실하면 모른다";
}

TEST(OverlayState, RecordStateMapping) {
  RecordState r;
  r.active = 1;
  OverlayHudState hud;
  hud_apply_record_state(r, true, &hud);
  // route를 쓰는 중이면 REC
  ASSERT_TRUE(hud.recording);
  ASSERT_FALSE(hud.storage_full);
  r.active = 0;
  r.storage_blocked = 1;
  hud_apply_record_state(r, true, &hud);
  // 저장 공간 때문에 멈추면 REC 대신 저장 공간 경고
  ASSERT_FALSE(hud.recording);
  ASSERT_TRUE(hud.storage_full);
  r.active = 1;
  hud_apply_record_state(r, false, &hud);
  // recordd가 멈춰 스냅샷이 낡으면 둘 다 끈다
  ASSERT_FALSE(hud.recording);
  ASSERT_FALSE(hud.storage_full);
}

}  // namespace
