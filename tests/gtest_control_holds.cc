/* 조향 경로 게이트(lateral_path: 모델 plan의 도달 거리와 점 수)와 문서화된 안전 홀드 두 개
 * (control_holds: Panda 헬스 공백은 100 ms까지, 잘못된 plan 프레임은 150 ms까지). 경계값은
 * docs/source-layout.md의 Control safety holds와 같다. */
#include "controls/control_holds.h"
#include "common/ipc_messages.h"
#include "controls/lateral_path.h"
#include "common/model_output.h"

#include <gtest/gtest.h>

namespace {

/* t=1.0 s에 나온, 60 m 이상 뻗은 조향 가능 plan. */
ModelState usable_model_state() {
  ModelState state;
  state.valid = 1;
  state.model_timestamp_ns = 1000000000ULL;
  state.plan_probability = 0.9f;
  state.lane_probabilities[1] = 0.8f;
  state.lane_probabilities[2] = 0.7f;
  for (int i = 0; i < kTrajectorySize; ++i) {
    const float x = 2.0f * static_cast<float>(i + 1);
    state.plan[i].x = x;
    state.plan[i].y = -0.0004f * x * x;
  }
  return state;
}

// 점 수는 충분하지만 몇 미터로 주저앉은 plan.
void collapse_plan(ModelState *state) {
  for (int i = 0; i < kTrajectorySize; ++i)
    state->plan[i].x = 1.0f + 0.1f * static_cast<float>(i);
}

TEST(ControlHolds, ModelPathAdapter) {
  const ModelState state = usable_model_state();
  const LateralPath path =
      path_from_model_state(state, 1100000000ULL, 250000000ULL);
  // 모델 경로 변환의 유효 판정
  ASSERT_TRUE(path.usable_for_steering);
  ASSERT_TRUE(path.left_valid);
  ASSERT_TRUE(path.right_valid);
  // 모델 경로 변환은 앞쪽 plan 점을 모두 센다
  ASSERT_EQ(path.point_count, kTrajectorySize);
  ASSERT_GE(path.reach_m, 60.0f);

  /* 정차에서 plan이 몇 미터로 주저앉으면 점 수는 충분해도 조향에 못 쓴다. */
  ModelState short_state = state;
  collapse_plan(&short_state);
  const LateralPath short_path =
      path_from_model_state(short_state, 1100000000ULL, 250000000ULL);
  // plan 도달 거리가 짧으면 조향 게이트에서 막힌다
  ASSERT_FALSE(short_path.usable_for_steering);
  ASSERT_EQ(short_path.invalid_reason, "path_invalid");
}

/* 문서화된 안전 홀드 1: Panda 헬스 스냅샷 공백은 100 ms까지만, 신선한
 * controls_allowed=0은 절대 유지하지 않는다. */
TEST(ControlHolds, PandaHealthHold) {
  const uint64_t t0 = 1000000000ULL;
  PandaState ready;
  ready.timestamp_ns = t0;
  ready.connected = ready.comms_healthy = ready.tx_enabled = 1;
  ready.controls_allowed = 1;
  ready.safety_mode = kExpectedPandaSafetyModel;
  ready.safety_param = kExpectedPandaSafetyParam;
  PandaState unhealthy = ready;
  unhealthy.comms_healthy = 0;

  PandaHealthGate gate;
  PandaGateOutput out = gate.update(ready, t0, false);
  // 준비된 panda는 홀드 없이 게이트를 통과한다
  ASSERT_TRUE(out.state_fresh);
  ASSERT_TRUE(out.ready_raw);
  ASSERT_TRUE(out.ready);
  ASSERT_TRUE(out.controls_allowed);
  ASSERT_FALSE(out.hold_applied);
  out = gate.update(unhealthy, t0 + 50000000ULL, false);
  // 50 ms 상태 공백은 마지막 판정을 유지한다
  ASSERT_FALSE(out.ready_raw);
  ASSERT_TRUE(out.hold_applied);
  ASSERT_TRUE(out.ready);
  ASSERT_TRUE(out.controls_allowed);
  out = gate.update(unhealthy, t0 + 100000000ULL, false);
  ASSERT_TRUE(out.hold_applied) << "홀드는 정확히 100 ms까지 덮는다";
  out = gate.update(unhealthy, t0 + 100000001ULL, false);
  // 홀드는 100 ms 뒤 끝난다
  ASSERT_FALSE(out.hold_applied);
  ASSERT_FALSE(out.ready);
  ASSERT_FALSE(out.controls_allowed);
  out = gate.update(ready, t0 + 1200000000ULL, false);
  // 1.2 s 지난 스냅샷은 필드가 준비돼 보여도 낡은 것이다
  ASSERT_FALSE(out.state_fresh);
  ASSERT_FALSE(out.ready_raw);
  ASSERT_FALSE(out.ready);

  PandaHealthGate explicit_off;
  explicit_off.update(ready, t0, false);
  PandaState off = ready;
  off.controls_allowed = 0;
  off.timestamp_ns = t0 + 10000000ULL;
  out = explicit_off.update(off, t0 + 10000000ULL, false);
  // 신선하고 전송이 준비된 controls_allowed=0은 홀드하지 않는다
  ASSERT_TRUE(out.ready_raw);
  ASSERT_TRUE(out.controls_off_explicit);
  ASSERT_FALSE(out.hold_applied);
  ASSERT_TRUE(out.ready);
  ASSERT_FALSE(out.controls_allowed);
  PandaState gap = off;
  gap.comms_healthy = 0;
  out = explicit_off.update(gap, t0 + 60000000ULL, false);
  // 명시적 off 뒤 상태 공백은 controls를 끈 채로 둔다
  ASSERT_TRUE(out.hold_applied);
  ASSERT_TRUE(out.ready);
  ASSERT_FALSE(out.controls_allowed);

  PandaHealthGate cold;
  out = cold.update(unhealthy, t0, false);
  // 첫 준비 스냅샷 전에는 홀드가 없다
  ASSERT_FALSE(out.ready);
  ASSERT_FALSE(out.hold_applied);
  out = cold.update(unhealthy, t0, true);
  // force_engaged는 panda 게이트를 건너뛴다
  ASSERT_TRUE(out.ready);
  ASSERT_TRUE(out.controls_allowed);
  ASSERT_FALSE(out.ready_raw);
}

/* 문서화된 안전 홀드 2: 잘못된 plan 프레임은 150 ms까지 마지막 유효 경로로
 * 덮고, 모델 freshness 타임아웃은 그대로 하드 게이트다. */
TEST(ControlHolds, PathInvalidHold) {
  const uint64_t timeout_ns = 250000000ULL;
  const ModelState good = usable_model_state();

  PathHoldGate gate;
  PathHoldOutput out = gate.update(good, 1100000000ULL, timeout_ns);
  // 쓸 수 있는 plan은 홀드를 그대로 지난다
  ASSERT_TRUE(out.path.usable_for_steering);
  ASSERT_FALSE(out.hold_applied);
  ModelState collapsed = good;
  collapsed.model_timestamp_ns = 1050000000ULL;
  collapse_plan(&collapsed);
  out = gate.update(collapsed, 1100000000ULL, timeout_ns);
  // 무너진 프레임 하나는 마지막 쓸 수 있는 경로로 덮는다
  ASSERT_EQ(out.raw.invalid_reason, "path_invalid");
  ASSERT_TRUE(out.hold_applied);
  ASSERT_TRUE(out.path.usable_for_steering);
  ASSERT_TRUE(out.path.invalid_reason.empty());
  collapsed.model_timestamp_ns = 1120000000ULL;
  out = gate.update(collapsed, 1150000000ULL, timeout_ns);
  ASSERT_TRUE(out.hold_applied) << "홀드는 정확히 150 ms까지 덮는다";
  out = gate.update(collapsed, 1150000001ULL, timeout_ns);
  // 홀드는 마지막 쓸 수 있는 프레임 150 ms 뒤 끝난다
  ASSERT_FALSE(out.hold_applied);
  ASSERT_FALSE(out.path.usable_for_steering);
  ASSERT_EQ(out.path.invalid_reason, "path_invalid");

  PathHoldGate stale_gate;
  stale_gate.update(good, 1100000000ULL, timeout_ns);
  ModelState stale = collapsed;
  stale.model_timestamp_ns = 1000000000ULL;
  out = stale_gate.update(stale, 1400000000ULL, timeout_ns);
  // 낡은 모델은 바로 막고 홀드하지 않는다
  ASSERT_FALSE(out.hold_applied);
  ASSERT_EQ(out.path.invalid_reason, "model_stale");

  PathHoldGate invalid_gate;
  invalid_gate.update(good, 1100000000ULL, timeout_ns);
  ModelState invalid = good;
  invalid.valid = 0;
  invalid.model_timestamp_ns = 1120000000ULL;
  out = invalid_gate.update(invalid, 1130000000ULL, timeout_ns);
  // 무효 모델은 홀드하지 않는다
  ASSERT_FALSE(out.hold_applied);
  ASSERT_EQ(out.path.invalid_reason, "model_invalid");
}

}  // namespace
