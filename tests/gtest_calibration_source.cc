// 워프 보정 출처 고르기(model/calibration_source): 설정이 포즈면 포즈 값 그대로, 광류면 쓸 만할 때 광류의
// pitch·yaw, 결합 중 설정 변경은 해제까지 대기, 바뀔 때는 천천히 옮긴다.
#include "model/calibration_source.h"

#include "model/flow_calibration.h"

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr float kDeg = 3.14159265358979323846f / 180.0f;
constexpr float kFrameS = 0.05f;

FlowCalibrationState usable_flow(float pitch_deg, float yaw_deg) {
    FlowCalibrationState f;
    f.status = 2;
    f.blocks = 10;
    f.estimate_blocks = 10;
    f.pitch = pitch_deg * kDeg;
    f.yaw = yaw_deg * kDeg;
    f.sd_pitch = 0.3f * kDeg;
    f.sd_yaw = 0.4f * kDeg;
    return f;
}

struct Rpy {
    float v[3];
};

Rpy out(const CalibrationSourceSelector &sel) {
    Rpy r{};
    sel.rpy(r.v);
    return r;
}

// 같은 입력으로 seconds초 동안 프레임마다 부른다
void run(CalibrationSourceSelector *sel, const float pose[3], const FlowCalibrationState &flow, bool requested,
         bool engaged, float seconds) {
    for (int i = 0; i < static_cast<int>(std::lround(seconds / kFrameS)); ++i)
        sel->update(pose, flow, requested, engaged, kFrameS);
}

TEST(CalibrationSource, PoseSettingPassesThePoseValueThrough) {
    CalibrationSourceSelector sel;
    const FlowCalibrationState flow = usable_flow(-0.2f, -0.03f);
    float pose[3] = {0.001f, -0.5f * kDeg, -0.3f * kDeg};
    sel.update(pose, flow, false, false, 0.0f);
    EXPECT_FALSE(sel.flow_applied());
    EXPECT_FLOAT_EQ(out(sel).v[2], pose[2]);
    EXPECT_EQ(sel.flags(), kCalibrationSourceFlowUsable) << "쓸 수 있다는 표시만";
    pose[2] = 1.2f * kDeg;  // 포즈 보정기가 장착 변경으로 크게 옮겨도 그대로 따라간다(지금 동작)
    sel.update(pose, flow, false, false, kFrameS);
    for (int i = 0; i < 3; ++i) EXPECT_FLOAT_EQ(out(sel).v[i], pose[i]);
}

TEST(CalibrationSource, StartsOnTheFlowValueWhenSet) {
    CalibrationSourceSelector sel;
    const float pose[3] = {0.001f, -0.5f * kDeg, -0.3f * kDeg};
    sel.update(pose, usable_flow(-0.2f, -0.03f), true, false, 0.0f);
    EXPECT_TRUE(sel.flow_applied());
    EXPECT_FLOAT_EQ(out(sel).v[0], 0.001f) << "roll은 포즈에서";
    EXPECT_FLOAT_EQ(out(sel).v[1], -0.2f * kDeg);
    EXPECT_FLOAT_EQ(out(sel).v[2], -0.03f * kDeg);
    EXPECT_EQ(sel.flags() & kCalibrationSourceRamping, 0u) << "시작할 때는 바로";
}

TEST(CalibrationSource, RampsWhenTheFlowEstimateArrives) {
    CalibrationSourceSelector sel;
    const float pose[3] = {0.0f, -0.5f * kDeg, -0.3f * kDeg};
    FlowCalibrationState collecting = usable_flow(-0.2f, -0.03f);
    collecting.status = 1;
    collecting.estimate_blocks = 2;
    sel.update(pose, collecting, true, false, 0.0f);
    EXPECT_FALSE(sel.flow_applied()) << "블록이 모자라면 포즈";
    EXPECT_FLOAT_EQ(out(sel).v[2], pose[2]);

    const FlowCalibrationState flow = usable_flow(-0.2f, -0.03f);
    run(&sel, pose, flow, true, false, 1.0f);
    EXPECT_TRUE(sel.flow_applied());
    EXPECT_NE(sel.flags() & kCalibrationSourceRamping, 0u);
    const float moved = out(sel).v[2] - pose[2];
    EXPECT_GT(moved, 0.0f);
    // 0.27° 차이: 초당 max(0.05°, 차이/5) = 0.054°보다 많이 가지 않는다
    EXPECT_LE(moved, 0.055f * kDeg);
    run(&sel, pose, flow, true, false, 30.0f);
    EXPECT_NEAR(out(sel).v[2], -0.03f * kDeg, 1e-6f);
    EXPECT_NEAR(out(sel).v[1], -0.2f * kDeg, 1e-6f);
    EXPECT_EQ(sel.flags() & kCalibrationSourceRamping, 0u);
}

TEST(CalibrationSource, LargeChangesFollowWithinSeconds) {
    CalibrationSourceSelector sel;
    const float pose[3] = {0.0f, 0.0f, 0.0f};
    sel.update(pose, usable_flow(0.0f, 0.0f), true, false, 0.0f);
    const FlowCalibrationState moved = usable_flow(0.0f, 2.0f);  // 장착 변경 뒤 새 추정
    run(&sel, pose, moved, true, false, 10.0f);
    const float left = 2.0f * kDeg - out(sel).v[2];
    EXPECT_GT(left, 0.2f * kDeg);
    EXPECT_LT(left, 0.35f * kDeg) << "차이의 1/5를 초당 옮긴다(10초 뒤 e^-2)";
    run(&sel, pose, moved, true, false, 10.0f);
    EXPECT_NEAR(out(sel).v[2], 2.0f * kDeg, 1e-6f);
}

TEST(CalibrationSource, SettingWaitsWhileEngaged) {
    CalibrationSourceSelector sel;
    const float pose[3] = {0.0f, -0.5f * kDeg, -0.3f * kDeg};
    const FlowCalibrationState flow = usable_flow(-0.2f, -0.03f);
    sel.update(pose, flow, false, false, 0.0f);
    run(&sel, pose, flow, true, true, 2.0f);
    EXPECT_FALSE(sel.flow_applied()) << "결합 중에는 바꾸지 않는다";
    EXPECT_NE(sel.flags() & kCalibrationSourcePending, 0u);
    EXPECT_FLOAT_EQ(out(sel).v[2], pose[2]);
    sel.update(pose, flow, true, false, kFrameS);
    EXPECT_TRUE(sel.flow_applied()) << "해제되면 바꾼다";
    EXPECT_EQ(sel.flags() & kCalibrationSourcePending, 0u);
    EXPECT_NE(sel.flags() & kCalibrationSourceRamping, 0u);

    run(&sel, pose, flow, true, false, 30.0f);
    run(&sel, pose, flow, false, true, 2.0f);
    EXPECT_TRUE(sel.flow_applied()) << "포즈로 돌아가는 것도 해제를 기다린다";
    EXPECT_FLOAT_EQ(out(sel).v[2], -0.03f * kDeg);
}

TEST(CalibrationSource, FlowReadyWhileEngagedWaitsForDisengage) {
    CalibrationSourceSelector sel;
    const float pose[3] = {0.0f, -0.5f * kDeg, -0.3f * kDeg};
    FlowCalibrationState collecting = usable_flow(-0.2f, -0.03f);
    collecting.status = 1;
    collecting.estimate_blocks = 2;
    sel.update(pose, collecting, true, false, 0.0f);
    run(&sel, pose, usable_flow(-0.2f, -0.03f), true, true, 2.0f);  // 주행 중 3블록째가 왔다
    EXPECT_FALSE(sel.flow_applied()) << "결합 중에는 광류로 바꾸지 않는다";
    EXPECT_NE(sel.flags() & kCalibrationSourcePending, 0u);
    EXPECT_FLOAT_EQ(out(sel).v[2], pose[2]);
    sel.update(pose, usable_flow(-0.2f, -0.03f), true, false, kFrameS);
    EXPECT_TRUE(sel.flow_applied());
    EXPECT_EQ(sel.flags() & kCalibrationSourcePending, 0u);
}

TEST(CalibrationSource, FallsBackToPoseEvenWhileEngaged) {
    CalibrationSourceSelector sel;
    const float pose[3] = {0.0f, -0.5f * kDeg, -0.3f * kDeg};
    sel.update(pose, usable_flow(-0.2f, -0.03f), true, false, 0.0f);
    ASSERT_TRUE(sel.flow_applied());
    FlowCalibrationState scattered = usable_flow(-0.2f, -0.03f);
    scattered.sd_yaw = 0.9f * kDeg;
    sel.update(pose, scattered, true, true, kFrameS);
    EXPECT_FALSE(sel.flow_applied()) << "쓸 만하지 않은 광류는 결합 중에도 놓는다";
    EXPECT_NE(sel.flags() & kCalibrationSourceRamping, 0u) << "천천히 돌아간다";
}

TEST(CalibrationSource, HysteresisKeepsAnAppliedEstimate) {
    CalibrationSourceSelector sel;
    const float pose[3] = {0.0f, -0.5f * kDeg, -0.3f * kDeg};
    FlowCalibrationState flow = usable_flow(-0.2f, -0.03f);
    flow.sd_yaw = 0.6f * kDeg;
    sel.update(pose, flow, true, false, 0.0f);
    EXPECT_FALSE(sel.flow_applied()) << "0.5°를 넘으면 쓰기 시작하지 않는다";
    flow.sd_yaw = 0.4f * kDeg;
    sel.update(pose, flow, true, false, kFrameS);
    EXPECT_TRUE(sel.flow_applied());
    flow.sd_yaw = 0.7f * kDeg;
    sel.update(pose, flow, true, false, kFrameS);
    EXPECT_TRUE(sel.flow_applied()) << "쓰는 중에는 0.8°까지 계속 쓴다";
    flow.sd_yaw = 0.85f * kDeg;
    sel.update(pose, flow, true, false, kFrameS);
    EXPECT_FALSE(sel.flow_applied());
}

TEST(CalibrationSource, AppliedEstimateIsFollowedWhileEngaged) {
    CalibrationSourceSelector sel;
    const float pose[3] = {0.0f, 0.0f, 0.0f};
    sel.update(pose, usable_flow(0.0f, 0.0f), true, false, 0.0f);
    run(&sel, pose, usable_flow(0.0f, 1.5f), true, true, 30.0f);  // 주행 중 장착 변경
    EXPECT_TRUE(sel.flow_applied());
    EXPECT_NEAR(out(sel).v[2], 1.5f * kDeg, 1e-6f) << "쓰고 있는 광류 추정의 변화는 결합 중에도 따라간다";
}

TEST(CalibrationSource, BackToPoseRampsThenPassesThrough) {
    CalibrationSourceSelector sel;
    float pose[3] = {0.0f, -0.5f * kDeg, -0.3f * kDeg};
    const FlowCalibrationState flow = usable_flow(-0.2f, -0.03f);
    sel.update(pose, flow, true, false, 0.0f);
    sel.update(pose, flow, false, false, kFrameS);
    EXPECT_FALSE(sel.flow_applied());
    EXPECT_NE(sel.flags() & kCalibrationSourceRamping, 0u);
    EXPECT_GT(out(sel).v[2], pose[2]) << "한 번에 돌아가지 않는다";
    run(&sel, pose, flow, false, false, 30.0f);
    EXPECT_FLOAT_EQ(out(sel).v[2], pose[2]);
    pose[2] = 0.4f * kDeg;
    sel.update(pose, flow, false, false, kFrameS);
    EXPECT_FLOAT_EQ(out(sel).v[2], pose[2]) << "다 돌아온 뒤에는 포즈 값 그대로";
}

TEST(CalibrationSource, UnusableFlowIsNotApplied) {
    const FlowCalibrationState good = usable_flow(-0.2f, -0.03f);
    EXPECT_TRUE(CalibrationSourceSelector::flow_usable(good));
    FlowCalibrationState f = good;
    f.status = 1;
    EXPECT_FALSE(CalibrationSourceSelector::flow_usable(f));
    f = good;
    f.sd_yaw = 2.0f * kDeg;
    EXPECT_FALSE(CalibrationSourceSelector::flow_usable(f)) << "블록끼리 맞지 않는다";
    f = good;
    f.yaw = 5.0f * kDeg;
    EXPECT_FALSE(CalibrationSourceSelector::flow_usable(f)) << "upstream yaw 한계(3.96°) 밖";
    f = good;
    f.pitch = std::nanf("");
    EXPECT_FALSE(CalibrationSourceSelector::flow_usable(f)) << "한계 비교에서 걸린다";

    CalibrationSourceSelector sel;
    const float pose[3] = {0.0f, -0.5f * kDeg, -0.3f * kDeg};
    sel.update(pose, f, true, false, 0.0f);
    EXPECT_FALSE(sel.flow_applied());
    EXPECT_FLOAT_EQ(out(sel).v[1], pose[1]);
    EXPECT_EQ(sel.flags() & kCalibrationSourceFlowUsable, 0u);
}

TEST(CalibrationSource, SnapToJumpsAfterAReset) {
    CalibrationSourceSelector sel;
    const float pose[3] = {0.0f, -0.5f * kDeg, -0.3f * kDeg};
    sel.update(pose, usable_flow(-0.2f, -0.03f), true, false, 0.0f);
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    sel.snap_to(zero);
    EXPECT_FLOAT_EQ(out(sel).v[2], 0.0f);
    FlowCalibrationState collecting;
    collecting.status = 1;
    sel.update(zero, collecting, true, false, kFrameS);
    EXPECT_FALSE(sel.flow_applied());
    EXPECT_FLOAT_EQ(out(sel).v[2], 0.0f) << "초기화 뒤 포즈 0에서 다시 시작한다";
}

}  // namespace
