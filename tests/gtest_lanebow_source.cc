// Lanebow 차선 출처(planning/lanebow_source): LD 선을 따르는지, LD가 없거나 무효면 확률 0인지, 같은 LD를 두 번
// 섞지 않는지, LD 나이를 자차 움직임으로 메우는지, 빠진 선을 LD 시각으로 잠깐 붙잡는지, 선 밟기와 옆 차로 선을
// 버리는지, 시야 너머를 꺾임 없이 LD 폭 그대로 잇는지, 한쪽 선이 없으면 그 자리를 남은 선에서 기본 폭만큼 두는지 본다.
#include "common/ipc_messages.h"
#include "common/model_output.h"
#include "planning/lanebow_source.h"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace {

constexpr uint64_t kFrameNs = 50'000'000ULL;

double grid(int i) { return model_x_idx(i); }

// supercombo가 낸 프레임: 직진 plan, 자차 좌우 선(곡률 kappa의 포물선)과 확률, 촬영 시각
ModelState model_at(uint64_t t_ns, float left, float right, float prob = 0.95f, float kappa = 0.0f, float v = 20.0f) {
    ModelState ms{};
    ms.valid = 1;
    ms.capture_timestamp_ns = t_ns;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float t = model_t_idx(i);
        const float x = model_x_idx(i);
        ms.model_t[i] = t;
        ms.lane_t[i] = x / v;
        ms.plan[i] = {v * t, 0.5f * kappa * v * t * v * t, 0.0f};
        ms.lanes[1][i] = {x, left + 0.5f * kappa * x * x, 1.2f};
        ms.lanes[2][i] = {x, right + 0.5f * kappa * x * x, 1.2f};
    }
    ms.lane_probabilities[1] = ms.lane_probabilities[2] = prob;
    ms.lane_stds[1] = ms.lane_stds[2] = 0.05f;
    return ms;
}

// LD 결과: 자차 선을 reach m까지(곡률 kappa의 포물선), 신뢰도 confidence
void add_line(LaneState *lane, int8_t slot, float y0, float kappa = 0.0f, float reach = 36.0f,
              float confidence = 0.9f) {
    LaneRoadLine &road = lane->road[lane->road_count++];
    road.kind = kLaneRoadKindLine;
    road.slot = slot;
    road.confidence = confidence;
    for (unsigned i = 0; i < kLaneRoadPoints; ++i) {
        const float x = model_x_idx(static_cast<int>(i));
        if (x > reach) break;
        road.y[i] = y0 + 0.5f * kappa * x * x;
        if (x >= 4.5f && road.first < 0) road.first = static_cast<int8_t>(i);
        road.last = static_cast<int8_t>(i);
    }
}

LaneState lane_at(uint64_t frame, uint64_t t_ns) {
    LaneState lane;
    lane.frame_id = frame;
    lane.capture_timestamp_ns = t_ns;
    lane.valid = 1;
    return lane;
}

TEST(LanebowSource, FollowsTheLdLines) {
    LanebowSource source;
    LaneState lane = lane_at(1, 10 * kFrameNs);
    add_line(&lane, kLaneSlotEgoLeft, -1.6f);
    add_line(&lane, kLaneSlotEgoRight, 1.9f);
    const LaneMeasurement m = source.measure(&lane, model_at(10 * kFrameNs, -1.75f, 1.75f), 20.0, 0.0);
    EXPECT_DOUBLE_EQ(m.left_prob, 1.0);
    EXPECT_DOUBLE_EQ(m.right_prob, 1.0);
    EXPECT_EQ(source.lane_frame_id(), 1u);
    EXPECT_NEAR(source.lane_age_s(), 0.0, 1e-9);
    for (int i = 0; i < kTrajectorySize; ++i) {
        ASSERT_TRUE(std::isfinite(m.left[i]) && std::isfinite(m.right[i])) << i;
        EXPECT_NEAR(m.left[i], -1.6, 1e-3) << "x " << grid(i) << ": 시야 너머도 LD 위치를 잇는다";
        EXPECT_NEAR(m.right[i], 1.9, 1e-3) << "x " << grid(i);
    }
    EXPECT_EQ(m.width_index, LanebowSource::kWidthIndex);
}

TEST(LanebowSource, NoUsableLdGivesZeroProbability) {
    LanebowSource source;
    const ModelState model = model_at(kFrameNs, -1.75f, 1.75f);
    for (const bool invalid : {false, true}) {
        LaneState unfilled = lane_at(3, kFrameNs);
        add_line(&unfilled, kLaneSlotEgoLeft, -1.6f);
        unfilled.valid = 0;
        const LaneMeasurement m = source.measure(invalid ? &unfilled : nullptr, model, 20.0, 0.0);
        EXPECT_EQ(m.left_prob, 0.0);
        EXPECT_EQ(m.right_prob, 0.0);
        EXPECT_NEAR(m.left[0], -1.75, 1e-6) << "선은 유한한 자리 표시(supercombo 선)";
        EXPECT_LT(source.lane_age_s(), 0.0);
    }
    LaneState weak = lane_at(4, kFrameNs);
    add_line(&weak, kLaneSlotEgoLeft, -1.6f, 0.0f, 36.0f, 0.1f);  // 신뢰도 낮음
    add_line(&weak, kLaneSlotEgoRight, 1.9f, 0.0f, 9.0f);         // 9 m까지만
    const LaneMeasurement m = source.measure(&weak, model, 20.0, 0.0);
    EXPECT_EQ(m.left_prob, 0.0);
    EXPECT_EQ(m.right_prob, 0.0);
}

// LD는 보통 한 프레임 늦어 같은 LaneState가 두 플래너 프레임에 올 수 있다. 다시 섞으면 같은 측정을 두 번 믿는다.
TEST(LanebowSource, RepeatedLaneStateIsNotBlendedTwice) {
    LanebowSource source;
    LaneState first = lane_at(1, 10 * kFrameNs);
    add_line(&first, kLaneSlotEgoLeft, -1.6f);
    add_line(&first, kLaneSlotEgoRight, 1.9f);
    source.measure(&first, model_at(10 * kFrameNs, -1.75f, 1.75f), 20.0, 0.0);
    LaneState second = lane_at(2, 11 * kFrameNs);
    add_line(&second, kLaneSlotEgoLeft, -1.5f);
    add_line(&second, kLaneSlotEgoRight, 1.9f);
    const LaneMeasurement once = source.measure(&second, model_at(11 * kFrameNs, -1.75f, 1.75f), 20.0, 0.0);
    const LaneMeasurement again = source.measure(&second, model_at(12 * kFrameNs, -1.75f, 1.75f), 20.0, 0.0);
    EXPECT_NEAR(once.left[8], -1.6 + kLaneLineFilterWeight * 0.1, 1e-4) << "시간 필터가 섞었다";
    EXPECT_NEAR(again.left[8], once.left[8], 1e-4) << "같은 LD는 옮기기만 한다";
    EXPECT_NEAR(source.lane_age_s(), 0.05, 1e-9);
}

// 차가 도는 동안 한 프레임 늦은 LD 선은 자차가 돈 만큼 어긋난다. 모델 시각으로 옮기면 그 시각의 선과 맞는다.
TEST(LanebowSource, CompensatesTheLdAge) {
    const double v = 25.0, yaw_rate = 0.06;
    // 곧은 길의 선 Y = offset을 방향 heading인 차에서 본다(횡위치는 0으로 둔다): y = (offset - x·sin h) / cos h
    auto seen = [](double offset, double heading, double x) {
        return (offset - x * std::sin(heading)) / std::cos(heading);
    };
    auto lane_with_heading = [&](uint64_t frame, uint64_t t_ns, double heading) {
        LaneState lane = lane_at(frame, t_ns);
        for (const auto &[slot, offset] :
             {std::pair<int8_t, double>{kLaneSlotEgoLeft, -1.7}, std::pair<int8_t, double>{kLaneSlotEgoRight, 1.8}}) {
            LaneRoadLine &road = lane.road[lane.road_count++];
            road.kind = kLaneRoadKindLine;
            road.slot = slot;
            road.confidence = 0.9f;
            for (unsigned i = 0; i < kLaneRoadPoints && grid(static_cast<int>(i)) <= 36.0; ++i)
                road.y[i] = static_cast<float>(seen(offset, heading, grid(static_cast<int>(i))));
        }
        return lane;
    };
    LanebowSource source;
    source.set_filter_weight(1.0);  // 섞지 않고 옮기기만 본다
    const LaneState lane = lane_with_heading(1, 20 * kFrameNs, 0.0);
    const ModelState model = model_at(21 * kFrameNs, -1.75f, 1.75f, 0.95f, 0.0f, static_cast<float>(v));
    const LaneMeasurement m = source.measure(&lane, model, v, yaw_rate / v);
    const double heading_now = yaw_rate * 0.05;
    const int i20 = 10;  // 18.75 m
    EXPECT_NEAR(m.left[i20], seen(-1.7, heading_now, grid(i20) + 0.0), 0.01) << "모델 시각의 선";
    EXPECT_GT(std::fabs(lane.road[0].y[i20] - seen(-1.7, heading_now, grid(i20))), 0.04) << "옮기지 않으면 5 cm";
}

TEST(LanebowSource, HoldsADroppedSideBriefly) {
    LanebowSource source;
    LaneState both = lane_at(1, 10 * kFrameNs);
    add_line(&both, kLaneSlotEgoLeft, -1.6f);
    add_line(&both, kLaneSlotEgoRight, 1.9f);
    source.measure(&both, model_at(10 * kFrameNs, -1.75f, 1.75f), 20.0, 0.0);
    double right_prob[4] = {};
    double right_near[4] = {};
    for (int k = 1; k <= 4; ++k) {
        LaneState left_only = lane_at(1 + k, (10 + k) * kFrameNs);
        add_line(&left_only, kLaneSlotEgoLeft, -1.6f);
        const LaneMeasurement m = source.measure(&left_only, model_at((10 + k) * kFrameNs, -1.75f, 1.75f), 20.0, 0.0);
        right_prob[k - 1] = m.right_prob;
        right_near[k - 1] = m.right[0];
        EXPECT_DOUBLE_EQ(m.left_prob, 1.0);
    }
    EXPECT_DOUBLE_EQ(right_prob[0], 1.0) << "한 프레임 빠짐은 그대로";
    EXPECT_DOUBLE_EQ(right_prob[1], 1.0) << "두 프레임(0.1 s)도";
    EXPECT_DOUBLE_EQ(right_prob[2], 0.0) << "0.15 s 뒤에는 0(차선 코어가 2/s로 내린다)";
    EXPECT_NEAR(right_near[2], 1.9, 1e-3) << "그동안 선은 예측으로 유한하다";
}

// 붙잡는 시간은 LD 시각으로 잰다. LD가 늘 한 프레임 늦어도 빠진 선을 LD 두 프레임 동안 붙잡는다(모델 시각으로 재면
// 늦은 만큼 짧아져 한 프레임만 붙잡았다).
TEST(LanebowSource, HoldUsesTheLdClock) {
    LanebowSource source;
    LaneState both = lane_at(1, 9 * kFrameNs);
    add_line(&both, kLaneSlotEgoLeft, -1.6f);
    add_line(&both, kLaneSlotEgoRight, 1.9f);
    source.measure(&both, model_at(10 * kFrameNs, -1.75f, 1.75f), 20.0, 0.0);
    double right_prob[3] = {};
    for (int k = 1; k <= 3; ++k) {
        LaneState left_only = lane_at(1 + k, (9 + k) * kFrameNs);  // 모델보다 한 프레임 늦은 LD
        add_line(&left_only, kLaneSlotEgoLeft, -1.6f);
        right_prob[k - 1] =
            source.measure(&left_only, model_at((10 + k) * kFrameNs, -1.75f, 1.75f), 20.0, 0.0).right_prob;
    }
    EXPECT_DOUBLE_EQ(right_prob[0], 1.0);
    EXPECT_DOUBLE_EQ(right_prob[1], 1.0) << "LD 두 프레임째도(모델 시각으로는 0.15 s)";
    EXPECT_DOUBLE_EQ(right_prob[2], 0.0);
    EXPECT_NEAR(source.lane_age_s(), 0.05, 1e-9) << "쓴 선(왼선)의 LD 나이";
}

TEST(LanebowSource, RejectsStraddlingAndCrossedLines) {
    const ModelState model = model_at(kFrameNs, -1.75f, 1.75f);
    LanebowSource straddle;
    LaneState over = lane_at(1, kFrameNs);
    add_line(&over, kLaneSlotEgoLeft, 0.2f);  // 왼선이 차 오른쪽에 있다(선 밟기)
    add_line(&over, kLaneSlotEgoRight, 3.6f);
    LaneMeasurement m = straddle.measure(&over, model, 20.0, 0.0);
    EXPECT_EQ(m.left_prob, 0.0);
    EXPECT_EQ(m.right_prob, 1.0) << "제자리에 있는 선은 쓴다";

    LanebowSource narrow;
    LaneState close = lane_at(1, kFrameNs);
    add_line(&close, kLaneSlotEgoLeft, -0.9f);
    add_line(&close, kLaneSlotEgoRight, 0.9f);  // 폭 1.8 m
    m = narrow.measure(&close, model, 20.0, 0.0);
    EXPECT_EQ(m.left_prob, 0.0);
    EXPECT_EQ(m.right_prob, 0.0);

    // 한 줄만 보일 때 옆 차로 선을 자차 선으로 붙인 경우(차에서 3.6 m 넘게 떨어짐)는 버린다
    for (const float y0 : {-5.2f, -3.0f}) {
        LanebowSource lone;
        LaneState far = lane_at(1, kFrameNs);
        add_line(&far, kLaneSlotEgoLeft, y0);
        m = lone.measure(&far, model, 20.0, 0.0);
        EXPECT_EQ(m.left_prob, y0 < -LanebowSource::kMaxEgoOffsetM ? 0.0 : 1.0) << y0;
    }

    LanebowSource crossed;
    LaneState cross = lane_at(1, kFrameNs);
    add_line(&cross, kLaneSlotEgoLeft, -1.5f, 0.02f);   // 오른쪽으로 휘어
    add_line(&cross, kLaneSlotEgoRight, 1.5f, -0.02f);  // 왼쪽으로 휘어 15 m 안에서 엇갈린다
    m = crossed.measure(&cross, model, 20.0, 0.0);
    EXPECT_EQ(m.left_prob, 0.0);
    EXPECT_EQ(m.right_prob, 0.0);
}

// LD는 30 m 남짓까지 본다. 그 너머는 같은 쪽 supercombo 선의 모양에 LD 위치를 맞춰 잇는다: 커브가 이어지고, 이음매에
// 꺾임이 없다.
TEST(LanebowSource, ExtensionJoinsWithoutAKink) {
    const float kappa = 0.004f;
    LanebowSource source;
    LaneState lane = lane_at(1, kFrameNs);
    add_line(&lane, kLaneSlotEgoLeft, -1.6f, kappa, 30.0f);
    add_line(&lane, kLaneSlotEgoRight, 1.9f, kappa, 30.0f);
    // supercombo 선은 같은 커브지만 0.2 m 오른쪽으로 치우쳤다
    const ModelState model = model_at(kFrameNs, -1.55f, 1.95f, 0.9f, kappa);
    const LaneMeasurement m = source.measure(&lane, model, 20.0, 0.0);
    for (int i = 0; i < 20; ++i) {
        const double x = grid(i);
        EXPECT_NEAR(m.left[i], -1.6 + 0.5 * kappa * x * x, 0.05) << "x " << x;
    }
    // 이음매(27 m 격자) 앞뒤의 기울기가 이어진다
    for (int i = 10; i < 17; ++i) {
        const double slope = (m.left[i + 1] - m.left[i]) / (grid(i + 1) - grid(i));
        const double expected = kappa * 0.5 * (grid(i + 1) + grid(i));
        EXPECT_NEAR(slope, expected, 0.01) << "x " << grid(i);
    }
}

// 두 선은 시야 너머에서 같은 모양을 쓴다: supercombo 왼선만 먼 곳에서 왼쪽으로 벌어져도(분기, 먼 곳 오차) 두 LD 선은
// 나란히 이어져 폭이 LD 폭 그대로다. 선마다 다른 모양을 빌리면 먼 폭이 커져 차선 코어의 폭 검사가 확률을 깎았다.
TEST(LanebowSource, ExtensionKeepsTheLdWidth) {
    LanebowSource source;
    LaneState lane = lane_at(1, kFrameNs);
    add_line(&lane, kLaneSlotEgoLeft, -1.7f, 0.0f, 30.0f);
    add_line(&lane, kLaneSlotEgoRight, 1.8f, 0.0f, 30.0f);
    ModelState model = model_at(kFrameNs, -1.75f, 1.75f);
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float x = model_x_idx(i);
        if (x > 40.0f) model.lanes[1][i].y -= 0.002f * (x - 40.0f) * (x - 40.0f);  // 90 m에서 5 m 벌어진다
    }
    const LaneMeasurement m = source.measure(&lane, model, 20.0, 0.0);
    for (int i = 0; i < kTrajectorySize; ++i) EXPECT_NEAR(m.right[i] - m.left[i], 3.5, 0.02) << "x " << grid(i);
    EXPECT_LT(m.left[kTrajectorySize - 1], -1.7 - 1.0) << "모양은 supercombo 두 선을 섞어 따른다";
}

// 한쪽 LD 선이 없으면(1초 넘게 빠짐, 처음부터 없음) 그 자리는 남은 선을 기본 폭만큼 옮긴 선이다(확률 0). 엉뚱한
// supercombo 선을 두면 폭 검사가 남은 선까지 깎는다.
TEST(LanebowSource, MissingSideMirrorsTheOtherLine) {
    LanebowSource source;
    LaneState both = lane_at(1, 10 * kFrameNs);
    add_line(&both, kLaneSlotEgoLeft, -1.6f);
    add_line(&both, kLaneSlotEgoRight, 2.0f);
    source.measure(&both, model_at(10 * kFrameNs, -1.75f, 1.75f), 20.0, 0.0);
    LaneMeasurement m;
    for (int k = 1; k <= 30; ++k) {
        LaneState left_only = lane_at(1 + k, (10 + k) * kFrameNs);
        add_line(&left_only, kLaneSlotEgoLeft, -1.6f);
        m = source.measure(&left_only, model_at((10 + k) * kFrameNs, -1.75f, 8.0f, 0.3f), 20.0, 0.0);
    }
    EXPECT_DOUBLE_EQ(m.left_prob, 1.0);
    EXPECT_EQ(m.right_prob, 0.0);
    for (int i = 0; i < kTrajectorySize; ++i)
        EXPECT_NEAR(m.right[i] - m.left[i], LanebowSource::kDefaultWidthM, 1e-6) << "x " << grid(i);

    LanebowSource fresh;
    LaneState right_only = lane_at(1, kFrameNs);
    add_line(&right_only, kLaneSlotEgoRight, 1.8f);
    m = fresh.measure(&right_only, model_at(kFrameNs, -6.0f, 1.75f), 20.0, 0.0);
    EXPECT_NEAR(m.right[0] - m.left[0], LanebowSource::kDefaultWidthM, 1e-6);
}

}  // namespace
