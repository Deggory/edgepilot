/* model/lead_tracker: 모델 앞차 거리와 광류 확대율을 섞는 칼만 필터. 잡음을 정해 둔 합성 앞차로 본다. */
#include "model/lead_tracker.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

constexpr double kDt = 0.05;  // 20 Hz

// 고정 씨앗 정규 잡음
class Gauss {
public:
    double operator()(double sd)
    {
        const double u1 = (next() + 0.5) / 4294967296.0, u2 = (next() + 0.5) / 4294967296.0;
        return sd * std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
    }

private:
    double next()
    {
        rng_ = rng_ * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(rng_ >> 32);
    }
    uint64_t rng_ = 12345;
};

LeadTrackInput measured(double t, double d, double v, Gauss &noise, double d_sd, bool flow, uint64_t frame)
{
    LeadTrackInput in;
    in.t_s = t;
    in.lead_valid = true;
    in.distance_m = static_cast<float>(d + noise(d_sd));
    in.relative_speed_mps = static_cast<float>(0.4 * v);  // 모델은 상대 속도를 덜 읽는다(시작 때만 쓴다)
    in.flow_valid = flow;
    in.flow_frame_id = frame;
    in.flow_rate = static_cast<float>(-v / d + noise(0.03));
    return in;
}

double std_of_steps(const std::vector<double> &x)
{
    double s = 0.0, s2 = 0.0;
    for (size_t i = 1; i < x.size(); ++i) {
        const double d = x[i] - x[i - 1];
        s += d;
        s2 += d * d;
    }
    const double n = static_cast<double>(x.size() - 1);
    return std::sqrt(s2 / n - (s / n) * (s / n));
}

TEST(LeadTracker, ClosingLeadIsSmoothedAndItsSpeedFound)
{
    LeadTracker tracker;
    Gauss noise;
    std::vector<double> raw, fused;
    double v_err = 0.0, worst = 0.0;
    int n = 0;
    for (int f = 0; f <= 200; ++f) {  // 40 m에서 3 m/s로 10초 다가온다(10 m까지)
        const double t = f * kDt, d = 40.0 - 3.0 * t;
        const LeadTrackInput in = measured(t, d, -3.0, noise, 0.5 + 0.025 * d, true, static_cast<uint64_t>(f));
        tracker.update(in);
        const LeadTrackState &st = tracker.state();
        ASSERT_EQ(st.valid, 1u);
        if (t < 3.0) continue;
        raw.push_back(in.distance_m);
        fused.push_back(st.distance_m);
        v_err += std::fabs(st.relative_speed_mps + 3.0);
        worst = std::max(worst, std::fabs(st.relative_speed_mps + 3.0));
        ++n;
    }
    EXPECT_LT(std_of_steps(fused), 0.3 * std_of_steps(raw)) << "프레임마다 흔들림이 크게 준다";
    EXPECT_LT(v_err / n, 0.4) << "모델이 덜 읽는 상대 속도에서 시작해도 실제 접근 속도를 찾는다";
    EXPECT_LT(worst, 1.2);
    EXPECT_NEAR(tracker.state().distance_m, 40.0 - 3.0 * 10.0, 1.0);
}

TEST(LeadTracker, DistanceOutliersAreDroppedButAnotherLeadStartsOver)
{
    LeadTracker tracker;
    Gauss noise;
    int f = 0;
    for (; f < 60; ++f) tracker.update(measured(f * kDt, 30.0, 0.0, noise, 0.3, false, 0));
    const float before = tracker.state().distance_m;
    LeadTrackInput spike = measured(f * kDt, 30.0, 0.0, noise, 0.0, false, 0);
    spike.distance_m = 38.0f;  // 한 프레임 8 m 튄다
    tracker.update(spike);
    ++f;
    EXPECT_NE(tracker.state().flags & kLeadTrackDistanceRejected, 0u);
    EXPECT_NEAR(tracker.state().distance_m, before, 0.3) << "튄 값 하나는 따라가지 않는다";
    tracker.update(measured(f * kDt, 30.0, 0.0, noise, 0.0, false, 0));
    ++f;
    // 끼어든 차: 15 m에 머문다. 4프레임은 버티고 5프레임째 다시 시작한다.
    for (int k = 0; k < kLeadTrackRestartMisses; ++k, ++f) {
        tracker.update(measured(f * kDt, 15.0, 0.0, noise, 0.0, false, 0));
        if (k < kLeadTrackRestartMisses - 1) EXPECT_GT(tracker.state().distance_m, 25.0f) << k;
    }
    EXPECT_NE(tracker.state().flags & kLeadTrackStarted, 0u);
    EXPECT_NEAR(tracker.state().distance_m, 15.0, 0.01);
}

/* 앞차가 갑자기 3 m/s로 다가오기 시작한다. 확대율을 넣은 추적은 모델 거리만 거른 추적과 비슷하게 따라붙고
 * (지연은 거의 같다), 그 뒤로 상대 속도가 더 정확하고 덜 흔들린다. */
TEST(LeadTracker, FlowMakesTheClosingSpeedSteadierThanDistanceAlone)
{
    LeadTracker with_flow, distance_only;
    Gauss noise_a, noise_b;
    double first_a = -1.0, first_b = -1.0, err_a = 0.0, err_b = 0.0;
    std::vector<double> va_hist, vb_hist;
    int n = 0;
    for (int f = 0; f <= 400; ++f) {
        const double t = f * kDt, closing = t < 5.0 ? 0.0 : 2.0;
        const double d = t < 5.0 ? 45.0 : 45.0 - 2.0 * (t - 5.0);
        with_flow.update(measured(t, d, -closing, noise_a, 0.5 + 0.025 * d, true, static_cast<uint64_t>(f)));
        distance_only.update(measured(t, d, -closing, noise_b, 0.5 + 0.025 * d, false, 0));
        if (t < 5.0) continue;
        const double va = with_flow.state().relative_speed_mps, vb = distance_only.state().relative_speed_mps;
        if (first_a < 0.0 && va < -1.5) first_a = t - 5.0;
        if (first_b < 0.0 && vb < -1.5) first_b = t - 5.0;
        if (t >= 7.0) {
            err_a += (va + 2.0) * (va + 2.0);
            err_b += (vb + 2.0) * (vb + 2.0);
            va_hist.push_back(va);
            vb_hist.push_back(vb);
            ++n;
        }
    }
    ASSERT_GT(first_a, 0.0);
    ASSERT_GT(first_b, 0.0);
    EXPECT_LT(first_a, first_b + 0.3) << "따라붙는 데 걸리는 시간은 거의 같다";
    const double rms_a = std::sqrt(err_a / n), rms_b = std::sqrt(err_b / n);
    EXPECT_LT(rms_a, 0.75 * rms_b) << "확대율이 있으면 더 정확하다: " << rms_a << " vs " << rms_b;
    EXPECT_LT(std_of_steps(va_hist), 0.75 * std_of_steps(vb_hist)) << "덜 흔들린다";
}

TEST(LeadTracker, FlowSpikesAndRepeatsAreNotUsed)
{
    LeadTracker tracker;
    Gauss noise;
    int f = 0;
    for (; f < 80; ++f) tracker.update(measured(f * kDt, 30.0, 0.0, noise, 0.3, true, static_cast<uint64_t>(f)));
    const float v_before = tracker.state().relative_speed_mps;
    LeadTrackInput spike = measured(f * kDt, 30.0, 0.0, noise, 0.0, true, static_cast<uint64_t>(f));
    spike.flow_rate = 0.5f;  // 30 m에서 -15 m/s라는 확대율
    tracker.update(spike);
    ++f;
    EXPECT_NE(tracker.state().flags & kLeadTrackFlowRejected, 0u);
    EXPECT_NEAR(tracker.state().relative_speed_mps, v_before, 0.3);
    // 광류 작업이 늦어 같은 확대율(같은 창 끝 프레임)이 다시 오면 넣지 않는다
    LeadTrackInput first = measured(f * kDt, 30.0, 0.0, noise, 0.0, true, 1000);
    tracker.update(first);
    ++f;
    EXPECT_NE(tracker.state().flags & kLeadTrackFlowUsed, 0u);
    LeadTrackInput again = measured(f * kDt, 30.0, 0.0, noise, 0.0, true, 1000);
    tracker.update(again);
    EXPECT_EQ(tracker.state().flags & (kLeadTrackFlowUsed | kLeadTrackFlowRejected), 0u);
}

TEST(LeadTracker, LostLeadOrAGapStartsOver)
{
    LeadTracker tracker;
    Gauss noise;
    for (int f = 0; f < 20; ++f) tracker.update(measured(f * kDt, 25.0, -1.0, noise, 0.3, false, 0));
    ASSERT_EQ(tracker.state().valid, 1u);
    LeadTrackInput none;
    none.t_s = 20 * kDt;
    tracker.update(none);
    EXPECT_EQ(tracker.state().valid, 0u) << "앞차가 없으면 추적도 없다";
    tracker.update(measured(21 * kDt, 25.0, -1.0, noise, 0.0, false, 0));
    EXPECT_NE(tracker.state().flags & kLeadTrackStarted, 0u);
    tracker.update(measured(22 * kDt, 25.0, -1.0, noise, 0.0, false, 0));
    EXPECT_EQ(tracker.state().flags & kLeadTrackStarted, 0u);
    tracker.update(measured(22 * kDt + 0.8, 25.0, -1.0, noise, 0.0, false, 0));
    EXPECT_NE(tracker.state().flags & kLeadTrackStarted, 0u) << "프레임이 0.5초 넘게 끊기면 다시 시작한다";
    LeadTrackInput unknown = measured(40.0, 25.0, -1.0, noise, 0.0, false, 0);
    unknown.relative_speed_mps = NAN;
    tracker.update(unknown);
    EXPECT_EQ(tracker.state().relative_speed_mps, 0.0f) << "자차 속도를 모르면 상대 속도 0에서 시작한다";
}

}  // namespace
