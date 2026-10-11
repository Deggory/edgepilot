/* model/lead_flow: 앞차 상자 투영, 원본 해상도 점 고르기, 유사변환 확대율, 창. 합성 장면으로 본다(IVE 없이). */
#include "model/lead_flow.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

const FlowIntrinsics kK{754.16, 753.9, 626.7533, 368.4};  // MaixCAM2 1280x720
constexpr int kW = 1280, kH = 720;

LeadFlowInput lead_at(float x)
{
    LeadFlowInput in;
    in.valid = true;
    in.probability = 0.9f;
    in.x = x;
    in.camera_height_m = 1.22f;
    return in;
}

// 균등 잡음 ±amp(고정 씨앗)
class Noise {
public:
    explicit Noise(uint32_t seed = 11) : rng_(seed) {}
    float operator()(float amp)
    {
        rng_ = rng_ * 1103515245u + 12345u;
        return (static_cast<float>((rng_ >> 8) & 0xffff) / 65535.0f - 0.5f) * 2.0f * amp;
    }

private:
    uint32_t rng_;
};

TEST(LeadFlow, BoxIsThePinholeProjectionOfTheLeadsRear)
{
    const float rpy[3] = {};
    LeadBox box;
    ASSERT_TRUE(project_lead_box(kK, rpy, 1.22f, 0.0f, 30.0f, 0.0f, kW, kH, &box));
    // 폭 1.8 m를 양옆 12%씩 줄인 것: 754·1.8/30·0.76 = 34 px, 가운데는 cx
    EXPECT_NEAR(box.width(), 754.16 * 1.8 / 30.0 * 0.76, 2.0);
    EXPECT_NEAR(0.5 * (box.x0 + box.x1), 626.75, 1.5);
    // 도로 위 0.3~1.4 m: 카메라(1.22 m) 높이 둘레
    const double v_bottom = 368.4 + 753.9 * (1.22 - 0.3) / 30.0, v_top = 368.4 + 753.9 * (1.22 - 1.4) / 30.0;
    EXPECT_GT(box.y0, v_top);
    EXPECT_LT(box.y1, v_bottom + 1.0);

    // 장착 오프셋: 보드가 운전석 쪽(+)이면 모델의 정면은 실제 카메라에서 오른쪽이다(common/projection.cc)
    LeadBox shifted;
    ASSERT_TRUE(project_lead_box(kK, rpy, 1.22f, 0.3f, 30.0f, 0.0f, kW, kH, &shifted));
    EXPECT_NEAR(0.5 * (shifted.x0 + shifted.x1) - 0.5 * (box.x0 + box.x1), 754.16 * 0.3 / 30.0, 1.5);

    // 너무 멀거나 가까운 앞차, 영상 밖 상자는 없다
    EXPECT_FALSE(project_lead_box(kK, rpy, 1.22f, 0.0f, 95.0f, 0.0f, kW, kH, &box));
    EXPECT_FALSE(project_lead_box(kK, rpy, 1.22f, 0.0f, 5.0f, 0.0f, kW, kH, &box));
    EXPECT_FALSE(project_lead_box(kK, rpy, 1.22f, 0.0f, 10.0f, 9.0f, kW, kH, &box));
}

TEST(LeadFlow, CornersComeFromTheBoxTexture)
{
    std::vector<uint8_t> img(static_cast<size_t>(kW) * kH, 100);
    LeadBox box{600, 360, 660, 400};
    for (int y = box.y0; y < box.y1; ++y)  // 4 px 바둑판
        for (int x = box.x0; x < box.x1; ++x)
            img[static_cast<size_t>(y) * kW + x] = ((x / 4 + y / 4) % 2) ? 220 : 30;
    std::vector<float> xy(2 * kLeadMaxPoints);
    const int n = pick_lead_corners(img.data(), kW, box, kLeadMaxPoints, xy.data());
    EXPECT_GE(n, 30) << "무늬가 있는 상자에서는 점을 넉넉히 고른다";
    for (int i = 0; i < n; ++i) {
        EXPECT_GE(xy[2 * i], box.x0);
        EXPECT_LT(xy[2 * i], box.x1);
        EXPECT_GE(xy[2 * i + 1], box.y0);
        EXPECT_LT(xy[2 * i + 1], box.y1);
        for (int j = 0; j < i; ++j) {
            const float dx = xy[2 * i] - xy[2 * j], dy = xy[2 * i + 1] - xy[2 * j + 1];
            EXPECT_GE(dx * dx + dy * dy, 9.0f) << "고른 점끼리 3화소는 떨어진다";
        }
    }
    std::vector<uint8_t> flat(static_cast<size_t>(kW) * kH, 100);
    EXPECT_EQ(pick_lead_corners(flat.data(), kW, box, kLeadMaxPoints, xy.data()), 0) << "평평한 곳에는 점이 없다";
}

TEST(LeadFlow, SimilarityScaleIgnoresBackgroundPoints)
{
    std::vector<float> a, b;
    Noise noise(7);
    const float cx = 630.0f, cy = 380.0f, s = 1.05f;
    for (int i = 0; i < 30; ++i) {  // 앞차: 5% 커지고 (2, -1) 옮겨 간다
        const float x = cx - 20.0f + 40.0f * static_cast<float>(i % 6) / 5.0f;
        const float y = cy - 15.0f + 30.0f * static_cast<float>(i / 6) / 4.0f;
        a.push_back(x);
        a.push_back(y);
        b.push_back(cx + s * (x - cx) + 2.0f + noise(0.1f));
        b.push_back(cy + s * (y - cy) - 1.0f + noise(0.1f));
    }
    for (int i = 0; i < 10; ++i) {  // 배경: 제멋대로 움직인다
        a.push_back(cx + 3.0f * static_cast<float>(i));
        a.push_back(cy + 18.0f);
        b.push_back(cx + 3.0f * static_cast<float>(i) - 4.0f + static_cast<float>(i % 3));
        b.push_back(cy + 18.0f + 3.0f);
    }
    const ScaleFit fit = fit_similarity_scale(a.data(), b.data(), static_cast<int>(a.size() / 2), kLeadInlierPx);
    ASSERT_TRUE(fit.valid);
    EXPECT_NEAR(fit.scale, 1.05, 0.004);
    EXPECT_GE(fit.inliers, 28);
    EXPECT_LE(fit.inliers, 31) << "배경 점은 앞차의 확대·이동에 맞지 않는다";
}

/* 일정하게 다가오는 앞차. 뒷면 판(폭 1.8 m, 도로 위 0.3~1.4 m)에 부드러운 무늬를 칠해 거리 x(f)에서 본 영상을
 * 그리고(카메라 높이 1.22 m, 보정 0, 판 밖은 평평한 회색), LK 대신 판 위 점의 실제 이동(주점에서 x(f-1)/x(f)배로
 * 퍼진다)에 걸음마다 잡음을 더해 준다. */
class Approach {
public:
    Approach(float x0, float closing) : x0_(x0), closing_(closing) {}
    float x(int frame) const { return x0_ - closing_ * 0.05f * static_cast<float>(frame); }

    void render(int frame, std::vector<uint8_t> *img) const
    {
        img->assign(static_cast<size_t>(kW) * kH, 100);
        const double x = this->x(frame);
        const int u0 = static_cast<int>(std::floor(kK.cx - kK.fx * 0.9 / x)) - 1;
        const int u1 = static_cast<int>(std::ceil(kK.cx + kK.fx * 0.9 / x)) + 1;
        const int v0 = static_cast<int>(std::floor(kK.cy + kK.fy * (1.22 - 1.4) / x)) - 1;
        const int v1 = static_cast<int>(std::ceil(kK.cy + kK.fy * (1.22 - 0.3) / x)) + 1;
        for (int v = v0; v <= v1; ++v) {
            for (int u = u0; u <= u1; ++u) {
                const double lat = (u - kK.cx) * x / kK.fx, up = 1.22 - (v - kK.cy) * x / kK.fy;
                if (std::fabs(lat) > 0.9 || up < 0.3 || up > 1.4) continue;
                (*img)[static_cast<size_t>(v) * kW + u] = static_cast<uint8_t>(std::lround(texture(lat, up)));
            }
        }
    }
    // frame-1에서 (u, v)에 보이던 판 위 점이 frame에서 보이는 곳
    void move(int frame, float u, float v, float *u2, float *v2) const
    {
        const double s = static_cast<double>(x(frame - 1)) / x(frame);
        *u2 = static_cast<float>(kK.cx + (u - kK.cx) * s);
        *v2 = static_cast<float>(kK.cy + (v - kK.cy) * s);
    }

private:
    static double texture(double lat, double up)
    {
        const double tau = 6.283185307179586;
        return 128.0 + 45.0 * std::sin(tau * lat / 0.31) * std::sin(tau * up / 0.23) +
               25.0 * std::sin(tau * (lat + 0.7 * up) / 0.17);
    }
    float x0_, closing_;
};


/* 한 프레임: begin → (점이 있으면) 이동 + 잡음으로 따라가고, 되돌린 위치는 back_err만큼 어긋나게 → top_up.
 * back_err(점 번호, 위치)가 0이면 앞뒤 검사를 통과한다. */
template <class BackErr>
LeadFlowState step(LeadFlowTracker &tracker, const Approach &scene, int f, bool consecutive, float noise_px,
                   Noise &noise, BackErr back_err)
{
    std::vector<uint8_t> img;
    scene.render(f, &img);
    std::vector<float> in(2 * kLeadMaxPoints), out(2 * kLeadMaxPoints), back(2 * kLeadMaxPoints);
    std::vector<uint8_t> ok(kLeadMaxPoints, 1);
    if (tracker.begin(lead_at(scene.x(f)), static_cast<uint64_t>(f), 0.05 * f, consecutive, img.data(), kW)) {
        const int n = tracker.points(in.data());
        for (int j = 0; j < n; ++j) {
            scene.move(f, in[2 * j], in[2 * j + 1], &out[2 * j], &out[2 * j + 1]);
            out[2 * j] += noise(noise_px);
            out[2 * j + 1] += noise(noise_px);
            back[2 * j] = in[2 * j] + back_err(in[2 * j]);
            back[2 * j + 1] = in[2 * j + 1];
        }
        tracker.advance(out.data(), ok.data(), back.data(), ok.data());
    }
    const LeadFlowState st = tracker.state();
    tracker.top_up();
    return st;
}

auto no_back_err = [](float) { return 0.0f; };

TEST(LeadFlow, TrackerMeasuresTheClosingSpeedThroughChainDrift)
{
    LeadFlowTracker tracker(kK, kW, kH);
    const Approach scene(25.0f, 3.0f);  // 25 m에서 3 m/s로 다가온다
    Noise noise;
    LeadFlowState last;
    for (int f = 0; f <= 30; ++f) {
        // 걸음마다 ±0.4 px: 10걸음이면 점마다 0.7 px쯤 떠돌아 그대로 맞추면 대부분 0.7 px 문턱을 넘는다
        last = step(tracker, scene, f, true, 0.4f, noise, no_back_err);
        if (f < kLeadWindowSteps) EXPECT_FALSE(last.valid) << "창이 차기 전에는 내지 않는다";
        if (f >= kLeadWindowSteps) EXPECT_TRUE(last.valid) << f;
    }
    ASSERT_TRUE(last.valid);
    EXPECT_EQ(last.frame_id, 30u);
    EXPECT_NEAR(last.window_s, 0.5f, 1e-4);
    EXPECT_GE(last.points, 30u) << "무늬 판에서 점을 넉넉히 따라간다";
    EXPECT_GE(last.inliers, last.points * 9 / 10) << "10프레임 전 템플릿에 맞춰 떠돈 만큼을 되돌린다";
    // 창(0.5초)의 실제 크기 비로 낸 접근 속도: x(f)·ln(x(f-10)/x(f))/0.5
    const double expected = scene.x(30) * std::log(scene.x(20) / scene.x(30)) / 0.5;
    EXPECT_NEAR(-last.lead_x * last.expansion_rate, -expected, 0.1) << "상대 속도 -x·r";
}

TEST(LeadFlow, PointsThatDoNotTrackBackAreDroppedAndRefilled)
{
    LeadFlowTracker tracker(kK, kW, kH);
    const Approach scene(30.0f, 2.0f);
    Noise noise;
    for (int f = 0; f <= kLeadWindowSteps + 2; ++f) step(tracker, scene, f, true, 0.0f, noise, no_back_err);
    const int before = tracker.alive_points();
    ASSERT_GE(before, kLeadTopUpPoints);
    // 상자 왼쪽 절반의 점은 되돌리면 1 px 어긋난다(불빛 번짐처럼 미끄러진 점)
    const float center = 0.5f * static_cast<float>(tracker.box().x0 + tracker.box().x1);
    const int f = kLeadWindowSteps + 3;
    const LeadFlowState st =
        step(tracker, scene, f, true, 0.0f, noise, [center](float u) { return u < center ? 1.0f : 0.0f; });
    EXPECT_TRUE(st.valid) << "남은 점으로 창을 잇는다";
    EXPECT_LT(st.points, static_cast<uint32_t>(before) * 3 / 4) << "앞뒤가 맞지 않는 점은 버린다";
    EXPECT_GE(tracker.alive_points(), kLeadTopUpPoints) << "빈자리는 새 점으로 채운다";
    for (int g = f + 1; g <= f + kLeadWindowSteps + 1; ++g)
        EXPECT_TRUE(step(tracker, scene, g, true, 0.0f, noise, no_back_err).valid) << g;
    EXPECT_GT(tracker.state().points, st.points) << "채운 점도 창이 차면 쓴다";
}

TEST(LeadFlow, AnotherLeadOrAGapRestartsTheWindow)
{
    LeadFlowTracker tracker(kK, kW, kH);
    const Approach still(30.0f, 0.0f), near(18.0f, 0.0f);
    Noise noise;
    for (int f = 0; f <= kLeadWindowSteps; ++f) step(tracker, still, f, true, 0.0f, noise, no_back_err);
    ASSERT_TRUE(tracker.state().valid);
    EXPECT_NEAR(tracker.state().expansion_rate, 0.0f, 1e-4) << "움직이지 않으면 확대율 0";

    int f = kLeadWindowSteps + 1;
    EXPECT_FALSE(step(tracker, near, f++, true, 0.0f, noise, no_back_err).valid) << "거리가 튀면(다른 앞차) 창을 버린다";
    for (int i = 1; i < kLeadWindowSteps; ++i)
        EXPECT_FALSE(step(tracker, near, f++, true, 0.0f, noise, no_back_err).valid);
    EXPECT_TRUE(step(tracker, near, f++, true, 0.0f, noise, no_back_err).valid) << "새 앞차로 창이 다시 차면 낸다";

    EXPECT_FALSE(step(tracker, near, f + 3, false, 0.0f, noise, no_back_err).valid) << "프레임이 끊기면 창을 버린다";
    std::vector<uint8_t> img;
    near.render(0, &img);
    LeadFlowInput none = lead_at(18.0f);
    none.probability = 0.3f;
    EXPECT_FALSE(tracker.begin(none, 99, 5.0, true, img.data(), kW)) << "확률이 낮은 앞차는 따라가지 않는다";
    tracker.top_up();
    EXPECT_EQ(tracker.alive_points(), 0);
}

}  // namespace
