/* 차선선 시간 필터(planning/lane_line_filter): 도로에 고정된 선을 따라가는 동안 자차 움직임 보정이 늦지
 * 않는지, 잡음을 얼마나 줄이는지, 선이 바뀌거나 프레임이 끊기면 다시 시작하는지 본다. */
#include "planning/lane_line_filter.h"

#include "common/model_output.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <random>

namespace {

constexpr double kDt = 0.05;
constexpr double kLaneOffset = -1.8;  // 차로 중심에서 왼쪽 선까지

// 모델 차선 점의 거리(X_IDXS): 0~192 m, 제곱 간격
double x_idx(int i) { return 192.0 * (i / 32.0) * (i / 32.0); }

/* 곡률 kappa의 원을 따라가는 차에서 본, 차로 중심에서 kLaneOffset 옆의 선(동심원)의 횡위치. 원 위에서는
 * 어디서 보아도 같다. 차 기준 원의 중심은 (0, 1/kappa)이고 오른쪽이 +다. */
double lane_y_on_circle(double kappa, double x)
{
  if (std::fabs(kappa) < 1e-12) return kLaneOffset;
  const double r = 1.0 / kappa - kLaneOffset;
  return 1.0 / kappa - std::copysign(std::sqrt(std::max(0.0, r * r - x * x)), kappa);
}

struct DriveResult {
  double mean_error = 0.0;   // 거른 선 - 실제 선, 10~40 m 점 평균
  double rms_error = 0.0;
};

// v로 곡률 kappa의 원을 따라가며 잡음 sigma의 측정을 넣는다(처음 40 프레임은 버린다)
DriveResult drive(double weight, double kappa, double v, double sigma, int frames)
{
  LaneLineFilter filter;
  filter.set_measurement_weight(weight);
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, sigma);
  float x[kTrajectorySize], y[kTrajectorySize], out[kTrajectorySize];
  double sum = 0.0, sum2 = 0.0;
  int n = 0;
  for (int k = 0; k < frames; ++k) {
    for (int i = 0; i < kTrajectorySize; ++i) {
      x[i] = static_cast<float>(x_idx(i));
      y[i] = static_cast<float>(lane_y_on_circle(kappa, x_idx(i)) + noise(rng));
    }
    filter.update(k * kDt, x, y, v, kappa, out);
    if (k < 40) continue;
    for (int i = 0; i < kTrajectorySize; ++i) {
      if (x_idx(i) < 10.0 || x_idx(i) > 40.0) continue;
      const double e = out[i] - lane_y_on_circle(kappa, x_idx(i));
      sum += e;
      sum2 += e * e;
      ++n;
    }
  }
  return {sum / n, std::sqrt(sum2 / n)};
}

TEST(LaneLineFilter, WeightOnePassesTheMeasurementThrough) {
  LaneLineFilter filter;
  filter.set_measurement_weight(1.0);
  float x[kTrajectorySize], y[kTrajectorySize], out[kTrajectorySize];
  for (int k = 0; k < 3; ++k) {
    for (int i = 0; i < kTrajectorySize; ++i) {
      x[i] = static_cast<float>(x_idx(i));
      y[i] = static_cast<float>(-1.7 + 0.01 * k + 0.001 * i);
    }
    filter.update(k * kDt, x, y, 20.0, 0.001, out);
    for (int i = 0; i < kTrajectorySize; ++i) ASSERT_EQ(out[i], y[i]);
  }
}

/* 굽은 길을 따라가는 동안 선은 차체 좌표에서 그대로지만 점마다 다른 곳을 본다. 예측이 맞으면 치우침이 없다.
 * 직선 보간으로 예측을 읽으면 선이 커브 안쪽으로 8~17 mm 끌려갔다. */
TEST(LaneLineFilter, NoBiasOnACurve) {
  for (const double kappa : {0.0, 0.002, -0.004}) {
    const DriveResult run = drive(kLaneLineFilterWeight, kappa, 25.0, 0.0, 200);
    EXPECT_LT(std::fabs(run.mean_error), 0.002) << "kappa " << kappa;
  }
}

// 곧은 길에서 차가 방향을 틀면 차체 좌표의 선이 돈다. 자차 움직임만큼 옮겨 예측하므로 늦지 않는다.
TEST(LaneLineFilter, FollowsTheLinesWhileTheCarYaws) {
  LaneLineFilter filter;
  const double v = 25.0, yaw_rate = 0.02;   // 0.02 rad/s로 오른쪽으로 튼다
  float x[kTrajectorySize], y[kTrajectorySize], out[kTrajectorySize];
  double heading = 0.0, lateral = 0.0;
  for (int k = 0; k < 40; ++k) {
    for (int i = 0; i < kTrajectorySize; ++i) {
      // 도로의 곧은 선 Y = kLaneOffset을 횡위치 lateral, 방향 heading인 차에서 본다: 차체 점 (x, u)는
      // 도로에서 Y = lateral + x·sin(heading) + u·cos(heading)
      const double xi = x_idx(i);
      x[i] = static_cast<float>(xi);
      y[i] = static_cast<float>((kLaneOffset - lateral - xi * std::sin(heading)) / std::cos(heading));
    }
    filter.update(k * kDt, x, y, v, yaw_rate / v, out);
    heading += yaw_rate * kDt;
    lateral += v * std::sin(heading) * kDt;
  }
  // 2초 동안 27 m 점의 선이 2 m 옮겨 갔다. 같은 거리의 점끼리 섞었다면 7.5 cm 뒤처진다.
  EXPECT_NEAR(out[12], y[12], 0.01);
  EXPECT_NEAR(out[20], y[20], 0.02);
}

TEST(LaneLineFilter, NoiseFallsWithTheWeight) {
  const double sigma = 0.05;
  const double raw = drive(1.0, 0.001, 25.0, sigma, 400).rms_error;
  const double filtered = drive(kLaneLineFilterWeight, 0.001, 25.0, sigma, 400).rms_error;
  EXPECT_NEAR(raw, sigma, 0.005);
  // 흰 잡음이면 sqrt(w / (2 - w)): 가중 0.5에서 0.58배
  EXPECT_LT(filtered, 0.65 * raw);
}

TEST(LaneLineFilter, RestartsOnAJumpAndAGap) {
  LaneLineFilter filter;
  float x[kTrajectorySize], y[kTrajectorySize], out[kTrajectorySize];
  auto feed = [&](double t, double offset) {
    for (int i = 0; i < kTrajectorySize; ++i) {
      x[i] = static_cast<float>(x_idx(i));
      y[i] = static_cast<float>(offset);
    }
    filter.update(t, x, y, 20.0, 0.0, out);
  };
  for (int k = 0; k < 10; ++k) feed(k * kDt, -1.8);
  feed(10 * kDt, 1.7);  // 차로 변경으로 오른쪽 선이 왼쪽 선이 됐다
  EXPECT_FLOAT_EQ(out[5], 1.7f) << "선이 크게 바뀌면 측정에서 다시 시작한다";
  feed(11 * kDt, 1.6);
  EXPECT_NEAR(out[5], 1.7 - kLaneLineFilterWeight * 0.1, 1e-5) << "작은 변화는 섞는다";
  feed(11 * kDt + 0.5, 1.4);
  EXPECT_FLOAT_EQ(out[5], 1.4f) << "프레임이 끊기면 다시 시작한다";
}

/* tests/check_bev_lane_filter.mjs가 같은 장면으로 BEV의 JS 사본(lane_filter.js)을 이 값에 맞춘다. 알고리즘을 바꾸면
 * 두 파일의 값을 함께 고친다(check_web_console.py가 같은지 본다). 25 m/s로 40프레임: 차가 방향을 틀며 휘는 선,
 * 정해진 잡음, 25번째 프레임의 차로 변경(선이 3.5 m 뛴다), 32번째 프레임 앞의 0.35초 끊김. */
TEST(LaneLineFilter, GoldenSharedWithTheBev) {
  // GOLDEN BEGIN: 프레임 -> 0, 8, 12, 16, 20, 24번 점의 거른 선(C++ 출력)
  const struct { int frame; double y[6]; } golden[] = {
    {10, {-1.7860593, -1.6711308, -1.4910269, -1.1393055, -0.5273346, 0.4499038}},
    {24, {-1.7755761, -1.4648968, -1.0241125, -0.2886234, 0.8415040, 2.5161421}},
    {25, {1.7029999, 2.0276000, 2.4753499, 3.2546000, 4.4127498, 6.1616001}},
    {31, {1.7403402, 2.1345892, 2.6871150, 3.6010313, 4.9587660, 6.9622140}},
    {32, {1.7120000, 2.1226161, 2.6835561, 3.5988560, 5.0254998, 7.0568962}},
    {39, {1.7414960, 2.2348645, 2.9326613, 4.0234733, 5.6730695, 8.0137234}},
  };
  // GOLDEN END
  const int points[] = {0, 8, 12, 16, 20, 24};
  LaneLineFilter filter;
  float x[kTrajectorySize], y[kTrajectorySize], out[kTrajectorySize];
  for (int i = 0; i < kTrajectorySize; ++i) x[i] = static_cast<float>(x_idx(i));
  size_t next = 0;
  for (int k = 0; k < 40; ++k) {
    const double t = 100.0 + 0.05 * k + (k >= 32 ? 0.3 : 0.0);
    const double curvature = k < 20 ? 0.001 : -0.0005;
    for (int i = 0; i < kTrajectorySize; ++i) {
      const double xi = x[i];
      double v = -1.8;
      v += 0.001 * k * xi;
      v += (1e-4 + 2e-6 * k) * xi * xi;
      v += ((i * 7 + k * 13) % 11 - 5) * 0.003;
      if (k >= 25) v += 3.5;
      y[i] = static_cast<float>(v);
    }
    filter.update(t, x, y, 25.0, curvature, out);
    if (next < std::size(golden) && golden[next].frame == k) {
      for (int p = 0; p < 6; ++p) EXPECT_NEAR(out[points[p]], golden[next].y[p], 1e-6) << "frame " << k;
      ++next;
    }
  }
  EXPECT_EQ(next, std::size(golden));
}

}  // namespace
