#include "model/lead_tracker.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kProcessNoise = 4.0;      // 상대 가속도 잡음 세기((m/s^2)^2·s)
constexpr double kDistanceStd0 = 0.5;      // 모델 거리 잡음 = 0.5 + 0.025·d (m)
constexpr double kDistanceStdPerM = 0.025;
constexpr double kFlowRateStd = 0.09;      // 확대율 잡음 0.03/s를 창 겹침만큼 3배로
constexpr double kGate = 9.0;              // 혁신의 제곱 / 분산(3 sigma)
constexpr double kInitSpeedStd = 3.0;      // 시작할 때 상대 속도의 불확실성(m/s)
constexpr double kMaxGapS = 0.5;           // 프레임 사이가 이보다 길면 다시 시작한다
constexpr double kFlowMinDistanceM = 5.0;  // 추정 거리가 이보다 가까우면 확대율을 쓰지 않는다

double distance_variance(double d)
{
    const double s = kDistanceStd0 + kDistanceStdPerM * std::max(0.0, d);
    return s * s;
}

}  // namespace

void LeadTracker::reset()
{
    state_ = LeadTrackState{};
}

// 측정한 모델 거리와 상대 속도(모르면 0)에서 새로 시작한다
void LeadTracker::start(const LeadTrackInput &in)
{
    misses_ = 0;
    x_[0] = in.distance_m;
    x_[1] = std::isfinite(in.relative_speed_mps) ? in.relative_speed_mps : 0.0;
    p00_ = distance_variance(x_[0]);
    p01_ = 0.0;
    p11_ = kInitSpeedStd * kInitSpeedStd;
    state_.flags |= kLeadTrackStarted;
}

void LeadTracker::update(const LeadTrackInput &in)
{
    if (!in.lead_valid || !std::isfinite(in.distance_m)) {
        reset();
        return;
    }
    state_.flags = 0;
    const double dt = in.t_s - t_s_;
    if (!state_.valid || !(dt > 0.0) || dt > kMaxGapS) {
        start(in);
    } else {
        // 등속 예측: P = F P F' + Q, Q = q [dt^3/3 dt^2/2; dt^2/2 dt]
        const double q = kProcessNoise;
        x_[0] += dt * x_[1];
        p00_ = p00_ + dt * (p01_ + p01_) + dt * dt * p11_ + q * dt * dt * dt / 3.0;
        p01_ = p01_ + dt * p11_ + q * dt * dt / 2.0;
        p11_ += q * dt;
        // 모델 거리(H = [1 0]). 문 밖이면 버리고, 내리 버리면 다른 앞차로 보고 다시 시작한다.
        const double y = in.distance_m - x_[0];
        const double s = p00_ + distance_variance(in.distance_m);
        if (y * y < kGate * s) {
            const double k0 = p00_ / s, k1 = p01_ / s;
            x_[0] += k0 * y;
            x_[1] += k1 * y;
            p11_ -= k1 * p01_;
            p01_ *= 1.0 - k0;
            p00_ *= 1.0 - k0;
            misses_ = 0;
        } else {
            state_.flags |= kLeadTrackDistanceRejected;
            if (++misses_ >= kLeadTrackRestartMisses) start(in);
        }
    }
    t_s_ = in.t_s;
    // 확대율 r = -v/d(EKF, H = [v/d^2, -1/d]). 광류 작업이 모델보다 늦으면 같은 확대율이 두 번 오니 한 번만 쓴다.
    const bool fresh_flow = in.flow_valid && in.flow_frame_id != last_flow_frame_;
    if (fresh_flow) last_flow_frame_ = in.flow_frame_id;
    if (fresh_flow && std::isfinite(in.flow_rate) && x_[0] > kFlowMinDistanceM) {
        const double d = x_[0], v = x_[1];
        const double h0 = v / (d * d), h1 = -1.0 / d;
        const double ph0 = p00_ * h0 + p01_ * h1, ph1 = p01_ * h0 + p11_ * h1;
        const double s = h0 * ph0 + h1 * ph1 + kFlowRateStd * kFlowRateStd;
        const double y = in.flow_rate - (-v / d);
        if (y * y < kGate * s) {
            const double k0 = ph0 / s, k1 = ph1 / s;
            x_[0] += k0 * y;
            x_[1] += k1 * y;
            p00_ -= k0 * ph0;
            p01_ -= k0 * ph1;
            p11_ -= k1 * ph1;
            state_.flags |= kLeadTrackFlowUsed;
        } else {
            state_.flags |= kLeadTrackFlowRejected;
        }
    }
    state_.valid = 1;
    state_.distance_m = static_cast<float>(x_[0]);
    state_.relative_speed_mps = static_cast<float>(x_[1]);
    state_.distance_std_m = static_cast<float>(std::sqrt(std::max(0.0, p00_)));
    state_.relative_speed_std_mps = static_cast<float>(std::sqrt(std::max(0.0, p11_)));
}
