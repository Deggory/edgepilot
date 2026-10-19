#include "model/calibration_source.h"

#include "model/calibration_online.h"
#include "model/flow_calibration.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kReachedRad = 1e-6f;

}  // namespace

// status 2는 kFlowMinBlocks 이상의 블록 중앙값 평균이다(유한한 값만 블록이 된다).
bool CalibrationSourceSelector::flow_usable(const FlowCalibrationState &flow, float max_spread) {
    const float rpy[3] = {0.0f, flow.pitch, flow.yaw};
    return flow.status == 2 && flow.sd_pitch <= max_spread && flow.sd_yaw <= max_spread &&
           OnlineCalibrator::rpy_within_limits(rpy);
}

void CalibrationSourceSelector::update(const float pose_rpy[3], const FlowCalibrationState &flow, bool requested_flow,
                                       bool engaged, float dt_s) {
    if (!initialized_ || (requested_flow != active_ && !engaged)) active_ = requested_flow;
    // 쓰는 중이면 느슨한 문턱으로 계속 쓰고, 아니면 엄한 문턱을 넘고 해제 상태(또는 시작)일 때만 쓰기 시작한다
    const bool usable = flow_applied_ ? flow_usable(flow, kMaxFlowSpreadStayRad) : flow_usable(flow);
    const bool want_flow = active_ && usable && (flow_applied_ || !initialized_ || !engaged);
    float target[3] = {pose_rpy[0], pose_rpy[1], pose_rpy[2]};
    if (want_flow) {
        target[1] = flow.pitch;
        target[2] = flow.yaw;
    }

    if (!initialized_) {
        initialized_ = true;
        transition_ = false;
        std::copy(target, target + 3, rpy_);
    } else {
        if (flow_applied_ && !want_flow) transition_ = true;
        if (!want_flow && !transition_) {
            std::copy(target, target + 3, rpy_);  // 포즈만 쓸 때는 지금 동작 그대로
        } else {
            const float dt = std::clamp(dt_s, 0.0f, kMaxDtS);
            bool reached = true;
            for (int i = 0; i < 3; ++i) {
                const float d = target[i] - rpy_[i];
                const float step = std::max(kRampRadPerS, std::fabs(d) / kRampTimeS) * dt;
                rpy_[i] += std::clamp(d, -step, step);
                reached = reached && std::fabs(target[i] - rpy_[i]) <= kReachedRad;
            }
            if (!want_flow && reached) transition_ = false;
        }
    }
    flow_applied_ = want_flow;

    bool ramping = false;
    for (int i = 0; i < 3; ++i) ramping = ramping || std::fabs(target[i] - rpy_[i]) > kReachedRad;
    const bool waiting = requested_flow != active_ || (active_ && usable && !want_flow);  // 해제를 기다린다
    flags_ = (usable ? kCalibrationSourceFlowUsable : 0u) | (waiting ? kCalibrationSourcePending : 0u) |
             (ramping ? kCalibrationSourceRamping : 0u);
}

void CalibrationSourceSelector::snap_to(const float rpy[3]) {
    std::copy(rpy, rpy + 3, rpy_);
    transition_ = false;
    flow_applied_ = false;
}

void CalibrationSourceSelector::rpy(float out[3]) const { std::copy(rpy_, rpy_ + 3, out); }
