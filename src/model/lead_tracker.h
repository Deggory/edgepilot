#pragma once

// 앞차(CIPV) 추적. 모델 앞차 거리와 광류 확대율(model/lead_flow)을 칼만 필터 하나로 섞어 거리와 상대
// 속도를 낸다(ModelState.lead_track). 모델 거리는 프레임마다 0.6~1.6 m 흔들리고(앞뒤 2초 추세 둘레 3~7%),
// 모델 상대 속도는 25 m 밖에서 실제의 절반쯤만 읽는다. 확대율은 크기가 맞지만 0.5초 창이 겹쳐 프레임마다
// 시끄럽다. 둘을 섞으면(2026-10-11 녹화 11구간, 8~40 m) 거리 흔들림 0.91 -> 0.26 m/프레임, 상대 속도
// 오차 3.8 -> 2.3 km/h, 흔들림 2.2 -> 0.7 km/h/프레임이고, 지연은 0.25 -> 0.6초다
// (docs/lead-flow.md). 비전 크루즈가 이 거리와 상대 속도를 쓴다(controls_tick observe_cruise_lead).
//
// 상태는 [거리, 상대 속도](등속), 측정은 모델 거리(sigma 0.5 + 0.025·d m)와 확대율 r = -v/d(EKF).
// 두 측정 모두 3 sigma 문으로 튀는 값을 버린다. 모델 거리가 5프레임 내리 문 밖이면 다른 앞차로 보고
// 다시 시작한다. 하드웨어와 무관하다.

#include "common/ipc_messages.h"  // LeadTrackState

#include <cstdint>

constexpr int kLeadTrackRestartMisses = 5;  // 모델 거리가 이만큼 내리 문 밖이면 다른 앞차

// 모델 프레임 하나의 앞차 측정
struct LeadTrackInput {
    double t_s = 0.0;                 // 촬영 시각(s)
    bool lead_valid = false;          // 모델 앞차가 있다(확률 문턱 이상, 유한한 값)
    float distance_m = 0.0f;          // 모델 앞차 거리(카메라 기준)
    float relative_speed_mps = 0.0f;  // 모델 앞차 속도 - 자차 속도. 모르면 NaN(시작 때만 쓴다)
    bool flow_valid = false;          // 확대율이 있다(ModelState.lead_flow.valid)
    uint64_t flow_frame_id = 0;       // 그 확대율의 창 끝 프레임. 이미 쓴 프레임이면 다시 넣지 않는다
    float flow_rate = 0.0f;           // 1/s, +면 가까워진다
};

class LeadTracker {
public:
    void update(const LeadTrackInput &in);
    const LeadTrackState &state() const { return state_; }

private:
    void reset();
    void start(const LeadTrackInput &in);

    // 추적 중이면 state_.valid가 1이다
    double t_s_ = 0.0;
    double x_[2] = {};                          // 거리(m, 카메라 기준), 상대 속도(m/s)
    double p00_ = 0.0, p01_ = 0.0, p11_ = 0.0;  // 공분산(대칭)
    int misses_ = 0;
    uint64_t last_flow_frame_ = UINT64_MAX;  // 마지막으로 넣은 확대율의 창 끝 프레임
    LeadTrackState state_{};
};
