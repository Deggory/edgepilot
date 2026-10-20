#pragma once

// controlsd의 입력 안전 홀드 둘. 문서(source-layout.md "Control safety holds")가
// 말하는 바로 그 동작이며, 100 Hz 루프에서 분리해 호스트 검사가 경계값을 본다.
// - Panda 헬스 스냅샷(pandad가 0.5초마다 낸다)은 1.1초 동안 신선하고, 낡았거나 준비되지 않은
//   스냅샷은 100 ms 더 마지막 판정을 유지한다. 그래서 pandad가 멈추면 마지막 판정이 최대 약
//   1.2초 간다. 신선하고 전송 준비된 controls_allowed=0은 절대 유지하지 않는다.
// - 잘못된 plan 프레임 하나는 150 ms까지 마지막 유효 경로로 덮는다. 모델
//   freshness 타임아웃(kModelTimeoutNs, 250 ms)은 그대로 하드 게이트다.

#include <cstdint>

#include "common/ipc_messages.h"
#include "controls/lateral_target.h"
#include "panda/panda_protocol.h"

// pandad가 거는 safety 모드(매니저의 EDGEPILOT_PANDA_SAFETY 기본값 hyundaiCommunity, _PARAM 0). 다른 모드면 결합하지
// 않는다. 매니저 기본값과의 대조는 check_web_console.py가 한다.
constexpr uint32_t kExpectedPandaSafetyModel = kPandaSafetyHyundaiCommunity;
constexpr uint32_t kExpectedPandaSafetyParam = 0;
constexpr uint64_t kPandaStateTimeoutNs = 1100000000ULL;
constexpr uint64_t kPandaHealthHoldNs = 100000000ULL;
constexpr uint64_t kPathInvalidHoldNs = 150000000ULL;

struct PandaGateOutput {
    bool state_fresh = false;
    bool ready_raw = false;
    bool hold_applied = false;
    bool ready = false;
    bool controls_allowed = false;
};

class PandaHealthGate {
public:
    PandaGateOutput update(const PandaState &state, uint64_t now_ns, bool force_engaged);

private:
    bool have_last_ready_ = false;
    bool last_controls_allowed_ = false;
    uint64_t last_ready_ns_ = 0;
};

// 모델 plan의 점 개수와 도달 거리로 조향 가용성을 정한다. 모델이 timeout_ns보다 낡았으면 model_lost.
LateralPath path_from_model_state(const ModelState &state, uint64_t now_ns, uint64_t timeout_ns);

struct PathHoldOutput {
    LateralPath raw;
    LateralPath path;
    bool hold_applied = false;
};

class PathHoldGate {
public:
    PathHoldOutput update(const ModelState &model, uint64_t now_ns, uint64_t model_timeout_ns);

private:
    LateralPath last_usable_;
    uint64_t last_usable_model_timestamp_ns_ = 0;
};
