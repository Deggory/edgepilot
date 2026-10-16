#ifndef PLANNING_LANE_STATE_SELECTOR_H
#define PLANNING_LANE_STATE_SELECTOR_H

// 플래너 프레임마다 쓸 차선선 플러그인 결과(LaneState)를 고른다(docs/lanebow.md). controlsd와 재생 도구가 같이 쓴다.
// modeld는 같은 프레임의 LaneState를 ModelState를 낸 뒤에 내므로 플래너는 기다리지 않고 보통 직전 프레임의 것을
// 쓴다(LanebowSource가 자차 움직임만큼 옮긴다). 그래서 마지막으로 받은 유효한 것 하나면 된다: 부르는 쪽은 모두 같은
// 프레임의 LaneState를 그 ModelState보다 먼저 넘기지 않는다(modeld 발행 순서, controlsd는 틱마다 LD를 먼저 읽고,
// 재생은 기록 순서). 모델 프레임보다 새 것과 kMaxAgeNs보다 낡은 것은 쓰지 않는다.

#include "common/ipc_messages.h"

#include <cstdint>

class LaneStateSelector {
public:
    static constexpr uint64_t kMaxAgeNs = 120'000'000ULL;  // LD 두 프레임 남짓

    // 새로 읽은 LaneState. 해석하지 못한 프레임(valid = 0)은 받지 않는다(직전 것을 그대로 둔다).
    void offer(const LaneState &lane) {
        if (lane.valid) latest_ = lane;
    }

    // model_capture_ns(모델 프레임 촬영 시각)에 쓸 것: 0 <= 모델 - LD <= kMaxAgeNs이면 마지막 것, 아니면 nullptr.
    const LaneState *pick(uint64_t model_capture_ns) const {
        if (!latest_.valid || latest_.capture_timestamp_ns > model_capture_ns ||
            model_capture_ns - latest_.capture_timestamp_ns > kMaxAgeNs)
            return nullptr;
        return &latest_;
    }

private:
    LaneState latest_;
};

#endif
