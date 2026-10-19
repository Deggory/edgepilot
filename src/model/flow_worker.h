#ifndef FLOW_WORKER_H
#define FLOW_WORKER_H

// 광류 작업 스레드(modeld 전용). FlowWorker가 FlowTracker(model/flow_tracker)를 IVE 엔진으로 모델 루프 밖
// 스레드에서 돌린다. 모델 루프는 프레임 정보만 넘기고 기다리지 않는다. docs/flow-calibration.md.

#include "model/flow_calibration.h"
#include "model/flow_tracker.h"
#include "model/lead_flow.h"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

class FlowWorker : public FlowSource {
public:
    // 실패하면 std::runtime_error. restore가 있으면 그 블록(CalibrationService가 이미 거른 것)으로 시작한다(스레드를
    // 띄우기 전에).
    FlowWorker(const FlowIntrinsics &k, int width, int height, const FlowBlocks *restore = nullptr);
    ~FlowWorker() override;
    FlowWorker(const FlowWorker &) = delete;
    FlowWorker &operator=(const FlowWorker &) = delete;

    // 모델 루프에서 프레임마다. 밀리면(대기 3개) 가장 오래된 것을 버린다. 그러면 프레임이 이어지지 않으니
    // 진행 중인 창도 버려진다.
    void push(FlowFrame frame);
    // FlowSource. 보정 서비스가 모델 루프에서 부른다(초기화는 다음 프레임 전에 워커 스레드가 처리한다).
    FlowCalibrationState state(uint32_t *resets_applied) const override;
    bool take_blocks(FlowBlocks *out) override;
    void request_reset() override;
    // 마지막으로 처리한 프레임의 앞차 확대율(ModelState.lead_flow)
    LeadFlowState lead_state() const;

private:
    void run();

    FlowTracker tracker_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<FlowFrame> queue_;
    bool stop_ = false;
    bool reset_requested_ = false;
    uint32_t resets_applied_ = 0;
    uint64_t dropped_ = 0;
    FlowCalibrationState state_{};
    LeadFlowState lead_state_{};
    FlowBlocks blocks_;  // 마지막으로 끝난 블록까지(take_blocks)
    bool blocks_changed_ = false;
    std::thread thread_;
};

#endif
