#ifndef FLOW_WORKER_H
#define FLOW_WORKER_H

// 광류 작업 스레드(modeld 전용). FlowTracker가 프레임마다 IVE로 축소 단을 한 번 만들고, 광류 보정의 측정 창
// (model/flow_calibration)과 앞차 확대율의 창(model/lead_flow)의 점을 한 번의 LK로 함께 따라간다. 앞차 점은
// 앞뒤 검사를 위해 거꾸로 한 번 더 따라간다. 보정은 차가 곧고 빠를 때, 앞차는 모델 앞차가 있을 때만 일하고,
// 둘 다 없으면 IVE를 쓰지 않는다. FlowWorker가 그것을 모델 루프 밖 스레드에서 돌린다. 모델 루프는 프레임
// 정보만 넘기고 기다리지 않는다. docs/flow-calibration.md.

#include "model/flow_calibration.h"
#include "model/lead_flow.h"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class IveFlow;

// 카메라 프레임 하나와 그때의 조건
struct FlowFrame {
    // NV12/NV21 프레임의 Y 평면(CMM, 행 간격 stride, UV는 바로 뒤). 원본 단은 복사하지 않으므로 다음 프레임의
    // LK가 끝날 때까지 그대로여야 한다. still_valid가 있으면 축소 단을 만든 뒤와 LK 뒤에 불러(이전 프레임
    // 것도), 그동안 링 슬롯이 덮어써졌으면 그 걸음을 버린다.
    unsigned long long y_phys = 0;
    const uint8_t *y_virt = nullptr;  // 같은 평면의 CPU 매핑(앞차 점·템플릿). 없으면 앞차를 따라가지 않는다
    int stride = 0;
    uint64_t frame_id = 0;
    double t_s = 0.0;    // 촬영 시각(s)
    FlowGateInput gate;  // 광류 보정의 곧은 길 조건
    LeadFlowInput lead;  // 직전 모델 출력의 앞차와 워프에 쓴 보정
    std::function<bool()> still_valid;
};

class FlowTracker {
public:
    // 실패하면 std::runtime_error(IVE를 쓸 수 없다)
    FlowTracker(const FlowIntrinsics &k, int width, int height);
    ~FlowTracker();

    void process(const FlowFrame &f);
    const FlowCalibrator &calibrator() const { return calibrator_; }
    LeadFlowState lead_state() const { return lead_.state(); }
    // 저장해 둔 블록으로 시작한다(FlowCalibrator::restore). 진행 중인 창은 그대로다.
    bool restore(const FlowBlocks &blocks) { return calibrator_.restore(blocks); }
    // 보정 초기화: 블록과 진행 중인 창을 버린다(앞차 창은 그대로다).
    void reset_calibration();
    const char *pyramid_engine() const;

    struct Stats {
        uint64_t frames = 0;        // IVE로 처리한 프레임
        uint64_t windows_done = 0;  // 끝까지 간 보정 창
        uint64_t windows_lost = 0;  // 조건이 닫히거나 점·프레임을 잃어 버린 보정 창
        uint64_t rejected = 0;      // 끝까지 갔지만 풀이가 맞지 않은 보정 창
        uint64_t lead_frames = 0;   // 앞차 점을 따라간 프레임
        uint64_t lead_rates = 0;    // 그중 확대율을 낸 프레임
        double wall_ms = 0.0;       // 처리한 프레임의 벽시계 합
        double cpu_ms = 0.0;        // 같은 프레임의 스레드 CPU 합
    };
    const Stats &stats() const { return stats_; }

private:
    void start_window();
    void advance_window(uint64_t frame_id, int tracked);

    FlowIntrinsics k_;
    std::unique_ptr<IveFlow> ive_;
    FlowCalibrator calibrator_;
    LeadFlowTracker lead_;
    Stats stats_;
    bool in_window_ = false;
    bool have_last_ = false;            // 직전 프레임을 넣었다(LK의 이전 프레임)
    std::function<bool()> last_valid_;  // 직전 프레임의 링 슬롯이 그대로인지
    uint64_t last_frame_ = 0;
    int steps_ = 0;
    int n_ = 0;
    std::vector<float> start_, cur_;  // 보정 창 시작 점, 지금 위치(원본 좌표)
    std::vector<uint8_t> alive_;
    std::vector<int> live_;                  // 이번 걸음에 따라간 보정 점의 번호
    std::vector<float> step_in_, step_out_;  // 보정 점 다음에 앞차 점
    std::vector<uint8_t> step_ok_;
    std::vector<float> back_;  // 앞차 점을 거꾸로 따라간 위치(직전 프레임)
    std::vector<uint8_t> back_ok_;
    std::vector<float> fit_a_, fit_b_;
};

class FlowWorker : public FlowSource {
public:
    // 실패하면 std::runtime_error. restore가 있으면 그 블록으로 시작한다(스레드를 띄우기 전에). 잘못된
    // 내용이면 처음부터 모은다.
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
