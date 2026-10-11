#ifndef FLOW_CALIBRATION_WORKER_H
#define FLOW_CALIBRATION_WORKER_H

/* 광류 보정기를 보드에서 돌리는 부분(modeld 전용). FlowWindowTracker가 프레임마다 측정 창을 이어 가고
 * (IVE로 복사·피라미드·코너·LK, CPU로 코너 고르기·풀이·집계), FlowCalibrationWorker가 그것을 modeld의
 * 모델 루프 밖 스레드에서 돌린다. 모델 루프는 프레임 정보만 넘기고 기다리지 않는다. 차가 곧고 빠르게 갈
 * 때만 IVE를 쓴다(그 밖에는 할 일이 없다). docs/flow-calibration.md. */

#include "model/flow_calibration.h"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class IveFlow;

class FlowWindowTracker {
public:
    // 실패하면 std::runtime_error(IVE를 쓸 수 없다)
    FlowWindowTracker(const FlowIntrinsics &k, int width, int height);
    ~FlowWindowTracker();

    /* 카메라 프레임 하나. y_phys는 NV12/NV21 프레임의 Y 평면(CMM, 행 간격 stride, UV는 바로 뒤). 원본 단은
     * 복사하지 않으므로 다음 프레임의 LK가 끝날 때까지 그대로여야 한다. still_valid가 있으면 축소 단을 만든
     * 뒤와 LK 뒤에 불러(이전 프레임 것도), 그동안 링 슬롯이 덮어써졌으면 그 걸음을 버린다. */
    void process(unsigned long long y_phys, int stride, uint64_t frame_id, const FlowGateInput &gate,
                 const std::function<bool()> &still_valid);
    const FlowCalibrator &calibrator() const { return calibrator_; }
    // 저장해 둔 블록으로 시작한다(FlowCalibrator::restore). 진행 중인 창은 그대로다.
    bool restore(const FlowBlocks &blocks) { return calibrator_.restore(blocks); }
    // 보정 초기화: 블록과 진행 중인 창을 버린다.
    void reset_calibration();
    const char *pyramid_engine() const;

    struct Stats {
        uint64_t frames = 0;         // IVE로 처리한 프레임
        uint64_t windows_done = 0;   // 끝까지 간 창
        uint64_t windows_lost = 0;   // 조건이 닫히거나 점·프레임을 잃어 버린 창
        uint64_t rejected = 0;       // 끝까지 갔지만 풀이가 맞지 않은 창
        double wall_ms = 0.0;        // 처리한 프레임의 벽시계 합
        double cpu_ms = 0.0;         // 같은 프레임의 스레드 CPU 합
    };
    const Stats &stats() const { return stats_; }

private:
    void start_window(uint64_t frame_id);

    FlowIntrinsics k_;
    std::unique_ptr<IveFlow> ive_;
    FlowCalibrator calibrator_;
    Stats stats_;
    bool in_window_ = false;
    bool have_last_ = false;  // 직전 프레임을 넣었다(LK의 이전 프레임)
    std::function<bool()> last_valid_;  // 직전 프레임의 링 슬롯이 그대로인지
    uint64_t last_frame_ = 0;
    int steps_ = 0;
    int n_ = 0;
    std::vector<float> start_, cur_;   // 창 시작 점, 지금 위치(원본 좌표)
    std::vector<uint8_t> alive_;
    std::vector<int> live_;            // 이번 걸음에 따라갈 점의 번호
    std::vector<float> step_in_, step_out_;
    std::vector<uint8_t> step_ok_;
    std::vector<float> fit_a_, fit_b_;
};

class FlowCalibrationWorker : public FlowSource {
public:
    struct Frame {
        unsigned long long y_phys = 0;
        int stride = 0;
        uint64_t frame_id = 0;
        FlowGateInput gate;
        std::function<bool()> still_valid;
    };

    /* 실패하면 std::runtime_error. restore가 있으면 그 블록으로 시작한다(스레드를 띄우기 전에). 잘못된
     * 내용이면 처음부터 모은다. */
    FlowCalibrationWorker(const FlowIntrinsics &k, int width, int height, const FlowBlocks *restore = nullptr);
    ~FlowCalibrationWorker() override;
    FlowCalibrationWorker(const FlowCalibrationWorker &) = delete;
    FlowCalibrationWorker &operator=(const FlowCalibrationWorker &) = delete;

    /* 모델 루프에서 프레임마다. 밀리면(대기 3개) 가장 오래된 것을 버린다. 그러면 프레임이 이어지지 않으니
     * 진행 중인 창도 버려진다. */
    void push(Frame frame);
    // FlowSource. 보정 서비스가 모델 루프에서 부른다(초기화는 다음 프레임 전에 워커 스레드가 처리한다).
    FlowCalibrationState state(uint32_t *resets_applied) const override;
    bool take_blocks(FlowBlocks *out) override;
    void request_reset() override;

private:
    void run();

    FlowWindowTracker tracker_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Frame> queue_;
    bool stop_ = false;
    bool reset_requested_ = false;
    uint32_t resets_applied_ = 0;
    uint64_t dropped_ = 0;
    FlowCalibrationState state_{};
    FlowBlocks blocks_;          // 마지막으로 끝난 블록까지(take_blocks)
    bool blocks_changed_ = false;
    std::thread thread_;
};

#endif
