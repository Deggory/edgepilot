#pragma once

// 모델 입력 워프의 카메라 보정. 모델 포즈 보정기(OnlineCalibrator, upstream calibrationd)를 돌리고, 웹 기기
// 설정이 광류면 광류 추정(FlowSource, 보드에서는 광류 워커)의 pitch·yaw로 바꿔 쓴다(CalibrationSourceSelector).
// projection·input_rpy·snapshot은 언제나 워프에 쓰는 값이다. 포즈 보정기는 자기 값이 워프를 맡을 때만 돈다
// (upstream 루프는 모델이 자기 값으로 워프된 영상을 봤다고 가정한다). 포즈 보정은 calibration.json, 광류 블록은
// flow_calibration.json에 저장하고(SD가 바빠도 모델 루프가 서지 않게 BackgroundWriter로), 웹 콘솔의 초기화는 둘 다
// 지운다(광류 측정원에도 알린다). 설정을 읽는 것은 프로세스(modeld)이고, 여기서는 인자로 받는다.

#include "common/app_config.h"
#include "common/background_writer.h"
#include "common/model_output.h"
#include "common/projection.h"
#include "model/calibration_online.h"
#include "model/calibration_source.h"
#include "model/flow_calibration.h"

#include <chrono>
#include <string>

class CalibrationService {
public:
    // requested: 시작할 때의 보정 출처 설정(display.json calibration_use_flow). 첫 프레임 워프부터 맞춘다.
    explicit CalibrationService(const AppConfig &config, CalibrationSource requested = CalibrationSource::Pose);
    ~CalibrationService();
    CalibrationService(const CalibrationService &) = delete;
    CalibrationService &operator=(const CalibrationService &) = delete;

    // 광류 측정원을 붙인다(nullptr이면 뗀다). 측정원은 저장해 둔 블록(stored_flow_blocks)으로 시작해야 하고,
    // 붙어 있는 동안 살아 있어야 한다. 없으면 저장해 둔 추정을 그대로 쓴다(장착의 성질이다).
    void attach_flow(FlowSource *source) { flow_source_ = source; }
    // 모델 프레임마다. engaged는 controlsd의 결합 상태(광류로 바꾸는 것과 초기화는 해제까지 기다린다), requested는
    // 보정 출처 설정.
    void update(const ParsedModelOutput &output, float v_ego, bool engaged = false,
                CalibrationSource requested = CalibrationSource::Pose);
    ProjectionState projection() const { return projection_; }
    // 워프에 쓴 보정의 상태(ModelState.calibration). 출처가 광류면 Calibrated다.
    OnlineCalibrator::Snapshot snapshot() const { return applied_snapshot_; }
    void input_rpy(float rpy[3]) const;
    // 출처와 포즈 보정기 자신의 추정(ModelState.calibration_source)
    CalibrationSourceState source_state() const;

    // 저장해 둔 광류 블록(광류 측정원이 이것으로 시작한다)
    const FlowBlocks &stored_flow_blocks() const { return stored_flow_; }
    // 지금 쓰는 광류 상태(측정원 것, 없으면 저장해 둔 추정). ModelState.flow_calibration이 된다.
    FlowCalibrationState flow_state() const { return flow_state_; }

private:
    void maybe_persist(const OnlineCalibrator::Output &result);
    void poll_flow_source(std::chrono::steady_clock::time_point now);
    void maybe_persist_flow(std::chrono::steady_clock::time_point now, bool flush);
    void set_fixed_projection();
    void apply_selection(float dt_s);
    void poll_reset_request();
    void reset_online();

    bool auto_enabled_ = true;  // 끄면(EDGEPILOT_CALIB_AUTO=0) 저장된 보정을 고정해 쓴다(fixed_rpy_)
    bool has_persisted_ = false;
    float fixed_rpy_[3] = {};
    float online_rpy_[3] = {};   // 포즈 보정기 출력
    float applied_rpy_[3] = {};  // 워프에 쓰는 값
    float persisted_rpy_[3] = {};
    std::string calibration_path_;
    std::string flow_path_;
    std::string reset_request_path_;
    std::chrono::steady_clock::time_point last_reset_poll_{};
    std::chrono::steady_clock::time_point last_update_{};

    OnlineCalibrator calibrator_;
    OnlineCalibrator::Snapshot last_snapshot_{};     // 포즈 보정기
    OnlineCalibrator::Snapshot applied_snapshot_{};  // 워프에 쓴 값
    ProjectionState projection_;

    // 광류 출처
    FlowSource *flow_source_ = nullptr;
    CalibrationSourceSelector selector_;
    CalibrationSource requested_ = CalibrationSource::Pose;  // 마지막으로 받은 설정
    bool engaged_ = false;
    bool logged_flow_applied_ = false;
    bool logged_pending_ = false;
    FlowCalibrationState flow_state_{};
    FlowBlocks stored_flow_;
    // 광류 초기화: 요청한 수와 측정원이 처리했다고 알린 수. 앞의 것이 크면 측정원의 상태와 블록은 초기화 전
    // 것이다. 요청은 측정원이 붙어 있을 때 보낸다(없으면 붙을 때).
    uint32_t flow_resets_requested_ = 0;
    uint32_t flow_resets_sent_ = 0;
    uint32_t flow_resets_seen_ = 0;
    bool logged_reset_deferred_ = false;
    FlowBlocks latest_flow_blocks_;
    bool flow_blocks_dirty_ = false;
    bool has_flow_persisted_ = false;
    uint32_t persisted_mount_changes_ = 0;
    std::chrono::steady_clock::time_point last_flow_persist_{};

    std::chrono::steady_clock::time_point last_persist_{};

    // 주행 중 저장은 SD가 녹화로 바쁠 때 open/rename이 몇 초씩 막혀 모델 루프를 세웠다. 그래서 쓰기 스레드에
    // 넘긴다(경로마다 최신 것만, 없앨 때 남은 일을 마친다).
    BackgroundWriter writer_{"calibration: write"};
};
