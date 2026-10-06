#ifndef CALIBRATION_SERVICE_H
#define CALIBRATION_SERVICE_H

#include "common/app_config.h"
#include "common/model_output.h"
#include "model/calibration_online.h"
#include "common/projection.h"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

class CalibrationService {
public:
    explicit CalibrationService(const AppConfig &config);
    ~CalibrationService();
    CalibrationService(const CalibrationService &) = delete;
    CalibrationService &operator=(const CalibrationService &) = delete;

    void update(const ParsedModelOutput &output, float v_ego);
    ProjectionState projection() const { return projection_; }
    OnlineCalibrator::Snapshot snapshot() const { return last_snapshot_; }
    void input_rpy(float rpy[3]) const;
    bool can_apply_online() const { return auto_enabled_ && !manual_override_; }

private:
    const char *mode_name(const OnlineCalibrator::Snapshot &snapshot) const;
    void maybe_log(const OnlineCalibrator::UpdateResult &result);
    void maybe_persist(const OnlineCalibrator::UpdateResult &result);
    void set_fixed_projection();
    void persist_loop();
    void poll_reset_request();
    void reset_online();

    bool auto_enabled_ = true;
    bool manual_override_ = false;
    bool log_enabled_ = false;
    bool restored_ = false;
    bool has_persisted_ = false;
    float fixed_rpy_[3] = {};
    float online_rpy_[3] = {};
    float persisted_rpy_[3] = {};
    std::string params_dir_;
    std::string calibration_path_;
    std::string reset_request_path_;
    std::chrono::steady_clock::time_point last_reset_poll_{};

    OnlineCalibrator calibrator_;
    OnlineCalibrator::Snapshot last_snapshot_{};
    ProjectionState projection_;

    std::chrono::steady_clock::time_point last_log_{};
    std::chrono::steady_clock::time_point last_persist_{};
    CalibrationStatus last_status_ = CalibrationStatus::Uncalibrated;
    int last_valid_blocks_ = -1;

    /* 주행 중 저장은 SD가 녹화로 바쁠 때 open/rename이 몇 초씩 막혀 모델 루프를 세웠다.
     * 그래서 저장 스레드에 최신 값만 넘긴다. */
    struct PersistJob {
        float rpy[3] = {};
        OnlineCalibrator::Snapshot snapshot{};
        bool remove = false;  // 초기화: 저장 파일을 지운다
    };
    std::mutex persist_mutex_;
    std::condition_variable persist_cv_;
    bool persist_pending_ = false;
    bool persist_stop_ = false;
    PersistJob persist_job_;
    std::thread persist_thread_;
};

#endif
