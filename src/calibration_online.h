#ifndef CALIBRATION_ONLINE_H
#define CALIBRATION_ONLINE_H

#include "model_output.h"  // PoseObservation(모델 pose 출력)

#include <cstdint>

enum class CalibrationStatus {
    Uncalibrated = 0,
    Calibrated = 1,
    Invalid = 2,
    Recalibrating = 3,  // 장착이 바뀐 것으로 보고 마지막 블록부터 다시 모으는 중
};

const char *calibration_status_name(CalibrationStatus status);

class OnlineCalibrator {
public:
    struct Snapshot {
        float rpy[3] = {0.0f, 0.0f, 0.0f};
        float spread[3] = {0.0f, 0.0f, 0.0f};
        int valid_blocks = 0;
        int block_sample_count = 0;
        CalibrationStatus status = CalibrationStatus::Uncalibrated;
        uint64_t accepted_samples = 0;
        uint64_t rejected_samples = 0;
    };

    struct UpdateResult {
        Snapshot snapshot;
        bool accepted = false;
        bool block_completed = false;
    };

    OnlineCalibrator();

    UpdateResult update(const PoseObservation &pose, float v_ego);
    /* 저장된 보정으로 시작한다(openpilot reset). 블록 수를 그대로 써서 5 미만이면 미보정이다.
     * rpy가 유효하면 true. */
    bool restore(const float rpy[3], int valid_blocks, const float spread[3] = nullptr);
    Snapshot snapshot() const;
    void output_rpy(float rpy[3]) const;

private:
    static constexpr int kBlockSize = 100;
    static constexpr int kInputsNeeded = 5;
    static constexpr int kInputsWanted = 50;

    void reset_to_rpy(const float rpy[3], int valid_blocks, const float *smooth_from = nullptr);
    void update_status();
    void smooth_rpy(float out[3]) const;
    bool is_calibration_valid(const float rpy[3]) const;
    void sanity_clip(float rpy[3]) const;

    float block_rpys_[kInputsWanted][3] = {};
    int sample_idx_ = 0;
    int block_idx_ = 0;
    float old_rpy_[3] = {};
    float old_rpy_weight_ = 0.0f;
    Snapshot snapshot_;
};

#endif
