#include "calibration_online.h"
#include "utils_math.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kMinSpeedFilter = 15.0f * 0.44704f;
constexpr float kMaxVelAngleStd = 0.25f * kPi / 180.0f;
constexpr float kMaxYawRateFilter = 2.0f * kPi / 180.0f;
// openpilot calibrationd.py: 블록 사이 편차가 이보다 크면 장착이 바뀐 것으로 본다.
constexpr float kMaxAllowedYawSpread = 2.0f * kPi / 180.0f;
constexpr float kMaxAllowedPitchSpread = 4.0f * kPi / 180.0f;
// 그때 옛 값에서 새 값으로 넘어가는 표본 수(20 Hz에서 0.5초).
constexpr float kSmoothCycles = 10.0f;
constexpr float kPitchMin = -0.09074112085129739f;
constexpr float kPitchMax = 0.14907572052989657f;
constexpr float kYawMin = -0.06912048084718224f;
constexpr float kYawMax = 0.06912048084718235f;
constexpr float kSanityMargin = 0.005f;

bool finite3(const float v[3])
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

void rot_from_euler(const float rpy[3], float *rot)
{
    const float roll = rpy[0];
    const float pitch = rpy[1];
    const float yaw = rpy[2];
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);

    const float rx[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, cr, -sr,
        0.0f, sr, cr,
    };
    const float ry[9] = {
        cp, 0.0f, sp,
        0.0f, 1.0f, 0.0f,
        -sp, 0.0f, cp,
    };
    const float rz[9] = {
        cy, -sy, 0.0f,
        sy, cy, 0.0f,
        0.0f, 0.0f, 1.0f,
    };

    float tmp[9];
    matmul3(ry, rx, tmp);
    matmul3(rz, tmp, rot);
}

void euler_from_rot(const float *rot, float rpy[3])
{
    rpy[0] = std::atan2(rot[2 * 3 + 1], rot[2 * 3 + 2]);
    rpy[1] = std::asin(std::max(-1.0f, std::min(1.0f, -rot[2 * 3 + 0])));
    rpy[2] = std::atan2(rot[1 * 3 + 0], rot[0 * 3 + 0]);
}

void compose_rpy(const float base[3], const float observed[3], float out[3])
{
    float base_rot[9];
    float observed_rot[9];
    float composed[9];
    rot_from_euler(base, base_rot);
    rot_from_euler(observed, observed_rot);
    matmul3(base_rot, observed_rot, composed);
    euler_from_rot(composed, out);
}

} // namespace

const char *calibration_status_name(CalibrationStatus status)
{
    switch (status) {
    case CalibrationStatus::Uncalibrated:
        return "uncalibrated";
    case CalibrationStatus::Calibrated:
        return "calibrated";
    case CalibrationStatus::Invalid:
        return "invalid";
    case CalibrationStatus::Recalibrating:
        return "recalibrating";
    }
    return "unknown";
}

OnlineCalibrator::OnlineCalibrator()
{
    reset_to_rpy(snapshot_.rpy, 0);
}

OnlineCalibrator::UpdateResult OnlineCalibrator::update(const PoseObservation &pose,
                                                        float v_ego)
{
    UpdateResult result;
    result.snapshot = snapshot_;
    old_rpy_weight_ = std::max(0.0f, old_rpy_weight_ - 1.0f / kSmoothCycles);

    const bool valid_numbers = finite3(pose.trans) && finite3(pose.rot) &&
        finite3(pose.trans_std) && std::isfinite(v_ego);
    const bool straight_and_fast = valid_numbers &&
        v_ego > kMinSpeedFilter &&
        pose.trans[0] > kMinSpeedFilter &&
        std::fabs(pose.rot[2]) < kMaxYawRateFilter;
    const bool certain_if_calib = valid_numbers &&
        ((std::atan2(pose.trans_std[1], pose.trans[0]) < kMaxVelAngleStd) ||
         (snapshot_.valid_blocks < kInputsNeeded));

    if (!straight_and_fast || !certain_if_calib) {
        ++snapshot_.rejected_samples;
        result.snapshot = snapshot_;
        return result;
    }

    const float observed_rpy[3] = {
        0.0f,
        -std::atan2(pose.trans[2], pose.trans[0]),
        std::atan2(pose.trans[1], pose.trans[0]),
    };
    float base_rpy[3];
    float new_rpy[3];
    smooth_rpy(base_rpy);
    compose_rpy(base_rpy, observed_rpy, new_rpy);
    sanity_clip(new_rpy);
    if (!finite3(new_rpy)) {
        ++snapshot_.rejected_samples;
        result.snapshot = snapshot_;
        return result;
    }

    for (int i = 0; i < 3; ++i)
        block_rpys_[block_idx_][i] =
            (sample_idx_ * block_rpys_[block_idx_][i] + (kBlockSize - sample_idx_) * new_rpy[i]) /
            static_cast<float>(kBlockSize);

    sample_idx_ = (sample_idx_ + 1) % kBlockSize;
    ++snapshot_.accepted_samples;
    snapshot_.block_sample_count = sample_idx_;
    result.accepted = true;

    if (sample_idx_ == 0) {
        ++block_idx_;
        snapshot_.valid_blocks = std::min(kInputsWanted, std::max(block_idx_, snapshot_.valid_blocks));
        block_idx_ %= kInputsWanted;
        update_status();
        result.block_completed = true;
    } else {
        update_status();
    }

    result.snapshot = snapshot_;
    return result;
}

bool OnlineCalibrator::restore(const float rpy[3], int valid_blocks, const float spread[3])
{
    if (!rpy || !finite3(rpy) || !is_calibration_valid(rpy)) return false;
    /* openpilot calibrationd reset(rpy_init, valid_blocks)와 같이 저장된 블록 수를 그대로 쓴다.
     * 5블록 미만(저장소 기본값은 0)이면 rpy는 출발점일 뿐 미보정이고, 직진 5블록이 모일 때까지
     * 결합이 막힌다. 예전에는 5블록으로 올려 기본값을 보정 완료로 믿었다(2026-10-01 laneless
     * 0.2 m 치우침). */
    reset_to_rpy(rpy, std::max(0, valid_blocks));
    if (spread && finite3(spread)) {
        for (int i = 0; i < 3; ++i)
            snapshot_.spread[i] = std::max(0.0f, spread[i]);
    }
    return true;
}

OnlineCalibrator::Snapshot OnlineCalibrator::snapshot() const
{
    return snapshot_;
}

void OnlineCalibrator::output_rpy(float rpy[3]) const
{
    smooth_rpy(rpy);
}

void OnlineCalibrator::reset_to_rpy(const float rpy[3], int valid_blocks, const float *smooth_from)
{
    const int clamped_valid_blocks = std::max(0, std::min(valid_blocks, kInputsWanted));
    for (int b = 0; b < kInputsWanted; ++b) {
        for (int i = 0; i < 3; ++i)
            block_rpys_[b][i] = rpy[i];
    }
    sample_idx_ = 0;
    block_idx_ = 0;
    if (smooth_from) {
        for (int i = 0; i < 3; ++i)
            old_rpy_[i] = smooth_from[i];
        old_rpy_weight_ = 1.0f;
    } else {
        std::memset(old_rpy_, 0, sizeof(old_rpy_));
        old_rpy_weight_ = 0.0f;
    }
    for (int i = 0; i < 3; ++i) {
        snapshot_.rpy[i] = rpy[i];
        snapshot_.spread[i] = 0.0f;
    }
    snapshot_.valid_blocks = clamped_valid_blocks;
    snapshot_.block_sample_count = 0;
    snapshot_.status = clamped_valid_blocks >= kInputsNeeded && is_calibration_valid(rpy)
        ? CalibrationStatus::Calibrated
        : CalibrationStatus::Uncalibrated;
}

void OnlineCalibrator::update_status()
{
    float sum[3] = {};
    float min_v[3] = {};
    float max_v[3] = {};
    int valid_count = 0;
    for (int b = 0; b < snapshot_.valid_blocks; ++b) {
        if (b == block_idx_) continue;
        if (valid_count == 0) {
            min_v[0] = max_v[0] = block_rpys_[b][0];
            min_v[1] = max_v[1] = block_rpys_[b][1];
            min_v[2] = max_v[2] = block_rpys_[b][2];
        }
        for (int i = 0; i < 3; ++i) {
            const float value = block_rpys_[b][i];
            sum[i] += value;
            min_v[i] = std::min(min_v[i], value);
            max_v[i] = std::max(max_v[i], value);
        }
        ++valid_count;
    }

    if (valid_count > 0) {
        for (int i = 0; i < 3; ++i) {
            snapshot_.rpy[i] = sum[i] / static_cast<float>(valid_count);
            snapshot_.spread[i] = std::fabs(max_v[i] - min_v[i]);
        }
    } else {
        for (int i = 0; i < 3; ++i)
            snapshot_.spread[i] = 0.0f;
    }

    if (snapshot_.valid_blocks < kInputsNeeded) {
        if (snapshot_.status != CalibrationStatus::Recalibrating)
            snapshot_.status = CalibrationStatus::Uncalibrated;
    } else if (is_calibration_valid(snapshot_.rpy)) {
        snapshot_.status = CalibrationStatus::Calibrated;
    } else {
        snapshot_.status = CalibrationStatus::Invalid;
    }

    /* 편차가 크면 장착이 바뀐 것으로 보고 마지막 블록 하나에서 다시 모은다(openpilot과 같이
     * 1블록, recalibrating). 5블록이 다시 찰 때까지 보정 완료가 아니므로 저장하지 않고
     * 학습기도 보정을 믿지 않는다. roll은 관측하지 않으므로 보지 않는다. */
    const bool spread_too_high = snapshot_.spread[1] > kMaxAllowedPitchSpread ||
                                 snapshot_.spread[2] > kMaxAllowedYawSpread;
    if (snapshot_.status == CalibrationStatus::Calibrated && spread_too_high) {
        const int last_block = (block_idx_ + kInputsWanted - 1) % kInputsWanted;
        float smooth_from[3] = {snapshot_.rpy[0], snapshot_.rpy[1], snapshot_.rpy[2]};
        float last_rpy[3] = {block_rpys_[last_block][0], block_rpys_[last_block][1],
                             block_rpys_[last_block][2]};
        reset_to_rpy(last_rpy, 1, smooth_from);
        snapshot_.status = CalibrationStatus::Recalibrating;
    }
}

void OnlineCalibrator::smooth_rpy(float out[3]) const
{
    if (old_rpy_weight_ > 0.0f) {
        for (int i = 0; i < 3; ++i)
            out[i] = old_rpy_weight_ * old_rpy_[i] + (1.0f - old_rpy_weight_) * snapshot_.rpy[i];
    } else {
        for (int i = 0; i < 3; ++i)
            out[i] = snapshot_.rpy[i];
    }
}

bool OnlineCalibrator::is_calibration_valid(const float rpy[3]) const
{
    return kPitchMin < rpy[1] && rpy[1] < kPitchMax &&
           kYawMin < rpy[2] && rpy[2] < kYawMax;
}

void OnlineCalibrator::sanity_clip(float rpy[3]) const
{
    if (!finite3(rpy)) {
        rpy[0] = 0.0f;
        rpy[1] = 0.0f;
        rpy[2] = 0.0f;
        return;
    }
    rpy[1] = std::max(kPitchMin - kSanityMargin, std::min(kPitchMax + kSanityMargin, rpy[1]));
    rpy[2] = std::max(kYawMin - kSanityMargin, std::min(kYawMax + kSanityMargin, rpy[2]));
}
