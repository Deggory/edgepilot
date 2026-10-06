/* 온라인 보정과 모델 입력 변환을 openpilot 참조식과 대조한다: OnlineCalibrator ↔ calibrationd.py,
 * calibration_service의 저장·복원·수동 보정, app_config 환경 변수, 투영 행렬과 YUV6 워프 ↔
 * openpilot OpenCL 워프. */
#include "common/app_config.h"
#include "model/calibration_service.h"
#include "common/utils_math.h"
#include "model/model_input_transform.h"
#include "model/calibration_online.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>
#include <time.h>
#include <unistd.h>

namespace {

constexpr int kModelW = 512;
constexpr int kModelH = 256;
constexpr int kHalfW = kModelW / 2;
constexpr int kHalfH = kModelH / 2;
constexpr int kPlaneSize = kHalfW * kHalfH;
constexpr int kYuv6Floats = 6 * kPlaneSize;

constexpr double kPi = 3.14159265358979323846264338327950288;
// openpilot calibrationd.py 상수
constexpr double kMinSpeedFilter = 15.0 * 0.44704;
constexpr double kMaxVelAngleStd = 0.25 * kPi / 180.0;
constexpr double kMaxYawRateFilter = 2.0 * kPi / 180.0;
constexpr double kMaxAllowedYawSpread = 2.0 * kPi / 180.0;
constexpr double kMaxAllowedPitchSpread = 4.0 * kPi / 180.0;
constexpr double kSmoothCycles = 10.0;
constexpr double kPitchMin = -0.09074112085129739;
constexpr double kPitchMax = 0.17;  // upstream PITCH_LIMITS (mici 아님)
constexpr double kYawMin = -0.06912048084718224;
constexpr double kYawMax = 0.06912048084718235;
constexpr double kSanityMargin = 0.005;
const double kMaxHeightStd = std::exp(-3.5);
constexpr double kHeightInit = 1.22;
constexpr int kBlockSize = 100;
constexpr int kInputsNeeded = 5;
constexpr int kInputsWanted = 50;

void matmul3d(const double *a, const double *b, double *out)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k)
                sum += a[r * 3 + k] * b[k * 3 + c];
            out[r * 3 + c] = sum;
        }
    }
}

// rot = Rz(yaw)·Ry(pitch)·Rx(roll), openpilot rot_from_euler과 같은 순서
void rot_from_euler_ref(const double rpy[3], double *rot)
{
    const double cr = std::cos(rpy[0]);
    const double sr = std::sin(rpy[0]);
    const double cp = std::cos(rpy[1]);
    const double sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]);
    const double sy = std::sin(rpy[2]);

    const double rx[9] = {
        1.0, 0.0, 0.0,
        0.0, cr, -sr,
        0.0, sr, cr,
    };
    const double ry[9] = {
        cp, 0.0, sp,
        0.0, 1.0, 0.0,
        -sp, 0.0, cp,
    };
    const double rz[9] = {
        cy, -sy, 0.0,
        sy, cy, 0.0,
        0.0, 0.0, 1.0,
    };

    double tmp[9];
    matmul3d(ry, rx, tmp);
    matmul3d(rz, tmp, rot);
}

void euler_from_rot_ref(const double *rot, double rpy[3])
{
    rpy[0] = std::atan2(rot[2 * 3 + 1], rot[2 * 3 + 2]);
    rpy[1] = std::asin(std::max(-1.0, std::min(1.0, -rot[2 * 3 + 0])));
    rpy[2] = std::atan2(rot[1 * 3 + 0], rot[0 * 3 + 0]);
}

void compose_rpy_ref(const double base[3], const double observed[3], double out[3])
{
    double base_rot[9];
    double observed_rot[9];
    double composed[9];
    rot_from_euler_ref(base, base_rot);
    rot_from_euler_ref(observed, observed_rot);
    matmul3d(base_rot, observed_rot, composed);
    euler_from_rot_ref(composed, out);
}

bool finite3(const double v[3])
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

/* openpilot calibrationd.py의 Calibrator를 double로 옮긴 참조 구현. */
struct RefCalibrator {
    double rpys[kInputsWanted][3] = {};
    double heights[kInputsWanted] = {};
    double height = kHeightInit;
    double rpy[3] = {};
    double spread[3] = {};
    double old_rpy[3] = {};
    double old_rpy_weight = 0.0;
    int idx = 0;
    int block_idx = 0;
    int valid_blocks = 0;
    CalibrationStatus status = CalibrationStatus::Uncalibrated;
    uint64_t accepted = 0;
    uint64_t rejected = 0;

    void reset(const double init[3], int blocks, const double *smooth_from = nullptr,
               double height_init = kHeightInit)
    {
        valid_blocks = std::max(0, std::min(blocks, kInputsWanted));
        height = std::isfinite(height_init) ? height_init : kHeightInit;
        for (int b = 0; b < kInputsWanted; ++b) heights[b] = height;
        for (int i = 0; i < 3; ++i)
            rpy[i] = std::isfinite(init[i]) ? init[i] : 0.0;
        for (int b = 0; b < kInputsWanted; ++b) {
            for (int i = 0; i < 3; ++i)
                rpys[b][i] = rpy[i];
        }
        idx = 0;
        block_idx = 0;
        if (smooth_from) {
            for (int i = 0; i < 3; ++i)
                old_rpy[i] = smooth_from[i];
            old_rpy_weight = 1.0;
        } else {
            old_rpy[0] = old_rpy[1] = old_rpy[2] = 0.0;
            old_rpy_weight = 0.0;
        }
        update_status();
    }

    void smooth_rpy(double out[3]) const
    {
        if (old_rpy_weight > 0.0) {
            for (int i = 0; i < 3; ++i)
                out[i] = old_rpy_weight * old_rpy[i] + (1.0 - old_rpy_weight) * rpy[i];
        } else {
            for (int i = 0; i < 3; ++i)
                out[i] = rpy[i];
        }
    }

    static bool is_valid(const double rpy_in[3])
    {
        return kPitchMin < rpy_in[1] && rpy_in[1] < kPitchMax &&
               kYawMin < rpy_in[2] && rpy_in[2] < kYawMax;
    }

    static void sanity_clip(double rpy_in[3])
    {
        if (!finite3(rpy_in)) {
            rpy_in[0] = 0.0;
            rpy_in[1] = 0.0;
            rpy_in[2] = 0.0;
            return;
        }
        rpy_in[1] = std::max(kPitchMin - kSanityMargin, std::min(kPitchMax + kSanityMargin, rpy_in[1]));
        rpy_in[2] = std::max(kYawMin - kSanityMargin, std::min(kYawMax + kSanityMargin, rpy_in[2]));
    }

    void update_status()
    {
        double sum[3] = {};
        double height_sum = 0.0;
        double min_v[3] = {};
        double max_v[3] = {};
        int valid_count = 0;

        for (int b = 0; b < valid_blocks; ++b) {
            if (b == block_idx) continue;
            if (valid_count == 0) {
                for (int i = 0; i < 3; ++i)
                    min_v[i] = max_v[i] = rpys[b][i];
            }
            height_sum += heights[b];
            for (int i = 0; i < 3; ++i) {
                sum[i] += rpys[b][i];
                min_v[i] = std::min(min_v[i], rpys[b][i]);
                max_v[i] = std::max(max_v[i], rpys[b][i]);
            }
            ++valid_count;
        }

        if (valid_count > 0) {
            height = height_sum / valid_count;
            for (int i = 0; i < 3; ++i) {
                rpy[i] = sum[i] / valid_count;
                spread[i] = std::fabs(max_v[i] - min_v[i]);
            }
        } else {
            spread[0] = spread[1] = spread[2] = 0.0;
        }

        if (valid_blocks < kInputsNeeded) {
            if (status != CalibrationStatus::Recalibrating) status = CalibrationStatus::Uncalibrated;
        } else if (is_valid(rpy)) {
            status = CalibrationStatus::Calibrated;
        } else {
            status = CalibrationStatus::Invalid;
        }

        const bool spread_too_high = spread[1] > kMaxAllowedPitchSpread || spread[2] > kMaxAllowedYawSpread;
        if (status == CalibrationStatus::Calibrated && spread_too_high) {
            const int last_block = (block_idx + kInputsWanted - 1) % kInputsWanted;
            const double smooth_from[3] = {rpy[0], rpy[1], rpy[2]};
            const double last_rpy[3] = {rpys[last_block][0], rpys[last_block][1], rpys[last_block][2]};
            reset(last_rpy, 1, smooth_from);
            status = CalibrationStatus::Recalibrating;
        }
    }

    bool update(const PoseObservation &pose, double v_ego = 20.0)
    {
        old_rpy_weight = std::max(0.0, old_rpy_weight - 1.0 / kSmoothCycles);

        const double trans[3] = {pose.trans[0], pose.trans[1], pose.trans[2]};
        const double rot[3] = {pose.rot[0], pose.rot[1], pose.rot[2]};
        const double trans_std[3] = {pose.trans_std[0], pose.trans_std[1], pose.trans_std[2]};
        const bool valid_numbers = finite3(trans) && finite3(rot) && finite3(trans_std) &&
            std::isfinite(v_ego);
        const bool straight_and_fast = valid_numbers &&
            v_ego > kMinSpeedFilter &&
            trans[0] > kMinSpeedFilter &&
            std::fabs(rot[2]) < kMaxYawRateFilter;
        const bool rpy_certain = std::atan2(trans_std[1], trans[0]) < kMaxVelAngleStd;
        const bool height_certain = pose.road_trans_std[2] < kMaxHeightStd;
        const bool certain_if_calib = valid_numbers &&
            ((rpy_certain && height_certain) || (valid_blocks < kInputsNeeded));

        if (!straight_and_fast || !certain_if_calib) {
            ++rejected;
            return false;
        }

        const double observed_rpy[3] = {
            0.0,
            -std::atan2(trans[2], trans[0]),
            std::atan2(trans[1], trans[0]),
        };
        double base[3];
        double new_rpy[3];
        smooth_rpy(base);
        compose_rpy_ref(base, observed_rpy, new_rpy);
        sanity_clip(new_rpy);

        if (!finite3(new_rpy)) {
            ++rejected;
            return false;
        }

        for (int i = 0; i < 3; ++i)
            rpys[block_idx][i] =
                (idx * rpys[block_idx][i] + (kBlockSize - idx) * new_rpy[i]) /
                static_cast<double>(kBlockSize);
        const double new_height = std::isfinite(pose.road_trans[2]) ? pose.road_trans[2] : kHeightInit;
        heights[block_idx] = (idx * heights[block_idx] + (kBlockSize - idx) * new_height) /
                             static_cast<double>(kBlockSize);

        idx = (idx + 1) % kBlockSize;
        ++accepted;
        if (idx == 0) {
            ++block_idx;
            valid_blocks = std::min(kInputsWanted, std::max(block_idx, valid_blocks));
            block_idx %= kInputsWanted;
        }
        update_status();
        return true;
    }
};

PoseObservation make_pose(float tx = 20.0f, float ty = 0.2f, float tz = -0.4f,
                          float yaw_rate = 0.0f, float trans_std_y = 0.01f)
{
    PoseObservation pose{};
    pose.trans[0] = tx;
    pose.trans[1] = ty;
    pose.trans[2] = tz;
    pose.rot[0] = 0.0f;
    pose.rot[1] = 0.0f;
    pose.rot[2] = yaw_rate;
    pose.trans_std[0] = 0.01f;
    pose.trans_std[1] = trans_std_y;
    pose.trans_std[2] = 0.01f;
    pose.rot_std[0] = 0.01f;
    pose.rot_std[1] = 0.01f;
    pose.rot_std[2] = 0.01f;
    pose.road_trans[0] = 0.0f;
    pose.road_trans[1] = 0.0f;
    pose.road_trans[2] = 1.30f;
    pose.road_trans_std[0] = pose.road_trans_std[1] = pose.road_trans_std[2] = 0.01f;
    return pose;
}

void compare_snapshot(const OnlineCalibrator::Snapshot &actual, const RefCalibrator &expected,
                      const char *label)
{
    EXPECT_EQ(actual.valid_blocks, expected.valid_blocks) << std::string(label) + " valid_blocks";
    EXPECT_EQ(actual.block_sample_count, expected.idx)
        << std::string(label) + " block_sample_count";
    EXPECT_EQ(static_cast<int>(actual.status), static_cast<int>(expected.status))
        << std::string(label) + " status";
    EXPECT_NEAR(actual.accepted_samples, expected.accepted, 0.0)
        << std::string(label) + " accepted";
    EXPECT_NEAR(actual.rejected_samples, expected.rejected, 0.0)
        << std::string(label) + " rejected";
    EXPECT_NEAR(actual.height_m, expected.height, 1e-5) << std::string(label) + " height";
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual.rpy[i], expected.rpy[i], 1e-5) << std::string(label) + " rpy";
        EXPECT_NEAR(actual.spread[i], expected.spread[i], 1e-5) << std::string(label) + " spread";
    }
}

TEST(CalibrationEquivalence, OnlineCalibrator)
{
    const double zero[3] = {};
    RefCalibrator ref;
    ref.reset(zero, 0);
    OnlineCalibrator actual;

    PoseObservation low_speed = make_pose(6.0f);
    EXPECT_FALSE(actual.update(low_speed, 20.0f).accepted) << "카메라 속도가 낮은 표본은 버린다";

    PoseObservation low_vehicle_speed = make_pose(20.0f, 0.0f, 0.0f, 0.01f, 0.0f);
    EXPECT_FALSE(actual.update(low_vehicle_speed, 2.0f).accepted)
        << "CAN vEgo가 낮은 표본은 버린다";
    EXPECT_FALSE(ref.update(low_speed)) << "참조식도 저속 표본을 버린다";
    EXPECT_FALSE(ref.update(low_vehicle_speed, 2.0)) << "참조식도 CAN vEgo가 낮은 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "low_speed_reject");

    PoseObservation high_yaw = make_pose();
    high_yaw.rot[2] = static_cast<float>(3.0 * kPi / 180.0);
    EXPECT_FALSE(actual.update(high_yaw, 20.0f).accepted) << "요레이트가 큰 표본은 버린다";
    EXPECT_FALSE(ref.update(high_yaw)) << "참조식도 요레이트가 큰 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "high_yaw_reject");

    PoseObservation nan_pose = make_pose();
    nan_pose.trans[0] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(actual.update(nan_pose, 20.0f).accepted) << "NaN 표본은 버린다";
    EXPECT_FALSE(ref.update(nan_pose)) << "참조식도 NaN 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "nan_reject");

    const PoseObservation accepted_pose = make_pose();
    for (int i = 0; i < kBlockSize; ++i) {
        EXPECT_TRUE(actual.update(accepted_pose, 20.0f).accepted) << "블록 표본을 받아들인다";
        EXPECT_TRUE(ref.update(accepted_pose)) << "참조식도 블록 표본을 받아들인다";
    }
    compare_snapshot(actual.snapshot(), ref, "one_block");
    EXPECT_EQ(actual.snapshot().valid_blocks, 1) << "유효 블록 1개";

    for (int i = 0; i < 4 * kBlockSize; ++i) {
        actual.update(accepted_pose, 20.0f);
        ref.update(accepted_pose);
    }
    compare_snapshot(actual.snapshot(), ref, "five_blocks");
    EXPECT_EQ(static_cast<int>(actual.snapshot().status),
              static_cast<int>(CalibrationStatus::Calibrated))
        << "블록 5개가 차면 보정 완료";

    PoseObservation uncertain = make_pose();
    uncertain.trans_std[1] = 1.0f;
    EXPECT_FALSE(actual.update(uncertain, 20.0f).accepted)
        << "보정 뒤에도 trans 표준편차가 큰 표본은 버린다";
    EXPECT_FALSE(ref.update(uncertain)) << "참조식도 보정 뒤 trans 표준편차가 큰 표본을 버린다";
    compare_snapshot(actual.snapshot(), ref, "uncertain_after_calib");
}

/* upstream처럼 road_transform z로 카메라 높이를 블록 평균하고, 보정 뒤에는 높이 표준편차가
 * e^-3.5보다 큰 표본을 버린다. 장착 변경으로 다시 모을 때 높이는 HEIGHT_INIT로 돌아간다. */
TEST(CalibrationEquivalence, HeightFollowsUpstream)
{
    const double zero[3] = {};
    RefCalibrator ref;
    ref.reset(zero, 0);
    OnlineCalibrator actual;
    EXPECT_NEAR(actual.snapshot().height_m, 1.22f, 1e-6) << "시작 높이는 HEIGHT_INIT";
    PoseObservation pose = make_pose(20.0f, 0.0f, 0.0f);
    for (int i = 0; i < 5 * kBlockSize; ++i) {
        actual.update(pose, 20.0f);
        ref.update(pose);
    }
    compare_snapshot(actual.snapshot(), ref, "height_five_blocks");
    EXPECT_NEAR(actual.snapshot().height_m, 1.30f, 1e-4) << "블록 평균 높이";

    PoseObservation unsure = pose;
    unsure.road_trans_std[2] = 0.05f;
    EXPECT_FALSE(actual.update(unsure, 20.0f).accepted) << "보정 뒤 높이가 불확실한 표본은 버린다";
    EXPECT_FALSE(ref.update(unsure));
    compare_snapshot(actual.snapshot(), ref, "height_uncertain");

    OnlineCalibrator fresh;
    EXPECT_TRUE(fresh.update(unsure, 20.0f).accepted) << "보정 전에는 높이가 불확실해도 받는다";
}

/* upstream reset처럼 범위를 벗어난 저장값도 받아 Invalid로 두고, 저장된 높이로 시작한다. */
TEST(CalibrationEquivalence, RestoreOutOfRangeLikeUpstream)
{
    OnlineCalibrator calibrator;
    const float out_of_range[3] = {0.0f, 0.0f, 0.10f};  // yaw 5.7도: 한계 0.069 rad 밖
    ASSERT_TRUE(calibrator.restore(out_of_range, 12, nullptr, 1.35f));
    EXPECT_EQ(static_cast<int>(calibrator.snapshot().status), static_cast<int>(CalibrationStatus::Invalid));
    float out[3] = {};
    calibrator.output_rpy(out);
    EXPECT_NEAR(out[2], 0.10f, 1e-7) << "범위 밖이어도 그 rpy를 쓴다(upstream modeld도 rpyCalib를 그대로 쓴다)";
    EXPECT_NEAR(calibrator.snapshot().height_m, 1.35f, 1e-7) << "저장된 높이";
    const float nan_rpy[3] = {NAN, 0.0f, 0.0f};
    ASSERT_TRUE(calibrator.restore(nan_rpy, 3));
    calibrator.output_rpy(out);
    EXPECT_EQ(out[0], 0.0f) << "유한하지 않으면 RPY_INIT";
    EXPECT_EQ(calibrator.snapshot().valid_blocks, 3);
    const float pitch_016[3] = {0.0f, 0.16f, 0.0f};
    ASSERT_TRUE(calibrator.restore(pitch_016, 12));
    EXPECT_EQ(static_cast<int>(calibrator.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated))
        << "pitch 상한은 upstream 0.17";
}

TEST(CalibrationEquivalence, MountChangeRecalibratesLikeUpstream)
{
    const double zero[3] = {};
    RefCalibrator ref;
    ref.reset(zero, 0);
    OnlineCalibrator actual;
    const PoseObservation still = make_pose(20.0f, 0.0f, 0.0f);  // 현재 보정과 맞는 관측
    for (int i = 0; i < 6 * kBlockSize; ++i) {
        actual.update(still, 20.0f);
        ref.update(still);
    }
    ASSERT_EQ(static_cast<int>(actual.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated));

    // pitch 3도: openpilot 문턱(4도) 아래라 그대로 보정 완료로 섞인다.
    const float pitch3 = -20.0f * std::tan(static_cast<float>(3.0 * kPi / 180.0));
    const PoseObservation pitched = make_pose(20.0f, 0.0f, pitch3);
    for (int i = 0; i < kBlockSize; ++i) {
        actual.update(pitched, 20.0f);
        ref.update(pitched);
    }
    compare_snapshot(actual.snapshot(), ref, "pitch_3deg");
    EXPECT_EQ(static_cast<int>(actual.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated))
        << "pitch 편차 3도는 장착 변경으로 보지 않는다";

    // yaw 2도를 넘는 편차는 장착 변경: 마지막 블록 하나에서 다시 모은다.
    OnlineCalibrator yawed_actual;
    RefCalibrator yawed_ref;
    yawed_ref.reset(zero, 0);
    for (int i = 0; i < 6 * kBlockSize; ++i) {
        yawed_actual.update(still, 20.0f);
        yawed_ref.update(still);
    }
    const float yaw3 = 20.0f * std::tan(static_cast<float>(3.0 * kPi / 180.0));
    const PoseObservation yawed = make_pose(20.0f, yaw3, 0.0f);
    bool recalibrating = false;
    for (int i = 0; i < kBlockSize && !recalibrating; ++i) {
        yawed_actual.update(yawed, 20.0f);
        yawed_ref.update(yawed);
        recalibrating = yawed_actual.snapshot().status == CalibrationStatus::Recalibrating;
    }
    compare_snapshot(yawed_actual.snapshot(), yawed_ref, "yaw_3deg");
    ASSERT_TRUE(recalibrating) << "yaw 편차가 2도를 넘으면 recalibrating";
    EXPECT_EQ(yawed_actual.snapshot().valid_blocks, 1) << "openpilot처럼 1블록에서 다시 시작";
    EXPECT_GT(yawed_actual.snapshot().rpy[2], deg_to_rad(2.0f)) << "새 장착의 yaw로 옮겨 간다";

    // block_idx가 0부터 다시 세므로 openpilot도 새 블록 5개가 다 차야 보정 완료다.
    for (int i = 0; i < 4 * kBlockSize; ++i) {
        yawed_actual.update(still, 20.0f);
        yawed_ref.update(still);
    }
    compare_snapshot(yawed_actual.snapshot(), yawed_ref, "recalibrating_4_blocks");
    EXPECT_EQ(static_cast<int>(yawed_actual.snapshot().status),
              static_cast<int>(CalibrationStatus::Recalibrating)) << "5블록 전에는 recalibrating";
    for (int i = 0; i < kBlockSize; ++i) {
        yawed_actual.update(still, 20.0f);
        yawed_ref.update(still);
    }
    compare_snapshot(yawed_actual.snapshot(), yawed_ref, "recalibrated");
    EXPECT_EQ(static_cast<int>(yawed_actual.snapshot().status),
              static_cast<int>(CalibrationStatus::Calibrated)) << "5블록이 다시 차면 보정 완료";
}

ParsedModelOutput parsed_from_pose(const PoseObservation &pose)
{
    ParsedModelOutput output{};
    output.valid = true;
    output.has_pose = true;
    output.pose = pose;
    return output;
}

// 소스 트리를 더럽히지 않게 테스트마다 시스템 임시 폴더를 params 디렉터리로 쓰고 끝나면 지운다.
struct TempParamsDir {
    std::string dir;
    explicit TempParamsDir(const char *name)
        : dir((std::filesystem::temp_directory_path() / (std::string("edgepilot_test_") + name)).string())
    {
        std::filesystem::remove_all(dir);
        std::filesystem::create_directories(dir);
        setenv("EDGEPILOT_PARAMS_DIR", dir.c_str(), 1);
    }
    ~TempParamsDir()
    {
        unsetenv("EDGEPILOT_PARAMS_DIR");
        std::filesystem::remove_all(dir);
    }
    std::string file(const char *name) const { return dir + "/" + name; }
};

TEST(CalibrationEquivalence, CalibrationService)
{
    const TempParamsDir params("calibration_service");

    OnlineCalibrator restored_calibrator;
    const float restored_rpy[3] = {0.0f, deg_to_rad(2.0f), deg_to_rad(-0.75f)};
    EXPECT_TRUE(restored_calibrator.restore(restored_rpy, 12))
        << "저장된 유효 보정값을 복원한다";
    float restored_output[3] = {};
    restored_calibrator.output_rpy(restored_output);
    EXPECT_NEAR(restored_output[1], restored_rpy[1], 1e-7)
        << "복원한 pitch로 보정기를 초기화한다";
    EXPECT_NEAR(restored_output[2], restored_rpy[2], 1e-7) << "복원한 yaw로 보정기를 초기화한다";
    EXPECT_EQ(restored_calibrator.snapshot().valid_blocks, 12) << "복원한 유효 블록 수";

    AppConfig auto_config;
    auto_config.calibration_auto = true;
    auto_config.manual_calibration = false;
    auto_config.log_calibration = false;
    CalibrationService service(auto_config);

    const ParsedModelOutput output = parsed_from_pose(make_pose());
    for (int i = 0; i < kBlockSize; ++i)
        service.update(output, 20.0f);

    float input_rpy[3] = {};
    service.input_rpy(input_rpy);
    const OnlineCalibrator::Snapshot snapshot = service.snapshot();
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(input_rpy[i], snapshot.rpy[i], 1e-7)
            << "자동 모드의 input_rpy는 온라인 스냅샷을 따른다";

    AppConfig manual_config;
    manual_config.calibration_auto = true;
    manual_config.manual_calibration = true;
    manual_config.manual_roll = deg_to_rad(0.4f);
    manual_config.manual_pitch = deg_to_rad(1.0f);
    manual_config.manual_yaw = deg_to_rad(-0.5f);
    CalibrationService manual(manual_config);
    for (int i = 0; i < 5 * kBlockSize; ++i)
        manual.update(output, 20.0f);

    float manual_rpy[3] = {};
    manual.input_rpy(manual_rpy);
    EXPECT_NEAR(manual_rpy[0], manual_config.manual_roll, 1e-7) << "수동 roll이 우선한다";
    EXPECT_NEAR(manual_rpy[1], manual_config.manual_pitch, 1e-7) << "수동 pitch가 우선한다";
    EXPECT_NEAR(manual_rpy[2], manual_config.manual_yaw, 1e-7) << "수동 yaw가 우선한다";
    EXPECT_EQ(manual.snapshot().valid_blocks, 5)
        << "수동 보정도 저장된 보정값으로 남는다";

    AppConfig restored_config;
    restored_config.calibration_auto = true;
    restored_config.manual_calibration = false;
    CalibrationService restored(restored_config);
    float persisted_rpy[3] = {};
    restored.input_rpy(persisted_rpy);
    EXPECT_NEAR(persisted_rpy[0], manual_config.manual_roll, 1e-7) << "저장된 roll을 다시 읽는다";
    EXPECT_NEAR(persisted_rpy[1], manual_config.manual_pitch, 1e-7) << "저장된 pitch를 다시 읽는다";
    EXPECT_NEAR(persisted_rpy[2], manual_config.manual_yaw, 1e-7) << "저장된 yaw를 다시 읽는다";
    EXPECT_EQ(static_cast<int>(restored.snapshot().status),
              static_cast<int>(CalibrationStatus::Calibrated))
        << "다시 읽은 보정은 보정 완료 상태다";

}

TEST(CalibrationEquivalence, ResetRequestRecalibratesFromScratch)
{
    const TempParamsDir params("calibration_reset");
    const std::string calibration = params.file("calibration.json");
    const std::string reset_request = params.file("calibration_reset");
    setenv("EDGEPILOT_CALIBRATION_RESET_PATH", reset_request.c_str(), 1);

    AppConfig manual_config;
    manual_config.manual_calibration = true;
    manual_config.manual_pitch = deg_to_rad(2.3f);
    { CalibrationService seed(manual_config); }  // pitch 2.3도, 5블록을 저장해 둔다

    AppConfig auto_config;
    auto_config.calibration_auto = true;
    auto_config.manual_calibration = false;
    const ParsedModelOutput no_pose{};
    {
        CalibrationService service(auto_config);
        ASSERT_EQ(service.snapshot().valid_blocks, 5) << "저장된 보정으로 시작한다";
        service.update(no_pose, 0.0f);
        EXPECT_EQ(service.snapshot().valid_blocks, 5) << "요청이 없으면 그대로다";

        std::FILE *request = std::fopen(reset_request.c_str(), "w");
        ASSERT_NE(request, nullptr);
        std::fclose(request);
        // 요청은 1초에 한 번 본다. 첫 update에서 이미 봤으므로 간격을 넘겨 다시 부른다.
        for (int i = 0; i < 12 && service.snapshot().valid_blocks != 0; ++i) {
            struct timespec pause = {0, 100'000'000};
            nanosleep(&pause, nullptr);
            service.update(no_pose, 0.0f);
        }
        EXPECT_EQ(service.snapshot().valid_blocks, 0) << "초기화하면 블록이 없다";
        EXPECT_EQ(static_cast<int>(service.snapshot().status),
                  static_cast<int>(CalibrationStatus::Uncalibrated));
        float rpy[3] = {1.0f, 1.0f, 1.0f};
        service.input_rpy(rpy);
        EXPECT_NEAR(rpy[1], 0.0f, 1e-7) << "모델 입력도 0에서 다시 시작한다";
        EXPECT_NE(access(reset_request.c_str(), F_OK), 0) << "요청 파일은 한 번 쓰고 지운다";
    }  // 저장 스레드가 끝날 때까지 기다린다
    EXPECT_NE(access(calibration.c_str(), F_OK), 0) << "저장된 보정 파일을 지운다";
    {
        CalibrationService restarted(auto_config);
        EXPECT_EQ(restarted.snapshot().valid_blocks, 0) << "재시작해도 예전 값을 다시 읽지 않는다";
    }

    unsetenv("EDGEPILOT_CALIBRATION_RESET_PATH");
}

/* 초기화 뒤 다시 수렴한 보정은 저장된다(2026-10-01: 초기화의 삭제 표시가 남아 저장 대신 파일을
 * 지웠고, 재시작하면 저장소 기본값이 복원돼 laneless가 0.2 m 치우쳤다). */
TEST(CalibrationEquivalence, RecalibrationAfterResetIsSaved)
{
    // ctest가 병렬로 돌리므로 ResetRequestRecalibratesFromScratch와 다른 디렉터리를 쓴다.
    const TempParamsDir params("calibration_reset_save");
    const std::string calibration = params.file("calibration.json");
    const std::string reset_request = params.file("calibration_reset");
    setenv("EDGEPILOT_CALIBRATION_RESET_PATH", reset_request.c_str(), 1);

    AppConfig manual_config;
    manual_config.manual_calibration = true;
    manual_config.manual_pitch = deg_to_rad(2.3f);
    { CalibrationService seed(manual_config); }

    AppConfig auto_config;
    auto_config.calibration_auto = true;
    auto_config.manual_calibration = false;
    {
        CalibrationService service(auto_config);
        const ParsedModelOutput no_pose{};
        service.update(no_pose, 0.0f);
        std::FILE *request = std::fopen(reset_request.c_str(), "w");
        ASSERT_NE(request, nullptr);
        std::fclose(request);
        for (int i = 0; i < 12 && service.snapshot().valid_blocks != 0; ++i) {
            struct timespec pause = {0, 100'000'000};
            nanosleep(&pause, nullptr);
            service.update(no_pose, 0.0f);
        }
        ASSERT_EQ(service.snapshot().valid_blocks, 0);
        // 직진 5블록(500프레임)으로 다시 보정 완료
        ParsedModelOutput straight{};
        straight.has_pose = true;
        straight.pose = make_pose();
        for (int i = 0; i < 5 * 100; ++i) service.update(straight, 20.0f);
        ASSERT_EQ(static_cast<int>(service.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated));
    }  // 저장 스레드가 끝날 때까지 기다린다
    EXPECT_EQ(access(calibration.c_str(), F_OK), 0) << "다시 수렴한 보정을 저장한다";
    {
        CalibrationService restarted(auto_config);
        EXPECT_EQ(restarted.snapshot().valid_blocks, 5) << "재시작하면 새 보정으로 시작한다";
    }

    unsetenv("EDGEPILOT_CALIBRATION_RESET_PATH");
}

/* openpilot과 같이 저장된 블록 수를 그대로 쓴다: 5 미만(저장소 기본값 0)이면 rpy는 출발점일 뿐
 * 미보정이고, 직진 5블록이 모여야 보정 완료다. */
TEST(CalibrationEquivalence, StoredCalibrationBelowFiveBlocksStartsUncalibrated)
{
    const float stored[3] = {0.0f, deg_to_rad(-1.0f), deg_to_rad(-1.12f)};
    OnlineCalibrator calibrator;
    ASSERT_TRUE(calibrator.restore(stored, 0));
    EXPECT_EQ(calibrator.snapshot().valid_blocks, 0);
    EXPECT_EQ(static_cast<int>(calibrator.snapshot().status), static_cast<int>(CalibrationStatus::Uncalibrated));
    float out[3] = {};
    calibrator.output_rpy(out);
    EXPECT_NEAR(out[2], stored[2], 1e-7) << "모델 입력은 저장된 rpy에서 출발한다";
    for (int i = 0; i < 5 * 100; ++i) calibrator.update(make_pose(), 20.0f);
    EXPECT_EQ(calibrator.snapshot().valid_blocks, 5);
    EXPECT_EQ(static_cast<int>(calibrator.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated));

    OnlineCalibrator converged;
    ASSERT_TRUE(converged.restore(stored, 12));
    EXPECT_EQ(static_cast<int>(converged.snapshot().status), static_cast<int>(CalibrationStatus::Calibrated));

    // 서비스: 저장소 기본값 같은 파일(valid_blocks 0)로 시작하면 미보정
    const TempParamsDir params("calibration_uncalibrated");
    const std::string calibration = params.file("calibration.json");
    {
        std::FILE *f = std::fopen(calibration.c_str(), "w");
        ASSERT_NE(f, nullptr);
        std::fprintf(f, "{\n  \"version\": 1,\n  \"rpy_rad\": [0, %.9f, %.9f],\n  \"spread_rad\": [0, 0, 0],\n"
                        "  \"valid_blocks\": 0\n}\n", stored[1], stored[2]);
        std::fclose(f);
    }
    AppConfig auto_config;
    auto_config.calibration_auto = true;
    auto_config.manual_calibration = false;
    {
        CalibrationService service(auto_config);
        EXPECT_EQ(static_cast<int>(service.snapshot().status), static_cast<int>(CalibrationStatus::Uncalibrated));
        float rpy[3] = {};
        service.input_rpy(rpy);
        EXPECT_NEAR(rpy[2], stored[2], 1e-6);
    }
}

TEST(CalibrationEquivalence, AppConfigEnvFeedback)
{
    unsetenv("EDGEPILOT_CALIB_ROLL_DEG");
    unsetenv("EDGEPILOT_CALIB_PITCH_DEG");
    unsetenv("EDGEPILOT_CALIB_YAW_DEG");

    setenv("EDGEPILOT_CALIB_PITCH_DEG", "1.25", 1);
    setenv("EDGEPILOT_CALIB_YAW_DEG", "-0.75", 1);
    AppConfig fallback = AppConfig::from_env_defaults();
    EXPECT_TRUE(fallback.manual_calibration)
        << "수동 보정 환경 변수를 주면 수동 모드가 켜진다";
    EXPECT_NEAR(fallback.manual_pitch, deg_to_rad(1.25f), 1e-7) << "수동 pitch 환경 변수 해석";
    EXPECT_NEAR(fallback.manual_yaw, deg_to_rad(-0.75f), 1e-7) << "수동 yaw 환경 변수 해석";
    EXPECT_EQ(fallback.nv12_width, kDefaultAiWidth) << "ISP 출력 폭 기본값은 오버스캔 폭";
    EXPECT_EQ(fallback.nv12_height, kDefaultAiHeight) << "ISP 출력 높이 기본값은 오버스캔 높이";
    EXPECT_NEAR(fallback.input_warp_fx, kDefaultInputWarpFx, 1e-5)
        << "ISP 출력 크기로 카메라 fx를 맞춘다";
    EXPECT_NEAR(fallback.input_warp_fy, kDefaultInputWarpFy, 1e-5)
        << "ISP 출력 크기로 카메라 fy를 맞춘다";
    EXPECT_NEAR(fallback.input_warp_cx, kDefaultInputWarpCx, 1e-5)
        << "ISP 출력 크기로 카메라 cx를 맞춘다";
    EXPECT_NEAR(fallback.input_warp_cy, kDefaultInputWarpCy, 1e-5)
        << "ISP 출력 크기로 카메라 cy를 맞춘다";

    // 다른 카메라(예: K230 녹화 리플레이)의 1080p 내부 파라미터로 바꾸면 비례 환산된다
    setenv("EDGEPILOT_CAMERA_INTRINSICS", "1583.3981,1583.7622,954.9441,545.1774", 1);
    AppConfig k230 = AppConfig::from_env_defaults();
    EXPECT_NEAR(k230.input_warp_fx, 1583.3981f * 1280.0f / 1920.0f, 1e-3) << "fx 덮어쓰기";
    EXPECT_NEAR(k230.input_warp_cy, 545.1774f * 720.0f / 1080.0f, 1e-3) << "cy 덮어쓰기";
    k230.set_warp_source(1920, 1080);
    EXPECT_NEAR(k230.input_warp_cx, 954.9441f, 1e-3) << "1080p 소스면 그대로";
    setenv("EDGEPILOT_CAMERA_INTRINSICS", "oops", 1);
    EXPECT_THROW(AppConfig::from_env_defaults(), std::runtime_error) << "잘못된 형식은 거부";
    unsetenv("EDGEPILOT_CAMERA_INTRINSICS");

    unsetenv("EDGEPILOT_CALIB_ROLL_DEG");
    unsetenv("EDGEPILOT_CALIB_PITCH_DEG");
    unsetenv("EDGEPILOT_CALIB_YAW_DEG");
}

/* 모델 프레임 → 카메라 영상 투영 행렬의 참조식. openpilot common/transformations/model.py의
 * get_warp_matrix를 double로 옮겼다: intrinsics @ view_frame_from_device_frame @ rot_from_euler(rpy)
 * @ inv(model_intrinsics @ view_frame_from_device_frame). */
void projection_reference(float roll, float pitch, float yaw, float fx, float fy, float cx, float cy,
                          double *projection, ModelFrame model_frame = ModelFrame::MedModel)
{
    const bool big = model_frame == ModelFrame::SmallBigModel;
    const double model_f = big ? 455.0 : 910.0;
    const double model_k[9] = {
        model_f, 0.0, 256.0,
        0.0, model_f, big ? 0.5 * (256.0 + 47.6) : 47.6,
        0.0, 0.0, 1.0,
    };
    const double view[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};
    const double k[9] = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    const double rpy[3] = {roll, pitch, yaw};
    double rot[9];
    rot_from_euler_ref(rpy, rot);
    double kv[9], kvr[9], mkv[9];
    matmul3d(k, view, kv);
    matmul3d(kv, rot, kvr);
    matmul3d(model_k, view, mkv);
    // mkv의 역행렬(여인수 전개)
    const double *m = mkv;
    const double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
                       m[2] * (m[3] * m[7] - m[4] * m[6]);
    const double inv[9] = {
        (m[4] * m[8] - m[5] * m[7]) / det, (m[2] * m[7] - m[1] * m[8]) / det, (m[1] * m[5] - m[2] * m[4]) / det,
        (m[5] * m[6] - m[3] * m[8]) / det, (m[0] * m[8] - m[2] * m[6]) / det, (m[2] * m[3] - m[0] * m[5]) / det,
        (m[3] * m[7] - m[4] * m[6]) / det, (m[1] * m[6] - m[0] * m[7]) / det, (m[0] * m[4] - m[1] * m[3]) / det,
    };
    matmul3d(kvr, inv, projection);
}

// Y 평면 투영을 UV 평면(scale 0.5)으로 옮긴다
void transform_scale_buffer_ref(const double *in, double scale, double *out)
{
    const double transform_out[9] = {
        1.0 / scale, 0.0, 0.5,
        0.0, 1.0 / scale, 0.5,
        0.0, 0.0, 1.0,
    };
    const double transform_in[9] = {
        scale, 0.0, -0.5 * scale,
        0.0, scale, -0.5 * scale,
        0.0, 0.0, 1.0,
    };
    double tmp[9];
    matmul3d(in, transform_out, tmp);
    matmul3d(transform_in, tmp, out);
}

uint8_t clamp_u8(int value)
{
    return static_cast<uint8_t>(std::min(255, std::max(0, value)));
}

/* openpilot modeld transform.cl의 warpPerspective 한 픽셀(INTER_BITS 5, 계수 15비트 고정소수점). */
uint8_t warp_sample_opencl_ref(const uint8_t *src, int src_w, int src_h, int stride_bytes,
                               int bytes_per_pixel, int channel, const double *m, int dx, int dy)
{
    constexpr int kInterBits = 5;
    constexpr int kInterTabSize = 1 << kInterBits;
    constexpr int kCoefBits = 15;
    constexpr int kCoefScale = 1 << kCoefBits;

    const double x0 = m[0] * dx + m[1] * dy + m[2];
    const double y0 = m[3] * dx + m[4] * dy + m[5];
    const double w = m[6] * dx + m[7] * dy + m[8];
    const double scale = w != 0.0 ? static_cast<double>(kInterTabSize) / w : 0.0;
    const int x_fixed = static_cast<int>(std::rint(x0 * scale));
    const int y_fixed = static_cast<int>(std::rint(y0 * scale));
    const int sx = static_cast<int>(std::floor(static_cast<double>(x_fixed) / kInterTabSize));
    const int sy = static_cast<int>(std::floor(static_cast<double>(y_fixed) / kInterTabSize));
    const int ax = x_fixed - sx * kInterTabSize;
    const int ay = y_fixed - sy * kInterTabSize;
    const double tabx = static_cast<double>(ax) / kInterTabSize;
    const double taby = static_cast<double>(ay) / kInterTabSize;
    const int weights[4] = {
        static_cast<int>(std::lrint((1.0 - taby) * (1.0 - tabx) * kCoefScale)),
        static_cast<int>(std::lrint((1.0 - taby) * tabx * kCoefScale)),
        static_cast<int>(std::lrint(taby * (1.0 - tabx) * kCoefScale)),
        static_cast<int>(std::lrint(taby * tabx * kCoefScale)),
    };
    const int xs[4] = {sx, sx + 1, sx, sx + 1};
    const int ys[4] = {sy, sy, sy + 1, sy + 1};

    int64_t sum = 0;
    for (int i = 0; i < 4; ++i) {
        int value = 0;
        if (xs[i] >= 0 && xs[i] < src_w && ys[i] >= 0 && ys[i] < src_h) {
            value = src[ys[i] * stride_bytes + xs[i] * bytes_per_pixel + channel];
        }
        sum += static_cast<int64_t>(value) * weights[i];
    }
    return clamp_u8(static_cast<int>((sum + (1 << (kCoefBits - 1))) >> kCoefBits));
}

// 위치마다 값이 다른 합성 NV12
void fill_nv12(std::vector<uint8_t> &nv12, int width, int height)
{
    uint8_t *y_plane = nv12.data();
    uint8_t *uv_plane = y_plane + width * height;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x)
            y_plane[y * width + x] = static_cast<uint8_t>((x * 3 + y * 5 + (x * y) / 17) & 0xff);
    }
    for (int y = 0; y < height / 2; ++y) {
        for (int x = 0; x < width / 2; ++x) {
            uv_plane[y * width + x * 2] = static_cast<uint8_t>((64 + x * 2 + y * 3) & 0xff);
            uv_plane[y * width + x * 2 + 1] = static_cast<uint8_t>((192 + x * 5 + y) & 0xff);
        }
    }
}

// 워프 없이 NV12 512x256을 openpilot YUV6 순서(Y00, Y10, Y01, Y11, U, V)로 싼다
void pack_direct_openpilot_order(const uint8_t *nv12, float *out)
{
    const uint8_t *y_plane = nv12;
    const uint8_t *uv_plane = nv12 + kModelW * kModelH;
    float *y00_plane = out;
    float *y10_plane = y00_plane + kPlaneSize;
    float *y01_plane = y10_plane + kPlaneSize;
    float *y11_plane = y01_plane + kPlaneSize;
    float *u_plane = y11_plane + kPlaneSize;
    float *v_plane = u_plane + kPlaneSize;

    for (int y2 = 0; y2 < kHalfH; ++y2) {
        const uint8_t *y0 = y_plane + (y2 * 2) * kModelW;
        const uint8_t *y1 = y0 + kModelW;
        const uint8_t *uv = uv_plane + y2 * kModelW;
        for (int x2 = 0; x2 < kHalfW; ++x2) {
            const int dst = y2 * kHalfW + x2;
            y00_plane[dst] = static_cast<float>(y0[x2 * 2 + 0]);
            y10_plane[dst] = static_cast<float>(y1[x2 * 2 + 0]);
            y01_plane[dst] = static_cast<float>(y0[x2 * 2 + 1]);
            y11_plane[dst] = static_cast<float>(y1[x2 * 2 + 1]);
            u_plane[dst] = static_cast<float>(uv[x2 * 2 + 0]);
            v_plane[dst] = static_cast<float>(uv[x2 * 2 + 1]);
        }
    }
}

// OpenCL 참조 샘플러로 워프한 뒤 YUV6로 싼다
void warp_pack_opencl_ref(const uint8_t *nv12, int src_w, int src_h,
                          const double *projection_y, float *out)
{
    double projection_uv[9];
    transform_scale_buffer_ref(projection_y, 0.5, projection_uv);

    const uint8_t *y_src = nv12;
    const uint8_t *uv_src = nv12 + src_w * src_h;
    float *y00_plane = out;
    float *y10_plane = y00_plane + kPlaneSize;
    float *y01_plane = y10_plane + kPlaneSize;
    float *y11_plane = y01_plane + kPlaneSize;
    float *u_plane = y11_plane + kPlaneSize;
    float *v_plane = u_plane + kPlaneSize;

    for (int y2 = 0; y2 < kHalfH; ++y2) {
        for (int x2 = 0; x2 < kHalfW; ++x2) {
            const int dst = y2 * kHalfW + x2;
            const int ox = x2 * 2;
            const int oy = y2 * 2;
            y00_plane[dst] = warp_sample_opencl_ref(y_src, src_w, src_h, src_w, 1, 0,
                                                    projection_y, ox, oy);
            y10_plane[dst] = warp_sample_opencl_ref(y_src, src_w, src_h, src_w, 1, 0,
                                                    projection_y, ox, oy + 1);
            y01_plane[dst] = warp_sample_opencl_ref(y_src, src_w, src_h, src_w, 1, 0,
                                                    projection_y, ox + 1, oy);
            y11_plane[dst] = warp_sample_opencl_ref(y_src, src_w, src_h, src_w, 1, 0,
                                                    projection_y, ox + 1, oy + 1);
            u_plane[dst] = warp_sample_opencl_ref(uv_src, src_w / 2, src_h / 2, src_w, 2, 0,
                                                  projection_uv, x2, y2);
            v_plane[dst] = warp_sample_opencl_ref(uv_src, src_w / 2, src_h / 2, src_w, 2, 1,
                                                  projection_uv, x2, y2);
        }
    }
}

struct DiffStats {
    double mean = 0.0;
    double max = 0.0;
    double inner_mean = 0.0;
    double inner_max = 0.0;
};

// YUV6 두 개의 절대 오차. inner는 가장자리 8픽셀을 뺀 영역
DiffStats diff_stats(const std::vector<float> &a, const std::vector<float> &b)
{
    DiffStats stats;
    double sum = 0.0;
    double inner_sum = 0.0;
    int count = 0;
    int inner_count = 0;
    for (int plane = 0; plane < 6; ++plane) {
        for (int y = 0; y < kHalfH; ++y) {
            for (int x = 0; x < kHalfW; ++x) {
                const int idx = plane * kPlaneSize + y * kHalfW + x;
                const double d = std::fabs(a[idx] - b[idx]);
                sum += d;
                stats.max = std::max(stats.max, d);
                ++count;
                if (x >= 8 && x < kHalfW - 8 && y >= 8 && y < kHalfH - 8) {
                    inner_sum += d;
                    stats.inner_max = std::max(stats.inner_max, d);
                    ++inner_count;
                }
            }
        }
    }
    stats.mean = sum / count;
    stats.inner_mean = inner_sum / inner_count;
    return stats;
}

TEST(CalibrationEquivalence, ProjectionAndYuv6)
{
    AppConfig camera_config;
    ModelInputTransform camera_transform(camera_config);
    float camera_projection[9];
    double camera_reference[9];
    camera_transform.projection_matrix(camera_projection);
    projection_reference(0.0, 0.0, 0.0, camera_config.input_warp_fx,
                         camera_config.input_warp_fy, camera_config.input_warp_cx,
                         camera_config.input_warp_cy, camera_reference);
    for (int i = 0; i < 9; ++i)
        EXPECT_NEAR(camera_projection[i], camera_reference[i], 1e-4)
            << "기본 투영은 MaixCAM2 카메라 내부 파라미터를 쓴다";

    ModelInputTransform sbig_camera_transform(camera_config, ModelFrame::SmallBigModel);
    float sbig_camera_projection[9];
    double sbig_camera_reference[9];
    sbig_camera_transform.projection_matrix(sbig_camera_projection);
    projection_reference(0.0, 0.0, 0.0, camera_config.input_warp_fx,
                         camera_config.input_warp_fy, camera_config.input_warp_cx,
                         camera_config.input_warp_cy, sbig_camera_reference, ModelFrame::SmallBigModel);
    for (int i = 0; i < 9; ++i)
        EXPECT_NEAR(sbig_camera_projection[i], sbig_camera_reference[i], 1e-4)
            << "sbig 투영은 openpilot 가상 카메라를 쓴다";

    const std::array<std::array<float, 3>, 7> cases = {{
        {{0.0f, 0.0f, 0.0f}},
        {{0.0f, deg_to_rad(1.5f), 0.0f}},
        {{0.0f, deg_to_rad(-1.5f), 0.0f}},
        {{0.0f, 0.0f, deg_to_rad(1.0f)}},
        {{0.0f, deg_to_rad(1.1f), deg_to_rad(-0.8f)}},
        {{deg_to_rad(1.0f), 0.0f, 0.0f}},
        {{deg_to_rad(-2.0f), deg_to_rad(2.5f), deg_to_rad(1.5f)}},
    }};

    for (const auto &rpy : cases) {
        AppConfig config;
        config.manual_roll = rpy[0];
        config.manual_pitch = rpy[1];
        config.manual_yaw = rpy[2];
        ModelInputTransform transform(config);
        float actual[9];
        double expected[9];
        transform.projection_matrix(actual);
        projection_reference(rpy[0], rpy[1], rpy[2], config.input_warp_fx,
                             config.input_warp_fy, config.input_warp_cx,
                             config.input_warp_cy, expected);
        for (int i = 0; i < 9; ++i) {
            const double tolerance = std::max(1e-4, std::fabs(expected[i]) * 1e-5);
            EXPECT_NEAR(actual[i], expected[i], tolerance) << "투영 행렬";
        }
    }

    std::vector<uint8_t> nv12(kModelW * kModelH * 3 / 2);
    std::vector<float> direct(kYuv6Floats, 0.0f);
    std::vector<float> warped(kYuv6Floats, 0.0f);
    std::vector<float> ref(kYuv6Floats, 0.0f);
    std::vector<float> sbig_warped(kYuv6Floats, 0.0f);
    std::vector<float> sbig_ref(kYuv6Floats, 0.0f);
    fill_nv12(nv12, kModelW, kModelH);
    pack_direct_openpilot_order(nv12.data(), direct.data());

    AppConfig identity_config;
    identity_config.input_warp_fx = 910.0f;
    identity_config.input_warp_fy = 910.0f;
    identity_config.input_warp_cx = 256.0f;
    identity_config.input_warp_cy = 47.6f;
    ModelInputTransform identity(identity_config);
    identity.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, warped);
    DiffStats identity_diff = diff_stats(direct, warped);
    EXPECT_NEAR(identity_diff.max, 0.0, 0.0)
        << "rpy가 0이면 워프한 YUV6가 직접 패킹과 비트까지 같다";

    AppConfig pitch_config;
    pitch_config.input_warp_fx = kDefaultModelFx;
    pitch_config.input_warp_fy = kDefaultModelFy;
    pitch_config.input_warp_cx = kDefaultModelCx;
    pitch_config.input_warp_cy = kDefaultModelCy;
    pitch_config.manual_pitch = deg_to_rad(1.5f);
    pitch_config.manual_yaw = deg_to_rad(-0.6f);
    ModelInputTransform pitched(pitch_config);
    pitched.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, warped);
    double projection_y[9];
    projection_reference(0.0f, pitch_config.manual_pitch, pitch_config.manual_yaw,
                         pitch_config.input_warp_fx, pitch_config.input_warp_fy,
                         pitch_config.input_warp_cx, pitch_config.input_warp_cy, projection_y);
    warp_pack_opencl_ref(nv12.data(), kModelW, kModelH, projection_y, ref.data());
    DiffStats warp_diff = diff_stats(ref, warped);
    EXPECT_LT(warp_diff.mean, 1.0) << "워프 평균 절대 오차(openpilot OpenCL 참조 대비)";
    EXPECT_LT(warp_diff.inner_max, 8.0)
        << "워프 안쪽 최대 오차(openpilot OpenCL 참조 대비)";

    ModelInputTransform sbig_pitched(pitch_config, ModelFrame::SmallBigModel);
    sbig_pitched.nv12_to_yuv6_warped(nv12.data(), kModelW, kModelH, sbig_warped);
    projection_reference(0.0f, pitch_config.manual_pitch, pitch_config.manual_yaw,
                         pitch_config.input_warp_fx, pitch_config.input_warp_fy,
                         pitch_config.input_warp_cx, pitch_config.input_warp_cy, projection_y,
                         ModelFrame::SmallBigModel);
    warp_pack_opencl_ref(nv12.data(), kModelW, kModelH, projection_y, sbig_ref.data());
    DiffStats sbig_warp_diff = diff_stats(sbig_ref, sbig_warped);
    EXPECT_LT(sbig_warp_diff.mean, 1.0) << "sbig 워프 평균 절대 오차(openpilot OpenCL 참조 대비)";
    EXPECT_LT(sbig_warp_diff.inner_max, 8.0)
        << "sbig 워프 안쪽 최대 오차(openpilot OpenCL 참조 대비)";

    constexpr int kSourceW = 640;
    constexpr int kSourceH = 360;
    std::vector<uint8_t> source_nv12(kSourceW * kSourceH * 3 / 2);
    std::vector<float> compact(kYuv6Floats, 0.0f);
    std::vector<float> opencl(kYuv6Floats, 0.0f);
    fill_nv12(source_nv12, kSourceW, kSourceH);
    AppConfig source_config;
    const std::array<std::array<float, 3>, 6> rpy_cases = {{
        {{0.0f, 0.0f, 0.0f}},
        {{0.0f, deg_to_rad(-0.75f), deg_to_rad(1.1f)}},
        {{deg_to_rad(0.5f), deg_to_rad(2.0f), deg_to_rad(-2.5f)}},
        {{deg_to_rad(-0.5f), deg_to_rad(-3.0f), deg_to_rad(3.5f)}},
        {{0.0f, deg_to_rad(8.0f), deg_to_rad(-3.9f)}},
        {{0.0f, deg_to_rad(-5.0f), deg_to_rad(3.9f)}},
    }};
    DiffStats opencl_worst;
    for (ModelFrame frame : {ModelFrame::MedModel, ModelFrame::SmallBigModel}) {
        ModelInputTransform transform(source_config, frame);
        for (const auto &rpy : rpy_cases) {
            transform.set_calibration(rpy[0], rpy[1], rpy[2]);
            float projection[9];
            transform.projection_matrix(projection);
            transform.nv12_to_yuv6_warped(
                source_nv12.data(), kSourceW, kSourceH, compact.data());
            double projection_opencl[9];
            for (int i = 0; i < 9; ++i)
                projection_opencl[i] = projection[i];
            warp_pack_opencl_ref(source_nv12.data(), kSourceW, kSourceH,
                                 projection_opencl, opencl.data());
            const DiffStats opencl_diff = diff_stats(compact, opencl);
            opencl_worst.mean = std::max(opencl_worst.mean, opencl_diff.mean);
            opencl_worst.inner_max =
                std::max(opencl_worst.inner_max, opencl_diff.inner_max);
        }
    }
    EXPECT_LT(opencl_worst.mean, 1.0) << "640x360 고정소수점 워프 평균 절대 오차(openpilot OpenCL 참조 대비)";
    EXPECT_LT(opencl_worst.inner_max, 8.0)
        << "640x360 고정소수점 워프 안쪽 최대 오차(openpilot OpenCL 참조 대비)";
}

/* NV21(MaixCAM2 카메라)은 크로마 바이트 순서만 NV12와 반대다. 같은 영상을 두
 * 순서로 넣고 set_chroma_vu를 맞추면 워프 결과(uint8, 4번 평면 U·5번 평면 V)가
 * 비트까지 같아야 한다. */
TEST(CalibrationEquivalence, Nv21ChromaOrder)
{
    constexpr int kW = 640, kH = 360;
    std::vector<uint8_t> nv12(kW * kH * 3 / 2);
    fill_nv12(nv12, kW, kH);
    std::vector<uint8_t> nv21 = nv12;
    for (size_t i = kW * kH; i + 1 < nv21.size(); i += 2)
        std::swap(nv21[i], nv21[i + 1]);

    AppConfig config;
    config.manual_pitch = deg_to_rad(1.0f);
    ModelInputTransform from_nv12(config);
    ModelInputTransform from_nv21(config);
    from_nv21.set_chroma_vu(true);
    std::vector<uint8_t> a(kYuv6Floats), b(kYuv6Floats), swapped(kYuv6Floats);
    from_nv12.nv12_to_yuv6_warped(nv12.data(), kW, kH, a.data());
    from_nv21.nv12_to_yuv6_warped(nv21.data(), kW, kH, b.data());
    ASSERT_EQ(a, b) << "NV21 + set_chroma_vu(true) == NV12";
    from_nv12.nv12_to_yuv6_warped(nv21.data(), kW, kH, swapped.data());
    const size_t plane = kYuv6Floats / 6;
    ASSERT_TRUE(std::equal(a.begin() + 4 * plane, a.begin() + 5 * plane, swapped.begin() + 5 * plane))
        << "순서를 안 맞추면 U와 V가 뒤바뀐다";
}

} // namespace

/* 카메라 장착 보정(sunnypilot camera offset): 가상 카메라가 offset만큼 오른쪽에 있을 때 가상 기준
 * 도로면 점의 모델 픽셀은, 실제 카메라에서 그 도로 점(가상 위치 + offset)이 찍히는 픽셀로 간다.
 * offset 0이면 기존 행렬과 같다. */
TEST(CalibrationEquivalence, CameraMountShiftsGroundPlane)
{
    AppConfig config;
    config.set_warp_source(1280, 720);
    const float roll = 0.004f, pitch = -0.012f, yaw = 0.009f;
    const float offset = 0.2f, height = 1.3f;
    for (const bool big : {false, true}) {
        ModelInputTransform plain(config, big ? ModelFrame::SmallBigModel : ModelFrame::MedModel);
        ModelInputTransform shifted(config, big ? ModelFrame::SmallBigModel : ModelFrame::MedModel);
        plain.set_calibration(roll, pitch, yaw);
        shifted.set_calibration(roll, pitch, yaw);
        shifted.set_camera_mount(offset, height);
        float p0[9], p1[9];
        plain.projection_matrix(p0);
        shifted.projection_matrix(p1);
        ModelInputTransform zero(config, big ? ModelFrame::SmallBigModel : ModelFrame::MedModel);
        zero.set_calibration(roll, pitch, yaw);
        zero.set_camera_mount(0.0f, height);
        float pz[9];
        zero.projection_matrix(pz);
        for (int i = 0; i < 9; ++i) EXPECT_FLOAT_EQ(pz[i], p0[i]) << "offset 0은 기존과 같다";

        const float mf = big ? 455.0f : 910.0f, mcx = 256.0f, mcy = big ? 0.5f * (256.0f + 47.6f) : 47.6f;
        float rot[9];
        rotation_from_rpy(roll, pitch, yaw, rot);
        for (const float y_right : {-1.5f, 0.0f, 1.8f}) {
            for (const float x_fwd : {8.0f, 20.0f, 45.0f}) {
                // 가상 카메라 기준 도로면 점(보정 좌표계: x 앞, y 오른쪽, z 아래 = 높이)
                const float pv[3] = {x_fwd, y_right, height};
                // 모델 픽셀: K_m · view · P (view: (y, z, x))
                const float u = mf * pv[1] / pv[0] + mcx, v = mf * pv[2] / pv[0] + mcy;
                const float w = p1[6] * u + p1[7] * v + p1[8];
                const float sx = (p1[0] * u + p1[1] * v + p1[2]) / w, sy = (p1[3] * u + p1[4] * v + p1[5]) / w;
                // 실제 카메라: 같은 도로 점은 P_v + (0, offset, 0)
                const float pr[3] = {pv[0], pv[1] + offset, pv[2]};
                float d[3];
                for (int r = 0; r < 3; ++r) d[r] = rot[r * 3] * pr[0] + rot[r * 3 + 1] * pr[1] + rot[r * 3 + 2] * pr[2];
                const float ex = config.input_warp_fx * d[1] / d[0] + config.input_warp_cx;
                const float ey = config.input_warp_fy * d[2] / d[0] + config.input_warp_cy;
                EXPECT_NEAR(sx, ex, 0.02f) << "big " << big << " x " << x_fwd << " y " << y_right;
                EXPECT_NEAR(sy, ey, 0.02f);
            }
        }
    }
}
