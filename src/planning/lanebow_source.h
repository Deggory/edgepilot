#ifndef PLANNING_LANEBOW_SOURCE_H
#define PLANNING_LANEBOW_SOURCE_H

// Lanebow 모드의 차선 출처(docs/lanebow.md). 차선선 플러그인(LD)이 낸 자차 좌우 선(LaneState 도로면 블록, 안쪽
// 경계)을 횡 플래너의 차선 측정(LaneMeasurement)으로 바꾼다.
//
// - 새 LD 프레임마다 형상을 검사하고(왼선 < 0 < 오른선이고 차에서 kMaxEgoOffsetM 안, 둘이면 폭 2 m 이상에 20 m까지
//   엇갈리지 않음) LD 촬영 시각으로 선을 시간 필터에 거른다. 같은 LaneState가 다시 오면 다시 섞지 않는다.
// - 플래너 프레임마다 거른 선을 모델 촬영 시각까지 자차 움직임만큼 옮겨(LD는 보통 한 프레임 늦다) 33점 격자로
//   읽는다. LD 시야 너머는 두 선이 같이 쓰는 모양(supercombo 두 선의 모양을 그 확률만큼, 나머지는 plan)으로 이어,
//   시야 너머의 폭은 시야 끝의 LD 폭 그대로다. 선마다 다른 모양을 빌리면 먼 곳에서 벌어지거나 엇갈려 차선 코어의
//   폭 검사가 멀쩡한 LD 선의 확률을 깎았다(2026-10-15 리허설: 짧은 laneless 전환의 대부분).
// - 확률은 신뢰도와 시야 길이로 정한다. 선이 잠깐 빠지면 확률을 LD 시각으로 kHoldS 동안 유지하고, 그 뒤 0이 되면
//   차선 코어의 변화율 제한이 내린다. 그동안 선은 예측으로 유한하게 둔다. 한쪽 선이 아예 없으면 그 자리는 남은 선을
//   kDefaultWidthM만큼 옮긴 선이다(확률 0이라 경로에는 쓰이지 않고, 폭 검사가 엉뚱한 supercombo 선을 보지 않는다).
// LD가 없으면 확률이 0이라 플래너는 Lane 모드의 대체 경로(Laneless 법칙)로 간다.

#include "common/ipc_messages.h"
#include "planning/lane_line_filter.h"
#include "planning/lane_measurement.h"

#include <array>
#include <cstdint>

class LanebowSource {
public:
    // 선이 빠져도 확률을 유지하는 시간(LD 시각으로 두 프레임 남짓)과, 거른 선을 앞으로 옮겨 쓰는 최대 시간
    static constexpr double kHoldS = 0.12;
    static constexpr double kPredictS = 1.0;
    // 확률 = conf_factor(신뢰도) x reach_factor(시야 끝 거리): 신뢰도 kConfidence0..1, 시야 kReach0..1(m)에서 0→1
    static constexpr double kConfidence0 = 0.15, kConfidence1 = 0.4;
    static constexpr double kReach0M = 10.0, kReach1M = 20.0;
    // 형상 검사: x = 0에서 자차 선이 차에서 떨어질 수 있는 거리(옆 차로의 선은 차가 제 선 가까이 있어도 3.6 m 넘게
    // 떨어진다: 한 줄만 보일 때 옆 차로 선을 자차 선으로 잘못 붙여도 차로 하나만큼 튀지 않게), 두 선 사이 최소 폭,
    // 엇갈림을 보는 범위
    static constexpr double kMaxEgoOffsetM = 3.6;
    static constexpr double kMinWidthM = 2.0;
    static constexpr double kCrossCheckM = 20.0;
    // 시야 너머 잇기: 시야 끝에서 이 길이 안의 차이로 오프셋을 맞추고 그 안에서 서서히 바꾼다
    static constexpr double kBlendM = 10.0;
    // 한쪽 선만 있을 때 반대쪽 자리 표시의 폭(폭 검사만 보는 값이라 정확할 필요가 없다)
    static constexpr double kDefaultWidthM = 3.5;
    // 차로 폭을 배울 격자(X_IDXS[6] = 6.75 m, LD가 늘 측정하는 가장 가까운 쪽)
    static constexpr int kWidthIndex = 6;

    void set_filter_weight(double weight);

    // lane: 이번 플래너 프레임에 고른 LaneState(lane_state_selector, 없으면 nullptr). model: 이번 프레임의
    // ModelState(plan, supercombo 차선과 확률, lane_t). v_ego·curvature: 직전 틱의 자차 속도와 실측 곡률.
    LaneMeasurement measure(const LaneState *lane, const ModelState &model, double v_ego, double curvature);

    // 진단: 마지막 measure가 쓴 선(확률 > 0)의 가장 새 LD frame_id와 그 나이(s, 모델 촬영 시각 - 그 LD 촬영 시각).
    // 쓴 선이 없으면 0과 음수.
    double lane_age_s() const { return lane_age_s_; }
    uint64_t lane_frame_id() const { return lane_frame_id_; }

private:
    struct Side {
        LaneLineFilter filter;
        double seen_t_s = -1e9;   // 마지막으로 이 선을 받은 LD 촬영 시각
        uint64_t seen_frame = 0;  // 그 LD의 frame_id
        double seen_prob = 0.0;   // 그때의 확률
    };

    void take_frame(const LaneState &lane, double v_ego, double curvature);
    // 거른 선을 t_s로 옮겨 33점으로, 시야 너머는 shape(두 선 공통 모양)에 맞춰 잇는다. 쓸 수 없으면 false(out은 그대로).
    bool line_at(const Side &side, double t_s, double v_ego, double curvature,
                 const std::array<double, kTrajectorySize> &shape, std::array<double, kTrajectorySize> *out) const;

    Side side_[2];
    uint64_t taken_frame_ = 0;  // 마지막으로 거른 LD frame_id(같은 것을 두 번 섞지 않게)
    bool have_frame_ = false;
    uint64_t lane_frame_id_ = 0;
    double lane_age_s_ = -1.0;
};

#endif
