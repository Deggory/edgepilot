#include "planning/lateral_planner.h"

#include "common/ipc_messages.h"
#include "common/utils_math.h"
#include "controls/control_params.h"
#include "planning/desire_helper.h"
#include "planning/lane_line_filter.h"
#include "planning/lane_measurement.h"
#include "planning/lanebow_source.h"
#include "planning/lateral_mpc.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr double kDtModel = 0.05;
// plan 끝점이 이보다 가까우면 plan이 붕괴한 것(정지: 실측 5.6 m, 주행: 60 m+).
constexpr double kMinPlanReachM = 10.0;
// 경로 인계 램프(1/s). 차선을 버릴 때 0.5초, 되찾을 때 0.25초.
constexpr double kPlanMixOutRatePerS = 2.0;
constexpr double kPlanMixInRatePerS = 4.0;

double path_heading_at(const std::array<std::array<double, 3>, kTrajectorySize> &path, int index) {
    const int previous = std::max(0, index - 1);
    const int next = std::min(kTrajectorySize - 1, index + 1);
    if (previous == next) return 0.0;

    const double dx = path[next][0] - path[previous][0];
    const double dy = path[next][1] - path[previous][1];
    if (!std::isfinite(dx) || !std::isfinite(dy)) return 0.0;
    const double safe_dx = std::fabs(dx) < 1e-3 ? std::copysign(1e-3, dx == 0.0 ? 1.0 : dx) : dx;
    return std::atan2(dy, safe_dx);
}

class FirstOrderFilter {
public:
    FirstOrderFilter(double value, double rc) : value_(value), alpha_(kDtModel / (rc + kDtModel)) {}

    double update(double value) {
        value_ = (1.0 - alpha_) * value_ + alpha_ * value;
        return value_;
    }

    double value() const { return value_; }

private:
    double value_;
    double alpha_;
};

// supercombo 차선(ModelState lanes[1]·[2])을 시간 필터(planning/lane_line_filter)에 걸러 측정으로 낸다.
class ModelLaneSource {
public:
    void set_filter_weight(double weight) {
        left_filter_.set_measurement_weight(weight);
        right_filter_.set_measurement_weight(weight);
    }

    // v_ego와 곡률은 직전 프레임 뒤 자차 움직임이다.
    LaneMeasurement measure(const ModelState &model, double v_ego, double curvature) {
        const double t_s = static_cast<double>(model.capture_timestamp_ns) * 1e-9;
        std::array<float, kTrajectorySize> left_x{}, right_x{}, left{}, right{};
        LaneMeasurement m;
        for (int i = 0; i < kTrajectorySize; ++i) {
            m.t[i] = model.lane_t[i];
            m.x[i] = model.lanes[1][i].x;
            left_x[i] = model.lanes[1][i].x;
            right_x[i] = model.lanes[2][i].x;
            left[i] = model.lanes[1][i].y;
            right[i] = model.lanes[2][i].y;
        }
        left_filter_.update(t_s, left_x.data(), left.data(), v_ego, curvature, left.data());
        right_filter_.update(t_s, right_x.data(), right.data(), v_ego, curvature, right.data());
        for (int i = 0; i < kTrajectorySize; ++i) {
            m.left[i] = left[i];
            m.right[i] = right[i];
        }
        m.left_prob = model.lane_probabilities[1];
        m.right_prob = model.lane_probabilities[2];
        m.left_std = model.lane_stds[1];
        m.right_std = model.lane_stds[2];
        return m;
    }

private:
    LaneLineFilter left_filter_, right_filter_;
};

// 차선 측정에서 유효 확률, 차로 폭, 차선 중심 경로를 만든다(openpilot 0.8.x lane_planner). 출처마다 하나씩 둔다.
class LanePlanner {
public:
    explicit LanePlanner(double path_offset_m) : path_offset_m_(path_offset_m) {}

    void update_offsets(double path_offset_m) { path_offset_m_ = path_offset_m; }

    void set_measurement(const LaneMeasurement &m) {
        lane_t_ = m.t;
        lane_x_ = m.x;
        left_y_ = m.left;
        right_y_ = m.right;
        left_prob_ = m.left_prob;
        right_prob_ = m.right_prob;
        left_std_ = m.left_std;
        right_std_ = m.right_std;
        width_index_ = std::clamp(m.width_index, 0, kTrajectorySize - 1);
    }

    // 폭/std 보정을 반영한 유효 확률과 폭 추정을 갱신한다. 랜리스 모드에서도
    // 매 프레임 호출해야 폭 필터와 로그 값이 멈추지 않는다.
    void update_probabilities(float v_ego) {
        std::array<double, kTrajectorySize> width{};
        for (int i = 0; i < kTrajectorySize; ++i) width[i] = right_y_[i] - left_y_[i];

        double left_prob = left_prob_;
        double right_prob = right_prob_;
        // 폭 밴드를 상류 [4, 5]에서 0.5 m 올렸다. 국내 분기구간은 유도선이 차로
        // 한가운데를 지나 흰 차선 기준 폭이 4.2~5.0 m가 되는데, 상류 값에서는
        // 확률 0.95가 0.3 밑으로 깎여 laneless로 빠진다(2026-09-22 잠실대교 진입:
        // 전환 14.7회/분, 우측선 거리 요동 3.7배). 실주행 폭은 p99 4.69 m라
        // 5.5 m 위는 여전히 막는다.
        double width_mod = 1.0;
        for (double t : {0.0, 1.5, 3.0}) {
            const double lane_width = interp(t * (v_ego + 7.0), lane_x_.data(), width.data(), width.size());
            const double candidate = lane_width <= 4.5 ? 1.0 : lane_width >= 5.5 ? 0.0 : 5.5 - lane_width;
            width_mod = std::min(width_mod, candidate);
        }
        left_prob *= width_mod;
        right_prob *= width_mod;
        // 막 나타난 차선은 std가 높다. 컷오프를 0.3에서 0.4로 완화해
        // 재획득 직후에도 가중치가 일찍 차오르게 한다.
        const auto std_mod = [](double value) {
            return value <= 0.15 ? 1.0 : value >= 0.4 ? 0.0 : (0.4 - value) / 0.25;
        };
        left_prob *= std_mod(left_std_);
        right_prob *= std_mod(right_std_);
        // 차선 std는 std_mod 밴드(0.15~0.40)를 0.10초에 가로지른다(실측 2.43/s).
        // 그대로 쓰면 블렌드 가중치와 랜리스 전환이 같은 프레임에 통째로 뒤집힌다.
        // 둘의 공통 입력을 늦춘다. 평균 가중치는 바뀌지 않는다.
        const double prob_step = kProbRatePerS * kDtModel;
        left_prob_eff_ = std::clamp(left_prob, left_prob_eff_ - prob_step, left_prob_eff_ + prob_step);
        right_prob_eff_ = std::clamp(right_prob, right_prob_eff_ - prob_step, right_prob_eff_ + prob_step);
        left_prob = left_prob_eff_;
        right_prob = right_prob_eff_;

        const double certainty = left_prob * right_prob;
        lane_width_certainty_.update(certainty);
        // 양쪽이 다 보일 때만 폭을 학습한다. estimate는 certainty보다 시정수가 10배
        // 길어(약 10초 대 1초), 한 번 오염되면 재획득 직후 그대로 신뢰된다.
        if (certainty > kWidthLearnMinCertainty)
            lane_width_estimate_.update(std::fabs(right_y_[width_index_] - left_y_[width_index_]));
        const double speed_width =
            interp(static_cast<double>(v_ego), kLaneWidthSpeed.data(), kLaneWidth.data(), kLaneWidthSpeed.size());
        lane_width_ = lane_width_certainty_.value() * lane_width_estimate_.value() +
                      (1.0 - lane_width_certainty_.value()) * speed_width;
        d_prob_ = left_prob + right_prob - left_prob * right_prob;
    }

    std::array<std::array<double, 3>, kTrajectorySize> lane_path(
        const std::array<double, kTrajectorySize> &path_t,
        std::array<std::array<double, 3>, kTrajectorySize> path) const {
        const double half_width = std::min(4.0, lane_width_) * 0.5;
        const double denominator = left_prob_eff_ + right_prob_eff_ + 0.0001;

        // 경로 오프셋은 차선 중심에만 더한다(openpilot이 camera_offset을 차선선에만 더하듯).
        // 모델 경로는 차 기준이라 거기에 더하면 매 프레임 "지금 가려는 곳의 8 cm 옆"이 목표가
        // 되어 위치 고정점이 없다. 모델의 중앙 복원력이 약해(차선 1 m 벗어날 때 plan 0.28 m)
        // 교차로 랜리스 인계 때 차가 오프셋의 몇 배만큼 계속 밀렸다(2026-09-29 실차: 좌측 쏠림).
        // 이렇게 하면 실제 적용량은 차선 가중치 d_prob를 따르고, 모델 경로 구간으로 넘어가는 동안
        // 출력 블렌드(1 - plan_mix)만큼 더 줄어든다.
        std::array<double, kTrajectorySize> lane_path_y{};
        for (int i = 0; i < kTrajectorySize; ++i) {
            const double from_left = left_y_[i] + half_width;
            const double from_right = right_y_[i] - half_width;
            lane_path_y[i] = (left_prob_eff_ * from_left + right_prob_eff_ * from_right) / denominator + path_offset_m_;
        }

        std::array<double, kTrajectorySize> valid_t{};
        std::array<double, kTrajectorySize> valid_y{};
        size_t valid_count = 0;
        for (int i = 0; i < kTrajectorySize; ++i) {
            if (std::isfinite(lane_t_[i])) {
                valid_t[valid_count] = lane_t_[i];
                valid_y[valid_count] = lane_path_y[i];
                ++valid_count;
            }
        }
        if (valid_count > 0) {
            for (int i = 0; i < kTrajectorySize; ++i) {
                const double lane_y = interp(path_t[i], valid_t.data(), valid_y.data(), valid_count);
                path[i][1] = d_prob_ * lane_y + (1.0 - d_prob_) * path[i][1];
            }
        }
        return path;
    }

    // 랜리스 전환 판정값. 블렌드와 같은 유효 확률을 써서 두 판정이 어긋나지 않게 한다.
    double mean_effective_probability() const { return (left_prob_eff_ + right_prob_eff_) * 0.5; }
    // 로그/분석용 관측값 노출. left_prob()/right_prob()는 보정 전 모델 원값이다.
    double near_left_y() const { return left_y_[0]; }
    double near_right_y() const { return right_y_[0]; }
    double lane_width() const { return lane_width_; }
    double left_prob() const { return left_prob_; }
    double right_prob() const { return right_prob_; }
    double left_std() const { return left_std_; }
    double right_std() const { return right_std_; }
    double d_prob() const { return d_prob_; }
    void scale_near_probability(double scale) {
        left_prob_ *= scale;
        right_prob_ *= scale;
    }

private:
    static constexpr std::array<double, 2> kLaneWidthSpeed = {0.0, 31.0};
    static constexpr std::array<double, 2> kLaneWidth = {2.8, 3.5};
    static constexpr double kWidthLearnMinCertainty = 0.25;
    // 유효 확률 변화율 상한(1/s). 0→1에 0.5초.
    static constexpr double kProbRatePerS = 2.0;
    std::array<double, kTrajectorySize> lane_t_{};
    std::array<double, kTrajectorySize> lane_x_{};
    std::array<double, kTrajectorySize> left_y_{};
    std::array<double, kTrajectorySize> right_y_{};
    FirstOrderFilter lane_width_estimate_{3.7, 9.95};
    FirstOrderFilter lane_width_certainty_{1.0, 0.95};
    double left_prob_ = 0.0;
    double right_prob_ = 0.0;
    double left_prob_eff_ = 0.0;
    double right_prob_eff_ = 0.0;
    double left_std_ = 0.0;
    double right_std_ = 0.0;
    double lane_width_ = 3.7;
    double d_prob_ = 0.0;
    double path_offset_m_ = 0.0;
    int width_index_ = 0;
};

// MPC 노드 시각의 기준 횡위치와 heading.
struct MpcReferences {
    std::array<double, kLatMpcNodes> y{};
    std::array<double, kLatMpcNodes> heading{};
};

// 경로(차선 융합 뒤)에서 MPC 노드 시각의 목표를 차속 x 시간 거리로 보간한다. knot을 직접
// 인덱싱하면 모델 knot의 프레임 간 노이즈가 그대로 들어가 des가 2~3배 떨리고 토크 슬루 리미터가
// 요구 토크의 절반을 잘라낸다(0.8.x 재생 실측).
MpcReferences mpc_references(const std::array<std::array<double, 3>, kTrajectorySize> &path,
                             const std::array<double, kTrajectorySize> &path_t, float v_ego) {
    std::array<double, kTrajectorySize> distance{};
    std::array<double, kTrajectorySize> path_y{};
    std::array<double, kTrajectorySize> path_heading{};
    const bool plan_collapsed = path[kTrajectorySize - 1][0] < kMinPlanReachM;
    for (int i = 0; i < kTrajectorySize; ++i) {
        distance[i] = std::sqrt(path[i][0] * path[i][0] + path[i][1] * path[i][1] + path[i][2] * path[i][2]);
        path_y[i] = path[i][1];
        // 최종 경로와 heading을 같은 좌표계에서 계산한다. 차선 융합이나
        // 경로 오프셋 뒤에 모델 원본 heading을 재사용하면 좌우 곡률 부호가
        // 서로 달라져 한쪽 커브에서 경로를 안쪽으로 자를 수 있다.
        path_heading[i] = path_heading_at(path, i);
        // 정지 부근에서는 plan 전체가 몇 m로 붕괴해 기하학적 heading이 dy를
        // ±90도로 부풀리고, 양자화된 모델은 먼 knot이 0.5 m 이상 뒤로도 뛴다
        // (±180도 → 출발 시 좌측 급조향). 차가 못 움직인 구간이라 목표
        // heading은 현재 방위(0)가 맞다. 주행 중에도 역방향 step은 물리적으로
        // 불가능한 기하이므로 0으로 둔다.
        const int prev = std::max(0, i - 1), next = std::min(kTrajectorySize - 1, i + 1);
        if (plan_collapsed || path[next][0] <= path[prev][0]) path_heading[i] = 0.0;
    }
    make_monotonic(distance.data(), distance.size());

    MpcReferences references;
    for (int i = 0; i < kLatMpcNodes; ++i) {
        const double query = std::max(0.0f, v_ego) * path_t[i];
        references.y[i] = interp(query, distance.data(), path_y.data(), distance.size());
        references.heading[i] = interp(query, distance.data(), path_heading.data(), distance.size());
    }
    return references;
}

}  // namespace

struct LateralPlanner::Impl {
    explicit Impl(const SteeringParams &steering) : sc_lanes(steering.path_offset_m), ld_lanes(steering.path_offset_m) {
        update_params(steering);
    }

    void update_params(const SteeringParams &steering) {
        sc_lanes.update_offsets(steering.path_offset_m);
        ld_lanes.update_offsets(steering.path_offset_m);
        lane_path_weight = steering.lane_path_weight;
        DesireHelperParams desire_params;
        // desire_helper의 torque_applied는 carstate.steeringPressed에서 나오므로
        // 컨트롤러와 같은 임계값을 써야 한다.
        desire_params.steering_pressed_threshold = steering.steering_pressed_threshold;
        desire_params.lane_change_min_speed_mps = steering.lane_change_min_speed_kph / 3.6;
        desire_helper.update_params(desire_params);
        path_mode = steering.path();
        const double center_to_front = steering.center_to_front_m();
        constexpr double civic_mass = 1326.0 + 136.0;
        constexpr double civic_wheelbase = 2.70;
        constexpr double civic_center_to_front = civic_wheelbase * 0.4;
        double tire_rear = 202500.0 * steering.tire_stiffness_factor * steering.mass_kg / civic_mass;
        tire_rear *= (center_to_front / steering.wheelbase_m) / (civic_center_to_front / civic_wheelbase);
        factor1 = steering.wheelbase_m - center_to_front;
        factor2 = center_to_front * steering.mass_kg / (steering.wheelbase_m * tire_rear);
    }

    LateralTarget update(const ModelState &model, const LaneState *lane, const VehicleCanState &vehicle, float v_ego,
                         float measured_curvature, bool lat_active, bool steering_pressed) {
        LateralTarget target;
        if (!model.valid) return target;

        // 두 출처를 매 프레임 갱신해 모드를 바꿔도 필터·폭 상태가 데워져 있다. 경로는 path_mode가 고른 쪽이 낸다.
        sc_lanes.set_measurement(sc_source.measure(model, v_ego, measured_curvature));
        ld_lanes.set_measurement(ld_source.measure(lane, model, v_ego, measured_curvature));
        update_desire(model, vehicle, v_ego, lat_active, steering_pressed);
        std::array<std::array<double, 3>, kTrajectorySize> path{};
        std::array<double, kTrajectorySize> path_t{};
        for (int i = 0; i < kTrajectorySize; ++i) {
            path[i] = {model.plan[i].x, model.plan[i].y, model.plan[i].z};
            path_t[i] = model.model_t[i];
        }

        sc_lanes.update_probabilities(v_ego);
        ld_lanes.update_probabilities(v_ego);
        if (path_mode == PathMode::Laneless) return upstream_target(model, v_ego, measured_curvature, sc_lanes);
        // 여기부터는 Lane·Lanebow 모드다(laneless 모드는 위에서 upstream_target으로 끝난다). 둘은 차선 출처만 다르다.
        const bool lanebow = path_mode == PathMode::Lanebow;
        const LanePlanner &lanes = lanebow ? ld_lanes : sc_lanes;
        const bool use_model_path = update_path_source(lanes);
        const MpcReferences references = mpc_references(lanes.lane_path(path_t, path), path_t, v_ego);
        solve_lane_mpc(references, v_ego, measured_curvature, path_t);

        // 모델 경로 쪽 목표. 섞는 동안 MPC는 실제로 낸 곡률에서 다음 해를 시작해 되돌아올 때 튀지 않는다.
        const double model_weight = plan_mix;
        LateralTarget model_target;
        if (model_weight > 0.0) {
            model_target = plan_yaw_target(model, v_ego, lanes);
            std::array<double, kLateralControlN> times{}, model_curvatures{};
            for (int i = 0; i < kLateralControlN; ++i) {
                times[i] = model_t_idx_double(i);
                model_curvatures[i] = model_target.curvatures[i];
            }
            initial_curvature = (1.0 - model_weight) * initial_curvature +
                                model_weight * interp(kDtModel, times.data(), model_curvatures.data(), times.size());
        }

        target.valid = true;
        target.capture_timestamp_ns = model.capture_timestamp_ns;
        target.mpc_solution_valid = model_weight >= 1.0 || invalid_count < 2;
        target.laneless_mode = use_model_path;
        fill_lane_observations(lanes, &target);
        target.lane_d_prob = static_cast<float>(use_model_path ? 0.0 : lanes.d_prob());
        target.target_y_m = static_cast<float>(references.y[1]);
        target.heading_rad = static_cast<float>(mpc.nodes()[0].psi);
        target.curvature = static_cast<float>(mpc.nodes()[0].curvature);
        fill_desire(&target);
        for (int i = 0; i < kLateralControlN; ++i) {
            target.psis[i] = static_cast<float>(mpc.nodes()[i].psi);
            target.curvatures[i] = static_cast<float>(mpc.nodes()[i].curvature);
        }
        if (model_weight > 0.0) {
            // lag_adjusted_curvature는 psi와 곡률에 선형이라 입력을 섞으면 목표 곡률이 그대로 섞인다.
            const auto mix = [model_weight](float lane_value, float model_value) {
                return static_cast<float>((1.0 - model_weight) * lane_value + model_weight * model_value);
            };
            for (int i = 0; i < kLateralControlN; ++i) {
                target.psis[i] = mix(target.psis[i], model_target.psis[i]);
                target.curvatures[i] = mix(target.curvatures[i], model_target.curvatures[i]);
            }
            target.heading_rad = mix(target.heading_rad, model_target.heading_rad);
            target.curvature = mix(target.curvature, model_target.curvature);
            target.target_y_m = mix(target.target_y_m, model_target.target_y_m);
        }
        return target;
    }

    // 모델 프레임의 차선 변경 확률·차선·경계로 desire를 갱신하고, 변경 중이면 가까운 차선선을 뺀다.
    void update_desire(const ModelState &model, const VehicleCanState &vehicle, float v_ego, bool lat_active,
                       bool steering_pressed) {
        DesireModelInputs desire_inputs;
        desire_inputs.lane_change_prob = model.desire_state[3] + model.desire_state[4];
        for (size_t i = 0; i < desire_inputs.lane_probabilities.size(); ++i)
            desire_inputs.lane_probabilities[i] = model.lane_probabilities[i];
        for (size_t i = 0; i < desire_inputs.road_edge_stds.size(); ++i)
            desire_inputs.road_edge_stds[i] = model.road_edge_stds[i];
        desire_helper.update(vehicle, v_ego, lat_active, steering_pressed, desire_inputs);
        for (LanePlanner *lanes : {&sc_lanes, &ld_lanes}) {
            if (desire_helper.changing_lanes()) lanes->scale_near_probability(desire_helper.lane_change_lane_prob());
            // 회전 중에는 차선선을 빼 Lane 모드에서도 회전 desire를 받은 모델 경로를 따른다(교차로는 원래 차선이
            // 흐리다)
            if (desire_helper.turning()) lanes->scale_near_probability(0.0);
        }
    }

    // Lane 모드에서 이번 프레임에 모델 경로를 따를지(차선이 안 보일 때) 정하고, 출력 인계 비율 plan_mix를
    // 그쪽으로 옮긴다.
    bool update_path_source(const LanePlanner &lanes) {
        const double lane_probability = lanes.mean_effective_probability();
        bool use_model_path = false;
        const bool lane_change_off = desire_helper.lane_change_state() == LaneChangeState::Off;
        if (lane_probability < 0.3 && lane_change_off) {
            use_model_path = true;
            laneless_buffer = true;
            // 복귀 문턱을 openpilot의 0.5에서 0.4로 내렸다. 교차로 후 차선
            // 재획득이 빨라지고, 진입(0.3)과의 밴드가 남아 채터링은 없다.
        } else if (lane_probability > 0.4 && laneless_buffer && lane_change_off) {
            laneless_buffer = false;
        } else if (laneless_buffer && lane_change_off) {
            use_model_path = true;
        } else if (!lane_change_off) {
            laneless_buffer = false;
        }
        // 모델 경로 구간(교차로처럼 차선이 안 보일 때)은 laneless 모드와 같은 openpilot
        // 메인 계산(plan_yaw_target)으로 목표를 낸다. 예전에는 모델 경로 위치를 Lane MPC로 따라갔는데,
        // 경로를 "지금 속도 x 시간" 거리에서 읽고 MPC가 곡률 변화를 눌러 25 km/h 아래에서 laneless보다
        // 26~30% 덜 꺾고 0.3~0.4초 늦었다(2026-10-05 재생; 실주행 저속 회전의 운전자 토크 중앙값
        // 152 vs laneless 80). MPC는 늘 차선 경로를 따르고, 둘 사이의 인계만 출력(psi·곡률)에서 섞는다.
        // 전환 임계값(유효확률 0.3)에서는 차선 블렌드 가중치가 아직 0.51이라 한 프레임에 바꾸면 튄다
        // (2026-09-13 실측, 전환 434회). 차선을 버리는 방향은 0.5초, 되찾는 방향은 0.25초에 섞는다.
        const double mix_target = use_model_path ? 1.0 : 0.0;
        const double mix_step = (mix_target > plan_mix ? kPlanMixOutRatePerS : kPlanMixInRatePerS) * kDtModel;
        plan_mix = std::clamp(mix_target, plan_mix - mix_step, plan_mix + mix_step);
        return use_model_path;
    }

    // 차선 MPC를 references로 풀고 다음 사이클의 초기 곡률과 연속 실패 수(invalid_count)를 갱신한다.
    // 풀이가 실패하면 MPC를 비우고 실제 곡률에서 다시 시작한다.
    void solve_lane_mpc(const MpcReferences &references, float v_ego, float measured_curvature,
                        const std::array<double, kTrajectorySize> &path_t) {
        const double lateral_factor = std::max(0.0, factor1 - factor2 * v_ego * v_ego);
        // lane 모드도 laneless와 같은 스케줄. heading은 차선 경로에서 나오므로
        // 고속에서 heading 고정 1.0이면 횡 offset에 대한 DC 강성이 0이 되어
        // 커브에서 바깥쪽 0.3~0.5m 평형이 생긴다(2026-08-25 실측: 요구곡률
        // 3.4% 부족 + 바깥 offset +0.375m).
        const double heading_weight = v_ego <= 5.0f ? 1.0 : v_ego >= 10.0f ? 0.15 : 1.0 - (v_ego - 5.0) * 0.17;
        LateralMpcWeights weights;
        weights.path = lane_path_weight;
        weights.heading = heading_weight;
        mpc.run(initial_curvature, std::max(0.0f, v_ego), lateral_factor, references.y, references.heading, weights);
        const bool solver_failed = mpc.status() != 0;
        if (solver_failed) {
            mpc.reset();
            initial_curvature = measured_curvature;
        } else {
            std::array<double, kLatMpcNodes> curvatures{};
            for (int i = 0; i < kLatMpcNodes; ++i) curvatures[i] = mpc.nodes()[i].curvature;
            initial_curvature = interp(kDtModel, path_t.data(), curvatures.data(), curvatures.size());
        }
        invalid_count = (mpc.cost() > 20000.0 || solver_failed) ? invalid_count + 1 : 0;
    }

    // 차선 관측값(로그·HUD용): 가까운 좌우 차선, 폭, 출처의 원 확률과 std. Lanebow에서는 쓴 LD의 frame_id와 나이도.
    void fill_lane_observations(const LanePlanner &lanes, LateralTarget *target) const {
        if (&lanes == &ld_lanes) {
            target->lane_age_s = static_cast<float>(ld_source.lane_age_s());
            target->lane_frame_id = ld_source.lane_frame_id();
        }
        target->lane_left_y_m = static_cast<float>(lanes.near_left_y());
        target->lane_right_y_m = static_cast<float>(lanes.near_right_y());
        target->lane_width_m = static_cast<float>(lanes.lane_width());
        target->lane_left_prob = static_cast<float>(lanes.left_prob());
        target->lane_right_prob = static_cast<float>(lanes.right_prob());
        target->lane_left_std = static_cast<float>(lanes.left_std());
        target->lane_right_std = static_cast<float>(lanes.right_std());
    }

    // laneless 모드 = openpilot 메인의 get_curvature_from_plan. 차선·MPC·경로 오프셋 없이
    // 모델 plan의 yaw와 yaw rate를 그대로 목표로 넘긴다. 컨트롤러의 lag_adjusted_curvature가
    // 같은 식 2·ψ(t_d)/(v·t_d) − ψ̇(0)/v를 t_d = 조향 지연 + 실측 plan 나이에서 적용한다
    // (메인은 t_d = lateralDelay + 프레임 지연 50 ms + 25 ms). 경로 오프셋은 laneless에서
    // 위치가 아니라 꾸준한 곡률 편향이 되므로 쓰지 않는다.
    LateralTarget upstream_target(const ModelState &model, float v_ego, float measured_curvature,
                                  const LanePlanner &lanes) {
        LateralTarget target = plan_yaw_target(model, v_ego, lanes);
        // 차선 모드로 돌아가면 낡은 MPC 해가 아니라 지금 곡률에서 출발한다.
        laneless_buffer = false;
        plan_mix = 1.0;
        mpc.reset();
        initial_curvature = measured_curvature;
        invalid_count = 0;
        return target;
    }

    // openpilot 메인 get_curvature_from_plan 목표. 상태를 바꾸지 않아 Lane 모드의 모델 경로 구간도 쓴다. 차선
    // 관측값(로그)은 lanes에서.
    LateralTarget plan_yaw_target(const ModelState &model, float v_ego, const LanePlanner &lanes) const {
        constexpr double kMinSpeed = 1.0;  // openpilot drive_helpers.MIN_SPEED
        const double speed = std::max<double>(v_ego, kMinSpeed);
        std::array<double, kTrajectorySize> t{}, yaw{}, yaw_rate{}, y{};
        for (int i = 0; i < kTrajectorySize; ++i) {
            t[i] = model.model_t[i];
            yaw[i] = model.plan_yaw[i];
            yaw_rate[i] = model.plan_yaw_rate[i];
            y[i] = model.plan[i].y;
        }
        make_monotonic(t.data(), t.size());
        LateralTarget target;
        for (int i = 0; i < kLateralControlN; ++i) {
            const double ti = model_t_idx_double(i);
            target.psis[i] = static_cast<float>(interp(ti, t.data(), yaw.data(), t.size()));
            target.curvatures[i] = static_cast<float>(interp(ti, t.data(), yaw_rate.data(), t.size()) / speed);
        }
        target.valid = true;
        target.capture_timestamp_ns = model.capture_timestamp_ns;
        target.mpc_solution_valid = true;  // MPC를 쓰지 않는다
        target.laneless_mode = true;
        fill_lane_observations(lanes, &target);
        target.lane_d_prob = 0.0f;
        target.target_y_m = static_cast<float>(interp(model_t_idx_double(1), t.data(), y.data(), t.size()));
        target.heading_rad = target.psis[0];
        target.curvature = target.curvatures[0];
        fill_desire(&target);
        return target;
    }

    // 이번 프레임의 desire와 차선 변경 단계(LateralTarget의 정수 필드로).
    void fill_desire(LateralTarget *target) const {
        target->desire = static_cast<int>(desire_helper.desire());
        target->lane_change_state = static_cast<int>(desire_helper.lane_change_state());
        target->lane_change_direction = desire_helper.direction();
    }

    ModelLaneSource sc_source;
    LanePlanner sc_lanes;
    LanebowSource ld_source;
    LanePlanner ld_lanes;
    LateralMpc mpc;
    // 다음 사이클의 초기 curvature. 나머지 초기 상태는 자차 기준 0이다.
    double initial_curvature = 0.0;
    double factor1 = 0.0;
    double factor2 = 0.0;
    double lane_path_weight = 3.0;
    PathMode path_mode = PathMode::Lane;
    DesireHelper desire_helper;
    bool laneless_buffer = false;
    // 0 = 차선 융합 경로, 1 = 모델 플랜. 전환 판정을 그대로 따라가되 램프로 움직인다.
    double plan_mix = 1.0;
    int invalid_count = 0;
};

LateralPlanner::LateralPlanner(const SteeringParams &params) : impl_(std::make_unique<Impl>(params)) {}

LateralPlanner::~LateralPlanner() = default;

void LateralPlanner::update_params(const SteeringParams &params) { impl_->update_params(params); }

void LateralPlanner::set_lane_filter_weight(double weight) {
    impl_->sc_source.set_filter_weight(weight);
    impl_->ld_source.set_filter_weight(weight);
}

LateralTarget LateralPlanner::update(const ModelState &model, const LaneState *lane, const VehicleCanState &vehicle,
                                     float v_ego, float measured_curvature, bool lat_active, bool steering_pressed) {
    return impl_->update(model, lane, vehicle, v_ego, measured_curvature, lat_active, steering_pressed);
}
