#include "model_state_fill.h"

#include "utils_time.h"

#include <algorithm>
#include <cmath>

/* openpilot의 plan→lane_t: 차선 점 x(X_IDXS)마다 plan이 그 거리에 닿는 시각. plan 끝을 넘는 차선 점은
 * NaN이다(플래너가 보간에서 뺀다). */
void compute_lane_t(const ParsedPlan &plan, float lane_t[kTrajectorySize])
{
    std::fill_n(lane_t, kTrajectorySize, NAN);
    lane_t[0] = 0.0f;
    for (int xidx = 1, tidx = 0; xidx < kTrajectorySize; ++xidx) {
        for (int next_tid = tidx + 1;
             next_tid < kTrajectorySize && plan.points[next_tid].x < model_x_idx_double(xidx);
             ++next_tid) {
            ++tidx;
        }
        if (tidx == kTrajectorySize - 1) {
            lane_t[xidx] = model_t_idx(kTrajectorySize - 1);
            break;
        }

        const double current_x = plan.points[tidx].x;
        const double next_x = plan.points[tidx + 1].x;
        // 정차 부근의 양자화된 plan은 knot이 뒤로 뛴다. 보정 없이 두면 p가
        // 발산하거나 NaN이 되어 lane_t가 비단조가 된다. p를 [0,1]로 묶으면
        // lane_t는 항상 두 knot 시각 사이에 들어가 단조성이 보장된다.
        const double span = next_x - current_x;
        const double p = span > 0.0
            ? std::clamp((model_x_idx_double(xidx) - current_x) / span, 0.0, 1.0)
            : 1.0;
        lane_t[xidx] = static_cast<float>(
            p * model_t_idx_double(tidx + 1) + (1.0 - p) * model_t_idx_double(tidx));
    }
}

void fill_model_state(ModelState &state, const ParsedModelOutput &parsed,
                           const ProjectionState &projection,
                           const OnlineCalibrator::Snapshot &calibration,
                           uint64_t frame_id, uint64_t capture_timestamp_ns,
                           float model_execution_ms)
{
    state = ModelState{};
    state.frame_id = frame_id;
    state.capture_timestamp_ns = capture_timestamp_ns;
    state.model_timestamp_ns = monotonic_now_ns();
    state.model_execution_ms = model_execution_ms;
    state.valid = parsed.valid ? 1 : 0;
    state.best_plan = parsed.plan.best_index;
    state.plan_probability = parsed.plan.probability;

    for (int i = 0; i < kTrajectorySize; ++i) {
        state.model_t[i] = model_t_idx(i);
        state.plan[i] = {parsed.plan.points[i].x, parsed.plan.points[i].y, parsed.plan.points[i].z};
        state.plan_yaw[i] = parsed.plan.yaw[i];
        state.plan_yaw_rate[i] = parsed.plan.yaw_rate[i];
        for (int lane = 0; lane < 4; ++lane) {
            state.lanes[lane][i] = {
                parsed.lanes[lane].points[i].x,
                parsed.lanes[lane].points[i].y,
                parsed.lanes[lane].points[i].z,
            };
        }
        for (int edge = 0; edge < 2; ++edge) {
            state.road_edges[edge][i] = {
                parsed.road_edges[edge].points[i].x,
                parsed.road_edges[edge].points[i].y,
                parsed.road_edges[edge].points[i].z,
            };
        }
    }
    compute_lane_t(parsed.plan, state.lane_t);
    for (int lane = 0; lane < 4; ++lane) {
        state.lane_probabilities[lane] = parsed.lanes[lane].probability;
        state.lane_stds[lane] = parsed.lanes[lane].std;
    }
    for (int edge = 0; edge < 2; ++edge)
        state.road_edge_stds[edge] = parsed.road_edges[edge].std;
    for (int i = 0; i < kDesireLen; ++i)
        state.desire_state[i] = parsed.meta.desire_state[i];
    for (int i = 0; i < kMetaPressHorizons; ++i) {
        state.gas_press_probs[i] = parsed.meta.gas_press[i];
        state.brake_press_probs[i] = parsed.meta.brake_press[i];
    }

    ParsedLeadPoint lead;
    float lead_prob = 0.0f;
    if (parsed.leads.primary(0, 0.0f, &lead, &lead_prob)) {
        state.lead.valid = 1;
        state.lead.probability = lead_prob;
        state.lead.x = lead.x;
        state.lead.y = lead.y;
        state.lead.velocity = lead.velocity;
        state.lead.acceleration = lead.acceleration;
    }

    state.pose.valid = parsed.has_pose ? 1 : 0;
    if (parsed.has_pose) {
        for (int i = 0; i < 3; ++i) {
            state.pose.trans[i] = parsed.pose.trans[i];
            state.pose.rot[i] = parsed.pose.rot[i];
            state.pose.trans_std[i] = parsed.pose.trans_std[i];
            state.pose.rot_std[i] = parsed.pose.rot_std[i];
        }
    }

    state.calibration.status = static_cast<uint32_t>(calibration.status);
    state.calibration.valid_blocks = calibration.valid_blocks;
    state.calibration.roll = projection.roll;
    state.calibration.pitch = projection.pitch;
    state.calibration.yaw = projection.yaw;
    for (int i = 0; i < 3; ++i)
        state.calibration.spread[i] = calibration.spread[i];
}
