#include "model/model_state_fill.h"

#include "common/utils_time.h"

#include <algorithm>
#include <cmath>

ModelState fill_model_state(const ParsedModelOutput &parsed, const ProjectionState &projection,
                            const OnlineCalibrator::Snapshot &calibration, uint64_t frame_id,
                            uint64_t capture_timestamp_ns, float model_execution_ms) {
    ModelState state;
    state.frame_id = frame_id;
    state.capture_timestamp_ns = capture_timestamp_ns;
    state.model_timestamp_ns = monotonic_now_ns();
    state.model_execution_ms = model_execution_ms;
    state.valid = parsed.valid ? 1 : 0;

    for (int i = 0; i < kTrajectorySize; ++i) {
        state.plan[i] = {parsed.plan.points[i].x, parsed.plan.points[i].y, parsed.plan.points[i].z};
        state.plan_yaw[i] = parsed.plan.yaw[i];
        state.plan_yaw_rate[i] = parsed.plan.yaw_rate[i];
        for (int lane = 0; lane < 4; ++lane) {
            state.lanes[lane][i] = {parsed.lanes[lane].points[i].y, parsed.lanes[lane].points[i].z};
        }
        for (int edge = 0; edge < 2; ++edge) {
            state.road_edges[edge][i] = {parsed.road_edges[edge].points[i].y, parsed.road_edges[edge].points[i].z};
        }
    }
    for (int lane = 0; lane < 4; ++lane) {
        state.lane_probabilities[lane] = parsed.lanes[lane].probability;
        state.lane_stds[lane] = parsed.lanes[lane].std;
    }
    for (int edge = 0; edge < 2; ++edge) state.road_edge_stds[edge] = parsed.road_edges[edge].std;
    for (int i = 0; i < kDesireLen; ++i) state.desire_state[i] = parsed.meta.desire_state[i];
    for (int i = 0; i < kMetaPressHorizons; ++i) state.gas_press_probs[i] = parsed.meta.gas_press[i];

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

    state.pose.valid = parsed.valid ? 1 : 0;
    for (int i = 0; i < 3; ++i) {
        state.pose.trans[i] = parsed.pose.trans[i];
        state.pose.rot[i] = parsed.pose.rot[i];
        state.pose.trans_std[i] = parsed.pose.trans_std[i];
        state.pose.rot_std[i] = parsed.pose.rot_std[i];
    }

    state.calibration.status = static_cast<uint32_t>(calibration.status);
    state.calibration.valid_blocks = calibration.valid_blocks;
    state.calibration.roll = projection.roll;
    state.calibration.pitch = projection.pitch;
    state.calibration.yaw = projection.yaw;
    for (int i = 0; i < 3; ++i) state.calibration.spread[i] = calibration.spread[i];
    return state;
}
