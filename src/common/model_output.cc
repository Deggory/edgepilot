#include "common/model_output.h"

#include <algorithm>
#include <cmath>

using namespace model_output_layout;

namespace {

float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

void softmax(const float *input, float *output, int size) {
    const float max_value = *std::max_element(input, input + size);
    float denominator = 0.0f;
    for (int i = 0; i < size; ++i) {
        output[i] = std::exp(input[i] - max_value);
        denominator += output[i];
    }
    const float inv_denominator = 1.0f / denominator;
    for (int i = 0; i < size; ++i) output[i] *= inv_denominator;
}

}  // namespace

void compute_lane_t(const float plan_x[kTrajectorySize], float lane_t[kTrajectorySize]) {
    std::fill_n(lane_t, kTrajectorySize, NAN);
    lane_t[0] = 0.0f;
    for (int xidx = 1, tidx = 0; xidx < kTrajectorySize; ++xidx) {
        for (int next_tid = tidx + 1; next_tid < kTrajectorySize && plan_x[next_tid] < model_x_idx_double(xidx);
             ++next_tid) {
            ++tidx;
        }
        if (tidx == kTrajectorySize - 1) {
            lane_t[xidx] = model_t_idx(kTrajectorySize - 1);
            break;
        }

        const double current_x = plan_x[tidx];
        const double next_x = plan_x[tidx + 1];
        // 정차 부근의 양자화된 plan은 knot이 뒤로 뛴다. 보정 없이 두면 p가
        // 발산하거나 NaN이 되어 lane_t가 비단조가 된다. p를 [0,1]로 묶으면
        // lane_t는 항상 두 knot 시각 사이에 들어가 단조성이 보장된다.
        const double span = next_x - current_x;
        const double p = span > 0.0 ? std::clamp((model_x_idx_double(xidx) - current_x) / span, 0.0, 1.0) : 1.0;
        lane_t[xidx] = static_cast<float>(p * model_t_idx_double(tidx + 1) + (1.0 - p) * model_t_idx_double(tidx));
    }
}

bool ParsedLeads::primary(int time_idx, float min_probability, ParsedLeadPoint *lead, float *probability) const {
    const float global_prob = global_probabilities[time_idx];
    if (global_prob < min_probability) return false;

    *lead = predictions[time_idx].points[0];
    if (probability) *probability = global_prob;
    return true;
}

ParsedModelOutput ModelOutputParser::parse(const std::vector<float> &raw) {
    ParsedModelOutput output;
    // axmodel 계약이 2576 float로 고정이라 부분 길이를 받아줄 이유가 없다.
    output.valid = raw.size() >= static_cast<size_t>(kModelOutputFloats);
    if (!output.valid) return output;

    for (int i = 0; i < kTrajectorySize; ++i) {
        const int mean_base = kPlanOffset + i * kPlanWidth;
        output.plan.points[i] = {
            raw[mean_base + 0],
            raw[mean_base + 1],
            raw[mean_base + 2],
        };
        output.plan.yaw[i] = raw[mean_base + kPlanYawIndex];
        output.plan.yaw_rate[i] = raw[mean_base + kPlanYawRateIndex];
    }

    for (int lane = 0; lane < 4; ++lane) {
        ParsedLaneLine &line = output.lanes[lane];
        const int prob_idx = kLaneProbOffset + lane * 2 + 1;
        const int std_base = kLaneOffset + kLaneLineSize + lane * kTrajectorySize * 2;
        line.probability = sigmoid(raw[prob_idx]);
        line.std = std::exp(raw[std_base]);
        const int base = kLaneOffset + lane * kTrajectorySize * 2;
        for (int i = 0; i < kTrajectorySize; ++i) {
            line.points[i] = {
                model_x_idx(i),
                raw[base + i * 2 + 0],
                raw[base + i * 2 + 1],
            };
        }
    }

    for (int edge = 0; edge < 2; ++edge) {
        ParsedRoadEdge &road_edge = output.road_edges[edge];
        const int std_base = kRoadEdgeOffset + kRoadEdgeMeanSize + edge * kTrajectorySize * 2;
        road_edge.std = std::exp(raw[std_base]);
        const int base = kRoadEdgeOffset + edge * kTrajectorySize * 2;
        for (int i = 0; i < kTrajectorySize; ++i) {
            road_edge.points[i] = {
                model_x_idx(i),
                raw[base + i * 2 + 0],
                raw[base + i * 2 + 1],
            };
        }
    }

    for (int sel = 0; sel < kLeadMhpSelection; ++sel) {
        const int base = kLeadOffset + sel * kLeadTrajLen * kLeadElementSize;
        for (int i = 0; i < kLeadTrajLen; ++i) {
            output.leads.predictions[sel].points[i] = {
                raw[base + i * kLeadElementSize + 0],
                raw[base + i * kLeadElementSize + 1],
                raw[base + i * kLeadElementSize + 2],
                raw[base + i * kLeadElementSize + 3],
            };
        }
        output.leads.global_probabilities[sel] = sigmoid(raw[kLeadProbOffset + sel]);
    }

    softmax(raw.data() + kDesireStateOffset, output.meta.desire_state.data(), kDesireLen);
    for (int i = 0; i < kMetaPressHorizons; ++i) {
        output.meta.gas_press[i] = sigmoid(raw[kMetaOffset + kMetaGasPressIndex + i * kMetaPressStride]);
        output.meta.brake_press[i] = sigmoid(raw[kMetaOffset + kMetaBrakePressIndex + i * kMetaPressStride]);
    }

    const float *pose_src = raw.data() + kPoseOffset;
    for (int i = 0; i < 3; ++i) {
        output.pose.trans[i] = pose_src[i];
        output.pose.rot[i] = pose_src[3 + i];
        output.pose.trans_std[i] = std::exp(pose_src[6 + i]);
        output.pose.rot_std[i] = std::exp(pose_src[9 + i]);
    }
    const float *road_src = raw.data() + kRoadTransformOffset;  // 평균 6, 이어서 log std 6
    for (int i = 0; i < 3; ++i) {
        output.pose.road_trans[i] = road_src[i];
        output.pose.road_trans_std[i] = std::exp(road_src[6 + i]);
    }

    return output;
}
