#include "common/model_output.h"

#include <algorithm>
#include <cmath>

using namespace model_output_layout;

namespace {

float sigmoid_impl(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

void softmax(const float *input, float *output, int size)
{
    const float max_value = *std::max_element(input, input + size);
    float denominator = 0.0f;
    for (int i = 0; i < size; ++i) {
        output[i] = std::exp(input[i] - max_value);
        denominator += output[i];
    }
    const float inv_denominator = 1.0f / denominator;
    for (int i = 0; i < size; ++i)
        output[i] *= inv_denominator;
}

} // namespace

float ModelOutputParser::sigmoid(float x)
{
    return sigmoid_impl(x);
}

bool ParsedLeads::primary(int time_idx, float min_probability, ParsedLeadPoint *lead, float *probability) const
{
    if (!valid || !lead) return false;
    const int idx = std::max(0, std::min(kLeadMhpSelection - 1, time_idx));
    const float global_prob = global_probabilities[idx];
    if (global_prob < min_probability) return false;

    *lead = predictions[idx].points[0];
    if (probability) *probability = global_prob;
    return true;
}

ParsedModelOutput ModelOutputParser::parse(const std::vector<float> &raw)
{
    ParsedModelOutput output;
    // axmodel 계약이 2576 float로 고정이라 부분 길이를 받아줄 이유가 없다.
    output.valid = raw.size() >= static_cast<size_t>(kModelOutputFloats);
    if (!output.valid) return output;

    // 단일 가설이라 고를 것이 없다. 확률은 0.9.4 소비자 호환용으로 1.
    output.plan.valid = true;
    output.plan.best_index = 0;
    output.plan.probability = 1.0f;
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
        line.valid = true;
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
        road_edge.valid = true;
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

    output.leads.valid = true;
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

    softmax(raw.data() + kDesireStateOffset,
            output.meta.desire_state.data(), kDesireLen);
    for (int i = 0; i < kMetaPressHorizons; ++i) {
        output.meta.gas_press[i] = sigmoid(raw[kMetaOffset + kMetaGasPressIndex + i * kMetaPressStride]);
        output.meta.brake_press[i] = sigmoid(raw[kMetaOffset + kMetaBrakePressIndex + i * kMetaPressStride]);
    }

    output.has_pose = true;
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
