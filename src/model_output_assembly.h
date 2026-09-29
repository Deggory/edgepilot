#ifndef MODEL_OUTPUT_ASSEMBLY_H
#define MODEL_OUTPUT_ASSEMBLY_H

/* 출력 헤드를 나눈 axmodel(tools/model/axmodel/split_outputs.py)의 출력들을 openpilot의
 * 2576 레이아웃(model_output.h)으로 다시 모은다.
 *
 * 처음 axmodel은 헤드 13개를 이은 텐서 하나를 U16 하나의 눈금으로 양자화해 모든 값이
 * 약 0.007 단위로 나왔다. plan yaw·yaw rate(직선에서 수 mrad)가 0 아니면 ±0.007이 되어
 * laneless 곡률이 계단마다 약 0.6 m/s²씩 뛰었다(2026-09-29 실차). 헤드마다, 그리고 plan은
 * 위치·방향·표준편차로 나눠 각자 범위로 양자화한다. 이름과 순서는 split_outputs.py와 같다.
 * axengine에 의존하지 않아 호스트 검사가 레이아웃을 본다. */

#include "model_output.h"

#include <algorithm>
#include <cstddef>
#include <cstring>

namespace model_output_assembly {

enum class Piece {
    Contiguous,  // offset부터 count개 그대로
    PlanMotion,  // plan 평균 knot마다 열 0~8
    PlanOrient,  // plan 평균 knot마다 열 9~14
};

struct Part {
    const char *name;
    Piece piece;
    int offset;
    int count;
};

constexpr int kPlanKnots = kTrajectorySize;
constexpr int kPlanWidth = model_output_layout::kPlanWidth;  // 15
constexpr int kPlanMotionWidth = 9;
constexpr int kPlanOrientWidth = kPlanWidth - kPlanMotionWidth;

constexpr Part kParts[] = {
    {"out_meta", Piece::Contiguous, 0, 55},
    {"out_desire_pred", Piece::Contiguous, 55, 32},
    {"out_pose", Piece::Contiguous, 87, 12},
    {"out_wide_from_device", Piece::Contiguous, 99, 6},
    {"out_road_transform", Piece::Contiguous, 105, 12},
    {"out_lanes", Piece::Contiguous, 117, 528},
    {"out_lane_prob", Piece::Contiguous, 645, 8},
    {"out_road_edges", Piece::Contiguous, 653, 264},
    {"out_lead", Piece::Contiguous, 917, 144},
    {"out_lead_prob", Piece::Contiguous, 1061, 3},
    {"out_hidden", Piece::Contiguous, model_output_layout::kFeatureOffset, kModelFeatureLen},
    {"out_plan_motion", Piece::PlanMotion, model_output_layout::kPlanOffset, kPlanKnots * kPlanMotionWidth},
    {"out_plan_orient", Piece::PlanOrient, model_output_layout::kPlanOffset, kPlanKnots * kPlanOrientWidth},
    {"out_plan_std", Piece::Contiguous, model_output_layout::kPlanOffset + kPlanKnots * kPlanWidth,
     kPlanKnots * kPlanWidth},
    {"out_desire_state", Piece::Contiguous, model_output_layout::kDesireStateOffset, kDesireLen},
};
constexpr int kPartCount = static_cast<int>(sizeof(kParts) / sizeof(kParts[0]));

// 한 출력을 raw(kModelOutputFloats)의 제자리에 쓴다. 나머지 두 칸(패딩)은 호출자가 0으로 둔다.
inline void place(const Part &part, const float *src, float *raw)
{
    switch (part.piece) {
    case Piece::Contiguous:
        std::memcpy(raw + part.offset, src, sizeof(float) * static_cast<size_t>(part.count));
        break;
    case Piece::PlanMotion:
        for (int k = 0; k < kPlanKnots; ++k)
            std::memcpy(raw + part.offset + k * kPlanWidth, src + k * kPlanMotionWidth,
                        sizeof(float) * kPlanMotionWidth);
        break;
    case Piece::PlanOrient:
        for (int k = 0; k < kPlanKnots; ++k)
            std::memcpy(raw + part.offset + k * kPlanWidth + kPlanMotionWidth, src + k * kPlanOrientWidth,
                        sizeof(float) * kPlanOrientWidth);
        break;
    }
}

}  // namespace model_output_assembly

#endif
