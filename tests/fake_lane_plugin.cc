// gtest_lane_plugin이 여는 가짜 차선선 플러그인. fake_lane_plugin_set_mode의 모드에 따라 정상 결과, 실패 코드,
// 약속을 어긴 결과를 낸다. FAKE_LANE_PLUGIN_ABI를 주면 그 ABI 버전을 알린다(맞지 않는 플러그인 시험).
#include "model/lane_plugin_abi.h"

#include <cmath>
#include <cstring>
#include <stdexcept>

#ifndef FAKE_LANE_PLUGIN_ABI
#define FAKE_LANE_PLUGIN_ABI EP_LANE_PLUGIN_ABI_VERSION
#endif

namespace {

// 0 정상, 1 실패 코드 7, 2 열거값 밖, 3 유한하지 않은 점, 4 신뢰도 1.5(잘린다), 5 예외(계약 위반)
int g_mode = 0;

int32_t decode(const EpTensor *outputs, uint32_t output_count, const EpLaneGeometry *geometry, EpLaneResult *result) {
    if (g_mode == 1) return 7;
    if (g_mode == 5) throw std::out_of_range("fake decode threw");
    // 'line' 텐서의 첫 값을 가까운 점의 x로 쓴다(텐서가 이름과 함께 넘어오는지 본다)
    float x0 = 0.0f;
    for (uint32_t i = 0; i < output_count; ++i)
        if (std::strcmp(outputs[i].name, "line") == 0 && outputs[i].count > 0) x0 = outputs[i].data[0];
    const float bottom = static_cast<float>(geometry->crop_y + geometry->crop_height);
    const float top = static_cast<float>(geometry->crop_y);

    result->line_count = 1;
    EpLaneLine &line = result->lines[0];
    line.slot = EP_LANE_EGO_LEFT;
    line.shape = EP_LANE_DASHED;
    line.double_line = EP_LANE_SINGLE;
    line.color = g_mode == 2 ? 9 : EP_LANE_YELLOW;
    line.confidence = g_mode == 4 ? 1.5f : 0.8f;
    line.point_count = 2;
    line.points[0] = {x0, bottom, 10.0f, x0 + 5.0f};
    line.points[1] = {g_mode == 3 ? NAN : x0 + 40.0f, top, 4.0f, x0 + 42.0f};

    result->boundary_count = 1;
    EpLaneBoundary &boundary = result->boundaries[0];
    boundary.slot = EP_BOUNDARY_NEAR_LEFT;
    boundary.type = EP_BOUNDARY_GUARDRAIL;
    boundary.confidence = 0.6f;
    boundary.point_count = 1;
    boundary.points[0] = {x0 - 100.0f, bottom, 0.0f, x0 - 100.0f};

    result->vanishing_x = static_cast<float>(geometry->image_width) * 0.5f;
    result->vanishing_y = top;
    result->vanishing_confidence = 0.9f;
    return 0;
}

const EpLanePlugin kPlugin = {
    FAKE_LANE_PLUGIN_ABI, 0, "fake lane plugin", "fake.axmodel", {0.0f, 0.4f, 1.0f, 0.8f}, decode,
};

}  // namespace

extern "C" {

__attribute__((visibility("default"))) const EpLanePlugin *edgepilot_lane_plugin_v1(void) { return &kPlugin; }

__attribute__((visibility("default"))) void fake_lane_plugin_set_mode(int mode) { g_mode = mode; }
}
