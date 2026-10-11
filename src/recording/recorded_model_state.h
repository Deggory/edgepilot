#ifndef RECORDED_MODEL_STATE_H
#define RECORDED_MODEL_STATE_H

/* 녹화된 ModelState 페이로드를 현재 구조체로 읽는다(진단 도구 공용).
 *   v12: 현재 구조체 그대로
 *   v11: lead_flow가 없다(0 = 확대율 없음)
 *   v10: flow_calibration이 56 B(v11 끝의 네 필드가 없다)이고 calibration_source가 없다
 *        (0 = 모델 포즈 출처)
 *   v8~v9: flow_calibration도 없다(0 = 꺼짐)
 *   v7: gas_press_probs/brake_press_probs도 없다(0으로 둔다)
 *   v6: camera_offset_m/camera_height_m이 없다(오프셋 0, 높이 0 = 모름)
 *   v5: plan_yaw/plan_yaw_rate도 없다(0으로 둔다)
 *   v4 이하: plan 뒤에 stds/orientations가 더 있다
 *   v3 이하: lead 뒤에 stop_line(28 B)이 더 있다 */

#include "common/ipc_messages.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

inline bool decode_recorded_model_state(const char *src, uint32_t payload_size, uint32_t version,
                                        ModelState *out)
{
    *out = ModelState{};
    const size_t plan_extra = version <= 4 ? 2 * sizeof(out->plan) : 0;
    const size_t lead_extra = version <= 3 ? 28 : 0;
    const size_t body = version <= 5   ? offsetof(ModelState, plan_yaw)
                        : version == 6 ? offsetof(ModelState, camera_offset_m)
                        : version == 7 ? offsetof(ModelState, gas_press_probs)
                        : version <= 9 ? offsetof(ModelState, flow_calibration)
                        : version == 10 ? offsetof(ModelState, flow_calibration) + 56
                        : version == 11 ? offsetof(ModelState, lead_flow)
                                        : sizeof(ModelState);
    if (payload_size < body + plan_extra + lead_extra) return false;
    const size_t lanes_off = offsetof(ModelState, lanes);
    const size_t pose_off = offsetof(ModelState, pose);
    char *dst = reinterpret_cast<char *>(out);
    std::memcpy(dst, src, lanes_off);
    std::memcpy(dst + lanes_off, src + lanes_off + plan_extra, pose_off - lanes_off);
    std::memcpy(dst + pose_off, src + pose_off + plan_extra + lead_extra, body - pose_off);
    return true;
}

#endif
