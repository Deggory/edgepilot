#ifndef RECORDED_MODEL_STATE_H
#define RECORDED_MODEL_STATE_H

/* 녹화된 ModelState 페이로드를 현재 구조체로 읽는다(진단 도구 공용).
 *   v6: 현재 구조체 그대로
 *   v5: plan_yaw/plan_yaw_rate가 없다(0으로 둔다)
 *   v4 이하: plan 뒤에 stds/orientations가 더 있다
 *   v3 이하: lead 뒤에 stop_line(28 B)이 더 있다 */

#include "ipc_messages.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

inline bool decode_recorded_model_state(const char *src, uint32_t payload_size, uint32_t version,
                                        ModelState *out)
{
    *out = ModelState{};
    const size_t plan_extra = version <= 4 ? 2 * sizeof(out->plan) : 0;
    const size_t lead_extra = version <= 3 ? 28 : 0;
    const size_t body = version <= 5 ? offsetof(ModelState, plan_yaw) : sizeof(ModelState);
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
