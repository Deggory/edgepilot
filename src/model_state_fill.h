#ifndef MODEL_STATE_FILL_H
#define MODEL_STATE_FILL_H

/* modeld가 발행 직전에 ModelState를 채운다. 보정 상태(OnlineCalibrator)가 필요해 IPC 레이아웃
 * 헤더(ipc_messages.h)와 나눠, 소비자가 보정 알고리즘 헤더를 끌어오지 않게 한다. */

#include "calibration_online.h"
#include "ipc_messages.h"

void fill_model_state(ModelState &state, const ParsedModelOutput &parsed,
                      const ProjectionState &projection,
                      const OnlineCalibrator::Snapshot &calibration,
                      uint64_t frame_id, uint64_t capture_timestamp_ns,
                      float model_execution_ms);

#endif
