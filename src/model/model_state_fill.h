#pragma once

// modeld가 발행 직전에 ModelState를 채운다(model_state_fill.cc, model 라이브러리). 보정 상태
// (OnlineCalibrator)가 필요해 IPC 레이아웃 헤더(ipc_messages.h)와 나눠, 소비자가 보정 알고리즘 헤더를
// 끌어오지 않게 한다.

#include "common/ipc_messages.h"
#include "common/projection.h"
#include "model/calibration_online.h"

ModelState fill_model_state(const ParsedModelOutput &parsed, const ProjectionState &projection,
                            const OnlineCalibrator::Snapshot &calibration, uint64_t frame_id,
                            uint64_t capture_timestamp_ns, float model_execution_ms);
