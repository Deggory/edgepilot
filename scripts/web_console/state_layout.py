"""웹 콘솔이 위치로 읽는 ModelState·ControlState 필드. src/common/ipc_messages.h의 EDGEPILOT_*_AT 고정값과
같아야 하고, check_web_console.py가 대조한다. BEV 페이지는 이 위치를 /api/bev로 받아 직접 읽는다."""
from __future__ import annotations

import struct
from typing import Any

MODEL_STATE_TOPIC = "/edgepilot_model_state"
CONTROL_STATE_TOPIC = "/edgepilot_control_state"
LANE_STATE_TOPIC = "/edgepilot_lane_state2"
MODEL_STATE_SIZE = 3736
CONTROL_STATE_SIZE = 240

MODEL_STATE_AT = {
    "capture_timestamp_ns": 8, "model_timestamp_ns": 16, "valid": 28, "plan": 304, "lanes": 700, "lane_probabilities": 2284,
    "road_edges": 2316, "road_edge_stds": 3108, "lead": 3148, "gas_press_probs": 3528, "lead_flow": 3680,
    "lead_track": 3712,
}
MODEL_CALIBRATION_AT = 3224  # offsetof(ModelState, calibration)
MODEL_FLOW_CALIBRATION_AT = 3576  # offsetof(ModelState, flow_calibration)
MODEL_CALIBRATION_SOURCE_AT = 3648  # offsetof(ModelState, calibration_source)
CONTROL_STATE_AT = {
    "timestamp_ns": 0, "enabled": 8, "engaged": 12, "active": 16, "vehicle_fresh": 32,
    "left_blinker": 40, "right_blinker": 44, "gear": 52, "cluster_speed_kph": 56,
    "desired_curvature": 72, "actual_curvature": 76, "normalized_output": 80,
    "departure_alert_type": 144, "green_light_alert_armed": 152, "hud_flags": 184, "ego_speed_kph": 232,
}
# 차선선 플러그인 결과(LaneState)의 앞부분: 헤더와 도로면 블록(kLaneStateRoadPrefixBytes). 도로면 선 하나
# (LaneRoadLine)는 size 바이트이고 y는 거리 격자 X_IDXS의 앞쪽 points개 점의 가로 위치(m, 오른쪽 +)다.
LANE_STATE_PREFIX_SIZE = 848
LANE_STATE_AT = {"frame_id": 0, "capture_timestamp_ns": 8, "valid": 28, "road_count": 44, "road": 48}
LANE_ROAD_AT = {"size": 80, "points": 17, "kind": 0, "slot": 2, "type": 3, "double_line": 4, "color": 5,
                "confidence": 8, "y": 12}
HUD_FLAG_BITS = {"Laneless": 0, "LaneChanging": 5, "SteerPaused": 7, "BrakeLights": 9, "Lanebow": 10}  # ipc_messages.h kHudFlag<이름>의 비트
GEAR_PARK = 0  # car/can_frame.h kGearPark


def control_value(payload: bytes, name: str, fmt: str = "<I") -> Any:
    """ControlState 페이로드에서 필드 하나."""
    return struct.unpack_from(fmt, payload, CONTROL_STATE_AT[name])[0]
