"""웹 콘솔이 위치로 읽는 ModelState·ControlState 필드. src/common/ipc_messages.h의 EDGEPILOT_*_AT 고정값과
같아야 하고, check_web_console.py가 대조한다. BEV 페이지는 이 위치를 /api/bev로 받아 직접 읽는다."""
from __future__ import annotations

import struct
from typing import Any

MODEL_STATE_TOPIC = "/edgepilot_model_state"
CONTROL_STATE_TOPIC = "/edgepilot_control_state"
MODEL_STATE_SIZE = 2648
CONTROL_STATE_SIZE = 240

MODEL_STATE_AT = {
    "capture_timestamp_ns": 8, "model_timestamp_ns": 16, "valid": 28, "plan": 32, "lanes": 428,
    "lane_probabilities": 1484, "road_edges": 1516, "road_edge_stds": 2044, "lead": 2084, "gas_press_probs": 2464,
    "lead_flow": 2592, "lead_track": 2624,
}
MODEL_CALIBRATION_AT = 2160  # offsetof(ModelState, calibration)
MODEL_FLOW_CALIBRATION_AT = 2488  # offsetof(ModelState, flow_calibration)
MODEL_CALIBRATION_SOURCE_AT = 2560  # offsetof(ModelState, calibration_source)
CONTROL_STATE_AT = {
    "timestamp_ns": 0, "enabled": 8, "engaged": 12, "active": 16, "vehicle_fresh": 32, "steering_fault": 36,
    "left_blinker": 40, "right_blinker": 44, "gear": 52, "cluster_speed_kph": 56,
    "desired_curvature": 72, "actual_curvature": 76, "normalized_output": 80,
    "departure_alert_type": 144, "green_light_alert_armed": 152, "hud_flags": 184, "ego_speed_kph": 232,
}
# CONTROL_STATE_AT 필드의 형식: 여기 없는 것은 uint32다
CONTROL_FORMATS = {"timestamp_ns": "<Q", "gear": "<i", "cluster_speed_kph": "<f", "desired_curvature": "<f",
                   "actual_curvature": "<f", "normalized_output": "<f", "ego_speed_kph": "<f"}
# ipc_messages.h kHudFlag<이름>의 비트
HUD_FLAG_BITS = {"Laneless": 0, "SoftDisabling": 2, "SteerPaused": 7, "BrakeLights": 9}
GEAR_PARK = 0  # car/can_frame.h kGearPark
# 제어 상태를 믿는 나이. panda_firmware.cc kControlMaxAgeNs와 같다(controlsd는 100 Hz로 낸다)
CONTROL_MAX_AGE_S = 1.0


def control_value(payload: bytes, name: str) -> Any:
    """ControlState 페이로드에서 필드 하나."""
    return struct.unpack_from(CONTROL_FORMATS.get(name, "<I"), payload, CONTROL_STATE_AT[name])[0]


def control_engaged(payload: bytes) -> bool:
    """결합 중이거나 조향 중인가. 보정 초기화·출처 바꾸기와 Panda 플래싱이 이때는 받지 않는다."""
    return bool(control_value(payload, "engaged") or control_value(payload, "active"))
