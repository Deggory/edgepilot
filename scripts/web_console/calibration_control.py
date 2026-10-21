"""카메라 보정 카드: modeld가 ModelState에 싣는 보정 상태(워프에 쓴 값, 광류 추정, 포즈 보정기 자신의 추정, 출처)를
보여 주고, 해제 상태에서만 초기화와 보정 출처 바꾸기를 받는다. 초기화 요청은 파일 하나이고 modeld의
CalibrationService가 1초마다 보고 지운 뒤 포즈 보정과 광류 블록을 모두 버리고 처음부터 다시 모은다. 보정 출처는 기기
설정(display.json)의 calibration_use_flow이고 modeld가 1초마다 다시 읽는다. 이 카드만 바꾸는 항목이라(메타데이터
card="calibration") 파라미터 API로는 받지 않는다."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict

from .files import env_path
from .param_store import ParamStore
from .refusal import Refusal
from .shm_channel import IpcReader, flag_names, freshness, struct_layout, topic_path, unpack_fields
from .state_layout import (CONTROL_MAX_AGE_S, CONTROL_STATE_SIZE, CONTROL_STATE_TOPIC, MODEL_CALIBRATION_AT,
                           MODEL_CALIBRATION_SOURCE_AT, MODEL_FLOW_CALIBRATION_AT, MODEL_STATE_TOPIC, control_engaged)

# modeld(calibration_service.cc)와 같은 변수로 바꿀 수 있다
CALIBRATION_RESET_PATH = "/dev/shm/edgepilot_calibration_reset"
# ipc_messages.h CalibrationState, FlowCalibrationState, CalibrationSourceState
CALIBRATION_FIELDS = (("status", "I"), ("valid_blocks", "i"), ("rpy", "3f"), ("spread", "3f"))
FLOW_CALIBRATION_FIELDS = (
    ("window_frame_id", "Q"), ("status", "I"), ("windows", "I"), ("blocks", "I"),
    ("pitch", "f"), ("yaw", "f"), ("sd_pitch", "f"), ("sd_yaw", "f"),
    ("window_pitch", "f"), ("window_yaw", "f"), ("window_rotation", "f"), ("window_tracks", "I"),
    ("window_inliers", "I"), ("mount_changes", "I"), ("window_parallax", "f"), ("estimate_blocks", "I"),
    ("rejected_windows", "I"),
)
CALIBRATION_SOURCE_FIELDS = (("source", "I"), ("requested", "I"), ("flags", "I"), ("pose_status", "I"),
                             ("pose_valid_blocks", "i"), ("pose_rpy", "3f"))
CALIBRATION_STATE = struct_layout(CALIBRATION_FIELDS)
FLOW_CALIBRATION_STATE = struct_layout(FLOW_CALIBRATION_FIELDS)
CALIBRATION_SOURCE_STATE = struct_layout(CALIBRATION_SOURCE_FIELDS)
CALIBRATION_STATUS = ("uncalibrated", "calibrated", "invalid", "recalibrating")
FLOW_STATUS = ("off", "collecting", "estimate")  # ipc_messages.h FlowCalibrationStatus
CALIBRATION_SOURCES = ("pose", "flow")  # ipc_messages.h CalibrationSource
SOURCE_FLAGS = ("flow_usable", "switch_pending", "ramping")  # ipc_messages.h kCalibrationSource* 비트 순서
SOURCE_GROUP, SOURCE_KEY = "display", "calibration_use_flow"  # 보정 출처 설정(modeld ModeldSettings가 읽는다)
MODEL_MAX_AGE_S = 2.0  # 모델 상태는 20 Hz로 나온다


def status_name(names: tuple, value: int) -> str:
    return names[value] if value < len(names) else str(value)


def degrees(value: float) -> float:
    return round(math.degrees(value), 3)


class CalibrationControl:
    """초기화와 출처 바꾸기는 openpilot의 Reset Calibration과 같이 결합 중에는 받지 않는다. 학습값(paramsd·torqued)은
    CAN 요레이트로 배우므로 카메라 장착과 무관해 그대로 둔다."""

    def __init__(self, store: ParamStore, model_path: str = topic_path(MODEL_STATE_TOPIC),
                 control_path: str = topic_path(CONTROL_STATE_TOPIC), request_path: str | None = None):
        self.store = store
        self.model = IpcReader(model_path, MODEL_CALIBRATION_SOURCE_AT + CALIBRATION_SOURCE_STATE.size)
        self.control = IpcReader(control_path, CONTROL_STATE_SIZE)
        self.request_path = Path(request_path) if request_path else env_path("EDGEPILOT_CALIBRATION_RESET_PATH",
                                                                             CALIBRATION_RESET_PATH)

    def engaged(self) -> bool:
        """controlsd가 살아 있고 결합 중이면 True. controlsd가 없거나 멈췄으면 조향도 없다."""
        latest = self.control.read_fresh(CONTROL_MAX_AGE_S)
        return latest is not None and control_engaged(latest[2])

    def use_flow(self) -> bool:
        """설정 파일의 보정 출처. modeld가 다시 읽기 전(1초 안)에도 페이지가 고른 값을 보이게 파일에서 읽는다."""
        return self.store.read_group(SOURCE_GROUP)[SOURCE_KEY]

    def status(self) -> Dict[str, Any]:
        latest = self.model.read_payload()
        result: Dict[str, Any] = {
            **freshness(latest[1] if latest else None, MODEL_MAX_AGE_S),
            "engaged": self.engaged(),
            "reset_pending": self.request_path.exists(),
            "use_flow": self.use_flow(),
        }
        if latest is None:
            return result
        payload = latest[2]
        pose = unpack_fields(CALIBRATION_STATE, CALIBRATION_FIELDS, payload, MODEL_CALIBRATION_AT)
        flow = unpack_fields(FLOW_CALIBRATION_STATE, FLOW_CALIBRATION_FIELDS, payload, MODEL_FLOW_CALIBRATION_AT)
        source = unpack_fields(CALIBRATION_SOURCE_STATE, CALIBRATION_SOURCE_FIELDS, payload,
                               MODEL_CALIBRATION_SOURCE_AT)
        result.update({
            "status": status_name(CALIBRATION_STATUS, pose["status"]),
            "valid_blocks": pose["valid_blocks"],
            "rpy_deg": [degrees(v) for v in pose["rpy"]],
            "flow": {
                "status": status_name(FLOW_STATUS, flow["status"]),
                "windows": flow["windows"], "blocks": flow["blocks"], "estimate_blocks": flow["estimate_blocks"],
                "mount_changes": flow["mount_changes"], "rejected_windows": flow["rejected_windows"],
                "pitch_deg": degrees(flow["pitch"]), "yaw_deg": degrees(flow["yaw"]),
                "sd_deg": [degrees(flow["sd_pitch"]), degrees(flow["sd_yaw"])],
                "window": {"pitch_deg": degrees(flow["window_pitch"]), "yaw_deg": degrees(flow["window_yaw"]),
                           "rotation_deg": degrees(flow["window_rotation"]), "tracks": flow["window_tracks"],
                           "inliers": flow["window_inliers"], "parallax_px": round(flow["window_parallax"], 2)},
            },
            "source": status_name(CALIBRATION_SOURCES, source["source"]),
            **flag_names(source["flags"], SOURCE_FLAGS),
            "pose": {"status": status_name(CALIBRATION_STATUS, source["pose_status"]),
                     "valid_blocks": source["pose_valid_blocks"], "rpy_deg": [degrees(v) for v in source["pose_rpy"]]},
        })
        return result

    def request_reset(self) -> Dict[str, Any]:
        if self.engaged():
            raise Refusal("error.calibration_reset_engaged")
        self.request_path.parent.mkdir(parents=True, exist_ok=True)
        self.request_path.touch()
        return self.status()

    def set_source(self, use_flow: bool) -> Dict[str, Any]:
        """modeld도 결합 중에는 해제를 기다리지만(CalibrationSourceSelector), 바꾼 줄 아는데 적용이 늦는 일이 없게
        여기서도 받지 않는다."""
        if self.engaged():
            raise Refusal("error.calibration_source_engaged")
        self.store.update(SOURCE_GROUP, {SOURCE_KEY: use_flow}, allow_managed=True)
        return self.status()
