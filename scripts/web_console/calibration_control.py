"""카메라 캘리브레이션 카드: modeld가 ModelState에 싣는 보정 상태(워프에 쓴 값, 광류 추정, 포즈 보정기 자신의 추정,
출처)를 보여 주고, 해제 상태에서만 초기화와 보정 출처 바꾸기를 받는다. 초기화 요청은 파일 하나이고 modeld의
CalibrationService가 1초마다 보고 지운 뒤 포즈 보정과 광류 블록을 모두 버리고 처음부터 다시 모은다. 보정 출처는 기기
설정(display.json)의 calibration_use_flow이고 modeld가 1초마다 다시 읽는다. 이 카드만 바꾸는 항목이라(메타데이터
card="calibration") 파라미터 API로는 받지 않는다."""
from __future__ import annotations

import math
import os
import struct
from pathlib import Path
from typing import Any, Dict

from .param_store import ParamStore
from .shm_channel import IpcReader, age_s, topic_path
from .state_layout import (CONTROL_STATE_SIZE, CONTROL_STATE_TOPIC, MODEL_CALIBRATION_AT, MODEL_CALIBRATION_SOURCE_AT,
                           MODEL_FLOW_CALIBRATION_AT, MODEL_STATE_TOPIC, control_value)

# modeld(calibration_service.cc)와 같은 변수로 바꿀 수 있다
CALIBRATION_RESET_PATH = os.environ.get("EDGEPILOT_CALIBRATION_RESET_PATH", "/dev/shm/edgepilot_calibration_reset")
CALIBRATION_STATE = struct.Struct("<Ii3f3f")  # CalibrationState 필드: status, valid_blocks, rpy, spread
CALIBRATION_STATUS = ("uncalibrated", "calibrated", "invalid", "recalibrating")
# FlowCalibrationState 필드: window_frame_id, status, windows, blocks, pitch, yaw, sd_pitch, sd_yaw, window_pitch,
# window_yaw, window_rotation, window_tracks, window_inliers, mount_changes, window_parallax, estimate_blocks, rejected_windows
FLOW_CALIBRATION_STATE = struct.Struct("<Q3I4f3f2IIf2I")
FLOW_STATUS = ("off", "collecting", "estimate")
# CalibrationSourceState 필드: source, requested, flags, pose_status, pose_valid_blocks, 포즈 rpy
CALIBRATION_SOURCE_STATE = struct.Struct("<4Ii3f")
SOURCE_FLAG_FLOW_USABLE, SOURCE_FLAG_PENDING, SOURCE_FLAG_RAMPING = 1, 2, 4  # ipc_messages.h kCalibrationSource*
SOURCE_GROUP, SOURCE_KEY = "display", "calibration_use_flow"  # 보정 출처 설정(modeld ModeldSettings가 읽는다)
CONTROL_STALE_S = 2.0


def status_name(names: tuple, value: int) -> str:
    return names[value] if value < len(names) else str(value)


def degrees(values) -> list:
    return [round(math.degrees(v), 3) for v in values]


class CalibrationControl:
    """초기화와 출처 바꾸기는 openpilot의 Reset Calibration과 같이 결합 중에는 받지 않는다. 학습값(paramsd·torqued)은
    CAN 요레이트로 배우므로 카메라 장착과 무관해 그대로 둔다."""

    def __init__(self, store: ParamStore, model_path: str = topic_path(MODEL_STATE_TOPIC),
                 control_path: str = topic_path(CONTROL_STATE_TOPIC), request_path: str = CALIBRATION_RESET_PATH):
        self.store = store
        self.model = IpcReader(model_path, MODEL_CALIBRATION_SOURCE_AT + CALIBRATION_SOURCE_STATE.size)
        self.control = IpcReader(control_path, CONTROL_STATE_SIZE)
        self.request_path = Path(request_path)

    def engaged(self) -> bool:
        """controlsd가 살아 있고 결합 중이면 True. controlsd가 없거나 멈췄으면 조향도 없다."""
        latest = self.control.read_payload()
        if latest is None or age_s(latest[1]) > CONTROL_STALE_S:
            return False
        return bool(control_value(latest[2], "engaged"))

    def use_flow(self) -> bool:
        """설정 파일의 보정 출처. modeld가 다시 읽기 전(1초 안)에도 페이지가 고른 값을 보이게 파일에서 읽는다."""
        try:
            return bool(self.store.read_group(SOURCE_GROUP).get(SOURCE_KEY, False))
        except (KeyError, ValueError):
            return False

    def status(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "available": False,
            "engaged": self.engaged(),
            "reset_pending": self.request_path.exists(),
            "use_flow": self.use_flow(),
        }
        latest = self.model.read_payload()
        if latest is None:
            return result
        _, stamp, payload = latest
        status, valid_blocks, *values = CALIBRATION_STATE.unpack_from(payload, MODEL_CALIBRATION_AT)
        result.update({
            "available": True,
            "age_s": age_s(stamp),
            "status": status_name(CALIBRATION_STATUS, status),
            "valid_blocks": valid_blocks,
            "rpy_deg": degrees(values[:3]),
            "spread_deg": degrees(values[3:]),
        })
        (_, f_status, windows, blocks, f_pitch, f_yaw, sd_pitch, sd_yaw, w_pitch, w_yaw, w_rot, tracks, inliers,
         mount_changes, parallax, estimate_blocks, rejected) = FLOW_CALIBRATION_STATE.unpack_from(payload,
                                                                                                MODEL_FLOW_CALIBRATION_AT)
        result["flow"] = {
            "status": status_name(FLOW_STATUS, f_status),
            "windows": windows, "blocks": blocks, "estimate_blocks": estimate_blocks,
            "mount_changes": mount_changes, "rejected_windows": rejected,
            "pitch_deg": degrees([f_pitch])[0], "yaw_deg": degrees([f_yaw])[0],
            "sd_deg": degrees([sd_pitch, sd_yaw]),
            "window": {"pitch_deg": degrees([w_pitch])[0], "yaw_deg": degrees([w_yaw])[0],
                       "rotation_deg": degrees([w_rot])[0], "tracks": tracks, "inliers": inliers,
                       "parallax_px": round(parallax, 2)},
        }
        source, _requested, flags, p_status, p_blocks, *pose = CALIBRATION_SOURCE_STATE.unpack_from(
            payload, MODEL_CALIBRATION_SOURCE_AT)
        result.update({
            "source": "flow" if source == 1 else "pose",
            "flow_usable": bool(flags & SOURCE_FLAG_FLOW_USABLE),
            "switch_pending": bool(flags & SOURCE_FLAG_PENDING),
            "ramping": bool(flags & SOURCE_FLAG_RAMPING),
            "pose": {"status": status_name(CALIBRATION_STATUS, p_status), "valid_blocks": p_blocks,
                     "rpy_deg": degrees(pose)},
        })
        return result

    def request_reset(self) -> Dict[str, Any]:
        if self.engaged():
            raise PermissionError("결합 중에는 초기화할 수 없습니다. 먼저 해제하세요.")
        self.request_path.parent.mkdir(parents=True, exist_ok=True)
        self.request_path.touch()
        return self.status()

    def set_source(self, use_flow: bool) -> Dict[str, Any]:
        """modeld도 결합 중에는 해제를 기다리지만(CalibrationSourceSelector), 바꾼 줄 아는데 적용이 늦는 일이 없게
        여기서도 받지 않는다."""
        if self.engaged():
            raise PermissionError("결합 중에는 보정 출처를 바꿀 수 없습니다. 먼저 해제하세요.")
        self.store.update(SOURCE_GROUP, {SOURCE_KEY: bool(use_flow)}, allow_managed=True)
        return self.status()
