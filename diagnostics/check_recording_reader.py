#!/usr/bin/env python3
"""tools/model/recording_reader.py의 바이너리 배치가 C++ 헤더의 고정값과 같은지 본다:
ControlState·LearnerState·ModelState의 offsetof 고정값과 크기, LocalizationState·ImuSample·
CalibrationState 크기, 이벤트 레코드 머리·프레임 인덱스 레코드 크기, 기록 타입 번호. 도구가 쓰는
MaixCAM2 카메라 내부 파라미터(tools/model/model_warp.py)가 src/app_config.h와 같은지도 본다.
ctest가 저장소 루트에서 돌린다."""

import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "model"))

import model_warp  # noqa: E402
import recording_reader as rr  # noqa: E402

IPC = (ROOT / "src" / "ipc_messages.h").read_text(encoding="utf-8")
FORMAT = (ROOT / "src" / "recording_format.h").read_text(encoding="utf-8")
APP_CONFIG = (ROOT / "src" / "app_config.h").read_text(encoding="utf-8")


def pinned_offsets(macro: str) -> dict[str, int]:
    return {name: int(at) for name, at in re.findall(macro + r"\((\w+), (\d+)\);", IPC)}


def pinned(pattern: str, text: str = IPC) -> int:
    match = re.search(pattern, text)
    assert match, pattern
    return int(match.group(1))


def size_of(struct: str, text: str = IPC) -> int:
    return pinned(r"sizeof\(" + struct + r"\) == (\d+)", text)


def offset_in(dtype, name: str) -> int:
    return dtype.fields[name][1]


class RecordingReaderLayout(unittest.TestCase):
    def test_control_state(self):
        offsets = pinned_offsets("EDGEPILOT_CONTROL_STATE_AT")
        self.assertGreater(len(offsets), 40)
        for name, offset in offsets.items():
            self.assertIn(name, rr.CONTROL_STATE.names, "C++와 같은 필드 이름")
            self.assertEqual(offset_in(rr.CONTROL_STATE, name), offset, name)
        # C++ 구조체는 uint64_t로 시작해 8바이트 경계까지 끝을 덧댄다. dtype에는 그 덧댐이 없다.
        self.assertEqual(-(-rr.CONTROL_STATE.itemsize // 8) * 8, size_of("ControlState"))

    def test_learner_state(self):
        for name, offset in pinned_offsets("EDGEPILOT_LEARNER_STATE_AT").items():
            self.assertEqual(offset_in(rr.LEARNER_STATE, name), offset, name)
        self.assertEqual(rr.LEARNER_STATE.itemsize, size_of("LearnerState"))

    def test_localization_and_imu(self):
        self.assertEqual(rr.LOCALIZATION_STATE.itemsize, size_of("LocalizationState"))
        self.assertEqual(rr.IMU_SAMPLE.itemsize, size_of("ImuSample"))
        self.assertEqual(rr.IMU_BATCH_HEAD.size, pinned(r"sizeof\(ImuBatch\) == (\d+) \+"), "ImuBatch 머리")

    def test_model_state(self):
        layout = rr.model_state_layout(pinned(r"kRecordingVersion = (\d+)", FORMAT))
        self.assertEqual(layout["__size__"], size_of("ModelState"))
        for name, offset in pinned_offsets("EDGEPILOT_MODEL_STATE_AT").items():
            self.assertEqual(layout[name], offset, name)
        self.assertEqual(layout["calibration"], pinned(r"offsetof\(ModelState, calibration\) == (\d+)"))
        self.assertEqual(layout["plan_yaw"], pinned(r"offsetof\(ModelState, plan_yaw\) == (\d+)"))
        self.assertEqual(rr.CALIBRATION_STATE.itemsize, size_of("CalibrationState"))
        for name in rr.MODEL_STATE_HEAD.names:
            self.assertEqual(offset_in(rr.MODEL_STATE_HEAD, name), layout[name], name)

    def test_recording_format(self):
        self.assertEqual(rr.EVENT_RECORD_HEADER.size, size_of("EventRecordHeader", FORMAT))
        self.assertEqual(rr.FRAME_INDEX_RECORD.itemsize, size_of("FrameIndexRecord", FORMAT))
        body = re.search(r"enum class RecordType : uint16_t \{(.*?)\};", FORMAT, re.S).group(1)
        types = {name: int(value) for name, value in re.findall(r"(\w+) = (\d+),", body)}
        self.assertEqual(len(types), 8)
        for name, value in types.items():
            constant = "RECORD_" + re.sub(r"(?<!^)(?=[A-Z])", "_", name).upper()
            self.assertEqual(getattr(rr, constant), value, constant)


class CameraIntrinsics(unittest.TestCase):
    def test_maixcam2_matches_app_config(self):
        for axis, value in (("Fx", model_warp.MAIXCAM2_FX_1080), ("Fy", model_warp.MAIXCAM2_FY_1080),
                            ("Cx", model_warp.MAIXCAM2_CX_1080), ("Cy", model_warp.MAIXCAM2_CY_1080)):
            match = re.search(r"constexpr float kCamera" + axis + r" = ([\d.]+)f;", APP_CONFIG)
            self.assertIsNotNone(match, axis)
            self.assertEqual(float(match.group(1)), value, axis)


if __name__ == "__main__":
    unittest.main()
