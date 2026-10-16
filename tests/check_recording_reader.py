#!/usr/bin/env python3
"""tools/model/recording_reader.py의 바이너리 배치가 C++ 헤더의 고정값과 같은지 본다:
ControlState·LearnerState·ModelState·LaneState의 offsetof 고정값과 크기, LocalizationState·ImuSample·
CalibrationState 크기, 이벤트 레코드 머리·프레임 인덱스 레코드 크기, 기록 타입 번호. 도구가 쓰는
MaixCAM2 카메라 내부 파라미터(tools/model/model_warp.py)가 src/common/app_config.h와 같은지, 그 워프 행렬이
런타임(gtest_calibration의 기준값)과 같은지도 본다.
ctest가 저장소 루트에서 돌린다. 두 도구가 numpy를 쓰므로 numpy가 없으면 77로 끝나고 ctest는 건너뛴 것으로 센다."""

import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "model"))

try:
    import numpy as np
except ModuleNotFoundError:
    print("numpy가 없어 건너뛴다")
    sys.exit(77)

import model_warp  # noqa: E402
import recording_reader as rr  # noqa: E402

IPC = (ROOT / "src" / "common" / "ipc_messages.h").read_text(encoding="utf-8")
FORMAT = (ROOT / "src" / "recording" / "recording_format.h").read_text(encoding="utf-8")
APP_CONFIG = (ROOT / "src" / "common" / "app_config.h").read_text(encoding="utf-8")


def pinned_offsets(macro: str) -> dict[str, int]:
    return {name: int(at) for name, at in re.findall(macro + r"\((\w+), (\d+)\);", IPC)}


def pinned(pattern: str, text: str = IPC) -> int:
    match = re.search(pattern, text)
    assert match, pattern
    return int(match.group(1))


def size_of(struct: str, text: str = IPC) -> int:
    return pinned(r"sizeof\(" + struct + r"\) == (\d+)", text)


def struct_fields(name: str, text: str) -> list[tuple[str, str, int]]:
    """C++ 구조체의 (형식, 이름, 배열 길이) 목록. uint32_t·int32_t·float·uint8_t 필드만 있는 구조체용."""
    body = re.search(r"struct " + name + r" \{(.*?)\n\};", text, re.S).group(1)
    return [(kind, field, int(count or 1))
            for kind, field, count in re.findall(r"(uint32_t|int32_t|float|uint8_t) (\w+)(?:\[(\d+)\])?", body)]


def dtype_fields(dtype) -> list[tuple[str, str, int]]:
    kinds = {"<u4": "uint32_t", "<i4": "int32_t", "<f4": "float", "|u1": "uint8_t"}
    out = []
    for name in dtype.names:
        field = dtype.fields[name][0]
        base = field.base if field.shape else field
        out.append((kinds[base.str], name, int(np.prod(field.shape)) if field.shape else 1))
    return out


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
        offsets = pinned_offsets("EDGEPILOT_LEARNER_STATE_AT")
        self.assertGreater(len(offsets), 5)
        for name, offset in offsets.items():
            self.assertEqual(offset_in(rr.LEARNER_STATE, name), offset, name)
        self.assertEqual(rr.LEARNER_STATE.itemsize, size_of("LearnerState"))

    def test_lane_state(self):
        self.assertEqual(rr.LANE_STATE.itemsize, size_of("LaneState"))
        self.assertEqual(rr.LANE_LINE.itemsize, size_of("LaneLineState"))
        self.assertEqual(rr.LANE_BOUNDARY.itemsize, size_of("LaneBoundaryState"))
        self.assertEqual(rr.LANE_POINT.itemsize, size_of("LanePointState"))
        offsets = {name: int(at) for name, at in re.findall(r"offsetof\(LaneState, (\w+)\) == (\d+)", IPC)}
        self.assertGreater(len(offsets), 4)
        for name, offset in offsets.items():
            self.assertEqual(offset_in(rr.LANE_STATE, name), offset, name)
        self.assertEqual(rr.LANE_ROAD_LINE.itemsize, size_of("LaneRoadLine"))
        self.assertEqual(rr.LANE_MAX_POINTS, pinned(r"kLaneMaxPoints = (\d+);"))
        self.assertEqual(rr.LANE_ROAD_POINTS, pinned(r"kLaneRoadPoints = (\d+);"))

    def test_localization_and_imu(self):
        self.assertEqual(rr.LOCALIZATION_STATE.itemsize, size_of("LocalizationState"))
        self.assertEqual(rr.IMU_SAMPLE.itemsize, size_of("ImuSample"))
        self.assertEqual(rr.IMU_BATCH_HEAD.size, pinned(r"sizeof\(ImuBatch\) == (\d+) \+"), "ImuBatch 머리")

    def test_model_state(self):
        self.assertEqual(rr.RECORDING_VERSION, pinned(r"kRecordingVersion = (\d+)", FORMAT))
        layout = rr.model_state_layout(rr.RECORDING_VERSION)
        self.assertEqual(layout["__size__"], size_of("ModelState"))
        offsets = pinned_offsets("EDGEPILOT_MODEL_STATE_AT")
        self.assertGreater(len(offsets), 5)
        for name, offset in offsets.items():
            self.assertEqual(layout[name], offset, name)
        self.assertEqual(layout["calibration"], pinned(r"offsetof\(ModelState, calibration\) == (\d+)"))
        self.assertEqual(layout["plan_yaw"], pinned(r"offsetof\(ModelState, plan_yaw\) == (\d+)"))
        self.assertEqual(layout["flow_calibration"], pinned(r"offsetof\(ModelState, flow_calibration\) == (\d+)"))
        self.assertEqual(layout["calibration_source"], pinned(r"offsetof\(ModelState, calibration_source\) == (\d+)"))
        self.assertEqual(layout["lead_flow"], pinned(r"offsetof\(ModelState, lead_flow\) == (\d+)"))
        self.assertEqual(layout["lead_track"], pinned(r"offsetof\(ModelState, lead_track\) == (\d+)"))
        self.assertEqual(rr.CALIBRATION_STATE.itemsize, size_of("CalibrationState"))
        self.assertEqual(dtype_fields(rr.POSE_STATE), struct_fields("PoseState", IPC))
        self.assertEqual(rr.FLOW_CALIBRATION_STATE.itemsize, size_of("FlowCalibrationState"))
        self.assertEqual(rr.CALIBRATION_SOURCE_STATE.itemsize, size_of("CalibrationSourceState"))
        self.assertEqual(rr.LEAD_FLOW_STATE.itemsize, size_of("LeadFlowState"))
        self.assertEqual(rr.LEAD_TRACK_STATE.itemsize, size_of("LeadTrackState"))
        # v12 녹화는 lead_track 앞에서, v11 녹화는 lead_flow 앞에서 끝난다
        self.assertEqual(rr.model_state_layout(12)["__size__"], layout["lead_track"])
        self.assertEqual(rr.model_state_layout(11)["__size__"], layout["lead_flow"])
        # v10 녹화는 FlowCalibrationState의 앞 56 B만 담았다(v11이 끝에 덧붙였다)
        self.assertEqual(rr.model_state_layout(10)["__size__"], layout["flow_calibration"] + 56)
        for name in rr.MODEL_STATE_HEAD.names:
            self.assertEqual(offset_in(rr.MODEL_STATE_HEAD, name), layout[name], name)

    def test_calibration_helpers(self):
        import numpy as np
        v11 = bytearray(rr.model_state_layout(11)["__size__"])
        flow = np.zeros((), rr.FLOW_CALIBRATION_STATE)
        flow["status"], flow["yaw"], flow["estimate_blocks"], flow["window_parallax"] = 2, 0.01, 7, 3.5
        flow["sd_yaw"] = 0.002
        at = rr.model_state_layout(11)["flow_calibration"]
        v11[at:at + flow.itemsize] = flow.tobytes()
        source = np.zeros((), rr.CALIBRATION_SOURCE_STATE)
        source["source"], source["pose_yaw"] = 1, -0.005
        at = rr.model_state_layout(11)["calibration_source"]
        v11[at:at + source.itemsize] = source.tobytes()
        got = rr.flow_calibration(bytes(v11), 11)
        self.assertEqual((int(got["status"]), int(got["estimate_blocks"])), (2, 7))
        self.assertAlmostEqual(float(got["window_parallax"]), 3.5, places=5)
        self.assertAlmostEqual(float(got["sd_yaw"]), 0.002, places=6)
        self.assertEqual(int(rr.calibration_source(bytes(v11), 11)["source"]), 1)
        # v10: 앞 56 B만 있고 출처는 늘 포즈(0), pose_*는 calibration을 되풀이한다
        v10 = bytearray(v11[:rr.model_state_layout(10)["__size__"]])
        cal = np.zeros((), rr.CALIBRATION_STATE)
        cal["status"], cal["yaw"] = 1, -0.008
        at = rr.model_state_layout(10)["calibration"]
        v10[at:at + cal.itemsize] = cal.tobytes()
        old = rr.flow_calibration(bytes(v10), 10)
        self.assertEqual((int(old["status"]), int(old["estimate_blocks"])), (2, 0))
        src = rr.calibration_source(bytes(v10), 10)
        self.assertEqual(int(src["source"]), 0)
        self.assertAlmostEqual(float(src["pose_yaw"]), -0.008, places=6)
        self.assertEqual(int(rr.flow_calibration(bytes(v10), 9)["status"]), 0)

    def test_lead_flow_helper(self):
        import numpy as np
        v12 = bytearray(rr.model_state_layout(12)["__size__"])
        lead = np.zeros((), rr.LEAD_FLOW_STATE)
        lead["valid"], lead["expansion_rate"], lead["lead_x"], lead["inliers"] = 1, 0.08, 37.5, 21
        at = rr.model_state_layout(12)["lead_flow"]
        v12[at:at + lead.itemsize] = lead.tobytes()
        got = rr.lead_flow(bytes(v12), 12)
        self.assertEqual((int(got["valid"]), int(got["inliers"])), (1, 21))
        self.assertAlmostEqual(float(-got["lead_x"] * got["expansion_rate"]), -3.0, places=4)
        self.assertEqual(int(rr.lead_flow(bytes(v12[:rr.model_state_layout(11)["__size__"]]), 11)["valid"]), 0)

    def test_lead_track_helper(self):
        import numpy as np
        v13 = bytearray(rr.model_state_layout(13)["__size__"])
        track = np.zeros((), rr.LEAD_TRACK_STATE)
        track["valid"], track["flags"], track["distance_m"], track["relative_speed_mps"] = 1, rr.LEAD_TRACK_FLOW_USED, 31.5, -2.25
        at = rr.model_state_layout(13)["lead_track"]
        v13[at:at + track.itemsize] = track.tobytes()
        got = rr.lead_track(bytes(v13), 13)
        self.assertEqual((int(got["valid"]), int(got["flags"])), (1, rr.LEAD_TRACK_FLOW_USED))
        self.assertAlmostEqual(float(got["relative_speed_mps"]), -2.25, places=5)
        self.assertEqual(int(rr.lead_track(bytes(v13[:rr.model_state_layout(12)["__size__"]]), 12)["valid"]), 0)
        flags = {name: 1 << int(bit) for name, bit in re.findall(r"constexpr uint32_t kLeadTrack(\w+) = 1u << (\d+);", IPC)}
        self.assertEqual(flags, {"FlowUsed": rr.LEAD_TRACK_FLOW_USED, "FlowRejected": rr.LEAD_TRACK_FLOW_REJECTED,
                                 "DistanceRejected": rr.LEAD_TRACK_DISTANCE_REJECTED, "Started": rr.LEAD_TRACK_STARTED})

    def test_magics(self):
        def magic(name: str) -> bytes:
            chars = re.search(name + r"\[8\] = \{([^}]*)\}", FORMAT).group(1)
            return "".join(re.findall(r"'(.)'", chars)).encode()
        self.assertEqual(rr.EVENT_LOG_MAGICS, (magic("kEventLogMagic"), magic("kLegacyEventLogMagic")))
        self.assertEqual(rr.FRAME_INDEX_MAGICS, (magic("kFrameIndexMagic"), magic("kLegacyFrameIndexMagic")))

    def test_recording_format(self):
        self.assertEqual(rr.EVENT_RECORD_HEADER.size, size_of("EventRecordHeader", FORMAT))
        self.assertEqual(rr.RECORDED_CAN_FRAME.itemsize, size_of("RecordedCanFrame", FORMAT))
        self.assertEqual(dtype_fields(rr.RECORDED_CAN_FRAME), struct_fields("RecordedCanFrame", FORMAT))
        self.assertEqual(len(struct_fields("RecordedCanBatchHeader", FORMAT)) * 4, rr.RECORDED_CAN_BATCH_HEAD.size)
        self.assertEqual(rr.FRAME_INDEX_RECORD.itemsize, size_of("FrameIndexRecord", FORMAT))
        body = re.search(r"enum class RecordType : uint16_t \{(.*?)\};", FORMAT, re.S).group(1)
        types = {name: int(value) for name, value in re.findall(r"(\w+) = (\d+),", body)}
        self.assertEqual(len(types), 9)
        for name, value in types.items():
            constant = "RECORD_" + re.sub(r"(?<!^)(?=[A-Z])", "_", name).upper()
            self.assertEqual(getattr(rr, constant), value, constant)


class CanRecords(unittest.TestCase):
    def test_can_frames_reads_the_batch(self):
        """CanRx 레코드: 머리의 count만큼, 잘린 레코드면 들어 있는 만큼 읽는다."""
        frames = np.zeros(2, rr.RECORDED_CAN_FRAME)
        frames["address"] = [0x251, 0x4f1]
        frames["src"] = [0, 2]
        frames["data_len"] = [8, 4]
        frames["data"][0, :8] = range(1, 9)
        payload = rr.RECORDED_CAN_BATCH_HEAD.pack(2, 0) + frames.tobytes()
        read = rr.can_frames(payload)
        self.assertEqual(read["address"].tolist(), [0x251, 0x4f1])
        self.assertEqual(read["data"][0, :8].tolist(), list(range(1, 9)))
        self.assertEqual(len(rr.can_frames(payload[:-10])), 1)


class CameraIntrinsics(unittest.TestCase):
    def test_maixcam2_matches_app_config(self):
        for axis, value in (("Fx", model_warp.MAIXCAM2_FX_1080), ("Fy", model_warp.MAIXCAM2_FY_1080),
                            ("Cx", model_warp.MAIXCAM2_CX_1080), ("Cy", model_warp.MAIXCAM2_CY_1080)):
            match = re.search(r"constexpr float kCamera" + axis + r" = ([\d.]+)f;", APP_CONFIG)
            self.assertIsNotNone(match, axis)
            self.assertEqual(float(match.group(1)), value, axis)


class ModelWarp(unittest.TestCase):
    def test_projection_matches_the_cpp_golden(self):
        """model_warp.projection_matrix가 런타임(ModelInputTransform)과 같은 행렬을 낸다. 기준값은
        gtest_calibration ProjectionGoldenSharedWithModelWarpPy가 C++ 코드로 확인하는 것이다."""
        source = (ROOT / "tests" / "gtest_calibration.cc").read_text(encoding="utf-8")
        block = source[source.index("// GOLDEN BEGIN: roll"):source.index("// GOLDEN END")]
        number = r"(-?[\d.]+(?:e-?\d+)?)"
        cases = re.findall(r"\{" + r"f?,\s*".join([number] * 3) + r"f,\s*(true|false),\s*" + number + r"f,\s*"
                           + number + r"f,\s*\{([^}]*)\}\}", block)
        self.assertEqual(len(cases), 5)
        intrinsics = model_warp.maixcam2_intrinsics(1280, 720)
        for roll, pitch, yaw, sbig, offset, height, values in cases:
            expected = np.array([float(v) for v in values.split(",")])
            actual = model_warp.projection_matrix(*intrinsics, np.array([float(roll), float(pitch), float(yaw)]),
                                                  sbig == "true", float(offset), float(height)).ravel()
            tolerance = np.maximum(1.0, np.abs(expected)) * 1e-5
            tolerance[6:8] = 1e-9
            np.testing.assert_array_less(np.abs(actual - expected), tolerance, f"{roll} {pitch} {yaw} {sbig}")


if __name__ == "__main__":
    unittest.main()
