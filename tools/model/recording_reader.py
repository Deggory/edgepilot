"""recordd route(frames.bin / 이벤트 로그 / road.h264) 리더.

바이너리 배치는 src/recording/recording_format.h와 src/common/ipc_messages.h를 그대로 따른다. 구조체
크기를 스트림에서 본 페이로드 크기와 대조하므로, 배치가 어긋나면 쓰레기를 디코딩하지 않고
바로 실패한다.
"""

from __future__ import annotations

import functools
import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np

# recording_format.h kRecordingVersion: recordd가 지금 쓰는 배치(check_recording_reader.py가 대조한다)
RECORDING_VERSION = 13
# v9 매직이 앞에 온다. v8까지의 녹화는 이전 매직을 담고 있다(recording_format.h kLegacy*)
FRAME_INDEX_MAGICS = (b"EDGEIDX1", b"K230IDX1")
EVENT_LOG_MAGICS = (b"EDGELOG1", b"K230LOG1")
EVENT_RECORD_HEADER = struct.Struct("<QHHI")  # timestamp_ns, type, flags, size

RECORD_CAN_RX = 1
RECORD_CAN_TX = 2
RECORD_MODEL_STATE = 3
RECORD_CONTROL_STATE = 4
RECORD_PANDA_STATE = 5
RECORD_LEARNER_STATE = 6
RECORD_IMU = 7
RECORD_LOCALIZATION = 8

FRAME_INDEX_RECORD = np.dtype([
    ("frame_id", "<u8"),
    ("capture_timestamp_ns", "<u8"),
    ("encode_index", "<u8"),
    ("file_offset", "<u8"),
    ("packet_size", "<u4"),
    ("flags", "<u4"),
])

# ControlState: 자연 정렬이고 packing pragma가 없다. 멤버가 모두
# 4/8바이트라 끝의 8바이트 패딩 말고는 패딩이 없다.
CONTROL_STATE = np.dtype([
    ("timestamp_ns", "<u8"),
    ("enabled", "<u4"), ("engaged", "<u4"), ("active", "<u4"),
    ("should_send", "<u4"), ("path_usable", "<u4"), ("seeds_ready", "<u4"),
    ("vehicle_fresh", "<u4"), ("steering_fault", "<u4"),
    ("left_blinker", "<u4"), ("right_blinker", "<u4"), ("cruise_active", "<u4"),
    ("gear", "<i4"),
    ("cluster_speed_kph", "<f4"), ("cruise_max_speed_kph", "<f4"),
    ("cruise_command_speed_kph", "<f4"), ("steering_angle_deg", "<f4"),
    ("desired_curvature", "<f4"), ("actual_curvature", "<f4"),
    ("normalized_output", "<f4"),
    ("desired_torque", "<i4"), ("apply_torque", "<i4"), ("driver_torque", "<i4"),
    ("desire", "<u4"),
    ("active_block", "S32"),
    ("radar_lead_valid", "<u4"), ("radar_lead_distance_m", "<f4"),
    ("radar_lead_relative_speed_mps", "<f4"),
    ("departure_alert_type", "<u4"), ("departure_alert_event_id", "<u4"),
    ("green_light_alert_armed", "<u4"),
    ("tpms_valid", "<u4"), ("tpms_unit", "<u4"),
    ("tpms_pressure_fl", "<f4"), ("tpms_pressure_fr", "<f4"),
    ("tpms_pressure_rl", "<f4"), ("tpms_pressure_rr", "<f4"),
    ("tpms_warning", "<u4"), ("hud_flags", "<u4"),
    ("engage_event_id", "<u4"), ("disengage_event_id", "<u4"),
    ("engage_reject_event_id", "<u4"),
    ("engage_reject_block", "S32"),
    ("ego_speed_kph", "<f4"),
])

# ImuBatch(src/common/ipc_messages.h): 16바이트 머리 뒤에 `count`개 샘플(그만큼만 기록된다).
IMU_BATCH_HEAD = struct.Struct("<QII")
IMU_SAMPLE = np.dtype([
    ("timestamp_ns", "<u8"), ("accel_mps2", "<f4", 3), ("gyro_rad_s", "<f4", 3),
    ("temperature_c", "<f4"), ("reserved", "<u4"),
])
assert IMU_SAMPLE.itemsize == 40

# LocalizationState(locationd, ~20 Hz). flags 비트: ipc_messages.h kLocalization*.
LOCALIZATION_STATE = np.dtype([
    ("timestamp_ns", "<u8"), ("flags", "<u4"), ("lag_status", "<u4"),
    ("orientation_calib", "<f4", 3), ("orientation_std", "<f4", 3),
    ("angular_velocity_calib", "<f4", 3), ("angular_velocity_calib_std", "<f4", 3),
    ("velocity_device", "<f4", 3), ("velocity_device_std", "<f4", 3),
    ("acceleration_calib", "<f4", 3),
    ("lateral_delay_s", "<f4"), ("lag_estimate_s", "<f4"), ("lag_estimate_std_s", "<f4"),
    ("lag_valid_blocks", "<i4"), ("lag_cal_perc", "<i4"), ("lag_points", "<u4"), ("input_flags", "<u4"),
])
assert LOCALIZATION_STATE.itemsize == 128

# LearnerState(paramsd/torqued 출력, 20 Hz). flags 비트: ipc_messages.h kLearner*.
LEARNER_STATE = np.dtype([
    ("timestamp_ns", "<u8"), ("flags", "<u4"),
    ("steer_ratio", "<f4"), ("stiffness_factor", "<f4"), ("roll_rad", "<f4"),
    ("angle_offset_average_deg", "<f4"), ("angle_offset_deg", "<f4"),
    ("steer_ratio_std", "<f4"), ("stiffness_factor_std", "<f4"),
    ("angle_offset_average_std", "<f4"), ("angle_offset_fast_std", "<f4"),
    ("yaw_bias_rad_s", "<f4"),
    ("lat_accel_factor_raw", "<f4"), ("lat_accel_offset_raw", "<f4"), ("friction_raw", "<f4"),
    ("lat_accel_factor", "<f4"), ("lat_accel_offset", "<f4"), ("friction", "<f4"),
    ("decay", "<f4"), ("max_resets", "<f4"),
    ("total_bucket_points", "<i4"), ("cal_perc", "<i4"),
    ("road_bank_lat_accel", "<f4"),
    ("prior_steer_ratio", "<f4"), ("prior_lat_accel_factor", "<f4"), ("prior_friction", "<f4"),
    ("bucket_points", "<i2", (8,)), ("plan_delay_s", "<f4"),
])

# ModelState의 앞부분(frame_id ..). 이 도구들이 쓰는 필드만 디코딩하고,
# 가운데의 plan/lane 페이로드는 위치로 건너뛴다.
# CalibrationState는 구조체의 마지막 멤버라 페이로드 끝에서부터
# 위치를 잡는다.
MODEL_STATE_HEAD = np.dtype([
    ("frame_id", "<u8"),
    ("capture_timestamp_ns", "<u8"),
    ("model_timestamp_ns", "<u8"),
    ("model_execution_ms", "<f4"),
    ("valid", "<u4"),
    ("best_plan", "<i4"),
    ("plan_probability", "<f4"),
])

CALIBRATION_STATE = np.dtype([
    ("status", "<u4"),
    ("valid_blocks", "<i4"),
    ("roll", "<f4"), ("pitch", "<f4"), ("yaw", "<f4"),
    ("spread", "<f4", (3,)),
])

# ModelState.flow_calibration(v10+): 광류 보정기의 상태, 각도는 rad. v10은 앞 56바이트를 기록했고,
# v11이 mount_changes, window_parallax(px), estimate_blocks, rejected_windows를 덧붙였다. sd_*는
# 블록 중앙값들의 표준편차다. v10은 같은 자리에 그 범위(max - min)를 저장했다.
# flow_calibration()으로 읽는다.
_FLOW_CALIBRATION_FIELDS = [
    ("window_frame_id", "<u8"),
    ("status", "<u4"), ("windows", "<u4"), ("blocks", "<u4"),
    ("pitch", "<f4"), ("yaw", "<f4"), ("sd_pitch", "<f4"), ("sd_yaw", "<f4"),
    ("window_pitch", "<f4"), ("window_yaw", "<f4"), ("window_rotation", "<f4"),
    ("window_tracks", "<u4"), ("window_inliers", "<u4"),
]
FLOW_CALIBRATION_STATE_V10 = np.dtype(_FLOW_CALIBRATION_FIELDS)
FLOW_CALIBRATION_STATE = np.dtype(_FLOW_CALIBRATION_FIELDS + [
    ("mount_changes", "<u4"), ("window_parallax", "<f4"), ("estimate_blocks", "<u4"), ("rejected_windows", "<u4"),
])
assert FLOW_CALIBRATION_STATE_V10.itemsize == 56 and FLOW_CALIBRATION_STATE.itemsize == 72

# ModelState.calibration_source(v11+): 워프가 쓴 보정의 출처와 포즈 보정기 자신의 추정.
# v11 전에는 워프가 언제나 포즈 보정기(source 0)를 썼고 calibration이 그 값을 담았다.
CALIBRATION_SOURCE_STATE = np.dtype([
    ("source", "<u4"),       # 0 모델 포즈, 1 광류
    ("requested", "<u4"),    # 웹 설정(calibration_use_flow)
    ("flags", "<u4"),        # bit 0 광류 사용 가능, bit 1 전환 대기(결합 중), bit 2 워프 램핑 중
    ("pose_status", "<u4"), ("pose_valid_blocks", "<i4"),
    ("pose_roll", "<f4"), ("pose_pitch", "<f4"), ("pose_yaw", "<f4"),
])
assert CALIBRATION_SOURCE_STATE.itemsize == 32

# ModelState.lead_flow(v12+): 광류로 잰 앞차 확대율(model/lead_flow). 상대 속도는
# -lead_x * expansion_rate(m/s, 음수 = 가까워짐)이고 충돌까지 시간은 1 / expansion_rate다.
LEAD_FLOW_STATE = np.dtype([
    ("frame_id", "<u8"),
    ("valid", "<u4"), ("points", "<u4"), ("inliers", "<u4"),
    ("expansion_rate", "<f4"),   # 1/s, ln(크기 비) / window_s, 양수 = 가까워짐
    ("window_s", "<f4"),
    ("lead_x", "<f4"),           # 창 끝의 모델 앞차 거리(m, 카메라 기준)
])
assert LEAD_FLOW_STATE.itemsize == 32

# ModelState.lead_track(v13+): 모델 앞차 거리와 광류 확대율을 칼만 필터로 섞은 값
# (model/lead_tracker). 비전 크루즈가 쓴다.
LEAD_TRACK_FLOW_USED, LEAD_TRACK_FLOW_REJECTED, LEAD_TRACK_DISTANCE_REJECTED, LEAD_TRACK_STARTED = 1, 2, 4, 8
LEAD_TRACK_STATE = np.dtype([
    ("valid", "<u4"), ("flags", "<u4"),
    ("distance_m", "<f4"),               # 카메라 기준, lead.x와 같다
    ("relative_speed_mps", "<f4"),       # 앞차 - 자차, 음수 = 가까워짐
    ("distance_std_m", "<f4"), ("relative_speed_std_mps", "<f4"),
])
assert LEAD_TRACK_STATE.itemsize == 24

TRAJECTORY_SIZE = 33
IPC_POINT = 12  # IpcPoint: float32 세 개


@functools.lru_cache(maxsize=None)
def model_state_layout(version: int) -> dict[str, int]:
    """녹화 버전별 ModelState 페이로드 안의 바이트 위치.

    런타임이 아무도 쓰지 않던 것을 덜어 내면서 구조체가 필드를 잃었으므로,
    한 버전을 하드코딩한 리더는 다른 버전을 조용히 잘못 읽는다:
      v3 이하는 stop_line이 있다(페이로드 4076 + 패딩 4 = 4080)
      v4는 stop_line을 뺐다(4048)
      v5는 plan_position_stds와 plan_orientations를 뺐다(3256)
      v6은 plan_yaw와 plan_yaw_rate를 덧붙였다(3520)
      v7은 camera_offset_m과 camera_height_m을 덧붙였다(3528). 그 프레임의
         워프에 쓴 카메라 장착이다
      v8은 gas_press_probs와 brake_press_probs를 덧붙였다(3576). 0, 2, ..., 10 s의
         모델 페달 예측이다
      v10은 flow_calibration을 덧붙였다(3632). 광류 보정기의
         섀도 상태다(v9는 파일 매직만 바꿨다)
      v11은 flow_calibration을 72바이트로 늘리고 calibration_source를 덧붙였다
         (3680). calibration은 출처와 상관없이 워프에 쓴 값이다
      v12는 lead_flow를 덧붙였다(3712). 광류로 잰 앞차 확대율이다
      v13은 lead_track을 덧붙였다(3736). 모델 앞차와 그 확대율을 섞은 값이다
    호출자는 ``layout["__size__"]``를 실제로 본 페이로드 크기와 대조해야
    이후 배치가 바뀌었을 때 쓰레기를 디코딩하지 않고 바로 실패한다.
    """
    n, pt = TRAJECTORY_SIZE, IPC_POINT
    fields = [("frame_id", 8), ("capture_timestamp_ns", 8),
              ("model_timestamp_ns", 8), ("model_execution_ms", 4),
              ("valid", 4), ("best_plan", 4), ("plan_probability", 4),
              ("model_t", 4 * n), ("lane_t", 4 * n), ("plan", pt * n)]
    if version <= 4:
        fields += [("plan_position_stds", pt * n), ("plan_orientations", pt * n)]
    fields += [("lanes", 4 * pt * n), ("lane_probabilities", 16),
               ("lane_stds", 16), ("road_edges", 2 * pt * n),
               ("road_edge_stds", 8), ("desire_state", 32), ("lead", 24)]
    if version <= 3:
        fields += [("stop_line", 28)]
    fields += [("pose", 52), ("calibration", 32)]
    if version >= 6:
        fields += [("plan_yaw", 4 * n), ("plan_yaw_rate", 4 * n)]
    if version >= 7:
        fields += [("camera_offset_m", 4), ("camera_height_m", 4)]
    if version >= 8:
        fields += [("gas_press_probs", 24), ("brake_press_probs", 24)]
    if version == 10:
        fields += [("flow_calibration", 56)]
    if version >= 11:
        fields += [("flow_calibration", 72), ("calibration_source", 32)]
    if version >= 12:
        fields += [("lead_flow", 32)]
    if version >= 13:
        fields += [("lead_track", 24)]

    layout, offset = {}, 0
    for name, size in fields:
        layout[name] = offset
        offset += size
    layout["__size__"] = offset
    return layout



def flow_calibration(payload: bytes, version: int):
    """ModelState.flow_calibration을 FLOW_CALIBRATION_STATE 레코드 하나로 낸다(v10 전에는 0. v10은 마지막
    네 필드가 없어 0으로 두고, sd_*에는 블록 중앙값들의 범위가 들어 있다)."""
    out = np.zeros((), FLOW_CALIBRATION_STATE)
    if version < 10:
        return out
    offset = model_state_layout(version)["flow_calibration"]
    old = np.frombuffer(payload, FLOW_CALIBRATION_STATE_V10 if version == 10 else FLOW_CALIBRATION_STATE,
                        count=1, offset=offset)[0]
    for name in old.dtype.names:
        out[name] = old[name]
    return out


def calibration_source(payload: bytes, version: int):
    """ModelState.calibration_source를 CALIBRATION_SOURCE_STATE 레코드 하나로 낸다. v11 전에는 워프가 포즈
    보정기를 썼으므로 source는 0이고 pose_*는 ModelState.calibration을 되풀이한다."""
    if version >= 11:
        return np.frombuffer(payload, CALIBRATION_SOURCE_STATE, count=1,
                             offset=model_state_layout(version)["calibration_source"])[0]
    out = np.zeros((), CALIBRATION_SOURCE_STATE)
    cal = np.frombuffer(payload, CALIBRATION_STATE, count=1, offset=model_state_layout(version)["calibration"])[0]
    out["pose_status"], out["pose_valid_blocks"] = cal["status"], cal["valid_blocks"]
    out["pose_roll"], out["pose_pitch"], out["pose_yaw"] = cal["roll"], cal["pitch"], cal["yaw"]
    return out

def _model_record(payload: bytes, version: int, name: str, dtype: np.dtype):
    """ModelState 필드 하나를 dtype 레코드로 낸다. 그 버전에 없는 필드면 0이다."""
    layout = model_state_layout(version)
    if name not in layout:
        return np.zeros((), dtype)
    return np.frombuffer(payload, dtype, count=1, offset=layout[name])[0]


def lead_flow(payload: bytes, version: int):
    """ModelState.lead_flow를 LEAD_FLOW_STATE 레코드 하나로 낸다(v12 전에는 0: 확대율이 없다)."""
    return _model_record(payload, version, "lead_flow", LEAD_FLOW_STATE)


def lead_track(payload: bytes, version: int):
    """ModelState.lead_track을 LEAD_TRACK_STATE 레코드 하나로 낸다(v13 전에는 0: 추적이 없다)."""
    return _model_record(payload, version, "lead_track", LEAD_TRACK_STATE)

@dataclass
class SegmentInfo:
    path: Path
    width: int
    height: int
    fps: int
    segment_start_ns: int
    frames: np.ndarray  # FRAME_INDEX_RECORD 배열


@dataclass
class RouteEvents:
    control: np.ndarray            # CONTROL_STATE 레코드들
    control_log_ts: np.ndarray     # control 레코드의 이벤트 머리 타임스탬프
    model_frame_id: np.ndarray
    model_capture_ts: np.ndarray
    model_rpy: np.ndarray          # [N, 3] 보정 roll/pitch/yaw(rad)
    model_valid_blocks: np.ndarray


def read_segment_index(segment_dir: Path) -> SegmentInfo:
    data = (segment_dir / "frames.bin").read_bytes()
    magic, version, header_size, record_size, width, height, fps = struct.unpack_from(
        "<8sIIIIII", data, 0)
    if magic not in FRAME_INDEX_MAGICS:
        raise ValueError(f"{segment_dir}: bad frames.bin magic {magic!r}")
    segment_start_ns, = struct.unpack_from("<Q", data, 32)
    if record_size != FRAME_INDEX_RECORD.itemsize:
        raise ValueError(f"{segment_dir}: index record size {record_size} != "
                         f"{FRAME_INDEX_RECORD.itemsize}")
    # 깔끔하지 않게 멈추면 파일이 레코드 중간에서 잘릴 수 있다. 온전한 레코드만 남긴다
    count = (len(data) - header_size) // FRAME_INDEX_RECORD.itemsize
    frames = np.frombuffer(data, FRAME_INDEX_RECORD, count=count, offset=header_size)
    return SegmentInfo(segment_dir, width, height, fps, segment_start_ns, frames)


def segment_video(seg_dir: Path) -> Path | None:
    """세그먼트의 H.264 영상 파일, 없으면 None."""
    path = seg_dir / "road.h264"
    return path if path.exists() else None


def route_segments(route_dir: Path) -> list[SegmentInfo]:
    seg_root = route_dir / "segments"
    segs = []
    for seg_dir in sorted(seg_root.iterdir()):
        if not (seg_dir.is_dir() and (seg_dir / "frames.bin").exists()
                and segment_video(seg_dir)):
            continue
        try:
            info = read_segment_index(seg_dir)
        except (ValueError, struct.error) as error:
            print(f"skipping {seg_dir}: {error}")
            continue
        if len(info.frames):
            segs.append(info)
    return segs


def decode_route_yuv(segments: list[SegmentInfo]):
    """route 전체에서 (index_record, y, u, v)를 스트림 순서대로 낸다.

    한 route의 세그먼트들은 이어진 H.264 스트림 하나를 60 s씩 자른 조각이다:
    route 전체의 바이트를 디코더 하나에 넣고, 디코딩된 프레임을 순서대로 인덱스
    레코드와 짝짓는다(인덱스 레코드 하나에 인코더 출력 하나).
    """
    import av
    import re

    codec = av.CodecContext.create("h264", "r")

    def au_starts(payload: bytes) -> int:
        """payload 안에서 시작하는 access unit: first_mb_in_slice == 0인 슬라이스."""
        count = 0
        for match in re.finditer(b"\x00\x00\x01", payload):
            pos = match.end()
            if pos + 1 < len(payload) and (payload[pos] & 0x1F) in (1, 5) \
                    and (payload[pos + 1] >> 7) & 1:
                count += 1
        return count

    # access unit마다(스트림 순서) 그 AU가 시작하는 바이트 범위의 인덱스 레코드를
    # 대응시키고 패킷에 AU 번호를 pts로 붙여서, 디코더가 하나를 빠뜨려도
    # 디코딩된 프레임이 레코드와 정확히 짝지어지게 한다.
    au_records: list[tuple[np.void, int]] = []
    for segment in segments:
        data = segment_video(segment.path).read_bytes()
        for rec in segment.frames:
            payload = data[int(rec["file_offset"]):
                           int(rec["file_offset"]) + int(rec["packet_size"])]
            for _ in range(au_starts(payload)):
                au_records.append((rec, segment.height))

    au_index = 0

    def decode(packet):
        nonlocal au_index
        if packet is not None:
            packet.pts = au_index
            au_index += 1
        for frame in codec.decode(packet):
            rec, vis_h = au_records[frame.pts]
            yuv = frame.reformat(format="yuv420p")
            h, w = frame.height, frame.width
            y = np.asarray(yuv.planes[0]).reshape(h, yuv.planes[0].line_size)[:, :w]
            u = np.asarray(yuv.planes[1]).reshape(h // 2, yuv.planes[1].line_size)[:, : w // 2]
            v = np.asarray(yuv.planes[2]).reshape(h // 2, yuv.planes[2].line_size)[:, : w // 2]
            yield rec, y[:vis_h], u[: vis_h // 2], v[: vis_h // 2]

    for segment in segments:
        data = segment_video(segment.path).read_bytes()
        # 인덱스 레코드가 덮는 바이트만 믿을 수 있다. 깔끔하지 않게 멈추면
        # 꼬리가 일부만 쓰인 채 남을 수 있다
        last = segment.frames[-1]
        data = data[: int(last["file_offset"]) + int(last["packet_size"])]
        for chunk_start in range(0, len(data), 1 << 20):
            for packet in codec.parse(data[chunk_start : chunk_start + (1 << 20)]):
                yield from decode(packet)
    for packet in codec.parse(b""):
        yield from decode(packet)
    yield from decode(None)
    if au_index < len(au_records):
        raise ValueError(f"parser produced {au_index} AUs, expected {len(au_records)}")


def route_event_files(route_dir: Path) -> list[Path]:
    """기록 순서대로 놓은 이벤트 로그 파일: v3은 events/에 60 s 청크를 돌려 쓰고,
    v2 이하는 route 단위 events.bin 하나를 둔다."""
    chunk_dir = route_dir / "events"
    if chunk_dir.is_dir():
        return sorted(chunk_dir.glob("*.bin"))
    legacy = route_dir / "events.bin"
    return [legacy] if legacy.exists() else []


@dataclass
class EventRecord:
    """이벤트 로그 레코드 하나. ``payload``는 청크를 복사 없이 가리키는 뷰다."""
    path: Path
    version: int
    timestamp_ns: int
    type: int
    payload: memoryview

    def control_state(self) -> np.void:
        expected = CONTROL_STATE.itemsize + _pad8(CONTROL_STATE.itemsize)
        if len(self.payload) != expected:
            raise ValueError(f"{self.path}: control state payload {len(self.payload)} "
                             f"!= {expected}; ControlState changed without a "
                             f"recording-version bump")
        return np.frombuffer(self.payload, CONTROL_STATE, count=1)[0]

    def learner_state(self) -> np.void:
        if len(self.payload) != LEARNER_STATE.itemsize:
            raise ValueError(f"{self.path}: learner state payload {len(self.payload)} "
                             f"!= {LEARNER_STATE.itemsize}")
        return np.frombuffer(self.payload, LEARNER_STATE, count=1)[0]

    def model_layout(self) -> dict[str, int]:
        """이 레코드 버전의 ModelState 위치. 레코드 크기와 대조한다: 기록기가
        레코드를 8바이트로 패딩하므로, 구조체 크기 +0..7을 벗어나면 버전을
        올리지 않고 배치가 바뀐 것이다."""
        layout = model_state_layout(self.version)
        expected = layout["__size__"]
        if not expected <= len(self.payload) <= expected + 7:
            raise ValueError(f"{self.path}: model state payload {len(self.payload)} "
                             f"does not match the v{self.version} layout "
                             f"({expected}, +0..7 pad)")
        return layout


def iter_event_records(path: Path) -> Iterator[EventRecord]:
    """이벤트 청크 하나의 모든 레코드를 파일 순서대로 낸다. 깔끔하지 않게 멈춰
    잘린 꼬리에서 반복이 끝난다. 머리보다 짧은 파일은 아무것도 내지 않고,
    매직이 틀리면 예외를 던진다."""
    data = path.read_bytes()
    if len(data) < 16:
        return
    magic, version, header_size = struct.unpack_from("<8sII", data, 0)
    if magic not in EVENT_LOG_MAGICS:
        raise ValueError(f"{path}: bad event log magic {magic!r}")
    view = memoryview(data)
    offset, end = header_size, len(data)
    while offset + EVENT_RECORD_HEADER.size <= end:
        ts, rtype, _flags, size = EVENT_RECORD_HEADER.unpack_from(data, offset)
        offset += EVENT_RECORD_HEADER.size
        if offset + size > end:
            break
        yield EventRecord(path, version, ts, rtype, view[offset:offset + size])
        offset += size


def read_route_events(route_dir: Path) -> RouteEvents | None:
    paths = [p for p in route_event_files(route_dir) if p.stat().st_size > 32]
    if not paths:
        return None

    controls, control_ts = [], []
    model_fid, model_cts, model_rpy, model_blocks = [], [], [], []
    for path in paths:
        for rec in iter_event_records(path):
            if rec.type == RECORD_CONTROL_STATE:
                controls.append(rec.control_state())
                control_ts.append(rec.timestamp_ns)
            elif rec.type == RECORD_MODEL_STATE:
                layout = rec.model_layout()
                head = np.frombuffer(rec.payload, MODEL_STATE_HEAD, count=1)[0]
                calib = np.frombuffer(rec.payload, CALIBRATION_STATE, count=1,
                                      offset=layout["calibration"])[0]
                model_fid.append(head["frame_id"])
                model_cts.append(head["capture_timestamp_ns"])
                model_rpy.append((calib["roll"], calib["pitch"], calib["yaw"]))
                model_blocks.append(calib["valid_blocks"])

    if not model_fid and not controls:
        return None
    return RouteEvents(
        control=np.array(controls, dtype=CONTROL_STATE),
        control_log_ts=np.array(control_ts, dtype=np.uint64),
        model_frame_id=np.array(model_fid, dtype=np.uint64),
        model_capture_ts=np.array(model_cts, dtype=np.uint64),
        model_rpy=np.array(model_rpy, dtype=np.float32).reshape(-1, 3),
        model_valid_blocks=np.array(model_blocks, dtype=np.int32),
    )


def _pad8(size: int) -> int:
    return (8 - size % 8) % 8


def read_route_imu(route_dir: Path) -> np.ndarray | None:
    """route의 보드 IMU 샘플 전부(칩 원래 축, 자이로 바이어스 제거 전). route에
    IMU 레코드가 없으면(imud가 생기기 전 녹화) None."""
    chunks = []
    for path in route_event_files(route_dir):
        if path.stat().st_size <= 32:
            continue
        for rec in iter_event_records(path):
            if rec.type != RECORD_IMU:
                continue
            _, count, _ = IMU_BATCH_HEAD.unpack_from(rec.payload, 0)
            if IMU_BATCH_HEAD.size + count * IMU_SAMPLE.itemsize > len(rec.payload):
                continue
            chunks.append(np.frombuffer(rec.payload, IMU_SAMPLE, count, IMU_BATCH_HEAD.size))
    return np.concatenate(chunks) if chunks else None


def read_route_localization(route_dir: Path) -> np.ndarray | None:
    """route의 locationd 출력 전부. 없으면(locationd 전 녹화) None."""
    rows = []
    for path in route_event_files(route_dir):
        if path.stat().st_size <= 32:
            continue
        for rec in iter_event_records(path):
            if rec.type == RECORD_LOCALIZATION and len(rec.payload) == LOCALIZATION_STATE.itemsize:
                rows.append(np.frombuffer(rec.payload, LOCALIZATION_STATE, 1)[0])
    return np.array(rows, dtype=LOCALIZATION_STATE) if rows else None


def read_route_calibration(route_dir: Path) -> np.ndarray | None:
    path = route_dir / "params" / "calibration.json"
    if not path.exists():
        return None
    rpy = json.loads(path.read_text()).get("rpy_rad")
    return np.asarray(rpy, dtype=np.float32) if rpy else None
