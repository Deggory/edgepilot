"""recordd route(frames.bin / 이벤트 로그 / road.h264) 리더.

바이너리 배치는 src/recording/recording_format.h와 src/common/ipc_messages.h를 그대로 따른다. 구조체
크기를 스트림에서 본 페이로드 크기와 대조하므로, 배치가 어긋나면 쓰레기를 디코딩하지 않고
바로 실패한다.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np


# recording_format.h kRecordingVersion: recordd가 쓰는 배치(check_recording_reader.py가 대조한다). 옛 녹화는 모두 이
# 버전으로 다시 썼으므로 이 버전만 읽는다.
RECORDING_VERSION = 14
FRAME_INDEX_MAGIC = b"EDGEIDX1"
EVENT_LOG_MAGIC = b"EDGELOG1"
EVENT_RECORD_HEADER = struct.Struct("<QHHI")  # timestamp_ns, type, flags, size
MAX_RECORD_PAYLOAD = 1 << 20  # EventLogReader kMaxPayloadBytes: 이보다 긴 레코드는 끊긴 꼬리다

RECORD_CAN_RX = 1
RECORD_CAN_TX = 2
RECORD_MODEL_STATE = 3
RECORD_CONTROL_STATE = 4
RECORD_PANDA_STATE = 5
RECORD_LEARNER_STATE = 6
RECORD_IMU = 7
RECORD_LOCALIZATION = 8
LAST_RECORD_TYPE = RECORD_LOCALIZATION  # 범위 밖 타입은 끊긴 꼬리다(EventLogReader와 같다)

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
CONTROL_STATE_SIZE = (CONTROL_STATE.itemsize + 7) // 8 * 8  # C++ 구조체는 8바이트 정렬이라 꼬리 패딩이 붙는다

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

# ModelState의 앞부분(frame_id ..). 머리만 볼 도구가 쓴다.
MODEL_STATE_HEAD = np.dtype([
    ("frame_id", "<u8"),
    ("capture_timestamp_ns", "<u8"),
    ("model_timestamp_ns", "<u8"),
    ("model_execution_ms", "<f4"),
    ("valid", "<u4"),
])

CALIBRATION_STATE = np.dtype([
    ("status", "<u4"),
    ("valid_blocks", "<i4"),
    ("roll", "<f4"), ("pitch", "<f4"), ("yaw", "<f4"),
    ("spread", "<f4", (3,)),
])

# ModelState.pose: 모델의 카메라 오도메트리(ipc_messages.h PoseState). model_layout()["pose"]에서 읽는다.
POSE_STATE = np.dtype([
    ("valid", "<u4"),
    ("trans", "<f4", (3,)),
    ("rot", "<f4", (3,)),
    ("trans_std", "<f4", (3,)),
    ("rot_std", "<f4", (3,)),
])

# CanRx·CanTx 레코드: RecordedCanBatchHeader(count, dropped) 뒤에 RecordedCanFrame count개(recording_format.h).
# can_frames()로 읽는다.
RECORDED_CAN_BATCH_HEAD = struct.Struct("<II")
RECORDED_CAN_FRAME = np.dtype([
    ("address", "<u4"),
    ("src", "<u4"),
    ("bus_time", "<u4"),
    ("data_len", "<u4"),
    ("flags", "<u4"),
    ("data", "u1", (64,)),
])


def can_frames(payload: bytes) -> np.ndarray:
    """CanRx·CanTx 레코드 페이로드의 프레임들(잘린 레코드면 들어 있는 만큼)."""
    count, _dropped = RECORDED_CAN_BATCH_HEAD.unpack_from(payload, 0)
    count = min(count, (len(payload) - RECORDED_CAN_BATCH_HEAD.size) // RECORDED_CAN_FRAME.itemsize)
    return np.frombuffer(payload, RECORDED_CAN_FRAME, count, RECORDED_CAN_BATCH_HEAD.size)

# ModelState.flow_calibration: 광류 보정기의 상태, 각도는 rad. sd_*는 블록 중앙값들의 표준편차다.
FLOW_CALIBRATION_STATE = np.dtype([
    ("window_frame_id", "<u8"),
    ("status", "<u4"), ("windows", "<u4"), ("blocks", "<u4"),
    ("pitch", "<f4"), ("yaw", "<f4"), ("sd_pitch", "<f4"), ("sd_yaw", "<f4"),
    ("window_pitch", "<f4"), ("window_yaw", "<f4"), ("window_rotation", "<f4"),
    ("window_tracks", "<u4"), ("window_inliers", "<u4"),
    ("mount_changes", "<u4"), ("window_parallax", "<f4"), ("estimate_blocks", "<u4"), ("rejected_windows", "<u4"),
])
assert FLOW_CALIBRATION_STATE.itemsize == 72

# ModelState.calibration_source: 워프가 쓴 보정의 출처와 포즈 보정기 자신의 추정.
CALIBRATION_SOURCE_STATE = np.dtype([
    ("source", "<u4"),       # 0 모델 포즈, 1 광류
    ("requested", "<u4"),    # 웹 설정(calibration_use_flow)
    ("flags", "<u4"),        # bit 0 광류 사용 가능, bit 1 전환 대기(결합 중), bit 2 워프 램핑 중
    ("pose_status", "<u4"), ("pose_valid_blocks", "<i4"),
    ("pose_roll", "<f4"), ("pose_pitch", "<f4"), ("pose_yaw", "<f4"),
])
assert CALIBRATION_SOURCE_STATE.itemsize == 32

# ModelState.lead_flow: 광류로 잰 앞차 확대율(model/lead_flow). 상대 속도는
# -lead_x * expansion_rate(m/s, 음수 = 가까워짐)이고 충돌까지 시간은 1 / expansion_rate다.
LEAD_FLOW_STATE = np.dtype([
    ("frame_id", "<u8"),
    ("valid", "<u4"), ("points", "<u4"), ("inliers", "<u4"),
    ("expansion_rate", "<f4"),   # 1/s, ln(크기 비) / window_s, 양수 = 가까워짐
    ("window_s", "<f4"),
    ("lead_x", "<f4"),           # 창 끝의 모델 앞차 거리(m, 카메라 기준)
])
assert LEAD_FLOW_STATE.itemsize == 32

# ModelState.lead_track: 모델 앞차 거리와 광류 확대율을 칼만 필터로 섞은 값
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
# openpilot X_IDXS: 궤적 점마다 놓인 거리. 차선·도로 경계의 점은 이 격자 위에 있어 y, z만 담는다
X_IDXS = np.array([192.0 * (i / (TRAJECTORY_SIZE - 1.0)) ** 2 for i in range(TRAJECTORY_SIZE)])

LEAD_STATE = np.dtype([("valid", "<u4"), ("probability", "<f4"), ("x", "<f4"), ("y", "<f4"),
                       ("velocity", "<f4"), ("acceleration", "<f4")])

# ModelState(ipc_messages.h) 전체. 녹화는 구조체를 그대로 담는다(필드 사이 틈이 없다).
MODEL_STATE = np.dtype([
    ("frame_id", "<u8"), ("capture_timestamp_ns", "<u8"), ("model_timestamp_ns", "<u8"),
    ("model_execution_ms", "<f4"), ("valid", "<u4"),
    ("plan", "<f4", (TRAJECTORY_SIZE, 3)),          # x 앞, y 오른쪽, z 아래
    ("lanes", "<f4", (4, TRAJECTORY_SIZE, 2)),      # 왼쪽 바깥부터, X_IDXS 위의 (y, z)
    ("lane_probabilities", "<f4", (4,)), ("lane_stds", "<f4", (4,)),
    ("road_edges", "<f4", (2, TRAJECTORY_SIZE, 2)), ("road_edge_stds", "<f4", (2,)),
    ("desire_state", "<f4", (8,)),
    ("lead", LEAD_STATE), ("pose", POSE_STATE), ("calibration", CALIBRATION_STATE),
    ("plan_yaw", "<f4", (TRAJECTORY_SIZE,)), ("plan_yaw_rate", "<f4", (TRAJECTORY_SIZE,)),
    ("camera_offset_m", "<f4"), ("camera_height_m", "<f4"),
    ("gas_press_probs", "<f4", (6,)),
    ("flow_calibration", FLOW_CALIBRATION_STATE), ("calibration_source", CALIBRATION_SOURCE_STATE),
    ("lead_flow", LEAD_FLOW_STATE), ("lead_track", LEAD_TRACK_STATE),
])
assert MODEL_STATE.itemsize == 2648
# 필드 이름 -> 페이로드 안의 바이트 위치(웹 콘솔 state_layout.py와 같은 값, check_recording_reader.py가 C++와 대조)
MODEL_STATE_LAYOUT = {name: MODEL_STATE.fields[name][1] for name in MODEL_STATE.names}


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
    model_capture_ts: np.ndarray   # 모델 상태의 촬영 시각


def read_segment_index(segment_dir: Path) -> SegmentInfo:
    data = (segment_dir / "frames.bin").read_bytes()
    magic, version, header_size, record_size, width, height, fps = struct.unpack_from(
        "<8sIIIIII", data, 0)
    if magic != FRAME_INDEX_MAGIC or version != RECORDING_VERSION:
        raise ValueError(f"{segment_dir}: frames.bin is {magic!r} v{version}, not v{RECORDING_VERSION}")
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
    """영상이 있는 세그먼트들(기록 순서). segments/가 없는 route(영상 없이 이벤트만 옮긴 것)는 빈 목록이다:
    예전에는 예외로 lane_benchmark 같은 일괄 실행 전체가 멈췄다."""
    seg_root = route_dir / "segments"
    if not seg_root.is_dir():
        return []
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
    # 디코딩된 프레임이 레코드와 정확히 짝지어지게 한다. 세그먼트마다 그 세그먼트의
    # 레코드를 먼저 붙인 뒤 넣는다: 파서는 다음 AU를 본 뒤에야 패킷을 내므로 레코드가 늘 앞선다.
    au_records: list[tuple[np.void, int]] = []
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
        for rec in segment.frames:
            payload = data[int(rec["file_offset"]):
                           int(rec["file_offset"]) + int(rec["packet_size"])]
            au_records.extend([(rec, segment.height)] * au_starts(payload))
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
    """기록 순서대로 놓은 이벤트 로그 파일(events/의 60 s 청크)."""
    return sorted((route_dir / "events").glob("*.bin"))


@dataclass
class EventRecord:
    """이벤트 로그 레코드 하나. ``payload``는 청크를 복사 없이 가리키는 뷰다."""
    path: Path
    timestamp_ns: int
    type: int
    payload: memoryview

    def _record(self, dtype: np.dtype, what: str) -> np.void:
        if len(self.payload) != dtype.itemsize:
            raise ValueError(f"{self.path}: {what} payload {len(self.payload)} != {dtype.itemsize}; the layout "
                             f"changed without a recording-version bump")
        return np.frombuffer(self.payload, dtype, count=1)[0]

    def control_state(self) -> np.void:
        if len(self.payload) != CONTROL_STATE_SIZE:
            raise ValueError(f"{self.path}: control state payload {len(self.payload)} "
                             f"!= {CONTROL_STATE_SIZE}; ControlState changed without a "
                             f"recording-version bump")
        return np.frombuffer(self.payload, CONTROL_STATE, count=1)[0]

    def learner_state(self) -> np.void:
        return self._record(LEARNER_STATE, "learner state")

    def model_state(self) -> np.void:
        return self._record(MODEL_STATE, "model state")

    def model_lanes(self) -> tuple[np.ndarray, np.ndarray]:
        """ModelState의 차선 네 개(왼쪽 바깥부터) [4, 33, 3](x = X_IDXS, y 오른쪽 +, z)와 그 확률 [4]의 사본."""
        model = self.model_state()
        points = np.empty((4, TRAJECTORY_SIZE, 3), np.float32)
        points[..., 0] = X_IDXS
        points[..., 1:] = model["lanes"]
        return points, model["lane_probabilities"].copy()

    def camera_mount(self) -> tuple[float, float]:
        """이 ModelState의 워프에 쓴 카메라 장착(오른쪽 오프셋 m, 높이 m)."""
        model = self.model_state()
        return float(model["camera_offset_m"]), float(model["camera_height_m"])

    def imu_samples(self) -> np.ndarray:
        """IMU 레코드의 샘플(칩 원래 축, 자이로 바이어스 제거 전) 사본. 머리의 count만큼 들어 있지 않으면 빈 배열."""
        if len(self.payload) >= IMU_BATCH_HEAD.size:
            count = IMU_BATCH_HEAD.unpack_from(self.payload, 0)[1]
            if IMU_BATCH_HEAD.size + count * IMU_SAMPLE.itemsize <= len(self.payload):
                return np.frombuffer(self.payload, IMU_SAMPLE, count, IMU_BATCH_HEAD.size).copy()
        return np.zeros(0, IMU_SAMPLE)


def iter_event_records(path: Path) -> Iterator[EventRecord]:
    """이벤트 청크 하나의 모든 레코드를 파일 순서대로 낸다. 끊긴 꼬리(깔끔하지 않게 멈춰 잘린
    레코드, tmpfs가 차서 0으로 채워진 구간)에서 C++ EventLogReader처럼 끝난다: 레코드가 파일을 넘거나
    타입이 범위 밖이거나 MAX_RECORD_PAYLOAD보다 길다. 머리보다 짧은 파일은 아무것도 내지 않고, 매직이나
    버전이 다르면 예외를 던진다. 레코드의 payload(와 그 위의 frombuffer)는 청크 버퍼를 가리키는 뷰라, 오래 쥘
    값은 복사한다: 뷰 하나가 청크 전체를 붙잡는다."""
    data = path.read_bytes()
    if len(data) < 16:
        return
    magic, version, header_size = struct.unpack_from("<8sII", data, 0)
    if magic != EVENT_LOG_MAGIC or version != RECORDING_VERSION:
        raise ValueError(f"{path}: event log is {magic!r} v{version}, not v{RECORDING_VERSION}")
    view = memoryview(data)
    offset, end = header_size, len(data)
    while offset + EVENT_RECORD_HEADER.size <= end:
        ts, rtype, _flags, size = EVENT_RECORD_HEADER.unpack_from(data, offset)
        offset += EVENT_RECORD_HEADER.size
        if not 1 <= rtype <= LAST_RECORD_TYPE or size > MAX_RECORD_PAYLOAD or offset + size > end:
            break
        yield EventRecord(path, ts, rtype, view[offset:offset + size])
        offset += size


def read_route_events(route_dir: Path) -> RouteEvents | None:
    paths = [p for p in route_event_files(route_dir) if p.stat().st_size > 32]
    if not paths:
        return None

    controls, control_ts, model_cts = [], [], []
    for path in paths:
        for rec in iter_event_records(path):
            if rec.type == RECORD_CONTROL_STATE:
                controls.append(rec.control_state().copy())  # 뷰는 청크 버퍼 전체를 붙잡는다
                control_ts.append(rec.timestamp_ns)
            elif rec.type == RECORD_MODEL_STATE:
                model_cts.append(np.frombuffer(rec.payload, MODEL_STATE_HEAD, count=1)[0]["capture_timestamp_ns"])

    if not model_cts and not controls:
        return None
    return RouteEvents(
        control=np.array(controls, dtype=CONTROL_STATE),
        control_log_ts=np.array(control_ts, dtype=np.uint64),
        model_capture_ts=np.array(model_cts, dtype=np.uint64),
    )

