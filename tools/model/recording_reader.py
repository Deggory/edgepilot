"""Reader for recordd routes (frames.bin / event log / road.hevc or road.h264).

K230 routes carry HEVC (road.hevc); MaixCAM2 routes carry H.264 (road.h264).

Binary layouts mirror src/recording/recording_format.h and src/common/ipc_messages.h. Struct sizes
are asserted against the payload sizes found in the stream, so a layout drift
fails loudly instead of decoding garbage.
"""

from __future__ import annotations

import functools
import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np

# v9 magics first; recordings up to v8 carry the earlier ones (recording_format.h kLegacy*)
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

# ControlState: natural alignment, no packing pragma; all members are
# 4/8-byte so the layout is padding-free except the trailing 8-byte pad.
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

# ImuBatch (src/common/ipc_messages.h): 16-byte head, then `count` samples (only those are recorded).
IMU_BATCH_HEAD = struct.Struct("<QII")
IMU_SAMPLE = np.dtype([
    ("timestamp_ns", "<u8"), ("accel_mps2", "<f4", 3), ("gyro_rad_s", "<f4", 3),
    ("temperature_c", "<f4"), ("reserved", "<u4"),
])
assert IMU_SAMPLE.itemsize == 40

# LocalizationState (locationd, ~20 Hz). flags bits: ipc_messages.h kLocalization*.
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

# LearnerState (paramsd/torqued output, 20 Hz). flags bits: ipc_messages.h kLearner*.
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

# The head of ModelState (frame_id ..). Only the fields these tools need are
# decoded; the plan/lane payload in the middle is skipped by offset.
# CalibrationState is the struct's last member, so it is located from the
# end of the payload.
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

TRAJECTORY_SIZE = 33
IPC_POINT = 12  # IpcPoint: three float32


@functools.lru_cache(maxsize=None)
def model_state_layout(version: int) -> dict[str, int]:
    """Byte offsets inside a ModelState payload, per recording version.

    The struct lost fields as the runtime dropped things nothing consumed, so
    a reader that hardcodes one version silently misreads the others:
      v3 and older carry stop_line (payload 4076 + 4 pad = 4080)
      v4 dropped stop_line (4048)
      v5 dropped plan_position_stds and plan_orientations (3256)
      v6 appended plan_yaw and plan_yaw_rate (3520)
      v7 appended camera_offset_m and camera_height_m (3528), the camera
         mount the warp used for that frame
      v8 appended gas_press_probs and brake_press_probs (3576), the model's
         pedal predictions at 0, 2, ..., 10 s
    Callers should check ``layout["__size__"]`` against the payload size they
    actually saw so a future layout change fails loudly instead of decoding
    garbage.
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

    layout, offset = {}, 0
    for name, size in fields:
        layout[name] = offset
        offset += size
    layout["__size__"] = offset
    return layout


@dataclass
class SegmentInfo:
    path: Path
    width: int
    height: int
    fps: int
    segment_start_ns: int
    frames: np.ndarray  # FRAME_INDEX_RECORD array


@dataclass
class RouteEvents:
    control: np.ndarray            # CONTROL_STATE records
    control_log_ts: np.ndarray     # event-header timestamps for control records
    model_frame_id: np.ndarray
    model_capture_ts: np.ndarray
    model_rpy: np.ndarray          # [N, 3] calibration roll/pitch/yaw (rad)
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
    # an unclean stop can truncate the file mid-record; keep whole records only
    count = (len(data) - header_size) // FRAME_INDEX_RECORD.itemsize
    frames = np.frombuffer(data, FRAME_INDEX_RECORD, count=count, offset=header_size)
    return SegmentInfo(segment_dir, width, height, fps, segment_start_ns, frames)


def segment_video(seg_dir: Path) -> tuple[Path, str] | None:
    """The segment's video file and its codec ("h264" or "hevc"), or None."""
    for codec in ("h264", "hevc"):
        path = seg_dir / f"road.{codec}"
        if path.exists():
            return path, codec
    return None


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
    """Yield (index_record, y, u, v) across a whole route in stream order.

    The segments of a route are 60 s slices of one continuous HEVC (K230) or
    H.264 (MaixCAM2) stream, and the K230 MVX encoder splits large access units
    (keyframes) across several
    dequeued buffers, so the per-record packet boundaries in frames.bin are not
    reliable AU boundaries. The bytes ARE in stream order though: feed them
    through one ffmpeg parser + decoder for the whole route and pair decoded
    frames with index records by order (one encoder output per index record).
    """
    import av
    import re

    codec_name = segment_video(segments[0].path)[1]
    codec = av.CodecContext.create(codec_name, "r")

    def au_starts(payload: bytes) -> int:
        """Access units starting in payload: slices with first_slice_segment_in_pic
        (HEVC) or first_mb_in_slice == 0 (H.264)."""
        count = 0
        for match in re.finditer(b"\x00\x00\x01", payload):
            pos = match.end()
            if codec_name == "hevc":
                if pos + 2 < len(payload) and ((payload[pos] >> 1) & 0x3F) <= 31 \
                        and (payload[pos + 2] >> 7) & 1:
                    count += 1
            elif pos + 1 < len(payload) and (payload[pos] & 0x1F) in (1, 5) \
                    and (payload[pos + 1] >> 7) & 1:
                count += 1
        return count

    # The MVX encoder does not keep a 1:1 mapping between dequeued buffers
    # (= index records) and access units around large keyframes, so map each
    # AU (in stream order) to the record whose byte range starts it. The
    # decoder also skips AUs with the MVX RPS/POC nonconformance, so packets
    # are tagged with their AU index as pts to keep the pairing exact.
    au_records: list[tuple[np.void, int]] = []
    for segment in segments:
        data = segment_video(segment.path)[0].read_bytes()
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
        data = segment_video(segment.path)[0].read_bytes()
        # only bytes covered by index records are trustworthy; an unclean stop
        # can leave a partially written tail
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
    """Event log files in record order: v3 rotates 60 s chunks in events/,
    v2 and older keep a single route-level events.bin."""
    chunk_dir = route_dir / "events"
    if chunk_dir.is_dir():
        return sorted(chunk_dir.glob("*.bin"))
    legacy = route_dir / "events.bin"
    return [legacy] if legacy.exists() else []


@dataclass
class EventRecord:
    """One event-log record. ``payload`` is a zero-copy view into the chunk."""
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
        """ModelState offsets for this record's version, checked against
        its size: the writer pads records to 8 bytes, so anything outside
        struct size +0..7 means the layout moved without a version bump."""
        layout = model_state_layout(self.version)
        expected = layout["__size__"]
        if not expected <= len(self.payload) <= expected + 7:
            raise ValueError(f"{self.path}: model state payload {len(self.payload)} "
                             f"does not match the v{self.version} layout "
                             f"({expected}, +0..7 pad)")
        return layout


def iter_event_records(path: Path) -> Iterator[EventRecord]:
    """Every record of one events chunk in file order; a tail truncated by an
    unclean stop ends the iteration. A file shorter than its header yields
    nothing, a wrong magic raises."""
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
    """All board IMU samples of a route (raw chip axes, gyro bias not removed), or
    None when the route has no IMU records (recorded before imud existed)."""
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
    """All locationd outputs of a route, or None when it has none (recorded before locationd)."""
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
