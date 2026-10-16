#!/usr/bin/env python3
"""HUD 스냅샷 도우미.

  hud_tools.py inputs <route_dir> <out_dir> [--time SECONDS]
      recordd route에서 hud_snapshot 입력을 뽑는다: model.bin(현재 배치의 ModelState 원본, 옛 기록은 끝을
      0으로 채운다), control.bin(240바이트로 채운 ControlState 원본), lane.bin(같은 프레임의 LaneState 원본,
      차선선 플러그인으로 녹화했을 때), camera.png(맞는 도로 프레임).
      --time이 없으면 순간을 자동으로 고른다: 컨트롤러 활성, 40~95 km/h, 차선이 모두 확실하고, 있으면
      앞차도 있는 때.

  hud_tools.py compose <prefix> [camera.png]
      hud_snapshot의 EDGEARGB 프레임(<prefix>_<scenario>.argb, 640x480)을 PNG로 바꾼다:
      <name>_overlay.png(투명)와, 카메라 프레임이 있으면 화면처럼 프레임 가운데 4:3 위에 겹친
      <name>_composite.png.
"""
from __future__ import annotations

import argparse
import glob
import struct
import subprocess
import sys
from pathlib import Path
from typing import NamedTuple

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
from recording_reader import (LANE_STATE, RECORD_CONTROL_STATE, RECORD_LANE_STATE, RECORD_MODEL_STATE,  # noqa: E402
                              RECORDING_VERSION, iter_event_records, model_state_layout, route_event_files,
                              route_segments, segment_video)

MODEL_STATE_SIZE = model_state_layout(RECORDING_VERSION)["__size__"]  # hud_snapshot은 현재 구조체를 읽는다
CONTROL_STATE_SIZE = 240
FRAME_MAGIC = b"EDGEARGB"


# ---- 입력 ----

class ModelMoment(NamedTuple):
    ts: int
    payload: memoryview
    frame_id: int
    lanes: tuple[float, float, float, float]
    lead_valid: int


def scan(route: Path) -> tuple[list[tuple[int, np.void]], list[ModelMoment], dict[int, bytes]]:
    """route를 한 번 읽어 ControlState, ModelState 순간, 해석된 LaneState(frame_id → 원본 바이트)를 모은다."""
    controls, models, lane_states = [], [], {}
    for path in route_event_files(route):
        for rec in iter_event_records(path):
            if rec.type == RECORD_CONTROL_STATE:
                controls.append((rec.timestamp_ns, rec.control_state()))
            elif rec.type == RECORD_MODEL_STATE and rec.version >= 7:
                layout = rec.model_layout()
                frame_id, = struct.unpack_from("<Q", rec.payload, 0)
                lanes = struct.unpack_from("<4f", rec.payload, layout["lane_probabilities"])
                lead_valid, = struct.unpack_from("<I", rec.payload, layout["lead"])
                models.append(ModelMoment(rec.timestamp_ns, rec.payload, frame_id, lanes, lead_valid))
            elif rec.type == RECORD_LANE_STATE and len(rec.payload) == LANE_STATE.itemsize:
                lane = np.frombuffer(rec.payload, LANE_STATE, 1)[0]
                if lane["valid"] == 1:
                    lane_states.setdefault(int(lane["frame_id"]), bytes(rec.payload))
    return controls, models, lane_states


def score(control: np.void, model: ModelMoment) -> float:
    if control["active"] != 1:
        return -1.0
    speed = float(control["cluster_speed_kph"])
    if not 40.0 <= speed <= 95.0:
        return -1.0
    lanes = model.lanes
    value = 1.0 + 2.0 * min(lanes[1], lanes[2]) + 0.5 * (lanes[0] + lanes[3])
    value += 1.5 if control["radar_lead_valid"] else 0.0
    value += 1.0 if model.lead_valid else 0.0
    value += 0.5 if control["tpms_valid"] else 0.0
    return value


def cmd_inputs(args: argparse.Namespace) -> int:
    controls, models, lane_states = scan(args.route)
    if not controls or not models:
        sys.exit("route has no control/model records")
    control_ts = np.array([c[0] for c in controls], dtype=np.uint64)
    model_ts = np.array([m.ts for m in models], dtype=np.uint64)

    if args.time is not None:
        target = int(model_ts[0]) + int(args.time * 1e9)
        mi = int(np.clip(np.searchsorted(model_ts, target), 0, len(models) - 1))
    else:
        best = (-2.0, 0)
        for i in range(0, len(models), 3):
            j = int(np.searchsorted(control_ts, models[i].ts))
            if j >= len(controls):
                break
            value = score(controls[j][1], models[i])
            if value > best[0]:
                best = (value, i)
        mi = best[1]
    model = models[mi]
    cj = min(int(np.searchsorted(control_ts, model.ts)), len(controls) - 1)
    control = controls[cj][1]

    args.out.mkdir(parents=True, exist_ok=True)
    # v7에는 끝의 페달 예측이 없다. 현재 구조체 크기까지 0으로 채운다
    raw_model = bytes(model.payload[:MODEL_STATE_SIZE])
    (args.out / "model.bin").write_bytes(raw_model + b"\0" * (MODEL_STATE_SIZE - len(raw_model)))
    raw = control.tobytes()
    (args.out / "control.bin").write_bytes(raw + b"\0" * (CONTROL_STATE_SIZE - len(raw)))
    lane = lane_states.get(model.frame_id)
    if lane is not None:
        (args.out / "lane.bin").write_bytes(lane)
        lines = int(np.frombuffer(lane, LANE_STATE, 1)[0]["line_count"])
        print(f"lane state: frame {model.frame_id} lines={lines} -> {args.out / 'lane.bin'}")
    elif lane_states:
        print("no valid LaneState for this frame", file=sys.stderr)
    seconds = (int(model.ts) - int(model_ts[0])) / 1e9
    print(f"moment t={seconds:.1f}s frame_id={model.frame_id} speed={control['cluster_speed_kph']:.1f} "
          f"active={control['active']} lanes={np.round(model.lanes, 2)} lead={model.lead_valid}")

    if not (args.route / "segments").is_dir():
        print("no segments/ in this route copy: camera frame skipped", file=sys.stderr)
        return 0
    for segment in route_segments(args.route):
        frames = segment.frames
        hits = np.nonzero(frames["frame_id"] == model.frame_id)[0]
        if not len(hits):
            continue
        index = int(frames["encode_index"][hits[0]]) - int(frames["encode_index"][0])
        camera = args.out / "camera.png"
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(segment_video(segment.path)),
                        "-vf", f"select=eq(n\\,{index})", "-frames:v", "1", str(camera)], check=True)
        print(f"camera frame: {segment.path.name} index {index} -> {camera}")
        break
    else:
        print("camera frame not found in frames.bin", file=sys.stderr)
    return 0


# ---- 합성 ----

def load_frame(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if data[:8] != FRAME_MAGIC:
        raise ValueError(f"{path}: not an EDGEARGB file")
    width, height = struct.unpack_from("<II", data, 8)
    pixels = np.frombuffer(data, np.uint8, count=width * height * 4, offset=16)
    return pixels.reshape(height, width, 4)[..., [2, 1, 0, 3]].copy()  # BGRA를 RGBA로


def screen_camera(path: Path, size: tuple[int, int]) -> Image.Image:
    """overlayd가 보여 주는 카메라 프레임: 가운데 4:3을 화면 크기로 늘린다."""
    image = Image.open(path).convert("RGBA")
    crop_w = image.height * 4 // 3
    left = (image.width - crop_w) // 2
    return image.crop((left, 0, left + crop_w, image.height)).resize(size, Image.BILINEAR)


def cmd_compose(args: argparse.Namespace) -> int:
    frames = sorted(glob.glob(f"{args.prefix}_*.argb"))
    if not frames:
        print(f"no frames matching {args.prefix}_*.argb", file=sys.stderr)
        return 1
    camera = None
    for frame in frames:
        path = Path(frame)
        rgba = load_frame(path)
        overlay = Image.fromarray(rgba, "RGBA")
        overlay.save(path.with_name(path.stem + "_overlay.png"))
        coverage = (rgba[..., 3] > 0).mean() * 100.0
        line = f"{path.stem}: overlay coverage {coverage:.1f}%"
        if args.camera is not None:
            camera = camera or screen_camera(args.camera, overlay.size)
            composite = camera.copy()
            composite.alpha_composite(overlay)
            composite.convert("RGB").save(path.with_name(path.stem + "_composite.png"))
            line += " (+composite)"
        print(line)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    inputs = commands.add_parser("inputs")
    inputs.add_argument("route", type=Path)
    inputs.add_argument("out", type=Path)
    inputs.add_argument("--time", type=float, help="seconds from route start instead of auto-pick")
    inputs.set_defaults(run=cmd_inputs)
    compose = commands.add_parser("compose")
    compose.add_argument("prefix")
    compose.add_argument("camera", nargs="?", type=Path)
    compose.set_defaults(run=cmd_compose)
    args = parser.parse_args()
    return args.run(args)


if __name__ == "__main__":
    sys.exit(main())
