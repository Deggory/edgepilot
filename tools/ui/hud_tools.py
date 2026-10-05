#!/usr/bin/env python3
"""HUD snapshot helpers.

  hud_tools.py inputs <route_dir> <out_dir> [--time SECONDS]
      Extract hud_snapshot inputs from a recordd route: model.bin (raw
      ModelState in the current v8 layout, 3576 bytes; a v7 record is
      zero-filled at the end), control.bin (raw
      ControlState padded to 240 bytes) and camera.png (the matching road
      frame). Without --time the moment is chosen automatically: controller
      active, 40-95 km/h, all lane lines confident, a lead if any.

  hud_tools.py compose <prefix> [camera.png]
      Turn hud_snapshot K230ARGB frames (<prefix>_<scenario>.argb, 640x480)
      into PNGs: <name>_overlay.png (transparent) and, with a camera frame,
      <name>_composite.png over the centre 4:3 of the frame, as the screen
      shows it.
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
from recording_reader import (RECORD_CONTROL_STATE, RECORD_MODEL_STATE,  # noqa: E402
                              iter_event_records, model_state_layout,
                              route_event_files, route_segments, segment_video)

MODEL_STATE_SIZE = model_state_layout(8)["__size__"]  # hud_snapshot reads the current struct
CONTROL_STATE_SIZE = 240
FRAME_MAGIC = b"K230ARGB"


# ---- inputs ----

class ModelMoment(NamedTuple):
    ts: int
    payload: memoryview
    frame_id: int
    lanes: tuple[float, float, float, float]
    lead_valid: int


def scan(route: Path) -> tuple[list[tuple[int, np.void]], list[ModelMoment]]:
    controls, models = [], []
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
    return controls, models


def score(control: np.void, model: ModelMoment) -> float:
    if control["active"] != 1:
        return -1.0
    speed = float(control["speed_kph"])
    if not 40.0 <= speed <= 95.0:
        return -1.0
    lanes = model.lanes
    value = 1.0 + 2.0 * min(lanes[1], lanes[2]) + 0.5 * (lanes[0] + lanes[3])
    value += 1.5 if control["radar_lead_valid"] else 0.0
    value += 1.0 if model.lead_valid else 0.0
    value += 0.5 if control["tpms_valid"] else 0.0
    return value


def cmd_inputs(args: argparse.Namespace) -> int:
    controls, models = scan(args.route)
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
    # v7 has no pedal predictions at the end; zero-fill them to the current struct
    raw_model = bytes(model.payload[:MODEL_STATE_SIZE])
    (args.out / "model.bin").write_bytes(raw_model + b"\0" * (MODEL_STATE_SIZE - len(raw_model)))
    raw = control.tobytes()
    (args.out / "control.bin").write_bytes(raw + b"\0" * (CONTROL_STATE_SIZE - len(raw)))
    seconds = (int(model.ts) - int(model_ts[0])) / 1e9
    print(f"moment t={seconds:.1f}s frame_id={model.frame_id} speed={control['speed_kph']:.1f} "
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
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(segment_video(segment.path)[0]),
                        "-vf", f"select=eq(n\\,{index})", "-frames:v", "1", str(camera)], check=True)
        print(f"camera frame: {segment.path.name} index {index} -> {camera}")
        break
    else:
        print("camera frame not found in frames.bin", file=sys.stderr)
    return 0


# ---- compose ----

def load_frame(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if data[:8] != FRAME_MAGIC:
        raise ValueError(f"{path}: not a K230ARGB file")
    width, height = struct.unpack_from("<II", data, 8)
    pixels = np.frombuffer(data, np.uint8, count=width * height * 4, offset=16)
    return pixels.reshape(height, width, 4)[..., [2, 1, 0, 3]].copy()  # BGRA -> RGBA


def screen_camera(path: Path, size: tuple[int, int]) -> Image.Image:
    """The camera frame as overlayd shows it: the centre 4:3 scaled to the screen."""
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
