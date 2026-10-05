#!/usr/bin/env python3
"""Build an SCNV12R1 replay of recorded frames for modeld replay mode on the board.

modeld reads the file instead of the camera ring (EDGEPILOT_REPLAY_NV12) and runs the same GDC
warp, temporal plumbing and axmodel as live. See docs/diagnostics.md for the board side.

Usage:
  python make_replay.py --route ROUTE --out DIR --frames 100 --skip 400
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np

from recording_reader import decode_route_yuv, route_segments

MAGIC = b"SCNV12R1"


def nv12_bytes(y: np.ndarray, u: np.ndarray, v: np.ndarray) -> bytes:
    """One decoded frame (planar Y, U, V) as NV12: Y plane then interleaved UV."""
    nv12 = np.empty(y.size + u.size + v.size, np.uint8)
    nv12[: y.size] = y.reshape(-1)
    uv = nv12[y.size:].reshape(y.shape[0] // 2, y.shape[1])
    uv[:, 0::2] = u
    uv[:, 1::2] = v
    return nv12.tobytes()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--skip", type=int, default=400)
    args = parser.parse_args()

    route = Path(args.route)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    segments = route_segments(route)
    if not segments:
        raise SystemExit(f"{route}: no usable segments")
    width, height = segments[0].width, segments[0].height

    frames = []
    for index, (_, y, u, v) in enumerate(decode_route_yuv(segments)):
        if index < args.skip:
            continue
        frames.append(nv12_bytes(y, u, v))
        if len(frames) >= args.frames:
            break
    if len(frames) < args.frames:
        raise SystemExit(f"route yielded only {len(frames)} frames")

    replay = out_dir / "replay.scnv12"
    with open(replay, "wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<III", width, height, len(frames)))
        for frame in frames:
            f.write(frame)
    print(f"replay: {len(frames)} frames {width}x{height} -> {replay} "
          f"({replay.stat().st_size / 1e6:.0f} MB)")
    (out_dir / "replay_meta.json").write_text(json.dumps({
        "route": str(route), "skip": args.skip, "frames": len(frames),
    }, indent=2))


if __name__ == "__main__":
    main()
