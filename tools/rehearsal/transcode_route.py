#!/usr/bin/env python3
"""Prepare a recorded route for the on-board rehearsal (k230_replayd).

The AX630C hardware decoder only takes H.264, while routes are HEVC. This
re-encodes the chosen segments to H.264 (x264, no B-frames, 1 s GOP, SPS/PPS
and an access-unit delimiter on every frame; --crf sets the quality, default 18) and rewrites each segment's frames.bin so every frame
keeps its recorded frame id and capture timestamp. The matching events/NNN.bin
chunks, manifest and params are copied as they are.

  python3 transcode_route.py ROUTE OUT --segments 1-3
"""
import argparse
import shutil
import struct
import subprocess
from pathlib import Path

INDEX_HEADER = struct.Struct("<8sIIIIIIQQ")   # K230FrameIndexHeader (48 B)
INDEX_RECORD = struct.Struct("<QQQQII")       # K230FrameIndexRecord (40 B)


def read_index(path: Path):
    data = path.read_bytes()
    header = INDEX_HEADER.unpack_from(data)
    assert header[0] == b"K230IDX1" and header[3] == INDEX_RECORD.size, path
    start = header[2]
    records = [INDEX_RECORD.unpack_from(data, start + i * INDEX_RECORD.size)
               for i in range((len(data) - start) // INDEX_RECORD.size)]
    return data[:start], records


def access_units(stream: bytes):
    """Split an Annex B stream at access-unit delimiters (NAL type 9)."""
    starts = []
    i = 0
    while True:
        i = stream.find(b"\x00\x00\x01", i)
        if i < 0:
            break
        if stream[i + 3] & 0x1F == 9:
            starts.append(i - 1 if i > 0 and stream[i - 1] == 0 else i)
        i += 3
    starts.append(len(stream))
    units = []
    for a, b in zip(starts, starts[1:]):
        unit = stream[a:b]
        key = b"\x00\x00\x01\x65" in unit or b"\x00\x00\x01\x25" in unit or \
            any(unit[j + 3] & 0x1F == 5 for j in _starts(unit))
        units.append((a, b - a, key))
    return units


def _starts(unit: bytes):
    j = unit.find(b"\x00\x00\x01")
    while j >= 0 and j + 3 < len(unit):
        yield j
        j = unit.find(b"\x00\x00\x01", j + 3)


def transcode(segment: Path, out: Path, fps: int, args_crf: str):
    subprocess.run([
        "ffmpeg", "-v", "error", "-y", "-f", "hevc", "-r", str(fps), "-i", str(segment / "road.hevc"),
        "-c:v", "libx264", "-preset", "medium", "-crf", args_crf, "-profile:v", "main",
        "-pix_fmt", "yuv420p", "-bf", "0", "-g", str(fps), "-keyint_min", str(fps),
        "-sc_threshold", "0", "-x264-params", "repeat-headers=1:aud=1",
        "-f", "h264", str(out / "road.h264"),
    ], check=True)
    header, records = read_index(segment / "frames.bin")
    units = access_units((out / "road.h264").read_bytes())
    if len(units) != len(records):
        raise SystemExit(f"{segment}: {len(units)} H.264 frames vs {len(records)} indexed")
    with open(out / "frames.bin", "wb") as f:
        f.write(header)
        for i, (rec, (offset, size, key)) in enumerate(zip(records, units)):
            frame_id, capture_ns = rec[0], rec[1]
            f.write(INDEX_RECORD.pack(frame_id, capture_ns, i, offset, size, 1 if key else 0))
    return len(records)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--segments", required=True, help="first-last segment numbers, e.g. 1-3")
    ap.add_argument("--crf", type=int, default=18, help="x264 quality (lower = better, larger)")
    args = ap.parse_args()
    first, last = (int(x) for x in args.segments.split("-"))
    fps = 20
    for n in range(first, last + 1):
        name = f"{n:03d}"
        out = args.out / "segments" / name
        out.mkdir(parents=True, exist_ok=True)
        frames = transcode(args.route / "segments" / name, out, fps, str(args.crf))
        (args.out / "events").mkdir(exist_ok=True)
        chunk = args.route / "events" / f"{name}.bin"
        if chunk.exists():
            shutil.copy2(chunk, args.out / "events" / chunk.name)
        print(f"segment {name}: {frames} frames")
    for extra in ("manifest.json", "params"):
        src = args.route / extra
        if src.is_dir():
            shutil.copytree(src, args.out / extra, dirs_exist_ok=True)
        elif src.exists():
            shutil.copy2(src, args.out / extra)


if __name__ == "__main__":
    main()
