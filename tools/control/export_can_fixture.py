#!/usr/bin/env python3
"""녹화 events/NNN.bin 하나를 gtest_lateral_controller의 EDGECAN1 픽스처로 내보낸다.

수신 CAN(CanRx) 레코드의 차 프레임만 담고, 타임스탬프는 첫 프레임을 0으로 다시 잰다. controlsd의
is_vehicle_rx_frame(src/car/vehicle_can.h)처럼 Panda가 돌려준 우리 송신 에코·거부 프레임(flags != 0), CAN FD 길이,
가상 버스(src > 7)는 뺀다: 픽스처에는 flags가 없어 넣으면 차가 보낸 프레임으로 읽힌다.
사용: export_can_fixture.py <events/NNN.bin> <out.can>
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
from recording_reader import RECORD_CAN_RX, can_frames, iter_event_records  # noqa: E402

FIXTURE_MAGIC = b"EDGECAN1"
FIXTURE_RECORD = struct.Struct("<QIBB8s2x")  # 열: timestamp_us, address, bus, len, data, pad


def export(src: Path, dst: Path) -> tuple[int, float]:
    frames: list[tuple[int, int, int, int, bytes]] = []
    for rec in iter_event_records(src):
        if rec.type != RECORD_CAN_RX:
            continue
        for frame in can_frames(rec.payload):
            if frame["flags"] == 0 and frame["data_len"] <= 8 and frame["src"] <= 7:
                frames.append((rec.timestamp_ns, int(frame["address"]), int(frame["src"]), int(frame["data_len"]),
                               frame["data"][:8].tobytes()))
    if not frames:
        raise ValueError(f"{src}: no CAN frames")
    start_ns = frames[0][0]
    out = bytearray(FIXTURE_MAGIC + struct.pack("<IIQ", 1, FIXTURE_RECORD.size, len(frames)))
    for ts, address, bus, length, raw in frames:
        out += FIXTURE_RECORD.pack((ts - start_ns) // 1000, address, bus, length, raw)
    dst.write_bytes(out)
    return len(frames), (frames[-1][0] - start_ns) / 1e9


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("events", type=Path, help="events/NNN.bin")
    parser.add_argument("out", type=Path, help="out.can")
    args = parser.parse_args()
    count, duration_s = export(args.events, args.out)
    print(f"{args.out}: {count} frames, {duration_s:.3f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
