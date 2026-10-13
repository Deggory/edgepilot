#!/usr/bin/env python3
"""특정 화면 크기에 맞춘 체커보드 보정 표적을 만든다.

카메라 내부 파라미터는 평평한 표적을 찍은 사진으로 푼다. TV 패널은 어떤 인쇄 보드보다 평평하므로 표적을
패널의 원래 화소 격자에 맞춰 만든다. 칸마다 정확히 정수 개의 화소라서, TV가 영상을 1:1로 보여 주는 한
(오버스캔, 샤프닝, 확대 없음) 코너가 완벽한 격자 위에 놓인다.

기본값은 65인치 16:9 4K TV다. 출력은 표준 라이브러리만으로 쓰는 8비트 회색조 PNG라, numpy만 있으면 어느
호스트에서나 돈다.
"""

from __future__ import annotations

import argparse
import struct
import sys
import zlib
from pathlib import Path

import numpy as np

MM_PER_INCH = 25.4


def write_png_gray(path: Path, image: np.ndarray, pixels_per_meter: int | None) -> None:
    """모든 주사선에 Up 필터를 써서 8비트 회색조 PNG를 쓴다.

    체커보드는 한 칸 줄 안에서 이웃한 행이 똑같으므로, Up 필터가 영상 대부분을 0으로 만들어 파일이 작다.
    """
    if image.dtype != np.uint8 or image.ndim != 2:
        raise ValueError("image must be 2-D uint8")
    height, width = image.shape

    previous = np.vstack([np.zeros((1, width), np.uint8), image[:-1]])
    delta = (image.astype(np.int16) - previous.astype(np.int16)).astype(np.uint8)
    scanlines = np.hstack([np.full((height, 1), 2, np.uint8), delta])

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + tag + payload +
                struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    parts = [b"\x89PNG\r\n\x1a\n",
             chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))]
    if pixels_per_meter:
        parts.append(chunk(b"pHYs", struct.pack(">IIB", pixels_per_meter,
                                                pixels_per_meter, 1)))
    parts.append(chunk(b"IDAT", zlib.compress(scanlines.tobytes(), 9)))
    parts.append(chunk(b"IEND", b""))
    path.write_bytes(b"".join(parts))


def build_checkerboard(width: int, height: int, squares_x: int, squares_y: int,
                       square_px: int, dark: int, light: int) -> np.ndarray:
    board_w = squares_x * square_px
    board_h = squares_y * square_px
    if board_w > width or board_h > height:
        raise SystemExit(f"board {board_w}x{board_h} does not fit in {width}x{height}")

    image = np.full((height, width), light, np.uint8)
    offset_x = (width - board_w) // 2
    offset_y = (height - board_h) // 2

    # 보드 화소마다 속한 칸을 정하고, 홀수 패리티 칸을 칠한다.
    ys = (np.arange(board_h) // square_px)[:, None]
    xs = (np.arange(board_w) // square_px)[None, :]
    squares = np.where(((ys + xs) & 1) == 0, dark, light).astype(np.uint8)
    image[offset_y:offset_y + board_h, offset_x:offset_x + board_w] = squares
    return image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--width", type=int, default=3840, help="panel width in pixels")
    parser.add_argument("--height", type=int, default=2160, help="panel height in pixels")
    parser.add_argument("--diagonal-inches", type=float, default=65.0,
                        help="panel diagonal, used only to report the physical square size")
    parser.add_argument("--squares-x", type=int, default=18)
    parser.add_argument("--squares-y", type=int, default=9)
    parser.add_argument("--square-px", type=int, default=180)
    parser.add_argument("--dark", type=int, default=16,
                        help="dark square level; not 0, to stay out of the panel's clipping range")
    parser.add_argument("--light", type=int, default=235,
                        help="light square and quiet-zone level")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    if args.square_px < 8:
        raise SystemExit("--square-px must be at least 8")
    for name in ("squares_x", "squares_y"):
        if getattr(args, name) < 3:
            raise SystemExit(f"--{name.replace('_', '-')} must be at least 3")

    image = build_checkerboard(args.width, args.height, args.squares_x,
                               args.squares_y, args.square_px, args.dark, args.light)

    board_w = args.squares_x * args.square_px
    board_h = args.squares_y * args.square_px
    margin_x = (args.width - board_w) // 2
    margin_y = (args.height - board_h) // 2

    diagonal_px = (args.width ** 2 + args.height ** 2) ** 0.5
    mm_per_px = args.diagonal_inches * MM_PER_INCH / diagonal_px
    pixels_per_meter = int(round(1000.0 / mm_per_px))

    write_png_gray(args.out, image, pixels_per_meter)

    corners_x = args.squares_x - 1
    corners_y = args.squares_y - 1
    print(f"wrote {args.out} {args.width}x{args.height} "
          f"({args.out.stat().st_size / 1024:.0f} KiB)")
    print(f"  board          {board_w}x{board_h} px, "
          f"quiet zone {margin_x}x{margin_y} px "
          f"({margin_x / args.square_px:.2f}x{margin_y / args.square_px:.2f} squares)")
    print(f"  inner corners  {corners_x}x{corners_y} "
          f"-> findChessboardCorners(img, ({corners_x}, {corners_y}))")
    print(f"  square         {args.square_px} px = "
          f"{args.square_px * mm_per_px:.2f} mm on a {args.diagonal_inches:g}\" panel")
    print(f"  panel          {args.width * mm_per_px:.1f} x "
          f"{args.height * mm_per_px:.1f} mm, {mm_per_px:.5f} mm/px")

    if margin_x < args.square_px or margin_y < args.square_px:
        print("  WARNING: the quiet zone is under one square; corner detection "
              "needs a clear border around the board", file=sys.stderr)
    if (corners_x % 2) == (corners_y % 2):
        print("  WARNING: inner corner counts have the same parity, so the board "
              "is 180-degree ambiguous; make one dimension odd and the other even",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
