#!/usr/bin/env python3
"""Generate a checkerboard calibration target sized for a specific display.

The camera intrinsics are solved from photos of a flat target. A TV panel
is flatter than any printed board, so the target is generated at the panel's
native pixel grid: every square is an exact integer number of pixels, which
keeps the corners on a perfect lattice as long as the TV shows the image 1:1
(no overscan, no sharpening, no scaling).

Defaults describe a 65" 16:9 4K TV. The output is 8-bit grayscale PNG written
with the stdlib only, so this runs on any host with numpy.
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
    """Write an 8-bit grayscale PNG using the Up filter for every scanline.

    Consecutive rows of a checkerboard are identical inside a square row, so the
    Up filter turns most of the image into zeros and keeps the file small.
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

    # Index each board pixel by its square, then colour the odd parity ones.
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
