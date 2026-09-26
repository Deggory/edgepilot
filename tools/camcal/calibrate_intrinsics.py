#!/usr/bin/env python3
"""Solve MaixCAM2 camera intrinsics from checkerboard photos taken with k230_camcal.

The runtime warps model input with a plain pinhole intrinsic (fx, fy, cx, cy) and
applies no distortion correction, so the headline result is those four numbers at
the capture resolution. The distortion coefficients are still solved and reported,
because they say how much residual the pinhole-only warp is accepting.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def find_corners(gray: np.ndarray, pattern: tuple[int, int]) -> np.ndarray | None:
    """Detect inner corners, preferring the sector-based detector.

    findChessboardCornersSB is more accurate and needs no separate subpixel pass;
    the classic detector is kept as a fallback for frames it refuses.
    """
    found, corners = cv2.findChessboardCornersSB(
        gray, pattern, flags=cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_EXHAUSTIVE)
    if found:
        return corners

    found, corners = cv2.findChessboardCorners(
        gray, pattern,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
    return corners


def per_view_errors(obj_points, img_points, rvecs, tvecs, camera_matrix,
                    dist) -> np.ndarray:
    errors = []
    for i, objp in enumerate(obj_points):
        projected, _ = cv2.projectPoints(objp, rvecs[i], tvecs[i], camera_matrix, dist)
        delta = img_points[i].reshape(-1, 2) - projected.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(delta ** 2, axis=1)))))
    return np.array(errors)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("images", type=Path, nargs="+",
                        help="image files, or directories to scan for *.png/*.jpg")
    parser.add_argument("--corners", default="17x8",
                        help="inner corner count as COLSxROWS (default 17x8)")
    parser.add_argument("--square-mm", type=float, default=67.45,
                        help="physical square size; only scales the extrinsics")
    parser.add_argument("--max-view-error", type=float, default=0.0,
                        help="if set, drop views above this reprojection error and re-solve")
    parser.add_argument("--rational", action="store_true",
                        help="also fit the 8-coefficient rational model for comparison")
    parser.add_argument("--json", type=Path, help="write the result as JSON")
    args = parser.parse_args()

    cols, rows = (int(value) for value in args.corners.lower().split("x"))
    pattern = (cols, rows)

    paths: list[Path] = []
    for entry in args.images:
        if entry.is_dir():
            paths += sorted(p for p in entry.iterdir()
                            if p.suffix.lower() in (".png", ".jpg", ".jpeg"))
        else:
            paths.append(entry)
    if not paths:
        raise SystemExit("no images found")

    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square_mm

    obj_points: list[np.ndarray] = []
    img_points: list[np.ndarray] = []
    used: list[Path] = []
    size: tuple[int, int] | None = None

    print(f"pattern {cols}x{rows} inner corners, square {args.square_mm} mm")
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            print(f"  {path.name}  unreadable")
            continue
        if size is None:
            size = (image.shape[1], image.shape[0])
        elif (image.shape[1], image.shape[0]) != size:
            raise SystemExit(f"{path.name} is {image.shape[1]}x{image.shape[0]}, "
                             f"expected {size[0]}x{size[1]}")

        corners = find_corners(image, pattern)
        if corners is None:
            print(f"  {path.name}  no board")
            continue
        obj_points.append(objp)
        img_points.append(corners)
        used.append(path)

    print(f"detected {len(used)}/{len(paths)} views at {size[0]}x{size[1]}")
    if len(used) < 4:
        raise SystemExit("need at least 4 detected views")

    def solve(flags: int = 0):
        return cv2.calibrateCamera(obj_points, img_points, size, None, None, flags=flags)

    rms, camera_matrix, dist, rvecs, tvecs = solve()
    errors = per_view_errors(obj_points, img_points, rvecs, tvecs, camera_matrix, dist)

    if args.max_view_error > 0:
        keep = errors <= args.max_view_error
        if not keep.all():
            dropped = [used[i].name for i in np.flatnonzero(~keep)]
            print(f"dropping {len(dropped)} views over {args.max_view_error} px: "
                  f"{', '.join(dropped)}")
            obj_points = [obj_points[i] for i in np.flatnonzero(keep)]
            img_points = [img_points[i] for i in np.flatnonzero(keep)]
            used = [used[i] for i in np.flatnonzero(keep)]
            rms, camera_matrix, dist, rvecs, tvecs = solve()
            errors = per_view_errors(obj_points, img_points, rvecs, tvecs,
                                     camera_matrix, dist)

    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    coefficients = dist.ravel()

    print(f"\nviews {len(used)}  corners {len(used) * cols * rows}  "
          f"reprojection RMS {rms:.4f} px")
    print(f"  fx {fx:10.4f}   fy {fy:10.4f}")
    print(f"  cx {cx:10.4f}   cy {cy:10.4f}")
    print(f"  cx offset from image centre {cx - size[0] / 2:+.2f} px, "
          f"cy {cy - size[1] / 2:+.2f} px")
    names = ["k1", "k2", "p1", "p2", "k3"]
    print("  " + "  ".join(f"{n} {v:+.6f}" for n, v in zip(names, coefficients)))

    # Runtime constants (src/app_config.h kCamera*, at 1920x1080) scaled to this size.
    sx, sy = size[0] / 1920.0, size[1] / 1080.0
    runtime = {"fx": 1131.24 * sx, "fy": 1130.85 * sy, "cx": 940.13 * sx, "cy": 552.60 * sy}
    print("  vs runtime kCamera*: " + "  ".join(
        f"{k} {v - runtime[k]:+.2f} px ({(v - runtime[k]) / runtime[k] * 100:+.2f}%)"
        for k, v in (("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy))))

    hfov = 2 * np.degrees(np.arctan(size[0] / 2 / fx))
    vfov = 2 * np.degrees(np.arctan(size[1] / 2 / fy))
    print(f"  field of view {hfov:.2f} deg horizontal, {vfov:.2f} deg vertical")

    order = np.argsort(errors)[::-1]
    print("\nworst views by reprojection error")
    for i in order[:5]:
        print(f"  {used[i].name}  {errors[i]:.4f} px")
    print(f"  median {np.median(errors):.4f} px, best {errors.min():.4f} px")

    # How much the pinhole-only input warp gives up by ignoring distortion.
    grid = np.array([[[x, y]] for y in np.linspace(0, size[1] - 1, 25)
                     for x in np.linspace(0, size[0] - 1, 25)], np.float32)
    undistorted = cv2.undistortPoints(grid, camera_matrix, dist, P=camera_matrix)
    shift = np.linalg.norm(undistorted.reshape(-1, 2) - grid.reshape(-1, 2), axis=1)
    print(f"\ndistortion ignored by the pinhole warp: "
          f"max {shift.max():.2f} px, mean {shift.mean():.2f} px over the frame")

    if args.rational:
        rms_r, K_r, dist_r, rv_r, tv_r = solve(cv2.CALIB_RATIONAL_MODEL)
        print(f"\nrational model: RMS {rms_r:.4f} px  "
              f"fx {K_r[0,0]:.4f} fy {K_r[1,1]:.4f} "
              f"cx {K_r[0,2]:.4f} cy {K_r[1,2]:.4f}")

    if args.json:
        args.json.write_text(json.dumps({
            "width": size[0], "height": size[1],
            "views": len(used), "reprojection_rms_px": rms,
            "fx": fx, "fy": fy, "cx": cx, "cy": cy,
            "distortion": {n: float(v) for n, v in zip(names, coefficients)},
            "square_mm": args.square_mm,
            "pattern_corners": [cols, rows],
            "images": [p.name for p in used],
        }, indent=2) + "\n")
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
