#!/usr/bin/env python3
"""Preview what the model sees from a MaixCAM2 frame.

Warps a 1280x720 NV21 capture with the runtime's fixed-point warp
(tools/model/model_warp.py) and the calibrated MaixCAM2 intrinsics, then writes
one PNG: the source frame with the medmodel/sbigmodel footprints and the
model's vanishing point, plus both 512x256 model views. On a level road the
vanishing point should sit on the horizon; the remaining offset is what online
calibration (rpy) absorbs.

usage: warp_preview.py <frame_nv21.bin> <out.png> [roll pitch yaw (deg)]
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
from model_warp import ModelInputWarp, maixcam2_intrinsics, projection_matrix  # noqa: E402

W, H = 1280, 720
K720 = maixcam2_intrinsics(W, H)
MODEL_CY = 47.6                                         # kDefaultModelCy (medmodel)


def yuv6_to_y(yuv6):
    y = np.empty((256, 512), np.uint8)
    y[0::2, 0::2], y[1::2, 0::2], y[0::2, 1::2], y[1::2, 1::2] = yuv6[0], yuv6[1], yuv6[2], yuv6[3]
    return y


def main():
    parser = argparse.ArgumentParser(description="Preview what the model sees from a MaixCAM2 frame.")
    parser.add_argument("frame", help="1280x720 NV21 capture")
    parser.add_argument("out", help="PNG to write")
    parser.add_argument("rpy", nargs="*", type=float, metavar="DEG",
                        help="calibration roll pitch yaw in degrees (default 0 0 0)")
    args = parser.parse_args()
    if len(args.rpy) not in (0, 3):
        parser.error("give roll, pitch and yaw together")
    out = args.out
    rpy = np.radians(args.rpy) if args.rpy else np.zeros(3)
    d = np.fromfile(args.frame, np.uint8)
    y = d[:W * H].reshape(H, W)
    vu = d[W * H:].reshape(H // 2, W // 2, 2)
    u, v = vu[..., 1], vu[..., 0]                       # NV21: V first
    views, canvas = [], cv2.cvtColor(y, cv2.COLOR_GRAY2BGR)
    for sbig, color in ((False, (0, 200, 0)), (True, (0, 140, 255))):
        warp = ModelInputWarp(W, H, rpy.astype(np.float32), sbig=sbig, intrinsics=K720)
        views.append(cv2.cvtColor(yuv6_to_y(warp.warp(y, u, v)), cv2.COLOR_GRAY2BGR))
        P = projection_matrix(*K720, rpy.astype(np.float32), sbig)   # model px -> source px
        border = np.array([[x, 0] for x in range(0, 513, 32)] + [[512, yy] for yy in range(0, 257, 32)] +
                          [[x, 256] for x in range(512, -1, -32)] + [[0, yy] for yy in range(256, -1, -32)], np.float64)
        pts = np.c_[border, np.ones(len(border))] @ P.T
        pts = (pts[:, :2] / pts[:, 2:]).astype(np.int32)
        cv2.polylines(canvas, [pts], True, color, 2)
    # vanishing point = medmodel principal point; map it into the sbig view too
    p_med = projection_matrix(*K720, rpy.astype(np.float32), False).astype(np.float64)
    p_sbig = projection_matrix(*K720, rpy.astype(np.float32), True).astype(np.float64)
    vp_h = p_med @ np.array([256.0, MODEL_CY, 1.0])
    vp = (vp_h[:2] / vp_h[2]).astype(int)
    cv2.drawMarker(canvas, tuple(int(c) for c in vp), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
    cv2.line(canvas, (0, int(vp[1])), (W - 1, int(vp[1])), (0, 0, 255), 1)
    s_h = np.linalg.solve(p_sbig, vp_h)
    for view, row in ((views[0], MODEL_CY), (views[1], s_h[1] / s_h[2])):
        cv2.line(view, (0, int(round(row))), (511, int(round(row))), (0, 0, 255), 1)
    cv2.putText(canvas, "green: medmodel  orange: sbigmodel  red: model horizon", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    bottom = np.hstack([cv2.resize(views[0], (640, 320)), cv2.resize(views[1], (640, 320))])
    cv2.imwrite(out, np.vstack([canvas, bottom]))
    print("vanishing point in source:", vp.tolist(), "rpy deg:", np.degrees(rpy).round(2).tolist())


if __name__ == "__main__":
    main()
