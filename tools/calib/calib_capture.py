#!/usr/bin/env python3
"""Board-side chessboard capture for camera calibration (MaixCAM2).

Opens the camera exactly as the runtime does (1280x720 NV21, 20 fps, same
channel), so the intrinsics fit the frames modeld warps. The LCD shows a live
preview with the detected corners; touch the screen to save a frame. Frames
are saved as lossless Y-plane PNGs in OUT_DIR (the chessboard only needs luma).

usage (on the board, launcher stopped):  python3 calib_capture.py [out_dir] [cols rows]
  cols rows = inner corners (default 11 6)
"""
import os
import sys
import time

import cv2
import numpy as np
from maix import app, camera, display, image, touchscreen

W, H = 1280, 720
out_dir = sys.argv[1] if len(sys.argv) > 1 else "/root/calib"
pattern = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 3 else (11, 6)
os.makedirs(out_dir, exist_ok=True)

cam = camera.Camera(W, H, image.Format.FMT_YVU420SP, fps=20)
if abs(cam.fps() - 20) > 0.1:
    cam.set_fps(20)
disp = display.Display()
ts = touchscreen.TouchScreen()
dw, dh = disp.width(), disp.height()
saved = len([f for f in os.listdir(out_dir) if f.endswith(".png")])
was_pressed = False
flash_until = 0.0

while not app.need_exit():
    frame = cam.read()
    y = np.frombuffer(frame.to_bytes(), np.uint8, W * H).reshape(H, W)
    small = cv2.resize(y, (W // 2, H // 2), interpolation=cv2.INTER_AREA)
    found, corners = cv2.findChessboardCorners(
        small, pattern, flags=cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_ADAPTIVE_THRESH)

    x, yy, pressed = ts.read()
    if pressed and not was_pressed:
        if found:
            name = os.path.join(out_dir, f"calib_{saved:03d}.png")
            cv2.imwrite(name, y)
            saved += 1
            flash_until = time.time() + 0.4
    was_pressed = pressed

    preview = cv2.cvtColor(cv2.resize(small, (dw, dh)), cv2.COLOR_GRAY2RGB)
    if found:
        sx, sy = dw / small.shape[1], dh / small.shape[0]
        pts = corners.reshape(-1, 2) * [sx, sy]
        cv2.drawChessboardCorners(preview, pattern, pts.reshape(-1, 1, 2).astype(np.float32), True)
    if time.time() < flash_until:
        preview[:] = 255 - preview
    color = (0, 255, 0) if found else (255, 60, 60)
    cv2.putText(preview, f"{'FOUND' if found else 'no board'}  saved {saved}  (touch = save)",
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    disp.show(image.cv2image(preview, bgr=False, copy=False))
