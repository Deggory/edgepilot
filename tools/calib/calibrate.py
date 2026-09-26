"""Chessboard intrinsics from the MaixCAM2 camera-app photos (PC side).

usage: calibrate.py <photo_dir> [cols rows square_mm]   (default 11 6 100)
Writes intrinsics.json with K and distortion at the photo resolution and
scaled to the runtime 1280x720 frame (valid only if both share the same field
of view, which is checked separately).
"""
import glob, json, sys
import numpy as np, cv2

photo_dir = sys.argv[1]
cols, rows, sq = (int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (11, 6, 100.0)
objp = np.zeros((cols * rows, 3), np.float32)
objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * sq
obj, img, names, size = [], [], [], None
crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
for f in sorted(glob.glob(f"{photo_dir}/*.jpg") + glob.glob(f"{photo_dir}/*.png"), key=lambda p: int(''.join(c for c in p.split('/')[-1] if c.isdigit()) or 0)):
    g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
    size = g.shape[::-1]
    ok, c = cv2.findChessboardCornersSB(g, (cols, rows), flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok:
        print("no board:", f); continue
    obj.append(objp); img.append(c.astype(np.float32)); names.append(f)
print(f"boards found in {len(img)} photos at {size}")

def run(flags):
    return cv2.calibrateCameraExtended(obj, img, size, None, None, flags=flags, criteria=crit)

rms, K, D, rv, tv, _, _, per = run(cv2.CALIB_RATIONAL_MODEL * 0)
print(f"pinhole+k1k2p1p2k3: rms {rms:.3f}px")
# drop outlier views (> 2x median error) once and refit
per = per.ravel(); keep = per < max(2 * np.median(per), 0.5)
if not keep.all():
    print("dropping", [names[i].split('/')[-1] for i in np.where(~keep)[0]], "per-view", np.round(per[~keep], 2))
    obj = [o for o, k in zip(obj, keep) if k]; img = [i for i, k in zip(img, keep) if k]
    rms, K, D, rv, tv, _, _, per = run(0)
    print(f"refit on {len(img)} views: rms {rms:.3f}px")
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
s = 1280.0 / size[0]
hfov = 2 * np.degrees(np.arctan(size[0] / 2 / fx)); vfov = 2 * np.degrees(np.arctan(size[1] / 2 / fy))
print(f"K @ {size}: fx {fx:.1f} fy {fy:.1f} cx {cx:.1f} cy {cy:.1f}  (HFOV {hfov:.1f} deg, VFOV {vfov:.1f} deg)")
print(f"dist k1 k2 p1 p2 k3: {np.round(D.ravel(), 5)}")
print(f"scaled to 1280x720 (x{s:.4f}): fx {fx*s:.2f} fy {fy*s:.2f} cx {cx*s:.2f} cy {cy*s:.2f}")
# coverage: where corners landed
allc = np.concatenate([i.reshape(-1, 2) for i in img]); 
print("corner coverage x %.0f..%.0f of %d, y %.0f..%.0f of %d" % (allc[:,0].min(), allc[:,0].max(), size[0], allc[:,1].min(), allc[:,1].max(), size[1]))
json.dump(dict(photo_size=list(size), rms_px=rms, views=len(img), K=K.tolist(), dist=D.ravel().tolist(),
               runtime_1280x720=dict(fx=fx*s, fy=fy*s, cx=cx*s, cy=cy*s), square_mm=sq, pattern=[cols, rows]),
          open("intrinsics.json", "w"), indent=1)
