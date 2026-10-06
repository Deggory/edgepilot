#!/usr/bin/env python3
"""IMU 외부 회전(카메라 ← IMU) 추정: 녹화 route의 선회 구간에서 보드 자이로 회전축과 모델 pose
회전축(보정 → 기기)을 맞춘다. 결과는 src/localization/location_estimator.cc kDefaultImuExtrinsicRpy에 넣는다.

- 축 기울기 차: 우·좌회전을 합쳐 두 축의 기울기(pitch, roll) 차. x·y 각속도 바이어스는 선회
  방향에 따라 부호가 바뀌어 합치면 상쇄된다.
- Wahba(SVD): 같은 짝으로 최소제곱 회전. yaw는 수직축 선회로 관측되지 않는다.
roll은 모델 pose의 롤 성분 오차가 커서 route마다 1° 정도 흔들린다. 여러 route 평균을 쓴다.

사용: estimate_imu_extrinsic.py <route_dir>...
"""
import math
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
import recording_reader as rr  # noqa: E402

CAMODO_DELAY_S = 0.1   # locationd kCamOdoPoseDelay
MIN_YAW_RATE = 0.08    # rad/s
MIN_SPEED = 4.0        # m/s


def euler_rotate(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def load(route: Path):
    ctl_t, ctl_v, pose, imu = [], [], [], []
    for path in rr.route_event_files(route):
        if path.stat().st_size <= 32:
            continue
        for rec in rr.iter_event_records(path):
            if rec.type == rr.RECORD_CONTROL_STATE:
                c = rec.control_state()
                ctl_t.append(c["timestamp_ns"] * 1e-9)
                ctl_v.append(c["ego_speed_kph"] / 3.6)
            elif rec.type == rr.RECORD_MODEL_STATE:
                layout = rec.model_layout()
                cap, = struct.unpack_from("<Q", rec.payload, layout["capture_timestamp_ns"])
                p = struct.unpack_from("<I12f", rec.payload, layout["pose"])
                cal = struct.unpack_from("<Ii3f", rec.payload, layout["calibration"])
                if p[0] and cal[0] == 1:
                    pose.append((cap * 1e-9 - CAMODO_DELAY_S, *(euler_rotate(*cal[2:5]) @ np.array(p[4:7]))))
            elif rec.type == rr.RECORD_IMU:
                _, count, _ = rr.IMU_BATCH_HEAD.unpack_from(rec.payload, 0)
                imu.append(np.frombuffer(rec.payload, rr.IMU_SAMPLE, count, rr.IMU_BATCH_HEAD.size).copy())
    if not imu or not pose:
        raise SystemExit(f"{route}: no IMU or pose records")
    return np.array(ctl_t), np.array(ctl_v), np.array(pose), np.concatenate(imu)


def smooth(x, n):
    return np.convolve(x, np.ones(n) / n, "same")


def estimate(route: Path):
    ctl_t, ctl_v, pose, imu = load(route)
    ts = imu["timestamp_ns"] * 1e-9
    g = imu["gyro_rad_s"].astype(float)
    gyro = np.c_[-g[:, 2], g[:, 0], -g[:, 1]]  # 칩 → 기기 축(location_estimator handle_imu)
    stopped = np.interp(ts, ctl_t, ctl_v) < 0.05
    if stopped.sum() > 500:
        gyro -= gyro[stopped].mean(0)
    gyro_s = np.c_[[smooth(gyro[:, i], 52) for i in range(3)]].T       # 0.5 s
    cam = np.c_[[smooth(pose[:, i], 10) for i in (1, 2, 3)]].T          # 0.5 s
    gi = np.c_[[np.interp(pose[:, 0], ts, gyro_s[:, i]) for i in range(3)]].T
    v = np.interp(pose[:, 0], ctl_t, ctl_v)
    sel = (np.abs(cam[:, 2]) > MIN_YAW_RATE) & (np.abs(gi[:, 2]) > MIN_YAW_RATE) & (v > MIN_SPEED)

    def tilt(x):
        ax = (x * np.sign(x[:, 2])[:, None]).sum(0)
        return math.degrees(-math.atan2(ax[0], ax[2])), math.degrees(math.atan2(ax[1], ax[2]))

    gp, gr = tilt(gi[sel])
    cp, cr = tilt(cam[sel])
    u, _, vt = np.linalg.svd(cam[sel].T @ gi[sel])
    rot = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    w_roll, w_pitch = math.degrees(math.atan2(rot[2, 1], rot[2, 2])), math.degrees(-math.asin(rot[2, 0]))
    print(f"{route.name}: {sel.sum()} turning frames")
    print(f"  axis tilt difference: roll {gr - cr:+.2f} deg, pitch {gp - cp:+.2f} deg")
    print(f"  Wahba:                roll {w_roll:+.2f} deg, pitch {w_pitch:+.2f} deg")
    return (gr - cr + w_roll) / 2, (gp - cp + w_pitch) / 2


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    results = [estimate(Path(route)) for route in argv]
    roll = np.mean([r for r, _ in results])
    pitch = np.mean([p for _, p in results])
    print(f"mean over {len(results)} route(s): roll {roll:+.2f} deg, pitch {pitch:+.2f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
