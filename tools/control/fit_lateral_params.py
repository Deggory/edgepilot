#!/usr/bin/env python3
"""controlsd 텍스트 로그에서 토크-횡가속 관계를 회귀한다(openpilot torqued의 근사).

  fit  controlsd 텍스트 로그(1 Hz hz= 라인)에서 토크-횡가속 관계를 회귀한다.
       -> torque_lat_accel_factor, torque_friction, torque_lat_accel_offset
       근사다: 1 Hz 표본이고 torqued의 버킷·적합을 따로 옮긴 것이며, 로그 줄 형식(torque=, curveYaw,
       lat=)을 정규식으로 읽어 그 줄이 바뀌면 조용히 깨진다. 녹화가 있으면 실제 torqued를 돌리는
       diagnostics/replay_lateral_learners를 쓴다.

조향 지연(steer_actuator_delay)은 보드 lagd와 같은 코드를 녹화에 돌리는 diagnostics/replay_localization으로
잰다. 예전의 lag 하위 명령은 조향각 곡률로 재서 lagd(요레이트)보다 약 0.1 s 짧게 냈으므로 없앴다.

사용 예:
  python3 fit_lateral_params.py fit /tmp/edgepilot.log
"""

import argparse
import re
import sys

import numpy as np

# openpilot torqued 상수
MIN_SPEED_KPH = 54.0          # torqued MIN_VEL 15 m/s와 같다
STEER_MIN_THRESHOLD = 0.02
FRICTION_FACTOR = 1.5         # 잔차 표준편차의 ~85% 커버
STEER_BUCKET_BOUNDS = [(-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1), (-0.1, 0.0),
                       (0.0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5)]
MIN_BUCKET_POINTS = [10, 30, 50, 50, 50, 50, 30, 10]  # qlog 스케일(/10)
STEER_MAX = 384.0
TORQUE_OUTPUT_SIGN = -1       # controlsd kTorqueOutputSign: 보낸 토크(좌측 양수) = 부호 × 제어 출력(우측 양수)

HZ_RE = re.compile(r"controlsd: hz=")


def _field(line, key, cast=float):
    m = re.search(re.escape(key) + r"=(-?[\d.]+)", line)
    return cast(m.group(1)) if m else None


def parse_log_samples(paths, min_speed_kph, driver_max, steer_max):
    """hz= 라인에서 (정규화 토크, 실측 횡가속) 표본을 뽑는다."""
    samples = []
    skipped = {"inactive": 0, "driver": 0, "desire": 0, "slow": 0, "small": 0, "saturated": 0, "no_lat": 0}
    for path in paths:
        with open(path, "r", errors="replace") as f:
            for line in f:
                if not HZ_RE.search(line):
                    continue
                if _field(line, "active", int) != 1:
                    skipped["inactive"] += 1
                    continue
                if abs(_field(line, "driver", int)) >= driver_max:
                    skipped["driver"] += 1
                    continue
                if _field(line, "desire", int) != 0:
                    skipped["desire"] += 1
                    continue
                # 제어가 쓰는 휠 속도. 끊긴 틱은 wheel=nan이라 숫자로 읽히지 않는다.
                speed = _field(line, "wheel")
                if speed is None or speed < min_speed_kph:
                    skipped["slow"] += 1
                    continue
                # ESP12 실측 횡가속(openpilot torqued의 offset 정의와 같다). 끊긴 틱은 nan이다.
                lat_accel = _field(line, "lat")
                if lat_accel is None:
                    skipped["no_lat"] += 1
                    continue
                # 랙에 실제로 걸린 값은 적용 토크다. 곡률 관례(우측 양수)로 되돌린다.
                applied = int(re.search(r"torque=-?\d+/(-?\d+)", line).group(1))
                torque_norm = TORQUE_OUTPUT_SIGN * applied / steer_max
                if abs(torque_norm) < STEER_MIN_THRESHOLD:
                    skipped["small"] += 1
                    continue
                if abs(torque_norm) >= 1.0:
                    skipped["saturated"] += 1
                    continue
                samples.append((torque_norm, lat_accel))
    return np.array(samples), skipped


def bucket_and_fit(samples):
    """torqued의 total-least-squares 회귀. 버킷으로 좌우 균형을 강제한다."""
    buckets = {b: [] for b in STEER_BUCKET_BOUNDS}
    for x, y in samples:
        for lo, hi in STEER_BUCKET_BOUNDS:
            if lo <= x < hi:
                buckets[(lo, hi)].append((x, y))
                break
    counts = {b: len(v) for b, v in buckets.items()}
    lacking = [(b, c, m) for (b, c, m) in
               ((b, counts[b], m) for b, m in zip(STEER_BUCKET_BOUNDS, MIN_BUCKET_POINTS))
               if c < m]

    pts = np.array([p for v in buckets.values() for p in v])
    if len(pts) < 100:
        return None, counts, lacking
    # torqued estimate_params와 동일: [x, 1, y] SVD
    mat = np.column_stack([pts[:, 0], np.ones(len(pts)), pts[:, 1]])
    _, _, v = np.linalg.svd(mat, full_matrices=False)
    slope, offset = -v.T[0:2, 2] / v.T[2, 2]
    # torqued slope2rot에서 부호를 유지한다. |slope|로 만들면 slope<0에서
    # 잔차에 기울기 성분이 남아 friction이 부풀려진다.
    sin = slope / np.sqrt(slope**2 + 1)
    cos = 1.0 / np.sqrt(slope**2 + 1)
    rot = np.array([[cos, -sin], [sin, cos]])
    spread = (np.column_stack([pts[:, 0], pts[:, 1]]) @ rot)[:, 1]
    friction = float(np.std(spread) * FRICTION_FACTOR)
    return (float(slope), float(offset), friction), counts, lacking


def cmd_fit(args):
    samples, skipped = parse_log_samples(args.logs, args.min_speed,
                                         args.driver_max, args.steer_max)
    print(f"{len(samples)} samples  (skipped: " +
          " ".join(f"{k}={v}" for k, v in skipped.items() if v) + ")")
    if len(samples) < 100:
        print("Too few samples: more engaged driving at speed is needed.")
        return 1
    result, counts, lacking = bucket_and_fit(samples)
    print("Buckets:")
    for (lo, hi), c in counts.items():
        print(f"  [{lo:+.1f},{hi:+.1f})  {c}")
    if lacking:
        print("warning: these buckets are short, so the fit can lean to one side:")
        for (lo, hi), c, m in lacking:
            print(f"  [{lo:+.1f},{hi:+.1f})  {c} < {m}")
    if result is None:
        print("No fit: too few usable samples.")
        return 1
    slope, offset, friction = result
    print()
    print(f"latAccelFactor        {slope:+.3f} m/s^2 per unit torque")
    print(f"latAccelOffset        {offset:+.4f} m/s^2")
    print(f"frictionCoefficient   {friction:.4f} m/s^2")
    print()
    print("As parameters (steering.json):")
    print(f"  torque_lat_accel_factor    {slope:.2f}")
    print(f"  torque_lat_accel_offset    {offset:+.3f}")
    print(f"  torque_friction            {friction:.3f}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fit", help="torque to lateral acceleration fit from the controlsd log (approximate; "
                                   "with a recording use replay_lateral_learners)")
    f.add_argument("logs", nargs="+")
    f.add_argument("--min-speed", type=float, default=MIN_SPEED_KPH,
                   help="km/h, default 54 (openpilot MIN_VEL)")
    f.add_argument("--driver-max", type=int, default=50)
    f.add_argument("--steer-max", type=float, default=STEER_MAX,
                   help="controlsd steer_max")
    f.set_defaults(func=cmd_fit)
    args = ap.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
