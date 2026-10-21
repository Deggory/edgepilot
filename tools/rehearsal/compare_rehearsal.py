#!/usr/bin/env python3
"""보드 리허설 녹화를 그것이 재생한 route와 비교한다.

리허설(replayd)은 녹화된 route의 카메라와 CAN을 MaixCAM2 런타임에 넣고, 런타임은 자기 route를 따로
녹화한다. 두 route의 CAN 속도가 같으므로 speed_kph의 상호상관으로 MaixCAM2 제어 상태를 원본에 맞춘다.
원본 주행에서 횡제어가 활성이던 100 Hz 제어 틱마다 다음을 보고한다:

- 결합 일치(engaged/active),
- 요구 곡률: 상관, 평균 |차이|, p95 |차이|,
- 리허설이 보냈을 토크(그림자)와 주행 때 보낸 토크,
- 제어 루프와 모델 주기.

  python3 compare_rehearsal.py ORIGINAL_ROUTE REHEARSAL_ROUTE
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
import recording_reader as rr  # noqa: E402


def load(route: Path):
    """제어 틱의 시각(첫 틱부터 s, 이벤트 머리 시각)과 ControlState들, 모델 상태의 촬영 시각."""
    parts = []
    for chunk in rr.iter_states(route, rr.RECORD_CONTROL_STATE, rr.RECORD_MODEL_STATE):
        control_ts, control = chunk[rr.RECORD_CONTROL_STATE]
        _, model = chunk[rr.RECORD_MODEL_STATE]
        parts.append({"t": control_ts, "control": control, "model_t": model["capture_timestamp_ns"].copy()})
    v = rr.concat_chunks(parts)
    if not len(v["control"]):
        raise SystemExit(f"{route}: no control states")
    t = (v["t"].astype(np.int64) - int(v["t"][0])) / 1e9
    return t, v["control"], v["model_t"]


def resample(t, values, grid):
    return np.interp(grid, t, values.astype(np.float64))


def align(t_ref, v_ref, t_new, v_new):
    """t_new에 더하면 속도 곡선이 기준과 맞는 시간 차(s)와 그때의 평균 |속도 차|. 짧은 쪽 곡선을 긴 쪽 위에서
    밀어 본다: 리허설이 원본보다 길면(앞뒤에 더 녹화된 구간) 예전에는 탐색 범위가 비어 시간 차 0을 말없이 냈다."""
    if t_new[-1] - t_new[0] > t_ref[-1] - t_ref[0]:
        lag, err = align(t_new, v_new, t_ref, v_ref)
        return -lag, err
    dt = 0.05
    grid_new = np.arange(t_new[0], t_new[-1], dt)
    sn = resample(t_new, v_new, grid_new)
    best = (np.inf, 0.0)
    lags = np.arange(t_ref[0] - grid_new[0], t_ref[-1] - grid_new[-1] + dt / 2, dt)
    for lag in lags:
        sr = resample(t_ref, v_ref, grid_new + lag)
        err = np.mean(np.abs(sr - sn))
        if err < best[0]:
            best = (err, lag)
    return best[1], best[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("original", type=Path)
    parser.add_argument("rehearsal", type=Path)
    args = parser.parse_args()

    t_ref, c_ref, _ = load(args.original)
    t_new, c_new, model_ts = load(args.rehearsal)
    lag, speed_err = align(t_ref, c_ref["cluster_speed_kph"], t_new, c_new["cluster_speed_kph"])
    tn = t_new + lag
    inside = (tn >= t_ref[0]) & (tn <= t_ref[-1])
    tn, c_new = tn[inside], c_new[inside]
    idx = np.clip(np.searchsorted(t_ref, tn), 0, len(t_ref) - 1)
    ref = c_ref[idx]

    print(f"alignment: rehearsal t + {lag:.2f} s = original t, speed |err| {speed_err:.2f} kph")
    dur = tn[-1] - tn[0]
    print(f"overlap: {dur:.1f} s, {len(tn)} control ticks ({len(tn) / dur:.1f} Hz)")
    if len(model_ts) > 1:
        mt = (model_ts[-1] - model_ts[0]) / 1e9
        print(f"model states: {len(model_ts)} ({len(model_ts) / mt:.2f} Hz)")

    for field in ("engaged", "active"):
        a, b = ref[field].astype(bool), c_new[field].astype(bool)
        print(f"{field}: original {a.mean():.1%}, MaixCAM2 {b.mean():.1%}, agree {np.mean(a == b):.1%}")

    act = ref["active"].astype(bool) & c_new["active"].astype(bool)
    if act.sum() < 10:
        print("too few ticks where both were active to compare control")
        return
    for field, unit in (("desired_curvature", "1/m"), ("apply_torque", ""), ("desired_torque", "")):
        a = ref[field][act].astype(np.float64)
        b = c_new[field][act].astype(np.float64)
        d = np.abs(a - b)
        corr = np.corrcoef(a, b)[0, 1] if a.std() > 0 and b.std() > 0 else float("nan")
        print(f"{field} (both active, {act.sum()} ticks): corr {corr:.3f}, "
              f"mean|d| {d.mean():.5f}{unit}, p95|d| {np.percentile(d, 95):.5f}{unit}, "
              f"original |x| mean {np.abs(a).mean():.5f}")
    blocks = c_new["active_block"][~c_new["active"].astype(bool) & ref["active"].astype(bool)]
    if len(blocks):
        names, counts = np.unique(blocks, return_counts=True)
        top = sorted(zip(counts, names), reverse=True)[:5]
        print("MaixCAM2 inactive while original active, block reasons:",
              ", ".join(f"{name}:{count}" for count, name in top))


if __name__ == "__main__":
    main()
