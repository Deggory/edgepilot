#!/usr/bin/env python3
"""Compare an on-board rehearsal recording with the route it replayed.

The rehearsal (replayd) feeds a recorded route's camera and CAN to the
MaixCAM2 runtime, which records its own route. Both routes carry the same CAN
speed, so the MaixCAM2 control states are aligned to the original by
cross-correlating speed_kph. Reported per 100 Hz control tick, over the ticks
the original drive had lateral control active:

- engagement agreement (engaged/active),
- desired curvature: correlation, mean |difference|, p95 |difference|
  (the MaixCAM2 runs the openpilot master model, the K230 ran v0.9.4, so part
  of the difference is the model),
- apply torque the MaixCAM2 would have sent (shadow) vs what the K230 sent,
- control loop and model rates.

  python3 compare_rehearsal.py ORIGINAL_ROUTE REHEARSAL_ROUTE
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
import recording_reader as rr  # noqa: E402


def load(route: Path):
    events = rr.read_route_events(route)
    if events is None or len(events.control) == 0:
        raise SystemExit(f"{route}: no control states")
    c = events.control
    t = (events.control_log_ts.astype(np.int64) - int(events.control_log_ts[0])) / 1e9
    return t, c, events


def resample(t, values, grid):
    return np.interp(grid, t, values.astype(np.float64))


def align(t_ref, v_ref, t_new, v_new, max_lag_s=None):
    """Offset (s) to add to t_new so its speed trace lines up with the reference."""
    dt = 0.05
    grid_new = np.arange(t_new[0], t_new[-1], dt)
    sn = resample(t_new, v_new, grid_new)
    best = (np.inf, 0.0)
    lags = np.arange(t_ref[0] - grid_new[0], t_ref[-1] - grid_new[-1], dt)
    for lag in lags:
        sr = resample(t_ref, v_ref, grid_new + lag)
        err = np.mean(np.abs(sr - sn))
        if err < best[0]:
            best = (err, lag)
    return best[1], best[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("original", type=Path)
    ap.add_argument("rehearsal", type=Path)
    args = ap.parse_args()

    t_ref, c_ref, _ = load(args.original)
    t_new, c_new, ev_new = load(args.rehearsal)
    lag, speed_err = align(t_ref, c_ref["speed_kph"], t_new, c_new["speed_kph"])
    tn = t_new + lag
    inside = (tn >= t_ref[0]) & (tn <= t_ref[-1])
    tn, c_new = tn[inside], c_new[inside]
    idx = np.clip(np.searchsorted(t_ref, tn), 0, len(t_ref) - 1)
    ref = c_ref[idx]

    print(f"alignment: rehearsal t + {lag:.2f} s = original t, speed |err| {speed_err:.2f} kph")
    dur = tn[-1] - tn[0]
    print(f"overlap: {dur:.1f} s, {len(tn)} control ticks ({len(tn) / dur:.1f} Hz)")
    if len(ev_new.model_capture_ts) > 1:
        mt = (ev_new.model_capture_ts[-1] - ev_new.model_capture_ts[0]) / 1e9
        print(f"model states: {len(ev_new.model_capture_ts)} ({len(ev_new.model_capture_ts) / mt:.2f} Hz)")

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
              ", ".join(f"{n}:{k}" for k, n in top))


if __name__ == "__main__":
    main()
