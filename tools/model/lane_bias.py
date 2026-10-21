"""녹화된 주행의 횡방향 치우침을 재고 그것이 어디서 오는지 밝힌다.

횡방향 플래너는 +y가 오른쪽인 좌표계에서 laneLines[1](왼쪽)과
laneLines[2](오른쪽)의 중점을 향해 조향하므로

    offset(x) = (left_y(x) + right_y(x)) / 2

는 모델이 보기에 앞쪽 거리 x에서 차선 중앙이 카메라 기준으로 놓인 자리다.
양수는 "차선 중앙이 내 오른쪽에 있다", 곧 차가 중앙보다 왼쪽에 있다는
뜻이다.

offset(x) = a + b*x로 맞추면 이를 만드는 두 원인이 갈린다:

  a  평행 이동(단위 m). 카메라가 차량 중심선 위에 있지 않거나, 차가 실제로
     중앙에서 벗어나 있다. 카메라 내부 파라미터는 이것을 만들 수 없다: 주점이나
     초점거리 오차는 카메라를 중심으로 작용하므로 x=0에서 횡방향 효과가 정확히 0이다.
  b  회전(단위 rad/m). 보정 꼴의 항이다 -- 카메라 행렬이나 온라인 외부 파라미터에서
     온 yaw 오차. cx가 dcx 픽셀 틀리면 여기에 대략 dcx/fx로 나타난다.

그러므로 `a`가 크고 `b`가 0에 가까우면 치우침은 카메라 보정 문제가 아니고,
카메라 행렬을 다시 구해도 고쳐지지 않는다.

직진 여부는 조향각이 아니라 모델 자신의 50 m 경로로 판단한다. 조향각으로 거르는
쪽이 그럴듯해 보이지만 틀렸다: 차가 똑바로 가는 데 0이 아닌 각이 필요하면
|angle| < k 필터는 커브 분포의 한쪽만 남겨 있지도 않은 회전 항을 만들어 낸다.

사용:
    python tools/model/lane_bias.py <route_dir> [<route_dir> ...]
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from recording_reader import RECORD_CONTROL_STATE, RECORD_MODEL_STATE, X_IDXS, concat_chunks, iter_states

FIT_POINTS = 20          # 0 .. 75 m. 그 너머는 차선이 흔들린다
STRAIGHT_PLAN_Y_M = 0.5  # 48 m에서 |plan y|가 이보다 작으면 직진으로 친다
STRAIGHT_PLAN_INDEX = 16  # X_IDXS[16] = 48 m
MIN_FIT_SAMPLES = 200
MIN_SPEED_KPH = 60.0
MIN_LANE_PROB = 0.6
MAX_DRIVER_TORQUE = 50   # 운전자가 핸들을 살짝 건드린 프레임은 뺀다


@dataclass
class RouteLateral:
    """모델 레코드마다의 횡방향 값과 control 레코드의 값."""
    lane_offset: np.ndarray    # [N, TRAJECTORY_SIZE] 인식한 차선 중앙
    plan_y: np.ndarray         # [N, TRAJECTORY_SIZE] 모델 경로
    left_prob: np.ndarray
    right_prob: np.ndarray
    camera_offset: np.ndarray  # [N] 워프에 쓴 카메라 오프셋
    model_ts: np.ndarray
    control: dict[str, np.ndarray]


def read_route_lateral(route: Path) -> RouteLateral:
    models, controls = [], []
    for chunk in iter_states(route, RECORD_MODEL_STATE, RECORD_CONTROL_STATE):
        _, model = chunk[RECORD_MODEL_STATE]
        control_ts, state = chunk[RECORD_CONTROL_STATE]
        lanes = model["lanes"]  # (y, z)
        models.append({"lane_offset": (lanes[:, 1, :, 0] + lanes[:, 2, :, 0]) / 2.0,
                       "plan_y": model["plan"][:, :, 1].copy(),
                       "left_prob": model["lane_probabilities"][:, 1].astype(float),
                       "right_prob": model["lane_probabilities"][:, 2].astype(float),
                       "camera_offset": model["camera_offset_m"].astype(float),
                       "model_ts": model["capture_timestamp_ns"].copy()})
        controls.append({"ts": control_ts.astype(np.int64),
                         "active": state["active"].astype(np.int64),
                         "cluster_speed_kph": state["cluster_speed_kph"].astype(float),
                         "steer_deg": state["steering_angle_deg"].astype(float),
                         "desired_curv": state["desired_curvature"].astype(float),
                         "actual_curv": state["actual_curvature"].astype(float),
                         "driver_torque": state["driver_torque"].astype(np.int64)})
    return RouteLateral(**concat_chunks(models), control=concat_chunks(controls))


def route_offsets(route: Path) -> dict[str, float]:
    """이 주행이 시작될 때 적용되던 횡방향 튜닝. 주행의 param 스냅숏(steering.json)에서 읽는다."""
    path = route / "params" / "steering.json"
    params = json.loads(path.read_text()) if path.exists() else {}
    return {k: params[k] for k in ("path_offset_m", "angle_offset_deg") if k in params}


def analyse(route: Path) -> None:
    route = Path(route)
    data = read_route_lateral(route)
    if not len(data.model_ts) or not len(data.control["ts"]):
        print(f"{route.name}: no paired model/control records")
        return

    idx = np.clip(np.searchsorted(data.control["ts"], data.model_ts),
                  0, len(data.control["ts"]) - 1)
    ctl = {k: v[idx] for k, v in data.control.items() if k != "ts"}

    straight = np.abs(data.plan_y[:, STRAIGHT_PLAN_INDEX]) < STRAIGHT_PLAN_Y_M
    keep = (straight
            & (data.left_prob > MIN_LANE_PROB) & (data.right_prob > MIN_LANE_PROB)
            & (ctl["cluster_speed_kph"] > MIN_SPEED_KPH) & (ctl["active"] == 1)
            & (np.abs(ctl["driver_torque"]) < MAX_DRIVER_TORQUE)
            & np.isfinite(data.lane_offset).all(1))

    tuning = route_offsets(route)
    print(f"\n=== {route.name} ===")
    if tuning:
        print("  active tuning: " + "  ".join(f"{k}={v}" for k, v in tuning.items()))
    print(f"  {int(keep.sum())} straight engaged samples "
          f"(model 48 m path within {STRAIGHT_PLAN_Y_M} m, >{MIN_SPEED_KPH:.0f} km/h, "
          f"both lanes >{MIN_LANE_PROB}, no driver torque)")
    if keep.sum() < MIN_FIT_SAMPLES:
        print("  too few samples to fit")
        return

    profile = data.lane_offset[keep].mean(0)
    slope, intercept = np.polyfit(X_IDXS[:FIT_POINTS], profile[:FIT_POINTS], 1)
    print("  perceived lane-centre offset (+ = centre is to my right = I am left)")
    for j in (0, 4, 8, 12, 16, 20):
        print(f"    x={X_IDXS[j]:6.1f} m   {profile[j]:+.3f} m")
    print(f"  fit over 0-{X_IDXS[FIT_POINTS - 1]:.0f} m:")
    print(f"    translation {intercept * 100:+6.1f} cm    "
          f"<- camera intrinsics cannot cause this")
    print(f"    rotation    {slope * 1000:+6.2f} mrad  "
          f"({np.degrees(slope):+.3f} deg)  <- the calibration-shaped term")

    steer = ctl["steer_deg"][keep]
    print(f"  straight-line steering angle  mean {steer.mean():+.3f} deg  "
          f"sd {steer.std():.3f}")
    print(f"  curvature command  desired {ctl['desired_curv'][keep].mean():+.6f} 1/m"
          f"   actual {ctl['actual_curv'][keep].mean():+.6f} 1/m")
    if tuning.get("path_offset_m"):
        raw = intercept + tuning["path_offset_m"]
        print(f"  path_offset_m={tuning['path_offset_m']} is already applied, so the "
              f"uncompensated translation is about {raw * 100:+.1f} cm")
    # 워프가 모델의 시점을 카메라에서 오른쪽으로 camera_offset_m만큼 옮기므로,
    # (카메라 기준) y에 있는 차선 중앙은 모델에서 y - offset으로 읽힌다.
    used = data.camera_offset[keep]
    offset = float(used.mean())
    if used.max() - used.min() > 0.005:
        print(f"  camera_offset_m changed during the drive "
              f"({used.min():+.3f} .. {used.max():+.3f} m); using the mean")
    if offset:
        print(f"  camera_offset_m={offset:+.3f} m is in the warp: the translation above is from "
              f"the virtual camera; from the physical camera it is about "
              f"{(intercept + offset) * 100:+.1f} cm")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("routes", nargs="+", type=Path)
    for route in parser.parse_args().routes:
        analyse(route)


if __name__ == "__main__":
    main()
