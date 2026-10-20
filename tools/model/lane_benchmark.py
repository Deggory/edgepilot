"""순정 카메라 차선(LKA14)과 모델 차선을 주행마다 비교하고, 둘이 크게 갈린 장면을 영상에서 뽑는다.

K7 순정 카메라는 버스 2의 0x484 LKA14로 좌우 차선마다 품질(0..3), 위치(c0, m, 오른쪽이 +)와
방향각(c1, rad)을 70 ms마다 보낸다(신호 정의는 사용자 DBC). 모델 자차 차선(laneLines[1], [2])에는
0..50 m에서 3차식을 맞추고 앞 X0_M에서의 위치와 기울기를 구해 같은 두 값으로 만든다.

정렬(2026-10-15, 6개 주행 121분):
  - X0_M = 1.5 m에서 방향각 상관이 가장 높다. 위치 상관은 -2..2 m에서 거의 같다.
  - 순정 값은 모델 캡처 시각보다 늦다: 위치 0.3 s, 방향각 0.1 s.

비교하는 프레임: 순정 품질 3, 모델 차선 확률 0.7 초과, 30 km/h 이상. 차선 변경 중이거나 방향지시등이
켜진 때, 차선을 밟은 때(어느 쪽이든 c0가 0.7 m 안)와 넘는 순간(c0가 한 프레임에 1 m 넘게 뜀)은 앞뒤 2 s를
뺀다.

순정 카메라도 정답은 아니다. 지표는 두 인식의 일치도이고, 크게 갈린 장면은 영상으로 어느 쪽이 맞는지
본다:
  gap          둘 다 차선을 또렷이 보는데 위치 차이가 그 주행의 중앙값에서 0.25 m 넘게 0.5 s 이상 벗어남
  camera_only  순정만 봄: 순정 품질 3인데 모델 확률 0.3 미만이 1 s 이상
  model_only   모델만 봄: 모델 확률 0.7 초과인데 순정 품질 1 이하가 1 s 이상

장면마다 그 순간의 영상 프레임에 모델 차선(주황 실선)과 순정 차선(파랑 점선: 1.5 m의 위치와 방향을
직선으로 늘린 것)을 그린다. 프레임은 ffmpeg로 앞 키프레임부터 디코딩한다.

사용:
    python tools/model/lane_benchmark.py <route_dir> [<route_dir> ...] --out DIR [--scenes N] [--no-frames]

DIR/<route>/report.json에 지표와 장면을, DIR/<route>/scenes/*.jpg에 프레임을, DIR/scorecard.json에 주행별
지표를 쓰고 표를 출력한다.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from model_warp import maixcam2_intrinsics, rotation_from_rpy
from recording_reader import (RECORD_CAN_RX, RECORD_CONTROL_STATE, RECORD_MODEL_STATE, TRAJECTORY_SIZE, X_IDXS,
                              RouteVideo, concat_chunks, iter_states)

LKA14_ADDRESS, CAMERA_BUS = 0x484, 2
ESP12_ADDRESS, VEHICLE_BUS = 0x220, 0
HUD_LANE_CHANGING = 1 << 5  # ipc_messages.h kHudFlagLaneChanging
MODEL_HZ = 20

FIT_MAX_X_M = 50.0
X0_M = 1.5
CAMERA_POS_LAG_S = 0.3
CAMERA_HEAD_LAG_S = 0.1

MIN_SPEED_KPH = 30.0
MIN_LANE_PROB = 0.7
GUARD_S = 2.0          # 차선 변경·지시등·차선 밟기 앞뒤로 빼는 시간
CROSSING_JUMP_M = 1.0  # 한 프레임에 c0가 이만큼 뛰면 차선을 넘은 것(왼쪽 선이 오른쪽 선이 된다)
STRADDLE_M = 0.7       # 어느 쪽이든 c0가 이보다 가까우면 차가 차선을 밟고 있다(K7 폭 1.87 m)
STRAIGHT_MIN_SPEED_KPH = 50.0
STRAIGHT_MAX_STEER_DEG = 3.0
STRAIGHT_MAX_YAW_RATE_DPS = 0.6

GAP_M = 0.25
GAP_MIN_S = 0.5
ONE_SIDED_MIN_S = 1.0
MODEL_LOST_PROB = 0.3
CAMERA_LOST_QUALITY = 1
MERGE_S = 2.0          # 이보다 짧게 끊긴 구간은 한 장면으로 잇는다
SCENE_SPACING_S = 10.0
SERIES_HALF_S = 6.0
DRAW_MIN_X_M = 5.5     # 이보다 가까운 도로는 보닛에 가린다

SIDES = ("left", "right")
KINDS = ("gap", "camera_only", "model_only")


@dataclass
class RouteData:
    lka_t: np.ndarray       # LKA14 프레임의 로그 시각(ns)
    lka_raw: np.ndarray     # LKA14 8바이트(리틀 엔디언 u64)
    yaw_t: np.ndarray       # ESP12 요레이트의 로그 시각(ns)
    yaw_rate: np.ndarray    # deg/s
    model_t: np.ndarray     # 모델 프레임의 캡처 시각(ns)
    model_log_t: np.ndarray
    frame_id: np.ndarray
    lanes: np.ndarray       # [n, 2, 33, 3] 자차 좌우 차선 x, y, z(아래가 +)
    lane_prob: np.ndarray   # [n, 2]
    rpy: np.ndarray         # [n, 3] 워프에 쓴 보정(rad)
    camera_offset: np.ndarray
    cal_source: np.ndarray  # 0 모델 포즈, 1 광류
    control_t: np.ndarray
    speed_kph: np.ndarray
    steer_deg: np.ndarray
    hud_flags: np.ndarray
    blinker: np.ndarray


def read_route(route: Path) -> RouteData:
    parts = []
    for chunk in iter_states(route, RECORD_CAN_RX, RECORD_MODEL_STATE, RECORD_CONTROL_STATE):
        frame_t, frames = chunk[RECORD_CAN_RX]
        raw = frames["data"][:, :8].copy().view("<u8")[:, 0]  # 8바이트 리틀 엔디언
        received = frames["flags"] == 0
        lka = received & (frames["address"] == LKA14_ADDRESS) & (frames["src"] == CAMERA_BUS)
        esp = received & (frames["address"] == ESP12_ADDRESS) & (frames["src"] == VEHICLE_BUS)
        model_t, model = chunk[RECORD_MODEL_STATE]
        lanes = np.empty((len(model), 2, TRAJECTORY_SIZE, 3))
        lanes[..., 0] = X_IDXS.astype(np.float32)  # 녹화에 없는 x(X_IDXS 격자)를 차선 점으로 되붙인다
        lanes[..., 1:] = model["lanes"][:, 1:3]
        cal = model["calibration"]
        control_t, state = chunk[RECORD_CONTROL_STATE]
        parts.append({
            "lka_t": frame_t[lka].astype(np.int64), "lka_raw": raw[lka],
            "yaw_t": frame_t[esp].astype(np.int64),
            "yaw_rate": ((raw[esp] >> np.uint64(40)) & np.uint64(0x1FFF)) * 0.01 - 40.95,
            "model_t": model["capture_timestamp_ns"].astype(np.int64), "model_log_t": model_t.astype(np.int64),
            "frame_id": model["frame_id"].astype(np.int64), "lanes": lanes,
            "lane_prob": model["lane_probabilities"][:, 1:3].astype(np.float64),
            "rpy": np.stack([cal["roll"], cal["pitch"], cal["yaw"]], 1).astype(np.float64),
            "camera_offset": model["camera_offset_m"].astype(np.float64),
            "cal_source": model["calibration_source"]["source"].astype(np.int64),
            "control_t": control_t.astype(np.int64), "speed_kph": state["ego_speed_kph"].astype(np.float64),
            "steer_deg": state["steering_angle_deg"].astype(np.float64),
            "hud_flags": state["hud_flags"].astype(np.int64),
            "blinker": (state["left_blinker"] != 0) | (state["right_blinker"] != 0),
        })
    return RouteData(**concat_chunks(parts))


def decode_lka14(raw: np.ndarray) -> dict[str, np.ndarray]:
    """LKA14 신호(사용자 DBC). 좌는 비트 0부터, 우는 비트 32부터 같은 배치다."""
    def bits(start: int, size: int) -> np.ndarray:
        return ((raw >> np.uint64(start)) & np.uint64((1 << size) - 1)).astype(np.int64)

    out = {}
    for side, base in (("left", 0), ("right", 32)):
        position = bits(base + 8, 10)
        out[side + "_quality"] = bits(base, 2)
        out[side + "_position"] = np.where(position >= 512, position - 1024, position) / 256.0
        out[side + "_heading"] = bits(base + 18, 13) * 1.52588e-5 - 0.0625
    return out


def lane_position_heading(lanes_y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """[..., 33] 차선 y에 0..FIT_MAX_X_M 3차식을 맞춰 앞 X0_M의 위치(m)와 기울기(rad)를 낸다."""
    use = X_IDXS <= FIT_MAX_X_M
    # x를 FIT_MAX_X_M으로 나눠 조건수를 낮춘다
    solve = np.linalg.pinv(np.vander(X_IDXS[use] / FIT_MAX_X_M, 4, increasing=True))
    c = lanes_y[..., use] @ solve.T
    s = X0_M / FIT_MAX_X_M
    position = c[..., 0] + c[..., 1] * s + c[..., 2] * s ** 2 + c[..., 3] * s ** 3
    slope = (c[..., 1] + 2 * c[..., 2] * s + 3 * c[..., 3] * s ** 2) / FIT_MAX_X_M
    return position, np.arctan(slope)


def near(t: np.ndarray, hit: np.ndarray, half_s: float) -> np.ndarray:
    """hit인 시각에서 half_s 안에 드는 프레임."""
    hit_t = t[hit]
    if not len(hit_t):
        return np.zeros(len(t), bool)
    i = np.searchsorted(hit_t, t)
    before = hit_t[np.clip(i - 1, 0, len(hit_t) - 1)]
    after = hit_t[np.clip(i, 0, len(hit_t) - 1)]
    return (np.abs(t - before) <= half_s) | (np.abs(after - t) <= half_s)


@dataclass
class Aligned:
    """모델 프레임마다 맞춘 두 쪽 값. [n, 2] 배열의 둘째 축이 좌, 우다."""
    t: np.ndarray           # 주행 시작부터 s
    camera_pos: np.ndarray
    camera_head: np.ndarray
    camera_quality: np.ndarray
    model_pos: np.ndarray
    model_head: np.ndarray
    speed_kph: np.ndarray
    steer_deg: np.ndarray
    yaw_rate: np.ndarray
    valid: np.ndarray       # 두 쪽 값이 다 있고 움직이며 차선 변경·지시등·차선 밟기에서 멀다
    compared: np.ndarray    # valid이고 둘 다 차선을 또렷이 본다
    straight: np.ndarray


def align(data: RouteData) -> Aligned:
    lka = decode_lka14(data.lka_raw)
    pos_t = data.model_t + int(CAMERA_POS_LAG_S * 1e9)
    head_t = data.model_t + int(CAMERA_HEAD_LAG_S * 1e9)
    in_range = (pos_t >= data.lka_t[0]) & (pos_t <= data.lka_t[-1])
    camera_pos = np.stack([np.interp(pos_t, data.lka_t, lka[s + "_position"]) for s in SIDES], 1)
    camera_head = np.stack([np.interp(head_t, data.lka_t, lka[s + "_heading"]) for s in SIDES], 1)
    # 품질은 앞뒤 프레임이 모두 3일 때만 3 근처로 보간된다
    camera_quality = np.stack([np.interp(pos_t, data.lka_t, lka[s + "_quality"].astype(float)) for s in SIDES], 1)
    model_pos, model_head = lane_position_heading(data.lanes[..., 1])

    k = np.clip(np.searchsorted(data.control_t, data.model_log_t), 0, len(data.control_t) - 1)
    speed, steer = data.speed_kph[k], data.steer_deg[k]
    yaw_rate = np.interp(data.model_log_t, data.yaw_t, data.yaw_rate) if len(data.yaw_t) else np.zeros(len(k))
    t = (data.model_t - data.model_t[0]) / 1e9

    crossing = (np.abs(camera_pos) < STRADDLE_M).any(1) | (np.abs(model_pos) < STRADDLE_M).any(1)
    crossing[1:] |= (np.abs(np.diff(camera_pos, axis=0)) > CROSSING_JUMP_M).any(1) | \
                    (np.abs(np.diff(model_pos, axis=0)) > CROSSING_JUMP_M).any(1)
    lane_change = ((data.hud_flags[k] & HUD_LANE_CHANGING) != 0) | data.blinker[k]
    guard = near(t, crossing | lane_change, GUARD_S)
    valid = (in_range & np.isfinite(model_pos).all(1) & np.isfinite(model_head).all(1) & (speed > MIN_SPEED_KPH)
             & ~guard)
    compared = valid & (camera_quality > 2.5).all(1) & (data.lane_prob > MIN_LANE_PROB).all(1)
    straight = (compared & (speed > STRAIGHT_MIN_SPEED_KPH) & (np.abs(steer) < STRAIGHT_MAX_STEER_DEG)
                & (np.abs(yaw_rate) < STRAIGHT_MAX_YAW_RATE_DPS))
    return Aligned(t, camera_pos, camera_head, camera_quality, model_pos, model_head, speed, steer, yaw_rate,
                   valid, compared, straight)


def metrics(data: RouteData, a: Aligned) -> dict:
    c, s = a.compared, a.straight
    out = {"minutes": float(a.t[-1] / 60.0), "compared_min": float(c.sum() / MODEL_HZ / 60),
           "straight_min": float(s.sum() / MODEL_HZ / 60)}
    if c.sum() < 200:
        return out
    flow = data.cal_source[c].mean()
    out["calibration"] = "flow" if flow > 0.5 else "pose"
    out["cal_yaw_deg"] = float(np.degrees(np.median(data.rpy[c, 2])))
    out["cal_pitch_deg"] = float(np.degrees(np.median(data.rpy[c, 1])))
    for i, side in enumerate(SIDES):
        cp, mp = a.camera_pos[c, i], a.model_pos[c, i]
        ch, mh = a.camera_head[c, i], a.model_head[c, i]
        d = cp - mp
        out[side] = {
            "pos_bias_cm": float(d.mean() * 100), "pos_rms_cm": float(np.sqrt(np.mean(d ** 2)) * 100),
            "pos_corr": float(np.corrcoef(cp - cp.mean(), mp - mp.mean())[0, 1]),
            "head_bias_deg": float(np.degrees((ch - mh).mean())), "head_corr": float(np.corrcoef(ch, mh)[0, 1]),
        }
    width = (a.camera_pos[c, 1] - a.camera_pos[c, 0]) - (a.model_pos[c, 1] - a.model_pos[c, 0])
    centre = (a.camera_pos[c].sum(1) - a.model_pos[c].sum(1)) / 2
    out["centre_bias_cm"] = float(centre.mean() * 100)
    out["width_diff_cm"] = float(np.median(width) * 100)
    if s.sum() > 100:
        out["straight_head_diff_deg"] = float(np.degrees((a.camera_head[s].mean(1) - a.model_head[s].mean(1)).mean()))
    return out


def runs(mask: np.ndarray, t: np.ndarray, merge_s: float) -> list[tuple[int, int]]:
    """mask가 참인 구간 [시작, 끝]. merge_s보다 짧게 끊긴 곳은 잇는다."""
    edges = np.flatnonzero(np.diff(np.concatenate([[0], mask.astype(np.int8), [0]])))
    out: list[tuple[int, int]] = []
    for start, end in zip(edges[::2], edges[1::2] - 1):
        if out and t[start] - t[out[-1][1]] < merge_s:
            out[-1] = (out[-1][0], end)
        else:
            out.append((start, end))
    return out


def find_scenes(data: RouteData, a: Aligned, per_kind: dict[str, int], has_video: np.ndarray | None) -> list[dict]:
    """종류마다 점수가 큰 장면을 서로 SCENE_SPACING_S 넘게 떨어지게 고른다. has_video가 있으면 영상이 남은
    프레임에서만 대표 프레임을 고른다."""
    candidates = []
    for i, side in enumerate(SIDES):
        diff = a.camera_pos[:, i] - a.model_pos[:, i]
        median = float(np.median(diff[a.compared])) if a.compared.any() else 0.0
        off = diff - median
        masks = {
            "gap": (a.compared & (np.abs(off) > GAP_M), GAP_MIN_S),
            "camera_only": (a.valid & (a.camera_quality[:, i] > 2.5) & (data.lane_prob[:, i] < MODEL_LOST_PROB),
                            ONE_SIDED_MIN_S),
            "model_only": (a.valid & (a.camera_quality[:, i] < CAMERA_LOST_QUALITY + 0.5)
                           & (data.lane_prob[:, i] > MIN_LANE_PROB), ONE_SIDED_MIN_S),
        }
        for kind, (mask, min_s) in masks.items():
            for start, end in runs(mask, a.t, MERGE_S):
                # 길이는 조건이 실제로 참인 시간이다. 이어 붙인 구간 길이(end - start)에는 짧은 틈이 들어가, 깜빡이는
                # 짧은 불일치가 '1 s 이상' 장면으로 뽑혔다(2026-10-14 route의 오른쪽 gap: 구간 1.3 s, 참 0.1 s).
                duration = float(mask[start:end + 1].sum()) / MODEL_HZ
                if duration < min_s:
                    continue
                inside = np.arange(start, end + 1)[mask[start:end + 1]]
                if has_video is not None:
                    inside = inside[has_video[inside]]
                    if not len(inside):
                        continue
                if kind == "gap":
                    peak = int(inside[np.argmax(np.abs(off[inside]))])
                    score = abs(off[peak])
                else:
                    # 이어 붙인 구간의 가운데에 가장 가까운, 조건을 만족하는 프레임
                    peak, score = int(inside[np.argmin(np.abs(inside - (start + end) / 2))]), duration
                candidates.append({"kind": kind, "side": side, "start_s": float(a.t[start]),
                                   "end_s": float(a.t[end]), "duration_s": duration, "peak": peak,
                                   "score": float(score), "diff_m": float(diff[peak]), "route_median_m": median})
    chosen: list[dict] = []
    for kind in KINDS:
        picked = 0
        for scene in sorted((c for c in candidates if c["kind"] == kind), key=lambda c: -c["score"]):
            if picked == per_kind[kind]:
                break
            if all(abs(a.t[scene["peak"]] - a.t[o["peak"]]) >= SCENE_SPACING_S for o in chosen):
                chosen.append(scene)
                picked += 1
    for scene in chosen:
        p = scene["peak"]
        i = SIDES.index(scene["side"])
        scene.update({"t_s": float(a.t[p]), "frame_id": int(data.frame_id[p]), "speed_kph": float(a.speed_kph[p]),
                      "camera_quality": float(a.camera_quality[p, i]), "model_prob": float(data.lane_prob[p, i]),
                      "series": series(data, a, p)})
    return sorted(chosen, key=lambda s: s["t_s"])


def series(data: RouteData, a: Aligned, peak: int) -> dict:
    """장면 앞뒤 SERIES_HALF_S의 두 쪽 위치, 10 Hz."""
    window = np.flatnonzero(np.abs(a.t - a.t[peak]) <= SERIES_HALF_S)[::2]

    def values(x: np.ndarray, digits: int = 3) -> list:
        return [None if not np.isfinite(v) else round(float(v), digits) for v in x]

    out = {"t": values(a.t[window] - a.t[peak], 2), "compared": a.compared[window].astype(int).tolist()}
    for i, side in enumerate(SIDES):
        out[side] = {"camera_pos": values(a.camera_pos[window, i]), "model_pos": values(a.model_pos[window, i]),
                     "camera_quality": values(a.camera_quality[window, i], 1),
                     "model_prob": values(data.lane_prob[window, i], 2)}
    return out


def project(points: np.ndarray, rpy: np.ndarray, camera_offset: float, k: tuple) -> np.ndarray:
    """모델 좌표(x 앞, y 오른쪽, z 아래, 가상 카메라 기준) [n, 3] -> 화소 [n, 2]. 카메라 뒤는 nan."""
    fx, fy, cx, cy = k
    p = points.astype(np.float64).copy()
    p[:, 1] += camera_offset  # 가상 카메라가 오프셋만큼 오른쪽이라 모델은 실제 점을 y - 오프셋으로 본다
    device = p @ rotation_from_rpy(*rpy).T
    view = device[:, [1, 2, 0]]
    uv = np.stack([fx * view[:, 0] / view[:, 2] + cx, fy * view[:, 1] / view[:, 2] + cy], 1)
    uv[view[:, 2] < 1.0] = np.nan
    return uv


MODEL_COLOR = (235, 104, 52)
CAMERA_COLOR = (57, 135, 229)


def draw_scene(rgb: np.ndarray, data: RouteData, a: Aligned, scene: dict, label: str, path: Path) -> None:
    from PIL import Image, ImageDraw, ImageFont

    p, focus = scene["peak"], SIDES.index(scene["side"])
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image, "RGBA")
    k = maixcam2_intrinsics(image.width, image.height)
    rpy, offset = data.rpy[p], data.camera_offset[p]
    for i in range(2):
        width = 5 if i == focus else 3
        lane = data.lanes[p, i]
        uv = project(lane[(X_IDXS >= DRAW_MIN_X_M) & (X_IDXS <= FIT_MAX_X_M)], rpy, offset, k)
        line = [tuple(q) for q in uv if np.isfinite(q).all()]
        if len(line) > 1:
            draw.line(line, fill=MODEL_COLOR + (230,), width=width, joint="curve")
        # 순정: 1.5 m의 위치와 방향을 직선으로 늘린다. 지표와 같이 모델 좌표로 놓고, 높이는 같은 쪽 모델
        # 차선을 따른다
        xs = np.arange(DRAW_MIN_X_M, 30.01, 0.5)
        ys = a.camera_pos[p, i] + np.tan(a.camera_head[p, i]) * (xs - X0_M)
        zs = np.interp(xs, X_IDXS, lane[:, 2])
        uv = project(np.stack([xs, ys, zs], 1), rpy, offset, k)
        for j in range(0, len(uv) - 1, 2):
            if np.isfinite(uv[j:j + 2]).all():
                draw.line([tuple(uv[j]), tuple(uv[j + 1])], fill=CAMERA_COLOR + (240,), width=width)
    font = ImageFont.load_default(size=max(14, image.height // 40))
    lines = label.split("\n")
    pad, step = 8, font.size + 6
    box_w = max(draw.textlength(line, font=font) for line in lines) + 2 * pad
    draw.rectangle([0, 0, box_w, len(lines) * step + 2 * pad], fill=(0, 0, 0, 165))
    for n, line in enumerate(lines):
        draw.text((pad, pad + n * step), line, font=font, fill=(255, 255, 255))
    # 범례: 주황 실선 모델, 파랑 점선 순정
    model_text, camera_text = "model", "stock LKA14 (1.5 m, straight)"
    swatch, gap = 30, 12
    camera_x = pad + swatch + 8 + draw.textlength(model_text, font=font) + 2 * gap
    legend_w = camera_x + swatch + 8 + draw.textlength(camera_text, font=font) + pad
    top = image.height - step - 2 * pad
    mid = top + pad + step // 2
    draw.rectangle([0, top, legend_w, image.height], fill=(0, 0, 0, 165))
    draw.line([(pad, mid), (pad + swatch, mid)], fill=MODEL_COLOR, width=4)
    draw.text((pad + swatch + 8, top + pad), model_text, font=font, fill=(255, 255, 255))
    for j in range(0, swatch, 12):
        draw.line([(camera_x + j, mid), (camera_x + j + 6, mid)], fill=CAMERA_COLOR, width=4)
    draw.text((camera_x + swatch + 8, top + pad), camera_text, font=font, fill=(255, 255, 255))
    image.save(path, quality=82)


def scene_label(route: str, scene: dict) -> str:
    what = {"gap": f"stock - model {scene['diff_m']:+.2f} m (drive median {scene['route_median_m']:+.2f})",
            "camera_only": f"only stock sees it for {scene['duration_s']:.1f} s",
            "model_only": f"only model sees it for {scene['duration_s']:.1f} s"}[scene["kind"]]
    return (f"{route}  t={scene['t_s']:.1f} s  {scene['speed_kph']:.0f} km/h\n"
            f"{scene['kind']}  {scene['side']} lane  {what}\n"
            f"stock quality {scene['camera_quality']:.0f}  model prob {scene['model_prob']:.2f}")


def analyse(route: Path, out_dir: Path, per_kind: dict[str, int], frames: bool) -> dict | None:
    data = read_route(route)
    if not len(data.lka_t) or not len(data.model_t) or not len(data.control_t):
        print(f"{route.name}: no LKA14, model or control records")
        return None
    a = align(data)
    report = {"route": route.name, "x0_m": X0_M, "camera_pos_lag_s": CAMERA_POS_LAG_S,
              "camera_head_lag_s": CAMERA_HEAD_LAG_S, **metrics(data, a)}
    video = RouteVideo(route) if frames else None
    has_video = np.isin(data.frame_id, list(video.where)) if video else None
    if has_video is not None and not has_video.any():
        print(f"  no video in {route.name}; scenes without frames")
        has_video = None
    scenes = find_scenes(data, a, per_kind, has_video)
    route_out = out_dir / route.name
    (route_out / "scenes").mkdir(parents=True, exist_ok=True)
    for n, scene in enumerate(scenes):
        rgb = video.frame_rgb(scene["frame_id"]) if video else None
        if rgb is not None:
            name = f"{n:02d}_{scene['kind']}_{scene['side']}.jpg"
            draw_scene(rgb, data, a, scene, scene_label(route.name, scene), route_out / "scenes" / name)
            scene["image"] = f"scenes/{name}"
    report["scenes"] = [{k: v for k, v in s.items() if k != "peak"} for s in scenes]
    (route_out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1))
    return report


def print_scorecard(reports: list[dict]) -> None:
    print(f"\n{'route':26} {'calib':5} {'yaw':>6} {'cmp':>5} {'c0 rms L/R':>11} {'corr L/R':>10} "
          f"{'centre':>7} {'width':>6} {'c1 str':>7}  scenes")
    print(f"{'':26} {'':5} {'deg':>6} {'min':>5} {'cm':>11} {'':>10} {'cm':>7} {'cm':>6} {'deg':>7}")
    for r in reports:
        if "left" not in r:
            print(f"{r['route']:26} too few compared frames ({r['compared_min']:.1f} min)")
            continue
        kinds = {k: sum(s["kind"] == k for s in r["scenes"]) for k in KINDS}
        straight = r.get("straight_head_diff_deg")
        print(f"{r['route']:26} {r['calibration']:5} {r['cal_yaw_deg']:+6.2f} {r['compared_min']:5.1f} "
              f"{r['left']['pos_rms_cm']:5.1f}/{r['right']['pos_rms_cm']:<5.1f} "
              f"{r['left']['pos_corr']:4.2f}/{r['right']['pos_corr']:<5.2f} {r['centre_bias_cm']:+7.1f} "
              f"{r['width_diff_cm']:+6.1f} {'' if straight is None else f'{straight:+.3f}':>7}  "
              f"gap {kinds['gap']}, stock-only {kinds['camera_only']}, model-only {kinds['model_only']}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("routes", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--scenes", type=int, default=6,
                        help="gap scenes per route; stock-only and model-only get half each (at least 1)")
    parser.add_argument("--no-frames", action="store_true", help="skip decoding video frames")
    args = parser.parse_args()
    one_sided = max(1, args.scenes // 2)
    per_kind = {"gap": args.scenes, "camera_only": one_sided, "model_only": one_sided}
    reports = []
    for route in args.routes:
        print(f"{route.name} ...")
        report = analyse(route, args.out, per_kind, not args.no_frames)
        if report:
            reports.append(report)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "scorecard.json").write_text(json.dumps(
        [{k: v for k, v in r.items() if k != "scenes"} | {"scenes": len(r["scenes"])} for r in reports],
        ensure_ascii=False, indent=1))
    print_scorecard(reports)


if __name__ == "__main__":
    main()
