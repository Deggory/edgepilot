// 보드의 모델과 controlsd가 발행하는 것을 BEV(bev.js)가 그리는 형태로 옮긴다. 보드는 복사만 한다:
// /api/bev는 BEV가 읽는 필드가 ModelState, ControlState, LaneState(차선선 플러그인) 앞부분의 어디에 있는지 알려
// 주고(src/common/ipc_messages.h; 맞는지는 check_web_console.py가 지킨다), /api/bev/stream은 세 페이로드를 발행된
// 그대로, 새 모델 프레임마다 한 프레임씩 보낸다. 프레임은 20바이트 머리와 그 뒤의 페이로드다. 머리는 매직
// "BEV2"(u32), ModelState·ControlState·LaneState 앞부분 바이트 수(각 u16, 그리고 빈 2바이트), 보드의
// CLOCK_BOOTTIME ns(u64)다. 페이로드는 보드에 그것이 없거나, 모델이면 새것이 없을 때 0바이트다: 모델이 멈춰도
// 프레임은 그대로 매초 온다. LaneState는 그때 나와 있는 가장 새 것이다(보통 같은 프레임이나 한 프레임 전).
//
// scene()은 bev.js가 그리는 것으로, 차량 좌표계다: 미터, x는 앞 범퍼에서 앞으로, y는 왼쪽. 모델의 점은
// 카메라 기준이고 y가 오른쪽이다. 카메라는 controlsd와 HUD가 잡는 대로 범퍼에서 RADAR_TO_CAMERA 뒤에 있다.
// 모델은 이미 차의 중심선에서 보므로(modeld가 camera_offset_m만큼 워프한다) y는 옮길 필요가 없다.
//
// 두 자차 차선선은 controlsd의 플래너가 보는 대로 그린다: 같은 차선선 필터(lane_filter.js)에 캡처 시각과
// controlsd의 속도·측정 곡률로 거른다. 모델 자체의 차선선도 함께 남는다(scene().lanes raw).

import {t} from "./i18n.js";
import {LaneLineFilter} from "./lane_filter.js";

const MAGIC = 0x32564542;
const HEADER = 20;
const TRAJECTORY = 33;
// openpilot T_IDXS: 차가 각 계획 점에 있게 되는 시각, s
const T_IDXS = Array.from({length: TRAJECTORY}, (_, i) => 10 * (i / (TRAJECTORY - 1)) ** 2);
// openpilot X_IDXS: 차선선 점의 앞 거리, m (LaneState 도로면 선도 같은 격자)
const X_IDXS = Array.from({length: TRAJECTORY}, (_, i) => 192 * (i / (TRAJECTORY - 1)) ** 2);
const LANE_MATCH_S = 0.15;              // 같이 그릴 LD 결과와 모델 프레임의 촬영 시각 차 한도
const LD_SURE = 0.4;                    // hud_state.cc hud_lane_ego_y: LD 차로 폭을 재려면 두 자차 선이 이만큼 확실해야 한다
const RADAR_TO_CAMERA = 1.52;           // model_output.h kRadarToCameraDistanceM (check_web_console.py)
const LEAD_PROBABILITY = 0.5;           // model_output.h kLeadProbabilityThreshold (check_web_console.py)
const LANE_SURE = 0.5;                  // 차로 폭을 재려면 두 자차 차선선 모두 이만큼 확실해야 한다
const LANE_WIDTH_INDEX = 6;             // hud_scene.cc kLaneRulerIndex: 카메라에서 6.75 m
// hud_scene.cc draw_scene: 차선선과 경로는 계획이 닿는 데까지, 10~100 m
const DRAW = {min: 10, max: 100};
const PLAN_NOW_INDEX = 2;               // 계획의 현재 속도: 첫 점의 잡음을 지난 0.04 s 지점
const GAS_PRESS_HORIZON = 1;            // controls_tick.cc가 출발 알림에 넣는 gas_press_probs[1], 2 s 후
const DRIVE_GEAR = 5;                   // can_frame.h kGearDrive: 출발 알림은 D에서만 본다
const HISTORY_S = 10;                   // 카드가 출발 알림 입력을 얼마나 남겨 두는지
// 앞차 카드: 최근 LEAD_HISTORY_S 동안 앞차의 거리와 자차 대비 상대 속도를 모델, 광류(속도만), modeld의
// 앞차 추적(둘을 융합한 것)에서 가져와 모델 자체 거리의 추세와 견준다. 추세는 앞차 하나의 ±half s에 맞춘
// 직선이다(끊김 없이, jump m보다 큰 튐 없이, 양 끝에서 edge s 안까지 닿는 points 프레임 이상).
// docs/lead-flow.md가 이들을 검증하는 방식대로다. 추세는 half s 늦게 알게 된다.
export const LEAD_HISTORY_S = 20;
const TREND = {half: 2, jump: 4, points: 60, edge: 0.1};
const MODEL_STALE_S = 0.5, CONTROL_STALE_S = 1.0;
const RETRY_MS = [500, 5000];
// 이 페이지가 읽는 필드. 그중 하나라도 위치를 알려 주지 않는 서버는 페이지보다 오래됐거나 새것이다
const NEEDS = {
  model: ["size", "capture_timestamp_ns", "model_timestamp_ns", "valid", "plan", "lanes", "lane_probabilities", "road_edges", "road_edge_stds",
          "lead", "gas_press_probs", "lead_flow", "lead_track"],
  control: ["size", "timestamp_ns", "enabled", "engaged", "active", "left_blinker", "right_blinker", "gear",
            "desired_curvature", "actual_curvature",
            "normalized_output", "departure_alert_type", "green_light_alert_armed", "hud_flags", "ego_speed_kph"],
  lane: ["size", "capture_timestamp_ns", "valid", "road_count", "road"],
  lane_road: ["size", "points", "kind", "slot", "type", "double_line", "color", "confidence", "y"],
  hud_flags: ["Laneless", "LaneChanging", "SteerPaused", "BrakeLights", "Lanebow"],
};

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

// 모델 점 xs, ys의 [[x, y(, speed)], ...]. HUD가 그것들로 띠를 그리듯, 유한하고 x가 커지며 reach(모델 x)보다
// 멀지 않은 점만.
function linePoints(xs, ys, reach, speeds = null) {
  const points = [];
  let last = -Infinity;
  for (let i = 0; i < xs.length; i++) {
    const x = xs[i], y = ys[i];
    if (!Number.isFinite(x) || !Number.isFinite(y) || x > reach || x <= last) break;
    points.push(speeds ? [x - RADAR_TO_CAMERA, -y, speeds[i]] : [x - RADAR_TO_CAMERA, -y]);
    last = x;
  }
  return points;
}

// offset에 있는 IpcPoint 배열의 x와 y.
function pointsAt(view, offset) {
  const xs = [], ys = [];
  for (let i = 0; i < TRAJECTORY; i++) {
    xs.push(view.getFloat32(offset + 12 * i, true));
    ys.push(view.getFloat32(offset + 12 * i + 4, true));
  }
  return [xs, ys];
}

function polyline(view, offset, reach, speeds = null) {
  return linePoints(...pointsAt(view, offset), reach, speeds);
}

// 각 점에서 계획의 속도(m/s)를 T_IDXS에 걸친 위치로 구한다. 모델 자체의 속도 출력(ModelState에 없음)과
// 비교하면 2 s에서 0.04 m/s, 4 s에서 0.07 m/s 어긋나고(중앙값, 2026-10-04 주행), 보드의 U16 모델도 이를
// 지킨다(4 s까지 fp32와 상관 0.997).
function planSpeeds(view, offset) {
  const at = (i, k) => view.getFloat32(offset + 12 * i + 4 * k, true);
  return T_IDXS.map((_, i) => {
    const a = Math.max(0, i - 1), b = Math.min(TRAJECTORY - 1, i + 1);
    return Math.hypot(at(b, 0) - at(a, 0), at(b, 1) - at(a, 1)) / (T_IDXS[b] - T_IDXS[a]);
  });
}

// ModelState 페이로드(bytes) 하나를 layout이 알려 준 위치에서 읽는다.
function readModel(at, bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const f32 = (offset) => view.getFloat32(offset, true);
  const planEnd = f32(at.plan + 12 * (TRAJECTORY - 1));
  const reach = Number.isFinite(planEnd) ? clamp(planEnd, DRAW.min, DRAW.max) : DRAW.max;
  const speeds = planSpeeds(view, at.plan);
  const lines = [0, 1, 2, 3].map((i) => pointsAt(view, at.lanes + 12 * TRAJECTORY * i));
  const lanes = lines.map(([xs, ys], i) => ({probability: f32(at.lane_probabilities + 4 * i), points: linePoints(xs, ys, reach)}));
  const lead = at.lead, flow = at.lead_flow, track = at.lead_track;
  return {
    stamp: view.getBigUint64(at.model_timestamp_ns, true),
    captured: Number(view.getBigUint64(at.capture_timestamp_ns, true)) * 1e-9,
    valid: view.getUint32(at.valid, true) !== 0,
    path: polyline(view, at.plan, reach, speeds),
    planNow: speeds[PLAN_NOW_INDEX],
    planEnd,
    gas: Array.from({length: 6}, (_, i) => f32(at.gas_press_probs + 4 * i)),
    lanes, reach,
    egoLines: [lines[1], lines[2]],     // 차선선 필터(BevStream)에 넣을 자차 차선선의 모델 점
    edges: [0, 1].map((i) => ({std: f32(at.road_edge_stds + 4 * i), points: polyline(view, at.road_edges + 12 * TRAJECTORY * i, reach)})),
    laneWidth: laneWidth(lanes, lines[1][1], lines[2][1]),
    lead: view.getUint32(lead, true)
      ? {probability: f32(lead + 4), x: f32(lead + 8), y: f32(lead + 12), speed: f32(lead + 16), accel: f32(lead + 20)} : null,
    // LeadFlowState: 약 0.5 s 동안 앞차의 확대율(1/s, +면 가까워진다)과 그것을 구한 모델 거리
    // (m, 카메라 기준)
    flow: view.getUint32(flow + 8, true) ? {rate: f32(flow + 20), x: f32(flow + 28)} : null,
    // LeadTrackState: 모델 거리와 광류를 융합한 값(카메라에서 m, m/s, -면 가까워진다)
    track: view.getUint32(track, true) ? {x: f32(track + 8), relative: f32(track + 12)} : null,
  };
}

// LaneState 앞부분(차선선 플러그인 결과의 헤더와 도로면 블록) 하나: 선과 도로 경계를 차량 좌표의 점으로.
// 점은 측정이 덮은 거리까지(가까운 쪽은 늘린 값). kind 1은 도로 경계, type은 선이면 모양(0 실선, 1 점선,
// 2 보츠)이고 경계면 종류다(lane_plugin_abi.h). left·right는 자차 선(slot마다 신뢰도가 높은 것, 플래너와 같다),
// width는 그 사이의 LANE_WIDTH_INDEX 폭이다(둘 다 확실하지 않으면 NaN).
function readLane(at, road, bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const count = Math.min(view.getUint32(at.road_count, true), Math.floor((bytes.byteLength - at.road) / road.size));
  const lines = [];
  for (let k = 0; k < count; k++) {
    const base = at.road + road.size * k, i8 = (offset) => view.getInt8(base + offset);
    const points = [];
    for (let i = 0; i < road.points; i++) {
      const y = view.getFloat32(base + road.y + 4 * i, true);
      if (Number.isFinite(y)) points.push([X_IDXS[i] - RADAR_TO_CAMERA, -y]);
      else if (points.length) break;
    }
    lines.push({boundary: i8(road.kind) === 1, slot: i8(road.slot), type: i8(road.type), double: i8(road.double_line),
                color: i8(road.color), confidence: view.getFloat32(base + road.confidence, true), points,
                widthY: view.getFloat32(base + road.y + 4 * LANE_WIDTH_INDEX, true)});
  }
  const ego = (slot) => lines.reduce((best, line) =>
    !line.boundary && line.slot === slot && line.points.length >= 2 && (!best || line.confidence > best.confidence) ? line : best, null);
  const left = ego(0), right = ego(1);
  const sure = left && right && left.confidence >= LD_SURE && right.confidence >= LD_SURE;
  return {captured: Number(view.getBigUint64(at.capture_timestamp_ns, true)) * 1e-9,
          valid: view.getUint32(at.valid, true) !== 0, lines, left, right,
          width: sure ? right.widthY - left.widthY : NaN};
}

// LANE_WIDTH_INDEX에서 두 자차 차선선 사이의 폭. 둘 다 확실하지 않으면 NaN.
function laneWidth(lanes, left, right) {
  return lanes[1].probability >= LANE_SURE && lanes[2].probability >= LANE_SURE
    ? right[LANE_WIDTH_INDEX] - left[LANE_WIDTH_INDEX] : NaN;
}

// ControlState 페이로드 하나.
function readControl(at, flags, bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const u32 = (offset) => view.getUint32(offset, true), f32 = (offset) => view.getFloat32(offset, true);
  const hud = u32(at.hud_flags);
  return {
    stamp: view.getBigUint64(at.timestamp_ns, true),
    enabled: u32(at.enabled) !== 0, engaged: u32(at.engaged) !== 0, active: u32(at.active) !== 0,
    laneless: (hud >>> flags.Laneless & 1) === 1, paused: (hud >>> flags.SteerPaused & 1) === 1,
    lanebow: (hud >>> flags.Lanebow & 1) === 1, changing: (hud >>> flags.LaneChanging & 1) === 1,
    braking: (hud >>> flags.BrakeLights & 1) === 1,
    output: f32(at.normalized_output), speedKph: f32(at.ego_speed_kph),
    desired: f32(at.desired_curvature), actual: f32(at.actual_curvature),
    alert: u32(at.departure_alert_type), armed: u32(at.green_light_alert_armed) !== 0,
    drive: view.getInt32(at.gear, true) === DRIVE_GEAR,
    left: u32(at.left_blinker) !== 0, right: u32(at.right_blinker) !== 0,
  };
}

const ageOf = (now, stamp) => stamp ? Number(now - stamp) / 1e9 : Infinity;

// 보드 시각 now에서 최신 모델·제어 상태(둘 다 null일 수 있다)의 장면:
//   path [[x, y, 계획 속도 m/s]], planNow (m/s), lanes [{probability, points, raw}] ×4 (왼쪽 바깥, 왼쪽,
//   오른쪽, 오른쪽 바깥. controlsd가 도는 동안 자차 차선선의 points는 플래너가 거르는 대로 거른 것이고, raw는
//   모델 자체의 것으로 자차 차선선에만 있다), edges [{std, points}] ×2, laneWidth (m, 그려진 차선선 사이,
//   두 자차 차선선이 다 확실하지 않으면 NaN)
//   lead {x: 앞차 뒷면, y, gap, speed, accel, relative (m/s, 자차 속도가 없으면 null), ttc (s, 가까워지지
//     않으면 Infinity), tracked (x, gap, relative, speed가 앞차 추적에서 온다), raw {x, relative} (모델 자체의
//     것), flow {relative: -x · rate (m/s, 광류에서)} 또는 null} 또는 null
//   leads [{t s, x, v (모델의 것), flow, fx, fv (앞차 추적의 것), trend, level, segment}, ...]: 앞차 카드의
//     최근 LEAD_HISTORY_S, 거리는 자차 범퍼 기준. BevStream이 채운다(BevStream.addLead)
//   state {enabled, engaged, active, steering, laneless, lanebow, changing (차선 변경 중), output}, speedKph
//     (controlsd가 없으면 NaN)
//   curvature {desired, actual} (1/m, controlsd처럼 +가 오른쪽), controlsd가 없으면 null
//   blinkers {left, right} (둘 다면 비상등), controlsd가 없으면 null. controlsd는 램프가 마지막으로 깜박인
//   뒤 0.5 s 동안 각각을 켜 두므로, 램프가 깜박이는 동안 켜진 채로 있다
//   brakeLights: 자차의 브레이크등(페달을 밟았거나 AUTO HOLD: vehicle_can brake_lights_on)
//   departure {drive (D 단), armed, alert (0 없음, 1 앞차 출발, 2 녹색 신호), gas [0, 2, … 10 s],
//   plan10 (m, 10 s의 모델 x), history [[t s, gas 2 s, plan10]]}: departure_alert.cc가 판단에 쓰는 것
//   modelAge, controlAge (s, 없으면 Infinity)
//   ld: 모델 프레임과 LANE_MATCH_S 안의 마지막 유효한 차선선 플러그인 결과 {lines [{boundary, slot, type, double,
//     color, confidence, points}], left, right (자차 선), width (m)} 또는 null(플러그인이 없거나 낡았다).
//     state.lanebow는 경로 모드가 Lanebow인지
export function scene(model, control, now, history = [], lane = null) {
  const modelAge = model ? ageOf(now, model.stamp) : Infinity;
  const controlAge = control ? ageOf(now, control.stamp) : Infinity;
  const live = control && controlAge <= CONTROL_STALE_S ? control : null;
  const speed = live ? live.speedKph / 3.6 : NaN;
  const shown = model && model.valid ? model : null;
  const lead = shown && shown.lead;
  const seen = lead && lead.probability >= LEAD_PROBABILITY && Number.isFinite(lead.x) && Number.isFinite(lead.speed);
  const flow = seen && shown.flow && Number.isFinite(shown.flow.rate) && Number.isFinite(shown.flow.x) ? shown.flow : null;
  const track = seen && shown.track && Number.isFinite(shown.track.x) && Number.isFinite(shown.track.relative) ? shown.track : null;
  const raw = seen ? {x: lead.x - RADAR_TO_CAMERA, relative: Number.isFinite(speed) ? lead.speed - speed : null} : null;
  const x = track ? track.x - RADAR_TO_CAMERA : raw && raw.x, relative = track ? track.relative : raw && raw.relative;
  return {
    modelAge, controlAge, stale: modelAge > MODEL_STALE_S,
    path: shown ? shown.path : [], planNow: shown ? shown.planNow : NaN,
    lanes: shown ? shown.lanes : [], edges: shown ? shown.edges : [],
    laneWidth: shown ? shown.laneWidth : NaN,
    lead: seen ? {x, y: -lead.y, gap: Math.max(0, x), accel: lead.accel, relative,
                  speed: track ? (Number.isFinite(speed) ? speed + track.relative : lead.speed) : lead.speed,
                  ttc: relative !== null && relative < -0.1 ? Math.max(0, x) / -relative : Infinity,
                  tracked: !!track, raw, flow: flow ? {relative: -flow.x * flow.rate} : null} : null,
    leads: [],
    state: {enabled: !!live && live.enabled, engaged: !!live && live.engaged, active: !!live && live.active,
            steering: !!live && live.active && !live.paused, laneless: !!live && live.laneless,
            lanebow: !!live && live.lanebow, changing: !!live && live.changing, output: live ? live.output : 0},
    ld: shown && lane && Math.abs(lane.captured - shown.captured) <= LANE_MATCH_S ? lane : null,
    speedKph: live ? live.speedKph : NaN,
    curvature: live && Number.isFinite(live.desired) && Number.isFinite(live.actual) ? {desired: live.desired, actual: live.actual} : null,
    blinkers: live ? {left: live.left, right: live.right} : null,
    brakeLights: !!live && live.braking,
    departure: {drive: !!live && live.drive, armed: !!live && live.armed, alert: live ? live.alert : 0, gas: shown ? shown.gas : [],
                plan10: shown ? shown.planEnd : NaN, history},
  };
}

// rows[i] 앞차의 모델 거리에 ±TREND.half s 동안 맞춘 직선: [기울기 m/s, rows[i] 시각의 level m]. 그 앞차가
// 창을 다 덮지 않으면 NaN (scene().leads).
function trendAt(rows, i) {
  const {t, segment} = rows[i];
  let a = i, b = i;
  while (a > 0 && rows[a - 1].segment === segment && rows[a - 1].t >= t - TREND.half) a--;
  while (b + 1 < rows.length && rows[b + 1].segment === segment && rows[b + 1].t <= t + TREND.half) b++;
  const n = b - a + 1, reach = TREND.half - TREND.edge;
  if (n < TREND.points || rows[a].t > t - reach || rows[b].t < t + reach) return [NaN, NaN];
  let st = 0, sx = 0, stt = 0, stx = 0;
  for (let j = a; j <= b; j++) {
    const dt = rows[j].t - t, x = rows[j].x;
    st += dt; sx += x; stt += dt * dt; stx += dt * x;
  }
  const d = n * stt - st * st;
  if (!(d > 0)) return [NaN, NaN];
  const slope = (n * stx - st * sx) / d;
  return [slope, (sx - slope * st) / n];
}

function concat(a, b) {
  const joined = new Uint8Array(a.length + b.length);
  joined.set(a);
  joined.set(b, a.length);
  return joined;
}

// BEV 탭이 보이는 동안의 스트림: 프레임마다 onScene(scene()), 알릴 내용이 바뀌면 onStatus({live, text}).
// 스스로 다시 연결하며, 실패할 때마다 더 오래 기다린다.
export class BevStream {
  constructor({onScene, onStatus, hz = 20}) {
    Object.assign(this, {onScene, onStatus, hz});
    this.layout = null;
    this.model = this.control = this.lane = null;
    this.run = 0;
    this.abort = null;
    this.arrivals = [];                 // 최근 1초 동안 모델 프레임이 온 시각, ms
    this.history = [];                  // 최근 HISTORY_S 동안 모델 프레임의 [t s, gas 2 s, plan10]
    this.leads = [];                    // 앞차 카드의 행(scene().leads), 모델 프레임마다 하나
    this.segment = 0;                   // 행이 어느 앞차의 것인지(추세 맞춤은 한 앞차 안에서만)
    this.laneFilters = [new LaneLineFilter(), new LaneLineFilter()];   // 자차 차선선, 플래너의 것처럼
    this.said = "";
  }

  start() {
    if (this.abort) return;
    this.loop(++this.run);
  }

  stop() {
    this.run++;
    if (this.abort) this.abort.abort();
    this.abort = null;
  }

  status(live, text) {
    if (text === this.said) return;
    this.said = text;
    this.onStatus({live, text});
  }

  async loop(run) {
    let wait = RETRY_MS[0];
    while (run === this.run) {
      const abort = this.abort = new AbortController();
      try {
        this.status(false, t("bev.connecting"));
        if (!this.layout) {
          const response = await fetch("/api/bev", {cache: "no-store", signal: abort.signal});
          if (!response.ok) throw new Error(`HTTP ${response.status}`);
          const layout = await response.json();
          const missing = Object.entries(NEEDS).flatMap(([part, keys]) => keys.filter((key) => !(key in (layout[part] || {}))));
          if (missing.length) throw new Error(t("bev.version_mismatch", {fields: missing.join(", ")}));
          this.layout = layout;
        }
        const response = await fetch(`/api/bev/stream?hz=${this.hz}`, {cache: "no-store", signal: abort.signal});
        if (!response.ok || !response.body) throw new Error(`HTTP ${response.status}`);
        await this.read(response.body.getReader(), run);
        wait = RETRY_MS[0];
        if (run === this.run) this.status(false, t("bev.reconnecting"));
      } catch (error) {
        if (run !== this.run) return;
        this.status(false, t("bev.disconnected", {error: error.message || error}));
      }
      if (run !== this.run) return;
      await new Promise((resolve) => setTimeout(resolve, wait));
      wait = Math.min(2 * wait, RETRY_MS[1]);
    }
  }

  async read(reader, run) {
    let pending = new Uint8Array(0);
    for (;;) {
      const {value, done} = await reader.read();
      if (done || run !== this.run) return;
      pending = pending.length ? concat(pending, value) : value;
      let at = 0;
      while (pending.length - at >= HEADER) {
        const head = new DataView(pending.buffer, pending.byteOffset + at, HEADER);
        if (head.getUint32(0, true) !== MAGIC) throw new Error(t("bev.bad_frame"));
        const model = head.getUint16(4, true), control = head.getUint16(6, true), lane = head.getUint16(8, true);
        const body = at + HEADER, end = body + model + control + lane;
        if (end > pending.length) break;
        this.frame(pending.subarray(body, body + model), pending.subarray(body + model, body + model + control),
                   head.getBigUint64(12, true), pending.subarray(body + model + control, end));
        at = end;
      }
      pending = pending.subarray(at);
    }
  }

  // 모델 프레임 하나의 앞차를 앞차 카드(scene().leads)에 넣는다. 행의 trend와 level은 그보다 TREND.half s
  // 뒤의 행들이 들어오면 채워진다. 아직 기다리는 행은 언제나 가장 새로운 것들이다.
  addLead(stamp, lead) {
    const rows = this.leads, last = rows[rows.length - 1];
    if (last && stamp < last.t) rows.length = 0;                 // 재부팅
    const x = lead ? lead.raw.x : NaN;
    if (!lead || !last || !Number.isFinite(last.x) || Math.abs(x - last.x) > TREND.jump) this.segment++;
    rows.push({t: stamp, x, v: lead && lead.raw.relative !== null ? lead.raw.relative : NaN, flow: lead && lead.flow ? lead.flow.relative : NaN,
               fx: lead && lead.tracked ? lead.x : NaN, fv: lead && lead.tracked ? lead.relative : NaN,
               trend: undefined, level: undefined, segment: lead ? this.segment : -1});
    let i = rows.length - 1;
    while (i > 0 && rows[i - 1].trend === undefined) i--;
    for (; i < rows.length && rows[i].t <= stamp - TREND.half; i++) {
      [rows[i].trend, rows[i].level] = rows[i].segment < 0 ? [NaN, NaN] : trendAt(rows, i);
    }
    while (rows.length && rows[0].t < stamp - LEAD_HISTORY_S) rows.shift();
  }

  // 자차 차선선을 controlsd의 속도와 측정 곡률(플래너가 받는 그대로, ControlState ego_speed_kph와
  // actual_curvature)로 플래너의 차선선 필터에 거른다. 플래너처럼, 유효하지 않은 모델 프레임은 필터를 건드리지
  // 않는다. controlsd가 없으면 모델의 차선선이 그대로 남고 필터는 처음부터 다시 시작한다.
  filterLanes(model, now) {
    if (!model.valid) return;
    const control = this.control && ageOf(now, this.control.stamp) <= CONTROL_STALE_S ? this.control : null;
    const speed = control ? control.speedKph / 3.6 : NaN, curvature = control ? control.actual : NaN;
    if (!Number.isFinite(speed) || !Number.isFinite(curvature)) {
      for (const filter of this.laneFilters) filter.reset();
      return;
    }
    const filtered = model.egoLines.map(([xs, ys], side) =>
      this.laneFilters[side].update(model.captured, xs, ys, speed, curvature));
    model.egoLines.forEach(([xs], side) => {
      const lane = model.lanes[1 + side];
      model.lanes[1 + side] = {...lane, raw: lane.points, points: linePoints(xs, filtered[side], model.reach)};
    });
    model.laneWidth = laneWidth(model.lanes, filtered[0], filtered[1]);
  }

  frame(model, control, now, lane) {
    const layout = this.layout, fresh = model.length >= layout.model.size;
    // LD 결과는 모델 프레임과 함께 온다. 해석하지 못한 결과(valid 0)와 모델 없는 프레임은 직전 것을 그대로 둔다
    // (scene()이 모델 프레임과 LANE_MATCH_S 넘게 떨어진 것은 버린다)
    if (fresh && lane.length >= layout.lane.size) {
      const read = readLane(layout.lane, layout.lane_road, lane);
      if (read.valid) this.lane = read;
    }
    let stamp = 0;  // 모델 프레임 시각(s). t는 번역 함수라 쓰지 않는다
    this.control = control.length >= layout.control.size ? readControl(layout.control, layout.hud_flags, control) : null;
    if (fresh) {
      this.model = readModel(layout.model, model);
      this.filterLanes(this.model, now);
      this.arrivals.push(performance.now());
      stamp = Number(this.model.stamp) / 1e9;
      const history = this.history;
      if (history.length && stamp < history[history.length - 1][0]) history.length = 0;  // 재부팅
      history.push([stamp, this.model.gas[GAS_PRESS_HORIZON], this.model.planEnd]);
      while (history.length && history[0][0] < stamp - HISTORY_S) history.shift();
    }
    const shown = scene(this.model, this.control, now, this.history, this.lane);
    if (fresh) this.addLead(stamp, shown.lead);
    shown.leads = this.leads;
    this.onScene(shown);
    const since = performance.now() - 1000;
    while (this.arrivals.length && this.arrivals[0] < since) this.arrivals.shift();
    // 서버는 controlsd가 멈춰도 마지막 ControlState를 계속 보낸다: 오래된 것은 멈춤으로 알린다
    const note = !this.control ? ` · ${t("bev.no_controlsd")}`
      : shown.controlAge > CONTROL_STALE_S ? ` · ${t("bev.controlsd_stalled")}` : "";
    if (!this.model) this.status(false, t("bev.no_model") + note);
    else if (shown.stale) this.status(false, t("bev.model_stalled", {seconds: Math.round(shown.modelAge)}) + note);
    else this.status(true, t("bev.model_rate", {hz: this.arrivals.length}) + note);
  }
}
