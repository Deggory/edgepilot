// What the board's model and controlsd publish, as the BEV (bev.js) draws it. The board only copies:
// /api/bev says where the fields the BEV reads sit in ModelState and ControlState
// (src/common/ipc_messages.h; check_web_console.py holds the two together), and /api/bev/stream sends both
// payloads as they were published, a frame per new model frame. A frame is a 16-byte head, magic
// "BEV1" (u32), ModelState and ControlState byte counts (u16 each) and the board's CLOCK_BOOTTIME ns
// (u64), then the payloads. A payload is 0 bytes when the board has none or, for the model, nothing
// new: with the model stopped, a frame comes every second all the same.
//
// scene() is what bev.js draws, in vehicle coordinates: metres, x forward from the front bumper, y
// to the left. The model's points are from the camera, y to the right; the camera sits
// RADAR_TO_CAMERA behind the bumper, as controlsd and the HUD take it. The model already sees from
// the car's centre line (modeld warps by camera_offset_m), so y needs no shift.
//
// The two ego lines are drawn as controlsd's planner sees them: through the same lane-line filter
// (lane_filter.js), on the capture times and controlsd's speed and measured curvature. The model's own
// lines stay alongside (scene().lanes raw).

import {LaneLineFilter} from "./lane_filter.js";

const MAGIC = 0x31564542;
const HEADER = 16;
const TRAJECTORY = 33;
// openpilot T_IDXS: when the car is at each plan point, s
const T_IDXS = Array.from({length: TRAJECTORY}, (_, i) => 10 * (i / (TRAJECTORY - 1)) ** 2);
const RADAR_TO_CAMERA = 1.52;           // model_output.h kRadarToCameraDistanceM (check_web_console.py)
const LEAD_PROBABILITY = 0.5;           // model_output.h kLeadProbabilityThreshold (check_web_console.py)
const LANE_SURE = 0.5;                  // both ego lines at least this sure for the lane width
const LANE_WIDTH_INDEX = 6;             // hud_scene.cc kLaneRulerIndex: 6.75 m from the camera
// hud_scene.cc draw_scene: lines and the path reach as far as the plan does, 10 to 100 m
const DRAW = {min: 10, max: 100};
const PLAN_NOW_INDEX = 2;               // the plan's speed now: at 0.04 s, past the first point's noise
const GAS_PRESS_HORIZON = 1;            // controls_tick.cc feeds the departure alert gas_press_probs[1], 2 s ahead
const DRIVE_GEAR = 5;                   // can_frame.h kGearDrive: the departure alert only watches in D
const HISTORY_S = 10;                   // how much of the departure inputs the card keeps
// The lead card: the lead's distance and speed relative to ours over the last LEAD_HISTORY_S, from the
// model, from optical flow (speed only) and from modeld's lead track (the two fused), against the trend
// of the model's own distance: a straight line fitted over ±half s of one lead (no gap, no jump of more
// than jump m, at least points frames reaching within edge s of both ends), as docs/flow-calibration.md
// validates them. It is known half s late.
export const LEAD_HISTORY_S = 20;
const TREND = {half: 2, jump: 4, points: 60, edge: 0.1};
const MODEL_STALE_S = 0.5, CONTROL_STALE_S = 1.0;
const RETRY_MS = [500, 5000];
// the fields this page reads; a server that does not say where one is is older or newer than the page
const NEEDS = {
  model: ["size", "capture_timestamp_ns", "model_timestamp_ns", "valid", "plan", "lanes", "lane_probabilities", "road_edges", "road_edge_stds",
          "lead", "gas_press_probs", "lead_flow", "lead_track"],
  control: ["size", "timestamp_ns", "enabled", "engaged", "active", "left_blinker", "right_blinker", "gear",
            "desired_curvature", "actual_curvature",
            "normalized_output", "departure_alert_type", "green_light_alert_armed", "hud_flags", "ego_speed_kph"],
  hud_flags: ["Laneless", "SteerPaused", "BrakeLights"],
};

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

// [[x, y(, speed)], ...] of model points xs, ys, as the HUD draws a ribbon of them: finite points, x
// growing, no farther than reach (model x).
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

// The x and y of the IpcPoint array at offset.
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

// The plan's speed at each point, m/s, from its positions over T_IDXS. Against the model's own
// velocity output (not in ModelState) it is off by 0.04 m/s at 2 s and 0.07 m/s at 4 s (median,
// 2026-10-04 drives), and the board's U16 model keeps it (correlation 0.997 with fp32 to 4 s).
function planSpeeds(view, offset) {
  const at = (i, k) => view.getFloat32(offset + 12 * i + 4 * k, true);
  return T_IDXS.map((_, i) => {
    const a = Math.max(0, i - 1), b = Math.min(TRAJECTORY - 1, i + 1);
    return Math.hypot(at(b, 0) - at(a, 0), at(b, 1) - at(a, 1)) / (T_IDXS[b] - T_IDXS[a]);
  });
}

// One ModelState payload (bytes), read where layout says.
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
    egoLines: [lines[1], lines[2]],     // the ego lines' model points, for the lane filter (BevStream)
    edges: [0, 1].map((i) => ({std: f32(at.road_edge_stds + 4 * i), points: polyline(view, at.road_edges + 12 * TRAJECTORY * i, reach)})),
    laneWidth: laneWidth(lanes, lines[1][1], lines[2][1]),
    lead: view.getUint32(lead, true)
      ? {probability: f32(lead + 4), x: f32(lead + 8), y: f32(lead + 12), speed: f32(lead + 16), accel: f32(lead + 20)} : null,
    // LeadFlowState: the lead's expansion rate (1/s, + closing in) over about 0.5 s, and the model distance
    // it came from (m, from the camera)
    flow: view.getUint32(flow + 8, true) ? {rate: f32(flow + 20), x: f32(flow + 28)} : null,
    // LeadTrackState: the model distance and the flow fused (m from the camera, m/s, - closing)
    track: view.getUint32(track, true) ? {x: f32(track + 8), relative: f32(track + 12)} : null,
  };
}

// The width between the ego lines at LANE_WIDTH_INDEX, NaN unless both are sure.
function laneWidth(lanes, left, right) {
  return lanes[1].probability >= LANE_SURE && lanes[2].probability >= LANE_SURE
    ? right[LANE_WIDTH_INDEX] - left[LANE_WIDTH_INDEX] : NaN;
}

// One ControlState payload.
function readControl(at, flags, bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const u32 = (offset) => view.getUint32(offset, true), f32 = (offset) => view.getFloat32(offset, true);
  const hud = u32(at.hud_flags);
  return {
    stamp: view.getBigUint64(at.timestamp_ns, true),
    enabled: u32(at.enabled) !== 0, engaged: u32(at.engaged) !== 0, active: u32(at.active) !== 0,
    laneless: (hud >>> flags.Laneless & 1) === 1, paused: (hud >>> flags.SteerPaused & 1) === 1,
    braking: (hud >>> flags.BrakeLights & 1) === 1,
    output: f32(at.normalized_output), speedKph: f32(at.ego_speed_kph),
    desired: f32(at.desired_curvature), actual: f32(at.actual_curvature),
    alert: u32(at.departure_alert_type), armed: u32(at.green_light_alert_armed) !== 0,
    drive: view.getInt32(at.gear, true) === DRIVE_GEAR,
    left: u32(at.left_blinker) !== 0, right: u32(at.right_blinker) !== 0,
  };
}

const ageOf = (now, stamp) => stamp ? Number(now - stamp) / 1e9 : Infinity;

// The scene of the latest model and control state (either may be null) at board time now:
//   path [[x, y, planned speed m/s]], planNow (m/s), lanes [{probability, points, raw}] ×4 (far left,
//   left, right, far right; the ego lines' points filtered as the planner filters them while controlsd
//   runs, raw the model's own, only on the ego lines), edges [{std, points}] ×2, laneWidth (m, between
//   the lines drawn, NaN unless both ego lines are sure)
//   lead {x: its rear, y, gap, speed, accel, relative (m/s, null without our speed), ttc (s, Infinity unless
//     closing in), tracked (x, gap, relative and speed come from the lead track), raw {x, relative} (the
//     model's own), flow {relative: -x · rate (m/s, from optical flow)} or null} or null
//   leads [{t s, x, v (the model's), flow, fx, fv (the lead track's), trend, level, segment}, ...]: the lead
//     card's last LEAD_HISTORY_S, distances from our bumper; set by BevStream (BevStream.addLead)
//   state {enabled, engaged, active, steering, laneless, output}, speedKph (NaN without controlsd)
//   curvature {desired, actual} (1/m, + to the right as in controlsd) or null without controlsd
//   blinkers {left, right} (both: hazard lights) or null without controlsd. controlsd holds each on
//   for 0.5 s past the lamp's last flash, so they stay on while it flashes
//   brakeLights: the ego car's brake lights (pedal pressed, or AUTO HOLD: vehicle_can brake_lights_on)
//   departure {drive (in D), armed, alert (0 none, 1 lead moved, 2 green light), gas [0, 2, … 10 s],
//   plan10 (m, model x at 10 s), history [[t s, gas 2 s, plan10]]}: what departure_alert.cc decides from
//   modelAge, controlAge (s, Infinity if none)
export function scene(model, control, now, history = []) {
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
            steering: !!live && live.active && !live.paused, laneless: !!live && live.laneless, output: live ? live.output : 0},
    speedKph: live ? live.speedKph : NaN,
    curvature: live && Number.isFinite(live.desired) && Number.isFinite(live.actual) ? {desired: live.desired, actual: live.actual} : null,
    blinkers: live ? {left: live.left, right: live.right} : null,
    brakeLights: !!live && live.braking,
    departure: {drive: !!live && live.drive, armed: !!live && live.armed, alert: live ? live.alert : 0, gas: shown ? shown.gas : [],
                plan10: shown ? shown.planEnd : NaN, history},
  };
}

// A straight line fitted to the model distance of rows[i]'s lead over ±TREND.half s: [slope m/s, level m at
// rows[i]'s time], NaN where that lead does not cover the window (scene().leads).
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

// The stream while the BEV tab is shown: onScene(scene()) for every frame, onStatus({live, text})
// when what it has to say changes. It connects again by itself, waiting longer each time it fails.
export class BevStream {
  constructor({onScene, onStatus, hz = 20}) {
    Object.assign(this, {onScene, onStatus, hz});
    this.layout = null;
    this.model = this.control = null;
    this.run = 0;
    this.abort = null;
    this.arrivals = [];                 // when the model frames of the last second came, ms
    this.history = [];                  // [t s, gas 2 s, plan10] of the last HISTORY_S of model frames
    this.leads = [];                    // the lead card's rows (scene().leads), one per model frame
    this.segment = 0;                   // which lead a row belongs to (trend fits stay within one)
    this.laneFilters = [new LaneLineFilter(), new LaneLineFilter()];   // the ego lines, as the planner's
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
        this.status(false, "연결 중");
        if (!this.layout) {
          const response = await fetch("/api/bev", {cache: "no-store", signal: abort.signal});
          if (!response.ok) throw new Error(`HTTP ${response.status}`);
          const layout = await response.json();
          const missing = Object.entries(NEEDS).flatMap(([part, keys]) => keys.filter((key) => !(key in (layout[part] || {}))));
          if (missing.length) throw new Error(`서버와 페이지 버전이 다릅니다(${missing.join(", ")}): web_console/을 통째로 다시 올리세요`);
          this.layout = layout;
        }
        const response = await fetch(`/api/bev/stream?hz=${this.hz}`, {cache: "no-store", signal: abort.signal});
        if (!response.ok || !response.body) throw new Error(`HTTP ${response.status}`);
        await this.read(response.body.getReader(), run);
        wait = RETRY_MS[0];
        if (run === this.run) this.status(false, "연결 끊김 · 다시 연결 중");
      } catch (error) {
        if (run !== this.run) return;
        this.status(false, `연결 끊김 · ${error.message || error}`);
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
        if (head.getUint32(0, true) !== MAGIC) throw new Error("잘못된 프레임");
        const model = head.getUint16(4, true), control = head.getUint16(6, true), end = at + HEADER + model + control;
        if (end > pending.length) break;
        this.frame(pending.subarray(at + HEADER, at + HEADER + model), pending.subarray(at + HEADER + model, end),
                   head.getBigUint64(8, true));
        at = end;
      }
      pending = pending.subarray(at);
    }
  }

  // One model frame's lead for the lead card (scene().leads). A row's trend and level are filled once the
  // rows TREND.half s past it are in; the rows still waiting are always the newest ones.
  addLead(t, lead) {
    const rows = this.leads, last = rows[rows.length - 1];
    if (last && t < last.t) rows.length = 0;                     // another boot
    const x = lead ? lead.raw.x : NaN;
    if (!lead || !last || !Number.isFinite(last.x) || Math.abs(x - last.x) > TREND.jump) this.segment++;
    rows.push({t, x, v: lead && lead.raw.relative !== null ? lead.raw.relative : NaN, flow: lead && lead.flow ? lead.flow.relative : NaN,
               fx: lead && lead.tracked ? lead.x : NaN, fv: lead && lead.tracked ? lead.relative : NaN,
               trend: undefined, level: undefined, segment: lead ? this.segment : -1});
    let i = rows.length - 1;
    while (i > 0 && rows[i - 1].trend === undefined) i--;
    for (; i < rows.length && rows[i].t <= t - TREND.half; i++) {
      [rows[i].trend, rows[i].level] = rows[i].segment < 0 ? [NaN, NaN] : trendAt(rows, i);
    }
    while (rows.length && rows[0].t < t - LEAD_HISTORY_S) rows.shift();
  }

  /* The ego lines through the planner's lane filter, on controlsd's speed and measured curvature (as the
   * planner gets them, ControlState ego_speed_kph and actual_curvature). Like the planner, an invalid model
   * frame leaves the filters alone. Without controlsd the model's lines stand and the filters start again. */
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

  frame(model, control, now) {
    const layout = this.layout, fresh = model.length >= layout.model.size;
    let t = 0;
    this.control = control.length >= layout.control.size ? readControl(layout.control, layout.hud_flags, control) : null;
    if (fresh) {
      this.model = readModel(layout.model, model);
      this.filterLanes(this.model, now);
      this.arrivals.push(performance.now());
      t = Number(this.model.stamp) / 1e9;
      const history = this.history;
      if (history.length && t < history[history.length - 1][0]) history.length = 0;     // another boot
      history.push([t, this.model.gas[GAS_PRESS_HORIZON], this.model.planEnd]);
      while (history.length && history[0][0] < t - HISTORY_S) history.shift();
    }
    const shown = scene(this.model, this.control, now, this.history);
    if (fresh) this.addLead(t, shown.lead);
    shown.leads = this.leads;
    this.onScene(shown);
    const since = performance.now() - 1000;
    while (this.arrivals.length && this.arrivals[0] < since) this.arrivals.shift();
    const note = this.control ? "" : " · controlsd 없음";
    if (!this.model) this.status(false, `모델 상태 없음${note}`);
    else if (shown.stale) this.status(false, `모델 멈춤 · ${Math.round(shown.modelAge)}초 전${note}`);
    else this.status(true, `모델 ${this.arrivals.length} Hz${note}`);
  }
}
