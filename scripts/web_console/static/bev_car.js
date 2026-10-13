// BEV(bev.js)가 자차와 앞차로 그리는 차를 코드로 만든다: 미터 단위, x 앞, y 왼쪽, z 위, z = 0에 서고 x와
// y로는 가운데에 놓인다. 흔한 크기로 한 번, 플랫 셰이딩으로 그리는 로우폴리 메시로 만들고, 차마다 그
// 인스턴스를 제 크기로 늘인다. 바퀴는 따로 인스턴스라 어떻게 늘여도 둥글다. sv_recorder_bev의
// static/bev_models.js에서 차만 옮겨 왔다: 모델은 다른 클래스를 보지 않는다.
//
// carModel()은 {body, lamps, brake, turn, wheels, size}다:
//   body    position, color(부품마다 제 색)와 paint(인스턴스 색이 물들이는 곳은 1)
//   lamps   position과 color, 조명 없이 그린다: 헤드라이트와 어둡게 켠 테일라이트
//   brake   켜진 테일라이트, 어두운 것보다 살짝 바깥에 있다: 제동 중에 그 위에 그린다
//   turn    [왼쪽, 오른쪽] 방향지시등, 호박색, 깜빡이는 동안 조명 없이 그린다: 헤드라이트 아래 앞
//           모서리, 미러 리피터, 테일라이트 아래 뒤 모서리
//   wheels  모델 크기에서 바퀴마다 [x, y, z, 반지름, 폭], wheel()의 인스턴스용
//   size    모델을 만든 크기 [길이, 폭, 높이]

import * as THREE from "three";

const linear = (hex) => { const c = new THREE.Color(hex); return [c.r, c.g, c.b]; };
const PAINT = [1, 1, 1];
const GLASS = linear("#141a22"), DARK = linear("#1d2026"), TYRE = linear("#111214"), RIM = linear("#a3aab4");
const WHITE = linear("#f4f4f4"), HEAD = linear("#fff4dc"), TAIL = linear("#a8160c"), BRAKE = linear("#ff5a46");
const AMBER = linear("#ffa31a");

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const PLATE_HALF = 0.26;                // 번호판 폭의 절반(한국 번호판은 520 x 110 mm)
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const mean = (points) => [0, 1, 2].map((k) => points.reduce((sum, p) => sum + p[k], 0) / points.length);

// -- 모델 만들기: 정점마다 색과 paint 플래그를 단 삼각형 -----------------------------------------
class Builder {
  constructor() { this.positions = []; this.colors = []; this.paints = []; this.indices = []; }

  // three.js 지오메트리의 삼각형을 color로 넣는다. paint 1이면 인스턴스 색이 물들인다.
  add(geometry, color, paint = 0) {
    const p = geometry.attributes.position, base = this.positions.length / 3;
    for (let i = 0; i < p.count; i++) {
      this.positions.push(p.getX(i), p.getY(i), p.getZ(i));
      this.colors.push(...color);
      this.paints.push(paint);
    }
    if (geometry.index) for (const i of geometry.index.array) this.indices.push(base + i);
    else for (let i = 0; i < p.count; i++) this.indices.push(base + i);
    geometry.dispose();
  }

  // [x, y, z] 꼭짓점의 평평한 볼록 다각형, inside 반대쪽을 향하도록 감는다.
  polygon(corners, color, paint, inside) {
    corners = corners.filter((c, i) => i === 0 || Math.hypot(...sub(c, corners[i - 1])) > 1e-5);
    if (corners.length > 2 && Math.hypot(...sub(corners[0], corners[corners.length - 1])) < 1e-5) corners.pop();
    if (corners.length < 3) return;
    const n = cross(sub(corners[1], corners[0]), sub(corners[2], corners[0]));
    if (Math.hypot(...n) < 1e-9) return;
    if (dot(n, sub(mean(corners), inside)) < 0) corners.reverse();
    const base = this.positions.length / 3;
    for (const c of corners) { this.positions.push(...c); this.colors.push(...color); this.paints.push(paint); }
    for (let i = 1; i + 1 < corners.length; i++) this.indices.push(base, base + i, base + i + 1);
  }

  geometry(withPaint = true) {
    if (!this.indices.length) return null;
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(this.positions, 3));
    geometry.setAttribute("color", new THREE.Float32BufferAttribute(this.colors, 3));
    if (withPaint) geometry.setAttribute("paint", new THREE.Float32BufferAttribute(this.paints, 1));
    geometry.setIndex(this.indices);
    geometry.computeBoundingSphere();
    return geometry;
  }
}

function box(x0, x1, y0, y1, z0, z1) {
  const geometry = new THREE.BoxGeometry(Math.abs(x1 - x0), Math.abs(y1 - y0), Math.abs(z1 - z0));
  geometry.translate((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
  return geometry;
}

// -- 로프트: x를 따라 늘어선 단면으로 만든 몸체 -----------------------------------------------------
// 한 스테이션의 단면 {zb, zt: 바닥과 위, hb, ht: 그 높이의 반폭, cb, ct: 그 모따기}: 모따기한 사다리꼴을
// 점 [y, z]로, 그리고 한 점에서 다음 점까지 각 구간이 무엇인지: bottom, chamferB, side<band>(band는
// splits의 높이에서 나뉜다), chamferT 또는 top.
function section(s, splits) {
  const room = Math.max(s.zt - s.zb, 1e-3) / 2.01;
  s = {...s, cb: Math.min(s.cb, room, s.hb * 0.99), ct: Math.min(s.ct, room, s.ht * 0.99)};
  const zb1 = s.zb + s.cb, zt1 = s.zt - s.ct;
  const at = (z) => {
    const t = zt1 > zb1 ? clamp((z - zb1) / (zt1 - zb1), 0, 1) : 0;
    return [s.hb + (s.ht - s.hb) * t, zb1 + (zt1 - zb1) * t];
  };
  const rise = [[s.hb, zb1], ...splits.map(at), [s.ht, zt1]];
  const points = [[s.hb - s.cb, s.zb], ...rise, [s.ht - s.ct, s.zt], [-(s.ht - s.ct), s.zt],
                  ...rise.slice().reverse().map(([y, z]) => [-y, z]), [-(s.hb - s.cb), s.zb]];
  const bands = splits.map((_, k) => "side" + k).concat("side" + splits.length);
  const segments = ["chamferB", ...bands, "chamferT", "top", "chamferT", ...bands.slice().reverse(), "chamferB", "bottom"];
  return {points, segments, mid: (s.zb + s.zt) / 2};
}

// stations를 지나는 로프트, 각 단면은 shape(x)가 준다. pick({segment, x, y, z, normal})이 면마다
// [color, paint]를 주고, 빼려면 null을 준다. 평평한 양 끝면의 segment는 "cap"이다.
function loft(builder, stations, shape, pick, splits = []) {
  const rings = stations.map((x) => ({x, ...section(shape(x), splits)}));
  const n = rings[0].points.length;
  for (let i = 1; i < rings.length; i++) {
    const a = rings[i - 1], b = rings[i];
    for (let j = 0; j < n; j++) {
      const k = (j + 1) % n;
      const quad = [[a.x, ...a.points[j]], [a.x, ...a.points[k]], [b.x, ...b.points[k]], [b.x, ...b.points[j]]];
      const centre = mean(quad), inside = [centre[0], 0, (a.mid + b.mid) / 2];
      let normal = cross(sub(quad[2], quad[0]), sub(quad[3], quad[1]));
      const length = Math.hypot(...normal);
      if (length < 1e-9) continue;
      normal = normal.map((v) => v / length);
      if (dot(normal, sub(centre, inside)) < 0) normal = normal.map((v) => -v);
      const got = pick({segment: a.segments[j], x: centre[0], y: centre[1], z: centre[2], normal});
      if (got) builder.polygon(quad, got[0], got[1], inside);
    }
  }
  for (const [ring, out] of [[rings[0], -1], [rings[rings.length - 1], 1]]) {
    const got = pick({segment: "cap", x: ring.x, y: 0, z: ring.mid, normal: [out, 0, 0]});
    if (got) builder.polygon(ring.points.map(([y, z]) => [ring.x, y, z]), got[0], got[1], [ring.x - out, 0, ring.mid]);
  }
}

// x0부터 x1까지의 스테이션: 안쪽에 든 marks의 모든 점에, 그리고 적어도 step마다.
function stations(x0, x1, step, marks = []) {
  const xs = [x0, x1, ...marks.filter((x) => x > x0 && x < x1)];
  for (let x = x0 + step; x < x1; x += step) xs.push(x);
  xs.sort((a, b) => a - b);
  return xs.filter((x, i) => i === 0 || x - xs[i - 1] > 2e-3);
}

// 반지름 ra인 휠 아치 둘레의 스테이션: 각 아치 발 바로 바깥과 아치를 따라.
function archMarks(centres, ra, steps = 6) {
  const xs = [];
  for (const c of centres) {
    xs.push(c - ra - 0.01, c + ra + 0.01);
    for (let i = 0; i <= steps; i++) xs.push(c - ra * Math.cos(Math.PI * i / steps));
  }
  return xs;
}

// 밑면: bottom을 높이 zc를 중심으로 한 반지름 ra의 휠 아치마다 그 둘레에서 들어 올린다.
function archBottom(x, centres, ra, zc, bottom) {
  let z = bottom;
  for (const c of centres) {
    const dx = Math.abs(x - c);
    if (dx <= ra + 1e-6) z = Math.max(z, zc + Math.sqrt(Math.max(0, ra * ra - dx * dx)));
  }
  return z;
}

const underside = (paint) => ({segment}) => segment === "bottom" || segment === "chamferB" ? [DARK, 0] : paint;

// 차 한 대: 휠 아치가 있는 아래 몸체, 그 위의 글라스하우스(사방이 유리, 지붕과 필러는 도장), 등화, 그릴,
// 번호판과 미러. 1.6 m보다 높으면 SUV다.
function car(l, w, h) {
  const body = new Builder(), lamps = new Builder(), brake = new Builder(), turn = [new Builder(), new Builder()];
  const X = l / 2, half = w / 2, suv = h > 1.6;
  const r = suv ? 0.37 : 0.33, ra = r + 0.06, clearance = suv ? 0.2 : 0.15;
  const axles = [X - l * 0.2, -X + l * 0.21];
  const belt = h * (suv ? 0.58 : 0.6), nose = belt - (suv ? 0.08 : 0.15);
  const shield = X - l * (suv ? 0.29 : 0.31), roofFront = X - l * (suv ? 0.43 : 0.47);
  const roofRear = -X + l * (suv ? 0.08 : 0.24), rearFoot = -X + l * (suv ? 0.035 : 0.13);
  const plan = (x) => half * (1 - 0.09 * clamp((x - (X - 0.7)) / 0.7, 0, 1) ** 2 - 0.06 * clamp((-X + 0.5 - x) / 0.5, 0, 1) ** 2);
  const top = (x) => {
    let z = belt;
    if (x > shield) z -= (belt - nose) * ((x - shield) / (X - shield)) ** 1.6;
    if (x > X - 0.2) z -= 0.07 * ((x - (X - 0.2)) / 0.2) ** 2;
    if (x < -X + 0.2) z -= 0.06 * ((-X + 0.2 - x) / 0.2) ** 2;
    return z;
  };
  const bottom = (x) => archBottom(x, axles, ra, r, clearance + 0.12 * clamp((x - (X - 0.35)) / 0.35, 0, 1) ** 2
                                                         + 0.1 * clamp((-X + 0.35 - x) / 0.35, 0, 1) ** 2);
  const lower = stations(-X, X, 0.3, [-X + 0.06, -X + 0.2, X - 0.06, X - 0.2, X - 0.35, -X + 0.35, shield, ...archMarks(axles, ra)]);
  loft(body, lower, (x) => ({zb: bottom(x), zt: top(x), hb: plan(x) - 0.03, ht: plan(x), cb: 0.05, ct: 0.08}), underside([PAINT, 1]));

  // 글라스하우스: 윤곽은 지붕 양 끝에서 둥글리고, 옆면은 안으로 기울인다
  const base = belt - 0.02, crown = (x) => h - 0.03 * (2 * (x - (roofFront + roofRear) / 2) / (roofFront - roofRear)) ** 2;
  const outline = (x) => x >= roofFront ? base + (crown(roofFront) - base) * (shield - x) / (shield - roofFront)
                       : x <= roofRear ? base + (crown(roofRear) - base) * (x - rearFoot) / (roofRear - rearFoot) : crown(x);
  const roofLine = (x) => {
    let sum = 0;
    for (let i = -2; i <= 2; i++) sum += outline(clamp(x + i * 0.06, rearFoot, shield));
    return Math.max(base + 0.01, sum / 5);
  };
  const pillar = roofFront - (roofFront - roofRear) * 0.45;
  loft(body, stations(rearFoot, shield, 0.35, [roofRear - 0.12, roofRear, roofRear + 0.12, roofFront - 0.12, roofFront, roofFront + 0.12,
                                              pillar - 0.05, pillar + 0.05, rearFoot + 0.1, shield - 0.1]),
       (x) => {
         const zt = roofLine(x), hb = plan(x) - 0.07;
         return {zb: base, zt, hb, ht: hb - (zt - base) * 0.3, cb: 0, ct: 0.06};
       },
       ({segment, x, normal}) => {
         if (segment === "bottom" || segment === "chamferB") return null;
         if (segment === "top") return normal[2] > 0.93 ? [PAINT, 1] : [GLASS, 0];
         if (segment.startsWith("side")) {
           return Math.abs(x - pillar) < 0.05 || roofLine(x) - base < 0.1 ? [PAINT, 1] : [GLASS, 0];
         }
         return [PAINT, 1];
       });

  // 어느 쪽에서 보든 앞뒤를 한눈에 가르는 것: 앞은 요즘 차처럼 얇은 헤드라이트와 범퍼 아래쪽 번호판, 뒤는
  // 모서리를 감싸는 테일라이트, 그 사이의 번호판과 보조 제동등.
  const nx = X - 0.1, wrap = (x0, x1, z0, z1, side, color, into, out = 0.004) => {
    for (let x = x0; x < x1 - 1e-6; x += 0.1) {             // 옆면의 띠, 그 곡선을 따른다
      const a = x, b = Math.min(x1, x + 0.1), ya = side * (plan(a) + out), yb = side * (plan(b) + out);
      into.polygon([[a, ya, z0], [b, yb, z0], [b, yb, z1], [a, ya, z1]], color, 0, [(a + b) / 2, 0, (z0 + z1) / 2]);
    }
  };
  for (const side of [1, -1]) {
    lamps.add(box(X - 0.12, X + 0.004, side * 0.2 * w, side * (plan(nx) - 0.07), nose - 0.15, nose - 0.1), HEAD);
    body.add(box(shield - 0.18, shield - 0.06, side * (plan(shield) - 0.02), side * (plan(shield) + 0.11), belt + 0.01, belt + 0.1), PAINT, 1);
    // 방향지시등: 헤드라이트 아래로 앞 모서리를 돌아, 그리고 미러 밑동을 따라
    const signal = turn[side > 0 ? 0 : 1];
    signal.add(box(X - 0.1, X + 0.008, side * (plan(nx) - 0.28), side * (plan(nx) - 0.02), nose - 0.24, nose - 0.17), AMBER);
    wrap(X - 0.42, X - 0.1, nose - 0.24, nose - 0.17, side, AMBER, signal, 0.008);
    signal.add(box(shield - 0.17, shield - 0.07, side * (plan(shield) + 0.09), side * (plan(shield) + 0.118), belt + 0.006, belt + 0.035), AMBER);
  }
  body.add(box(X - 0.05, X + 0.005, -0.3 * w, 0.3 * w, clearance + 0.1, clearance + 0.28), DARK);         // 아래 흡기구
  body.add(box(X - 0.03, X + 0.016, -PLATE_HALF, PLATE_HALF, clearance + 0.16, clearance + 0.27), WHITE);

  const rear = plan(-X + 0.05) - 0.04, lampTop = belt - 0.1, lampBottom = belt - 0.23;
  for (const side of [1, -1]) {                // 모서리마다 그것을 감싸는 등
    lamps.add(box(-X - 0.008, -X + 0.08, side * 0.27 * w, side * rear, lampBottom, lampTop), TAIL);
    wrap(-X + 0.06, -X + 0.34, lampBottom + 0.02, lampTop, side, TAIL, lamps);
    brake.add(box(-X - 0.014, -X + 0.07, side * 0.26 * w, side * (rear + 0.006), lampBottom - 0.006, lampTop + 0.006), BRAKE);
    wrap(-X + 0.06, -X + 0.34, lampBottom + 0.014, lampTop + 0.006, side, BRAKE, brake, 0.01);
    // 그리고 테일라이트 아래, 뒤 모서리를 도는 띠
    const signal = turn[side > 0 ? 0 : 1];
    signal.add(box(-X - 0.012, -X + 0.06, side * 0.36 * w, side * (rear + 0.004), lampBottom - 0.075, lampBottom - 0.015), AMBER);
    wrap(-X + 0.06, -X + 0.3, lampBottom - 0.075, lampBottom - 0.015, side, AMBER, signal, 0.008);
  }
  body.add(box(-X - 0.012, -X + 0.02, -PLATE_HALF, PLATE_HALF, lampBottom - 0.01, lampBottom + 0.1), WHITE);
  body.add(box(-X - 0.004, -X + 0.05, -0.34 * w, 0.34 * w, clearance + 0.07, clearance + 0.17), DARK);    // 범퍼
  // 보조 제동등, 뒷유리 위쪽(SUV는 테일게이트)에
  const stop = roofRear - 0.04, stopZ = roofLine(stop) - 0.035;
  lamps.add(box(stop - 0.03, stop + 0.03, -0.17 * w, 0.17 * w, stopZ - 0.02, stopZ + 0.012), TAIL);
  brake.add(box(stop - 0.036, stop + 0.036, -0.175 * w, 0.175 * w, stopZ - 0.026, stopZ + 0.018), BRAKE);
  const outer = plan(axles[0]) - 0.012;
  return {body: body.geometry(), lamps: lamps.geometry(false), brake: brake.geometry(false),
          turn: turn.map((signal) => signal.geometry(false)), size: [l, w, h],
          wheels: axles.flatMap((x) => [1, -1].map((side) => [x, side * (outer - 0.11), r, r, 0.22]))};
}

// 반지름 1, 폭 1인 바퀴, 차축은 y 방향: 타이어, 양면의 림과 허브.
export function wheel() {
  const body = new Builder();
  body.add(new THREE.CylinderGeometry(1, 1, 1, 18), TYRE);
  body.add(new THREE.CylinderGeometry(0.64, 0.64, 1.02, 18), RIM);
  body.add(new THREE.CylinderGeometry(0.2, 0.2, 1.04, 8), DARK);
  return body.geometry(false);
}

// 흔한 세단 크기로 한 번 만든 차: 모든 차는 이것을 fit()으로 늘인 것이다.
let made = null;
export function carModel() {
  return made || (made = car(4.5, 1.84, 1.46));
}

// carModel()을 길이 x 폭 x 높이로 만드는 늘임 [x, y, z].
export function fit([l, w, h]) {
  const [ml, mw, mh] = carModel().size;
  return [l / ml, w / mw, h / mh];
}
