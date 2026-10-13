// BEV(bev.js)의 자차: BEV가 이 차에 대해 아는 것은 EGO_SIZE와 makeEgo(renderer)뿐이라, 다른 차는 같은 두
// export 뒤에 다른 모델을 두면 된다. 이것은 지금 edgepilot이 모는 차, Aurora Black Pearl 색의 2017 Kia K7
// (YG, "All New K7")이다. 공개 제원(4970 x 1870 x 1470 mm, 휠베이스 2855, 윤거 1602 / 1610, 245/45R18)과
// Kia가 옆·앞·뒤에서 찍은 스튜디오 사진(Kia Heritage, All-New K7)에서 잰 치수로 코드에서 만들었고, 세부는
// 차주들의 사진을 따랐다. 미터 단위, x는 앞, y는 왼쪽, z는 위. x와 y로는 가운데에 두고 z = 0 위에 선다.
//
// 그냥 세단이 아니라 K7로 보이게 하는 것:
//   - 얇은 글래스하우스 아래의 높은 차체: 후드는 앞코의 0.75 m에서 윈드실드의 1.04 m까지 오르고, 창턱은
//     1.0 m 높이로 이어지다 뒷문 뒤에서 위로 휘어 오른다
//   - 세로 크롬 날로 된 넓은 그릴, 그 윗변이 헤드램프와 만난다
//   - 위로, 뒤로 펜더 속까지 쓸어 올린 헤드램프, 각각 프로젝터 둘과 Z자 주간주행등
//   - 크롬 하키스틱 위의 큐브 네 개짜리 안개등
//   - 크롬 창 둘레와 문 아래쪽의 크롬 띠
//   - B필러 뒤에서 가장 높아지는 패스트백 지붕선과 작은 쿼터 글라스
//   - 꼬리 전체를 가로지르는 크롬 선 아래의 테일램프
//   - 붉은 반사판 사이 범퍼 아래쪽의 번호판, 사다리꼴 배기구 팁, 18인치 20스포크 휠
//
// 다른 차들(bev_car.js)과 달리 한 번만 만들어 인스턴스가 아니라 그 자체로 그린다. 차체는 매끈한 법선을 가진
// 곡면이고, 머리 위와 양옆의 조명 띠로 밝힌 어두운 스튜디오를 비추는 PBR 재질을 써서, BEV의 어두운 바닥
// 위에서도 검은 도장이 사진에서처럼 긴 하이라이트를 지닌 검은 도장으로 보인다. 테일램프는 밤처럼 켜져 있고,
// setLamps()가 방향지시등을 깜박이고 브레이크등을 켠다.

import * as THREE from "three";

export const EGO_SIZE = {length: 4.97, width: 1.87, height: 1.47};
const X = EGO_SIZE.length / 2, W = EGO_SIZE.width / 2;
const AXLES = [X - 0.975, X - 0.975 - 2.855];        // 앞 오버행 975 mm (4970 - 2855의 나머지는 뒤)
const TRACKS = [1.602 / 2, 1.610 / 2];
const TYRE = {radius: 0.339, width: 0.245, rim: 0.2286};        // 245/45R18
const ARCH = 0.385;
// 중심선 위의 x 위치: 윈드실드 아래끝, 지붕의 앞·꼭대기·뒤, 데크(뒷유리 아래끝). 옆 창: A필러 아래끝,
// 미러 아래 검은 세일의 끝, B필러, 뒷문 유리의 끝, 쿼터 글라스의 꼬리.
const COWL = 1.08, ROOF_FRONT = 0.15, ROOF_TOP = -0.45, ROOF_REAR = -1.25, DECK = -2.1;
const DLO = {front: 1.0, sail: 0.64, pillar: [-0.157, -0.075], divider: -1.0, tip: -1.365};

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const lerp = (a, b, t) => a + (b - a) * t;
const ease = (a, b, x) => { const t = clamp((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t); };
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const unit = (a) => { const l = Math.hypot(...a) || 1; return a.map((v) => v / l); };
const vec = (a) => new THREE.Vector3(...a);

// x가 커지는 [x, value] 매듭점을 지나는 매끈한 곡선: 매듭 사이에서는 단조(Fritsch-Carlson), 양 끝 너머는
// 평평하다.
function curve(knots) {
  const n = knots.length, xs = knots.map(([x]) => x), vs = knots.map(([, v]) => v);
  const d = xs.slice(1).map((x, i) => (vs[i + 1] - vs[i]) / (x - xs[i]));
  const m = vs.map((_, i) => (i === 0 ? d[0] : i === n - 1 ? d[n - 2] : d[i - 1] * d[i] <= 0 ? 0 : (d[i - 1] + d[i]) / 2));
  for (let i = 0; i < n - 1; i++) {
    if (d[i] === 0) { m[i] = m[i + 1] = 0; continue; }
    const a = m[i] / d[i], b = m[i + 1] / d[i], s = a * a + b * b;
    if (s > 9) { m[i] = 3 * a * d[i] / Math.sqrt(s); m[i + 1] = 3 * b * d[i] / Math.sqrt(s); }
  }
  return (x) => {
    if (x <= xs[0]) return vs[0];
    if (x >= xs[n - 1]) return vs[n - 1];
    let i = 0;
    while (x > xs[i + 1]) i++;
    const h = xs[i + 1] - xs[i], t = (x - xs[i]) / h, t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * vs[i] + (t3 - 2 * t2 + t) * h * m[i] + (3 * t2 - 2 * t3) * vs[i + 1] + (t3 - t2) * h * m[i + 1];
  };
}

// [y, z] 점들의 꺾은선이 |y| = a를 처음 지나는 곳의 z, 없으면 null.
function crossing(path, a) {
  for (let k = 1; k < path.length; k++) {
    const [y0, z0] = path[k - 1], [y1, z1] = path[k];
    if ((a - y0) * (a - y1) <= 0 && y0 !== y1) return lerp(z0, z1, (a - y0) / (y1 - y0));
  }
  return null;
}

// -- 차체 형상 ------------------------------------------------------------------------------------
// 숄더에서의 반폭. 앞코와 꼬리에서 둥글게 줄어든다.
const halfWidth = (x) => W - 0.17 * clamp((x - (X - 0.9)) / 0.9, 0, 1) ** 2 - 0.09 * clamp((-X + 0.62 - x) / 0.62, 0, 1) ** 2;
// 하부 차체 윗면의 중심선과 가장자리 높이: 카울에서 높고 앞코에서는 모서리보다 낮게 처지는 후드, 창 아래의
// 창턱, 립까지 이어지는 데크.
const hood = curve([[COWL, 1.035], [1.3, 1.025], [1.64, 0.99], [1.99, 0.935], [2.24, 0.87], [2.34, 0.835], [2.43, 0.79], [X, 0.752]]);
const fender = curve([[COWL, 0.99], [1.5, 0.955], [1.9, 0.918], [2.15, 0.895], [2.33, 0.882], [X, 0.875]]);
const belt = curve([[DECK, 1.075], [-1.3, 1.055], [-0.74, 1.03], [-0.17, 1.008], [0.675, 0.991], [COWL, 0.99]]);
const deck = curve([[-X, 1.075], [-2.38, 1.105], [DECK, 1.12]]);
const deckEdge = curve([[-X, 1.035], [-2.38, 1.06], [DECK, 1.075]]);
const top = (x) => (x >= COWL ? [hood(x), fender(x)] : x > DECK ? [belt(x), belt(x)] : [deck(x), deckEdge(x)]);
// 숄더: 헤드램프에서 도어 핸들 위를 지나 테일램프로 들어가는, 옆면을 따라가는 꺾임선.
const shoulder = curve([[-X, 0.94], [-1.875, 0.93], [-1.0, 0.895], [0.07, 0.885], [1.5, 0.86], [2.1, 0.84], [X, 0.80]]);

// 밑면: 사이드실보다 높은 범퍼 립과 휠 아치.
function bottom(x) {
  let z = 0.165 + 0.095 * clamp((x - (X - 0.45)) / 0.45, 0, 1) ** 2 + 0.14 * clamp((-X + 0.6 - x) / 0.6, 0, 1) ** 2;
  for (const c of AXLES) {
    const dx = Math.abs(x - c);
    if (dx <= ARCH) z = Math.max(z, TYRE.radius + Math.sqrt(ARCH * ARCH - dx * dx));
  }
  return z;
}

// x에서 하부 차체의 단면, 위 중앙에서 돌아 아래 중앙까지의 절반: [y, z]. 후드의 두 능선(1), 숄더(5),
// 아래 모서리(9)는 꺾임선이다. 숄더 아래에서 문은 안으로 들어갔다가 크롬 띠가 지나는 곳에서 다시 부푼다.
const ridge = (x) => 0.012 * ease(COWL, COWL + 0.15, x) * (1 - ease(X - 0.3, X - 0.08, x));
function lowerHalf(x) {
  const w = halfWidth(x), zb = bottom(x), [centre, edge] = top(x);
  const zs = clamp(shoulder(x), zb + 0.05, edge - 0.04);
  return [
    [0, centre],
    [w * 0.6, lerp(centre, edge, 0.3) + ridge(x)],
    [w - 0.085, edge],
    [w - 0.03, edge - Math.min(0.03, (edge - zs) * 0.4)],
    [w - 0.004, lerp(zs, edge, 0.3)],
    [w, zs],
    [w - 0.012, lerp(zs, zb, 0.45)],
    [w - 0.005, lerp(zs, zb, 0.78)],
    [w - 0.035, zb + Math.min(0.055, (zs - zb) * 0.15)],
    [w - 0.08, zb],
    [0, zb],
  ];
}

// 양 끝은 평평한 벽이 아니다. 평면에서는 모서리 쪽으로 휘어 들어가고(앞쪽 아래 모서리가 안개등을 둘러
// 가장 깊다), 입면에서는 뒤로 기운다: 후드 아래의 앞코와 턱, 립까지 올라가는 트렁크 리드 면, 범퍼 아랫부분.
// SETBACK(y, z)는 그 깊이로, 끝면에서는 온전히 들어가고 FADE만큼 안쪽에서 사라진다. 끝 근처의 모든 것은
// 차체든 그 위에 놓인 것이든 똑같이 bend()를 거친다.
const sweep = (y, far, y0, y1) => far * clamp((Math.abs(y) - y0) / (y1 - y0), 0, 1) ** 1.5;    // 모서리로 갈수록 가팔라진다
const SETBACK = {
  front: (y, z) => sweep(y, lerp(0.30, 0.20, ease(0.42, 0.62, z)), 0.40, 0.84) + 0.06 * ease(0.62, 0.86, z) ** 1.5 + 0.045 * ease(0.40, 0.20, z) ** 1.5,
  rear: (y, z) => sweep(y, 0.16, 0.35, 0.85) + 0.14 * clamp((z - 0.6) / 0.5, 0, 1) ** 2 + 0.08 * ease(0.45, 0.27, z) ** 1.5,
};
const FADE = {front: 0.6, rear: 0.55};
function bend([x, y, z]) {
  return [x - SETBACK.front(y, z) * ease(X - FADE.front, X, x) + SETBACK.rear(y, z) * ease(-X + FADE.rear, -X, x), y, z];
}

// 높이 z(윗면 가장자리와 사이드실 사이)에서 옆면이 얼마나 바깥에 있는지. 옆면에 놓이는 것들이 쓴다.
function sideY(x, z) {
  const half = lowerHalf(x);
  for (let k = 2; k < 9; k++) {
    const [y0, z0] = half[k], [y1, z1] = half[k + 1];
    if (z <= z0 && z >= z1) return z0 === z1 ? y0 : lerp(y0, y1, (z0 - z) / (z0 - z1));
  }
  return z > half[2][1] ? half[2][0] : half[9][0];
}

// 중심선 위 글래스하우스의 윗선: 윈드실드, 지붕, 긴 뒷유리.
const roofArc = (x) => 1.47 - 0.111 * (x - ROOF_TOP) ** 2;
function roofRaw(x) {
  if (x >= ROOF_FRONT) {
    const s = (COWL - x) / (COWL - ROOF_FRONT);
    return hood(COWL) + (roofArc(ROOF_FRONT) - hood(COWL)) * s * (1.12 - 0.12 * s);
  }
  if (x >= ROOF_REAR) return roofArc(x);
  const s = (ROOF_REAR - x) / (ROOF_REAR - DECK);
  return roofArc(ROOF_REAR) - (roofArc(ROOF_REAR) - deck(DECK)) * s ** 1.15;
}

function roofLine(x) {          // 유리와 지붕이 만나는 곳은 필렛, 양 끝에서는 정확히
  const r = Math.min(0.05, (x - DECK) / 2, (COWL - x) / 2);
  let sum = 0;
  for (let k = -2; k <= 2; k++) sum += roofRaw(x + k * r);
  return sum / 5;
}

// 옆 창의 개구부: A필러와 루프 레일을 따라 쿼터 글라스의 꼬리까지 내려가는 윗선, 그리고 뒷문 뒤에서 위로
// 휘어 올라 거기서 윗선과 만나는 창턱.
const dloTop = curve([[DLO.tip, 1.209], [-1.282, 1.247], [-1.0, 1.333], [-0.604, 1.392], [-0.233, 1.389], [-0.032, 1.37],
                      [0.311, 1.299], [0.668, 1.143], [DLO.front, 0.995]]);
const sweepUp = curve([[DLO.tip, 1.209], [-1.33, 1.16], [-1.3, 1.135], [-1.2, 1.105], [-1.074, 1.082], [-0.9, 1.06], [-0.74, 1.045]]);
const sill = (x) => Math.max(belt(x) + 0.015, x < -0.74 ? sweepUp(x) : 0);

// 글래스하우스의 윗면이 옆면으로 꺾여 내려가는 곳: 창 크롬의 위, 윈드실드의 모서리, 그리고 C필러에서는
// 뒷유리 가장자리를 따라 데크까지.
function sideTop(x) {
  if (x > DLO.front) return lerp(dloTop(DLO.front) + 0.03, belt(COWL) + 0.004, ease(DLO.front, COWL, x));
  if (x >= DLO.tip) return dloTop(x) + 0.03;
  return lerp(belt(DECK) + 0.02, dloTop(DLO.tip) + 0.03, ((x - DECK) / (DLO.tip - DECK)) ** 0.85);
}

// x에서 글래스하우스의 단면, 지붕 중앙에서 창턱 위의 아래끝까지의 절반: [y, z]. 옆면은 안으로 기울고
// (텀블홈) 크롬 사이에 창을 담는다: 창의 윗선(5)과 창턱(7).
function glassHalf(x) {
  const zb = belt(x), zt = Math.max(roofLine(x), zb + 0.001), zTop = clamp(sideTop(x), zb + 0.0005, zt - 0.0005);
  const wb = halfWidth(x) - 0.085 - 0.05 * ease(0.6, COWL, x), at = (z) => wb - (z - zb) * 0.42;
  const open = x > DLO.tip && x < DLO.front;
  const upper = open ? clamp(dloTop(x), zb, zTop - 0.014) : lerp(zb, zTop, 0.6), rail = Math.min(upper + 0.012, zTop);
  const lower = open ? clamp(sill(x), zb + 0.0002, upper) : lerp(zb, zTop, 0.4), chrome = Math.max(zb + 0.0001, lower - 0.013);
  const ye = at(zTop) - 0.01, drop = zt - zTop;
  const edge = x < ROOF_REAR ? lerp(0.9, 0.78, (ROOF_REAR - x) / (ROOF_REAR - DECK)) : 0.9;   // 뒷유리는 아래끝으로 갈수록 좁아진다
  return [[0, zt], [ye * 0.55, zt - drop * 0.18], [ye * edge, zt - drop * 0.62], [ye, zTop], [at(rail), rail], [at(upper), upper],
          [at((upper + lower) / 2), (upper + lower) / 2], [at(lower), lower], [at(chrome), chrome], [wb, zb]];
}

// 글래스하우스의 각 띠가 위치에 따라 무엇인지: 0과 1은 지붕 또는 유리, 2는 그 가장자리, 3은 창 위쪽 옆면,
// 4는 창 위를 두른 크롬, 5와 6은 창(또는 필러와 미러 아래 세일), 7은 창턱의 크롬, 8은 그 아래 옆면.
function glassPart(band, x) {
  const roof = x < ROOF_FRONT + 0.02 && x > ROOF_REAR - 0.02, open = x > DLO.tip && x < DLO.front;
  if (band <= 1) return roof ? "paint" : "glass";
  if (band === 2) return roof || x < ROOF_REAR ? "paint" : "gloss";
  if (band === 4 || band === 7) return open ? "chrome" : "paint";
  if (band === 5 || band === 6) {
    if (!open) return "paint";
    const pillar = (x > DLO.pillar[0] && x < DLO.pillar[1]) || Math.abs(x - DLO.divider) < 0.015;
    return x > DLO.sail || pillar ? "gloss" : "glass";
  }
  return "paint";
}

// x0에서 x1까지의 스테이션: 안쪽의 marks마다, 그리고 적어도 step마다 하나씩.
function stations(x0, x1, step, marks = []) {
  const xs = [x0, x1, ...marks.filter((x) => x > x0 && x < x1)];
  for (let x = x0 + step; x < x1; x += step) xs.push(x);
  xs.sort((a, b) => a - b);
  return xs.filter((x, i) => i === 0 || x - xs[i - 1] > 2e-3);
}

const archMarks = (steps) => AXLES.flatMap((c) => Array.from({length: steps + 1}, (_, i) => c - ARCH * Math.cos(Math.PI * i / steps))
                                                    .concat([c - ARCH - 0.01, c + ARCH + 0.01]));

// -- 재질별 삼각형 모으기 ------------------------------------------------------------------------------
class Parts {
  constructor() { this.lists = new Map(); }

  list(name) {
    if (!this.lists.has(name)) this.lists.set(name, {p: [], n: []});
    return this.lists.get(name);
  }

  tri(name, a, b, c, na, nb, nc) {
    const list = this.list(name);
    list.p.push(...a, ...b, ...c);
    list.n.push(...na, ...nb, ...nc);
  }

  // [x, y, z] 꼭짓점으로 된 평평한 다각형(볼록하거나, 첫 꼭짓점에서 부채꼴로), toward 쪽을 향한다.
  face(name, points, toward) {
    let n = unit(cross(sub(points[1], points[0]), sub(points[2], points[0])));
    if (dot(n, toward) < 0) { points = points.slice().reverse(); n = n.map((v) => -v); }
    for (let i = 1; i + 1 < points.length; i++) this.tri(name, points[0], points[i], points[i + 1], n, n, n);
  }

  // 같은 것에 꼭짓점마다 법선을 주어, 곡면의 한 조각용.
  smooth(name, points, normals, toward) {
    const n = unit(cross(sub(points[1], points[0]), sub(points[2], points[0])));
    if (dot(n, toward) < 0) { points = points.slice().reverse(); normals = normals.slice().reverse(); }
    for (let i = 1; i + 1 < points.length; i++) this.tri(name, points[0], points[i], points[i + 1], normals[0], normals[i], normals[i + 1]);
  }

  // 법선이 있는 three.js geometry를 matrix로 옮겨 넣는다(거울 반전 없이).
  geometry(name, geometry, matrix = null) {
    const g = geometry.index ? geometry.toNonIndexed() : geometry.clone();
    if (matrix) g.applyMatrix4(matrix);
    const list = this.list(name), p = g.attributes.position.array, n = g.attributes.normal.array;
    for (let i = 0; i < p.length; i++) { list.p.push(p[i]); list.n.push(n[i]); }
    geometry.dispose();
    g.dispose();
  }

  // 점의 행들을 지나는 곡면, rows[i][j] = [x, y, z]: 행 i, i + 1과 열 j, j + 1 사이의 사각형들(closed면
  // 행을 고리로 닫는다), 각각 pick(i, j)에 담고(null이면 뺀다) inside(i)에서 바깥쪽을 향하게 한다. 법선은
  // 곡면 전체에서 매끈하게 하되, creases에 있는 열을 가로질러서는 하지 않는다.
  surface(rows, {closed = false, creases = [], pick, inside}) {
    const n = rows[0].length, m = closed ? n : n - 1, crease = new Set(creases);
    const normals = rows.map(() => Array.from({length: n}, () => [[0, 0, 0], [0, 0, 0]]));   // [열의 왼쪽, 열의 오른쪽]
    const faces = [];
    for (let i = 0; i + 1 < rows.length; i++) {
      const centre = inside(i);
      for (let j = 0; j < m; j++) {
        const k = (j + 1) % n, a = rows[i][j], b = rows[i + 1][j], c = rows[i + 1][k], d = rows[i][k];
        let f = cross(sub(c, a), sub(d, b));
        if (Math.hypot(...f) < 1e-10) continue;
        const middle = [0, 1, 2].map((q) => (a[q] + b[q] + c[q] + d[q]) / 4);
        if (dot(f, sub(middle, centre)) < 0) f = f.map((v) => -v);
        faces.push({i, j, k, f});
        for (const [r, col, side] of [[i, j, 1], [i + 1, j, 1], [i + 1, k, 0], [i, k, 0]]) {
          for (const s of crease.has(col) ? [side] : [0, 1]) {
            const sum = normals[r][col][s];
            sum[0] += f[0]; sum[1] += f[1]; sum[2] += f[2];
          }
        }
      }
    }
    for (const {i, j, k, f} of faces) {
      const name = pick(i, j);
      if (!name) continue;
      const corners = [[rows[i][j], unit(normals[i][j][1])], [rows[i + 1][j], unit(normals[i + 1][j][1])],
                       [rows[i + 1][k], unit(normals[i + 1][k][0])], [rows[i][k], unit(normals[i][k][0])]];
      for (const [p, q, r] of [[0, 1, 2], [0, 2, 3]]) {
        const [[a, na], [b, nb], [c, nc]] = [corners[p], corners[q], corners[r]];
        if (dot(cross(sub(b, a), sub(c, a)), f) >= 0) this.tri(name, a, b, c, na, nb, nc);
        else this.tri(name, a, c, b, na, nc, nb);
      }
    }
  }

  meshes(materials) {
    const group = new THREE.Group();
    for (const [name, list] of this.lists) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.Float32BufferAttribute(list.p, 3));
      geometry.setAttribute("normal", new THREE.Float32BufferAttribute(list.n, 3));
      geometry.computeBoundingSphere();
      const mesh = new THREE.Mesh(geometry, materials[name]);
      mesh.name = name;
      group.add(mesh);
    }
    return group;
  }
}

// -- 차체 -------------------------------------------------------------------------------------------
function body(parts) {
  // 하부 차체: 스테이션마다 고리 하나, 왼쪽 절반 다음 오른쪽
  const ring = (x, half) => [...half.map(([y, z]) => [x, y, z]), ...half.slice(1, -1).reverse().map(([y, z]) => [x, -y, z])];
  const xs = stations(-X, X, 0.05, [...archMarks(10), X - 0.15, X - 0.08, X - 0.04, X - 0.015, -X + 0.15, -X + 0.08, -X + 0.04, -X + 0.015,
                                    -2.38, COWL, DECK]);
  const rows = xs.map((x) => ring(x, lowerHalf(x)).map(bend));
  const middle = (row) => [row[0][0], 0, (row[0][2] + row[10][2]) / 2];
  parts.surface(rows, {closed: true, creases: [1, 19, 5, 15, 9, 11], inside: (i) => middle(rows[i]),
                       pick: (i, j) => (j === 9 || j === 10 ? "under" : "paint")});
  // 끝면: 그 곡면을 따르는 셀로
  for (const out of [1, -1]) {
    const half = lowerHalf(out * X);
    onFace(parts, "paint", out, [...half, ...half.slice(1, -1).reverse().map(([y, z]) => [-y, z])], 0, false, false);
  }

  // 글래스하우스: 왼쪽 아래끝에서 지붕을 넘어 오른쪽 아래끝까지
  const glassXs = stations(DECK, COWL, 0.04, [ROOF_FRONT - 0.02, ROOF_FRONT + 0.02, ROOF_REAR - 0.02, ROOF_REAR + 0.02, ...DLO.pillar,
                                              DLO.divider - 0.015, DLO.divider + 0.015, DLO.tip, DLO.tip + 0.01, -1.33, -1.3, DLO.sail, DLO.front]);
  const glassRows = glassXs.map((x) => {
    const half = glassHalf(x);
    return [...half.slice().reverse().map(([y, z]) => [x, y, z]), ...half.slice(1).map(([y, z]) => [x, -y, z])].map(bend);
  });
  parts.surface(glassRows, {inside: (i) => [glassXs[i], 0, belt(glassXs[i]) - 0.3],
                            pick: (i, j) => glassPart(j < 9 ? 8 - j : j - 9, (glassXs[i] + glassXs[i + 1]) / 2)});
}

// -- 차체 위에 놓이는 것 --------------------------------------------------------------------------------
// 앞(out 1) 또는 뒤(out -1) 끝면 위에 [y, z] 꼭짓점으로 만든 다각형. 면에서 띄우고, mirrored면 양쪽에
// 놓는다. 모든 y가 다각형을 한 번만 잘라야 한다. 25 x 30 mm 셀로 붙여 면의 곡면을 따르고, 하지 말라고
// 하지 않으면 면의 윤곽으로 잘린다.
function onFace(parts, name, out, corners, lift = 0.004, mirrored = true, clip = true) {
  const ys = corners.map(([y]) => y), y0 = Math.min(...ys), y1 = Math.max(...ys), slices = Math.max(1, Math.ceil((y1 - y0) / 0.025));
  const outline = clip ? lowerHalf(out * X) : null;
  const span = (y) => {                                          // y에서 다각형의 [가장 낮은, 가장 높은] z, 면 안쪽만
    const zs = [];
    corners.forEach(([ya, za], k) => {
      const [yb, zb] = corners[(k + 1) % corners.length];
      if ((y - ya) * (y - yb) > 0) return;
      if (ya === yb) zs.push(za, zb);
      else zs.push(lerp(za, zb, (y - ya) / (yb - ya)));
    });
    if (!zs.length) return null;
    let lo = Math.min(...zs), hi = Math.max(...zs);
    if (outline) {
      const a = Math.abs(y), upper = crossing(outline.slice(0, 6), a), lower = crossing(outline.slice(5), a);
      if (upper === null || lower === null) return null;
      lo = Math.max(lo, lower + 0.004);
      hi = Math.min(hi, upper - 0.004);
    }
    return hi > lo - 1e-9 ? [lo, Math.max(lo, hi)] : null;
  };
  for (let k = 0; k < slices; k++) {
    const ya = lerp(y0, y1, k / slices), yb = lerp(y0, y1, (k + 1) / slices), a = span(ya), b = span(yb);
    if (!a || !b || Math.max(a[1] - a[0], b[1] - b[0]) < 1e-6) continue;  // 조각은 한쪽 끝에서 점으로 좁아질 수 있다
    const [la, ha] = a, [lb, hb] = b;
    const rows = Math.max(1, Math.ceil(Math.max(ha - la, hb - lb) / 0.03));
    for (let r = 0; r < rows; r++) {
      const t0 = r / rows, t1 = (r + 1) / rows;
      for (const s of mirrored ? [1, -1] : [1]) {
        const quad = [[ya, lerp(la, ha, t0)], [yb, lerp(lb, hb, t0)], [yb, lerp(lb, hb, t1)], [ya, lerp(la, ha, t1)]]
          .filter(([y, z], i, all) => { const [py, pz] = all[(i + 3) % 4]; return Math.abs(y - py) + Math.abs(z - pz) > 1e-9; });
        if (quad.length < 3) continue;
        parts.smooth(name, quad.map(([y, z]) => bend([out * (X + lift), s * y, z])), quad.map(([y, z]) => faceNormal(out, s * y, z)), [out, 0, 0]);
      }
    }
  }
}

// 앞(out 1) 또는 뒤(out -1) 끝면이 [y, z]에서 향하는 방향.
function faceNormal(out, y, z) {
  const at = (dy, dz) => bend([out * X, y + dy, z + dz]);
  const n = unit(cross(sub(at(0.004, 0), at(-0.004, 0)), sub(at(0, 0.004), at(0, -0.004))));
  return n[0] * out < 0 ? n.map((c) => -c) : n;
}

// 앞(out 1) 또는 뒤(out -1) 끝면 위 한 점의 위치(면에서 띄운)와 거기서 면이 놓인 방향: {p, u, v, n}, v가
// 면을 따라 위로, n이 면 바깥으로 향하는 오른손 좌표계.
function facePlace(out, y, z, lift = 0) {
  const at = (dy, dz) => bend([out * X, y + dy, z + dz]);
  const n = faceNormal(out, y, z), rise = unit(sub(at(0, 0.005), at(0, -0.005)));
  const v = unit(sub(rise, n.map((c) => c * dot(rise, n))));
  return {p: at(0, 0).map((c, q) => c + n[q] * lift), u: cross(v, n), v, n};
}

const frame = ({p, u, v, n}) => new THREE.Matrix4().makeBasis(vec(u), vec(v), vec(n)).setPosition(...p);

// 옆면 위 x0에서 x1까지, 높이 low(x)와 high(x) 사이의 띠. 면에서 띄운다.
function onSide(parts, name, x0, x1, low, high, lift = 0.004, steps = 10, sides = [1, -1]) {
  for (const s of sides) {
    for (let k = 0; k < steps; k++) {
      const xa = lerp(x0, x1, k / steps), xb = lerp(x0, x1, (k + 1) / steps), xm = (xa + xb) / 2;
      const corners = [[xa, low(xa)], [xb, low(xb)], [xb, high(xb)], [xa, high(xa)]];
      const out = 0.6 * (ease(X - 0.4, X, xm) - ease(-X + 0.4, -X, xm));       // 양 끝에서 옆면이 끝면으로 돌아 들어간다
      const at = (x, z) => bend([x, s * (sideY(x, z) + lift), z]);
      const normal = (x, z) => {
        const n = unit(cross(sub(at(x + 0.004, z), at(x - 0.004, z)), sub(at(x, z + 0.004), at(x, z - 0.004))));
        return dot(n, [out, s, 0]) < 0 ? n.map((c) => -c) : n;
      };
      parts.smooth(name, corners.map(([x, z]) => at(x, z)), corners.map(([x, z]) => normal(x, z)), [out, s, 0]);
    }
  }
}

// 끝면 위 [y, z] 점 경로를 따르는 얇은 띠. 꺾이는 곳은 마이터로 잇고, 20 mm 길이 조각으로 놓는다.
function lineOnFace(parts, name, out, path, width, lift = 0.006, mirrored = true) {
  const normals = path.slice(1).map(([y1, z1], k) => {                  // 각 선분의 법선(면 안에서)
    const [y0, z0] = path[k], length = Math.hypot(y1 - y0, z1 - z0) || 1;
    return [-(z1 - z0) / length, (y1 - y0) / length];
  });
  const offsets = path.map((_, k) => {                                   // 폭의 절반만큼 바깥으로, 마이터 처리
    const a = normals[Math.max(0, k - 1)], b = normals[Math.min(normals.length - 1, k)];
    const m = [a[0] + b[0], a[1] + b[1]], scale = width / 2 / Math.max(0.3, (m[0] * b[0] + m[1] * b[1]));
    return [m[0] * scale, m[1] * scale];
  });
  for (const s of mirrored ? [1, -1] : [1]) {
    for (let k = 1; k < path.length; k++) {
      const [y0, z0] = path[k - 1], [y1, z1] = path[k], [o0, p0] = offsets[k - 1], [o1, p1] = offsets[k];
      const pieces = Math.max(1, Math.ceil(Math.hypot(y1 - y0, z1 - z0) / 0.02));
      const edge = (t, side) => [lerp(y0 + side * o0, y1 + side * o1, t), lerp(z0 + side * p0, z1 + side * p1, t)];
      for (let i = 0; i < pieces; i++) {
        const quad = [edge(i / pieces, 1), edge((i + 1) / pieces, 1), edge((i + 1) / pieces, -1), edge(i / pieces, -1)];
        parts.smooth(name, quad.map(([y, z]) => bend([out * (X + lift), s * y, z])), quad.map(([y, z]) => faceNormal(out, s * y, z)), [out, 0, 0]);
      }
    }
  }
}

// centre에 놓인 타원 고리(와 그 안쪽). u와 v(단위 벡터)가 펼치는 평면 위에서 toward 쪽을 향한다.
function emblem(parts, centre, u, v, a, b, toward) {
  const at = (r, t) => centre.map((c, q) => c + Math.cos(t) * (a - r) * u[q] + Math.sin(t) * (b - r) * v[q]);
  for (let k = 0; k < 24; k++) {
    const t0 = 2 * Math.PI * k / 24, t1 = 2 * Math.PI * (k + 1) / 24;
    parts.face("chrome", [at(0, t0), at(0, t1), at(0.008, t1), at(0.008, t0)], toward);
    parts.face("grille", [centre, at(0.008, t0), at(0.008, t1)], toward);
  }
}

function front(parts) {
  // 그릴: 헤드램프와 만나는 위쪽이 넓고, 아랫변은 가운데로 휘어 내려간다. 세로 날들을 두른 크롬 테, 날은
  // 각각 두 면으로 된 V자
  const grille = [[0, 0.735], [0.2, 0.731], [0.4, 0.718], [0.515, 0.702], [0.497, 0.655], [0.472, 0.61], [0.452, 0.586], [0.40, 0.571],
                  [0.30, 0.554], [0.15, 0.537], [0, 0.528]];
  onFace(parts, "grille", 1, [...grille, ...grille.slice(1, -1).reverse().map(([y, z]) => [-y, z])], 0.002, false);
  lineOnFace(parts, "chrome", 1, grille, 0.022, 0.016);
  const upper = grille.slice(0, 4), lower = grille.slice(3).reverse();
  for (let y = -0.4875; y < 0.49; y += 0.0325) {
    const z1 = crossing(upper, Math.abs(y)) - 0.02, z0 = crossing(lower, Math.abs(y)) + 0.02;
    if (z1 - z0 < 0.03) continue;
    const place = facePlace(1, y, (z0 + z1) / 2, 0.009), length = Math.hypot(...sub(bend([X, y, z1]), bend([X, y, z0])));
    for (const side of [1, -1]) {
      const m = frame(place).multiply(new THREE.Matrix4().makeTranslation(side * 0.005, 0, 0)).multiply(new THREE.Matrix4().makeRotationY(side * 0.45));
      parts.geometry("chrome", new THREE.BoxGeometry(0.013, length, 0.004), m);
    }
  }
  // 후드 앞코의 KIA 엠블럼
  const n = unit([0.53, 0, 0.85]);
  emblem(parts, bend([X - 0.05, 0, hood(X - 0.05) + 0.004]), [0, 1, 0], unit(cross(n, [0, 1, 0])).map((c) => -c), 0.064, 0.032, n);

  // 헤드램프: 그릴의 위 모서리에서 후드 가장자리 아래로 위로·바깥으로 올라 펜더로 돌아 들어가며, 크롬 선
  // 위에 놓인다. 각각 프로젝터 둘, 그리고 아랫변을 따라가다 바깥 끝을 오르고 안쪽 끝에서는 블록이 되는 Z자
  // 주간주행등
  onFace(parts, "lens", 1, [[0.496, 0.758], [0.52, 0.688], [0.65, 0.697], [0.78, 0.713], [0.80, 0.80], [0.78, 0.87], [0.62, 0.81]]);
  onSide(parts, "lens", X - 0.55, X, (x) => lerp(0.84, 0.72, ease(X - 0.55, X - 0.46, x)) - 0.02 * ease(X - 0.46, X, x),
         (x) => lerp(0.84, 0.868, ease(X - 0.5, X - 0.2, x)), 0.004, 14);
  for (const s of [1, -1]) {
    for (const [y, z, r] of [[0.74, 0.786, 0.027], [0.658, 0.778, 0.026]]) {
      const m = frame(facePlace(1, s * y, z, 0.007));
      parts.geometry("headlight", new THREE.CircleGeometry(r * 0.7, 20), m);
      parts.geometry("chrome", new THREE.RingGeometry(r * 0.7, r, 20), m.clone().multiply(new THREE.Matrix4().makeTranslation(0, 0, -0.001)));
    }
  }
  lineOnFace(parts, "drl", 1, [[0.53, 0.699], [0.758, 0.716], [0.766, 0.79]], 0.008, 0.008);
  onFace(parts, "drl", 1, [[0.506, 0.724], [0.59, 0.727], [0.59, 0.752], [0.506, 0.756]], 0.007);
  for (const [s, name] of [[1, "turnLeft"], [-1, "turnRight"]]) {              // 방향지시 때 그 블록이 호박색 점멸
    onFace(parts, name, 1, [[s * 0.506, 0.724], [s * 0.59, 0.727], [s * 0.59, 0.752], [s * 0.506, 0.756]], 0.009, false);
  }
  onSide(parts, "drl", X - 0.45, X, (x) => lerp(0.728, 0.712, ease(X - 0.45, X, x)), (x) => lerp(0.737, 0.721, ease(X - 0.45, X, x)), 0.008, 12);
  lineOnFace(parts, "chrome", 1, [[0.515, 0.684], [0.768, 0.712]], 0.007);
  onSide(parts, "chrome", X - 0.47, X, (x) => lerp(0.712, 0.69, ease(X - 0.47, X, x)), (x) => lerp(0.719, 0.697, ease(X - 0.47, X, x)), 0.005, 12);

  // 범퍼: 크롬 베젤 속 안개등의 큐브 네 개, 그 아래의 하키스틱, 번호판, 슬랫이 달린 아래 흡기구
  onFace(parts, "trim", 1, [[0.57, 0.305], [0.79, 0.305], [0.79, 0.455], [0.57, 0.455]], 0.002);
  onFace(parts, "chrome", 1, [[0.605, 0.321], [0.757, 0.321], [0.757, 0.433], [0.605, 0.433]], 0.004);
  onFace(parts, "under", 1, [[0.611, 0.327], [0.751, 0.327], [0.751, 0.427], [0.611, 0.427]], 0.005);
  for (const [ya, yb] of [[0.615, 0.673], [0.689, 0.747]]) {
    for (const [za, zb] of [[0.331, 0.373], [0.381, 0.423]]) onFace(parts, "fog", 1, [[ya, za], [yb, za], [yb, zb], [ya, zb]], 0.007);
  }
  lineOnFace(parts, "chrome", 1, [[0.452, 0.382], [0.51, 0.312], [0.70, 0.29]], 0.012);
  onSide(parts, "chrome", X - 0.36, X, () => 0.274, () => 0.288, 0.005);
  onFace(parts, "trim", 1, [[0.275, 0.372], [0.275, 0.503], [-0.275, 0.503], [-0.275, 0.372]], 0.003, false);
  onFace(parts, "plate", 1, [[0.26, 0.382], [0.26, 0.493], [-0.26, 0.493], [-0.26, 0.382]], 0.006, false);
  onFace(parts, "grille", 1, [[0.452, 0.368], [0.40, 0.262], [-0.40, 0.262], [-0.452, 0.368]], 0.002, false);
  for (const z of [0.296, 0.332]) onFace(parts, "trim", 1, [[0.43, z], [0.43, z + 0.009], [-0.43, z + 0.009], [-0.43, z]], 0.004, false);
}

function rear(parts) {
  // 테일램프: 꼬리 전체를 가로지르는 크롬 선 아래, 펜더 위의 둥근 바깥 끝에서 트렁크 리드의 사선까지,
  // 그리고 모서리를 돌아 펜더를 따라 뾰족한 끝까지. 윗부분을 따라 방향지시등(호박색)과 후진등(투명), 그
  // 아래에 라이트 가이드
  onFace(parts, "tailLens", -1, [[0.322, 0.985], [0.464, 0.847], [0.80, 0.86], [0.86, 0.90], [0.86, 0.975], [0.80, 0.995], [0.34, 0.995]]);
  const tailLow = (x) => lerp(0.864, 0.983, ease(-2.05, -1.875, x)), tailTop = (x) => lerp(0.995, 0.983, ease(-X, -1.875, x));
  onSide(parts, "tailLens", -X, -1.875, tailLow, tailTop);
  onFace(parts, "amberLens", -1, [[0.63, 0.936], [0.738, 0.936], [0.738, 0.978], [0.63, 0.978]], 0.005);
  for (const [s, name] of [[1, "turnLeft"], [-1, "turnRight"]]) {
    onFace(parts, name, -1, [[s * 0.63, 0.936], [s * 0.738, 0.936], [s * 0.738, 0.978], [s * 0.63, 0.978]], 0.007, false);
  }
  onFace(parts, "reverse", -1, [[0.414, 0.932], [0.557, 0.932], [0.557, 0.963], [0.414, 0.963]], 0.005);
  for (const z of [0.89, 0.909, 0.925]) lineOnFace(parts, "tail", -1, [[0.47, z - 0.002], [0.765, z + 0.002]], 0.006);
  onSide(parts, "tail", -X, -2.0, () => 0.905, () => 0.913, 0.006);
  onSide(parts, "tail", -X, -1.94, () => 0.95, () => 0.958, 0.006);
  lineOnFace(parts, "chrome", -1, [[0.79, 1.0], [0, 1.0]], 0.007);
  onFace(parts, "chrome", -1, [[0.322, 0.976], [0.322, 0.998], [-0.322, 0.998], [-0.322, 0.976]], 0.005, false);
  lineOnFace(parts, "chrome", -1, [[0.778, 0.863], [0.464, 0.843], [0.322, 0.976]], 0.005);
  // 트렁크 리드의 아래 모서리, 엠블럼을 두르는 U자: 리드 위의 KIA, 그 왼쪽 아래의 K7
  lineOnFace(parts, "under", -1, [[0.62, 0.845], [0.60, 0.74], [0.577, 0.687], [0.508, 0.637], [0, 0.637]], 0.004, 0.003);
  const badge = facePlace(-1, 0, 0.895, 0.005);
  emblem(parts, badge.p, badge.u, badge.v, 0.07, 0.035, badge.n);
  onFace(parts, "chrome", -1, [[0.44, 0.73], [0.497, 0.73], [0.497, 0.76], [0.44, 0.76]], 0.005, false);
  // 범퍼: 크롬 띠 위에 낮게 단 번호판, 모서리를 돌아 옆면의 크롬 스피어로 이어지는 붉은 반사판, 그 아래의
  // 배기구 팁, 디퓨저 위의 은색 선
  onFace(parts, "trim", -1, [[0.275, 0.442], [0.275, 0.566], [-0.275, 0.566], [-0.275, 0.442]], 0.003, false);
  onFace(parts, "plate", -1, [[0.26, 0.452], [0.26, 0.556], [-0.26, 0.556], [-0.26, 0.452]], 0.006, false);
  onFace(parts, "chrome", -1, [[0.37, 0.408], [0.37, 0.416], [-0.37, 0.416], [-0.37, 0.408]], 0.004, false);
  onFace(parts, "reflector", -1, [[0.543, 0.417], [0.86, 0.417], [0.86, 0.457], [0.543, 0.457]], 0.005);
  onSide(parts, "reflector", -X, -2.2, () => 0.418, () => 0.456, 0.005, 4);
  onSide(parts, "chrome", -2.2, -1.88, (x) => lerp(0.436, 0.40, (x + 2.2) / 0.32), (x) => lerp(0.45, 0.414, (x + 2.2) / 0.32), 0.005, 6);
  onFace(parts, "chrome", -1, [[0.519, 0.387], [0.727, 0.387], [0.71, 0.325], [0.535, 0.325]], 0.006);
  onFace(parts, "under", -1, [[0.533, 0.378], [0.713, 0.378], [0.699, 0.333], [0.548, 0.333]], 0.009);
  onFace(parts, "chrome", -1, [[0.39, 0.296], [0.39, 0.302], [-0.39, 0.302], [-0.39, 0.296]], 0.004, false);
  onFace(parts, "trim", -1, [[0.49, 0.27], [0.49, 0.296], [-0.49, 0.296], [-0.49, 0.27]], 0.002, false);
}

function sides(parts) {
  // 세일 뒤 문 위의 미러: 앞이 둥근 하우징, 그 뒤의 거울, 암, 그리고 하우징 바깥 가장자리를 따라가는
  // 리피터
  for (const s of [1, -1]) {
    const y = s * 0.965;
    parts.geometry("paint", new THREE.SphereGeometry(1, 24, 14, Math.PI / 2, Math.PI), new THREE.Matrix4().makeTranslation(0.66, y, 1.058)
                                                                                     .scale(new THREE.Vector3(0.12, 0.085, 0.07)));
    parts.geometry("glass", new THREE.CircleGeometry(1, 24), new THREE.Matrix4().makeTranslation(0.659, y, 1.058).multiply(new THREE.Matrix4().makeRotationY(-Math.PI / 2))
                                                             .scale(new THREE.Vector3(0.062, 0.075, 1)));
    parts.geometry("gloss", new THREE.BoxGeometry(0.09, 0.07, 0.035), new THREE.Matrix4().makeTranslation(0.70, s * 0.885, 1.005));
    parts.geometry("amberLens", new THREE.BoxGeometry(0.075, 0.012, 0.012), new THREE.Matrix4().makeTranslation(0.72, s * 1.042, 1.03));
    parts.geometry(s > 0 ? "turnLeft" : "turnRight", new THREE.BoxGeometry(0.077, 0.014, 0.014), new THREE.Matrix4().makeTranslation(0.72, s * 1.042, 1.03));
    // 숄더 바로 아래의 도어 핸들
    for (const [x, z] of [[0.066, 0.846], [-0.996, 0.863]]) {
      parts.geometry("chrome", new THREE.BoxGeometry(0.17, 0.014, 0.028), new THREE.Matrix4().makeTranslation(x, s * (sideY(x, z) + 0.003), z));
    }
  }
  // 문 아래쪽을 따라 뒤로 갈수록 올라가는 크롬 띠
  onSide(parts, "chrome", -0.93, 0.87, (x) => lerp(0.386, 0.334, (x + 0.93) / 1.8), (x) => lerp(0.4, 0.348, (x + 0.93) / 1.8), 0.006, 12);
  // 왼쪽 뒷바퀴 뒤의 주유구 덮개
  const fuel = [[-1.74, -1.53], [0.78, 0.93]], line = 0.004;
  onSide(parts, "under", fuel[0][0], fuel[0][1], () => fuel[1][0], () => fuel[1][0] + line, 0.002, 4, [1]);
  onSide(parts, "under", fuel[0][0], fuel[0][1], () => fuel[1][1] - line, () => fuel[1][1], 0.002, 4, [1]);
  for (const x of fuel[0]) onSide(parts, "under", x - line / 2, x + line / 2, () => fuel[1][0], () => fuel[1][1], 0.002, 1, [1]);
  // 지붕 뒤쪽의 샤크핀, 그리고 뒷유리 위쪽을 따라가는 보조 브레이크등
  parts.geometry("gloss", new THREE.SphereGeometry(1, 20, 10), new THREE.Matrix4().makeTranslation(-1.145, 0, roofLine(-1.145) + 0.012)
                                                               .scale(new THREE.Vector3(0.085, 0.026, 0.06)));
  const hx = ROOF_REAR - 0.04, slope = (roofLine(hx + 0.02) - roofLine(hx - 0.02)) / 0.04;
  const up = unit([-slope, 0, 1]), hz = roofLine(hx) + 0.004;
  parts.face("stop", [[hx + 0.012, 0.28, hz + slope * 0.012], [hx - 0.012, 0.28, hz - slope * 0.012],
                      [hx - 0.012, -0.28, hz - slope * 0.012], [hx + 0.012, -0.28, hz + slope * 0.012]], up);
}

// x, y(바깥면이 바깥쪽, + 왼쪽)에서 지면에 선 바퀴: 타이어, 림의 립과 어두운 웰 위에 쌍으로 놓인 스무 개의
// 스포크, 허브, 그리고 뒤쪽의 브레이크 디스크와 캘리퍼.
function wheel(parts, x, y) {
  const out = Math.sign(y);
  const place = (geometry, local, name) => {
    const m = new THREE.Matrix4().makeTranslation(x, y, TYRE.radius);
    if (out < 0) m.multiply(new THREE.Matrix4().makeRotationZ(Math.PI));
    parts.geometry(name, geometry, m.multiply(local));
  };
  const r = TYRE.radius, rim = TYRE.rim, half = TYRE.width / 2;
  const profile = [[rim, -half + 0.008], [0.29, -half - 0.004], [0.325, -half + 0.006], [r, -half + 0.03], [r, half - 0.03],
                   [0.325, half - 0.006], [0.29, half + 0.004], [rim, half - 0.008]].map(([a, b]) => new THREE.Vector2(a, b));
  place(new THREE.LatheGeometry(profile, 40), new THREE.Matrix4(), "rubber");
  const face = new THREE.Matrix4().makeRotationX(-Math.PI / 2);                 // 원과 고리는 +y를 향한다
  place(new THREE.RingGeometry(rim - 0.018, rim + 0.002, 40), new THREE.Matrix4().makeTranslation(0, half - 0.012, 0).multiply(face), "rim");
  place(new THREE.CircleGeometry(rim - 0.016, 40), new THREE.Matrix4().makeTranslation(0, half - 0.05, 0).multiply(face), "rimDark");
  for (let k = 0; k < 20; k++) {
    const angle = 2 * Math.PI * (Math.floor(k / 2) + (k % 2 ? 0.17 : -0.17)) / 10, radial = (rim - 0.018 + 0.058) / 2;
    const m = new THREE.Matrix4().makeRotationY(-angle).multiply(new THREE.Matrix4().makeTranslation(radial, half - 0.03, 0))
      .multiply(new THREE.Matrix4().makeRotationZ(0.1));
    place(new THREE.BoxGeometry(rim - 0.076, 0.016, 0.011), m, "rim");
  }
  place(new THREE.CylinderGeometry(0.058, 0.062, 0.03, 24), new THREE.Matrix4().makeTranslation(0, half - 0.042, 0), "rim");
  place(new THREE.CircleGeometry(0.032, 24), new THREE.Matrix4().makeTranslation(0, half - 0.026, 0).multiply(face), "gloss");
  place(new THREE.CylinderGeometry(0.165, 0.165, 0.028, 32), new THREE.Matrix4().makeTranslation(0, 0.02, 0), "disc");
  place(new THREE.BoxGeometry(0.11, 0.05, 0.08), new THREE.Matrix4().makeTranslation(-0.09, 0.04, 0.12).multiply(new THREE.Matrix4().makeRotationY(0.6)), "trim");
}

// -- 재질 -------------------------------------------------------------------------------------------
function materials([dark, light]) {
  const env = {envMap: dark, envMapIntensity: 1}, shine = {envMap: light, envMapIntensity: 1};
  const unlit = (color) => new THREE.MeshBasicMaterial({color, toneMapped: false}), turn = unlit("#ffa31a");
  return {
    // Aurora Black Pearl: 스튜디오를 비추는 클리어코트 아래의 검정
    paint: new THREE.MeshPhysicalMaterial({color: "#0a0a0c", metalness: 0, roughness: 0.5, clearcoat: 1, clearcoatRoughness: 0.035, ...env}),
    gloss: new THREE.MeshPhysicalMaterial({color: "#040405", metalness: 0, roughness: 0.25, clearcoat: 1, clearcoatRoughness: 0.02, ...env}),
    trim: new THREE.MeshStandardMaterial({color: "#16181b", roughness: 0.8}),
    under: new THREE.MeshStandardMaterial({color: "#0b0c0d", roughness: 0.95}),
    grille: new THREE.MeshStandardMaterial({color: "#060708", roughness: 0.6}),
    glass: new THREE.MeshPhysicalMaterial({color: "#05080c", metalness: 0, roughness: 0.02, clearcoat: 1, ...env}),
    chrome: new THREE.MeshStandardMaterial({color: "#f2f4f7", metalness: 1, roughness: 0.1, ...shine, envMapIntensity: 1.2}),
    lens: new THREE.MeshStandardMaterial({color: "#9aa3ad", metalness: 0.85, roughness: 0.32, ...shine}),     // 유리 뒤의 반사경
    reverse: new THREE.MeshStandardMaterial({color: "#c3c8ce", metalness: 0.9, roughness: 0.22, ...shine}),
    amberLens: new THREE.MeshStandardMaterial({color: "#6b3a0c", metalness: 0.3, roughness: 0.3, ...shine}),
    rubber: new THREE.MeshStandardMaterial({color: "#161718", roughness: 0.92}),
    rim: new THREE.MeshStandardMaterial({color: "#d0d4da", metalness: 0.9, roughness: 0.25, ...shine}),
    rimDark: new THREE.MeshStandardMaterial({color: "#22252a", metalness: 0.6, roughness: 0.45, ...shine}),
    disc: new THREE.MeshStandardMaterial({color: "#55595f", metalness: 0.7, roughness: 0.5}),
    plate: new THREE.MeshStandardMaterial({color: "#f1f1ee", roughness: 0.5}),
    drl: unlit("#f2f6ff"), headlight: unlit("#fff4dc"), fog: unlit("#c8cfd9"), amber: unlit("#ff9a1f"),
    tailLens: unlit(BRAKE_LAMPS.tailLens[0]), tail: unlit(BRAKE_LAMPS.tail[0]), stop: unlit(BRAKE_LAMPS.stop[0]),
    reflector: unlit("#7c0f0e"), turnLeft: turn, turnRight: turn,
  };
}

// 제동이 바꾸는 램프: [테일램프 켜짐, 제동 중]. 보조 브레이크등은 제동 전까지 꺼져 있다.
const BRAKE_LAMPS = {tailLens: ["#4a0708", "#a8130e"], tail: ["#e8261c", "#ff7a66"], stop: ["#2e0606", "#ff3a2a"]};

// 차가 비추는 스튜디오: 눈높이에 밝은 창의 띠를 두른 어두운 방(BEV의 것)에, 머리 위로 긴 조명 띠 둘과
// 양옆에 하나씩, 쇼룸이 검은 차를 비추듯 놓는다: 후드, 지붕, 데크는 가는 하이라이트 둘 아래 검게 남는다.
// 위에서 본 서 있는 면은 바닥을 비춘다: 도장과 유리는 어두운 바닥을 받아 옆면이 하이라이트 아래 검게 남고,
// 크롬, 휠, 램프 반사경은 차 밑의 어두운 자리를 두른 밝은 바닥을 받는다. 그러지 않으면 BEV의 어느 시점에서
// 봐도 검게 보인다. 렌더러마다 한 번, 바닥마다 만든다.
function studioScene(lightFloor) {
  const scene = new THREE.Scene();
  const room = new THREE.Mesh(new THREE.BoxGeometry(40, 40, 20), new THREE.MeshBasicMaterial({color: "#0b0e13", side: THREE.BackSide}));
  const floor = new THREE.Mesh(new THREE.RingGeometry(2.1, 40, 48), new THREE.MeshBasicMaterial({color: lightFloor ? "#7d848f" : "#20242b",
                                                                                                 side: THREE.DoubleSide}));
  const under = new THREE.Mesh(new THREE.CircleGeometry(2.1, 48), new THREE.MeshBasicMaterial({color: "#14171c", side: THREE.DoubleSide}));
  floor.position.z = under.position.z = -1.2;                     // 밝은 바닥은 수평선 아래 30도부터 바깥쪽
  const canvas = Object.assign(document.createElement("canvas"), {width: 512, height: 8});
  const c = canvas.getContext("2d");
  c.fillStyle = "#1a1d22";
  c.fillRect(0, 0, 512, 8);
  c.fillStyle = "#ffffff";
  for (let k = 0; k < 14; k++) c.fillRect(k * 512 / 14 + 6, 0, 512 / 14 - 12, 8);       // 기둥 사이의 창
  const windows = new THREE.Mesh(new THREE.CylinderGeometry(14, 14, 2.4, 56, 1, true),
                                 new THREE.MeshBasicMaterial({map: new THREE.CanvasTexture(canvas), color: new THREE.Color(1.1, 1.1, 1.1),
                                                              side: THREE.BackSide}));
  windows.rotation.x = Math.PI / 2;                               // 축이 위(z)로
  windows.position.z = 1.4;
  scene.add(room, floor, under, windows);
  const panel = (w, h, position, rotation, intensity) => {
    const light = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({
      color: new THREE.Color(1, 1, 1).multiplyScalar(intensity), side: THREE.DoubleSide}));
    light.position.set(...position);
    light.rotation.set(...rotation);
    scene.add(light);
  };
  for (const s of [1, -1]) {
    panel(9, 0.4, [0, s * 0.9, 7], [0, 0, 0], 3);                     // 머리 위, 차를 따라
    panel(14, 0.6, [0, s * 8, 2.6], [Math.PI / 2, 0, 0], 3.2);         // 양옆에 하나씩 띠
    panel(0.6, 6, [s * 9, 0, 3], [0, Math.PI / 2, 0], 1.6);            // 그리고 앞뒤
  }
  return scene;
}

const studios = new WeakMap();
function studio(renderer) {
  if (!studios.has(renderer)) {
    const generator = new THREE.PMREMGenerator(renderer);
    studios.set(renderer, [false, true].map((light) => generator.fromScene(studioScene(light), 0.02).texture));
    generator.dispose();
  }
  return studios.get(renderer);
}

// 이 차를 그릴 renderer용으로 재질마다 메시 하나씩 묶은 그룹. 램프는 group.setLamps({left, right, brake})로
// 바꾼다: 방향지시등이 켜진 동안 그 켜진 렌즈가 보이고, 제동은 테일램프를 밝히고 보조 브레이크등을 켠다.
// setLamps()는 바뀐 것이 있는지를 돌려준다.
export function makeEgo(renderer) {
  const parts = new Parts();
  body(parts);
  front(parts);
  rear(parts);
  sides(parts);
  AXLES.forEach((x, a) => { for (const s of [1, -1]) wheel(parts, x, s * TRACKS[a]); });
  const lit = materials(studio(renderer)), group = parts.meshes(lit);
  group.name = "ego";
  const turns = ["turnLeft", "turnRight"].map((name) => group.getObjectByName(name));
  let now = null;
  group.setLamps = ({left = false, right = false, brake = false} = {}) => {
    const state = `${+left}${+right}${+brake}`;
    if (state === now) return false;
    now = state;
    turns[0].visible = left;
    turns[1].visible = right;
    for (const [name, colors] of Object.entries(BRAKE_LAMPS)) lit[name].color.set(colors[brake ? 1 : 0]);
    return true;
  };
  group.setLamps();
  return group;
}
