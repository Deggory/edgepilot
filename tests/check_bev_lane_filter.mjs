// BEV의 차선선 필터 사본(scripts/web_console/static/lane_filter.js)과 C++ 필터(src/planning/lane_line_filter.cc):
// 같은 장면이면 C++ 기준 출력과 같은 선을 내야 한다. 그 기준값은 gtest_lane_line_filter(GoldenSharedWithTheBev)가
// C++ 코드에 묶어 둔다. check_web_console.py가 jsc(macOS)나 node가 있으면 이 파일을 돌리고, 두 파일의 기준 숫자가
// 같은지 본다.
import {LaneLineFilter, LANE_FILTER_WEIGHT} from "../scripts/web_console/static/lane_filter.js";

const say = globalThis.print ?? console.log;
let failures = 0;
const check = (ok, what) => {
  if (!ok) failures++;
  say(`${ok ? "ok  " : "FAIL"} ${what}`);
};

const N = 33;
const X = Array.from({length: N}, (_, i) => Math.fround(192 * (i / 32) * (i / 32)));

// GOLDEN BEGIN: 프레임 -> 0, 8, 12, 16, 20, 24번 점의 거른 선(C++ 출력)
const GOLDEN = [
  [10, [-1.7860593, -1.6711308, -1.4910269, -1.1393055, -0.5273346, 0.4499038]],
  [24, [-1.7755761, -1.4648968, -1.0241125, -0.2886234, 0.8415040, 2.5161421]],
  [25, [1.7029999, 2.0276000, 2.4753499, 3.2546000, 4.4127498, 6.1616001]],
  [31, [1.7403402, 2.1345892, 2.6871150, 3.6010313, 4.9587660, 6.9622140]],
  [32, [1.7120000, 2.1226161, 2.6835561, 3.5988560, 5.0254998, 7.0568962]],
  [39, [1.7414960, 2.2348645, 2.9326613, 4.0234733, 5.6730695, 8.0137234]],
];
// GOLDEN END
const POINTS = [0, 8, 12, 16, 20, 24];

// 25 m/s로 40프레임: 차가 요잉하는 만큼 휘는 선, 정해진 잡음 무늬, 25번 프레임의 차선 변경(선이 3.5 m
// 뛴다), 32번 프레임 앞의 0.35 s 끊김
{
  const filter = new LaneLineFilter();
  let worst = 0;
  for (let k = 0; k < 40; k++) {
    const t = 100 + 0.05 * k + (k >= 32 ? 0.3 : 0);
    const curvature = k < 20 ? 0.001 : -0.0005;
    const y = X.map((xi, i) => {
      let v = -1.8;
      v += 0.001 * k * xi;
      v += (1e-4 + 2e-6 * k) * xi * xi;
      v += ((i * 7 + k * 13) % 11 - 5) * 0.003;
      if (k >= 25) v += 3.5;
      return Math.fround(v);
    });
    const out = filter.update(t, X, y, 25, curvature);
    const golden = GOLDEN.find(([frame]) => frame === k);
    if (golden) POINTS.forEach((p, n) => { worst = Math.max(worst, Math.abs(out[p] - golden[1][n])); });
  }
  check(worst < 1e-5, `matches the C++ output (worst ${worst.toExponential(1)} m)`);
}

// 가중치 1은 측정을 그대로 넘긴다
{
  const filter = new LaneLineFilter(1);
  const y = X.map((_, i) => -1.7 + 0.001 * i);
  filter.update(0, X, y, 20, 0.001);
  check(filter.update(0.05, X, y, 20, 0.001).every((v, i) => v === y[i]), "weight 1 is off");
}

// 원을 따라 돌면 선은 차 좌표에서 제자리에 있다. 2차식으로 읽으면 그대로 남고, 직선으로 읽으면 안쪽으로
// 8~17 mm 끌린다
{
  const kappa = 0.002, offset = -1.8, r = 1 / kappa - offset;
  const y = X.map((x) => Math.fround(1 / kappa - Math.sqrt(Math.max(0, r * r - x * x))));
  const filter = new LaneLineFilter();
  let out = [];
  for (let k = 0; k < 200; k++) out = filter.update(k * 0.05, X, y, 25, kappa);
  const near = X.map((x, i) => [x, out[i] - y[i]]).filter(([x]) => x >= 10 && x <= 40);
  const bias = near.reduce((s, [, e]) => s + e, 0) / near.length;
  check(Math.abs(bias) < 0.002, `no bias on a curve (${(bias * 1000).toFixed(2)} mm)`);
}

check(LANE_FILTER_WEIGHT === 0.5, "default weight");
if (failures) throw new Error(`${failures} failed`);
say("all passed");
