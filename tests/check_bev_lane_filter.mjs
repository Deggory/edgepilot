// The BEV's copy of the lane-line filter (scripts/web_console/static/lane_filter.js) against the C++ one
// (src/planning/lane_line_filter.cc): the same scenario gives the same lines as the C++ golden output, which
// gtest_lane_line_filter (GoldenSharedWithTheBev) holds to the C++ code. check_web_console.py runs this with
// jsc (macOS) or node when one is installed, and checks that both files carry the same golden numbers.
import {LaneLineFilter, LANE_FILTER_WEIGHT} from "../scripts/web_console/static/lane_filter.js";

const say = globalThis.print ?? console.log;
let failures = 0;
const check = (ok, what) => {
  if (!ok) failures++;
  say(`${ok ? "ok  " : "FAIL"} ${what}`);
};

const N = 33;
const X = Array.from({length: N}, (_, i) => Math.fround(192 * (i / 32) * (i / 32)));

// GOLDEN BEGIN: frame -> the filtered line at points 0, 8, 12, 16, 20, 24 (C++ output)
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

// 40 frames at 25 m/s: a curving line that turns as the car yaws, a fixed pattern of noise, a lane change
// at frame 25 (the lines jump 3.5 m) and a 0.35 s gap before frame 32
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

// weight 1 hands the measurement through
{
  const filter = new LaneLineFilter(1);
  const y = X.map((_, i) => -1.7 + 0.001 * i);
  filter.update(0, X, y, 20, 0.001);
  check(filter.update(0.05, X, y, 20, 0.001).every((v, i) => v === y[i]), "weight 1 is off");
}

// following a circle the lines stay put in the car's frame; the quadratic read keeps them there (a
// straight-line read drags them 8-17 mm toward the inside)
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
