// The lane-line filter controlsd's planner runs on the two ego lines (src/planning/lane_line_filter.cc), so
// the BEV draws the lines steering follows. Line for line the same: the last filtered line moved by the
// car's own motion (an arc of the measured curvature over v·dt), read at the model's point distances with a
// three-point quadratic interpolation and blended with the new frame. check_web_console.py holds the
// constants to the C++ ones and runs tests/check_bev_lane_filter.mjs, which checks this file against the
// C++ output. docs/lane-line-filter.md

export const LANE_FILTER_WEIGHT = 0.5;  // lane_line_filter.h kLaneLineFilterWeight
const MAX_GAP_S = 0.2;                  // lane_line_filter.cc kMaxGapS: a longer gap starts again
const RESTART_REACH_M = 20.0;           // kRestartReachM: the near points that judge a restart
const RESTART_M = 0.5;                  // kRestartM: their mean distance from the prediction that restarts

// The line through (px, py) at x, with px[j] <= x <= px[j + 1]: a quadratic through j, j + 1 and the
// neighbour nearer x, or a straight line where points coincide.
function interpolate(px, py, j, x) {
  const n = px.length, span = px[j + 1] - px[j];
  if (!(span > 1e-6)) return py[j + 1];
  const left = j > 0 && (x - px[j] < px[j + 1] - x || j + 2 >= n);
  const a = left ? j - 1 : j;
  if (a + 2 >= n || !(px[a + 1] - px[a] > 1e-6) || !(px[a + 2] - px[a + 1] > 1e-6)) {
    return py[j] + (x - px[j]) / span * (py[j + 1] - py[j]);
  }
  const x0 = px[a], x1 = px[a + 1], x2 = px[a + 2];
  return py[a] * (x - x1) * (x - x2) / ((x0 - x1) * (x0 - x2)) +
         py[a + 1] * (x - x0) * (x - x2) / ((x1 - x0) * (x1 - x2)) +
         py[a + 2] * (x - x0) * (x - x1) / ((x2 - x0) * (x2 - x1));
}

export class LaneLineFilter {
  constructor(weight = LANE_FILTER_WEIGHT) {
    this.weight = weight;
    this.have = false;
    this.t = 0;
    this.x = [];
    this.y = [];
  }

  reset() {
    this.have = false;
  }

  // x and y are one line's points (m from the camera, y to the right), t the capture time (s), v our
  // speed (m/s), curvature the measured one (1/m, + to the right). Returns the filtered y.
  update(t, x, y, v, curvature) {
    const n = x.length;
    if (![...x, ...y].every(Number.isFinite)) {
      this.have = false;
      return Array.from(y);
    }
    const dt = t - this.t;
    let restart = !this.have || !(dt > 0) || dt > MAX_GAP_S || this.weight >= 1;
    const predicted = new Array(n);
    if (!restart) {
      const d = (v > 0 ? v : 0) * dt, theta = curvature * d;
      const straight = Math.abs(theta) < 1e-9;
      const dx = straight ? d : d * Math.sin(theta) / theta;
      const dy = straight ? 0.5 * d * theta : d * (1 - Math.cos(theta)) / theta;
      const c = Math.cos(theta), s = Math.sin(theta);
      const px = new Array(n), py = new Array(n);
      for (let j = 0; j < n; j++) {
        const qx = this.x[j] - dx, qy = this.y[j] - dy;
        px[j] = c * qx + s * qy;
        py[j] = -s * qx + c * qy;
        if (j > 0) px[j] = Math.max(px[j], px[j - 1]);
      }
      let j = 0, error = 0, near = 0;
      for (let i = 0; i < n; i++) {
        if (x[i] > px[n - 1]) {
          predicted[i] = y[i];
        } else if (x[i] <= px[0]) {
          predicted[i] = py[0];
        } else {
          while (j + 1 < n - 1 && px[j + 1] < x[i]) j++;
          predicted[i] = interpolate(px, py, j, x[i]);
        }
        if (x[i] <= RESTART_REACH_M) {
          error += Math.abs(y[i] - predicted[i]);
          near++;
        }
      }
      restart = near > 0 && error / near > RESTART_M;
    }
    const out = new Array(n);
    for (let i = 0; i < n; i++) out[i] = restart ? y[i] : predicted[i] + this.weight * (y[i] - predicted[i]);
    this.x = Array.from(x);
    this.y = out.slice();
    this.t = t;
    this.have = true;
    return out;
  }
}
