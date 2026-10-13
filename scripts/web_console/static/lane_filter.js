// controlsd의 플래너가 자차 차로의 두 선에 거는 차선선 필터(src/planning/lane_line_filter.cc)다. 그래서
// BEV가 조향이 따르는 선을 그린다. C++와 한 줄 한 줄 같다: 직전에 거른 선을 자차 움직임(실측 곡률로
// v·dt만큼 간 원호)만큼 옮기고, 모델의 점 거리에서 세 점 2차 보간으로 읽어 새 프레임과 섞는다.
// check_web_console.py가 상수를 C++ 상수와 맞춰 보고 tests/check_bev_lane_filter.mjs를 돌리며, 이
// 스크립트가 이 파일을 C++ 출력과 비교한다. docs/lane-line-filter.md

export const LANE_FILTER_WEIGHT = 0.5;  // lane_line_filter.h kLaneLineFilterWeight
const MAX_GAP_S = 0.2;                  // lane_line_filter.cc kMaxGapS: 이보다 긴 끊김은 다시 시작한다
const RESTART_REACH_M = 20.0;           // kRestartReachM: 다시 시작을 판단하는 가까운 점의 범위
const RESTART_M = 0.5;                  // kRestartM: 그 점들이 예측에서 평균 이만큼 벗어나면 다시 시작한다

// px[j] <= x <= px[j + 1]인 x에서 (px, py)를 지나는 선의 값: j, j + 1과 x에 더 가까운 쪽 이웃을 지나는
// 2차식이고, 점이 겹치는 곳에서는 직선이다.
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

  // x와 y는 선 하나의 점(카메라에서 m, y는 오른쪽), t는 캡처 시각(s), v는 자차 속도(m/s), curvature는 실측
  // 곡률(1/m, 오른쪽이 +)이다. 거른 y를 돌려준다.
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
