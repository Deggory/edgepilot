// BEV 모듈(bev.js, bev_data.js, bev_car.js, bev_ego.js)이 같이 쓰는 작은 수학: 범위 자르기, 매끈한 계단, 3차원
// 벡터, 그리고 차 모델을 x를 따라 로프트할 때의 스테이션과 휠 아치.
export const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
// x가 a에서 b로 갈 때 0에서 1로 매끈하게(smoothstep). a > b면 거꾸로 줄어든다.
export const smoothstep = (a, b, x) => {
  const u = clamp((x - a) / (b - a), 0, 1);
  return u * u * (3 - 2 * u);
};
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

// x0부터 x1까지의 스테이션: 안쪽에 든 marks의 모든 점에, 그리고 적어도 step마다.
export function stations(x0, x1, step, marks = []) {
  const xs = [x0, x1, ...marks.filter((x) => x > x0 && x < x1)];
  for (let x = x0 + step; x < x1; x += step) xs.push(x);
  xs.sort((a, b) => a - b);
  return xs.filter((x, i) => i === 0 || x - xs[i - 1] > 2e-3);
}

// 중심이 centres이고 반지름 ra인 휠 아치 둘레의 스테이션: 각 아치 발 바로 바깥과 아치를 따라 steps 등분.
export function archMarks(centres, ra, steps = 6) {
  const xs = [];
  for (const c of centres) {
    xs.push(c - ra - 0.01, c + ra + 0.01);
    for (let i = 0; i <= steps; i++) xs.push(c - ra * Math.cos(Math.PI * i / steps));
  }
  return xs;
}

// 밑면 높이 bottom을, 높이 zc를 중심으로 한 반지름 ra의 휠 아치마다 그 둘레에서 들어 올린다.
export function archBottom(x, centres, ra, zc, bottom) {
  let z = bottom;
  for (const c of centres) {
    const dx = Math.abs(x - c);
    if (dx <= ra + 1e-6) z = Math.max(z, zc + Math.sqrt(Math.max(0, ra * ra - dx * dx)));
  }
  return z;
}
