// 웹 콘솔 공용 도구: DOM 만들기, 카드·배지·표, 토스트, 확인 창, API 호출, 주기 폴링, 숫자 표기.
//
// 톤(배지·안내·상태 글자의 색)은 어느 카드에서나 같은 뜻이다: good 동작·사용 중, warn 기다림·주의, bad 없음·실패·
// 낡음, accent 진행 중·고름, muted 그 밖(정보).
import {locale, t} from "./i18n.js";

// el("div.card.live", {title: "..."}, child, "text", ...): 태그 이름 뒤 .클래스, 속성(on<event>는 리스너),
// 자식(노드·문자열·배열, null·false는 건너뜀).
export function el(spec, attributes = {}, ...children) {
  const [tag, ...classes] = spec.split(".");
  const node = document.createElement(tag || "div");
  if (classes.length) node.className = classes.join(" ");
  for (const [name, value] of Object.entries(attributes)) {
    if (value === undefined || value === null || value === false) continue;
    if (name.startsWith("on") && typeof value === "function") node.addEventListener(name.slice(2), value);
    else if (name === "dataset") Object.assign(node.dataset, value);
    else if (name === "text") node.textContent = value;
    else if (name in node && typeof node[name] !== "function" && name !== "list") node[name] = value;
    else node.setAttribute(name, value === true ? "" : String(value));
  }
  node.append(...children.flat(Infinity).filter(child => child !== null && child !== undefined && child !== false));
  return node;
}

function svg(tag, attributes = {}) {
  const node = document.createElementNS("http://www.w3.org/2000/svg", tag);
  for (const [name, value] of Object.entries(attributes)) node.setAttribute(name, String(value));
  return node;
}

// ------------------------------------------------------------ 작은 시간 차트
// 차량 특성 탭의 추이와 BEV의 출발 알림·앞차 카드가 같이 쓴다. sparkline()은 이름 붙인 선(series)과 가로 기준선
// (levels)이 있는 SVG 하나이고, 이름이 곧 CSS 클래스라 색·굵기·점선은 스타일 시트가 정한다(굵기는 차트 크기와 상관없이
// vector-effect: non-scaling-stroke). 한 번 만들고 draw()로 다시 그린다:
//   x, y      [처음, 끝] 범위(데이터 단위). y 범위 밖의 값은 끝에 붙인다
//   data      {선 이름: [[x, y], ...]}, 선마다 있다. 값이 없는 점(NaN)에서 선을 끊는다
//   at        {기준선 이름: y}. 없거나 NaN이면 그 선을 숨긴다
//   inset     위아래 여백(차트 높이에 대한 비율)
export function sparkline({className = "", height = 60, series = [], levels = [], label = ""}) {
  const element = svg("svg", {class: className, viewBox: `0 0 100 ${height}`, preserveAspectRatio: "none", role: "img",
                              "aria-label": label});
  const lines = new Map(levels.map(name => [name, element.appendChild(svg("line", {class: name, x1: 0, x2: 100}))]));
  const paths = new Map(series.map(name => [name, element.appendChild(svg("path", {class: name}))]));
  return {
    element,
    draw({x: [x0, x1], y: [lo, hi], data, at = {}, inset = 0}) {
      const px = v => ((v - x0) / (x1 - x0 || 1)) * 100;
      const py = v => height * (inset + (1 - 2 * inset) * (1 - (Math.min(hi, Math.max(lo, v)) - lo) / (hi - lo || 1)));
      for (const [name, path] of paths) {
        path.setAttribute("d", gapPath(data[name].map(([u, v]) => [px(u), Number.isFinite(v) ? py(v) : NaN])));
      }
      for (const [name, line] of lines) {
        const v = at[name];
        line.style.display = Number.isFinite(v) ? "" : "none";
        if (Number.isFinite(v)) for (const end of ["y1", "y2"]) line.setAttribute(end, py(v).toFixed(1));
      }
    },
  };
}

// 차트의 y 범위 [lo, hi]. 보이는 값(NaN은 뺀다)의 최저에서 최고까지를 적어도 minSpan으로 가운데에서 넓히고, 위아래로
// 그 폭의 pad배씩 더 둔다. symmetric이면 0이 가운데인 [-p, p]이고 p는 minSpan/2 이상에서 step의 배수로 올린다(상대
// 속도처럼 부호가 뜻을 가질 때). 값이 하나도 없으면 fallback을 같은 방식으로 넓힌다.
export function fitRange(values, {minSpan = 0, pad = 0, symmetric = false, step = 0, fallback = [0, 1]} = {}) {
  const finite = values.filter(Number.isFinite);
  if (symmetric) {
    const peak = Math.max(minSpan / 2, ...finite.map(Math.abs));
    const rounded = step ? Math.ceil(peak / step) * step : peak;
    return [-rounded, rounded];
  }
  let [lo, hi] = finite.length ? [Math.min(...finite), Math.max(...finite)] : fallback;
  if (hi - lo < minSpan) {
    const middle = (lo + hi) / 2;
    [lo, hi] = [middle - minSpan / 2, middle + minSpan / 2];
  }
  const margin = (hi - lo) * pad;
  return [lo - margin, hi + margin];
}

// 값이 없는 곳(y가 NaN)에서 끊기는 SVG path의 d. points는 그림 좌표의 [[x, y], ...]다.
function gapPath(points) {
  let d = "", gap = true;
  for (const [x, y] of points) {
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      gap = true;
      continue;
    }
    d += `${gap ? "M" : "L"}${x.toFixed(1)},${y.toFixed(1)}`;
    gap = false;
  }
  return d;
}

// ------------------------------------------------------------ 숫자 표기
export const DASH = "–";  // 값이 없을 때

export function num(value, digits) {
  return Number.isFinite(value) ? value.toFixed(digits) : DASH;
}

export function sgn(value, digits) {
  if (!Number.isFinite(value)) return DASH;
  return `${value < 0 ? "−" : "+"}${Math.abs(value).toFixed(digits)}`;
}

// 값과 단위: 언제나 한 칸 띄운다(50 %, 1.8 s)
export function withUnit(value, unit) {
  return unit ? `${value} ${unit}` : String(value);
}

export function deg(rad) {
  return rad * 180 / Math.PI;
}

export function ms(seconds) {
  return `${Math.round(seconds * 1000)} ms`;
}

export function clock(date = new Date()) {
  return date.toLocaleTimeString(locale, {hour12: false});
}

// ------------------------------------------------------------ 배지·표·카드
// [텍스트, 톤] 목록(맨 위의 톤 규칙). null은 건너뛴다.
export function badges(list) {
  return list.filter(Boolean).map(([text, tone]) => el(`span.badge.${tone || "muted"}`, {text}));
}

// 이미 있는 배지 하나를 [텍스트, 톤]으로 바꾼다
export function setBadge(node, [text, tone]) {
  node.className = `badge ${tone || "muted"}`;
  node.textContent = text;
}

// 낡은 상태(서버 freshness의 stale)이면 "N초 전" 배지, 상태가 없거나 신선하면 null
export function staleBadge(state) {
  return state && state.available && state.stale ? [t("common.ago", {seconds: Math.round(state.age_s)}), "bad"] : null;
}

// 머리 줄과 행들. 셀은 문자열이나 [문자열, 톤]. 둘째 열은 값(굵게), 넷째 열은 설명(흐리게).
export function infoTable(head, rows) {
  const cell = (tag, value, kind) => {
    const [text, tone] = Array.isArray(value) ? value : [value, ""];
    return el(`${tag}${kind ? `.${kind}` : ""}${tone ? `.${tone}` : ""}`, {text});
  };
  return el("table.info-table",
    {},
    el("thead", {}, el("tr", {}, head.map(text => el("th", {text})))),
    el("tbody", {}, rows.map(row => el("tr", {}, row.map((value, index) =>
      cell("td", value, index === 1 ? "value" : index === 3 ? "note" : ""))))));
}

// 상태 카드: 제목·배지·(머리의 컨트롤)·본문·버튼 줄. 머리와 버튼은 한 번 만들고 본문과 배지만 바꾼다
// (누르는 중인 스위치를 매초 갈아엎지 않게). 버튼은 첫 상태가 오기 전과 읽지 못한 동안(fail) 막아 둔다: 버튼을 다시
// 여는 것은 그 카드가 상태를 보고 한다.
export function liveCard(title, {control = null, actions = []} = {}) {
  const badgeRow = el("span.badge-row");
  const body = el("div.card-body");
  for (const action of actions) action.disabled = true;
  const element = el("article.card",
    {},
    el("header.card-head", {}, el("h3.card-title", {text: title}), badgeRow, control),
    body,
    actions.length ? el("div.action-row", {}, actions) : null);
  const update = (badgeList, children) => {
    badgeRow.replaceChildren(...badges(badgeList));
    body.replaceChildren(...children.flat().filter(Boolean));
  };
  return {
    element,
    update,
    fail(error) {
      for (const action of actions) action.disabled = true;
      update([[t("common.read_failed"), "bad"]], [note(error.message, "warn")]);
    },
  };
}

// url(또는 url을 돌려주는 함수)을 intervalMs마다 읽어 onData(응답)를, 읽지 못하면 onError(오류)를 부른다. start()와
// stop()은 poller와 같다.
export function pollJson(url, intervalMs, onData, onError) {
  return poller(async isCurrent => {
    try {
      const data = await api.get(typeof url === "function" ? url() : url);
      if (isCurrent()) onData(data);
    } catch (error) {
      if (isCurrent()) onError(error);
    }
  }, intervalMs);
}

// 서버 상태 하나를 intervalMs마다 읽어 그리는 상태 카드(Panda 펌웨어, 학습 토크 모델). render(data)가 {badges,
// children}을 돌려준다. 읽지 못하면 어느 카드나 같은 모양(liveCard.fail)이다. show(data)는 요청의 응답을 곧바로 그릴
// 때, redraw()는 다른 항목이 바뀌어 마지막 상태를 다시 그릴 때 쓴다(읽지 못한 뒤에는 옛 상태로 덮지 않는다).
export function pollingCard(title, {url, intervalMs, render, actions = []}) {
  const card = liveCard(title, {actions});
  let latest = null;
  const show = data => {
    latest = data;
    const {badges: badgeList, children} = render(data);
    card.update(badgeList, children);
  };
  const polling = pollJson(url, intervalMs, show, error => {
    latest = null;
    card.fail(error);
  });
  return {element: card.element, show, redraw: () => latest && show(latest), start: polling.start, stop: polling.stop};
}

// 이름마다 한 줄인 목록(학습 토크 모델, 주행 기록)을 names에 맞춘다. rows는 이름 -> make(이름)이 만든 줄({element,
// ...})이다. 이름 목록이 바뀔 때만 줄을 다시 만든다: 폴링마다 갈아 끼우면 누르는 사이에 바뀐 줄이 탭이나 체크를 놓치고
// 키보드 초점도 사라진다.
export function syncRows(list, rows, names, make) {
  if (names.join("\n") === [...rows.keys()].join("\n")) return;
  rows.clear();
  for (const name of names) rows.set(name, make(name));
  list.replaceChildren(...[...rows.values()].map(row => row.element));
}

// 버튼 하나의 요청. confirm(확인 창 줄 목록, 첫 줄이 질문)이 있으면 먼저 묻는다. 도는 동안 busy(true), 끝나면
// busy(false)를 부르고, 성공하면 done(응답)의 문구를, 실패하면 failed(이유)의 문구를 토스트로 알린다. 응답을
// 돌려주고, 취소하거나 실패하면 null이다.
export async function runAction({confirm = null, busy = () => {}, call, done = null, failed}) {
  if (confirm && !confirmAction(confirm)) return null;
  busy(true);
  try {
    const result = await call();
    if (done) toast(done(result));
    return result;
  } catch (error) {
    toast(failed(error.message), {failed: true});
    return null;
  } finally {
    busy(false);
  }
}

// 몇 개 중 하나를 고르는 버튼 묶음(학습기의 자동 학습·수동, 보정 출처). options는 [{value, label,
// title}]이고, 지금 값이 아닌 것을 누르면 onChoose(value)를 부른다. show(value)가 누른 것을 칠한다(없는 값이면 아무
// 것도 칠하지 않는다).
export function segmented(label, options, onChoose) {
  let current;
  const buttons = options.map(option => el("button",
    {type: "button", text: option.label, title: option.title, onclick: () => option.value !== current && onChoose(option.value)}));
  return {
    element: el("div.segmented", {role: "group", "aria-label": label}, buttons),
    show(value) {
      current = value;
      buttons.forEach((button, index) => button.setAttribute("aria-pressed", String(options[index].value === value)));
    },
    setDisabled(disabled) {
      for (const button of buttons) button.disabled = disabled;
    },
  };
}

// 카드 아래쪽의 큰 버튼: 아이콘, 제목, 작은 설명 줄들. danger면 붉은 테.
export function actionButton({icon, danger = false, onclick}) {
  const button = el(`button.action${danger ? ".danger" : ""}`, {type: "button", onclick});
  button.setContent = (title, ...notes) => {
    button.replaceChildren(el("span.action-icon", {text: icon}),
      el("span.action-text", {}, title, notes.filter(Boolean).map(note => el("small", {text: note}))));
  };
  return button;
}

export function note(text, tone = "") {
  return el(`p.card-note${tone ? `.${tone}` : ""}`, {text});
}

// 긴 설명(항목 설명, 카드의 긴 안내): 처음에는 세 줄까지 보이고, 넘치면 "더 보기"로 펼친다. 넘치지 않으면 버튼이
// 없다(화면에 붙은 뒤 크기가 정해질 때마다 잰다). className은 글자 모양(param-description, card-note, group-note).
// 줄 자르기는 안쪽 글자에 건다: 안쪽 여백이 있는 상자(group-note)에 걸면 넷째 줄이 여백에 반쯤 보인다.
export function helpText(text, className = "param-description") {
  const body = el("span.help-body.clamped", {text});
  const more = el("button.link-button.help-more", {type: "button", hidden: true, "aria-expanded": "false",
                                                   text: t("common.more")});
  const toggle = () => {
    const open = body.classList.toggle("clamped") === false;
    more.textContent = open ? t("common.less") : t("common.more");
    more.setAttribute("aria-expanded", String(open));
  };
  more.addEventListener("click", toggle);
  body.addEventListener("click", () => more.hidden || toggle());
  new ResizeObserver(() => {
    if (body.classList.contains("clamped")) more.hidden = body.scrollHeight <= body.clientHeight + 1;
  }).observe(body);
  return el("div.help", {}, el(`p.${className}`, {}, body), more);
}

// 섹션: 제목(count가 있으면 항목 수), 설명 한 줄과 오른쪽의 컨트롤(aside, 예: 학습기의 자동 학습·수동), 본문.
// grid면 본문을 카드 격자로 놓는다.
export function section(title, children, {count = null, grid = true, description = "", aside = null} = {}) {
  const heading = el("h2.section-title", {}, title, count === null ? null : el("span", {text: t("common.count", {count})}));
  const head = description || aside
    ? el("div.section-head", {}, el("div.section-heading", {}, heading, description ? el("p.section-description", {text: description}) : null), aside)
    : heading;
  return el("section.section", {}, head, el(grid ? "div.card-grid" : "div", {}, children));
}

// ------------------------------------------------------------ 토스트·확인
let toastRoot = null;

export function toast(text, {failed = false} = {}) {
  toastRoot = toastRoot || document.getElementById("toasts");
  const item = el(`div.toast${failed ? ".failed" : ""}`, {role: failed ? "alert" : "status", text});
  toastRoot.append(item);
  window.setTimeout(() => item.classList.add("leaving"), failed ? 5000 : 1800);
  window.setTimeout(() => item.remove(), failed ? 5400 : 2200);
}

// 줄 목록을 확인 창으로. 첫 줄이 질문이다.
export function confirmAction(lines) {
  return window.confirm(lines.filter(line => line !== null && line !== undefined).join("\n"));
}

// ------------------------------------------------------------ API
// GET의 시한. 보드가 말없이 사라지면(전원, Wi-Fi) 응답이 오지 않아 폴링이 멈추고 화면은 마지막 값을 정상처럼 보였다.
// POST·PATCH에는 두지 않는다: 서버가 처리했는데 늦게 답하면 실패로 알리게 된다.
const GET_TIMEOUT_MS = 10000;

async function request(method, url, body, signal, timeoutMs = 0) {
  const controller = new AbortController();
  const cancel = () => controller.abort();
  if (signal?.aborted) cancel();
  signal?.addEventListener("abort", cancel, {once: true});
  let timedOut = false;
  const timer = timeoutMs ? window.setTimeout(() => {
    timedOut = true;
    controller.abort();
  }, timeoutMs) : null;
  try {
    const response = await fetch(url, {
      method,
      signal: controller.signal,
      cache: "no-store",
      headers: body === undefined ? {} : {"Content-Type": "application/json"},
      body: body === undefined ? undefined : JSON.stringify(body),
    });
    const data = await response.json().catch(error => {
      if (timedOut) throw error;  // 본문을 받다가 시한이 지났다
      return {};
    });
    if (!response.ok) throw new Error(errorText(data.detail) || response.statusText || `HTTP ${response.status}`);
    return data;
  } catch (error) {
    if (timedOut) throw new Error(t("common.no_response", {seconds: timeoutMs / 1000}));
    throw error;
  } finally {
    window.clearTimeout(timer);
    signal?.removeEventListener("abort", cancel);
  }
}

// 서버가 받지 않은 이유: 문구 키({code, 자리 값}, 예: 결합 중이라 거부) 또는 글자. FastAPI의 검사 오류(목록)는
// 상태 글자로 대신한다.
function errorText(detail) {
  if (typeof detail === "string") return detail;
  return detail && typeof detail === "object" && typeof detail.code === "string" ? t(detail.code, detail) : "";
}

export const api = {
  get: (url, {signal} = {}) => request("GET", url, undefined, signal, GET_TIMEOUT_MS),
  post: (url, body = {}) => request("POST", url, body),
  patch: (url, body) => request("PATCH", url, body),
};

// fn을 interval_ms마다 부른다(겹치지 않게). start()는 곧바로 한 번 부르고, stop() 뒤에는 늦게 끝난 호출도
// 버린다.
export function poller(fn, intervalMs) {
  let timer = null;
  let generation = 0;
  const tick = async current => {
    try {
      await fn(() => current === generation);
    } finally {
      if (current === generation) timer = window.setTimeout(() => tick(current), intervalMs);
    }
  };
  return {
    start() {
      if (timer !== null) return;
      generation += 1;
      timer = 0;
      tick(generation);
    },
    stop() {
      generation += 1;
      if (timer) window.clearTimeout(timer);
      timer = null;
    },
  };
}
