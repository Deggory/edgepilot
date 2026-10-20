// 녹화 탭의 주행 기록 카드(docs/runtime-options.md). recordd가 SD 카드에 쌓은 route마다 시작 시각·길이·크기를 보이고,
// 고른 것을 지운다. 10초마다 읽는다(서버가 route를 훑어 크기를 낸다). 녹화 중이거나 SD로 옮기는 중인 route는 고를 수
// 없고, 서버도 받지 않는다.
import {codeText, locale, t} from "./i18n.js";
import {actionButton, api, badges, DASH, el, helpText, note, pollingCard, runAction, section, syncRows} from "./ui.js";

const INTERVAL_MS = 10000;
const CONFIRM_LISTED = 8;  // 확인 창에 이름을 늘어놓는 수
const UNITS = [[1e9, "GB"], [1e6, "MB"], [1e3, "kB"]];

// SD 카드처럼 1000 단위. 10 아래는 소수 한 자리(3.0 GB, 2.1 GB처럼 자릿수를 맞춘다).
function size(bytes) {
  const [scale, unit] = UNITS.find(([at]) => bytes >= at) || [1, "B"];
  const value = bytes / scale;
  const digits = value < 10 && scale > 1 ? 1 : 0;
  return `${value.toLocaleString(locale, {minimumFractionDigits: digits, maximumFractionDigits: digits})} ${unit}`;
}

// 영상 길이(h:mm:ss, 한 시간 아래는 m:ss). 모르면(쓰는 중, 끝맺지 못한 route) 세그먼트 수로 어림한다(하나가 약 60 s).
function length(route) {
  if (route.duration_s === null) return route.segments ? t("recordings.about_minutes", {count: route.segments}) : DASH;
  const total = Math.round(route.duration_s);
  const pad = value => String(value).padStart(2, "0");
  const [hours, minutes, seconds] = [Math.floor(total / 3600), Math.floor(total / 60) % 60, total % 60];
  return hours ? `${hours}:${pad(minutes)}:${pad(seconds)}` : `${minutes}:${pad(seconds)}`;
}

// route 이름(시작 시각, KST)을 날짜와 시각으로: 2026-10-16--23-07-05-554 → 2026-10-16, 23:07:05
function started(name) {
  const [date, time] = name.split("--");
  return {date, time: time.split("-").slice(0, 3).join(":")};
}

function recorderBadge(recording) {
  if (!recording.available || recording.stale) return [t("recordings.no_recordd"), "muted"];
  if (recording.storage_blocked) return [codeText("recordings.blocked", recording.storage_blocked), "bad"];
  return recording.active ? [t("recordings.recording"), "accent"] : [t("recordings.not_recording"), "muted"];
}

function freeBadge(disk) {
  if (!disk) return null;
  const tone = disk.free_bytes < disk.reserve_bytes ? "bad" : disk.free_bytes < 2 * disk.reserve_bytes ? "warn" : "good";
  return [t("recordings.free", {size: size(disk.free_bytes)}), tone];
}

export function createRecordingsCard() {
  const list = el("div.recordings");
  const selectAll = el("input", {type: "checkbox", "aria-label": t("recordings.select_all")});
  const selectAllRow = el("label.recordings-all", {}, selectAll, el("span", {text: t("recordings.select_all")}));
  const about = helpText(t("recordings.about"), "card-note");  // 한 번 만들어 펼친 상태가 폴링에 접히지 않게
  const button = actionButton({icon: "✕", danger: true, onclick: () => remove()});
  // 이름 -> {element, box, length, size, tags}(ui.js syncRows). 고른 이름은 다음 상태에도 남고, 없어진 route는
  // 고르기에서 빠진다.
  const rows = new Map();
  const selected = new Set();
  let latest = null;
  let busy = false;

  const routesByName = () => new Map((latest ? latest.routes : []).map(route => [route.name, route]));
  const deletable = () => (latest ? latest.routes.filter(route => !route.busy) : []);

  const refreshSelection = () => {
    const routes = routesByName();
    for (const name of [...selected]) if (!routes.has(name) || routes.get(name).busy) selected.delete(name);
    const bytes = [...selected].reduce((sum, name) => sum + routes.get(name).bytes, 0);
    button.setContent(t("recordings.delete"),
                      selected.size ? t("recordings.delete_note", {count: selected.size, size: size(bytes)})
                        : t("recordings.delete_none"));
    button.disabled = busy || !latest || selected.size === 0;
    const candidates = deletable();
    selectAll.checked = candidates.length > 0 && candidates.every(route => selected.has(route.name));
    selectAll.indeterminate = selected.size > 0 && !selectAll.checked;
    selectAll.disabled = busy || candidates.length === 0;
    for (const [name, row] of rows) {
      row.box.checked = selected.has(name);
      row.box.disabled = busy || routes.get(name)?.busy;
    }
  };

  const makeRow = name => {
    const {date, time} = started(name);
    const box = el("input", {type: "checkbox", "aria-label": t("recordings.choose", {name})});
    box.addEventListener("change", () => {
      if (box.checked) selected.add(name);
      else selected.delete(name);
      refreshSelection();
    });
    const row = {box, length: el("span.recording-length"), size: el("span.recording-size"), tags: el("span.badge-row")};
    row.element = el("label.recording",
      {title: name},
      box,
      el("span.recording-start", {}, el("code", {text: date}), el("span", {text: time})),
      row.length, row.size, row.tags);
    return row;
  };

  const render = s => {
    latest = s;
    syncRows(list, rows, s.routes.map(route => route.name), makeRow);
    for (const route of s.routes) {
      const row = rows.get(route.name);
      row.length.textContent = route.busy ? DASH : length(route);
      row.size.textContent = size(route.bytes);
      row.tags.replaceChildren(...badges([
        route.busy ? [t("recordings.in_use"), "accent"] : null,
        !route.busy && route.complete !== true ? [t("recordings.not_closed"), "warn"] : null,
      ]));
      row.element.classList.toggle("busy", route.busy);
    }
    refreshSelection();
    const disk = s.disk;
    return {
      badges: [recorderBadge(s.recording), freeBadge(disk)],
      children: [
        note(t("recordings.summary", {count: s.routes.length, size: size(s.total_bytes)})),
        disk ? note(t("recordings.disk", {free: size(disk.free_bytes), total: size(disk.total_bytes),
                                          reserve: size(disk.reserve_bytes)})) : note(t("recordings.no_disk"), "warn"),
        s.routes.length ? [deletable().length ? selectAllRow : null, list] : note(t("recordings.none")),
        about,
      ],
    };
  };
  const card = pollingCard(t("recordings.card"), {url: "/api/recordings", intervalMs: INTERVAL_MS, render,
                                                   actions: [button]});

  selectAll.addEventListener("change", () => {
    for (const route of deletable()) {
      if (selectAll.checked) selected.add(route.name);
      else selected.delete(route.name);
    }
    refreshSelection();
  });

  const remove = async () => {
    const routes = routesByName();
    const names = [...selected].filter(name => routes.has(name)).sort().reverse();
    if (!names.length) return;
    const bytes = names.reduce((sum, name) => sum + routes.get(name).bytes, 0);
    const listed = names.slice(0, CONFIRM_LISTED).map(name => {
      const {date, time} = started(name);
      return `${date} ${time} · ${size(routes.get(name).bytes)}`;
    });
    const result = await runAction({
      confirm: [t("recordings.confirm", {count: names.length, size: size(bytes)}), ...listed,
                names.length > CONFIRM_LISTED ? t("recordings.confirm_more", {count: names.length - CONFIRM_LISTED}) : null,
                "", t("recordings.confirm_note")],
      busy: on => {
        busy = on;
        refreshSelection();
      },
      call: () => api.post("/api/recordings/delete", {names}),
      done: r => t("recordings.deleted", {count: r.deleted.length, size: size(r.freed_bytes)}),
      failed: error => t("recordings.delete_failed", {error}),
    });
    if (result) {
      selected.clear();
      card.show(result);
    }
  };

  button.setContent(t("recordings.delete"), t("recordings.delete_none"));  // 첫 상태가 오기 전(막혀 있다)
  return {element: section(t("recordings.section"), [card.element]), after: "recording", start: card.start,
          stop: card.stop};
}
