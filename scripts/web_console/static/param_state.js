// 파라미터 상태: /api/params 스냅샷(그룹·차량 특성 탭 구성, 현재값, 기본값, 항목 메타데이터, 안내가 보는 런타임
// 사실)과 값 바꾸기. 파라미터 탭과 차량 특성 탭이 같이 쓴다. 서버는 구조(섹션 id, 단위, 범위)만 내고, 이름·안내·설명은
// 읽을 때 고른 언어의 문구로 채운다(group.*, vehicle.*, param.<그룹>.<키>.*, unit.*, caution.*, unused.*, role.*,
// toggle.*).
import {codeText, has, t} from "./i18n.js";
import {api} from "./ui.js";

let snapshot = null;
// 값이 저장될 때마다 부르는 함수들(다른 항목의 값에 따라 바뀌는 안내를 다시 고른다). 화면은 스냅샷을 다시 읽을 때
// 새로 만들어지므로 그때 비운다.
let changeListeners = new Set();

const optional = key => (has(key) ? t(key) : "");

// 단위: 언어마다 다르게 쓰는 것만 문구가 있다(unit.s = 초/s). 나머지는 기호 그대로.
export const unitText = unit => (unit ? codeText("unit", unit) : "");

export const sectionName = id => t(`section.${id}`);

// 항목 하나의 글자: 이름, 설명, 줄이거나 끄면·키우거나 켜면, 자동 학습 중의 쓰임(learned), 주의(caution). 이름을
// 붙인 켜고 끄기(labels = [켜면, 끄면], 예: 자동 학습·수동)는 버튼 묶음으로 고르고, choices = [{value, label}]가
// true·false마다 있다(toggle.<이름>).
function describe(group, key, meta) {
  const base = `param.${group}.${key}`;
  return {
    ...meta,
    unit: unitText(meta.unit || ""),
    label: t(`${base}.label`),
    description: t(`${base}.description`),
    increase: t(`${base}.increase`),
    decrease: t(`${base}.decrease`),
    choices: meta.labels ? meta.labels.map((name, index) => ({value: index === 0, label: t(`toggle.${name}`)})) : undefined,
    learned_role: meta.learned ? t(`${base}.learned_role`) : "",
    caution: meta.caution ? t(`caution.${meta.caution}`) : "",
    unused: (meta.unused || []).map(condition => ({...condition, text: t(`unused.${condition.text}`)})),
    role: (meta.role || []).map(condition => ({...condition, text: t(`role.${condition.text}`)})),
  };
}

export async function loadParams() {
  const next = await api.get("/api/params");
  for (const [id, group] of Object.entries(next.groups)) {
    group.label = t(`group.${id}.label`);
    group.note = optional(`group.${id}.note`);
  }
  next.vehicle_tab.label = t("vehicle.label");
  next.vehicle_tab.note = t("vehicle.note");
  for (const [group, items] of Object.entries(next.metadata)) {
    for (const [key, meta] of Object.entries(items)) items[key] = describe(group, key, meta);
  }
  snapshot = next;
  changeListeners = new Set();
  return snapshot;
}

// 저장된 값이 바뀔 때(이 페이지에서 저장한 뒤) listener를 부른다.
export function onParamsChanged(listener) {
  changeListeners.add(listener);
}

// 조건(param_metadata.when)이 참인가: 그 항목의 값(group "runtime"이면 서버가 알린 런타임 사실, 모르면 null)이
// value와 같고, also가 있으면 그것도 참이어야 한다.
function holds(condition, group) {
  const value = condition.group === "runtime" ? snapshot.runtime?.[condition.key]
    : snapshot.params[condition.group || group]?.[condition.key];
  return value === condition.value && (!condition.also || holds(condition.also, group));
}

// 항목 하나에 지금 해당하는 안내: 다른 항목 때문에 쓰이지 않으면 unused, 쓰임이 바뀌면 role(각각 문구 목록).
export function conditionNotes(group, key) {
  const meta = metaFor(group, key);
  return {
    unused: (meta.unused || []).filter(condition => holds(condition, group)).map(condition => condition.text),
    role: (meta.role || []).filter(condition => holds(condition, group)).map(condition => condition.text),
  };
}

export function params() {
  return snapshot;
}

// 메타데이터가 없는 항목(파일에는 있고 메타데이터에는 없는 키)의 기본 설명
export function metaFor(group, key) {
  const meta = snapshot.metadata[group]?.[key];
  if (meta) return meta;
  const value = snapshot.params[group][key];
  return {
    label: key, section: "other", unit: "", step: Number.isInteger(value) ? 1 : 0.01, min: -1e6, max: 1e6,
    description: t("params.undocumented"), increase: t("params.undocumented_up"), decrease: t("params.undocumented_down"),
  };
}

export function valueOf(group, key) {
  return snapshot.params[group][key];
}

export function defaultOf(group, key) {
  return snapshot.defaults?.[group]?.[key];
}

export function isModified(group, key) {
  const fallback = defaultOf(group, key);
  return fallback !== undefined && JSON.stringify(fallback) !== JSON.stringify(valueOf(group, key));
}

function decimals(step) {
  const text = String(step);
  if (text.includes("e-")) return Number(text.split("e-")[1]);
  return text.includes(".") ? text.split(".")[1].length : 0;
}

// 범위 안으로 자르고 step의 자릿수(minDigits가 더 크면 그 자릿수)로 반올림
export function clampToMeta(value, meta, minDigits = 0) {
  return Number(Math.min(meta.max, Math.max(meta.min, value)).toFixed(Math.max(decimals(meta.step), minDigits)));
}

// 값 하나를 저장하고 스냅샷을 갱신한다. {applied, notified}를 돌려주고, 실패하면 예외를 던진다.
export async function saveParam(group, key, value) {
  const update = await api.patch(`/api/params/${group}`, {values: {[key]: value}});
  snapshot.params[group] = update.params;
  for (const listener of changeListeners) {
    try {
      listener(group, key);
    } catch (error) {
      console.error(error);  // 저장은 됐다. 안내 하나를 못 고친 것으로 저장 실패를 알리지 않는다
    }
  }
  return {applied: update.params[key], notified: update.notified || []};
}
