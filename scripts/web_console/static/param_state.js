// 파라미터 상태: /api/params 스냅샷(그룹·차량 특성 탭 구성, 현재값, 기본값, 항목 메타데이터)과 값 바꾸기. 파라미터
// 탭과 차량 특성 탭이 같이 쓴다. 서버는 구조(섹션 id, 단위, 범위)만 내고, 이름·안내·설명은 읽을 때 고른 언어의
// 문구로 채운다(group.*, vehicle.*, param.<그룹>.<키>.*, unit.*, caution.*).
import {has, t} from "./i18n.js";
import {api} from "./ui.js";

let snapshot = null;

const optional = key => (has(key) ? t(key) : "");

// 단위: 언어마다 다르게 쓰는 것만 문구가 있다(unit.s = 초/s). 나머지는 기호 그대로.
export const unitText = unit => (unit && has(`unit.${unit}`) ? t(`unit.${unit}`) : unit);

export const sectionName = id => t(`section.${id}`);

// 항목 하나의 글자: 이름, 설명, 줄이거나 끄면·키우거나 켜면(고르는 항목은 선택지마다 이름과 설명), 자동 학습 중의
// 쓰임(learned), 주의(caution)
function describe(group, key, meta) {
  const base = `param.${group}.${key}`;
  const choice = meta.control === "choice";
  return {
    ...meta,
    unit: unitText(meta.unit || ""),
    label: t(`${base}.label`),
    description: t(`${base}.description`),
    increase: choice ? "" : t(`${base}.increase`),
    decrease: choice ? "" : t(`${base}.decrease`),
    options: choice ? meta.options.map(name => ({
      name, label: t(`${base}.option.${name}.label`), description: t(`${base}.option.${name}.description`),
    })) : undefined,
    learned_role: meta.learned ? t(`${base}.learned_role`) : "",
    caution: meta.caution ? t(`caution.${meta.caution}`) : "",
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
  return snapshot;
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
  return {applied: update.params[key], notified: update.notified || []};
}
