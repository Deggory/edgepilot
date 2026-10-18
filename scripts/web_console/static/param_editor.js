// 파라미터 항목 하나를 고치는 부품: 컨트롤(켜고 끄기, 버튼 묶음, −/+, 슬라이더), 저장(결과를 상태 줄에), 바뀜 표시와
// 기본값으로 되돌리기, 그리고 이것들로 짠 기본 항목 카드. 파라미터 탭과 차량 특성 탭이 같이 쓴다.
import {t} from "./i18n.js";
import {
  clampToMeta, conditionNotes, defaultOf, isModified, metaFor, onParamsChanged, params, saveParam, valueOf,
} from "./param_state.js";
import {clock, confirmAction, el, helpText, note, segmented, toast, withUnit} from "./ui.js";

export function formatValue(value, meta) {
  const choice = meta.choices?.find(item => item.value === value);
  if (choice) return choice.label;
  if (typeof value === "boolean") return value ? t("common.on") : t("common.off");
  return withUnit(value, meta.unit);
}

// 항목 하나의 편집기. card가 있으면 저장하는 동안 그 카드를 흐리게 하고 끝나면 잠깐 테를 칠한다. onSaved는 저장이
// 끝날 때마다(실패해도) 불린다. refresh()는 스냅샷 값으로 다시 그린다.
// 저장하는 동안 컨트롤을 막지 않는다: 그동안 고른 값은 마지막 것 하나만 기억했다가 이어서 저장한다(빨리 누른 −/+를
// 버리지 않고, 저장 요청과 다시 읽기 신호도 몰아서 한 번).
export function paramEditor(group, key, {card = null, onSaved = () => {}} = {}) {
  const meta = metaFor(group, key);
  const notify = params().groups[group].notify;
  const status = el("span.param-status");
  const modifiedBadge = el("span.badge.warn", {text: t("params.changed"), title: t("params.changed_title")});
  const resetButton = el("button.link-button", {type: "button", text: t("params.reset")});
  let control = null;
  let saving = false;
  let queued;

  // 저장하는 동안은 컨트롤의 값을 덮지 않는다(저장이 다른 항목의 안내를 다시 그리며 이것도 부른다): 기다리는 값이
  // 사라지면 그다음 −/+가 옛 값에서 센다. 마지막 저장이 끝나면 다시 그린다.
  const refresh = () => {
    if (!saving) control.show(valueOf(group, key));
    const modified = isModified(group, key);
    modifiedBadge.hidden = !modified;
    resetButton.hidden = !modified;
  };

  // 고른 값은 곧바로 컨트롤에 보인다: 컨트롤은 보이는 값과 같은 선택을 다시 저장하지 않으므로, 저장하는 동안 원래
  // 값으로 되돌린 것도(0.8 → +0.85 → −0.8) 기다리는 값으로 남는다.
  const save = async next => {
    control.show(next);
    if (saving) {
      queued = next;
      return;
    }
    saving = true;
    card?.classList.add("busy");
    status.className = "param-status";
    status.textContent = t("params.applying");
    try {
      const {notified} = await saveParam(group, key, next);
      const waiting = notify && !notified.length;
      status.className = `param-status ${waiting ? "warn" : "good"}`;
      status.textContent = waiting ? t("params.saved_not_running", {process: notify}) : t("params.applied", {time: clock()});
      if (card) {
        card.classList.add("saved");
        window.setTimeout(() => card.classList.remove("saved"), 900);
      }
    } catch (error) {
      queued = undefined;  // 받지 않았으면 이어서 고른 값도 버린다
      status.className = "param-status bad";
      status.textContent = t("params.apply_failed");
      toast(t("params.save_failed", {label: meta.label, error: error.message}), {failed: true});
    }
    saving = false;
    const following = queued;
    queued = undefined;
    if (following !== undefined && following !== valueOf(group, key)) {
      save(following);
      return;
    }
    card?.classList.remove("busy");
    refresh();
    onSaved();
  };

  // 기본값으로 되돌리기. 바꾸면 학습을 버리는 항목(caution)은 먼저 묻는다.
  const reset = () => {
    const fallback = defaultOf(group, key);
    if (meta.caution && !confirmAction([t("params.reset_confirm", {label: meta.label}),
                                        `${formatValue(valueOf(group, key), meta)} → ${formatValue(fallback, meta)}`, "",
                                        meta.caution])) return;
    save(fallback);
  };

  const value = valueOf(group, key);
  control = meta.choices ? segmented(meta.label, meta.choices, save)
    : typeof value === "boolean" ? switchControl(meta, save)
    : meta.control === "slider" ? sliderControl(meta, save) : stepperControl(meta, save);
  resetButton.addEventListener("click", reset);
  refresh();
  return {meta, control, status, modifiedBadge, resetButton, refresh, save};
}

// 접힌 "바꾸면": 값을 줄이거나 끄면, 키우거나 켜면 어떻게 되는지. 이름을 붙인 켜고 끄기는 그 이름으로(ESP12, locationd).
export function effectsDetails(meta, boolean) {
  const named = value => meta.choices?.find(item => item.value === value)?.label;
  const down = named(false) || (boolean ? t("params.when_off") : t("params.when_lower"));
  const up = named(true) || (boolean ? t("params.when_on") : t("params.when_higher"));
  return el("details.param-effects",
    {},
    el("summary", {text: t("params.effects")}),
    el("div.effects",
      {},
      el("div.effect.down", {}, el("b", {text: down}), meta.decrease),
      el("div.effect.up", {}, el("b", {text: up}), meta.increase)));
}

// 다른 항목 때문에 쓰이지 않거나 쓰임이 바뀐다는 안내. update()는 지금 값으로 다시 고르고(바뀐 것이 없으면 그대로
// 둔다), 쓰이지 않으면 카드에 ignored를 달아 흐리게 한 뒤 참을 돌려준다. 이유가 여럿이면 메타데이터 순서로 첫째(가장
// 직접적인 것)만 보이고, 쓰이지 않는 동안은 쓰임이 바뀐다는 안내를 숨긴다.
export function conditionNoteList(group, key, card) {
  const element = el("div.param-notes");
  let shown = null;
  const update = () => {
    const {unused, role} = conditionNotes(group, key);
    const notes = unused.length ? [["ignored", unused[0]]] : role.map(text => ["prior", text]);
    const signature = JSON.stringify(notes);
    if (signature !== shown) {
      shown = signature;
      card.classList.toggle("ignored", unused.length > 0);
      element.replaceChildren(...notes.map(([tone, text]) => note(text, tone)));
    }
    return unused.length > 0;
  };
  return {element, update};
}

// 항목 카드: 이름·키·기본값, 설명(세 줄, 더 보기), 주의, 다른 항목에 따른 안내, 컨트롤, 바꾸면, 되돌리기, 상태.
// 파라미터 탭의 검색이 쓰는 search와 isModified, 다시 그리는 refresh를 단다.
export function paramCard(group, key) {
  const card = el("article.card.param", {dataset: {key}});
  const editor = paramEditor(group, key, {card});
  const {meta} = editor;
  const fallback = defaultOf(group, key);
  const conditions = conditionNoteList(group, key, card);
  conditions.update();
  card.append(
    el("header.param-head",
      {},
      el("div.param-name",
        {},
        el("h3.card-title", {text: meta.label}),
        el("div.param-key",
          {},
          el("code", {text: key}),
          fallback === undefined ? null : el("span", {text: ` · ${t("params.default", {value: formatValue(fallback, meta)})}`}))),
      editor.modifiedBadge,
      meta.unit ? el("span.param-unit", {text: meta.unit}) : null),
    helpText(meta.description),
    meta.caution ? note(meta.caution, "warn") : "",
    conditions.element,
    editor.control.element,
    el("footer.param-foot", {}, effectsDetails(meta, typeof valueOf(group, key) === "boolean"), editor.resetButton,
      editor.status));
  card.search = `${meta.label} ${key} ${meta.description}`.toLowerCase();
  card.isModified = () => isModified(group, key);
  card.refresh = () => {
    editor.refresh();
    conditions.update();
  };
  // 다른 항목(다른 탭 것도)이 저장되면 값과 안내를 다시 그린다
  onParamsChanged(card.refresh);
  return card;
}

// ------------------------------------------------------------ 컨트롤: {element, show(value), setDisabled(bool)}
function switchControl(meta, save) {
  const text = el("span");
  const button = el("button.switch-control",
    {type: "button", role: "switch", "aria-label": meta.label, onclick: () => save(button.getAttribute("aria-checked") !== "true")},
    text,
    el("span.switch"));
  return {
    element: button,
    show(value) {
      button.setAttribute("aria-checked", String(value));
      text.textContent = value ? t("common.on") : t("common.off");
    },
    setDisabled(disabled) {
      button.disabled = disabled;
    },
  };
}

// − [값] +. 빈칸이나 숫자가 아닌 글자는 저장하지 않고(valueAsNumber가 NaN), 쳐 넣은 값이 범위 밖이면 범위 끝으로
// 맞추고 알린다. 음수가 되는 항목은 숫자 자판(iOS는 −가 없다)을 고르지 않는다.
function stepperControl(meta, save) {
  const input = el("input.stepper-value",
    {type: "number", inputMode: meta.min < 0 ? undefined : "decimal", step: String(meta.step), min: String(meta.min),
     max: String(meta.max), "aria-label": t("params.value_of", {label: meta.label})});
  let shown = null;
  const commit = (next, typed) => {
    if (!Number.isFinite(next)) {
      toast(t("params.enter_number"), {failed: true});
      input.value = String(shown);
      return;
    }
    const value = clampToMeta(next, meta);
    if (typed && (next < meta.min || next > meta.max)) toast(t("params.clamped", {min: meta.min, max: meta.max, value}));
    input.value = String(value);
    if (value !== shown) save(value);
  };
  // −/+는 칸에 보이는 값(저장을 기다리는 값일 수 있다)에서 센다. 칸이 비었으면 저장된 값에서.
  const current = () => (Number.isFinite(input.valueAsNumber) ? input.valueAsNumber : shown);
  const stepButton = (direction, label) => {
    const title = t(direction < 0 ? "params.step_down" : "params.step_up", {step: withUnit(meta.step, meta.unit)});
    return el("button.stepper-button",
      {type: "button", text: label, title, "aria-label": title, onclick: () => commit(current() + direction * meta.step, false)});
  };
  const minus = stepButton(-1, "−");
  const plus = stepButton(1, "+");
  input.addEventListener("keydown", event => {
    if (event.key === "Enter") input.blur();
  });
  input.addEventListener("change", () => commit(input.valueAsNumber, true));
  return {
    element: el("div.stepper",
      {},
      minus,
      el("label.stepper-field", {}, input, el("span.stepper-range", {text: withUnit(`${meta.min} ~ ${meta.max}`, meta.unit)})),
      plus),
    show(value) {
      shown = value;
      input.value = String(value);
    },
    setDisabled(disabled) {
      for (const control of [minus, input, plus]) control.disabled = disabled;
    },
  };
}

function sliderControl(meta, save) {
  const input = el("input.slider", {type: "range", min: String(meta.min), max: String(meta.max), step: String(meta.step),
                                   "aria-label": t("params.value_of", {label: meta.label})});
  const output = el("output.slider-value");
  let shown = null;
  input.addEventListener("input", () => {
    output.textContent = withUnit(input.value, meta.unit);
  });
  input.addEventListener("change", () => {
    const value = clampToMeta(Number(input.value), meta);
    if (value !== shown) save(value);
  });
  return {
    element: el("div.slider-control", {}, input, output),
    show(value) {
      shown = value;
      input.value = String(value);
      output.textContent = withUnit(value, meta.unit);
    },
    setDisabled(disabled) {
      input.disabled = disabled;
    },
  };
}
