// 조향 탭의 학습 토크 모델 카드(docs/lateral-nn.md). 모델은 한 차종의 주행으로 학습한 것이라 그 차의 것을 고른다.
// models/lateral_nn/의 파일 이름, 고른 것, controlsd가 실제로 쓰는 것을 2초마다 읽는다. 다른 파일을 누르면 고르기
// 파일을 바꾸고, controlsd가 재시작 없이 1초 안에 읽어 바꾼다. 학습 피드포워드 스위치(nn_feedforward)가 꺼져 있으면
// 모델은 읽혀도 쓰이지 않는다.
import {has, t} from "./i18n.js";
import {onParamsChanged, valueOf} from "./param_state.js";
import {api, confirmAction, el, liveCard, note, poller, section, toast} from "./ui.js";

const POLL_MS = 2000;
const fileName = name => `${name}.json`;
const coded = (prefix, code) => (has(`${prefix}.${code}`) ? t(`${prefix}.${code}`) : code);

export function createLateralNnCard() {
  const card = liveCard(t("lateral_nn.card"));
  const list = el("div.nn-models");
  let latest = null;
  let busy = false;

  const choose = async name => {
    if (busy || !latest || name === latest.selected) return;
    const using = latest.controlsd?.loaded;
    if (!confirmAction([t("lateral_nn.confirm", {model: fileName(name)}),
                        using ? t("lateral_nn.confirm_now", {model: fileName(using)}) : null, "",
                        t("lateral_nn.confirm_car")])) return;
    busy = true;
    try {
      show(await api.post("/api/lateral_nn", {model: name}));
      toast(t("lateral_nn.selected", {model: fileName(name)}));
    } catch (error) {
      toast(t("lateral_nn.select_failed", {error: error.message}), {failed: true});
    } finally {
      busy = false;
    }
  };

  const row = (name, s) => {
    const loaded = s.controlsd?.loaded === name;
    const refused = s.controlsd?.selected === name && s.controlsd?.error && !loaded;
    const selected = s.selected === name;
    const badgeList = [
      loaded ? [t("lateral_nn.in_use"), "good"] : null,
      selected && !loaded ? (refused ? [t("lateral_nn.refused_badge"), "bad"] : [t("lateral_nn.switching"), "accent"])
        : null,
      s.default === name ? [t("lateral_nn.default"), "muted"] : null,
    ].filter(Boolean);
    return el("button.nn-model",
      {type: "button", "aria-pressed": String(selected), disabled: busy, onclick: () => choose(name)},
      el("code.nn-model-name", {text: fileName(name)}),
      badgeList.map(([text, tone]) => el(`span.badge.${tone}`, {text})));
  };

  const show = s => {
    latest = s;
    const status = s.controlsd;
    const badgeList = [
      !status ? [t("lateral_nn.no_status"), "muted"]
        : status.loaded ? [t("lateral_nn.using", {model: fileName(status.loaded)}), "good"]
        : [t("lateral_nn.none_loaded"), "warn"],
    ];
    list.replaceChildren(...s.models.map(name => row(name, s)));
    card.update(badgeList, [
      s.models.length ? list : note(t("lateral_nn.no_models")),
      s.selection_error ? note(coded("lateral_nn.selection_error", s.selection_error), "warn") : null,
      status?.error ? note(t("lateral_nn.refused", {model: status.selected || "–", error: status.error,
                                                     kept: status.loaded || t("lateral_nn.none")}), "warn") : null,
      valueOf("steering", "nn_feedforward") === false ? note(t("lateral_nn.switch_off"), "ignored") : null,
      note(t("lateral_nn.about")),
    ]);
  };

  onParamsChanged(() => latest && show(latest));
  const polling = poller(async isCurrent => {
    try {
      const s = await api.get("/api/lateral_nn");
      if (isCurrent()) show(s);
    } catch (error) {
      if (isCurrent()) card.update([[t("lateral_nn.read_failed"), "bad"]], [note(error.message, "warn")]);
    }
  }, POLL_MS);
  return {element: section(t("lateral_nn.section"), [card.element], {grid: false}), after: "torque_controller",
          start: polling.start, stop: polling.stop};
}
