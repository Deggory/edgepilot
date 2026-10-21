// 조향 탭의 학습 토크 모델 카드(docs/lateral-nn.md). 모델은 한 차종의 주행으로 학습한 것이라 그 차의 것을 고른다.
// models/lateral_nn/의 파일 이름, 고른 것, controlsd가 실제로 쓰는 것을 2초마다 읽는다. 다른 파일을 누르면 고르기
// 파일을 바꾸고, controlsd가 재시작 없이 1초 안에 읽어 바꾼다. 학습 피드포워드 스위치(nn_feedforward)가 꺼져 있으면
// 모델은 읽혀도 쓰이지 않는다.
import {codeText, t} from "./i18n.js";
import {onParamsChanged, updateRuntime, valueOf} from "./param_state.js";
import {api, badges, el, helpText, note, pollingCard, runAction, section, syncRows} from "./ui.js";

const fileName = name => (name ? `${name}.json` : "–");

export function createLateralNnCard() {
  const list = el("div.nn-models");
  const about = helpText(t("lateral_nn.about"), "card-note");  // 한 번 만들어 펼친 상태가 폴링에 접히지 않게
  const rows = new Map();  // 이름 -> {element(버튼), tags}(ui.js syncRows)
  let latest = null;
  let busy = false;

  const choose = async name => {
    if (busy || !latest || name === latest.selected) return;
    const using = latest.controlsd?.loaded;
    const result = await runAction({
      confirm: [t("lateral_nn.confirm", {model: fileName(name)}),
                using ? t("lateral_nn.confirm_now", {model: fileName(using)}) : null, "", t("lateral_nn.confirm_car")],
      busy: on => {
        busy = on;
        rows.forEach(row => {
          row.element.disabled = on;
        });
      },
      call: () => api.post("/api/lateral_nn", {model: name}),
      done: () => t("lateral_nn.selected", {model: fileName(name)}),
      failed: error => t("lateral_nn.select_failed", {error}),
    });
    if (result) card.show(result);
  };

  const makeRow = name => {
    const tags = el("span.badge-row");
    return {tags, element: el("button.nn-model", {type: "button", onclick: () => choose(name)},
                              el("code.nn-model-name", {text: fileName(name)}), tags)};
  };

  const render = s => {
    latest = s;
    updateRuntime(s.runtime);
    const status = s.controlsd;  // controlsd가 돌지 않으면 null
    syncRows(list, rows, s.models, makeRow);
    for (const [name, row] of rows) {
      const loaded = status?.loaded === name;
      const selected = s.selected === name;
      row.element.setAttribute("aria-pressed", String(selected));
      row.element.disabled = busy;
      row.tags.replaceChildren(...badges([
        loaded ? [t("common.in_use"), "good"] : null,
        !selected || loaded ? null
          : !status ? [t("lateral_nn.chosen"), "accent"]
          : status.selected === name && status.error ? [t("lateral_nn.refused_badge"), "bad"]
          : [t("lateral_nn.switching"), "accent"],
        s.default === name ? [t("lateral_nn.default"), "muted"] : null,
      ]));
    }
    // controlsd의 거부 이유는 "<이름>: <이유>"다. 이름은 앞에서 따로 쓴다.
    const reason = status?.error ? status.error.replace(`${status.selected}: `, "") : "";
    return {
      badges: [!status ? [t("lateral_nn.no_status"), "bad"]
        : status.loaded ? [t("lateral_nn.using", {model: fileName(status.loaded)}), "good"]
        : [t("lateral_nn.none_loaded"), "warn"]],
      children: [
        s.models.length ? list : note(t("lateral_nn.no_models")),
        s.selection_error ? note(codeText("lateral_nn.selection_error", s.selection_error), "warn") : null,
        reason ? note(t("lateral_nn.refused", {model: fileName(status.selected), error: reason,
                                               kept: status.loaded ? fileName(status.loaded) : t("lateral_nn.none")}), "warn")
          : null,
        valueOf("steering", "nn_feedforward") === false ? note(t("lateral_nn.switch_off"), "ignored") : null,
        about,
      ],
    };
  };
  const card = pollingCard(t("lateral_nn.card"), {url: "/api/lateral_nn", intervalMs: 2000, render});

  onParamsChanged(() => card.refresh());
  return {element: section(t("lateral_nn.section"), [card.element]), after: "torque_controller",
          start: card.start, stop: card.stop};
}
