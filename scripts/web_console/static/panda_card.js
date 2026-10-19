// 기기 설정 탭의 Panda 펌웨어 카드. pandad가 쓰는 상태를 1초마다 읽고, 주차 중일 때만 설치된 이미지로 플래싱을
// 요청한다(쓰는 것은 pandad이고, 그쪽도 같은 조건을 다시 본다). 서버는 사유·단계·이미지 문제를 코드로 낸다.
import {codeText, locale, t} from "./i18n.js";
import {actionButton, api, DASH, el, helpText, note, pollingCard, runAction, section} from "./ui.js";

const imageError = image => codeText("panda.image", image.error, {max_bytes: image.max_bytes});
const bytes = size => t("panda.bytes", {size: size.toLocaleString(locale)});

// conditions: 플래싱 조건의 긴 안내(한 번 만든 helpText, 펼친 상태가 1초마다 접히지 않게)
function describe(p, conditions) {
  const flash = p.flash;
  const flashing = flash.state === "flashing" || p.request_pending;
  const badgeList = [
    p.stale ? [t("panda.no_pandad"), "bad"] : p.mode === "app" ? [t("panda.connected"), "good"]
      : p.mode === "bootstub" ? ["bootstub", "warn"] : [t("panda.no_panda"), "bad"],
    p.mode === "app" && p.image.valid ? (p.up_to_date ? [t("panda.up_to_date"), "good"] : [t("panda.other_version"), "warn"]) : null,
    flashing ? [t("panda.writing", {percent: flash.percent || 0}), "accent"]
      : flash.state === "done" ? [t("panda.flashed"), "good"]
      : flash.state === "failed" ? [t("panda.last_failed"), "bad"] : null,
  ];
  const current = p.mode === "app" ? (p.firmware_version || DASH) : p.mode === "bootstub" ? t("panda.bootstub_only") : DASH;
  const step = flash.step ? codeText("panda.step", flash.step) : t("panda.requesting");
  const children = [
    el("dl.facts",
      {},
      el("dt", {text: t("panda.panda")}), el("dd", {}, el("code", {text: current}), p.hw_name ? el("small", {text: p.hw_name}) : null),
      el("dt", {text: t("panda.installed_image")}),
      el("dd", {}, el("code", {text: p.image.present ? (p.image.version || DASH) : t("panda.none")}),
         p.image.present ? el("small", {text: bytes(p.image.size)}) : null)),
    p.image.present && !p.image.valid ? note(imageError(p.image), "warn") : null,
    p.mode === "app" && /-RELEASE$/.test(p.firmware_version || "") ? note(t("panda.release_warning"), "warn") : null,
    flashing ? note(t("panda.flashing_note", {step}), "prior")
      : flash.state === "failed"
        ? note(t("panda.last_attempt", {reason: codeText("panda.reason", flash.error || "")}) + (flash.detail ? ` · ${flash.detail}` : ""), "warn")
      : flash.state === "done" ? note(t("panda.last_flash", {version: flash.version})) : null,
    conditions,
  ];
  const reason = p.stale ? t("panda.reason_no_pandad")
    : flashing ? t("panda.reason_flashing")
    : !p.image.valid ? (p.image.present ? t("panda.reason_image_unusable") : t("panda.reason_no_image"))
    : p.mode === "none" ? codeText("panda.reason", "no_panda")
    : p.mode === "app" && !p.hw_supported ? codeText("panda.reason", "hw_unsupported")
    : p.blocker ? codeText("panda.reason", p.blocker) : "";
  const title = p.mode === "bootstub" ? t("panda.rewrite") : p.up_to_date ? t("panda.rewrite_same") : t("panda.flash");
  return {badgeList, children, reason, title};
}

export function createPandaCard() {
  let latest = null;
  const button = actionButton({icon: "⤓", danger: true, onclick: () => flash()});
  button.setContent(t("panda.flash"));  // 첫 상태가 오기 전(막혀 있다)
  const conditions = helpText(t("panda.conditions"), "card-note");
  const card = pollingCard(t("panda.card"), {
    url: "/api/panda", intervalMs: 1000, actions: [button],
    render: p => {
      latest = p;  // 누를 때 볼 상태(읽지 못하면 버튼이 막힌다: liveCard.fail)
      const {badgeList, children, reason, title} = describe(p, conditions);
      button.setContent(title, reason || t("panda.image_to_write", {version: p.image.version}));
      button.disabled = Boolean(reason);
      return {badges: badgeList, children};
    },
  });

  const flash = async () => {
    const p = latest;
    if (!p || !p.image.valid) return;
    const result = await runAction({
      confirm: [t("panda.confirm"),
                t("panda.confirm_now", {current: p.mode === "app" ? p.firmware_version : p.mode}),
                t("panda.confirm_image", {version: p.image.version, size: bytes(p.image.size)}), "",
                t("panda.confirm_effect"), t("panda.confirm_parked")],
      busy: on => {
        button.disabled = on;
      },
      call: () => api.post("/api/panda/flash", {version: p.image.version}),
      done: () => t("panda.requested"),
      failed: error => t("panda.request_failed", {error}),
    });
    if (result) card.show(result);
  };
  return {element: section(t("panda.section"), [card.element]), start: card.start, stop: card.stop};
}
