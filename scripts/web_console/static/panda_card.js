// 기기 설정 탭의 Panda 펌웨어 카드. pandad가 쓰는 상태를 1초마다 읽고, 주차 중일 때만 설치된 이미지로 플래싱을
// 요청한다(쓰는 것은 pandad이고, 그쪽도 같은 조건을 다시 본다). 서버는 사유·단계·이미지 문제를 코드로 낸다.
import {has, locale, t} from "./i18n.js";
import {actionButton, api, confirmAction, el, liveCard, note, poller, section, toast} from "./ui.js";

// 코드의 문구, 모르는 코드면 코드 그대로
const coded = (prefix, code, vars = {}) => (has(`${prefix}.${code}`) ? t(`${prefix}.${code}`, vars) : code);
const imageError = image => coded("panda.image", image.error, {max_bytes: image.max_bytes});
const bytes = size => t("panda.bytes", {size: size.toLocaleString(locale)});

function describe(p) {
  const flash = p.flash || {};
  const flashing = flash.state === "flashing" || p.request_pending;
  const badgeList = [
    !p.available ? [t("panda.no_pandad"), "bad"] : p.mode === "app" ? [t("panda.connected"), "good"]
      : p.mode === "bootstub" ? ["bootstub", "warn"] : [t("panda.no_panda"), "bad"],
    p.mode === "app" && p.image.valid ? (p.up_to_date ? [t("panda.up_to_date"), "accent"] : [t("panda.other_version"), "warn"]) : null,
    flashing ? [t("panda.writing", {percent: flash.percent || 0}), "accent"]
      : flash.state === "done" ? [t("panda.flashed"), "good"]
      : flash.state === "failed" ? [t("panda.last_failed"), "bad"] : null,
  ];
  const current = p.mode === "app" ? (p.firmware_version || "–") : p.mode === "bootstub" ? t("panda.bootstub_only") : "–";
  const step = flash.step ? coded("panda.step", flash.step) : t("panda.requesting");
  const children = [
    el("dl.facts",
      {},
      el("dt", {text: t("panda.panda")}), el("dd", {}, el("code", {text: current}), p.hw_name ? el("small", {text: p.hw_name}) : null),
      el("dt", {text: t("panda.installed_image")}),
      el("dd", {}, el("code", {text: p.image.present ? (p.image.version || "–") : t("panda.none")}),
         p.image.present ? el("small", {text: bytes(p.image.size)}) : null)),
    p.image.present && !p.image.valid ? note(imageError(p.image), "warn") : null,
    p.mode === "app" && /-RELEASE$/.test(p.firmware_version || "") ? note(t("panda.release_warning"), "warn") : null,
    flashing ? note(t("panda.flashing_note", {step}), "prior")
      : flash.state === "failed"
        ? note(t("panda.last_attempt", {reason: coded("panda.reason", flash.error || "")}) + (flash.detail ? ` · ${flash.detail}` : ""), "warn")
      : flash.state === "done" ? note(t("panda.last_flash", {version: flash.version})) : null,
    note(t("panda.conditions")),
  ];
  const reason = !p.available ? t("panda.reason_no_pandad")
    : flashing ? t("panda.reason_flashing")
    : !p.image.valid ? (p.image.present ? t("panda.reason_image_unusable") : t("panda.reason_no_image"))
    : p.mode === "none" ? coded("panda.reason", "no_panda")
    : p.mode === "app" && !p.hw_supported ? coded("panda.reason", "hw_unsupported")
    : p.blocker ? coded("panda.reason", p.blocker) : "";
  const title = p.mode === "bootstub" ? t("panda.rewrite") : p.up_to_date ? t("panda.rewrite_same") : t("panda.flash");
  return {badgeList, children, reason, title};
}

export function createPandaCard() {
  let latest = null;
  const button = actionButton({icon: "⤓", danger: true, onclick: () => flash()});
  const card = liveCard(t("panda.card"), {actions: [button]});

  const show = p => {
    latest = p;
    const {badgeList, children, reason, title} = describe(p);
    card.update(badgeList, children);
    button.setContent(title, reason || t("panda.image_to_write", {version: p.image.version}));
    button.disabled = Boolean(reason);
  };

  const flash = async () => {
    const p = latest;
    if (!p || !p.image.valid) return;
    if (!confirmAction([t("panda.confirm"),
                        t("panda.confirm_now", {current: p.mode === "app" ? p.firmware_version : p.mode}),
                        t("panda.confirm_image", {version: p.image.version, size: bytes(p.image.size)}), "",
                        t("panda.confirm_effect"),
                        t("panda.confirm_parked")])) return;
    button.disabled = true;
    try {
      show(await api.post("/api/panda/flash", {version: p.image.version}));
      toast(t("panda.requested"));
    } catch (error) {
      toast(t("panda.request_failed", {error: error.message}), {failed: true});
      button.disabled = false;
    }
  };

  const polling = poller(async isCurrent => {
    try {
      const p = await api.get("/api/panda");
      if (isCurrent()) show(p);
    } catch (error) {
      if (isCurrent()) card.update([[t("panda.read_failed"), "bad"]], [note(error.message, "warn")]);
    }
  }, 1000);
  return {element: section(t("panda.section"), [card.element]), start: polling.start, stop: polling.stop};
}
