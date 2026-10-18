// BEV 탭: 모델이 본 길과 앞차를 3D로(bev.js). 보드는 모델·제어 상태를 바이트 그대로 흘려보내기만 하고 해석과
// 그리기는 이 브라우저가 한다. three.js는 이 탭을 처음 열 때 불러오고, 패널과 WebGL은 한 번만 만들어 탭을
// 오가도 다시 쓴다. 스트림은 start()부터 stop()까지만 연다(탭이 보이고 페이지가 가려지지 않은 동안).
import {t} from "./i18n.js";
import {el, helpText} from "./ui.js";

export function createBevView() {
  const statusText = el("span", {text: t("app.loading")});
  const status = el("p.stream-status", {dataset: {state: "wait"}}, el("span.dot"), statusText);
  const body = el("div.bev-body");
  let bev = null;
  let loading = null;
  let shown = false;

  // state: good(모델과 controlsd), warn(모델만), bad(끊김, 모델 없음·멈춤), wait(연결 중)
  const showStatus = ({state, text}) => {
    status.dataset.state = state;
    statusText.textContent = text;
  };
  const follow = () => {
    if (!bev) return;
    if (shown) bev.start();
    else bev.stop();
  };

  const load = async () => {
    try {
      loading = loading || import("./bev.js").then(module => module.createBev(showStatus));
      bev = await loading;
      body.replaceChildren(bev.element);
      follow();
    } catch (error) {
      // 브라우저는 실패한 모듈을 기억해 다시 import해도 다시 받지 않는다: 페이지를 새로 고쳐야 한다
      showStatus({state: "bad", text: t("bev.load_failed")});
      body.replaceChildren(el("div.empty",
        {},
        el("p", {text: t("bev.load_failed_detail", {error: error.message})}),
        el("button.button", {type: "button", text: t("bev.reload_page"), onclick: () => window.location.reload()})));
    }
  };

  return {
    element: el("div.view", {}, helpText(t("bev.note"), "group-note"), status, body),
    start() {
      shown = true;
      if (bev) follow();
      else load();
    },
    stop() {
      shown = false;
      follow();
    },
  };
}
