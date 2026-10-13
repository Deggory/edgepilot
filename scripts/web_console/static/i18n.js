// 화면 글자의 언어. 고른 언어의 문구 묶음(i18n/<언어>.json)을 이 모듈을 읽을 때 한 번 받으므로(최상위 await),
// 이 모듈을 import한 곳에서는 모듈을 읽는 동안에도 t()를 바로 쓸 수 있다. 언어는 주소의 ?lang=, 이 브라우저에 기억한
// 선택, 브라우저 언어 순으로 정하고, 바꾸면 페이지를 다시 연다(화면이 문구를 만들 때 한 번 넣으므로).

// 언어 코드와 그 언어로 쓴 이름(언어 버튼의 설명). 어느 언어 화면에서도 같은 글자다.
export const LANGUAGES = {ko: "한국어", en: "English"};  // i18n-allow
const FALLBACK = "en";
const STORAGE_KEY = "edgepilot.lang";

function stored() {
  try {
    return window.localStorage.getItem(STORAGE_KEY);
  } catch {
    return null;
  }
}

function pick() {
  const candidates = [new URLSearchParams(window.location.search).get("lang"), stored(), ...(navigator.languages || [])];
  for (const candidate of candidates) {
    const code = String(candidate || "").toLowerCase().split("-")[0];
    if (code in LANGUAGES) return code;
  }
  return FALLBACK;
}

export const lang = pick();
// 숫자와 시각 표기의 지역
export const locale = lang === "ko" ? "ko-KR" : "en-GB";
document.documentElement.lang = lang;

const messages = await fetch(`/static/i18n/${lang}.json`)
  .then(response => (response.ok ? response.json() : {}))
  .catch(() => ({}));
const plurals = new Intl.PluralRules(locale);
const missing = new Set();

export function has(key) {
  return Object.hasOwn(messages, key);
}

// key의 문구에서 {이름}을 vars 값으로 바꾼다. 문구가 {one, other, ...}이면 vars.count로 복수형을 고른다. 없는 키는
// 키를 그대로 보인다(콘솔에 한 번 알린다).
export function t(key, vars = {}) {
  let text = messages[key];
  if (text === undefined) {
    if (!missing.has(key)) console.warn(`i18n: no "${key}" in ${lang}.json`);
    missing.add(key);
    return key;
  }
  if (typeof text === "object") text = text[plurals.select(Number(vars.count))] ?? text.other;
  return text.replace(/\{(\w+)\}/g, (match, name) => (vars[name] === undefined ? match : String(vars[name])));
}

// 언어를 바꾸고 페이지를 다시 연다. 기억할 수 없는 브라우저면 주소의 ?lang=으로 넘긴다.
export function setLanguage(next) {
  const url = new URL(window.location.href);
  try {
    window.localStorage.setItem(STORAGE_KEY, next);
    url.searchParams.delete("lang");
  } catch {
    url.searchParams.set("lang", next);
  }
  // 주소가 같으면(#탭만 있으면) replace는 다시 열지 않는다
  if (url.href === window.location.href) window.location.reload();
  else window.location.replace(url.href);
}
