// 이 브라우저에만 기억하는 작은 선택(화면 언어, BEV 레이어, 앞차 카드 접힘). 저장소가 없거나 거부해도(사생활 보호
// 창, 막은 사이트 데이터) 페이지는 기본값으로 돈다.
export function recall(key) {
  try {
    return window.localStorage.getItem(key);
  } catch {
    return null;
  }
}

// 기억했으면 true
export function remember(key, value) {
  try {
    window.localStorage.setItem(key, value);
    return true;
  } catch {
    return false;
  }
}
