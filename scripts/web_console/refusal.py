"""요청을 받지 않을 때의 이유. 서버는 언어를 모르고 문구 키만 내며, 페이지가 고른 언어의 문구(static/i18n)로 바꾼다."""
from __future__ import annotations

from typing import Any, Dict


class Refusal(Exception):
    """code는 문구 키(예: "error.calibration_reset_engaged"), values는 그 문구의 자리 값이다. status는 HTTP 상태다:
    409 지금 상태로는 받지 않음(기본), 400 잘못된 값, 404 없는 것, 500 파일을 읽지 못함, 503 하드웨어를 쓸 수 없음.
    서버(console_server.py)는 그 상태와 {"detail": {"code", 자리 값...}}로 답한다."""

    def __init__(self, code: str, status: int = 409, **values: Any):
        super().__init__(code)
        self.code = code
        self.status = status
        self.values = values

    def detail(self) -> Dict[str, Any]:
        return {"code": self.code, **self.values}
