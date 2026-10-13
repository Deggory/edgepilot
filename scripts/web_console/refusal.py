"""요청을 받지 않을 때의 이유. 서버는 언어를 모르고 문구 키만 내며, 페이지가 고른 언어의 문구(static/i18n)로 바꾼다."""
from __future__ import annotations

from typing import Any, Dict


class Refusal(PermissionError):
    """code는 문구 키(예: "error.calibration_engaged"), values는 그 문구의 자리 값이다. 서버는 409와
    {"code", 자리 값...}으로 답한다."""

    def __init__(self, code: str, **values: Any):
        super().__init__(code)
        self.code = code
        self.values = values

    def detail(self) -> Dict[str, Any]:
        return {"code": self.code, **self.values}
