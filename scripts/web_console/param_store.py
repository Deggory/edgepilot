"""파라미터 파일(params/*.json) 읽기·쓰기. 바꾸면 그 그룹을 읽는 프로세스에 SIGHUP으로 알리고(controlsd), 기기
설정의 백라이트 항목은 하드웨어에도 바로 적용한다. 기본값은 params/ 옆의 params.defaults/(업로드 스크립트가
저장소의 params/에서 채운다)이고, 런타임 파일의 키 집합을 기본값에 맞춰 둔다.

값은 런타임 로더가 받는 꼴만 쓴다(check_value). 로더는 켜고 끄는 값이 true/false가 아니거나 숫자가 유한하지 않으면
파일 전체를 거부하고, 시작할 때 거부하면 controlsd가 뜨지 못한다(인증 없는 포트라 페이지의 검사만 믿지 않는다).
받지 않는 요청은 문구 키로 알린다(Refusal)."""
from __future__ import annotations

import math
import threading
from pathlib import Path
from typing import Any, Callable, Dict, List

from .backlight import DisplayBacklight
from .files import defaults_dir, params_dir, read_json_object, write_json_atomic
from .param_metadata import PARAM_GROUPS, PARAM_METADATA, VEHICLE_TAB
from .process_status import signal_process
from .refusal import Refusal

# 백라이트 하드웨어에 적용하는 기기 설정 항목. 나머지(알림음 등)는 overlayd·modeld가 파일에서 읽는다.
BACKLIGHT_KEYS = ("enabled", "brightness_percent")

# 런타임 로더는 숫자를 strtof로 읽고 범위 오류(ERANGE)면 파일을 거부한다: float로 나타낼 수 없는 큰 수와 정규화되지
# 않는 작은 수(0이 아닌데 FLT_MIN보다 작은 것)
FLOAT_MAX = 3.4028234663852886e38
FLOAT_MIN_NORMAL = 1.1754943508222875e-38


def loader_float(value: Any) -> float | None:
    """런타임 로더(utils_json.cc parse_json_float_value)가 받는 숫자면 그 float, 아니면 None."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except OverflowError:  # 아주 큰 정수
        return None
    if not math.isfinite(number) or abs(number) > FLOAT_MAX or 0 < abs(number) < FLOAT_MIN_NORMAL:
        return None
    return number


def check_value(group: str, key: str, value: Any, current: Any) -> Any:
    """값 하나를 런타임 로더가 받는 꼴인지 보고 저장할 값을 돌려준다. 받지 않으면 Refusal(400,
    error.param_invalid.<이유>). 켜고 끄는 항목(지금 값이 true/false)은 true/false만, 나머지는 로더가 float로 읽을 수
    있는 숫자만 받고(loader_float), 메타데이터가 있으면 그 범위 안이어야 한다. step이 정수인 항목은 정수로 쓴다."""
    meta = PARAM_METADATA.get(group, {}).get(key, {})
    if isinstance(current, bool):
        if not isinstance(value, bool):
            raise Refusal("error.param_invalid.boolean", status=400, name=key)
        return value
    if loader_float(value) is None:
        raise Refusal("error.param_invalid.number", status=400, name=key)
    if "step" in meta and float(meta["step"]).is_integer():
        if not float(value).is_integer():
            raise Refusal("error.param_invalid.integer", status=400, name=key)
        value = int(value)
    if "min" in meta and not meta["min"] <= value <= meta["max"]:
        raise Refusal("error.param_invalid.range", status=400, name=key, min=meta["min"], max=meta["max"])
    return value


def group_paths(directory: Path) -> Dict[str, Path]:
    return {group: directory / spec["file"] for group, spec in PARAM_GROUPS.items()}


def read_document(path: Path) -> Dict[str, Any]:
    """파라미터 파일 하나. 없거나 깨졌거나 객체가 아니면 ValueError."""
    try:
        return read_json_object(path)
    except FileNotFoundError as exc:
        raise ValueError(f"parameter file not found: {path}") from exc
    except ValueError as exc:
        raise ValueError(f"invalid parameter file {path}: {exc}") from exc


class ParamStore:
    def __init__(self, paths: Dict[str, Path] | None = None, default_paths: Dict[str, Path] | None = None,
                 notify: Callable[[str], List[int]] = signal_process, backlight: DisplayBacklight | None = None):
        directory = params_dir()
        self.paths = paths if paths is not None else group_paths(directory)
        if default_paths is None:
            default_paths = group_paths(defaults_dir(directory)) if paths is None else {}
        self.default_paths = default_paths
        self.notify = notify
        self.backlight = backlight
        self.lock = threading.Lock()
        self._sync_with_defaults()
        if self.backlight is not None and "display" in self.paths:
            try:
                self.backlight.apply(self.read_group("display"))
            except (Refusal, RuntimeError, ValueError):
                pass

    def _sync_with_defaults(self) -> None:
        """런타임 파일의 키 집합을 기본값에 맞춘다. 빠진 키는 기본값으로 채우고, 기본값에서 사라진 키는 지운다.
        살아남은 키의 값은 그대로 둔다. 지우지 않으면 코드가 더 이상 읽지 않는 파라미터가 편집기에 남아 튜닝이
        적용된 것처럼 보인다."""
        for group, defaults in self.defaults().items():
            runtime_path = self.paths.get(group)
            if runtime_path is None:
                continue
            try:
                runtime = read_document(runtime_path) if runtime_path.is_file() else {}
            except ValueError:
                continue
            merged = {key: runtime.get(key, value) for key, value in defaults.items()}
            if merged != runtime:
                write_json_atomic(runtime_path, merged)

    def defaults(self) -> Dict[str, Dict[str, Any]]:
        """그룹별 기본값. 기본값 파일이 없거나 깨진 그룹은 뺀다."""
        result = {}
        for group, path in self.default_paths.items():
            try:
                result[group] = read_document(path)
            except ValueError:
                continue
        return result

    def read_group(self, group: str) -> Dict[str, Any]:
        """그룹 파일. 없는 그룹이면 Refusal(404), 파일이 없거나 깨졌으면 Refusal(500, error.param_file)."""
        path = self._path(group)
        try:
            return read_document(path)
        except ValueError as exc:
            raise Refusal("error.param_file", status=500, error=str(exc)) from exc

    def snapshot(self) -> Dict[str, Any]:
        """페이지가 파라미터 탭을 그리는 데 필요한 것 전부: 그룹 구성(섹션 순서, 알릴 프로세스), 차량 특성 탭 구성,
        현재값, 기본값, 항목 메타데이터, 파일 경로. 글자는 페이지가 고른 언어의 문구로 채운다(param_metadata)."""
        with self.lock:
            documents = {group: self.read_group(group) for group in self.paths}
        return {
            "groups": {group: {key: spec[key] for key in ("sections", "notify")}
                       for group, spec in PARAM_GROUPS.items() if group in self.paths},
            "vehicle_tab": VEHICLE_TAB,
            "params": documents,
            "defaults": self.defaults(),
            "metadata": PARAM_METADATA,
            "paths": {group: str(path) for group, path in self.paths.items()},
        }

    def update(self, group: str, values: Dict[str, Any], *, allow_managed: bool = False) -> Dict[str, Any]:
        """값을 바꾼다. 전용 카드가 바꾸는 항목(메타데이터 card, 예: 보정 출처)은 그 카드의 API만
        allow_managed=True로 바꾼다. 그 API가 받을 조건(예: 해제 상태)을 일반 경로로 건너뛰지 않게 한다."""
        if not values:
            raise Refusal("error.param_no_values", status=400)
        managed = sorted(key for key in values if PARAM_METADATA.get(group, {}).get(key, {}).get("card"))
        if managed and not allow_managed:
            raise Refusal("error.param_managed", status=400, name=", ".join(managed))
        with self.lock:
            document = self.read_group(group)
            unknown = sorted(set(values) - set(document))
            if unknown:
                raise Refusal("error.param_unknown", status=404, name=", ".join(unknown))
            document.update({key: check_value(group, key, value, document[key]) for key, value in values.items()})
            if group == "display" and any(key in values for key in BACKLIGHT_KEYS):
                if self.backlight is None:
                    raise Refusal("error.backlight_unavailable", status=503)
                try:
                    self.backlight.apply(document)
                except (RuntimeError, ValueError) as exc:
                    raise Refusal("error.backlight_failed", status=503, error=str(exc)) from exc
            write_json_atomic(self._path(group), document)
        process = PARAM_GROUPS.get(group, {}).get("notify", "")
        return {"group": group, "params": document, "notified": self.notify(process) if process else []}

    def _path(self, group: str) -> Path:
        if group not in self.paths:
            raise Refusal("error.param_unknown", status=404, name=group)
        return self.paths[group]
