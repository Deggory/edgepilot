"""웹 콘솔 모듈이 같이 쓰는 경로와 파일 읽기·쓰기. 다른 콘솔 모듈을 import하지 않아 어느 모듈이나 쓸 수 있다.

경로는 C++ 프로세스들처럼 작업 디렉터리 기준이다(매니저가 설치 디렉터리 /root/edgepilot로 옮겨 띄운다). 환경 변수는
비어 있으면 없는 것으로 본다(src/common/utils_process.h env_string과 같음)."""
from __future__ import annotations

import json
import os
import stat
import tempfile
from pathlib import Path
from typing import Any, Dict


def env_path(name: str, default: str) -> Path:
    """환경 변수 name의 경로. 없거나 비어 있으면 default."""
    return Path(os.environ.get(name) or default)


def params_dir() -> Path:
    """런타임 파라미터 디렉터리. 프로세스들과 같이 EDGEPILOT_PARAMS_DIR, 없으면 params/."""
    return env_path("EDGEPILOT_PARAMS_DIR", "params")


def defaults_dir(directory: Path | None = None) -> Path:
    """기본값 디렉터리: 파라미터 디렉터리 이름에 .defaults를 붙인 옆 디렉터리(params.defaults/, 업로드 스크립트가
    저장소의 params/에서 채운다)."""
    directory = directory if directory is not None else params_dir()
    return directory.with_name(directory.name + ".defaults")


def read_json_object(path: Path) -> Dict[str, Any]:
    """JSON 객체 하나인 파일. 없으면 FileNotFoundError, 깨졌거나 객체가 아니면 ValueError."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"not a JSON object: {path}")
    return data


def read_json_or_none(path: Path) -> Dict[str, Any] | None:
    """다른 프로세스가 쓰는 상태 파일처럼 없을 수 있는 JSON 객체. 읽지 못하면 None."""
    try:
        return read_json_object(path)
    except (OSError, ValueError):
        return None


def write_text_atomic(path: Path, text: str) -> None:
    """임시 파일에 쓰고 fsync한 뒤 rename으로 바꿔 넣는다(읽는 쪽이 반쯤 쓴 파일을 보지 않게). 있던 파일의 권한은
    그대로 둔다."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o664
    fd, temporary_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        os.fchmod(fd, mode)
        with os.fdopen(fd, "w", encoding="utf-8") as file:
            file.write(text)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary.exists():
            temporary.unlink()


def write_json_atomic(path: Path, document: Dict[str, Any]) -> None:
    """JSON 객체를 사람이 읽을 수 있게(들여쓰기 2) write_text_atomic으로 쓴다."""
    write_text_atomic(path, json.dumps(document, ensure_ascii=False, indent=2) + "\n")
