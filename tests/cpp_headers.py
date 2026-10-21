"""시험이 C++ 헤더의 고정값을 읽는 도우미. ipc_messages.h 등의 static_assert(sizeof), EDGEPILOT_<구조체>_AT
오프셋과 constexpr 상수를 Python 쪽 배치와 대조할 때 쓴다(check_*.py가 같은 폴더에서 가져온다)."""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IPC_MESSAGES = ROOT / "src" / "common" / "ipc_messages.h"
IPC = IPC_MESSAGES.read_text(encoding="utf-8")


def pinned(pattern: str, text: str = IPC) -> int:
    """text에서 pattern의 첫 그룹(10진수). 없으면 AssertionError."""
    match = re.search(pattern, text)
    assert match, pattern
    return int(match.group(1))


def cpp_size(struct: str, text: str = IPC) -> int:
    """static_assert(sizeof(struct) == N)의 N."""
    return pinned(r"sizeof\(" + struct + r"\) == (\d+)", text)


def cpp_offsets(struct: str, text: str = IPC) -> dict[str, int]:
    """EDGEPILOT_<struct>_AT(필드, 위치) 고정값들. struct는 MODEL_STATE처럼 매크로 가운데 이름이다."""
    return {name: int(at) for name, at in re.findall(rf"EDGEPILOT_{struct}_AT\((\w+), (\d+)\);", text)}


def cpp_constant(name: str, text: str = IPC) -> int:
    """constexpr <타입> name = 값;의 값(10진수나 0x)."""
    match = re.search(rf"constexpr \w+ {name} = (0x[0-9a-fA-F]+|\d+);", text)
    assert match, name
    return int(match.group(1), 0)
