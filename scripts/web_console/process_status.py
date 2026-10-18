"""런타임 프로세스 상태: 매니저가 1초마다 내는 ManagerState(프로세스 이름과 실행 여부, src/common/ipc_messages.h),
실행 파일에 새긴 빌드 버전(src/common/build_version.h), 파라미터를 바꿨을 때 다시 읽으라고 신호를 보낼 프로세스 찾기."""
from __future__ import annotations

import os
import re
import signal
import struct
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .shm_channel import IpcReader, freshness, topic_path

MANAGER_STATE_TOPIC = "/edgepilot_manager_state"
MANAGER_STATE_HEAD = struct.Struct("<QII")  # timestamp_ns, process_count, reserved
PROCESS_STATE = struct.Struct("<16sI")      # ProcessState 필드: name, running
MAX_PROCESSES = 12
MANAGER_MAX_AGE_S = 3.0  # 매니저는 1초마다 낸다
VERSION_MARKER = b"EDGEPILOT_VERSION="  # build_version.h EDGEPILOT_VERSION_MARKER_PREFIX
VERSION_MAX = 64                        # cmake/build_version.cmake가 자르는 길이
VERSION_SCAN_LIMIT = 64 << 20           # 실행 파일은 1 MB도 안 된다. 엉뚱한 큰 파일을 다 읽지 않는다
# 매니저 표의 이름과 실행 파일 이름이 다른 칸: 리허설에서는 replayd가 camerad 자리에서 돈다.
PROCESS_BINARIES = {"camerad": ("camerad", "replayd")}


# 표식 뒤에 cmake/build_version.cmake가 남기는 글자만 64자까지 오고 NUL로 끝나야 버전이다(같은 글자가 다른
# 문자열 안에 있어도 고르지 않는다).
_VERSION_PATTERN = re.compile(re.escape(VERSION_MARKER) + rb"([A-Za-z0-9._+-]{1,%d})\x00" % VERSION_MAX)


def read_binary_version(path) -> Optional[str]:
    """실행 파일에 새긴 빌드 버전(cmake/build_version.cmake). 표식이 없거나 읽지 못하면 None."""
    try:
        with open(path, "rb") as f:
            data = f.read(VERSION_SCAN_LIMIT)
    except OSError:
        return None
    match = _VERSION_PATTERN.search(data)
    return match.group(1).decode("ascii") if match else None


def process_table(proc: Path = Path("/proc")) -> Dict[str, List[int]]:
    """argv[0]의 파일 이름 -> 그 프로세스들의 pid(오름차순). 머리 줄이 2초마다 부르므로 Path 대신 os로 읽는다(보드에서
    프로세스 125개에 83 ms → 19 ms). cmdline이 빈 커널 스레드는 뺀다."""
    table: Dict[str, List[int]] = {}
    try:
        entries = os.listdir(proc)
    except OSError:
        return table
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            with open(os.path.join(proc, entry, "cmdline"), "rb") as file:
                argv0 = file.read(4096).split(b"\0", 1)[0]
        except OSError:
            continue
        if argv0:
            table.setdefault(os.path.basename(os.fsdecode(argv0)), []).append(int(entry))
    return {name: sorted(pids) for name, pids in table.items()}


def find_process_pids(name: str, proc: Path = Path("/proc")) -> List[int]:
    """argv[0]의 파일 이름이 name인 프로세스들."""
    return process_table(proc).get(name, [])


def signal_process(name: str, signum: int = signal.SIGHUP) -> List[int]:
    """name 프로세스들에 신호를 보내고, 받은 pid를 돌려준다."""
    notified = []
    for pid in find_process_pids(name):
        try:
            os.kill(pid, signum)
            notified.append(pid)
        except (PermissionError, ProcessLookupError):
            continue
    return notified


class ProcessStatus:
    """install_dir은 실행 파일이 있는 곳이다: 매니저가 설치 디렉터리(보드 /root/edgepilot)에서 띄우므로 작업 디렉터리."""

    def __init__(self, path: str = topic_path(MANAGER_STATE_TOPIC), install_dir: Path = Path("."),
                 proc: Path = Path("/proc")):
        self.reader = IpcReader(path, MANAGER_STATE_HEAD.size + PROCESS_STATE.size * MAX_PROCESSES)
        self.install_dir = install_dir
        self.proc = proc
        self._versions: Dict[Tuple[int, int, int, int], Optional[str]] = {}

    def binary_version(self, path: Path) -> Optional[str]:
        """read_binary_version을 파일(장치, inode, 수정 시각, 크기)마다 한 번만 한다. /proc/<pid>/exe는 실행 중인
        파일을 가리키므로 배포로 바뀌어 지워진 파일도 읽힌다."""
        try:
            st = os.stat(path)
        except OSError:
            return None
        key = (st.st_dev, st.st_ino, st.st_mtime_ns, st.st_size)
        if key not in self._versions:
            if len(self._versions) > 64:
                self._versions.clear()
            self._versions[key] = read_binary_version(path)
        return self._versions[key]

    def versions(self, name: str, table: Dict[str, List[int]]) -> Tuple[Optional[str], Optional[str]]:
        """name 칸의 (실행 중인 실행 파일의 버전, 설치된 실행 파일의 버전). table은 process_table()이다. 손으로
        배포한 뒤 다시 띄우기 전이면 둘이 다르다. 버전을 새기기 전 실행 파일이나 web_console(파이썬)은 None이다."""
        binaries = PROCESS_BINARIES.get(name, (name,))
        for binary in binaries:
            pids = table.get(binary)
            if pids:
                return self.binary_version(self.proc / str(pids[0]) / "exe"), self.binary_version(self.install_dir / binary)
        return None, self.binary_version(self.install_dir / binaries[0])

    def status(self, versions: bool = False) -> Dict[str, Any]:
        """{"available", "stale", "age_s", "processes": [{"name", "running", "version", "installed_version"}]}.
        매니저가 없거나 3초 넘게 멈췄으면 stale이다. 빌드 버전은 versions일 때만 읽는다(/proc 전체를 훑으므로 머리 줄의
        목록을 펼쳤을 때만, 보드에서 약 19 ms). 아니면 버전 칸은 None이다."""
        latest = self.reader.read_payload()
        result: Dict[str, Any] = {**freshness(latest[1] if latest else None, MANAGER_MAX_AGE_S), "processes": []}
        if latest is None:
            return result
        payload = latest[2]
        _, count, _ = MANAGER_STATE_HEAD.unpack_from(payload)
        table = process_table(self.proc) if versions else None
        for index in range(min(count, MAX_PROCESSES)):
            raw_name, running = PROCESS_STATE.unpack_from(payload, MANAGER_STATE_HEAD.size + index * PROCESS_STATE.size)
            name = raw_name.split(b"\0", 1)[0].decode("ascii", "replace")
            version, installed = self.versions(name, table) if table is not None else (None, None)
            result["processes"].append({"name": name, "running": bool(running), "version": version,
                                        "installed_version": installed})
        return result
