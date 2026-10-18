"""녹화 탭의 주행 기록 카드(docs/runtime-options.md): recordd가 SD 카드에 쌓은 route(recordings/<이름>)마다 크기와 길이를
보이고, 고른 route를 지운다.

- route는 이름 규칙(시작 시각 YYYY-MM-DD--HH-MM-SS-mmm, recording_writer.cc route_name)에 맞는 디렉터리만 다룬다. 다른
  파일과 디렉터리, 링크는 보이지도 지우지도 않는다.
- 녹화 중인 route는 tmpfs 스테이징(recording_writer.cc, EDGEPILOT_RECORD_STAGING)에 쓰이고 닫힌 파일부터 SD로 옮겨지며,
  다 옮기면 스테이징의 디렉터리가 없어진다(StagingMover::move_tree). 그래서 스테이징에 같은 이름이 있으면 쓰는 중이거나
  옮기는 중이라 지우지 않는다(busy). 막 시작해 스테이징에만 있는 route도 busy로 보인다.
- 크기는 할당된 블록(du와 같은 값)이라 지우면 그만큼 빈다.
- recordd는 남은 공간이 5 GiB와 SD의 10% 가운데 큰 값 밑이면 녹화를 거부하거나 멈춘다(recording_writer.cc
  storage_problem). 카드는 그 기준과 recordd가 내는 녹화 상태(recordState)를 함께 보인다."""
from __future__ import annotations

import os
import re
import shutil
import struct
from pathlib import Path
from typing import Any, Dict, List

from .files import env_path, read_json_or_none
from .refusal import Refusal
from .shm_channel import IpcReader, freshness, topic_path

# recordd(recordd.cc)·recording_writer.cc와 같은 경로와 변수. 경로는 작업 디렉터리(설치 디렉터리) 기준이다.
RECORD_ROOT = "recordings"
STAGING_ROOT = "/tmp/record_staging"
ROUTE_NAME = re.compile(r"\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2}-\d{3}")
# recording_writer.cc kMinimumFreeBytes, kMinimumFreePercent
MIN_FREE_BYTES = 5 << 30
MIN_FREE_PERCENT = 10
# ipc_messages.h RecordState(recordd가 0.5 s마다 낸다)와 kStorageBlocked* 값
RECORD_STATE_TOPIC = "/edgepilot_record_state"
RECORD_STATE = struct.Struct("<QII")  # timestamp_ns, active, storage_blocked
RECORD_STATE_MAX_AGE_S = 2.0
STORAGE_BLOCKED = {1: "sd_reserve", 2: "staging"}


def route_names(directory: Path) -> set:
    """디렉터리 안에서 이름 규칙에 맞는 route 디렉터리(링크가 아닌 것)의 이름들. 없으면 빈 집합."""
    try:
        with os.scandir(directory) as entries:
            return {entry.name for entry in entries
                    if ROUTE_NAME.fullmatch(entry.name) and entry.is_dir(follow_symlinks=False)}
    except OSError:
        return set()


def tree_bytes(path: Path) -> int:
    """디렉터리 아래 파일들이 차지한 블록(du와 같은 값). 링크는 따라가지 않고, 읽는 사이에 사라진 것은 건너뛴다."""
    total = 0
    pending = [path]
    while pending:
        try:
            with os.scandir(pending.pop()) as entries:
                for entry in entries:
                    try:
                        if entry.is_dir(follow_symlinks=False):
                            pending.append(Path(entry.path))
                        else:
                            total += entry.stat(follow_symlinks=False).st_blocks * 512
                    except OSError:
                        continue
        except OSError:
            continue
    return total


def segment_count(*routes: Path) -> int:
    """route들(SD와 스테이징의 같은 route)의 segments/ 아래 세그먼트 디렉터리 수. 한 세그먼트가 약 60 s다."""
    names = set()
    for route in routes:
        try:
            with os.scandir(route / "segments") as entries:
                names.update(entry.name for entry in entries if entry.is_dir(follow_symlinks=False))
        except OSError:
            continue
    return len(names)


def video_seconds(manifest: Dict[str, Any] | None) -> float | None:
    """닫힌 route의 영상 길이(manifest의 video_frames / fps). 쓰는 중이거나 끝맺지 못한 route(전원이 끊긴 주행)는
    manifest가 시작할 때의 값이라 None이다."""
    if not manifest or manifest.get("complete") is not True:
        return None
    frames, fps = manifest.get("video_frames"), manifest.get("fps")
    if not isinstance(frames, int) or not isinstance(fps, (int, float)) or fps <= 0:
        return None
    return round(frames / fps, 1)


class Recordings:
    def __init__(self, root: Path | None = None, staging: Path | None = None, state_path: str | None = None):
        self.root = root if root is not None else env_path("EDGEPILOT_RECORD_ROOT", RECORD_ROOT)
        self.staging = staging if staging is not None else env_path("EDGEPILOT_RECORD_STAGING", STAGING_ROOT)
        self.state = IpcReader(state_path or topic_path(RECORD_STATE_TOPIC), RECORD_STATE.size)

    def routes(self) -> List[Dict[str, Any]]:
        """route마다 {name, bytes, busy, complete, duration_s, segments}, 최근 것부터(이름이 시작 시각이다).
        complete는 manifest의 값이고 manifest가 없으면 None이다."""
        stored, staged = route_names(self.root), route_names(self.staging)
        out = []
        for name in sorted(stored | staged, reverse=True):
            busy = name in staged
            places = [path for path, present in ((self.root / name, name in stored), (self.staging / name, busy))
                      if present]
            manifest = read_json_or_none(places[0] / "manifest.json")
            out.append({
                "name": name,
                "bytes": sum(tree_bytes(path) for path in places),
                "busy": busy,
                "complete": manifest.get("complete") is True if manifest else None,
                "duration_s": None if busy else video_seconds(manifest),
                "segments": segment_count(*places),
            })
        return out

    def recording(self) -> Dict[str, Any]:
        """recordd의 녹화 상태: 신선도와 active(route를 쓰는 중), storage_blocked(공간 때문에 멈춘 이유 코드, 없으면 "")."""
        latest = self.state.read_payload()
        state = {**freshness(latest[1] if latest else None, RECORD_STATE_MAX_AGE_S), "active": False,
                 "storage_blocked": ""}
        if latest is not None:
            _, active, blocked = RECORD_STATE.unpack_from(latest[2])
            state.update(active=bool(active), storage_blocked=STORAGE_BLOCKED.get(blocked, str(blocked) if blocked else ""))
        return state

    def disk(self) -> Dict[str, Any] | None:
        """route를 쌓는 파일 시스템의 전체·남은 크기와 recordd가 녹화를 멈추는 남은 공간. 디렉터리가 없으면 None."""
        try:
            space = os.statvfs(self.root)
        except OSError:
            return None
        block = space.f_frsize or space.f_bsize
        total, free = block * space.f_blocks, block * space.f_bavail
        return {"total_bytes": total, "free_bytes": free,
                "reserve_bytes": max(MIN_FREE_BYTES, total * MIN_FREE_PERCENT // 100)}

    def status(self) -> Dict[str, Any]:
        routes = self.routes()
        return {"routes": routes, "total_bytes": sum(route["bytes"] for route in routes), "disk": self.disk(),
                "recording": self.recording(), "root": str(self.root)}

    def delete(self, names: List[str]) -> Dict[str, Any]:
        """고른 route들을 지우고 새 상태와 지운 이름, 비운 크기를 돌려준다. 하나라도 받을 수 없으면 아무것도 지우지
        않는다: 이름 규칙 밖(400), 없는 것(404), 쓰는 중이거나 옮기는 중(409)."""
        chosen = list(dict.fromkeys(names))
        if not chosen:
            raise Refusal("error.recordings_none", status=400)
        staged = route_names(self.staging)
        for name in chosen:
            if not isinstance(name, str) or not ROUTE_NAME.fullmatch(name):
                raise Refusal("error.recording_bad_name", status=400, name=str(name))
            if name in staged:
                raise Refusal("error.recording_busy", name=name)
            path = self.root / name
            if path.is_symlink() or not path.is_dir():
                raise Refusal("error.recording_unknown", status=404, name=name)
        freed = 0
        for name in chosen:
            path = self.root / name
            freed += tree_bytes(path)
            shutil.rmtree(path)
        return {**self.status(), "deleted": chosen, "freed_bytes": freed}
