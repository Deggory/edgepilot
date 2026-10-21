"""Panda 펌웨어 카드: pandad가 1초마다 쓰는 연결·버전·플래싱 진행(src/panda/pandad.cc)과 설치된 이미지를 보여
주고, 주차 중일 때만 플래싱을 요청한다. 요청 파일에는 쓸 이미지의 버전이 들어 있고, pandad는 그 버전이 설치된
이미지와 같고 차 상태가 맞을 때만 쓴다(같은 조건을 다시 본다). 이미지 규칙과 주차 조건은
src/panda/panda_firmware.cc와 같다."""
from __future__ import annotations

import re
import struct
from pathlib import Path
from typing import Any, Dict

from .files import env_path, read_json_or_none, write_text_atomic
from .refusal import Refusal
from .shm_channel import IpcReader, freshness, topic_path
from .state_layout import (CONTROL_STATE_MAX_AGE_S, CONTROL_STATE_SIZE, CONTROL_STATE_TOPIC, GEAR_PARK, control_engaged,
                           control_value)

PANDA_STATUS_PATH = "/dev/shm/edgepilot_panda_status.json"
PANDA_FLASH_REQUEST_PATH = "/dev/shm/edgepilot_panda_flash"
# pandad와 같이 EDGEPILOT_PANDA_FIRMWARE, 없으면 작업(설치) 디렉터리의 firmware/panda.bin.signed
PANDA_FIRMWARE_PATH = "firmware/panda.bin.signed"
PANDA_STATUS_MAX_AGE_S = 3.0  # pandad는 1초마다 쓴다
# panda_protocol.h와 같은 값: 서명 128바이트, 앱 영역(섹터 1~3) 48 KB, F413 보드(black, uno, dos)
PANDA_SIGNATURE_BYTES = 128
PANDA_APP_MAX_BYTES = 3 * 16 * 1024
PANDA_FLASHABLE_HW = (3, 5, 6)
# panda_firmware.cc find_version과 같이 -DEBUG를 이미지 전체에서 먼저 찾고, 없으면 -RELEASE를 찾는다
PANDA_VERSION_PATTERNS = tuple(re.compile(rb"(?<![0-9A-Za-z])[A-Z]{3,8}-[0-9A-Za-z]{8}-" + suffix + rb"(?![0-9A-Za-z])")
                               for suffix in (rb"DEBUG", rb"RELEASE"))
PARKED_MAX_SPEED_KPH = 1.0
# pandad·panda_flasher와 flash_blocker의 이유 코드, pandad가 알리는 플래싱 단계. 페이지가 panda.reason.<코드>,
# panda.step.<단계> 문구로 보인다.
REASONS = ("control_stale", "vehicle_stale", "engaged", "not_park", "moving", "image_invalid", "image_changed",
           "no_panda", "hw_unsupported", "busy", "usb_error", "bootstub_timeout", "flasher_missing", "erase_failed",
           "write_failed", "verify_failed", "app_timeout", "version_mismatch")
STEPS = ("bootstub", "erase", "write", "verify", "reboot", "check")
# 설치된 이미지의 문제(panda_image_info의 error). 페이지가 panda.image.<코드> 문구로 보인다.
IMAGE_ERRORS = ("missing", "too_small", "too_large", "unaligned", "unsigned", "no_tail", "no_version")


def panda_image_info(path: str) -> Dict[str, Any]:
    """설치된 앱 이미지(firmware/panda의 panda.bin.signed)를 panda_firmware.cc와 같은 규칙으로 본다. error는
    IMAGE_ERRORS의 코드이고 max_bytes는 앱 영역의 크기다."""
    info: Dict[str, Any] = {"present": False, "valid": False, "version": "", "size": 0, "error": "",
                            "max_bytes": PANDA_APP_MAX_BYTES}
    try:
        data = Path(path).read_bytes()
    except OSError:
        info["error"] = "missing"
        return info
    size = len(data)
    body = size - PANDA_SIGNATURE_BYTES
    if size <= PANDA_SIGNATURE_BYTES + 16:
        error = "too_small"
    elif size > PANDA_APP_MAX_BYTES:
        error = "too_large"
    elif size % 4:
        error = "unaligned"
    elif struct.unpack_from("<I", data)[0] != body:
        error = "unsigned"
    elif data[body - 8:body - 4] != b"VERS":
        error = "no_tail"
    else:
        error = ""
    match = next(filter(None, (pattern.search(data) for pattern in PANDA_VERSION_PATTERNS)), None)
    if not error and not match:
        error = "no_version"
    info.update(present=True, size=size, valid=not error, error=error,
                version=match.group().decode() if match else "")
    return info


def flash_blocker(control: IpcReader, bootstub: bool = False) -> str:
    """panda_firmware.cc의 panda_flash_allowed와 같은 조건. 막는 이유 코드, 없으면 빈 문자열. 판다가 bootstub에
    머물러 있으면 차 상태는 보지 않고(CAN이 없어 알 수 없다, 상류 pandad도 같다), 살아 있는 제어 상태가 결합
    중이라고 할 때만 막는다."""
    latest = control.read_fresh(CONTROL_STATE_MAX_AGE_S)
    if bootstub:
        return "engaged" if latest is not None and control_engaged(latest[2]) else ""
    if latest is None:
        return "control_stale"
    payload = latest[2]
    if not control_value(payload, "vehicle_fresh"):
        return "vehicle_stale"
    if control_engaged(payload):
        return "engaged"
    if control_value(payload, "gear") != GEAR_PARK:
        return "not_park"
    speed = max(control_value(payload, "ego_speed_kph"), control_value(payload, "cluster_speed_kph"))
    if speed >= PARKED_MAX_SPEED_KPH:
        return "moving"
    return ""


class PandaUpdate:
    def __init__(self, status_path: str = PANDA_STATUS_PATH, request_path: str = PANDA_FLASH_REQUEST_PATH,
                 image_path: str | None = None, control_path: str = topic_path(CONTROL_STATE_TOPIC)):
        self.status_path = Path(status_path)
        self.request_path = Path(request_path)
        self.image_path = image_path or str(env_path("EDGEPILOT_PANDA_FIRMWARE", PANDA_FIRMWARE_PATH))
        self.control = IpcReader(control_path, CONTROL_STATE_SIZE)

    def status(self) -> Dict[str, Any]:
        """pandad 상태 파일(stamp_ns가 CLOCK_BOOTTIME)의 신선도와, 신선할 때만 그 내용."""
        document = read_json_or_none(self.status_path)
        stamp = document.get("stamp_ns") if document else None
        fresh = freshness(stamp if isinstance(stamp, int) else None, PANDA_STATUS_MAX_AGE_S)
        link = document if not fresh["stale"] else {}
        flash = link.get("flash") or {}
        image = panda_image_info(self.image_path)
        blocker = flash_blocker(self.control, bootstub=link.get("mode") == "bootstub")
        version = link.get("firmware_version", "")
        return {
            **fresh,
            "mode": link.get("mode", "none"),
            "hw_type": link.get("hw_type", 0),
            "hw_name": link.get("hw_name", ""),
            "hw_supported": link.get("hw_type", 0) in PANDA_FLASHABLE_HW,
            "firmware_version": version,
            "flash": flash,
            "image": image,
            "blocker": blocker,
            "request_pending": self.request_path.exists(),
            "up_to_date": bool(image["version"]) and image["version"] == version,
        }

    def request_flash(self, version: str) -> Dict[str, Any]:
        status = self.status()
        image = status["image"]
        if status["stale"]:
            raise Refusal("error.pandad_missing")
        if status["request_pending"] or status["flash"].get("state") == "flashing":
            raise Refusal("error.panda_flashing")
        if not image["valid"]:
            raise Refusal(f"panda.image.{image['error']}", max_bytes=image["max_bytes"])
        if version != image["version"]:
            raise Refusal("panda.reason.image_changed")
        if status["mode"] == "none":
            raise Refusal("panda.reason.no_panda")
        if status["mode"] == "app" and status["hw_type"] not in PANDA_FLASHABLE_HW:
            raise Refusal("panda.reason.hw_unsupported")
        if status["blocker"]:
            raise Refusal(f"panda.reason.{status['blocker']}")
        write_text_atomic(self.request_path, version + "\n")
        return self.status()
