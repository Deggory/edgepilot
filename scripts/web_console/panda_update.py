"""Panda 펌웨어 카드: pandad가 1초마다 쓰는 연결·버전·플래싱 진행(src/panda/pandad.cc)과 설치된 이미지를 보여
주고, 주차 중일 때만 플래싱을 요청한다. 요청 파일에는 쓸 이미지의 버전이 들어 있고, pandad는 그 버전이 설치된
이미지와 같고 차 상태가 맞을 때만 쓴다(같은 조건을 다시 본다). 이미지 규칙과 주차 조건은
src/panda/panda_firmware.cc와 같다."""
from __future__ import annotations

import json
import os
import re
import struct
from pathlib import Path
from typing import Any, Dict

from .refusal import Refusal
from .shm_channel import IpcReader, age_s, topic_path
from .state_layout import CONTROL_STATE_SIZE, CONTROL_STATE_TOPIC, GEAR_PARK, control_value

PANDA_STATUS_PATH = "/dev/shm/edgepilot_panda_status.json"
PANDA_FLASH_REQUEST_PATH = "/dev/shm/edgepilot_panda_flash"
# pandad와 같이 EDGEPILOT_PANDA_FIRMWARE, 없으면 설치 디렉터리의 firmware/panda.bin.signed
INSTALL_DIR = Path(__file__).resolve().parents[1]
PANDA_FIRMWARE_PATH = str(INSTALL_DIR / (os.environ.get("EDGEPILOT_PANDA_FIRMWARE") or "firmware/panda.bin.signed"))
PANDA_STATUS_STALE_S = 3.0
# panda_protocol.h와 같은 값: 서명 128바이트, 앱 영역(섹터 1~3) 48 KB, F413 보드(black, uno, dos)
PANDA_SIGNATURE_BYTES = 128
PANDA_APP_MAX_BYTES = 3 * 16 * 1024
PANDA_FLASHABLE_HW = (3, 5, 6)
PANDA_VERSION_PATTERN = re.compile(rb"(?<![0-9A-Za-z])[A-Z]{3,8}-[0-9A-Za-z]{8}-(?:DEBUG|RELEASE)(?![0-9A-Za-z])")
CONTROL_MAX_AGE_S = 1.0
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
    info: Dict[str, Any] = {"path": path, "present": False, "valid": False, "version": "", "size": 0, "error": "",
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
    match = PANDA_VERSION_PATTERN.search(data)
    if not error and not match:
        error = "no_version"
    info.update(present=True, size=size, valid=not error, error=error,
                version=match.group().decode() if match else "")
    return info


def flash_blocker(control: IpcReader, bootstub: bool = False) -> str:
    """panda_firmware.cc의 panda_flash_allowed와 같은 조건. 막는 이유 코드, 없으면 빈 문자열. 판다가 bootstub에
    머물러 있으면 차 상태는 보지 않고(CAN이 없어 알 수 없다, 상류 pandad도 같다), 살아 있는 제어 상태가 결합
    중이라고 할 때만 막는다."""
    latest = control.read_payload()
    fresh = latest is not None and age_s(latest[1]) <= CONTROL_MAX_AGE_S
    if bootstub:
        engaged = fresh and (control_value(latest[2], "engaged") or control_value(latest[2], "active"))
        return "engaged" if engaged else ""
    if not fresh:
        return "control_stale"
    payload = latest[2]
    if not control_value(payload, "vehicle_fresh"):
        return "vehicle_stale"
    if control_value(payload, "engaged") or control_value(payload, "active"):
        return "engaged"
    if control_value(payload, "gear", "<i") != GEAR_PARK:
        return "not_park"
    speed = max(control_value(payload, "ego_speed_kph", "<f"), control_value(payload, "cluster_speed_kph", "<f"))
    if speed >= PARKED_MAX_SPEED_KPH:
        return "moving"
    return ""


class PandaUpdate:
    def __init__(self, status_path: str = PANDA_STATUS_PATH, request_path: str = PANDA_FLASH_REQUEST_PATH,
                 image_path: str = PANDA_FIRMWARE_PATH, control_path: str = topic_path(CONTROL_STATE_TOPIC)):
        self.status_path = Path(status_path)
        self.request_path = Path(request_path)
        self.image_path = image_path
        self.control = IpcReader(control_path, CONTROL_STATE_SIZE)

    def _pandad(self) -> tuple[Dict[str, Any] | None, float]:
        """pandad의 상태 파일과 그 나이. 없거나 오래됐으면 (None, 나이)."""
        try:
            document = json.loads(self.status_path.read_text(encoding="utf-8"))
            age = age_s(int(document["stamp_ns"]))
        except (OSError, ValueError, KeyError, TypeError):
            return None, 0.0
        return (document if age <= PANDA_STATUS_STALE_S else None), age

    def status(self) -> Dict[str, Any]:
        pandad, age = self._pandad()
        link = pandad or {}
        flash = link.get("flash") or {}
        image = panda_image_info(self.image_path)
        blocker = flash_blocker(self.control, bootstub=link.get("mode") == "bootstub")
        version = link.get("firmware_version", "")
        return {
            "available": pandad is not None,
            "age_s": age,
            "mode": link.get("mode", "none"),
            "serial": link.get("serial", ""),
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

    def request(self, version: str) -> Dict[str, Any]:
        status = self.status()
        image = status["image"]
        if not status["available"]:
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
        temporary = self.request_path.with_name(self.request_path.name + ".tmp")
        temporary.write_text(version + "\n", encoding="utf-8")
        os.replace(temporary, self.request_path)
        return self.status()
