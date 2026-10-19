"""MaixCAM2 LCD 백라이트 상태. 백라이트는 PWM3(pwmchip0/pwm3, 정극성) 하나이고, 밝기는
duty = 주기 x 밝기% x 보드 최대치%(/boot/board disp_max_backlight)다(maix_display.cc). overlayd가
시작할 때 /boot/configs 값으로 켜고 곧바로 display.json 값으로 바꾼다. 웹 콘솔은 파일만 쓰고 여기서 실제 값을
읽어 보인다."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict


PWM_CHIP = Path("/sys/class/pwm/pwmchip0")
PWM_CHANNEL = 3
BOARD_FILE = Path("/boot/board")
DEFAULT_MAX_PERCENT = 95


def board_max_percent(board_file: Path = BOARD_FILE) -> int:
    for line in board_file.read_text().splitlines():
        if line.startswith("disp_max_backlight="):
            return int(line.split("=", 1)[1])
    return DEFAULT_MAX_PERCENT


class DisplayBacklight:
    """백라이트의 실제 상태를 sysfs에서 읽는다. 쓰는 것은 overlayd뿐이다: display.json을 1초 안에 적용하고, 다시 뜰
    때마다 다시 적용한다. 예전에는 웹 콘솔도 따로 써서, overlayd가 다시 뜨면 밝기가 /boot/configs 값으로 돌아갔는데
    페이지는 옛 값을 보였다."""

    def __init__(self, pwm_chip: Path = PWM_CHIP, board_file: Path = BOARD_FILE) -> None:
        self.pwm_chip = pwm_chip
        self.pwm_path = pwm_chip / f"pwm{PWM_CHANNEL}"
        self.max_percent = board_max_percent(board_file)

    def status(self) -> Dict[str, Any]:
        """{"available", "enabled", "brightness_percent"(보드 최대치로 나눈 밝기, 모르면 None)}. overlayd가 PWM을
        내보내기(export) 전이면 꺼짐이다."""
        if not self.pwm_path.exists():
            return {"available": self.pwm_chip.exists(), "enabled": False, "brightness_percent": None}
        enabled = int((self.pwm_path / "enable").read_text()) == 1
        period = int((self.pwm_path / "period").read_text())  # 막 내보낸 PWM은 주기가 0이다
        duty = int((self.pwm_path / "duty_cycle").read_text())
        return {"available": True, "enabled": enabled and duty > 0,
                "brightness_percent": round(duty * 10_000 / (period * self.max_percent)) if period else None}
