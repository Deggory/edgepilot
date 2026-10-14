"""파라미터 탭의 구성: 그룹(탭)마다 파일, 바뀌면 다시 읽으라고 알릴 프로세스, 섹션 순서와, 항목마다 섹션·단위·범위.
숫자 항목의 min/max는 런타임의 클램프 표와 같아야 한다(check_web_console.py가 대조). 화면 글자(그룹 이름과 안내, 섹션
이름, 항목의 이름·설명·효과)는 언어마다 페이지의 문구 묶음 static/i18n/<언어>.json에 있다: group.<그룹>.label·note,
section.<섹션>, param.<그룹>.<키>.label·description·increase·decrease·learned_role, unit.<단위>, caution.<주의>.
check_web_console.py가 두 언어 모두 빠짐없는지 본다. 차량마다 다른 값(MDPS 위조 속도 등)은 설명에 적지 않는다.
페이지가 기본값(params.defaults)을 옆에 보여 준다."""
from __future__ import annotations

from typing import Any, Dict


def _with(meta: Dict[str, Any], **extra: Any) -> Dict[str, Any]:
    """비어 있지 않은 선택 필드만 붙인다."""
    meta.update({key: value for key, value in extra.items() if value})
    return meta


def number(section: str, unit: str, step: float, minimum: float, maximum: float, *, control: str = "", tab: str = "",
           learned: bool = False, caution: str = "") -> Dict[str, Any]:
    """숫자 항목. control="slider"면 −/+ 대신 슬라이더로 고친다. tab="vehicle"이면 조향 탭 대신 차량 특성 탭(아래
    VEHICLE_TAB)에 나온다. learned면 자동 학습 중 이 수동값이 하는 일(learned_role 문구)이 있고, caution은 언제나
    보이는 주의(caution.<이름> 문구)다. unit은 기호 그대로 보이고, 언어마다 다르게 쓰는 것만 unit.<단위> 문구가 있다."""
    return _with({"section": section, "unit": unit, "step": step, "min": minimum, "max": maximum},
                 control=control, tab=tab, learned=learned, caution=caution)


def toggle(section: str, *, tab: str = "", learned: bool = False, card: str = "") -> Dict[str, Any]:
    """켜고 끄는 항목. increase/decrease 문구가 켜면·끄면의 설명이다. tab·learned는 number와 같다. card는 그 탭의
    섹션 대신 전용 카드가 바꾸는 항목의 카드 이름이다(예: tab="vehicle", card="calibration" = 차량 특성 탭의 카메라
    캘리브레이션 카드, POST /api/calibration/source). 파라미터 API로는 바꿀 수 없고(ParamStore.update), section은
    파일 그룹의 섹션이다."""
    return _with({"section": section}, tab=tab, learned=learned, card=card)


PARAM_GROUPS: Dict[str, Dict[str, Any]] = {
    "steering": {
        "file": "steering.json", "notify": "controlsd",
        "sections": ["basics", "torque_controller", "path_mode", "lane_position", "driver", "lkas_fault"],
    },
    "adaptive_cruise": {
        "file": "adaptive_cruise.json", "notify": "controlsd",
        "sections": ["cruise", "gap", "speed_response", "resume", "vision", "buttons"],
    },
    "recording": {"file": "recording.json", "notify": "", "sections": ["recording"]},
    "display": {"file": "display.json", "notify": "", "sections": ["backlight", "sound", "camera_mount"]},
}

# 차량 특성 탭: steering.json 항목 중 차의 성질인 것(tab="vehicle"), 섹션 순서대로. 학습기 섹션의 값들은 학습기
# 하나가 함께 내므로 스위치도 하나다. 켜면(자동 학습) 학습값을, 학습이 값을 내기 전과 끄면(수동) 수동값을 제어에
# 쓴다(controlsd LateralController::live_params, plan_delay_s). 고정 제원은 학습하지 않는 차량 값이다.
VEHICLE_TAB: Dict[str, Any] = {
    "sections": ["vehicle_model", "torque_model", "steer_delay", "learner_inputs", "fixed_specs"],
    "learners": {
        "vehicle_model": {"learner": "paramsd", "switch": "use_live_vehicle_params"},
        "torque_model": {"learner": "torqued", "switch": "use_live_torque_params"},
        "steer_delay": {"learner": "lagd", "switch": "use_live_delay"},
    },
}

PARAM_METADATA: Dict[str, Dict[str, Dict[str, Any]]] = {
    "steering": {
        "enabled": toggle("basics"),
        "steering_pressed_threshold": number("driver", "MDPS raw", 10, 0, 500),
        "torque_lat_accel_factor": number("torque_model", "m/s²", 0.01, 0.5, 5.0, tab="vehicle", learned=True,
                                          caution="torqued_restart"),
        "torque_kp": number("torque_controller", "gain", 0.05, 0, 10),
        "torque_ki": number("torque_controller", "gain", 0.01, 0, 2),
        "torque_friction": number("torque_model", "torque", 0.005, 0, 0.3, tab="vehicle", learned=True,
                                  caution="torqued_restart"),
        "steer_ratio": number("vehicle_model", "ratio", 0.1, 8, 25, tab="vehicle", learned=True),
        "tire_stiffness_factor": number("vehicle_model", "x", 0.05, 0.2, 2, tab="vehicle", learned=True,
                                        caution="paramsd_restart"),
        "steer_actuator_delay": number("steer_delay", "s", 0.01, 0.01, 1, tab="vehicle", learned=True),
        "avoid_lkas_fault_enabled": toggle("lkas_fault"),
        "avoid_lkas_fault_hold_angle_deg": number("lkas_fault", "°", 1, 0, 84),
        "live_bank_compensation": toggle("vehicle_model", tab="vehicle", learned=True),
        "use_live_vehicle_params": toggle("vehicle_model", tab="vehicle"),
        "use_live_torque_params": toggle("torque_model", tab="vehicle"),
        "use_live_delay": toggle("steer_delay", tab="vehicle"),
        "use_locationd_learner_inputs": toggle("learner_inputs", tab="vehicle"),
        "torque_lat_accel_offset": number("torque_model", "m/s²", 0.01, -1.0, 1.0, tab="vehicle", learned=True),
        "angle_offset_deg": number("vehicle_model", "°", 0.1, -10, 10, tab="vehicle", learned=True),
        "mass_kg": number("fixed_specs", "kg", 10, 1000, 2600, tab="vehicle", caution="paramsd_restart"),
        "wheelbase_m": number("fixed_specs", "m", 0.01, 2, 3.5, tab="vehicle", caution="paramsd_restart"),
        "center_to_front_ratio": number("fixed_specs", "ratio", 0.01, 0.2, 0.7, tab="vehicle",
                                        caution="paramsd_restart"),
        "path_offset_m": number("lane_position", "m", 0.01, -1, 1),
        "lane_path_weight": number("lane_position", "weight", 0.1, 0.5, 10),
        "min_steer_speed_mps": number("basics", "m/s", 0.1, 0, 5),
        "laneless_mode": toggle("path_mode"),
        "lane_change_min_speed_kph": number("driver", "km/h", 1, 0, 80),
    },
    "adaptive_cruise": {
        "enabled": toggle("cruise"),
        "following_time_s": number("gap", "s", 0.1, 0.8, 4.0),
        "standstill_gap_m": number("gap", "m", 0.5, 2.0, 20.0),
        "gap_correction_gain": number("speed_response", "gain", 0.05, 0.05, 1.0),
        "max_slowdown_correction_mps": number("speed_response", "m/s", 0.5, 0.5, 10.0),
        "max_speedup_correction_mps": number("speed_response", "m/s", 0.5, 0.0, 5.0),
        "deceleration_rate_kph_per_s": number("speed_response", "km/h/s", 0.1, 0.2, 5.0),
        "lead_restore_delay_s": number("resume", "s", 0.5, 1.0, 10.0),
        "command_interval_s": number("buttons", "s", 0.1, 0.5, 5.0),
        "lead_hold_s": number("vision", "s", 0.1, 0.1, 2.0),
    },
    "recording": {
        "enabled": toggle("recording"),
        "bitrate_bps": number("recording", "bps", 500000, 1000000, 10000000),
    },
    "display": {
        "enabled": toggle("backlight"),
        "brightness_percent": number("backlight", "%", 1, 1, 100, control="slider"),
        "alert_volume_percent": number("sound", "%", 1, 0, 100, control="slider"),
        "camera_offset_m": number("camera_mount", "m", 0.01, -0.35, 0.35),
        "camera_height_m": number("camera_mount", "m", 0.01, 0.8, 2.0),
        "calibration_use_flow": toggle("camera_mount", tab="vehicle", card="calibration"),
    },
}
