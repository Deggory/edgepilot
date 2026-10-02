#!/usr/bin/env python3
import json
import re
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.display_control import duty_cycle_ns
from scripts.param_server import (
    CALIBRATION_STATE,
    CONTROL_STATE_HEAD,
    HTML,
    MODEL_CALIBRATION_OFFSET,
    CalibrationControl,
    IPC_HEADER,
    IPC_MAGIC,
    LEARNER_FIELDS,
    LEARNER_STATE,
    LearnerMonitor,
    LearnerStateReader,
    LOCALIZATION_FIELDS,
    LOCALIZATION_STATE,
    LocalizationReader,
    LOCALIZATION_FLAGS,
    LOCALIZATION_INPUT_FLAGS,
    PARAM_METADATA,
    ParamStore,
    boottime_ns,
    fixed_lateral_values,
)


class FakeDisplayController:
    def __init__(self):
        self.applied = []

    def apply(self, document):
        self.applied.append(dict(document))

    def status(self):
        latest = self.applied[-1] if self.applied else {}
        return {
            "available": True,
            "enabled": latest.get("enabled", True),
            "brightness_percent": latest.get("brightness_percent", 100),
            "mode": "test",
            "error": "",
        }


class ParamStoreTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.paths = {
            "steering": root / "steering.json",
            "driving": root / "driving.json",
            "adaptive_cruise": root / "adaptive_cruise.json",
            "recording": root / "recording.json",
            "display": root / "display.json",
        }
        self.paths["steering"].write_text(
            json.dumps({"gain": 10, "enabled": True}), encoding="utf-8"
        )
        self.paths["driving"].write_text(
            json.dumps({"delay": 0.4}), encoding="utf-8"
        )
        self.paths["adaptive_cruise"].write_text(
            json.dumps({"following_time_s": 1.8}), encoding="utf-8"
        )
        self.paths["recording"].write_text(
            json.dumps({"enabled": False}), encoding="utf-8"
        )
        self.paths["display"].write_text(
            json.dumps({"enabled": True, "brightness_percent": 100}), encoding="utf-8"
        )
        self.notifications = 0

        def notify():
            self.notifications += 1
            return [123]

        self.store = ParamStore(self.paths, notify)

    def tearDown(self):
        self.temporary.cleanup()

    def test_atomic_single_value_update(self):
        result = self.store.update("steering", {"gain": 12})
        self.assertEqual(result["notified_pids"], [123])
        self.assertEqual(self.notifications, 1)
        self.assertEqual(self.store.read_group("steering"), {"gain": 12, "enabled": True})
        self.assertEqual(list(self.paths["steering"].parent.glob("*.tmp")), [])

    def test_unknown_parameter_is_rejected(self):
        with self.assertRaises(KeyError):
            self.store.update("driving", {"unknown": 1})
        self.assertEqual(self.notifications, 0)
        self.assertEqual(self.store.read_group("driving"), {"delay": 0.4})

    def test_recording_toggle_does_not_signal_controlsd(self):
        result = self.store.update("recording", {"enabled": True})
        self.assertEqual(result["notified_pids"], [])
        self.assertEqual(self.notifications, 0)

    def test_display_update_applies_hardware_without_signaling_controlsd(self):
        controller = FakeDisplayController()
        store = ParamStore(
            self.paths,
            self.store.notifier,
            display_controller=controller,
        )
        controller.applied.clear()
        result = store.update("display", {"brightness_percent": 35})
        self.assertEqual(result["notified_pids"], [])
        self.assertEqual(controller.applied, [{"enabled": True, "brightness_percent": 35}])
        self.assertEqual(store.read_group("display")["brightness_percent"], 35)

    def test_alert_volume_update_leaves_backlight_alone(self):
        self.paths["display"].write_text(
            json.dumps({"enabled": True, "brightness_percent": 80, "alert_volume_percent": 70}),
            encoding="utf-8")
        controller = FakeDisplayController()
        store = ParamStore(self.paths, self.store.notifier, display_controller=controller)
        controller.applied.clear()
        result = store.update("display", {"alert_volume_percent": 30})
        self.assertEqual(result["params"]["alert_volume_percent"], 30)
        self.assertEqual(result["notified_pids"], [])
        self.assertEqual(controller.applied, [], "volume is read by overlayd, not the backlight")
        store.update("display", {"brightness_percent": 60})
        self.assertEqual(controller.applied[-1]["brightness_percent"], 60)
        self.assertEqual(json.loads(self.paths["display"].read_text())["alert_volume_percent"], 30)

    def test_display_pwm_duty_scales_by_board_maximum(self):
        self.assertEqual(duty_cycle_ns(100), 95_000)
        self.assertEqual(duty_cycle_ns(51), 48_450)
        self.assertEqual(duty_cycle_ns(1), 950)
        self.assertEqual(duty_cycle_ns(0), 0)
        self.assertEqual(duty_cycle_ns(100, max_percent=100), 100_000)

    def test_missing_defaults_are_added_without_overwriting_tuning(self):
        defaults = Path(self.temporary.name) / "adaptive.defaults.json"
        defaults.write_text(
            json.dumps({"following_time_s": 1.8, "deceleration_rate_kph_per_s": 1.5}),
            encoding="utf-8",
        )
        self.paths["adaptive_cruise"].write_text(
            json.dumps({"following_time_s": 2.2}), encoding="utf-8"
        )
        store = ParamStore(
            self.paths,
            lambda: [],
            {"adaptive_cruise": defaults},
        )
        self.assertEqual(
            store.read_group("adaptive_cruise"),
            {"following_time_s": 2.2, "deceleration_rate_kph_per_s": 1.5},
        )

    def test_repository_params_have_complete_ui_metadata(self):
        root = Path(__file__).resolve().parents[1]
        for group, filename in (
            ("steering", "steering.json"),
            ("driving", "driving.json"),
            ("adaptive_cruise", "adaptive_cruise.json"),
            ("recording", "recording.json"),
            ("display", "display.json"),
        ):
            params = json.loads((root / "params" / filename).read_text(encoding="utf-8"))
            self.assertEqual(set(params), set(PARAM_METADATA[group]))
            for key, value in params.items():
                metadata = PARAM_METADATA[group][key]
                for field in ("label", "section", "description", "increase", "decrease"):
                    self.assertTrue(metadata.get(field), f"{group}.{key}.{field}")
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    for field in ("step", "min", "max"):
                        self.assertIsInstance(
                            metadata.get(field), (int, float), f"{group}.{key}.{field}"
                        )

    def test_ui_ranges_match_runtime_clamps(self):
        """The loaders clamp to their Json*Field tables; the editor must show
        the same min/max or it accepts values the runtime silently changes."""
        root = Path(__file__).resolve().parents[1]
        row = re.compile(r'\{"(\w+)",\s*(-?[\d.]+)f?,\s*(-?[\d.]+)f?,\s*&\w+::\w+\}')
        tables = {
            "steering": ("src/control_params.cc", ("kSteeringInts", "kSteeringFloats")),
            "driving": ("src/control_params.cc", ("kDrivingInts", "kDrivingFloats")),
            "adaptive_cruise": ("src/adaptive_cruise.cc", ("kAdaptiveInts", "kAdaptiveFloats")),
            # display는 백라이트(display_control.py) 항목도 있어 런타임이 읽는 키만 대조한다
            "display": ("src/device_settings.h", ("kDeviceSettingsFloats",)),
        }
        for group, (source, names) in tables.items():
            text = (root / source).read_text(encoding="utf-8")
            runtime = {}
            for name in names:
                body = re.search(name + r"\[\] = \{(.*?)\n\};", text, re.S)
                self.assertIsNotNone(body, f"{source}: {name}")
                runtime.update({key: (float(low), float(high))
                                for key, low, high in row.findall(body.group(1))})
            self.assertTrue(runtime, f"{source}: no rows parsed")
            ui = {key: meta for key, meta in PARAM_METADATA[group].items() if "min" in meta}
            if group == "display":
                ui = {key: meta for key, meta in ui.items() if key in runtime}
            self.assertEqual(set(runtime), set(ui), group)
            for key, (low, high) in runtime.items():
                self.assertEqual((float(ui[key]["min"]), float(ui[key]["max"])), (low, high),
                                 f"{group}.{key}")


def learner_payload(**values):
    fields = []
    for name, fmt in LEARNER_FIELDS:
        default = [0] * int(fmt[:-1]) if len(fmt) > 1 else 0
        value = values.get(name, default)
        fields.extend(value if isinstance(value, list) else [value])
    return LEARNER_STATE.pack(*fields)


class LearnerStateTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary.name) / "edgepilot_learner_state"

    def tearDown(self):
        self.temporary.cleanup()

    def publish(self, seq, **values):
        payload = learner_payload(**values)
        header = IPC_HEADER.pack(IPC_MAGIC, 1, LEARNER_STATE.size, 0, seq, 123, len(payload), 0)
        self.path.write_bytes(header + payload)

    def test_layout_matches_cpp_offsets(self):
        """ipc_messages.h의 offsetof 고정값과 Python 필드 배치가 같아야 한다."""
        source = (Path(__file__).resolve().parents[1] / "src" / "ipc_messages.h").read_text(encoding="utf-8")
        size = int(re.search(r"sizeof\(LearnerState\) == (\d+)", source).group(1))
        self.assertEqual(LEARNER_STATE.size, size)
        offsets, offset = {}, 0
        for name, fmt in LEARNER_FIELDS:
            offsets[name] = offset
            offset += struct.calcsize("<" + fmt)
        asserted = re.findall(r"EDGEPILOT_LEARNER_STATE_AT\((\w+), (\d+)\);", source)
        self.assertGreaterEqual(len(asserted), 8)
        for name, expected in asserted:
            self.assertEqual(offsets[name], int(expected), name)

    def test_reader_decodes_and_skips_torn_writes(self):
        self.publish(4, timestamp_ns=5_000_000_000, flags=0b1000000011, steer_ratio=14.88,
                     bucket_points=[12, 204, 464, 1440, 1500, 1036, 292, 49])
        reader = LearnerStateReader(str(self.path))
        seq, _, state = reader.read()
        self.assertEqual(seq, 4)
        self.assertAlmostEqual(state["steer_ratio"], 14.88, places=5)
        self.assertEqual(state["bucket_points"][4], 1500)
        self.assertTrue(state["flags"]["vehicle_inputs_ok"] and state["flags"]["vehicle_valid"])
        self.assertTrue(state["flags"]["use_vehicle"] and not state["flags"]["use_torque"])
        self.publish(5, steer_ratio=1.0)  # 쓰는 중
        self.assertIsNone(reader.read())
        self.path.unlink()
        self.assertIsNone(reader.read())

    def test_monitor_keeps_one_trend_row_per_publish(self):
        monitor = LearnerMonitor(LearnerStateReader(str(self.path)))
        self.assertFalse(monitor.snapshot({})["available"])
        self.publish(2, timestamp_ns=1_000_000_000, lat_accel_factor=4.44)
        monitor.sample()
        monitor.sample()
        self.publish(4, timestamp_ns=2_000_000_000, lat_accel_factor=4.40)
        snapshot = monitor.snapshot(json.loads(
            (Path(__file__).resolve().parents[1] / "params" / "steering.json").read_text(encoding="utf-8")))
        self.assertTrue(snapshot["available"])
        self.assertEqual([row[0] for row in monitor.trend()], [1.0, 2.0])
        self.assertAlmostEqual(snapshot["trend_row"][7], 4.40, places=5)
        self.publish(6, timestamp_ns=500_000_000)  # 시각이 거꾸로: 다른 부팅
        monitor.sample()
        self.assertEqual([row[0] for row in monitor.trend()], [0.5])

    def test_fixed_values_are_the_manual_values(self):
        fixed = fixed_lateral_values({"torque_lat_accel_factor": 2.8, "torque_friction": 0.1,
                                      "steer_ratio": 14.9})
        self.assertEqual((fixed["lat_accel_factor"], fixed["friction"], fixed["steer_ratio"]),
                         (2.8, 0.1, 14.9))

    def test_page_has_learner_tab(self):
        self.assertIn('data-group="learners"', HTML)
        self.assertIn("/api/learners/trend", HTML)
        # HTML은 일반 문자열이라 JS의 \n을 두 번 이스케이프해야 한다(아니면 스크립트 전체가 죽는다)
        self.assertIn('lines.join("\\n")', HTML)
        # 학습 스위치는 실시간 학습 탭에만 있다(조향 탭에서 숨김)
        hidden = re.search(r"hiddenKeys = \{steering: \[(.*?)\]\}", HTML, re.S)
        self.assertIsNotNone(hidden)
        self.assertEqual(set(re.findall(r'"(\w+)"', hidden.group(1))),
                         {"use_live_vehicle_params", "use_live_torque_params", "use_live_delay",
                          "use_locationd_learner_inputs"})
        self.assertIn('inputShell()', HTML)



class CalibrationControlTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.model = root / "edgepilot_model_state"
        self.control = root / "edgepilot_control_state"
        self.request = root / "edgepilot_calibration_reset"
        self.calibration = CalibrationControl(str(self.model), str(self.control), str(self.request))

    def tearDown(self):
        self.temporary.cleanup()

    @staticmethod
    def write(path, payload, stamp=None):
        stamp = boottime_ns() if stamp is None else stamp
        path.write_bytes(IPC_HEADER.pack(IPC_MAGIC, 1, len(payload), 0, 2, stamp, len(payload), 0) + payload)

    def publish_model(self, status, blocks, rpy_deg):
        payload = bytearray(3256)
        rad = [x * 3.141592653589793 / 180 for x in rpy_deg]
        CALIBRATION_STATE.pack_into(payload, MODEL_CALIBRATION_OFFSET, status, blocks, *rad, 0.1, 0.2, 0.3)
        self.write(self.model, bytes(payload))

    def publish_control(self, engaged, stamp=None):
        self.write(self.control, CONTROL_STATE_HEAD.pack(1, 1, engaged) + bytes(228), stamp)

    def test_offset_matches_cpp(self):
        source = (Path(__file__).resolve().parents[1] / "src" / "ipc_messages.h").read_text(encoding="utf-8")
        offset = re.search(r"offsetof\(ModelState, calibration\) == (\d+)", source)
        self.assertEqual(int(offset.group(1)), MODEL_CALIBRATION_OFFSET)
        self.assertEqual(CALIBRATION_STATE.size, 32)
        self.assertRegex(source, r"EDGEPILOT_CONTROL_STATE_AT\(engaged, 12\);")

    def test_status_decodes_model_state(self):
        self.assertFalse(self.calibration.status()["available"])
        self.publish_model(1, 7, [0.0, 0.29, -0.18])
        status = self.calibration.status()
        self.assertTrue(status["available"])
        self.assertEqual((status["status"], status["valid_blocks"]), ("calibrated", 7))
        self.assertAlmostEqual(status["rpy_deg"][1], 0.29, places=3)
        self.assertAlmostEqual(status["rpy_deg"][2], -0.18, places=3)

    def test_reset_is_refused_while_engaged(self):
        self.publish_control(engaged=1)
        with self.assertRaises(PermissionError):
            self.calibration.request_reset()
        self.assertFalse(self.request.exists())
        self.publish_control(engaged=0)
        self.assertTrue(self.calibration.request_reset()["reset_pending"])
        self.assertTrue(self.request.exists())

    def test_stale_or_missing_controlsd_does_not_block(self):
        self.assertFalse(self.calibration.engaged())
        self.publish_control(engaged=1, stamp=boottime_ns() - 5_000_000_000)
        self.assertFalse(self.calibration.engaged())

    def test_page_has_reset_button(self):
        self.assertIn("/api/calibration/reset", HTML)


class LocalizationStateTest(unittest.TestCase):
    def test_layout_matches_cpp_size(self):
        source = (Path(__file__).resolve().parents[1] / "src" / "ipc_messages.h").read_text(encoding="utf-8")
        size = int(re.search(r"sizeof\(LocalizationState\) == (\d+)", source).group(1))
        self.assertEqual(LOCALIZATION_STATE.size, size)
        flags = re.findall(r"constexpr uint32_t kLocalization(\w+) = 1U << (\d+);", source)
        state_flags = [int(bit) for name, bit in flags if not name.startswith(("Invalid", "CameraGuarded"))]
        input_flags = [int(bit) for name, bit in flags if name.startswith(("Invalid", "CameraGuarded"))]
        self.assertEqual(state_flags, list(range(len(LOCALIZATION_FLAGS))))
        self.assertEqual(input_flags, list(range(len(LOCALIZATION_INPUT_FLAGS))))

    def test_reader_decodes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "edgepilot_localization"
            reader = LocalizationReader(str(path))
            self.assertFalse(reader.snapshot({})["available"])
            fields = []
            values = {"flags": 0b100001, "lag_status": 1, "input_flags": 0b1010, "lateral_delay_s": 0.42, "lag_valid_blocks": 6,
                      "angular_velocity_calib": [0.0, 0.0, 0.1]}
            for name, fmt in LOCALIZATION_FIELDS:
                default = [0.0] * int(fmt[:-1]) if len(fmt) > 1 else 0
                value = values.get(name, default)
                fields.extend(value if isinstance(value, list) else [value])
            payload = LOCALIZATION_STATE.pack(*fields)
            header = IPC_HEADER.pack(IPC_MAGIC, 1, LOCALIZATION_STATE.size, 0, 2, boottime_ns(), len(payload), 0)
            path.write_bytes(header + payload)
            result = reader.snapshot({"steer_actuator_delay": 0.34})
            self.assertTrue(result["available"])
            self.assertEqual(result["initial_lag"], 0.34)
            state = result["state"]
            self.assertAlmostEqual(state["lateral_delay_s"], 0.42, places=5)
            self.assertEqual(state["lag_valid_blocks"], 6)
            self.assertAlmostEqual(state["angular_velocity_calib"][2], 0.1, places=5)
            self.assertTrue(state["flags"]["filter_valid"] and state["flags"]["lag_restored"])
            self.assertFalse(state["flags"]["inputs_ok"])
            self.assertTrue(state["input_flags"]["gyro_invalid"] and state["input_flags"]["camera_guarded"])
            self.assertFalse(state["input_flags"]["camera_invalid"])

    def test_page_has_lag_card(self):
        self.assertIn("lagd · 조향 지연", HTML)


if __name__ == "__main__":
    unittest.main()
