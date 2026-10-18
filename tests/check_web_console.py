#!/usr/bin/env python3
"""웹 콘솔(scripts/web_console) 검사. stdlib만 쓴다(fastapi 없이): 파라미터 저장, 공유 메모리 상태 읽기와 그
배치가 src/common/ipc_messages.h와 같은지, Panda 플래싱 조건, BEV 스트림, 페이지 파일과 서버 API가 맞는지."""
import ast
import asyncio
import gzip
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.web_console.backlight import duty_cycle_ns
from scripts.web_console.bev_stream import BEV_FRAME, BEV_FRAME_MAGIC, bev_frames, bev_layout
from scripts.web_console.calibration_control import (
    CALIBRATION_RESET_PATH,
    CALIBRATION_SOURCE_STATE,
    CALIBRATION_STATE,
    FLOW_CALIBRATION_STATE,
    CalibrationControl,
)
from scripts.web_console.learner_monitor import (
    LEARNER_FIELDS,
    LEARNER_FLAGS,
    LEARNER_STATE,
    LEARNER_TOPIC,
    LOCALIZATION_FIELDS,
    LOCALIZATION_FLAGS,
    LOCALIZATION_INPUT_FLAGS,
    LOCALIZATION_STATE,
    LOCALIZATION_TOPIC,
    LearnerMonitor,
    LearnerStateReader,
    LocalizationReader,
)
from scripts.web_console.lateral_nn_models import (
    MODEL_DIR,
    SELECTION_FILE,
    STATUS_PATH,
    LateralNnModels,
    read_choice,
)
from scripts.web_console.panda_update import (
    CONTROL_MAX_AGE_S,
    IMAGE_ERRORS,
    PANDA_APP_MAX_BYTES,
    PANDA_FLASHABLE_HW,
    PANDA_FLASH_REQUEST_PATH,
    PANDA_SIGNATURE_BYTES,
    PANDA_STATUS_PATH,
    PARKED_MAX_SPEED_KPH,
    REASONS,
    STEPS,
    PandaUpdate,
    panda_image_info,
)
from scripts.web_console.param_metadata import PARAM_GROUPS, PARAM_METADATA, RUNTIME_FACTS, VEHICLE_TAB
from scripts.web_console.param_store import ParamStore, group_paths, read_document
from scripts.web_console.refusal import Refusal
from scripts.web_console.process_status import (
    MANAGER_STATE_HEAD,
    MANAGER_STATE_TOPIC,
    MAX_PROCESSES,
    PROCESS_STATE,
    VERSION_MARKER,
    ProcessStatus,
    read_binary_version,
)
from scripts.web_console.shm_channel import IPC_HEADER, IPC_MAGIC, IPC_VERSION, boottime_ns
from scripts.web_console.state_layout import (
    CONTROL_STATE_AT,
    CONTROL_STATE_SIZE,
    CONTROL_STATE_TOPIC,
    GEAR_PARK,
    HUD_FLAG_BITS,
    MODEL_CALIBRATION_AT,
    MODEL_CALIBRATION_SOURCE_AT,
    MODEL_FLOW_CALIBRATION_AT,
    MODEL_STATE_AT,
    MODEL_STATE_SIZE,
    MODEL_STATE_TOPIC,
)
from scripts.web_console.static_assets import STATIC_DIR, StaticAssets

IPC_MESSAGES = ROOT / "src" / "common" / "ipc_messages.h"
WEB_CONSOLE = ROOT / "scripts" / "web_console"
I18N_DIR = STATIC_DIR / "i18n"
LANGUAGES = ("ko", "en")
HANGUL = re.compile(r"[\uac00-\ud7a3]")
# 메타데이터 card(전용 카드가 바꾸는 항목)마다 그 카드가 부르는 API
CARD_ROUTES = {"calibration": "/api/calibration/source"}


def catalog(lang: str) -> dict:
    return json.loads((I18N_DIR / f"{lang}.json").read_text(encoding="utf-8"))


def js_code(text: str) -> list:
    """JS 소스의 (줄 번호, 주석을 뺀 줄). 문자열 밖의 // 뒤와 한 줄 /* */를 지운다(이 저장소의 JS는 // 주석만 쓴다)."""
    lines = []
    for number, line in enumerate(text.splitlines(), 1):
        line = re.sub(r"/\*.*?\*/", "", line)
        for match in re.finditer(r"(?:^|(?<=\s))//", line):
            before = line[:match.start()]
            if all(before.count(quote) % 2 == 0 for quote in "\"'`"):
                line = before
                break
        lines.append((number, line))
    return lines


def page_modules() -> list:
    return sorted(path for path in STATIC_DIR.glob("*.js"))


def cpp(path: Path = IPC_MESSAGES) -> str:
    return path.read_text(encoding="utf-8")


def cpp_offsets(macro: str) -> dict:
    """EDGEPILOT_<macro>_AT(필드, 위치) 고정값들."""
    return {name: int(at) for name, at in re.findall(rf"EDGEPILOT_{macro}_AT\((\w+), (\d+)\);", cpp())}


def cpp_size(struct_name: str) -> int:
    return int(re.search(rf"sizeof\({struct_name}\) == (\d+)", cpp()).group(1))


def snake(name: str) -> str:
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def publish(path: Path, payload: bytes, seq: int = 2, stamp: int | None = None) -> None:
    """LatestChannel 하나를 쓴다(seq가 홀수면 쓰는 중)."""
    stamp = boottime_ns() if stamp is None else stamp
    path.write_bytes(IPC_HEADER.pack(IPC_MAGIC, IPC_VERSION, len(payload), 0, seq, stamp, len(payload), 0) + payload)


def pack_fields(layout, fields, values) -> bytes:
    flat = []
    for name, fmt in fields:
        value = values.get(name, [0] * int(fmt[:-1]) if len(fmt) > 1 else 0)
        flat.extend(value if isinstance(value, list) else [value])
    return layout.pack(*flat)


def control_payload(**values) -> bytes:
    """ControlState: 이름=값(정수 필드), gear는 부호 있는 정수, *_kph는 float."""
    payload = bytearray(CONTROL_STATE_SIZE)
    for name, value in values.items():
        fmt = "<f" if name.endswith("_kph") else "<i" if name == "gear" else "<I"
        struct.pack_into(fmt, payload, CONTROL_STATE_AT[name], value)
    return bytes(payload)


class FakeBacklight:
    def __init__(self):
        self.applied = []

    def apply(self, document):
        self.applied.append(dict(document))

    def status(self):
        latest = self.applied[-1] if self.applied else {}
        return {"available": True, "enabled": latest.get("enabled", True),
                "brightness_percent": latest.get("brightness_percent", 100), "mode": "test", "error": ""}


# ------------------------------------------------------------ 파라미터
class ParamStoreTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.paths = group_paths(self.root / "params")
        documents = {
            "steering": {"gain": 10, "enabled": True},
            "adaptive_cruise": {"following_time_s": 1.8},
            "recording": {"enabled": False},
            "display": {"enabled": True, "brightness_percent": 100},
        }
        self.paths["steering"].parent.mkdir()
        for group, document in documents.items():
            self.paths[group].write_text(json.dumps(document), encoding="utf-8")
        self.notified = []

        def notify(name):
            self.notified.append(name)
            return [123]

        self.notify = notify
        self.store = ParamStore(self.paths, notify=notify)

    def tearDown(self):
        self.temporary.cleanup()

    def test_atomic_single_value_update_signals_the_reader(self):
        result = self.store.update("steering", {"gain": 12})
        self.assertEqual(result["notified"], [123])
        self.assertEqual(self.notified, ["controlsd"])
        self.assertEqual(self.store.read_group("steering"), {"gain": 12, "enabled": True})
        self.assertEqual(list(self.paths["steering"].parent.glob("*.tmp")), [])

    def test_card_managed_setting_is_changed_only_through_its_card(self):
        """전용 카드가 바꾸는 항목(메타데이터 card)은 일반 경로로 받지 않는다. 카드의 API가 받을 조건을 건너뛰지 않게."""
        self.paths["display"].write_text(
            json.dumps({"enabled": True, "brightness_percent": 100, "calibration_use_flow": False}), encoding="utf-8")
        with self.assertRaises(ValueError):
            self.store.update("display", {"calibration_use_flow": True})
        self.assertIs(self.store.read_group("display")["calibration_use_flow"], False)
        self.store.update("display", {"calibration_use_flow": True}, allow_managed=True)
        self.assertIs(self.store.read_group("display")["calibration_use_flow"], True)

    def test_unknown_parameter_is_rejected(self):
        with self.assertRaises(KeyError):
            self.store.update("adaptive_cruise", {"unknown": 1})
        with self.assertRaises(KeyError):
            self.store.update("nonexistent", {"following_time_s": 1})
        with self.assertRaises(ValueError):
            self.store.update("adaptive_cruise", {})
        self.assertEqual(self.notified, [])
        self.assertEqual(self.store.read_group("adaptive_cruise"), {"following_time_s": 1.8})

    def test_files_read_by_their_own_process_signal_nobody(self):
        """recordd·overlayd·modeld는 파일을 스스로 다시 읽는다."""
        self.assertEqual(self.store.update("recording", {"enabled": True})["notified"], [])
        self.assertEqual(self.notified, [])

    def test_display_update_applies_hardware_without_signaling(self):
        backlight = FakeBacklight()
        store = ParamStore(self.paths, notify=self.notify, backlight=backlight)
        self.assertEqual(backlight.applied, [{"enabled": True, "brightness_percent": 100}], "applied at start")
        backlight.applied.clear()
        result = store.update("display", {"brightness_percent": 35})
        self.assertEqual(result["notified"], [])
        self.assertEqual(backlight.applied, [{"enabled": True, "brightness_percent": 35}])
        self.assertEqual(store.read_group("display")["brightness_percent"], 35)

    def test_display_update_without_backlight_is_refused(self):
        with self.assertRaises(ValueError):
            self.store.update("display", {"brightness_percent": 35})
        self.assertEqual(self.store.read_group("display")["brightness_percent"], 100)

    def test_alert_volume_update_leaves_backlight_alone(self):
        self.paths["display"].write_text(
            json.dumps({"enabled": True, "brightness_percent": 80, "alert_volume_percent": 70}), encoding="utf-8")
        backlight = FakeBacklight()
        store = ParamStore(self.paths, notify=self.notify, backlight=backlight)
        backlight.applied.clear()
        result = store.update("display", {"alert_volume_percent": 30})
        self.assertEqual(result["params"]["alert_volume_percent"], 30)
        self.assertEqual(backlight.applied, [], "volume is read by overlayd, not the backlight")
        store.update("display", {"brightness_percent": 60})
        self.assertEqual(backlight.applied[-1]["brightness_percent"], 60)
        self.assertEqual(json.loads(self.paths["display"].read_text())["alert_volume_percent"], 30)

    def test_display_pwm_duty_scales_by_board_maximum(self):
        self.assertEqual(duty_cycle_ns(100), 95_000)
        self.assertEqual(duty_cycle_ns(51), 48_450)
        self.assertEqual(duty_cycle_ns(1), 950)
        self.assertEqual(duty_cycle_ns(0), 0)
        self.assertEqual(duty_cycle_ns(100, max_percent=100), 100_000)

    def test_runtime_keys_follow_the_defaults(self):
        """빠진 키는 기본값으로 채우고 기본값에서 사라진 키는 지운다. 남은 키의 튜닝은 그대로."""
        defaults = self.root / "adaptive.defaults.json"
        defaults.write_text(json.dumps({"following_time_s": 1.8, "deceleration_rate_kph_per_s": 1.5}),
                            encoding="utf-8")
        self.paths["adaptive_cruise"].write_text(json.dumps({"following_time_s": 2.2, "retired": 1}),
                                                 encoding="utf-8")
        store = ParamStore(self.paths, {"adaptive_cruise": defaults}, notify=self.notify)
        self.assertEqual(store.read_group("adaptive_cruise"),
                         {"following_time_s": 2.2, "deceleration_rate_kph_per_s": 1.5})
        self.assertEqual(store.defaults(), {"adaptive_cruise": {"following_time_s": 1.8,
                                                                "deceleration_rate_kph_per_s": 1.5}})

    def test_defaults_sit_next_to_the_params_directory(self):
        """EDGEPILOT_PARAMS_DIR(없으면 params/)와 그 옆의 <이름>.defaults/(업로드 스크립트가 채운다)."""
        defaults = group_paths(self.root / "params.defaults")
        defaults["adaptive_cruise"].parent.mkdir()
        defaults["adaptive_cruise"].write_text(json.dumps({"following_time_s": 1.5, "added": True}), encoding="utf-8")
        with mock.patch.dict(os.environ, {"EDGEPILOT_PARAMS_DIR": str(self.root / "params")}):
            store = ParamStore(notify=self.notify)
        self.assertEqual(store.paths, self.paths)
        self.assertEqual(store.read_group("adaptive_cruise"), {"following_time_s": 1.8, "added": True})
        self.assertEqual(store.snapshot()["defaults"],
                         {"adaptive_cruise": {"following_time_s": 1.5, "added": True}})

    def test_snapshot_has_what_the_page_reads(self):
        snapshot = self.store.snapshot()
        self.assertEqual(set(snapshot), {"groups", "vehicle_tab", "params", "defaults", "metadata", "paths"})
        self.assertEqual(list(snapshot["groups"]), list(PARAM_GROUPS), "tab order")
        for group, spec in snapshot["groups"].items():
            self.assertEqual(set(spec), {"sections", "notify"}, group)
        self.assertEqual(snapshot["paths"]["steering"], str(self.paths["steering"]))


class ParamMetadataTest(unittest.TestCase):
    def test_repository_params_have_complete_ui_metadata(self):
        """항목마다 섹션이 있고 두 언어 모두 이름·설명·효과 문구가 있으며, 섹션은 그 항목이 나오는 탭(파일 그룹,
        tab="vehicle"이면 차량 특성 탭)의 섹션이고, 빈 섹션이 없다."""
        catalogs = {lang: catalog(lang) for lang in LANGUAGES}
        tabs = {}
        for group, spec in PARAM_GROUPS.items():
            params = json.loads((ROOT / "params" / spec["file"]).read_text(encoding="utf-8"))
            metadata = PARAM_METADATA[group]
            self.assertEqual(set(params), set(metadata), group)
            tabs.setdefault(group, (spec["sections"], set()))
            for key, value in params.items():
                meta = metadata[key]
                self.assertTrue(meta.get("section"), f"{group}.{key}.section")
                fields = ["label", "description", "increase", "decrease"] + (["learned_role"] if meta.get("learned") else [])
                for lang, texts in catalogs.items():
                    for field in fields:
                        self.assertTrue(texts.get(f"param.{group}.{key}.{field}"), f"{lang}: param.{group}.{key}.{field}")
                    if not meta.get("learned"):
                        self.assertNotIn(f"param.{group}.{key}.learned_role", texts, f"{lang}: {group}.{key}")
                    if meta.get("caution"):
                        self.assertTrue(texts.get(f"caution.{meta['caution']}"), f"{lang}: caution.{meta['caution']}")
                self.assertIn(meta.get("tab", group), {group, "vehicle"}, f"{group}.{key}")
                if meta.get("card"):
                    # 차량 특성 탭의 전용 카드가 자기 API로 바꾸는 항목: 섹션은 파일 그룹의 것이지만 그 탭에 카드가
                    # 없으니 섹션을 채우지 않는다
                    self.assertEqual(meta.get("tab"), "vehicle", f"{group}.{key}")
                    self.assertIn(meta["card"], CARD_ROUTES, f"{group}.{key}")
                    self.assertIn(meta["section"], spec["sections"], f"{group}.{key}")
                    page = (STATIC_DIR / "vehicle_view.js").read_text(encoding="utf-8")
                    self.assertIn(f'api.post("{CARD_ROUTES[meta["card"]]}"', page, f"{group}.{key}")
                else:
                    if meta.get("tab") == "vehicle":
                        self.assertEqual(group, "steering", f"{group}.{key}: the vehicle tab edits steering.json")
                    sections, used = tabs.setdefault(meta.get("tab", group), (VEHICLE_TAB["sections"], set()))
                    self.assertIn(meta["section"], sections, f"{group}.{key}")
                    used.add(meta["section"])
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    for field in ("step", "min", "max"):
                        self.assertIsInstance(meta.get(field), (int, float), f"{group}.{key}.{field}")
                    self.assertLessEqual(meta["min"], value, f"{group}.{key}")
                    self.assertLessEqual(value, meta["max"], f"{group}.{key}")
        for tab, (sections, used) in tabs.items():
            self.assertEqual([name for name in sections if name not in used], [], f"{tab}: empty sections")

    def test_conditions_name_real_options(self):
        """다른 항목에 따른 안내(unused·role, param_metadata.when): 조건의 항목이 있고 자기 자신이 아니며, 값은 그
        항목이 가질 수 있는 것(켜고 끄는 항목은 bool, 숫자 항목은 범위 안의 수, 런타임 사실은 bool)이다. also도 같고
        문구가 없다. 문구가 두 언어에 다 있는지는 I18nCatalogTest가 본다."""
        files = {group: read_document(path) for group, path in group_paths(ROOT / "params").items()}

        def check(condition, group, key, name):
            self.assertEqual(set(condition) - {"group", "also"}, {"key", "value", "text"}, name)
            other_group, other_key, value = condition.get("group", group), condition["key"], condition["value"]
            if other_group == "runtime":
                self.assertIn(other_key, RUNTIME_FACTS, name)
                self.assertIsInstance(value, bool, name)
            else:
                self.assertIn(other_key, files[other_group], name)
                self.assertNotEqual((other_group, other_key), (group, key), name)
                other = PARAM_METADATA[other_group][other_key]
                if isinstance(files[other_group][other_key], bool):
                    self.assertIsInstance(value, bool, name)
                else:
                    self.assertIsInstance(value, (int, float), name)
                    self.assertNotIsInstance(value, bool, name)
                    self.assertTrue(other["min"] <= value <= other["max"], name)
            if "also" in condition:
                self.assertEqual(condition["also"]["text"], "", name)
                check(condition["also"], group, key, f"{name} also")

        seen = 0
        for group, items in PARAM_METADATA.items():
            for key, meta in items.items():
                for kind in ("unused", "role"):
                    for condition in meta.get(kind, []):
                        seen += 1
                        self.assertTrue(condition["text"], f"{group}.{key} {kind}")
                        check(condition, group, key, f"{group}.{key} {kind} {condition}")
        self.assertGreater(seen, 0)

    def test_vehicle_tab_matches_the_page(self):
        """차량 특성 탭: 학습기 섹션마다 스위치가 하나 있고, 나머지 항목은 학습값을 읽는 법(vehicle_view.js LIVE)과
        자동 학습 중의 쓰임(learned, learned_role 문구)이 있다. 학습기 이름은 페이지의 LEARNERS와 같다."""
        steering = PARAM_METADATA["steering"]
        page = (STATIC_DIR / "vehicle_view.js").read_text(encoding="utf-8")
        live = set(re.findall(r"^  (\w+): \{$", re.search(r"const LIVE = \{\n(.*?)\n\};", page, re.S).group(1), re.M))
        learners = set(re.findall(r"^  (\w+): \{$", re.search(r"const LEARNERS = \{\n(.*?)\n\};", page, re.S).group(1), re.M))
        learned = set()
        for name, spec in VEHICLE_TAB["learners"].items():
            self.assertIn(name, VEHICLE_TAB["sections"])
            switch = steering[spec["switch"]]
            self.assertEqual((switch.get("tab"), switch["section"]), ("vehicle", name), spec["switch"])
            keys = {key for key, meta in steering.items() if meta.get("tab") == "vehicle" and meta["section"] == name}
            for key in keys - {spec["switch"]}:
                self.assertTrue(steering[key].get("learned"), key)
                learned.add(key)
        self.assertEqual(live, learned)
        self.assertEqual(learners, {spec["learner"] for spec in VEHICLE_TAB["learners"].values()})
        self.assertFalse([key for key, meta in steering.items() if meta.get("learned") and key not in learned])

    def test_ui_ranges_match_runtime_clamps(self):
        """로더는 Json*Field 표대로 값을 자른다. 편집기도 같은 min/max를 보여야 하고, 아니면 런타임이 말없이
        바꾸는 값을 받아들인다."""
        row = re.compile(r'\{"(\w+)",\s*(-?[\d.]+)f?,\s*(-?[\d.]+)f?,\s*&\w+::\w+\}')
        tables = {
            "steering": ("src/controls/control_params.cc", ("kSteeringInts", "kSteeringFloats")),
            "adaptive_cruise": ("src/controls/adaptive_cruise.cc", ("kAdaptiveFloats",)),
            "recording": ("src/recording/recordd.cc", ("kRecordingInts",)),
            # display는 백라이트(backlight.py) 항목도 있어 런타임이 읽는 키만 대조한다
            "display": ("src/common/device_settings.h", ("kDeviceSettingsFloats",)),
        }
        for group, (source, names) in tables.items():
            text = (ROOT / source).read_text(encoding="utf-8")
            runtime = {}
            for name in names:
                body = re.search(name + r"\[\] = \{(.*?)\n\};", text, re.S)
                self.assertIsNotNone(body, f"{source}: {name}")
                runtime.update({key: (float(low), float(high)) for key, low, high in row.findall(body.group(1))})
            self.assertTrue(runtime, f"{source}: no rows parsed")
            ui = {key: meta for key, meta in PARAM_METADATA[group].items() if "min" in meta}
            if group == "display":
                ui = {key: meta for key, meta in ui.items() if key in runtime}
            self.assertEqual(set(runtime), set(ui), group)
            for key, (low, high) in runtime.items():
                self.assertEqual((float(ui[key]["min"]), float(ui[key]["max"])), (low, high), f"{group}.{key}")

    def test_ui_text_names_no_particular_car(self):
        """다른 차도 붙일 수 있게 화면 문구는 차종을 말하지 않는다. 한글이 바로 붙는 "K7은"도 잡는다."""
        texts = [json.dumps(PARAM_METADATA, ensure_ascii=False), json.dumps(PARAM_GROUPS, ensure_ascii=False)]
        texts += [(I18N_DIR / f"{lang}.json").read_text(encoding="utf-8") for lang in LANGUAGES]
        texts += [path.read_text(encoding="utf-8") for path in STATIC_DIR.glob("*.html")]
        texts += [path.read_text(encoding="utf-8") for path in STATIC_DIR.glob("*_view.js")]
        texts += [(STATIC_DIR / name).read_text(encoding="utf-8")
                  for name in ("console.js", "panda_card.js", "lateral_nn_card.js")]
        for text in texts:
            self.assertNotRegex(text, r"(?i)(?<![0-9a-z])k7(?![0-9a-z])|그랜저|grandeur")
        self.assertEqual([path.name for path in STATIC_DIR.rglob("*") if "k7" in path.name.lower()], [])



class I18nCatalogTest(unittest.TestCase):
    """화면 글자는 언어마다 static/i18n/<언어>.json에만 있다: 두 언어의 키와 자리 값이 같고, 페이지와 서버가 쓰는 키가
    다 있으며 안 쓰는 키가 없고, 코드에는 화면에 나갈 한글이 남지 않는다."""

    def setUp(self):
        self.catalogs = {lang: catalog(lang) for lang in LANGUAGES}

    def test_languages_have_the_same_keys_and_placeholders(self):
        placeholders = lambda text: set(re.findall(r"\{(\w+)\}", text))
        ko, en = self.catalogs["ko"], self.catalogs["en"]
        self.assertEqual(sorted(set(ko) ^ set(en)), [])
        for key in ko:
            forms = {lang: texts[key] if isinstance(texts[key], dict) else {"other": texts[key]}
                     for lang, texts in self.catalogs.items()}
            for lang, by_count in forms.items():
                self.assertIn("other", by_count, f"{lang}: {key}")
                self.assertLessEqual(set(by_count), {"zero", "one", "two", "few", "many", "other"}, f"{lang}: {key}")
                for text in by_count.values():
                    self.assertIsInstance(text, str, f"{lang}: {key}")
                    self.assertTrue(text.strip(), f"{lang}: {key}")
                    if len(by_count) > 1:
                        self.assertIn("count", placeholders(text), f"{lang}: {key} plural without {{count}}")
            names = {lang: set().union(*(placeholders(text) for text in by_count.values())) for lang, by_count in forms.items()}
            self.assertEqual(names["ko"], names["en"], key)
            self.assertFalse(HANGUL.search(json.dumps(en[key], ensure_ascii=False)), f"en: {key}")

    def test_page_and_server_keys_exist_and_none_is_unused(self):
        """페이지의 "이름공간.키" 글자와 index.html의 data-i18n*, 서버의 거부 코드가 다 있어야 한다. 반대로 문구마다
        그것을 쓰는 곳(글자 그대로, `접두사.${...}` 템플릿, 메타데이터에서 나오는 키, 서버 코드)이 있어야 한다."""
        keys = set(self.catalogs["en"])
        spaces = {key.split(".")[0] for key in keys}
        used, prefixes = set(), set()
        for module in page_modules():
            code = "\n".join(line for _, line in js_code(module.read_text(encoding="utf-8")))
            # el("section.section", ...)의 "태그.클래스"는 키가 아니다
            for key in re.findall(r"(?<!el\()[\"'`]([a-z_]+(?:\.[A-Za-z0-9_]+)+)[\"'`]", code):
                if key.split(".")[0] not in spaces:
                    continue
                if key not in keys and any(other.startswith(f"{key}.") for other in keys):
                    prefixes.add(f"{key}.")  # coded("panda.reason", 코드)처럼 접두사만 쓰는 글자
                    continue
                self.assertIn(key, keys, f"{module.name}: {key}")
                used.add(key)
            prefixes.update(re.findall(r"`([a-z_]+(?:\.[A-Za-z0-9_]+)*\.)\$\{", code))
        page = (STATIC_DIR / "index.html").read_text(encoding="utf-8")
        for key in re.findall(r'data-i18n(?:-[a-z-]+)?="([^"]+)"', page):
            self.assertIn(key, keys, f"index.html: {key}")
            used.add(key)
        # 메타데이터에서 나오는 키(param_state.js가 만든다)
        for group, items in PARAM_METADATA.items():
            for key, meta in items.items():
                fields = ["label", "description", "increase", "decrease"] + (["learned_role"] if meta.get("learned") else [])
                used.update(f"param.{group}.{key}.{field}" for field in fields)
                used.add(f"section.{meta['section']}")
                if meta.get("caution"):
                    used.add(f"caution.{meta['caution']}")
                for kind in ("unused", "role"):
                    used.update(f"{kind}.{condition['text']}" for condition in meta.get(kind, []))
                if meta.get("unit") and f"unit.{meta['unit']}" in keys:
                    used.add(f"unit.{meta['unit']}")
        for group, spec in PARAM_GROUPS.items():
            used.add(f"group.{group}.label")
            used.update(f"section.{name}" for name in spec["sections"])
            if f"group.{group}.note" in keys:
                used.add(f"group.{group}.note")
        used.update(f"section.{name}" for name in VEHICLE_TAB["sections"])
        used.add("section.other")
        # 서버의 코드: Panda 사유·단계·이미지 문제와 거부(Refusal)
        used.update(f"panda.reason.{code}" for code in REASONS)
        used.update(f"panda.step.{code}" for code in STEPS)
        used.update(f"panda.image.{code}" for code in IMAGE_ERRORS if code)
        for module in WEB_CONSOLE.glob("*.py"):
            text = module.read_text(encoding="utf-8")
            for code in re.findall(r'Refusal\(\s*"([^"]+)"', text):
                self.assertIn(code, keys, f"{module.name}: {code}")
                used.add(code)
            for prefix in re.findall(r'Refusal\(\s*f"([a-z_.]+)\{', text):
                self.assertTrue(any(key.startswith(prefix) for key in keys), f"{module.name}: {prefix}")
        missing = sorted(used - keys)
        self.assertEqual(missing, [], "keys used but not in the catalogs")
        unused = sorted(key for key in keys - used if not any(key.startswith(prefix) for prefix in prefixes))
        self.assertEqual(unused, [], "catalog keys nothing uses")

    def test_nothing_shadows_the_translation_function(self):
        """t를 가져다 쓰는 모듈은 t라는 이름을 따로 묶지 않는다(지역 변수, 매개변수). 가리면 t("...")가 숫자를 부르다
        멈춘다: BEV의 frame()이 모델 시각을 t에 두어 상태 줄이 "t is not a function"으로 끊겼다(2026-10-14)."""
        binding = re.compile(r"\b(?:let|const|var)\s+t\b|(?<![\w.$])t\s*=>|\bcatch\s*\(\s*t\s*\)")
        params = re.compile(r"(?<![\w.$])(\w*)\s*\(([^()]*)\)\s*(?:=>|\{)")
        found = []
        for module in page_modules():
            text = module.read_text(encoding="utf-8")
            if not re.search(r'import\s*\{[^}]*\bt\b[^}]*\}\s*from\s*"\./i18n\.js"', text):
                continue
            for number, line in js_code(text):
                if binding.search(line):
                    found.append(f"{module.name}:{number}")
                    continue
                for name, args in params.findall(line):
                    if name in ("if", "while", "for", "switch", "return"):
                        continue
                    names = [re.split(r"[\s=]", arg.strip().lstrip("{[").rstrip("}]"))[0] for arg in args.split(",")]
                    if "t" in names:
                        found.append(f"{module.name}:{number}")
        self.assertEqual(found, [])

    def test_no_screen_text_left_in_code(self):
        """한글은 주석과 문구 묶음에만 있다: 페이지 모듈과 index.html의 코드, 서버 모듈의 문자열(독스트링 빼고).
        그 언어로 쓴 언어 이름처럼 일부러 둔 줄은 i18n-allow 주석을 단다."""
        found = []
        for module in page_modules():
            text = module.read_text(encoding="utf-8")
            raw = text.splitlines()
            found += [f"{module.name}:{number}" for number, line in js_code(text)
                      if HANGUL.search(line) and "i18n-allow" not in raw[number - 1]]
        page = re.sub(r"<!--.*?-->", "", (STATIC_DIR / "index.html").read_text(encoding="utf-8"), flags=re.S)
        found += ["index.html" for _ in HANGUL.findall(page)][:1]
        for module in WEB_CONSOLE.glob("*.py"):
            tree = ast.parse(module.read_text(encoding="utf-8"))
            docstrings = {id(node.body[0].value) for node in ast.walk(tree)
                          if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                          and node.body and isinstance(node.body[0], ast.Expr)
                          and isinstance(node.body[0].value, ast.Constant)}
            found += [f"{module.name}:{node.lineno}" for node in ast.walk(tree)
                      if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docstrings
                      and HANGUL.search(node.value)]
        self.assertEqual(found, [])

# ------------------------------------------------------------ 학습 상태
def learner_payload(**values):
    return pack_fields(LEARNER_STATE, LEARNER_FIELDS, values)


class LearnerStateTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary.name) / "edgepilot_learner_state"

    def tearDown(self):
        self.temporary.cleanup()

    def publish(self, seq, **values):
        publish(self.path, learner_payload(**values), seq=seq, stamp=123)

    def test_layout_matches_cpp(self):
        """ipc_messages.h의 크기·offsetof 고정값·플래그 비트와 Python 필드 배치가 같아야 한다."""
        self.assertEqual(LEARNER_STATE.size, cpp_size("LearnerState"))
        offsets, offset = {}, 0
        for name, fmt in LEARNER_FIELDS:
            offsets[name] = offset
            offset += struct.calcsize("<" + fmt)
        asserted = cpp_offsets("LEARNER_STATE")
        self.assertGreaterEqual(len(asserted), 8)
        for name, expected in asserted.items():
            self.assertEqual(offsets[name], expected, name)
        bits = re.findall(r"constexpr uint32_t kLearner(\w+) = 1U << (\d+);", cpp())
        self.assertEqual([snake(name) for name, _ in bits], list(LEARNER_FLAGS))
        self.assertEqual([int(bit) for _, bit in bits], list(range(len(LEARNER_FLAGS))))
        self.assertIn(f'kLearnerStateTopic[] = "{LEARNER_TOPIC}";', cpp())

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

    def test_runtime_facts_follow_a_fresh_state(self):
        """파라미터 안내가 보는 런타임 사실: controlsd가 학습한 토크 모델을 읽었는지(플래그). 상태가 없거나 낡으면
        None이다(그때는 어느 조건도 참이 아니다)."""
        monitor = LearnerMonitor(LearnerStateReader(str(self.path)))
        self.assertEqual(monitor.runtime_facts(), {"lateral_nn_loaded": None})
        self.assertEqual(set(monitor.runtime_facts()), set(RUNTIME_FACTS))
        loaded = 1 << LEARNER_FLAGS.index("lateral_nn_loaded")
        publish(self.path, learner_payload(flags=loaded), seq=2)
        self.assertEqual(monitor.runtime_facts(), {"lateral_nn_loaded": True})
        publish(self.path, learner_payload(flags=0), seq=4)
        self.assertEqual(monitor.runtime_facts(), {"lateral_nn_loaded": False})
        publish(self.path, learner_payload(flags=loaded), seq=6, stamp=123)  # 오래전 상태
        self.assertEqual(monitor.runtime_facts(), {"lateral_nn_loaded": None})

    def test_monitor_keeps_one_trend_row_per_publish(self):
        monitor = LearnerMonitor(LearnerStateReader(str(self.path)))
        self.assertFalse(monitor.status()["available"])
        self.publish(2, timestamp_ns=1_000_000_000, lat_accel_factor=4.44)
        monitor.sample()
        monitor.sample()
        self.publish(4, timestamp_ns=2_000_000_000, lat_accel_factor=4.40)
        status = monitor.status()
        self.assertTrue(status["available"])
        self.assertEqual([row[0] for row in monitor.trend()["rows"]], [1.0, 2.0])
        self.assertAlmostEqual(status["trend_row"][7], 4.40, places=5)
        self.publish(6, timestamp_ns=500_000_000)  # 시각이 거꾸로: 다른 부팅
        monitor.sample()
        self.assertEqual([row[0] for row in monitor.trend()["rows"]], [0.5])


class LocalizationStateTest(unittest.TestCase):
    def test_layout_matches_cpp(self):
        self.assertEqual(LOCALIZATION_STATE.size, cpp_size("LocalizationState"))
        flags = re.findall(r"constexpr uint32_t kLocalization(\w+) = 1U << (\d+);", cpp())
        state_flags = [int(bit) for name, bit in flags if not name.startswith(("Invalid", "CameraGuarded"))]
        input_flags = [int(bit) for name, bit in flags if name.startswith(("Invalid", "CameraGuarded"))]
        self.assertEqual(state_flags, list(range(len(LOCALIZATION_FLAGS))))
        self.assertEqual(input_flags, list(range(len(LOCALIZATION_INPUT_FLAGS))))
        self.assertIn(f'kLocalizationStateTopic[] = "{LOCALIZATION_TOPIC}";', cpp())

    def test_reader_decodes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "edgepilot_localization"
            reader = LocalizationReader(str(path))
            self.assertFalse(reader.status()["available"])
            publish(path, pack_fields(LOCALIZATION_STATE, LOCALIZATION_FIELDS, {
                "flags": 0b100001, "lag_status": 1, "input_flags": 0b1010, "lateral_delay_s": 0.42,
                "lag_valid_blocks": 6, "angular_velocity_calib": [0.0, 0.0, 0.1]}))
            result = reader.status()
            self.assertTrue(result["available"])
            state = result["state"]
            self.assertAlmostEqual(state["lateral_delay_s"], 0.42, places=5)
            self.assertEqual(state["lag_valid_blocks"], 6)
            self.assertAlmostEqual(state["angular_velocity_calib"][2], 0.1, places=5)
            self.assertTrue(state["flags"]["filter_valid"] and state["flags"]["lag_restored"])
            self.assertFalse(state["flags"]["inputs_ok"])
            self.assertTrue(state["input_flags"]["gyro_invalid"] and state["input_flags"]["camera_guarded"])
            self.assertFalse(state["input_flags"]["camera_invalid"])


# ------------------------------------------------------------ 카메라 보정
class CalibrationControlTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.model = root / "edgepilot_model_state"
        self.control = root / "edgepilot_control_state"
        self.request = root / "edgepilot_calibration_reset"
        paths = group_paths(root / "params")
        paths["display"].parent.mkdir()
        paths["display"].write_text(json.dumps({"enabled": True, "calibration_use_flow": False}), encoding="utf-8")
        self.store = ParamStore(paths, notify=lambda name: [])
        self.calibration = CalibrationControl(self.store, str(self.model), str(self.control), str(self.request))

    def tearDown(self):
        self.temporary.cleanup()

    def publish_model(self, status, blocks, rpy_deg, flow=None, source=None):
        payload = bytearray(MODEL_STATE_SIZE)
        rad = [x * 3.141592653589793 / 180 for x in rpy_deg]
        CALIBRATION_STATE.pack_into(payload, MODEL_CALIBRATION_AT, status, blocks, *rad, 0.1, 0.2, 0.3)
        if flow is not None:
            FLOW_CALIBRATION_STATE.pack_into(payload, MODEL_FLOW_CALIBRATION_AT, *flow)
        if source is not None:
            CALIBRATION_SOURCE_STATE.pack_into(payload, MODEL_CALIBRATION_SOURCE_AT, *source)
        publish(self.model, bytes(payload))

    def publish_control(self, engaged, stamp=None):
        publish(self.control, control_payload(engaged=engaged, vehicle_fresh=1), stamp=stamp)

    def test_layout_and_request_path_match_cpp(self):
        offset = re.search(r"offsetof\(ModelState, calibration\) == (\d+)", cpp())
        self.assertEqual(int(offset.group(1)), MODEL_CALIBRATION_AT)
        self.assertEqual(CALIBRATION_STATE.size, 32)
        for field, at in (("flow_calibration", MODEL_FLOW_CALIBRATION_AT), ("calibration_source", MODEL_CALIBRATION_SOURCE_AT)):
            self.assertEqual(int(re.search(rf"offsetof\(ModelState, {field}\) == (\d+)", cpp()).group(1)), at, field)
        self.assertEqual(FLOW_CALIBRATION_STATE.size, int(re.search(r"sizeof\(FlowCalibrationState\) == (\d+)", cpp()).group(1)))
        self.assertEqual(CALIBRATION_SOURCE_STATE.size,
                         int(re.search(r"sizeof\(CalibrationSourceState\) == (\d+)", cpp()).group(1)))
        for name, bit in (("FlowUsable", 0), ("Pending", 1), ("Ramping", 2)):
            self.assertIn(f"kCalibrationSource{name} = 1u << {bit};", cpp())
        service = cpp(ROOT / "src" / "model" / "calibration_service.cc")
        self.assertRegex(service, rf'"EDGEPILOT_CALIBRATION_RESET_PATH",\s*"{re.escape(CALIBRATION_RESET_PATH)}"')

    def test_status_decodes_model_state(self):
        self.assertFalse(self.calibration.status()["available"])
        self.publish_model(1, 7, [0.0, 0.29, -0.18])
        status = self.calibration.status()
        self.assertTrue(status["available"])
        self.assertEqual((status["status"], status["valid_blocks"]), ("calibrated", 7))
        self.assertAlmostEqual(status["rpy_deg"][1], 0.29, places=3)
        self.assertAlmostEqual(status["rpy_deg"][2], -0.18, places=3)

    def test_status_decodes_flow_and_source(self):
        rad = 3.141592653589793 / 180
        flow = (1234, 2, 240, 14, -0.15 * rad, -0.03 * rad, 0.04 * rad, 0.06 * rad, -0.2 * rad, 0.1 * rad, 0.002 * rad,
                297, 296, 1, 6.1, 12, 9)
        source = (1, 1, 1 | 4, 1, 50, 0.0, -0.06 * rad, -0.30 * rad)
        self.publish_model(1, 50, [0.0, -0.15, -0.03], flow=flow, source=source)
        status = self.calibration.status()
        self.assertEqual((status["source"], status["flow_usable"], status["ramping"]), ("flow", True, True))
        self.assertFalse(status["switch_pending"])
        f = status["flow"]
        self.assertEqual((f["status"], f["estimate_blocks"], f["windows"], f["mount_changes"], f["rejected_windows"]),
                         ("estimate", 12, 240, 1, 9))
        self.assertAlmostEqual(f["yaw_deg"], -0.03, places=3)
        self.assertAlmostEqual(f["sd_deg"][1], 0.06, places=3)
        self.assertAlmostEqual(f["window"]["parallax_px"], 6.1, places=2)
        self.assertEqual(f["window"]["tracks"], 297)
        self.assertEqual((status["pose"]["status"], status["pose"]["valid_blocks"]), ("calibrated", 50))
        self.assertAlmostEqual(status["pose"]["rpy_deg"][2], -0.30, places=3)

    def test_reset_is_refused_while_engaged(self):
        self.publish_control(engaged=1)
        with self.assertRaises(PermissionError):
            self.calibration.request_reset()
        self.assertFalse(self.request.exists())
        self.publish_control(engaged=0)
        self.assertTrue(self.calibration.request_reset()["reset_pending"])
        self.assertTrue(self.request.exists())

    def test_source_is_changed_only_while_disengaged(self):
        """보정 출처는 이 카드만 바꾸고(파라미터 API는 받지 않는다) 결합 중에는 받지 않는다. 상태의 use_flow는 modeld가
        다시 읽기 전에도 고른 값을 보이게 설정 파일에서 읽는다."""
        self.publish_control(engaged=1)
        with self.assertRaises(PermissionError):
            self.calibration.set_source(True)
        self.assertIs(self.store.read_group("display")["calibration_use_flow"], False)
        self.publish_control(engaged=0)
        self.assertTrue(self.calibration.set_source(True)["use_flow"])
        self.assertIs(self.store.read_group("display")["calibration_use_flow"], True)
        self.assertFalse(self.calibration.set_source(False)["use_flow"])

    def test_stale_or_missing_controlsd_does_not_block(self):
        self.assertFalse(self.calibration.engaged())
        self.publish_control(engaged=1, stamp=boottime_ns() - 5_000_000_000)
        self.assertFalse(self.calibration.engaged())


# ------------------------------------------------------------ Panda 펌웨어
def panda_image(version="EDGE-f9907afb-DEBUG", body=1024):
    """sign.py의 꼴: [본문 길이][... 버전 ...][VERS][2] 뒤에 서명 128바이트."""
    data = bytearray(b"\x11" * body)
    struct.pack_into("<I", data, 0, body)
    data[63:64 + len(version) + 1] = b"\0" + version.encode() + b"\0"
    data[body - 8:body] = b"VERS" + struct.pack("<I", 2)
    return bytes(data) + b"\xab" * PANDA_SIGNATURE_BYTES


class LateralNnModelsTest(unittest.TestCase):
    """학습 토크 모델 카드: 레포의 모델 파일 이름, 고르기 파일 읽기·쓰기, 거부, controlsd 상태, C++과 같은 경로·규칙."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.models_dir = root / "models"
        shutil.copytree(ROOT / MODEL_DIR, self.models_dir)
        self.selection = root / "params" / SELECTION_FILE
        self.default = root / "params.defaults" / SELECTION_FILE
        self.status_file = root / "status.json"
        self.models = LateralNnModels(self.models_dir, self.selection, self.default, str(self.status_file))

    def tearDown(self):
        self.temporary.cleanup()

    def test_paths_and_name_rule_match_cpp(self):
        path_cc = cpp(ROOT / "src" / "common" / "lateral_nn_path.h")
        self.assertIn(f'kLateralNnModelDir = "{MODEL_DIR.as_posix()}";', path_cc)
        self.assertIn(f'kLateralNnSelectionFile = "{SELECTION_FILE}";', path_cc)
        select_cc = cpp(ROOT / "src" / "controls" / "lateral_nn_select.cc")
        self.assertIn(f'"EDGEPILOT_LATERAL_NN_STATUS", "{STATUS_PATH}"', select_cc)
        self.assertEqual(json.loads((ROOT / "params" / SELECTION_FILE).read_text())["model"], "k7_hev_v3")
        for name, ok in (("k7_hev_v3", True), ("Car-2.1", True), (".hidden", False), ("../x", False),
                         ("a" * 64, True), ("a" * 65, False)):
            self.selection.parent.mkdir(exist_ok=True)
            self.selection.write_text(json.dumps({"model": name}))
            self.assertEqual(read_choice(self.selection), (name, "") if ok else ("", "bad_name"), name)

    def test_lists_the_model_files_by_name(self):
        self.assertEqual(self.models.models(), ["k7_hev_v1", "k7_hev_v2", "k7_hev_v3"])
        (self.models_dir / "k7_hev_v10.json").write_text("{}")
        (self.models_dir / ".hidden.json").write_text("{}")
        (self.models_dir / "notes.txt").write_text("")
        self.assertEqual(self.models.models()[-2:], ["k7_hev_v3", "k7_hev_v10"], "숫자는 수로 정렬, 규칙 밖 이름은 뺀다")

    def test_select_writes_the_choice_and_refuses_unknown(self):
        self.assertEqual(self.models.status()["selection_error"], "none")
        status = self.models.select("k7_hev_v2")
        self.assertEqual(json.loads(self.selection.read_text()), {"model": "k7_hev_v2"})
        self.assertEqual((status["selected"], status["selection_error"]), ("k7_hev_v2", ""))
        for name in ("k7_hev_v9", "../steering"):
            with self.assertRaises(Refusal) as refused:
                self.models.select(name)
            self.assertEqual(refused.exception.code, "error.lateral_nn_unknown")
        self.assertEqual(json.loads(self.selection.read_text()), {"model": "k7_hev_v2"}, "거부하면 그대로")

    def test_status_reports_default_and_controlsd(self):
        self.default.parent.mkdir()
        self.default.write_text(json.dumps({"model": "k7_hev_v3"}))
        self.assertIsNone(self.models.status()["controlsd"])
        self.status_file.write_text(json.dumps({"selected": "k7_hev_v2", "loaded": "k7_hev_v3", "path": "",
                                                "error": "k7_hev_v2: bad", "inputs": 33, "unix_time": 1}))
        status = self.models.status()
        self.assertEqual(status["default"], "k7_hev_v3")
        self.assertEqual(status["controlsd"]["loaded"], "k7_hev_v3")


class PandaUpdateTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.status = root / "edgepilot_panda_status.json"
        self.request = root / "edgepilot_panda_flash"
        self.image = root / "panda.bin.signed"
        self.control = root / "edgepilot_control_state"
        self.panda = PandaUpdate(str(self.status), str(self.request), str(self.image), str(self.control))

    def tearDown(self):
        self.temporary.cleanup()

    def publish_pandad(self, mode="app", version="DEV-23456789-DEBUG", hw_type=3, flash=None, stamp=None):
        self.status.write_text(json.dumps({
            "stamp_ns": boottime_ns() if stamp is None else stamp, "mode": mode, "serial": "abc",
            "hw_type": hw_type, "hw_name": "black panda", "firmware_version": version,
            "flash": flash or {"state": "idle", "step": "", "percent": 0, "error": "", "detail": "",
                               "version": "", "stamp_ns": 0}}))

    def publish_control(self, gear=GEAR_PARK, speed=0.0, vehicle_fresh=1, engaged=0, stamp=None):
        publish(self.control, control_payload(gear=gear, ego_speed_kph=speed, vehicle_fresh=vehicle_fresh,
                                              engaged=engaged), stamp=stamp)

    def test_constants_match_cpp(self):
        can_frame = cpp(ROOT / "src" / "car" / "can_frame.h")
        self.assertEqual(int(re.search(r"kGearPark = (\d+);", can_frame).group(1)), GEAR_PARK)
        protocol = cpp(ROOT / "src" / "panda" / "panda_protocol.h")
        self.assertIn(f"kPandaSignatureBytes = {PANDA_SIGNATURE_BYTES};", protocol)
        self.assertIn("kPandaAppMaxBytes = 3 * 16 * 1024;", protocol)
        self.assertEqual(PANDA_APP_MAX_BYTES, 3 * 16 * 1024)
        pandad = cpp(ROOT / "src" / "panda" / "pandad.cc")
        self.assertIn(f'kFirmwareStatusPath[] = "{PANDA_STATUS_PATH}";', pandad)
        self.assertIn(f'kFlashRequestPath[] = "{PANDA_FLASH_REQUEST_PATH}";', pandad)
        self.assertIn('kDefaultFirmwarePath[] = "firmware/panda.bin.signed";', pandad)
        # 플래싱 조건(panda_flash_allowed)과 지원 보드(panda_hw_type_flashable)도 같아야 한다
        firmware = cpp(ROOT / "src" / "panda" / "panda_firmware.cc")
        max_age_ns = int(re.search(r"kControlMaxAgeNs = (\d+)ULL;", firmware).group(1))
        self.assertEqual(max_age_ns, round(CONTROL_MAX_AGE_S * 1e9))
        self.assertEqual(float(re.search(r"kParkedMaxSpeedKph = ([\d.]+)f;", firmware).group(1)), PARKED_MAX_SPEED_KPH)
        flashable = re.search(r"return (hw_type == kPandaHw\w+(?: \|\| hw_type == kPandaHw\w+)*);", firmware).group(1)
        hw = {name: int(value) for name, value in re.findall(r"kPandaHw(\w+) = (\d+);", protocol)}
        self.assertEqual(sorted(hw[name] for name in re.findall(r"kPandaHw(\w+)", flashable)), sorted(PANDA_FLASHABLE_HW))

    def test_image_version_prefers_debug_like_cpp(self):
        """panda_firmware.cc find_version처럼 -DEBUG를 먼저 찾는다(앞쪽에 -RELEASE 꼴이 있어도)."""
        image = bytearray(panda_image(version="EDGE-f9907afb-DEBUG"))
        image[20:20 + 23] = b"\0REL-12345678-RELEASE\0"  # 버전(64바이트째)보다 앞
        self.image.write_bytes(bytes(image))
        self.assertEqual(panda_image_info(str(self.image))["version"], "EDGE-f9907afb-DEBUG")

    def test_image_info_follows_panda_firmware_rules(self):
        self.assertFalse(panda_image_info(str(self.image))["present"])
        self.image.write_bytes(panda_image())
        info = panda_image_info(str(self.image))
        self.assertTrue(info["valid"], info["error"])
        self.assertEqual((info["version"], info["size"]), ("EDGE-f9907afb-DEBUG", 1024 + PANDA_SIGNATURE_BYTES))
        broken = bytearray(panda_image())
        broken[0] ^= 4
        self.image.write_bytes(bytes(broken))
        self.assertFalse(panda_image_info(str(self.image))["valid"])
        self.image.write_bytes(panda_image(body=PANDA_APP_MAX_BYTES))
        self.assertFalse(panda_image_info(str(self.image))["valid"])

    def test_flash_is_requested_only_when_parked(self):
        self.image.write_bytes(panda_image())
        self.publish_pandad()
        self.publish_control(gear=5)
        status = self.panda.status()
        self.assertEqual(status["blocker"], "not_park")
        self.assertFalse(status["up_to_date"])
        with self.assertRaises(Refusal) as refused:
            self.panda.request("EDGE-f9907afb-DEBUG")
        self.assertEqual(refused.exception.detail(), {"code": "panda.reason.not_park"})
        self.assertFalse(self.request.exists())
        for blocker, kwargs in (("moving", {"speed": 3.0}), ("engaged", {"engaged": 1}),
                                ("vehicle_stale", {"vehicle_fresh": 0}),
                                ("control_stale", {"stamp": boottime_ns() - 5_000_000_000})):
            self.publish_control(**kwargs)
            self.assertEqual(self.panda.status()["blocker"], blocker)

        self.publish_control()
        with self.assertRaises(PermissionError):
            self.panda.request("EDGE-00000000-DEBUG")  # 페이지가 본 이미지가 아니다
        status = self.panda.request("EDGE-f9907afb-DEBUG")
        self.assertTrue(status["request_pending"])
        self.assertEqual(self.request.read_text(), "EDGE-f9907afb-DEBUG\n")
        with self.assertRaises(PermissionError):
            self.panda.request("EDGE-f9907afb-DEBUG")  # 이미 요청했다

    def test_flash_needs_a_running_pandad_and_a_panda(self):
        self.image.write_bytes(panda_image())
        self.publish_control()
        with self.assertRaises(PermissionError):
            self.panda.request("EDGE-f9907afb-DEBUG")
        self.publish_pandad(stamp=boottime_ns() - 10_000_000_000)
        self.assertFalse(self.panda.status()["available"])
        self.publish_pandad(mode="none", version="")
        with self.assertRaises(PermissionError):
            self.panda.request("EDGE-f9907afb-DEBUG")
        self.publish_pandad(hw_type=7)
        self.assertFalse(self.panda.status()["hw_supported"])
        with self.assertRaises(Refusal) as refused:
            self.panda.request("EDGE-f9907afb-DEBUG")
        self.assertEqual(refused.exception.code, "panda.reason.hw_unsupported")
        # 앱이 서지 않은 bootstub 판다는 CAN을 넘기지 않아 차 상태가 낡지만, 차 상태 없이 다시 쓸 수 있다
        self.publish_control(vehicle_fresh=0, gear=5, speed=40.0)
        self.publish_pandad(mode="bootstub", version="", hw_type=0)
        self.assertEqual(self.panda.status()["blocker"], "")
        self.publish_control(vehicle_fresh=0, engaged=1)  # 살아 있는 제어 상태가 결합 중이면 막는다
        self.assertEqual(self.panda.status()["blocker"], "engaged")
        self.publish_control(vehicle_fresh=0, stamp=boottime_ns() - 5_000_000_000)
        self.assertEqual(self.panda.status()["blocker"], "")
        self.assertTrue(self.panda.request("EDGE-f9907afb-DEBUG")["request_pending"])

    def test_status_reports_flash_progress(self):
        self.publish_pandad(version="EDGE-f9907afb-DEBUG",
                            flash={"state": "failed", "step": "write", "percent": 40, "error": "app_timeout",
                                   "detail": "x", "version": "EDGE-f9907afb-DEBUG", "stamp_ns": 1})
        self.image.write_bytes(panda_image())
        status = self.panda.status()
        self.assertTrue(status["up_to_date"])
        self.assertEqual((status["flash"]["error"], status["flash"]["step"]), ("app_timeout", "write"))
        self.assertTrue(status["hw_supported"])
        # 서버는 코드만 낸다: 글자는 페이지의 panda.reason.<코드>, panda.step.<단계> 문구
        self.assertFalse([key for key in status if key.endswith("_text")])


# ------------------------------------------------------------ 런타임 상태
class ProcessStatusTest(unittest.TestCase):
    def test_layout_matches_manager_and_cpp(self):
        """manager.py가 쓰는 ManagerState(overlayd도 읽는다)와 웹 콘솔이 읽는 배치가 ipc_messages.h와 같아야 한다."""
        from scripts import manager

        def constant(name):
            return int(re.search(rf"constexpr \w+ {name} = (0x[0-9a-fA-F]+|\d+);", cpp()).group(1), 0)

        self.assertEqual(manager.IPC_MAGIC, constant("kIpcMagic"))
        self.assertEqual(manager.IPC_VERSION, constant("kIpcVersion"))
        self.assertEqual(manager.MAX_PROCESSES, constant("kMaxProcesses"))
        self.assertEqual(manager.HEADER_SIZE, cpp_size("IpcHeader"))
        self.assertEqual(manager.PROCESS.size, cpp_size("ProcessState"))
        self.assertEqual(manager.MANAGER_STATE_SIZE, cpp_size("ManagerState"))
        self.assertEqual((IPC_MAGIC, IPC_VERSION, IPC_HEADER.size),
                         (constant("kIpcMagic"), constant("kIpcVersion"), cpp_size("IpcHeader")))
        self.assertEqual((PROCESS_STATE.format, MAX_PROCESSES), (manager.PROCESS.format, manager.MAX_PROCESSES))
        self.assertLessEqual(MANAGER_STATE_HEAD.size + PROCESS_STATE.size * MAX_PROCESSES, manager.MANAGER_STATE_SIZE)
        self.assertIn(f'kManagerStateTopic[] = "{MANAGER_STATE_TOPIC}";', cpp())
        self.assertIn(f'LatestPublisher("{MANAGER_STATE_TOPIC}"', cpp(ROOT / "scripts" / "manager.py"))

    @staticmethod
    def publish_processes(path, entries, stamp=None):
        stamp = boottime_ns() if stamp is None else stamp
        payload = bytearray(MANAGER_STATE_HEAD.size + PROCESS_STATE.size * MAX_PROCESSES)
        MANAGER_STATE_HEAD.pack_into(payload, 0, stamp, len(entries), 0)
        for index, (name, running) in enumerate(entries):
            PROCESS_STATE.pack_into(payload, MANAGER_STATE_HEAD.size + index * PROCESS_STATE.size, name.encode(), running)
        publish(path, bytes(payload), stamp=stamp)

    def test_status_lists_processes_and_goes_stale(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "edgepilot_manager_state"
            processes = ProcessStatus(str(path), install_dir=Path(directory), proc=Path(directory) / "proc")
            self.assertEqual(processes.status(), {"available": False, "age_s": 0.0, "processes": []})

            self.publish_processes(path, [("controlsd", 1), ("web_console", 0)])
            status = processes.status()
            self.assertTrue(status["available"])
            self.assertEqual(status["processes"], [
                {"name": "controlsd", "running": True, "version": None, "installed_version": None},
                {"name": "web_console", "running": False, "version": None, "installed_version": None}])
            self.publish_processes(path, [("controlsd", 1), ("web_console", 0)], stamp=boottime_ns() - 10_000_000_000)
            self.assertFalse(processes.status()["available"])

    def test_versions_come_from_the_running_and_installed_binaries(self):
        """버전은 실행 중인 실행 파일(/proc/<pid>/exe)과 설치된 실행 파일에 새긴 표식에서 읽는다. 리허설에서는 replayd가
        camerad 칸에서 돈다."""
        def binary(path, version, pad=5000):
            path.write_bytes(b"\x7fELF" + b"\0" * pad + VERSION_MARKER + version.encode() + b"\0more")

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            proc, install = root / "proc", root / "install"
            install.mkdir()
            for pid, argv0, version in ((101, "./controlsd", "v0.0.4-6-gaaaaaaa"), (102, "./replayd", "v0.0.4-6-gaaaaaaa"),
                                        (103, "/usr/bin/python3", "")):
                (proc / str(pid)).mkdir(parents=True)
                (proc / str(pid) / "cmdline").write_bytes(argv0.encode() + b"\0-m\0")
                if version:
                    binary(proc / str(pid) / "exe", version)
            binary(install / "controlsd", "v0.0.4-7-gbbbbbbb-dirty")  # 배포했지만 아직 다시 뜨지 않았다
            binary(install / "replayd", "v0.0.4-6-gaaaaaaa")
            binary(install / "modeld", "v0.0.4-6-gaaaaaaa")
            path = root / "edgepilot_manager_state"
            processes = ProcessStatus(str(path), install_dir=install, proc=proc)
            self.publish_processes(path, [("camerad", 1), ("controlsd", 1), ("modeld", 0), ("web_console", 1)])
            by_name = {p["name"]: (p["version"], p["installed_version"]) for p in processes.status()["processes"]}
            self.assertEqual(by_name, {"camerad": ("v0.0.4-6-gaaaaaaa", "v0.0.4-6-gaaaaaaa"),
                                       "controlsd": ("v0.0.4-6-gaaaaaaa", "v0.0.4-7-gbbbbbbb-dirty"),
                                       "modeld": (None, "v0.0.4-6-gaaaaaaa"),
                                       "web_console": (None, None)})

    def test_binary_version_marker(self):
        header = cpp(ROOT / "src" / "common" / "build_version.h")
        self.assertIn(f'#define EDGEPILOT_VERSION_MARKER_PREFIX "{VERSION_MARKER.decode()}"', header)
        template = cpp(ROOT / "cmake" / "build_version.cc.in")
        self.assertIn('EDGEPILOT_VERSION_MARKER_PREFIX "@version@"', template)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "binary"
            for content, expected in ((VERSION_MARKER + b"unknown\0", "unknown"),
                                      (b"x" * 100 + VERSION_MARKER + b"\0", None),          # 빈 버전
                                      (VERSION_MARKER + b"v0.0.4", None),                    # 끝나지 않았다
                                      (VERSION_MARKER + b"v" * 65 + b"\0", None),           # 64자보다 길다
                                      (b"no marker here\0", None),
                                      # 같은 글자로 시작하는 다른 문자열(테스트의 식 문자열 같은)은 건너뛴다
                                      (VERSION_MARKER + b'" + version\0' + VERSION_MARKER + b"v1\0", "v1")):
                path.write_bytes(content)
                self.assertEqual(read_binary_version(path), expected, content[:40])
            self.assertIsNone(read_binary_version(Path(directory) / "missing"))

    def test_manager_backs_off_a_crash_loop(self):
        """빨리 죽기를 거듭하는 프로세스는 세 번까지 1초, 그 뒤 2·4·8·16초에 다시 띄운다. camerad는 늘 1초."""
        from scripts import manager
        self.assertEqual([manager.restart_delay("web_console", n) for n in range(10)],
                         [1.0, 1.0, 1.0, 1.0, 2.0, 4.0, 8.0, 16.0, 16.0, 16.0])
        self.assertEqual({manager.restart_delay("camerad", n) for n in range(10)}, {1.0})

    def test_manager_does_not_count_its_own_stops(self):
        """매니저가 멈춘 프로세스(camerad를 따라 내린 overlayd 등)의 종료는 빠른 종료로 세지 않는다."""
        from scripts import manager
        spec = manager.ProcSpec("overlayd", ["/bin/true"])
        state = manager.ProcState(spec, started_monotonic=100.0, quick_exits=3)
        state.stop_requested = True
        self.assertEqual(manager.note_exit(state, 102.0), 1.0)
        self.assertEqual(state.quick_exits, 3)
        self.assertFalse(state.stop_requested)
        self.assertEqual(manager.note_exit(state, 102.0), 2.0)  # 스스로 죽으면 센다
        self.assertEqual(state.quick_exits, 4)
        state.started_monotonic = 0.0
        self.assertEqual(manager.note_exit(state, 102.0), 1.0)  # 10초 넘게 살았으면 처음부터
        self.assertEqual(state.quick_exits, 0)

    def test_manager_trims_a_log_even_when_the_copy_fails(self):
        """.log.1로 남기지 못해도(공간 부족 등) 로그는 비운다. tmpfs가 차는 것이 더 나쁘다."""
        from scripts import manager
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "camerad.log"
            log.write_bytes(b"x" * (manager.LOG_MAX_BYTES + 10))
            (Path(directory) / "camerad.log.1").mkdir()  # 사본 자리에 디렉터리가 있어 복사가 실패한다
            m = object.__new__(manager.Manager)
            m.procs = {"camerad": None}
            m.log_dir = directory
            m.trim_logs()
            self.assertEqual(log.stat().st_size, 0)

    def test_manager_starts_the_package(self):
        from scripts import manager

        source = cpp(ROOT / "scripts" / "manager.py")
        self.assertIn('ProcSpec("web_console", [sys.executable, "-m", "web_console"]', source)
        self.assertIn('installed="web_console/__main__.py"', source)
        self.assertTrue((ROOT / "scripts" / "web_console" / "__main__.py").is_file())
        self.assertTrue(hasattr(manager, "ProcSpec"))


# ------------------------------------------------------------ BEV
class BevTest(unittest.TestCase):
    def test_layout_matches_cpp_offsets(self):
        """페이지가 위치로 읽는 필드는 ipc_messages.h가 offsetof로 고정한 그 위치여야 한다."""
        self.assertEqual(cpp_size("ModelState"), MODEL_STATE_SIZE)
        self.assertEqual(cpp_size("ControlState"), CONTROL_STATE_SIZE)
        self.assertEqual(cpp_offsets("MODEL_STATE"), MODEL_STATE_AT)
        control = cpp_offsets("CONTROL_STATE")
        for name, at in CONTROL_STATE_AT.items():
            self.assertEqual(control.get(name), at, name)
        flags = {name: int(bit) for name, bit in re.findall(r"constexpr uint32_t kHudFlag(\w+) = 1U << (\d+);", cpp())}
        for name, bit in HUD_FLAG_BITS.items():
            self.assertEqual(flags.get(name), bit, name)
        self.assertIn(f'kModelStateTopic[] = "{MODEL_STATE_TOPIC}";', cpp())
        self.assertIn(f'kControlStateTopic[] = "{CONTROL_STATE_TOPIC}";', cpp())
        layout = bev_layout()
        self.assertEqual((layout["model"]["size"], layout["control"]["size"]), (MODEL_STATE_SIZE, CONTROL_STATE_SIZE))

    def test_lead_constants_match_cpp(self):
        """페이지의 앞차 확률 문턱과 레이더-카메라 거리는 비전 크루즈·출발 알림·HUD와 같은 model_output.h 값이어야 한다."""
        source = cpp(ROOT / "src" / "common" / "model_output.h")
        text = (STATIC_DIR / "bev_data.js").read_text(encoding="utf-8")
        for cpp_name, page_name in (("kLeadProbabilityThreshold", "LEAD_PROBABILITY"),
                                    ("kRadarToCameraDistanceM", "RADAR_TO_CAMERA")):
            value = float(re.search(rf"constexpr float {cpp_name} = ([\d.]+)f;", source).group(1))
            page = float(re.search(rf"const {page_name} = ([\d.]+);", text).group(1))
            self.assertEqual(page, value, page_name)

    def test_bev_constants_match_cpp(self):
        """BEV가 HUD·출발 알림처럼 그리고 판단하도록 옮겨 온 값들(hud_scene.cc, departure_alert.cc, can_frame.h)이
        C++과 같아야 한다."""
        bev = (STATIC_DIR / "bev.js").read_text(encoding="utf-8")
        data = (STATIC_DIR / "bev_data.js").read_text(encoding="utf-8")
        scene = cpp(ROOT / "src" / "hud" / "hud_scene.cc")
        alert = cpp(ROOT / "src" / "controls" / "departure_alert.cc")

        def cpp_value(text, name):
            return float(re.search(rf"constexpr (?:float|double|int) {name} = (-?[\d.]+)f?;", text).group(1))

        def js_object(text, name):
            body = re.search(rf"const {name} = \{{([^}}]*)\}};", text).group(1)
            return {key: float(value) for key, value in re.findall(r"(\w+): (-?[\d.]+)", body)}

        def js_value(text, name):
            return float(re.search(rf"\b{name} = (-?[\d.]+)", text).group(1))

        self.assertEqual(js_value(bev, "PATH_WIDTH"), 2 * cpp_value(scene, "kPathHalfWidth"))
        self.assertEqual(js_value(bev, "LANE_MIN_PROBABILITY"), cpp_value(scene, "kLaneMinProbability"))
        self.assertEqual(js_value(bev, "EDGE_MIN_CONFIDENCE"), cpp_value(scene, "kEdgeMinConfidence"))
        risk = js_object(bev, "RISK")
        self.assertEqual((risk["distance"], risk["closing"]),
                         (cpp_value(scene, "kLeadRiskDistanceM"), cpp_value(scene, "kLeadRiskClosingMps")))
        depart = js_object(bev, "DEPART")
        self.assertEqual(depart["stopped"], cpp_value(alert, "kStoppedSpeedMps"))
        self.assertEqual(depart["moving"], cpp_value(alert, "kMovingResetSpeedMps"))
        self.assertEqual(depart["gas"], cpp_value(alert, "kGasPressGoProbability"))
        self.assertEqual(depart["open"], cpp_value(alert, "kOpenPlanDistanceM"))
        self.assertEqual(depart["arm"], cpp_value(alert, "kStoppedPlanDistanceM"))
        self.assertEqual(js_value(data, "LANE_WIDTH_INDEX"), cpp_value(scene, "kLaneRulerIndex"))
        draw = js_object(data, "DRAW")
        self.assertEqual((draw["min"], draw["max"]),
                         (cpp_value(scene, "kMinDrawDistance"), cpp_value(scene, "kMaxDrawDistance")))
        self.assertEqual(js_value(data, "DRIVE_GEAR"), cpp_value(cpp(ROOT / "src" / "car" / "can_frame.h"), "kGearDrive"))

    def test_vehicle_tab_bands_and_bev_frame_match(self):
        """차량 특성 탭이 보이는 허용 폭(torqued 배율 ±30%, 마찰 ±50%, paramsd 조향비 0.5~2배)과 BEV 프레임 매직이
        학습기·서버와 같아야 한다."""
        page = (STATIC_DIR / "vehicle_view.js").read_text(encoding="utf-8")
        torque = cpp(ROOT / "src" / "learners" / "torque_estimator.cc")
        learner = cpp(ROOT / "src" / "learners" / "vehicle_params_learner.cc")
        factor = float(re.search(r"kFactorSanity = ([\d.]+);", torque).group(1))
        friction = float(re.search(r"kFrictionSanity = ([\d.]+);", torque).group(1))
        self.assertIn(f"s.lat_accel_factor_raw, s.prior_lat_accel_factor, {factor}, 2)", page)
        self.assertIn(f"s.friction_raw, s.prior_friction, {friction}, 3)", page)
        low = float(re.search(r"min_sr_\(([\d.]+) \* c\.steer_ratio\)", learner).group(1))
        high = float(re.search(r"max_sr_\(([\d.]+) \* c\.steer_ratio\)", learner).group(1))
        self.assertIn(f"s.prior_steer_ratio * {low:g}, 1)", page)
        self.assertIn(f"s.prior_steer_ratio * {high:g}, 1)", page)
        magic = int(re.search(r"const MAGIC = (0x[0-9a-fA-F]+);", (STATIC_DIR / "bev_data.js").read_text()).group(1), 16)
        self.assertEqual(magic, BEV_FRAME_MAGIC)

    def test_lane_filter_constants_match_cpp(self):
        """BEV의 차선 필터 사본(lane_filter.js)의 가중·끊김·다시 시작 기준은 플래너의 lane_line_filter와 같아야 한다."""
        header = cpp(ROOT / "src" / "planning" / "lane_line_filter.h")
        source = cpp(ROOT / "src" / "planning" / "lane_line_filter.cc")
        page = (STATIC_DIR / "lane_filter.js").read_text(encoding="utf-8")
        for cpp_text, cpp_name, page_name in ((header, "kLaneLineFilterWeight", "LANE_FILTER_WEIGHT"),
                                              (source, "kMaxGapS", "MAX_GAP_S"),
                                              (source, "kRestartReachM", "RESTART_REACH_M"),
                                              (source, "kRestartM", "RESTART_M")):
            value = float(re.search(rf"constexpr double {cpp_name} = ([\d.]+);", cpp_text).group(1))
            ours = float(re.search(rf"const {page_name} = ([\d.]+);", page).group(1))
            self.assertEqual(ours, value, page_name)

    def test_lane_filter_golden_is_shared(self):
        """C++ 필터가 내는 기준 출력(gtest_lane_line_filter)과 JS 사본이 맞추는 값(check_bev_lane_filter.mjs)이 같아야 한다."""
        def golden(path):
            text = path.read_text(encoding="utf-8")
            block = re.search(r"GOLDEN BEGIN(.*?)GOLDEN END", text, re.S).group(1)
            return [float(v) for v in re.findall(r"-?\d+\.\d+", block)]
        cpp_golden = golden(ROOT / "tests" / "gtest_lane_line_filter.cc")
        self.assertEqual(len(cpp_golden), 36)
        self.assertEqual(golden(ROOT / "tests" / "check_bev_lane_filter.mjs"), cpp_golden)

    def test_lane_filter_copy_matches_cpp(self):
        """JS 사본을 jsc(macOS)나 node로 돌려 C++ 기준 출력과 맞춘다. 둘 다 없으면 건너뛴다."""
        jsc = Path("/System/Library/Frameworks/JavaScriptCore.framework/Versions/Current/Helpers/jsc")
        script = ROOT / "tests" / "check_bev_lane_filter.mjs"
        if jsc.is_file():
            command = [str(jsc), "-m", str(script)]
        elif shutil.which("node"):
            command = ["node", str(script)]
        else:
            self.skipTest("no JavaScript engine (jsc or node)")
        run = subprocess.run(command, capture_output=True, text=True, timeout=60)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        self.assertIn("all passed", run.stdout)

    def test_page_reads_what_the_server_says(self):
        """bev_data.js NEEDS의 필드가 /api/bev 레이아웃에 다 있어야 한다(빠지면 페이지가 그리지 않는다). ControlState
        위치표는 Panda·보정 카드와 같이 쓰므로 BEV가 읽지 않는 필드도 있다."""
        text = (STATIC_DIR / "bev_data.js").read_text(encoding="utf-8")
        needs = re.search(r"const NEEDS = \{(.*?)\n\};", text, re.S).group(1)
        parts = {part: set(re.findall(r'"(\w+)"', keys)) for part, keys in re.findall(r"(\w+): \[(.*?)\]", needs, re.S)}
        layout = bev_layout()
        self.assertEqual(set(parts), set(layout))
        for part, fields in parts.items():
            self.assertLessEqual(fields, set(layout[part]), part)
        self.assertEqual(parts["model"], set(layout["model"]), "ModelState fields are the BEV's alone")

    def test_stream_sends_new_model_frames_and_idles_without_them(self):
        def fill(path, seq, value, size):
            publish(path, bytes([value]) * size, seq=seq)

        def split(frame):
            magic, model, control, now = BEV_FRAME.unpack_from(frame)
            self.assertEqual(magic, BEV_FRAME_MAGIC)
            self.assertEqual(len(frame), BEV_FRAME.size + model + control)
            body = frame[BEV_FRAME.size:]
            return body[:model], body[model:], now

        with tempfile.TemporaryDirectory() as directory:
            model, control = Path(directory) / "model_state", Path(directory) / "control_state"
            fill(model, 2, 1, MODEL_STATE_SIZE)
            fill(control, 2, 7, CONTROL_STATE_SIZE)

            async def take():
                frames = bev_frames(20, str(model), str(control), idle_s=0.2)
                first = await anext(frames)
                fill(model, 4, 2, MODEL_STATE_SIZE)
                second = await anext(frames)
                control.unlink()
                third = await anext(frames)  # 새 모델 프레임 없음: idle_s 뒤 머리만
                await frames.aclose()
                return first, second, third

            first, second, third = (split(frame) for frame in asyncio.run(take()))
        self.assertEqual((first[0], first[1]), (bytes([1]) * MODEL_STATE_SIZE, bytes([7]) * CONTROL_STATE_SIZE))
        self.assertEqual(second[0], bytes([2]) * MODEL_STATE_SIZE)
        self.assertEqual((third[0], third[1]), (b"", b""))
        self.assertLess(first[2], third[2])

    def test_stream_ends_when_the_server_stops(self):
        """uvicorn은 응답이 끝나기를 기다리므로, 서버가 내려가기 시작하면 스트림이 스스로 끝나야 한다."""
        stopping = [False]

        async def take():
            frames = bev_frames(20, "/nonexistent/model", "/nonexistent/control", idle_s=0.05,
                                stopping=lambda: stopping[0])
            first = await anext(frames)
            stopping[0] = True
            with self.assertRaises(StopAsyncIteration):
                await asyncio.wait_for(anext(frames), 1.0)
            return first

        self.assertEqual(BEV_FRAME.unpack_from(asyncio.run(take()))[1:3], (0, 0))


# ------------------------------------------------------------ 페이지 파일
class StaticPageTest(unittest.TestCase):
    def test_asset_cache_is_keyed_by_the_resolved_file(self):
        """같은 파일을 다른 글자로 요청해도 캐시는 하나다(요청마다 사본이 쌓이지 않는다)."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "static"
            (root / "three").mkdir(parents=True)
            (root / "three" / "a.js").write_text("export const a = 1;\n", encoding="utf-8")
            assets = StaticAssets(root)
            for name in ("three/a.js", "./three/a.js", "three//a.js", "three/./a.js"):
                self.assertIsNotNone(assets.get(name), name)
            self.assertIsNone(assets.get("three/../three/a.js"))  # ..은 숨김 경로처럼 거부한다
            self.assertEqual(list(assets.cache), ["three/a.js"])

    def test_assets_are_gzipped_once_and_stay_in_static(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "static"
            (root / "three").mkdir(parents=True)
            (root / "three" / "a.js").write_text("export const a = 1;\n", encoding="utf-8")
            (root / "notes.txt").write_text("x", encoding="utf-8")
            (root / ".hidden.js").write_text("x", encoding="utf-8")
            (Path(directory) / "outside.js").write_text("x", encoding="utf-8")
            assets = StaticAssets(root)
            first = assets.get("three/a.js")
            self.assertEqual(gzip.decompress(first.gzipped), b"export const a = 1;\n")
            self.assertEqual(first.media_type, "text/javascript; charset=utf-8")
            self.assertIs(assets.get("three/a.js").gzipped, first.gzipped)
            for name in ("../outside.js", "notes.txt", ".hidden.js", "missing.js", "/etc/hosts", "three", ""):
                self.assertIsNone(assets.get(name), name)
            (root / "three" / "a.js").write_text("export const a = 22;\n", encoding="utf-8")
            self.assertNotEqual(assets.get("three/a.js").etag, first.etag)

    def test_page_modules_and_three_resolve(self):
        page = (STATIC_DIR / "index.html").read_text(encoding="utf-8")
        imports = json.loads(re.search(r'<script type="importmap">(.*?)</script>', page, re.S).group(1))["imports"]
        for url in [*imports.values(), *re.findall(r'(?:src|href)="(/static/[^"]+)"', page)]:
            self.assertTrue((STATIC_DIR / Path(url).relative_to("/static")).is_file(), url)
        self.assertIn('src="/static/console.js"', page)
        for module in [*STATIC_DIR.glob("*.js"), *STATIC_DIR.glob("three/*.js")]:
            text = module.read_text(encoding="utf-8")
            for target in re.findall(r"""(?:from|import)\s*\(?\s*["'](\./[^"']+)["']""", text):
                self.assertTrue((module.parent / target).is_file(), f"{module.name} imports {target}")
            for bare in re.findall(r"""^import[^;]*?from\s*["']([^"'./][^"']*)["']""", text, re.M):
                self.assertIn(bare, imports, f"{module.name} imports {bare}")
        for element_id in ("main", "tabs", "runtime", "runtime-panel", "reload", "toasts"):
            self.assertIn(f'id="{element_id}"', page)

    def test_page_calls_only_routes_the_server_has(self):
        server = (ROOT / "scripts" / "web_console" / "console_server.py").read_text(encoding="utf-8")
        routes = []
        for method, path in re.findall(r'@application\.(get|post|patch)\("([^"]+)"\)', server):
            pattern = re.sub(r"\\\{\w+:path\\\}", ".+", re.sub(r"\\\{\w+\\\}", "[^/]+", re.escape(path)))
            routes.append((method.upper(), re.compile(pattern + r"\Z")))
        calls = set()
        for module in STATIC_DIR.glob("*.js"):
            text = module.read_text(encoding="utf-8")
            for method, url in re.findall(r"api\.(get|post|patch)\(\s*[`\"]([^`\"]+)[`\"]", text):
                calls.add((method.upper(), url))
            for url in re.findall(r"fetch\(\s*[`\"]([^`\"]+)[`\"]", text):
                calls.add(("GET", url))
        self.assertGreaterEqual(len(calls), 9)
        for method, url in sorted(calls):
            path = re.sub(r"\$\{[^}]+\}", "x", url).split("?", 1)[0]
            self.assertTrue(any(method == route_method and pattern.match(path) for route_method, pattern in routes),
                            f"{method} {url}")


if __name__ == "__main__":
    unittest.main()
