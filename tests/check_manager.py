#!/usr/bin/env python3
"""scripts/manager.py 검사. stdlib만 쓴다: 매니저가 쓰는 ManagerState 배치가 src/common/ipc_messages.h와 같은지,
다시 띄우기 규칙(빨리 죽기를 거듭하면 늦추기, 스스로 멈춘 것은 세지 않기), 끝낼 때 recordd를 더 기다리기, 로그 자르기,
pandad에 주는 safety 기본값이 controlsd가 기다리는 값과 같은지. ctest가 저장소 루트에서 돌린다."""
import re
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import manager  # noqa: E402

IPC_MESSAGES = ROOT / "src" / "common" / "ipc_messages.h"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def constant(name: str) -> int:
    return int(re.search(rf"constexpr \w+ {name} = (0x[0-9a-fA-F]+|\d+);", read(IPC_MESSAGES)).group(1), 0)


def cpp_size(struct_name: str) -> int:
    return int(re.search(rf"sizeof\({struct_name}\) == (\d+)", read(IPC_MESSAGES)).group(1))


class ManagerTest(unittest.TestCase):
    def test_layout_matches_cpp(self):
        """매니저가 쓰는 ManagerState(overlayd와 웹 콘솔이 읽는다)가 ipc_messages.h와 같아야 한다."""
        self.assertEqual(manager.IPC_MAGIC, constant("kIpcMagic"))
        self.assertEqual(manager.IPC_VERSION, constant("kIpcVersion"))
        self.assertEqual(manager.MAX_PROCESSES, constant("kMaxProcesses"))
        self.assertEqual(manager.HEADER_SIZE, cpp_size("IpcHeader"))
        self.assertEqual(manager.PROCESS.size, cpp_size("ProcessState"))
        self.assertEqual(manager.MANAGER_STATE_SIZE, cpp_size("ManagerState"))
        topic = re.search(r'kManagerStateTopic\[\] = "([^"]+)";', read(IPC_MESSAGES)).group(1)
        self.assertIn(f'LatestPublisher("{topic}"', read(ROOT / "scripts" / "manager.py"))

    def test_backs_off_a_crash_loop(self):
        """빨리 죽기를 거듭하는 프로세스는 세 번까지 1초, 그 뒤 2·4·8·16초에 다시 띄운다. camerad는 늘 1초."""
        self.assertEqual([manager.restart_delay("web_console", n) for n in range(10)],
                         [1.0, 1.0, 1.0, 1.0, 2.0, 4.0, 8.0, 16.0, 16.0, 16.0])
        self.assertEqual({manager.restart_delay("camerad", n) for n in range(10)}, {1.0})

    def test_does_not_count_its_own_stops(self):
        """매니저가 멈춘 프로세스(camerad를 따라 내린 overlayd 등)의 종료는 빠른 종료로 세지 않는다."""
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

    def test_waits_longer_for_recordd_on_shutdown(self):
        """끝낼 때는 recordd만 더 기다린다: SIGTERM 뒤 스테이징에 남은 녹화를 SD로 옮기고 스스로 끝난다. 다른 프로세스는
        평소 시한에 SIGKILL한다."""
        m = object.__new__(manager.Manager)
        script = "trap 'sleep 0.6; exit 0' TERM; while :; do sleep 0.05; done"
        m.procs = {}
        for name in ("recordd", "overlayd"):
            state = manager.ProcState(manager.ProcSpec(name, ["/bin/sh", "-c", script]))
            # 셸이 SIGTERM으로 끝난 sleep을 "Terminated: 15"로 알리므로 출력은 버린다
            state.proc = subprocess.Popen(state.spec.cmd, start_new_session=True, stderr=subprocess.DEVNULL)
            m.procs[name] = state
        time.sleep(0.3)  # trap이 걸리기를 기다린다
        m.stop_all(timeout_s=0.3, recordd_timeout_s=3.0)
        self.assertEqual(m.procs["recordd"].proc.wait(timeout=5), 0, "recordd는 스스로 끝날 때까지 기다렸다")
        self.assertEqual(m.procs["overlayd"].proc.wait(timeout=5), -signal.SIGKILL, "다른 프로세스는 시한에 내렸다")
        self.assertLess(manager.RECORDD_STOP_TIMEOUT_S, 10.0, "edgepilot.service TimeoutStopSec 안")

    def test_sets_the_safety_mode_controlsd_expects(self):
        """매니저가 pandad에 주는 safety 모드·인자 기본값이 controlsd가 기다리는 값(control_holds.h)과 같다. 다르면
        Panda가 준비돼도 결합하지 못한다."""
        source = read(ROOT / "scripts" / "manager.py")
        mode = re.search(r'setdefault\("EDGEPILOT_PANDA_SAFETY", "(\w+)"\)', source).group(1)
        param = re.search(r'setdefault\("EDGEPILOT_PANDA_SAFETY_PARAM", "(\d+)"\)', source).group(1)
        pandad = read(ROOT / "src" / "panda" / "pandad.cc")
        constant = re.search(r'if \(mode == "' + mode + r'"\) return (\w+);', pandad).group(1)
        holds = read(ROOT / "src" / "controls" / "control_holds.h")
        self.assertIn(f"kExpectedPandaSafetyModel = {constant};", holds)
        self.assertIn(f"kExpectedPandaSafetyParam = {param};", holds)

    def test_trims_a_log_even_when_the_copy_fails(self):
        """.log.1로 남기지 못해도(공간 부족 등) 로그는 비운다. tmpfs가 차는 것이 더 나쁘다."""
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "camerad.log"
            log.write_bytes(b"x" * (manager.LOG_MAX_BYTES + 10))
            (Path(directory) / "camerad.log.1").mkdir()  # 사본 자리에 디렉터리가 있어 복사가 실패한다
            m = object.__new__(manager.Manager)
            m.procs = {"camerad": None}
            m.log_dir = directory
            m.trim_logs()
            self.assertEqual(log.stat().st_size, 0)

    def test_starts_the_package(self):
        source = read(ROOT / "scripts" / "manager.py")
        self.assertIn('ProcSpec("web_console", [sys.executable, "-m", "web_console"]', source)
        self.assertIn('installed="web_console/__main__.py"', source)
        self.assertTrue((ROOT / "scripts" / "web_console" / "__main__.py").is_file())
        self.assertTrue(hasattr(manager, "ProcSpec"))


if __name__ == "__main__":
    unittest.main()
