"""조향 탭의 학습 토크 모델 카드(docs/lateral-nn.md): models/lateral_nn/의 모델 파일 이름, 지금 고른 것
(params/lateral_nn.json), controlsd가 실제로 쓰는 것(상태 파일, src/controls/lateral_nn_select.cc)을 보여 주고
고르기를 바꾼다. controlsd는 1초 안에 새 고르기를 보고 백그라운드에서 읽어 갈아 끼운다(재시작 없음, 피드포워드는 두
모델 출력의 차이만큼만 움직인다). 이름 규칙과 경로는 src/common/lateral_nn_path.cc와 같다.

파일 내용은 읽지 않는다. 모델 검사(형식, 시험 벡터)는 controlsd가 읽을 때 하고, 거부하면 그 이유가 상태 파일에 남는다.
controlsd는 모델을 읽을 때만 상태 파일을 쓰므로 controlsd가 멈춰도 파일은 남는다: controlsd가 도는지는 서버가
따로(학습 상태의 신선도로) 보고 status에 넘긴다."""
from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, List, Tuple

from .files import defaults_dir, env_path, params_dir, read_json_object, read_json_or_none, write_json_atomic
from .refusal import Refusal

# controlsd와 같이 작업 디렉터리 기준(kLateralNnModelDir, kLateralNnSelectionFile)
MODEL_DIR = Path("models/lateral_nn")
SELECTION_FILE = "lateral_nn.json"
STATUS_PATH = "/dev/shm/edgepilot_lateral_nn_status.json"  # EDGEPILOT_LATERAL_NN_STATUS로 바꿀 수 있다
# valid_lateral_nn_name: 영문·숫자·_-., 64자까지, '.'로 시작하지 않음
NAME_PATTERN = re.compile(r"[A-Za-z0-9_-][A-Za-z0-9_.-]{0,63}")


def natural_key(name: str) -> List[Any]:
    """k7_hev_v10이 k7_hev_v9 뒤에 오게 숫자는 수로 비교한다."""
    return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", name)]


def read_choice(path: Path) -> Tuple[str, str]:
    """고르기 파일의 (이름, 오류). lateral_nn_choice_from과 같은 규칙: 파일이 없으면 ("", "none")."""
    try:
        document = read_json_object(path)
    except FileNotFoundError:
        return "", "none"
    except (OSError, ValueError):
        return "", "unreadable"
    name = document.get("model")
    if not isinstance(name, str):
        return "", "no_model"
    return (name, "") if NAME_PATTERN.fullmatch(name) else ("", "bad_name")


class LateralNnModels:
    def __init__(self, model_dir: Path = MODEL_DIR, selection_path: Path | None = None,
                 default_path: Path | None = None, status_path: str | None = None):
        self.model_dir = model_dir
        self.selection_path = selection_path or params_dir() / SELECTION_FILE
        self.default_path = default_path or defaults_dir() / SELECTION_FILE
        self.status_path = Path(status_path) if status_path else env_path("EDGEPILOT_LATERAL_NN_STATUS", STATUS_PATH)

    def models(self) -> List[str]:
        """고를 수 있는 모델 이름(파일 이름에서 .json을 뺀 것)."""
        names = [path.stem for path in self.model_dir.glob("*.json") if NAME_PATTERN.fullmatch(path.stem)]
        return sorted(names, key=natural_key)

    def status(self, controlsd_running: bool) -> Dict[str, Any]:
        """controlsd는 controlsd가 마지막으로 읽은 결과({selected, loaded, path, error, inputs, unix_time})이고,
        controlsd가 돌지 않으면 None이다."""
        selected, selection_error = read_choice(self.selection_path)
        default, _ = read_choice(self.default_path)
        controlsd = read_json_or_none(self.status_path) if controlsd_running else None
        return {"models": self.models(), "selected": selected, "selection_error": selection_error,
                "default": default, "controlsd": controlsd}

    def select(self, name: str) -> None:
        """고르기를 바꾼다. 목록에 있는 이름만 받는다."""
        if name not in self.models():
            raise Refusal("error.lateral_nn_unknown", status=404, name=name)
        write_json_atomic(self.selection_path, {"model": name})
