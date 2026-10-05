# 스크립트

호스트에서 빌드·배포·검사에 쓰는 스크립트와, 보드에 설치돼 런타임과 함께 도는 Python이 있다.
보드용 파일은 `upload_to_board.sh`가 보드 설치 디렉터리(`/root/edgepilot`)의 최상위에 실행 파일과
나란히 둔다.

## 호스트

| 스크립트 | 사용 | 하는 일 |
| --- | --- | --- |
| `fetch_maixcam2_sdk.sh` | `[root@보드]` | MaixCAM2 교차 빌드 의존성을 `deps/ax630/`에 받는다. MSP SDK `v3.0.0_20250319114413`(SHA256 고정)과 MaixCDK 헤더(커밋 `30f4b8b`)는 네트워크에서, 보드의 `/opt/lib`, `libmaixcam_lib.so.1.2.5`, `libsamplerate`, OpenCV 4.11은 보드에서 SSH로 그대로 복사한다 |
| `upload_to_board.sh` | `[root@보드]` | `build-ax630/bin`의 런타임, 보드용 Python, UI 스프라이트, 파라미터 기본값, 모델(`models/supercombo.axmodel`, 바뀌었을 때만)을 보드에 올린다 |
| `run_host_tests.sh` | | 호스트 단위 테스트를 빌드하고 `ctest`로 전부 돌린다([gtest/](../gtest/README.md)). 보드도 `deps/`도 필요 없다 |

빌드 자체는 `tools/docker_ax630/build.sh`가 arm64 Ubuntu 22.04 컨테이너에서 한다(결과는
`build-ax630/bin`). 보드 기본 주소는 `root@192.168.219.117`이고 SSH 키 인증을 쓴다.

환경 변수로 바꿀 수 있는 것:

- `upload_to_board.sh`: `models/supercombo.axmodel`은 보드 것과 체크섬이 다를 때만 보낸다. `EDGEPILOT_BIN_DIR`
  (기본 `build-ax630/bin`), `EDGEPILOT_BOARD_DIR`(기본 `/root/edgepilot`). 실행 중인 바이너리는
  덮어쓸 수 없으므로 `.upload/`에 올린 뒤 `mv`로 바꾼다. 매니저는 다시 띄우지 않는다. 보드의
  `params/`는 덮어쓰지 않고, 기본값은 `params.defaults/`에 두어 없는 파일만 채운다.
- `run_host_tests.sh`: `EDGEPILOT_HOST_BUILD_DIR`(기본 `build-host`), `JOBS`.

자세한 빌드와 배포 절차는 [Build and deploy](../docs/build-and-deploy.md)에 있다.

## 보드

| 파일 | 사용 | 하는 일 |
| --- | --- | --- |
| `manager.py` | `python3 /root/edgepilot/manager.py [supercombo.axmodel]` | 런타임 감시자. 보드 UI 런처를 멈추고 프로세스를 순서대로 띄우며 죽으면 1초 뒤 다시 띄운다. 아직 부팅 때 자동으로 실행되지 않는다(init 스크립트·systemd 유닛 없음) |
| `param_server.py` | `[--host 주소] [--port 포트]` | 파라미터 편집 웹 서버(FastAPI, 기본 `0.0.0.0:8080`). 매니저가 함께 띄운다 |
| `web/` | (정적 파일) | 웹 콘솔 BEV 탭의 JS(`bev.js`, 자차 K7 모델 `bev_k7.js`, 앞차 모델 `bev_car.js`, `bev_data.js`)와 three.js 0.186.1(`three/`). 보드에는 `param_server.py` 옆 `web/`에 둔다. 아직 `upload_to_board.sh`가 올리지 않는다 |
| `display_control.py` | (모듈) | LCD 백라이트 제어. 파라미터 서버가 `display.json`을 적용할 때 쓴다. 아직 K230 핀(IO25, `pwmchip3`) 기준이라 MaixCAM2 백라이트(`pwmchip0/pwm3`)와 맞지 않는다 |
| `requirements-param-server.txt` | `python3 -m pip install -r ...` | 파라미터 서버 의존성(fastapi, uvicorn) |

어떤 프로세스를 띄울지와 환경 변수는 [분할 런타임](../docs/runtime.md)과
[런타임 옵션](../docs/runtime-options.md)에 있다.

## 작성 규칙

- 첫머리 주석(Python은 모듈 docstring)에 용도를 한글로 적고 `사용:` 줄로 끝낸다.
- 셸 스크립트는 `set -euo pipefail`로 시작하고, 저장소 루트를 스스로 찾아 어디서 불러도
  돌게 한다. 보드에서도 도는 스크립트는 POSIX `sh`로 쓴다.
