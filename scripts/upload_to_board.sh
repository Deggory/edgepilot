#!/usr/bin/env bash
# 교차 빌드한 런타임을 MaixCAM2에 올린다: 실행 파일, 보드용 Python, UI 스프라이트, 파라미터 기본값,
# 그리고 모델(models/supercombo.axmodel, 보드의 것과 체크섬이 다를 때만 보낸다).
# 보드의 params/는 덮어쓰지 않고, 기본값은 params.defaults/에 두어 없는 파일만 채운다.
# 실행 중인 바이너리는 덮어쓸 수 없으므로 .upload/에 올린 뒤 mv로 바꾼다. 매니저는 다시
# 띄우지 않는다.
# 사용: scripts/upload_to_board.sh [root@보드]
#   (기본 root@192.168.219.117, 설치 디렉터리 EDGEPILOT_BOARD_DIR=/root/edgepilot, 빌드 build-ax630)
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

BOARD="${1:-root@192.168.219.117}"
DEST="${EDGEPILOT_BOARD_DIR:-/root/edgepilot}"
BIN_DIR="${EDGEPILOT_BIN_DIR:-build-ax630/bin}"
AXMODEL="models/supercombo.axmodel"
SSH=(ssh -o StrictHostKeyChecking=no)
SCP=(scp -q -o StrictHostKeyChecking=no)

runtime_files=(
  "${BIN_DIR}/camerad"
  "${BIN_DIR}/modeld"
  "${BIN_DIR}/overlayd"
  "${BIN_DIR}/controlsd"
  "${BIN_DIR}/recordd"
  "${BIN_DIR}/camcal"
  "${BIN_DIR}/imud"
  scripts/manager.py
  scripts/param_server.py
  scripts/display_control.py
  scripts/requirements-param-server.txt
)
[ -x "${BIN_DIR}/pandad" ] && runtime_files+=("${BIN_DIR}/pandad")
param_files=(calibration.json adaptive_cruise.json steering.json driving.json recording.json display.json)
ui_assets=(
  assets/ui/traffic_wait_red_retro-270x155-v3.png
  assets/ui/traffic_go_green_retro-270x155-v3.png
)

for file in "${runtime_files[@]}" "${ui_assets[@]}" "$AXMODEL"; do
  [ -f "$file" ] || { echo "Missing: $file" >&2; exit 1; }
done

"${SSH[@]}" "$BOARD" "rm -rf '$DEST/.upload' && mkdir -p '$DEST/.upload/assets/ui' '$DEST/assets/ui' '$DEST/models' '$DEST/params' '$DEST/params.defaults'"
"${SCP[@]}" "${runtime_files[@]}" "$BOARD:$DEST/.upload/"
"${SCP[@]}" "${ui_assets[@]}" "$BOARD:$DEST/.upload/assets/ui/"
model_sha="$(shasum -a 256 "$AXMODEL" | cut -d' ' -f1)"
board_sha="$("${SSH[@]}" "$BOARD" "sha256sum '$DEST/models/supercombo.axmodel' 2>/dev/null | cut -d' ' -f1" || true)"
model_note="model unchanged"
if [ "$model_sha" != "$board_sha" ]; then
  "${SCP[@]}" "$AXMODEL" "$BOARD:$DEST/.upload/supercombo.axmodel"
  model_note="model updated"
fi
"${SCP[@]}" "${param_files[@]/#/params/}" "$BOARD:$DEST/params.defaults/"
"${SSH[@]}" "$BOARD" "set -e; cd '$DEST'
  for f in .upload/assets/ui/*; do mv \"\$f\" assets/ui/; done
  if [ -f .upload/supercombo.axmodel ]; then mv .upload/supercombo.axmodel models/; fi
  rm -f models/supercombo_npu1.axmodel  # 예전 AI-ISP용 별도 모델(이제 supercombo.axmodel 하나)
  for f in .upload/*; do [ -f \"\$f\" ] && mv \"\$f\" .; done
  for name in ${param_files[*]}; do [ -e params/\$name ] || cp params.defaults/\$name params/; done
  rm -rf .upload; sync"
echo "Uploaded runtime to $BOARD:$DEST ($model_note)"
