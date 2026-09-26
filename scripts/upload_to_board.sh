#!/usr/bin/env bash
# 빌드한 런타임을 보드에 올린다: 실행 파일, 보드용 Python, 모델, UI 스프라이트, 파라미터 기본값.
# 보드의 params/는 덮어쓰지 않고, 기본값은 params.defaults/에 두어 없는 파일만 채운다.
# 사용: scripts/upload_to_board.sh [root@보드]   (기본 root@192.168.219.111, 바이너리는 EDGEPILOT_BUILD_DIR/bin)
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

BOARD="${1:-root@192.168.219.111}"
DEST="${EDGEPILOT_BOARD_DIR:-/root/edgepilot}"
BUILD_DIR="${EDGEPILOT_BUILD_DIR:-build}"
BIN_DIR="${EDGEPILOT_BIN_DIR:-${BUILD_DIR}/bin}"
read -r -a SSH_CMD <<< "${EDGEPILOT_SSH:-ssh}"
read -r -a SCP_CMD <<< "${EDGEPILOT_SCP:-scp}"
SSH_OPTIONS=(
  -o StrictHostKeyChecking=no
)

runtime_files=(
  "${BIN_DIR}/camerad"
  "${BIN_DIR}/modeld"
  "${BIN_DIR}/overlayd"
  "${BIN_DIR}/recordd"
  scripts/manager.py
  scripts/param_server.py
  scripts/display_control.py
  scripts/requirements-param-server.txt
)
model="models/supercombo.kmodel"
param_files=(calibration.json adaptive_cruise.json steering.json driving.json recording.json display.json)
ui_assets=(
  assets/ui/traffic_wait_red_retro-270x155-v3.png
  assets/ui/traffic_go_green_retro-270x155-v3.png
)

if [ -x "${BIN_DIR}/pandad" ]; then
  runtime_files+=("${BIN_DIR}/pandad" "${BIN_DIR}/controlsd")
fi

for runtime_file in "${runtime_files[@]}"; do
  if [ ! -f "${runtime_file}" ]; then
    echo "Missing runtime file: ${runtime_file}" >&2
    exit 1
  fi
done
for ui_asset in "${ui_assets[@]}"; do
  if [ ! -f "${ui_asset}" ]; then
    echo "Missing UI asset: ${ui_asset}" >&2
    exit 1
  fi
done

# init 스크립트는 저장소가 가진다. 예전 이미지의 S35supercombo_k230은 지우기만 하고(멈추지 않는다),
# 그 설치(/root/supercombo_k230)에 학습된 params가 있으면 새 설치로 한 번 옮긴다.
# 돌고 있는 런타임은 멈추지 않는다: 멈춘 직후 카메라/화면 DMA가 해제된 메모리에 계속 써서
# 방금 올린 파일과 파일시스템이 깨진 적이 있다(2026-09-26). 파일은 mv로 새 inode가 되므로
# 도는 프로세스는 옛 파일을 계속 쓰고, 새 런타임은 재부팅 때 뜬다.
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" scripts/S35edgepilot "$BOARD:/etc/init.d/S35edgepilot.tmp"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" \
  "chmod 755 /etc/init.d/S35edgepilot.tmp && mv /etc/init.d/S35edgepilot.tmp /etc/init.d/S35edgepilot;
   rm -f /etc/init.d/S35supercombo_k230;
   if [ -d /root/supercombo_k230/params ] && [ ! -d '$DEST/params' ]; then mkdir -p '$DEST' && cp -a /root/supercombo_k230/params '$DEST/params'; fi;
   rm -rf '$DEST/.upload'; mkdir -p '$DEST/.upload' '$DEST/models' '$DEST/params' '$DEST/params.defaults'"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "${runtime_files[@]}" "$BOARD:$DEST/.upload/"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "for source in '$DEST/.upload/'*; do mv \"\$source\" '$DEST/'; done"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "mkdir -p '$DEST/.upload/assets/ui' '$DEST/assets/ui'"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "${ui_assets[@]}" "$BOARD:$DEST/.upload/assets/ui/"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" \
  "rm -f '$DEST/assets/ui/'*.png; for source in '$DEST/.upload/assets/ui/'*; do mv \"\$source\" '$DEST/assets/ui/'; done"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "$model" "$BOARD:$DEST/.upload/supercombo.kmodel"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "mv '$DEST/.upload/supercombo.kmodel' '$DEST/models/supercombo.kmodel'"
"${SCP_CMD[@]}" "${SSH_OPTIONS[@]}" "${param_files[@]/#/params/}" "$BOARD:$DEST/params.defaults/"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" \
  "for name in ${param_files[*]}; do test -e '$DEST/params/'\"\$name\" || cp '$DEST/params.defaults/'\"\$name\" '$DEST/params/'\"\$name\"; done"
"${SSH_CMD[@]}" "${SSH_OPTIONS[@]}" "$BOARD" "rm -rf '$DEST/.upload'; sync"
echo "Uploaded runtime files to $BOARD:$DEST"
echo "Reboot the board to run them (do not restart S35edgepilot; see docs/runtime.md)."
