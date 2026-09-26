#!/usr/bin/env bash
# MaixCAM2에 부팅 자동 시작을 설치한다: edgepilot.service를 켜고 기본 UI 런처는 부팅 때
# 뜨지 않게 하며, Wi-Fi 재연결 때 DHCP를 새로 받는 wifi-dhcp-renew.service도 켠다.
# 되돌리기: scripts/install_autostart.sh --remove
# 사용: scripts/install_autostart.sh [--remove] [root@보드]   (설치 디렉터리 EDGEPILOT_BOARD_DIR=/root/edgepilot)
set -euo pipefail
cd "$(dirname "$0")/.."
REMOVE=0
if [ "${1:-}" = "--remove" ]; then REMOVE=1; shift; fi
BOARD="${1:-root@192.168.219.117}"
DEST="${EDGEPILOT_BOARD_DIR:-/root/edgepilot}"
SSH=(ssh -o StrictHostKeyChecking=no "$BOARD")

if [ "$REMOVE" = 1 ]; then
  "${SSH[@]}" "systemctl disable --now edgepilot.service; rm -f /etc/systemd/system/edgepilot.service /etc/systemd/system/camcal.service;
    systemctl daemon-reload; systemctl enable --now launcher.service"
  echo "autostart removed; stock launcher restored on $BOARD"
  exit 0
fi

# Wi-Fi가 다른 AP로 넘어갈 때 DHCP를 새로 받는 훅(보드 udhcpc는 이전 임대를 들고 있다).
"${SSH[@]}" "cat > /usr/local/sbin/wifi-dhcp-renew.sh && chmod 755 /usr/local/sbin/wifi-dhcp-renew.sh" \
  < scripts/wifi-dhcp-renew.sh
"${SSH[@]}" "cat > /etc/systemd/system/wifi-dhcp-renew.service && systemctl daemon-reload &&
  systemctl enable --now wifi-dhcp-renew.service" < scripts/wifi-dhcp-renew.service

sed "s|@DEST@|$DEST|g" scripts/edgepilot.service |
  "${SSH[@]}" "test -x '$DEST/manager.py' || { echo 'run scripts/upload_to_board.sh first' >&2; exit 1; }
    cat > /etc/systemd/system/edgepilot.service && systemctl daemon-reload &&
    systemctl disable launcher.service && systemctl enable edgepilot.service && sync"
# 카메라 캘리브레이션 캡처(수동 시작 전용, docs/camcal.md).
sed "s|@DEST@|$DEST|g" scripts/camcal.service |
  "${SSH[@]}" "cat > /etc/systemd/system/camcal.service && systemctl daemon-reload"
echo "autostart installed on $BOARD (starts at next boot, or: ssh $BOARD systemctl start edgepilot)"
