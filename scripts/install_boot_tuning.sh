#!/usr/bin/env bash
# MaixCAM2 부팅을 빠르게 한다(docs/boot-time.md). 순정 이미지에서 edgepilot이 늦게 뜨는 이유는 세 가지다:
#   1) AX 드라이버를 rc.local에서 올리는데 rc-local.service는 Wi-Fi 연결(network-online) 뒤에 돈다.
#   2) 쓰지 않는 서비스(tailscale·nginx·bluetooth·ttyd·maixvision·rsyslog)가 같은 시각에 떠서
#      느린 SD와 CPU 두 개를 나눠 쓴다.
#   3) 영구 journal이 700MB 가까이 쌓였고, 선 없는 eth0의 dhclient와 avahi-monitor가 몇 초마다 로그를 쓴다.
# 모두 되돌릴 수 있다: scripts/install_boot_tuning.sh --remove [root@보드]
# 사용: scripts/install_boot_tuning.sh [--remove] [root@보드]   (적용은 다음 부팅부터)
set -euo pipefail
cd "$(dirname "$0")/.."
REMOVE=0
if [ "${1:-}" = "--remove" ]; then REMOVE=1; shift; fi
BOARD="${1:-root@192.168.219.117}"
SSH=(ssh -o StrictHostKeyChecking=no "$BOARD")

# 순정 이미지에서 켜져 있지만 edgepilot에 필요 없는 서비스. bluetooth와 rsyslog는 D-Bus·소켓으로
# 다시 깨어나므로 mask한다.
DISABLE="tailscaled.service nginx.service ttyd.service maixvision-server.service avahi-monitor.service"
MASK="bluetooth.service rsyslog.service syslog.socket"

if [ "$REMOVE" = 1 ]; then
  "${SSH[@]}" "set -e
    systemctl disable edgepilot-drivers.service 2>/dev/null || true
    rm -f /etc/systemd/system/edgepilot-drivers.service /etc/systemd/journald.conf.d/edgepilot.conf \
      /etc/systemd/system.conf.d/edgepilot.conf
    [ -f /etc/rc.local.edgepilot-orig ] && cp -p /etc/rc.local.edgepilot-orig /etc/rc.local
    sed -i 's/^#allow-hotplug eth0  # edgepilot$/allow-hotplug eth0/' /etc/network/interfaces
    systemctl unmask $MASK
    systemctl daemon-reload
    systemctl enable $DISABLE bluetooth.service rsyslog.service
    sync"
  echo "boot tuning removed on $BOARD (takes effect at next boot)"
  exit 0
fi

"${SSH[@]}" "cat > /etc/systemd/system/edgepilot-drivers.service" < scripts/edgepilot-drivers.service
"${SSH[@]}" "set -e
  # rc.local은 드라이버가 이미 올라와 있으면(edgepilot-drivers.service) 같은 insmod를 건너뛴다.
  # npu_set_bw_limiter.sh는 두 번 돌아도 된다(이미 등록돼 있으면 그냥 끝난다).
  [ -f /etc/rc.local.edgepilot-orig ] || cp -p /etc/rc.local /etc/rc.local.edgepilot-orig
  sed -i -E 's#^(bash /soc/scripts/auto_load_all_drv\.sh)\$#[ -d /sys/module/ax_sys ] || \1#' /etc/rc.local
  grep -q '^\[ -d /sys/module/ax_sys \] || bash /soc/scripts/auto_load_all_drv.sh' /etc/rc.local
  # 선 없는 eth0에 dhclient가 3초마다 DISCOVER를 보내고 그때마다 로그를 쓴다(필요하면 ifup eth0).
  sed -i 's/^allow-hotplug eth0\$/#allow-hotplug eth0  # edgepilot/' /etc/network/interfaces
  mkdir -p /etc/systemd/journald.conf.d
  printf '[Journal]\n# 느린 SD에 쌓이는 영구 journal을 줄인다(이전 부팅 로그는 남는다).\nSystemMaxUse=64M\n' \
    > /etc/systemd/journald.conf.d/edgepilot.conf
  # 종료 마지막 단계(systemd-shutdown)가 멈추면 순정 설정으론 하드웨어 워치독이 10분 뒤에야 리셋한다
  # (2026-10-01 재부팅이 SIGTERM 단계에서 멈췄다). 30초로 줄인다.
  mkdir -p /etc/systemd/system.conf.d
  printf '[Manager]\nRebootWatchdogSec=30s\n' > /etc/systemd/system.conf.d/edgepilot.conf
  systemctl daemon-reload
  systemctl enable edgepilot-drivers.service
  systemctl disable $DISABLE
  systemctl mask $MASK
  systemctl restart systemd-journald
  journalctl --vacuum-size=64M >/dev/null
  rm -f /var/log/syslog.[0-9]* /var/log/syslog.*.gz
  sync"
echo "boot tuning installed on $BOARD (takes effect at next boot)"
