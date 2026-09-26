#!/bin/sh
# wpa_cli -a 훅($1=인터페이스, $2=이벤트). 다른 AP(예: 집 Wi-Fi -> 아이폰 핫스팟)에 붙으면
# 보드의 udhcpc가 이전 임대를 그대로 들고 있으므로, 반납(USR2) 후 새로 요청(USR1)한다.
[ "$2" = CONNECTED ] || exit 0
pid=$(cat "/run/udhcpc.$1.pid" 2>/dev/null) || exit 0
kill -USR2 "$pid" && sleep 1 && kill -USR1 "$pid"
logger -t wifi-dhcp-renew "$1 connected to $(wpa_cli -i "$1" status | sed -n "s/^ssid=//p"), DHCP renewed"
