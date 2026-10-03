# 정차 출발 알림

[← Documentation index](../README.md)

## 동작

`controlsd`가 정차 중 두 종류의 이벤트를 판단한다.

- 선행 차량 출발: 모델의 vision lead 거리가 기준보다 0.5 m 이상 증가하고
  상대속도가 0.5 m/s를 넘는 상태가 0.3초 유지되면 알린다.
- 신호 변경 추정: 모델이 여기서 멈추겠다는 짧은 경로(5 m 미만)를 예측한 채
  3초 넘게 정차하면 신호 대기로 보고 적색 신호등을 표시한다. 이후 경로가 10 m
  넘게 0.3초 열리면 신호 변경을 알린다. 앞차가 있어도 무장한다. K230의 v0.9.4
  모델에서는 lead 확률이 앞차 유무를 가르지 못했고, 앞차가 출발하면 경로도 같은
  식으로 열리기 때문이다(master 모델에서 lead 확률을 다시 확인하지는 않았다).
  같은 프레임에 선행 차량 출발도 잡히면 그쪽 알림이 뜬다.

한 번 정차할 때 하나의 이벤트만 발생한다. 기어가 `D`가 아니거나 가속 페달을
누르거나 차량이 다시 움직이면 다음 정차 주기를 준비한다. 알림은 engage 여부와
무관하며 LCD에 3초 동안 표시된다.

K230에서 passive piezo로 내던 알람은 MaixCAM2 보드 스피커로 낸다(`src/alert_sound.cc`).
멜로디(음높이·길이)는 K230 부저와 같고, 음색은 사인파에 약한 배음을 섞었다.
`overlayd`가 시작할 때 `aplay` 하나를 띄워 두고 소리 스레드가 평소엔 무음을, 알림이 오면
멜로디를 흘려 넣는다. 알림 순간에 프로세스를 띄우지 않으므로 화면 루프가 멈추지 않고,
앰프가 계속 켜져 있어 첫 음이 잘리지 않는다(지연 0.1초 안쪽).
모든 알림은 로그 줄(`overlayd: alert=<이름>`)로도 남고, 화면에는 출발 알림과
engage 거부 토스트가 뜬다. 알림은 다음 상태 전이에 울린다.

- `signal_changed`: 선행 차량 출발 또는 신호 변경
- `engage`: 제어 engage 성공
- `disengage`: 제어 disengage 또는 fault에 의한 해제
- `unavailable`: 제어/Panda 상태가 stale이 되거나 Panda/조향 fault가 검출됨
- `unable`: engage 조건을 만족하지 못한 상태에서 engage 명령을 거부할 때

실제 차량/제어 조건으로 engage가 거부되면 LCD 하단 알림 카드에 `UNABLE TO ENGAGE`와
그 아래 줄에 사유를 3초간 표시한다. openpilot의 refuse 알림에 대응한다.
Panda의 `not ready`/`controls off`는 SET edge와 health 응답 사이의 정상적인
비동기 구간이므로 최대 1초 동안 대기한다. 그 사이 Panda 허가가 들어오면
`engage`만 한 번 알린다. 허가가 끝내 오지 않거나 Panda 회복 뒤 다른 정적 조건이
남아 있으면 그때 `UNABLE`로 거부한다.

조향을 넘겨받으라는 경고(`overlayd: alert=take_control`)는 켜지는 순간 `unable` 소리를 한 번
내고, 켜져 있는 동안 LCD 하단에 띄운다. openpilot의 두 알림에 대응한다.

- `TAKE CONTROL: CALIBRATING / RECALIBRATING / CALIB INVALID`: 카메라 캘리브레이션이
  완료 상태가 아니다. 이 상태에서는 engage를 거부한다(`UNABLE TO ENGAGE` / `CALIBRATING`).
  engage 중에 이 상태가 되면(마운트 변경 감지 등) 경고를 띄운 채 3초 더 조향하고
  해제한다(openpilot의 soft disable). 3초 안에 완료로 돌아오면 해제하지 않는다.
- `TAKE CONTROL` / `Turn exceeds steering limit`(openpilot steerSaturated): 시속 36 km 이상에서
  목표 곡률이 횡가속 한계(3.3 m/s²)에 잘리거나 출력이 한계에 붙은 채 0.4초 넘게 이어지고,
  목표 횡가속이 1 m/s²를 넘으며 실제의 1.2배 이상인데, 최근 2초 안에 핸들을 잡지 않았을
  때다. 2026-09-25~27 실차 3회 주행에서는 한 번도 해당하지 않았다.

K230의 피에조 환경 변수(`K230_PIEZO_BUZZER`, `K230_PIEZO_PIN`)는 없어졌고, 대신
`EDGEPILOT_ALERT_SOUND=0`(끄기), `EDGEPILOT_ALERT_VOLUME`(0~100, 기본 70), `EDGEPILOT_ALERT_PCM`
(ALSA 장치, 기본 `plughw:0,1`)를 쓴다. 크기는 웹 `기기 설정`의 알림음 크기
(`params/display.json`의 `alert_volume_percent`)로 실행 중에 바꾸며, 이 값이 있으면 환경 변수보다 우선한다.

## 판단 근거

신호 대기 판별에 정지선 출력을 쓰지 않는다. 배포 모델(openpilot master
supercombo, K230에서는 v0.9.4)이 `stop_lines`를 내지 않기 때문이며, 대신 "모델이
정지 계획"을 근거로 삼는다.
알림 발동 조건은 정지선과 무관하게 모델 경로가 10 m 넘게 열리는 것이라, 무장이
헛나가도 길이 실제로 열리지 않는 한 알림은 뜨지 않는다.

현재 차량의 레이더 입력은 사용하지 않는다. 모델의 lead x에서 카메라-레이더 기준 거리 1.52 m를 빼고, 모델 lead 속도에서 자차 속도를
빼 상대속도를 계산한다. lead 확률은 50% 이상이어야 하며, vision 거리 노이즈로 인한
오경보를 막기 위해 거리와 상대속도 조건을 0.3초 함께 확인한다.

## 계기판 차임 실차 검증

현재 형상은 계기판 차임 CAN을 송신하지 않는다.

- C2의 `chimeAtResume`은 차량 계기판이 아니라 장치의 `soundd`가
  `dingdong.wav`를 재생한다.
- K7의 `LKAS11`(`0x340`)에 있는 `CF_Lkas_SysWarning`은 독립 차임이 아니라
  조향/LDWS 경고 상태다. 차임 용도로 변경하면 아이콘 점멸이나 경고등을 만들 수
  있다.

2026-07-30에 K7 YG HEV 실차에서(K230 형상) 다음 조건을 확인했다.

- 순정 `LFAHDA_MFC`(`0x485`)는 카메라 bus 2에서 `03 00 00 00`으로 약 20 Hz
  수신된다.
- 순정 포워딩을 억제한 뒤 `HDA_Chime`을 300 ms 동안 단독 송신했다.
- 그랜저 IG에서 쓰는 `LKAS11.CF_Lkas_SysWarning=9`를 송신했다.
- C2의 K7 YG 경로와 같은 `CF_Lkas_SysWarning=3`,
  `CF_Lkas_LdwsSysState=3`을 2초 동안 송신했다.
- 마지막 시험은 기어 `D`, 클러스터 속도와 네 바퀴 속도 모두 `0 km/h`에서
  수행했다.
- 모든 시험에서 Panda 차단, CAN 송신 오류, checksum 오류는 없었지만 계기판
  표시와 차임 모두 반응하지 않았다.

따라서 K7 YG HEV에서는 이 두 CAN 경로를 계기판 차임으로 사용하지 않는다.
정차 출발 알림은 LCD 표시와 보드 스피커를 사용한다(K230에서는 보드 PWM 피에조였다).
