#ifndef ALERT_SOUND_H
#define ALERT_SOUND_H

/* HUD 알림음(alert_tones.h)을 MaixCAM2 보드 스피커로 낸다.
 *
 * aplay 하나를 시작할 때 띄워 표준입력(raw 48 kHz 스테레오 S16)을 계속 열어 두고, 소리
 * 스레드가 평소엔 무음을, 알림이 오면 그 멜로디를 20 ms 단위로 흘려 넣는다. 알림 순간에
 * 프로세스를 띄우거나 디스크를 읽지 않으므로 호출한 화면 루프가 멈추지 않고(예전엔 알림마다
 * aplay를 띄웠는데, vfork 동안 부모가 멈춰 SD가 바쁠 때 HUD가 몇 초씩 얼었다), 스피커
 * 앰프가 계속 켜져 있어 첫 음이 잘리지 않는다. 파이프와 ALSA 버퍼를 작게 잡아 지연은
 * 0.1초 안쪽이다. 새 알림은 재생 중인 알림을 끊는다. aplay가 죽으면 소리 스레드가 다시
 * 띄운다.
 * 크기는 웹 기기 설정(params/display.json의 alert_volume_percent)으로 실행 중에 바꾼다.
 * 환경: EDGEPILOT_ALERT_SOUND=0이면 끈다, EDGEPILOT_ALERT_VOLUME(0~100, 기본 70: 설정
 * 파일에 값이 없을 때의 크기), EDGEPILOT_ALERT_PCM(ALSA 장치, 기본 plughw:0,1). */

#include "alert_tones.h"

#include <atomic>
#include <cstdint>
#include <string>
#include <sys/types.h>
#include <thread>
#include <vector>

class AlertSound {
public:
    AlertSound();
    ~AlertSound();
    AlertSound(const AlertSound &) = delete;
    AlertSound &operator=(const AlertSound &) = delete;

    // 스레드에 알리기만 하고 곧바로 돌아온다.
    void play(AlertSoundId id);
    bool enabled() const { return enabled_; }
    // 0~100. 다음 20 ms 조각부터 적용된다.
    void set_volume_percent(float percent);
    float volume_percent() const { return volume_.load() * 100.0f; }

private:
    void loop();
    bool start_player();
    void stop_player();

    bool enabled_ = false;
    std::string pcm_;
    std::vector<std::vector<int16_t>> clips_;  // 알림별 인터리브 스테레오 샘플
    std::atomic<int> pending_{-1};
    std::atomic<float> volume_{0.7f};  // 0~1, clips_(100% 크기)에 곱한다
    std::atomic<bool> stop_{false};
    pid_t player_ = -1;
    int pipe_fd_ = -1;
    std::thread thread_;
};

#endif
