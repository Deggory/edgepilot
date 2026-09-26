#ifndef ALERT_SOUND_H
#define ALERT_SOUND_H

/* HUD 알림음. K230 피에조 부저의 멜로디(음높이·길이)를 그대로 옮겨 MaixCAM2 보드
 * 스피커로 낸다.
 *
 * aplay 하나를 시작할 때 띄워 표준입력(raw 48 kHz 스테레오 S16)을 계속 열어 두고, 소리
 * 스레드가 평소엔 무음을, 알림이 오면 그 멜로디를 20 ms 단위로 흘려 넣는다. 알림 순간에
 * 프로세스를 띄우거나 디스크를 읽지 않으므로 호출한 화면 루프가 멈추지 않고(예전엔 알림마다
 * aplay를 띄웠는데, vfork 동안 부모가 멈춰 SD가 바쁠 때 HUD가 몇 초씩 얼었다), 스피커
 * 앰프가 계속 켜져 있어 첫 음이 잘리지 않는다. 파이프와 ALSA 버퍼를 작게 잡아 지연은
 * 0.1초 안쪽이다. 새 알림은 재생 중인 알림을 끊는다. aplay가 죽으면 소리 스레드가 다시
 * 띄운다.
 * 환경: K230_ALERT_SOUND=0이면 끈다, K230_ALERT_VOLUME(0~100, 기본 70),
 * K230_ALERT_PCM(ALSA 장치, 기본 plughw:0,1). */

#include <atomic>
#include <cstdint>
#include <string>
#include <sys/types.h>
#include <thread>
#include <vector>

enum class AlertSoundId { unable, engage, disengage, signal_changed, unavailable, count };

class AlertSound {
public:
    AlertSound();
    ~AlertSound();
    AlertSound(const AlertSound &) = delete;
    AlertSound &operator=(const AlertSound &) = delete;

    // 스레드에 알리기만 하고 곧바로 돌아온다.
    void play(AlertSoundId id);
    bool enabled() const { return enabled_; }

private:
    void loop();
    bool start_player();
    void stop_player();

    bool enabled_ = false;
    std::string pcm_;
    std::vector<std::vector<int16_t>> clips_;  // 알림별 인터리브 스테레오 샘플
    std::atomic<int> pending_{-1};
    std::atomic<bool> stop_{false};
    pid_t player_ = -1;
    int pipe_fd_ = -1;
    std::thread thread_;
};

#endif
