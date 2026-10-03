#ifndef MAIXCAM2_TOUCH_H
#define MAIXCAM2_TOUCH_H

/* MaixCAM2 터치스크린(hyn_ts, 멀티터치 B형)에서 짧은 탭만 화면 좌표(MaixDisplay 640x480)로
 * 알려 준다. 패널은 480x640 세로라 MaixCDK(maix_touchscreen_maixcam2.hpp)처럼 시계 방향
 * 90°로 돌린다. 장치는 막지 않고(O_NONBLOCK) 읽으며 첫 손가락(슬롯 0)만 본다. */

#include <cstdint>

class MaixTouch {
public:
    MaixTouch() = default;
    ~MaixTouch();
    MaixTouch(const MaixTouch &) = delete;
    MaixTouch &operator=(const MaixTouch &) = delete;

    // 장치가 없으면 false. 그 뒤 poll_tap은 아무것도 하지 않는다.
    bool open();
    // 쌓인 이벤트를 읽고, 그사이 끝난 탭이 있으면 누른 자리와 true.
    bool poll_tap(int *x, int *y);

private:
    void to_screen(int *x, int *y) const;

    int fd_ = -1;
    int raw_x_max_ = 0;
    int slot_ = 0;
    int raw_x_ = 0;  // 슬롯 0의 최근 좌표(패널 기준)
    int raw_y_ = 0;
    bool touching_ = false;  // BTN_TOUCH
    bool pressed_ = false;   // 마지막 SYN_REPORT 때의 touching_
    int down_x_ = 0;         // 누른 순간의 화면 좌표
    int down_y_ = 0;
    uint64_t down_us_ = 0;
};

#endif
