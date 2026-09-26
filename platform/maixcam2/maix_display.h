#ifndef MAIXCAM2_DISPLAY_H
#define MAIXCAM2_DISPLAY_H

/* MaixCAM2 LCD를 VO 하드웨어 두 레이어로 쓴다. 레이어 0은 카메라 YUV의 가운데 4:3을
 * IVPS로 잘라 화면 크기로 줄여 올리고(비율 유지), 레이어 1(OSD)은 BGRA를 알파로 그 위에 합성한다. 세로
 * 패널(480x640)로의 90° 회전과 보드 disp_flip/disp_mirror도 VO가 처리한다.
 * AX 헤더는 이 파일 밖으로 새지 않는다. */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

class MaixDisplay {
public:
    static constexpr int kWidth = 640;
    static constexpr int kHeight = 480;

    MaixDisplay();
    ~MaixDisplay();
    MaixDisplay(const MaixDisplay &) = delete;
    MaixDisplay &operator=(const MaixDisplay &) = delete;

    /* CMM 블록(프레임 링 슬롯)의 NV12/NV21에서 가운데 4:3을 IVPS가 직접 잘라 화면에
     * 맞추고, source_still_valid()가 참일 때만 레이어 0에 올린다(그동안 소스가
     * 덮어써졌으면 찢어진 프레임을 내보내지 않는다). */
    bool show_video_phys(unsigned long long phys, int width, int height, bool nv21,
                         const std::function<bool()> &source_still_valid);
    /* 복사 없는 경로: begin_overlay가 캐시 가능한 CMM 블록(kWidth*4 stride BGRA)을
     * 빌려주고, 거기에 그린 뒤 end_overlay가 캐시를 밀어내고 레이어 1에 올린다. */
    uint8_t *begin_overlay();
    bool end_overlay();
    /* true면 가운데 4:3 크롭 대신 소스 전체를 비율 그대로 줄여 위아래를 검게 채운다
     * (camcal 미리보기). */
    void set_letterbox(bool letterbox) { letterbox_ = letterbox; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool letterbox_ = false;
};

#endif
