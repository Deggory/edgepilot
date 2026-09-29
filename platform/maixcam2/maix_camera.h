#ifndef MAIXCAM2_CAMERA_H
#define MAIXCAM2_CAMERA_H

/* MaixCAM2 카메라(VI)를 C++에서 직접 연다. libmaixcam_lib의 ax_middleware 클래스를
 * 쓰며 AX 헤더는 이 파일 밖으로 새지 않는다. AI-ISP(NPU 한 코어)는 늘 켠다.
 * 센서는 fps로 돌리고 자동 노출은 켜 둔다. */

#include <cstdint>
#include <memory>

class MaixCamera {
public:
    MaixCamera(int width, int height, int fps, bool nv12);
    ~MaixCamera();
    MaixCamera(const MaixCamera &) = delete;
    MaixCamera &operator=(const MaixCamera &) = delete;

    /* 다음 프레임을 IVPS(TDP) 하드웨어 복사로 dst_phys(NV12 CMM 블록)에 옮긴다.
     * timeout·실패면 false. age_us가 있으면 복사를 마친 시점에 이 프레임의 하드웨어
     * PTS(VI가 찍음)로부터 지난 시간을 적는다(AX_SYS_GetCurPTS와 같은 µs 시계).
     * pop_age_us는 VI에서 꺼낸 직후의 나이다. */
    bool read_to(unsigned long long dst_phys, int timeout_ms = 1000,
                 uint64_t *age_us = nullptr, uint64_t *pop_age_us = nullptr);
    int width() const { return width_; }
    int height() const { return height_; }
    uint32_t fourcc() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    int width_;
    int height_;
    bool nv12_;
};

#endif
