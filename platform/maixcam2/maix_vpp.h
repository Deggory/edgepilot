#ifndef MAIXCAM2_MAIX_VPP_H
#define MAIXCAM2_MAIX_VPP_H

// IVPS VPP로 NV12 프레임의 한 영역을 잘라 다른 CMM 버퍼에 늘이거나 줄여 쓴다(bilinear). 차선선 플러그인이 프레임 링
// 슬롯에서 모델 입력 텐서로 바로 쓰는 데 쓴다(CPU가 화소를 만지지 않는다). AX_IVPS_Init은 같은 프로세스의
// GdcWarp가 해 둔 뒤에 부른다.

#include <cstdint>
#include <mutex>

// 한 프로세스의 VPP 동기 호출은 한 번에 하나씩: 광류 피라미드(작업 스레드)와 차선선 플러그인(주 스레드)이 같은
// IVPS 작업 큐를 쓴다.
inline std::mutex &vpp_mutex() {
    static std::mutex mutex;
    return mutex;
}

struct VppNv12 {
    uint64_t phys = 0;  // Y 평면 시작. UV 평면은 바로 뒤(행 간격 = 폭)
    int width = 0;
    int height = 0;
    bool chroma_vu = false;  // NV21(V 먼저)
};

// src의 (x, y, w, h) 영역을 dst 크기로 bilinear 리사이즈해 NV12로 쓴다. 영역과 크기는 짝수여야 한다.
// 0이면 성공, 아니면 IVPS 오류 코드.
int vpp_crop_resize_nv12(const VppNv12 &src, int x, int y, int w, int h, const VppNv12 &dst);

#endif
