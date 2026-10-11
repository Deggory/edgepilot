#ifndef MAIXCAM2_IVE_FLOW_H
#define MAIXCAM2_IVE_FLOW_H

/* 광류 작업(model/flow_worker: 광류 보정과 앞차 확대율)의 하드웨어 부분. 프레임마다 축소 단(1/2, 1/4)을 VPP
 * 한 번으로 만들고(modeld가 GDC 때문에 쥐고만 있던 VPP 작업 큐), 측정 창 시작 프레임에서 IVE로 Shi-Tomasi
 * 응답을 내고, IVE 피라미드 LK로 점을 한 프레임씩(앞뒤 검사는 거꾸로도) 따라간다. 원본 단은 링 슬롯을 복사하지
 * 않고 그대로 읽는다. CPU는 호출과 좌표 변환만 한다(프레임당 약 1 ms).
 *
 * IVE는 헤더 문서와 다르게 동작한다(2026-10 보드 실측, docs/flow-calibration.md).
 *  - LK 시작점은 가장 거친 단의 좌표로 준다(원본 좌표 / 2^(단 수 - 1)). 출력은 원본 좌표다.
 *  - 전체 이동량은 ±8 px에서 잘리고, 음수 이동은 +16 px로 나온다. 그래서 한 프레임씩 따라가고, 되돌리고,
 *    kMaxStepPx를 넘는 점은 버린다. bUseInitFlow로도 범위는 늘지 않는다.
 *  - CornerDet(ST)의 점 목록은 늘 비어 있다. 응답 영상은 5x5 창 최소 고유값과 정확히 같으므로 점은
 *    pick_flow_corners가 고른다. Sobel은 U16C1 출력만 받지만 값은 부호가 있다(CornerDet에는 S16C1로 준다).
 *  - AX_IVE_Query(block)는 작업이 안 끝났어도 몇 ms 뒤 AX_ERR_IVE_QUERY_TIMEOUT(0x80150054)을 낸다.
 *    그러면 다시 묻는다. 묻지 않고 넘어가면 다음 작업들이 줄줄이 실패한다.
 *  - IVPS 동기 API는 AX_IVPS_Init을 부른 프로세스마다 TDP·VPP 작업 큐를 한 쌍씩 잡는데 보드 전체에
 *    넷뿐이다(camerad·modeld·recordd·overlayd가 다 쓴다). 다섯째 프로세스(도구)는 BUSY라서, VPP가
 *    실패하면 IVE Filter(5x5 가우시안) + DMA 간격 복사로 물러난다(docs/flow-calibration.md).
 * AX_SYS_Init·AX_IVPS_Init은 같은 프로세스(modeld의 AxEngineSession·GdcWarp)가 해 둔 뒤에 만든다. AX 헤더는
 * 이 파일 밖으로 새지 않는다. */

#include <cstddef>
#include <cstdint>
#include <memory>

class IveFlow {
public:
    static constexpr int kLevels = 3;          // LK 피라미드(원본, 1/2, 1/4)
    static constexpr int kBandTop = 88;        // 코너를 찾는 1/2 단의 띠(하늘·보닛 제외, 16의 배수 높이)
    static constexpr int kBandRows = 240;
    static constexpr float kMaxStepPx = 7.5f;  // 한 프레임 이동 한계(하드웨어는 8 px에서 자른다)
    static constexpr int kMaxPoints = 500;     // IVE LK·CornerDet의 한계

    IveFlow(int width, int height);  // 실패하면 std::runtime_error
    ~IveFlow();
    IveFlow(const IveFlow &) = delete;
    IveFlow &operator=(const IveFlow &) = delete;

    /* NV12/NV21 프레임(Y 평면 y_phys, 행 간격 stride, UV 평면 uv_phys; 0이면 Y 바로 뒤)으로 축소 단을 만든다.
     * 성공하면 이 프레임이 '현재', 앞의 현재 프레임이 '이전'이 된다. 원본 단은 복사하지 않으므로 두 프레임의
     * 버퍼는 다음 track()이 끝날 때까지 그대로여야 한다(링 슬롯이면 호출자가 seqlock으로 확인한다). */
    bool push(unsigned long long y_phys, int stride, unsigned long long uv_phys = 0);
    // 축소 단을 만드는 엔진: "vpp", "vpp-nv12"(YUV400 출력을 못 낼 때) 또는 "ive"(VPP를 못 쓸 때)
    const char *pyramid_engine() const;
    /* 현재 프레임 1/2 단 띠(kBandRows행, 폭 width/2)의 코너 응답. 다음 호출까지 유효하다. 실패하면 nullptr. */
    const int16_t *corner_response();
    /* 이전 프레임의 점 n개(원본 좌표)를 현재 프레임으로 따라간다. backward면 거꾸로 현재 프레임의 점을 이전
     * 프레임으로 따라간다(앞뒤 검사). ok[i]가 0이면 놓쳤거나 한계를 넘었다. IVE 호출이 실패하면 false. */
    bool track(const float *xy, int n, float *out_xy, uint8_t *ok, bool backward = false);

    int width() const { return width_; }
    int height() const { return height_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    int width_ = 0;
    int height_ = 0;
};

#endif
