#ifndef OVERLAY_RENDERER_H
#define OVERLAY_RENDERER_H

#include "overlay_state.h"
#include "model_output.h"
#include "projection.h"

#include <array>
#include <cstdint>
#include <vector>

/* 깜빡이 애니메이션은 켜진 순간을 0으로 하는 단계 수로 그린다. 단계 진행은
 * overlayd가 시각 기준으로 계산하므로 재그리기 빈도에 영향받지 않는다. */
constexpr int kTurnSignalSteps = 25;

/* VO 레이어 1의 BGRA8888(스트레이트 알파) 버퍼. MaixCAM2 화면 640x480. */
struct OverlayTarget {
    void *map = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
};

/* HUD 한 프레임을 그린다. 매번 전부 다시 그리며, 들고 있는 것은 행 커버리지 작업 공간과
 * 버퍼마다 지난번 그린 칸뿐이다. VO 풀은 같은 블록 몇 개를 돌려 쓰므로, 아는 버퍼를 다시
 * 받으면 그 칸만 지우고 처음 보는 버퍼는 전부 지운다(버퍼에는 이 렌더러만 쓴다고 본다). */
class OverlayRenderer {
public:
    void draw(const OverlayTarget &target, const ParsedModelOutput &output,
              const ProjectionState &projection, const OverlayHudState &hud = OverlayHudState{});

private:
    struct BufferDamage {
        const void *map = nullptr;
        uint32_t width = 0;
        std::vector<uint16_t> rows;
    };
    // 풀 블록 수(3)보다 넉넉하게. 넘치면 먼저 기억한 버퍼부터 잊는다.
    std::array<BufferDamage, 4> damage_;
    uint32_t next_slot_ = 0;
    std::vector<uint16_t> coverage_;
};

/* 터치 좌표(화면 px)가 오른쪽 위 상태 알약(녹화·와이파이) 쪽인지. 손가락 크기만큼 넉넉하게
 * 본다. overlayd가 네트워크 카드를 여닫는 데 쓴다. */
bool hud_status_touch(int x, int y, int width);

#endif
