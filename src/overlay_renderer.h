#ifndef OVERLAY_RENDERER_H
#define OVERLAY_RENDERER_H

#include "overlay_canvas.h"
#include "overlay_state.h"
#include "model_output.h"
#include "projection.h"

#include <array>
#include <cstdint>
#include <vector>

/* HUD를 그릴 BGRA8888(스트레이트 알파) 버퍼. width·height는 화면(가로 640x480) 크기, stride는
 * 버퍼 행 바이트. 보드에서는 세로 패널 방향의 그림판이라 orientation이 transpose다. */
struct OverlayTarget {
    void *map = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    HudOrientation orientation{};
};

/* HUD 한 프레임을 그린다. 매번 전부 다시 그리며, 들고 있는 것은 행 커버리지 작업 공간과
 * 버퍼마다 지난번 그린 칸뿐이다. VO 풀은 같은 블록 몇 개를 돌려 쓰므로, 아는 버퍼를 다시
 * 받으면 그 칸만 지우고 처음 보는 버퍼는 전부 지운다(버퍼에는 이 렌더러만 쓴다고 본다). */
class OverlayRenderer {
public:
    void draw(const OverlayTarget &target, const ParsedModelOutput &output,
              const ProjectionState &projection, const OverlayHudState &hud = OverlayHudState{});
    /* 마지막 draw가 그린 칸: 버퍼 행마다 (1 << last_tile_shift()) 폭 칸의 비트. 화면 쪽이 바뀐
     * 칸만 옮길 때 지난번 것과 합쳐 쓴다. */
    const std::vector<uint16_t> &last_damage() const { return *last_damage_; }
    int last_tile_shift() const { return last_tile_shift_; }

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
    const std::vector<uint16_t> *last_damage_ = &damage_[0].rows;
    int last_tile_shift_ = 6;
};

/* 터치 좌표(화면 px)가 오른쪽 위 상태 알약(녹화·와이파이) 쪽인지. 손가락 크기만큼 넉넉하게
 * 본다. overlayd가 네트워크 카드를 여닫는 데 쓴다. */
bool hud_status_touch(int x, int y, int width);
/* 터치 좌표가 왼쪽 열(설정 속도 카드, 모드·조작 칩, 진단 카드)인지. 아래 모서리 TPMS 카드 위까지,
 * 넉넉하게 본다. overlayd가 진단 카드를 켜고 끄는 데 쓴다. */
bool hud_left_column_touch(int x, int y, int height);

#endif
