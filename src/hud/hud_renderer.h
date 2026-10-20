#ifndef HUD_RENDERER_H
#define HUD_RENDERER_H

#include "common/model_output.h"
#include "common/projection.h"
#include "hud/hud_canvas.h"
#include "hud/hud_state.h"

#include <cstdint>
#include <vector>

// HUD를 그릴 BGRA8888(스트레이트 알파) 버퍼. width·height는 화면(가로 640x480) 크기, stride는
// 버퍼 행 바이트. 보드에서는 세로 패널 방향의 그림판이라 transpose다.
struct HudTarget {
    void *map = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t stride = 0;
    bool transpose = false;
};

// HUD 한 프레임을 그린다. 매번 전부 다시 그리며, 들고 있는 것은 행 커버리지 작업 공간과 지난번
// 그린 칸뿐이다. 지난번과 같은 버퍼를 받으면 그 칸만 지우고, 다른 버퍼는 전부 지운다(버퍼에는 이
// 렌더러만 쓴다고 본다).
class HudRenderer {
public:
    void draw(const HudTarget &target, const ParsedModelOutput &output, const ProjectionState &projection,
              const HudState &hud);
    // 마지막 draw가 그린 칸: 버퍼 행마다 (1 << last_tile_shift()) 폭 칸의 비트. 화면 쪽이 바뀐
    // 칸만 옮길 때 지난번 것과 합쳐 쓴다.
    const std::vector<uint32_t> &last_damage() const { return damage_; }
    int last_tile_shift() const { return tile_shift_; }

private:
    const void *map_ = nullptr;     // 지난번에 그린 버퍼
    std::vector<uint32_t> damage_;  // 그 버퍼에 그린 칸
    std::vector<uint16_t> coverage_;
    int tile_shift_ = 4;
};

// 터치 좌표(화면 px)가 오른쪽 위 상태 알약(녹화·와이파이) 쪽인지. 손가락 크기만큼 넉넉하게
// 본다. overlayd가 네트워크 카드를 여닫는 데 쓴다.
bool hud_status_touch(int x, int y, int width);
// 터치 좌표가 왼쪽 열(설정 속도 카드, 모드·조작 칩, 진단 카드)인지. 아래 모서리 TPMS 카드 위까지,
// 넉넉하게 본다. overlayd가 진단 카드를 켜고 끄는 데 쓴다.
bool hud_left_column_touch(int x, int y, int height);

#endif
