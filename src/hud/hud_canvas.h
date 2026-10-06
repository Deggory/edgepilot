#ifndef HUD_CANVAS_H
#define HUD_CANVAS_H

/* HUD를 그리는 바탕. 스트레이트 알파 BGRA 버퍼 위에 커버리지로 over 합성한다. 다각형은 행마다
 * 4개 부표본 행이 지나는 구간만 쌓아 칠하므로(안티앨리어싱) 일이 덮인 넓이에 비례하고, 합성은
 * 나눗셈 대신 역수 표를 쓴다. 그린 자리는 버퍼 행마다 가로 칸 비트(damage)로 남겨, 같은 버퍼를
 * 다시 받으면 그 칸만 지운다. 그리는 쪽은 늘 가로 화면 좌표를 쓰고, 버퍼가 세로 패널 방향이면
 * 캔버스가 좌표를 옮긴다(HudOrientation). OpenCV가 필요 없다. */

#include "hud/hud_font.h"

#include <cstdint>
#include <string_view>
#include <vector>

struct HudPoint {
    float x;
    float y;
};

// 0xAARRGGBB(스트레이트 알파)
constexpr uint32_t hud_argb(uint32_t a, uint32_t r, uint32_t g, uint32_t b)
{
    return a << 24 | r << 16 | g << 8 | b;
}

// 알파에 scale(0..1)을 곱한다.
constexpr uint32_t hud_fade(uint32_t color, float scale)
{
    const float alpha = static_cast<float>(color >> 24) * (scale < 0.0f ? 0.0f : scale > 1.0f ? 1.0f : scale);
    return static_cast<uint32_t>(alpha + 0.5f) << 24 | (color & 0x00ffffffu);
}

enum class HudAlign { left, center, right };

/* 화면 좌표(가로 width x height)를 버퍼 화소로 옮기는 법. 세로 패널에 바로 쓰는 버퍼는 transpose
 * (버퍼 행 = 화면 x)이고, 보드 disp_flip/disp_mirror만큼 버퍼 축을 뒤집는다. 기본은 그대로. */
struct HudOrientation {
    bool transpose = false;
    bool flip_x = false;  // 버퍼 열을 거꾸로
    bool flip_y = false;  // 버퍼 행을 거꾸로
};

class HudCanvas {
public:
    /* width·height는 화면(가로) 크기, stride는 버퍼 행 바이트. coverage는 버퍼 한 행 너비 이상의
     * 작업 공간(그릴 때마다 비워 둔다). damage는 이 버퍼의 행마다 그린 가로 칸(폭/16 이상, 최소
     * 64 px)의 비트로, 버퍼와 함께 이어 쓴다. */
    HudCanvas(void *pixels, int width, int height, int stride_bytes, std::vector<uint16_t> &coverage,
                  std::vector<uint16_t> &damage, HudOrientation orientation = {});

    int width() const { return width_; }
    int height() const { return height_; }
    int tile_shift() const { return tile_shift_; }  // damage 칸 폭 = 1 << tile_shift()

    /* 비운다. only_damaged면 damage에 남은 칸(이 버퍼에 지난번 그린 자리)만 지우고, 아니면
     * 전부 지운다(처음 받은 버퍼). */
    void clear(bool only_damaged);
    void fill_rect(int x, int y, int w, int h, uint32_t color);
    /* 모서리만 다각형으로 칠한다. 좌표가 모두 정수면 모서리 사이 곧은 행은 fill_rect로 바로
     * 칠한다(카드·칩 대부분). */
    void fill_round_rect(float x, float y, float w, float h, float radius, uint32_t color);
    /* 다각형(짝홀 규칙)을 칠한다. 도로 띠처럼 먼 쪽을 흐리게 하려면 fade_far를 쓰는데, 화면
     * near_y 높이에서 원래 알파, far_y 높이에서 far_alpha배가 되도록 줄인다. */
    void fill_polygon(const HudPoint *points, int count, uint32_t color);
    void fill_polygon_faded(const HudPoint *points, int count, uint32_t color, float near_y,
                            float far_y, float far_alpha);
    /* 글자 상자의 왼쪽 위(정렬 기준점 x, 줄 위 y)에 쓴다. 카드 없이 영상 위에 쓰는 글은
     * shadow로 1 px 아래 오른쪽에 그림자를 깐다. 쓴 폭을 돌려준다. */
    int text(int x, int y, std::string_view text, const HudFont &font, uint32_t color,
             HudAlign align = HudAlign::left, bool shadow = false);

private:
    // 아래는 모두 버퍼 좌표
    uint32_t *row(int y) const;
    HudPoint to_buffer(HudPoint p) const;  // 화면 점 → 버퍼 점(연속 좌표)
    void mark(int y, int x0, int x1);  // y행 [x0, x1)을 그렸다
    void fill_buffer_rect(int x0, int y0, int x1, int y1, uint32_t color);
    void fill_rows(const HudPoint *points, int count, uint32_t color, float near_y, float far_y,
                   float far_alpha);
    void glyphs(int x, int y, std::string_view text, const HudFont &font, uint32_t color);

    uint8_t *base_;
    int width_;      // 화면
    int height_;
    int buffer_w_;   // 버퍼
    int buffer_h_;
    int stride_;
    HudOrientation orientation_;
    std::vector<uint16_t> &coverage_;  // 한 행의 부표본 커버리지 합(0..256)
    std::vector<uint16_t> &damage_;    // 행마다 그린 칸 비트
    int tile_shift_ = 6;               // 칸 폭 = 1 << tile_shift_
};

#endif
