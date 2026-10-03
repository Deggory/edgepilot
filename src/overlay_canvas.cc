#include "overlay_canvas.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <initializer_list>

namespace {

constexpr int kSubRows = 4;
constexpr int kSubRowWeight = 256 / kSubRows;
constexpr int kMaxCrossings = 16;
constexpr int kMaxEdges = 72;  // 궤적 띠(33쌍 = 66변)가 들어가는 크기

// 다각형 변: 위 끝(작은 y)에서 아래 끝까지, 위 끝의 x와 기울기 dx/dy.
struct Edge {
    float top;
    float bottom;
    float x;
    float slope;
};
constexpr int kCornerSegments = 6;  // 반지름 6 px 이상 모서리의 선분 수(작은 모서리는 덜 쓴다)
constexpr uint32_t kTextShadow = hud_argb(150, 0, 0, 0);

// 사분원 위 점 (cos, sin), 0°..90°를 kCornerSegments 등분. 모서리마다 돌려 쓴다.
const std::array<HudPoint, kCornerSegments + 1> kQuarterArc = [] {
    std::array<HudPoint, kCornerSegments + 1> arc{};
    for (int i = 0; i <= kCornerSegments; ++i) {
        const float angle = static_cast<float>(i) / kCornerSegments * 1.5707964f;
        arc[i] = {std::cos(angle), std::sin(angle)};
    }
    return arc;
}();

// 1/x를 16비트 고정소수점으로(x = 1..255).
constexpr std::array<uint32_t, 256> kInverse = [] {
    std::array<uint32_t, 256> table{};
    for (uint32_t i = 1; i < 256; ++i) table[i] = (65536 + i / 2) / i;
    return table;
}();

inline uint32_t div255(uint32_t x)
{
    return (x + 128 + ((x + 128) >> 8)) >> 8;
}

/* dst 위에 rgb를 알파 a(0..255)로 over 합성한다(둘 다 스트레이트 알파). 투명한 바탕(대부분의
 * 화소)과 불투명한 위 색은 바로 쓴다. 분자는 255·oa를 넘지 않아 역수를 곱해도 넘치지 않는다.
 * 화소 루프마다 펼쳐지도록 늘 인라인한다(그냥 두면 GCC가 호출로 남겨 화소마다 호출한다). */
[[gnu::always_inline]] inline void blend(uint32_t &dst, uint32_t rgb, uint32_t a)
{
    if (a == 0) return;
    const uint32_t da = dst >> 24;
    if (a == 255 || da == 0) {
        dst = a << 24 | rgb;
        return;
    }
    const uint32_t dw = div255(da * (255 - a));
    const uint32_t oa = a + dw;
    const uint32_t inverse = kInverse[oa];
    auto channel = [&](int shift) {
        return ((((rgb >> shift) & 0xff) * a + ((dst >> shift) & 0xff) * dw) * inverse + 32768) >> 16;
    };
    dst = oa << 24 | channel(16) << 16 | channel(8) << 8 | channel(0);
}

/* 한 행의 부표본 커버리지 합(화소마다 0..256)과 덮인 화소 범위 [min, max]. 칸은 합성하며
 * 0으로 되돌린다. */
struct RowCoverage {
    uint16_t *cells;
    int width;
    int min;
    int max;

    // 부표본 행 하나의 구간 [x0, x1)을 더한다. 양 끝 화소는 덮인 비율만큼.
    [[gnu::always_inline]] void add(float x0, float x1)
    {
        x0 = std::max(x0, 0.0f);
        x1 = std::min(x1, static_cast<float>(width));
        if (x1 <= x0) return;
        const int i0 = static_cast<int>(x0);
        const int i1 = std::min(static_cast<int>(x1), width - 1);
        if (i0 == i1) {
            cells[i0] += static_cast<uint16_t>((x1 - x0) * kSubRowWeight + 0.5f);
        } else {
            cells[i0] += static_cast<uint16_t>((static_cast<float>(i0 + 1) - x0) * kSubRowWeight + 0.5f);
            for (int x = i0 + 1; x < i1; ++x) cells[x] += kSubRowWeight;
            cells[i1] += static_cast<uint16_t>((x1 - static_cast<float>(i1)) * kSubRowWeight + 0.5f);
        }
        min = std::min(min, i0);
        max = std::max(max, i1);
    }
};

}  // namespace

OverlayCanvas::OverlayCanvas(void *pixels, int width, int height, int stride_bytes,
                             std::vector<uint16_t> &coverage, std::vector<uint16_t> &damage)
    : base_(static_cast<uint8_t *>(pixels)), width_(width), height_(height), stride_(stride_bytes),
      coverage_(coverage), damage_(damage)
{
    if (static_cast<int>(coverage_.size()) < width_) coverage_.assign(width_, 0);
    if (static_cast<int>(damage_.size()) < height_) damage_.assign(height_, 0);
    while ((width_ - 1) >> tile_shift_ >= 16) ++tile_shift_;  // 칸이 16개 안에 들게
}

uint32_t *OverlayCanvas::row(int y) const
{
    return reinterpret_cast<uint32_t *>(base_ + static_cast<size_t>(y) * stride_);
}

void OverlayCanvas::mark(int y, int x0, int x1)
{
    const uint32_t first = static_cast<uint32_t>(x0) >> tile_shift_;
    const uint32_t last = static_cast<uint32_t>(x1 - 1) >> tile_shift_;
    damage_[y] |= static_cast<uint16_t>((2u << last) - (1u << first));
}

/* 이어진 칸은 memset 한 번으로 지운다. */
void OverlayCanvas::clear(bool only_damaged)
{
    for (int y = 0; y < height_; ++y) {
        uint32_t tiles = only_damaged ? damage_[y] : 0xffffu;
        damage_[y] = 0;
        while (tiles) {
            const int first = __builtin_ctz(tiles);
            const int end = first + __builtin_ctz(~(tiles >> first));  // 이어진 칸의 끝(제외)
            const int x0 = first << tile_shift_, x1 = std::min(width_, end << tile_shift_);
            if (x0 < x1) std::memset(row(y) + x0, 0, static_cast<size_t>(x1 - x0) * 4);
            tiles &= ~((1u << end) - (1u << first));
        }
    }
}

void OverlayCanvas::fill_rect(int x, int y, int w, int h, uint32_t color)
{
    const int x0 = std::max(0, x), x1 = std::min(width_, x + w);
    const int y0 = std::max(0, y), y1 = std::min(height_, y + h);
    if (x0 >= x1) return;
    const uint32_t rgb = color & 0x00ffffffu, a = color >> 24;
    for (int yy = y0; yy < y1; ++yy) {
        uint32_t *pixels = row(yy);
        for (int xx = x0; xx < x1; ++xx) blend(pixels[xx], rgb, a);
        mark(yy, x0, x1);
    }
}

/* 모서리 넷을 시계 방향(오른쪽 위부터)으로 이은 다각형. 곧은 행을 따로 칠할 때는 위 모서리
 * 둘(왼쪽 위, 오른쪽 위)과 아래 모서리 둘(오른쪽 아래, 왼쪽 아래)을 따로 칠한다. 둘을 잇는
 * 변은 수평이라 커버리지에 끼지 않는다. */
void OverlayCanvas::fill_round_rect(float x, float y, float w, float h, float radius, uint32_t color)
{
    radius = std::clamp(radius, 0.0f, std::min(w, h) / 2);
    const HudPoint centers[] = {{x + w - radius, y + radius}, {x + w - radius, y + h - radius},
                                {x + radius, y + h - radius}, {x + radius, y + radius}};
    // 모서리 c는 (c-1)·90°에서 c·90°까지: 사분원 점 (cos, sin)을 90°씩 돌린 것.
    constexpr float kTurn[4][4] = {{0, 1, -1, 0}, {1, 0, 0, 1}, {0, -1, 1, 0}, {-1, 0, 0, -1}};
    const int step = radius < 3.0f ? 3 : radius < 6.0f ? 2 : 1;  // 선분 2, 3, 6개
    auto corners = [&](std::initializer_list<int> order) {
        HudPoint points[4 * (kCornerSegments + 1)];
        int n = 0;
        for (int corner : order) {
            const float *t = kTurn[corner];
            for (int i = 0; i <= kCornerSegments; i += step) {
                const HudPoint &arc = kQuarterArc[i];
                points[n++] = {centers[corner].x + radius * (t[0] * arc.x + t[1] * arc.y),
                               centers[corner].y + radius * (t[2] * arc.x + t[3] * arc.y)};
            }
        }
        fill_polygon(points, n, color);
    };
    const bool integral = x == std::floor(x) && y == std::floor(y) && w == std::floor(w) &&
                          h == std::floor(h) && radius == std::floor(radius);
    if (!integral || h <= 2 * radius) {
        corners({0, 1, 2, 3});
        return;
    }
    corners({3, 0});
    fill_rect(static_cast<int>(x), static_cast<int>(y + radius), static_cast<int>(w), static_cast<int>(h - 2 * radius),
              color);
    corners({1, 2});
}

void OverlayCanvas::fill_polygon(const HudPoint *points, int count, uint32_t color)
{
    fill_rows(points, count, color, 0.0f, 0.0f, 1.0f);
}

void OverlayCanvas::fill_polygon_faded(const HudPoint *points, int count, uint32_t color, float near_y,
                                       float far_y, float far_alpha)
{
    fill_rows(points, count, color, near_y, far_y, far_alpha);
}

/* 행마다 부표본 행 4개가 변과 만나는 x로 짝홀 구간을 구한다. 변은 위 끝 순으로 정렬해 지금
 * 걸친 변만 본다(띠는 66변 중 둘). 네 부표본 행이 모두 구간 하나면(이 HUD의 도형은 거의 다)
 * 넷 다 덮는 안쪽은 바로 합성하고 양쪽 가장자리 띠만 커버리지로 쌓는다. 아니면 구간 전부를
 * 쌓는다. near_y == far_y면 알파를 줄이지 않는다. */
void OverlayCanvas::fill_rows(const HudPoint *points, int count, uint32_t color, float near_y,
                              float far_y, float far_alpha)
{
    if (count < 3 || count > kMaxEdges) return;
    Edge edges[kMaxEdges];
    int edge_count = 0;
    float max_y = points[0].y;
    for (int i = 0, j = count - 1; i < count; j = i++) {
        const HudPoint &a = points[j], &b = points[i];
        max_y = std::max(max_y, b.y);
        if (a.y == b.y) continue;  // 수평 변은 부표본 행과 만나지 않는다
        const HudPoint &top = a.y < b.y ? a : b, &bottom = a.y < b.y ? b : a;
        edges[edge_count++] = {top.y, bottom.y, top.x, (bottom.x - top.x) / (bottom.y - top.y)};
    }
    if (edge_count < 2) return;
    std::sort(edges, edges + edge_count, [](const Edge &l, const Edge &r) { return l.top < r.top; });

    const int y0 = std::max(0, static_cast<int>(std::floor(edges[0].top)));
    const int y1 = std::min(height_ - 1, static_cast<int>(std::ceil(max_y)));
    const uint32_t rgb = color & 0x00ffffffu;
    const float alpha = static_cast<float>(color >> 24);
    const bool faded = near_y != far_y;

    int next = 0, active[kMaxCrossings], active_count = 0;
    for (int y = y0; y <= y1; ++y) {
        RowCoverage cover{coverage_.data(), width_, width_, -1};
        float left[kSubRows], right[kSubRows];
        bool single = true;
        for (int s = 0; s < kSubRows; ++s) {
            const float sy = static_cast<float>(y) + (static_cast<float>(s) + 0.5f) / kSubRows;
            while (next < edge_count && edges[next].top <= sy) {
                if (active_count < kMaxCrossings) active[active_count++] = next;
                ++next;
            }
            float xs[kMaxCrossings];
            int crossings = 0, kept = 0;
            for (int k = 0; k < active_count; ++k) {
                const Edge &e = edges[active[k]];
                if (e.bottom <= sy) continue;
                active[kept++] = active[k];
                xs[crossings++] = e.x + (sy - e.top) * e.slope;
            }
            active_count = kept;
            if (crossings == 2 && single) {
                left[s] = std::min(xs[0], xs[1]);
                right[s] = std::max(xs[0], xs[1]);
                continue;
            }
            if (single) {  // 앞서 미뤄 둔 구간도 쌓는다
                for (int k = 0; k < s; ++k) cover.add(left[k], right[k]);
                single = false;
            }
            std::sort(xs, xs + crossings);
            for (int k = 0; k + 1 < crossings; k += 2) cover.add(xs[k], xs[k + 1]);
        }

        // 넷 다 덮는 안쪽 [inner_l, inner_r)
        int inner_l = 0, inner_r = 0;
        if (single) {
            float max_left = left[0], min_right = right[0];
            for (int s = 1; s < kSubRows; ++s) {
                max_left = std::max(max_left, left[s]);
                min_right = std::min(min_right, right[s]);
            }
            inner_l = std::max(0, static_cast<int>(std::ceil(max_left)));
            inner_r = std::min(width_, static_cast<int>(std::floor(min_right)));
            for (int s = 0; s < kSubRows; ++s) {
                if (inner_l < inner_r) {
                    cover.add(left[s], static_cast<float>(inner_l));
                    cover.add(static_cast<float>(inner_r), right[s]);
                } else {
                    cover.add(left[s], right[s]);
                }
            }
        }
        if (inner_l >= inner_r && cover.max < cover.min) continue;

        float row_alpha = alpha;
        if (faded) {
            const float t = std::clamp((near_y - (static_cast<float>(y) + 0.5f)) / (near_y - far_y), 0.0f, 1.0f);
            row_alpha *= 1.0f - t * (1.0f - far_alpha);
        }
        const uint32_t a = static_cast<uint32_t>(row_alpha + 0.5f);
        uint32_t *pixels = row(y);
        auto blend_covered = [&](int from, int to) {
            for (int x = from; x <= to; ++x) {
                blend(pixels[x], rgb, div255(a * std::min<uint32_t>(cover.cells[x], 255)));
                cover.cells[x] = 0;
            }
        };
        if (inner_l < inner_r) {
            blend_covered(cover.min, std::min(cover.max, inner_l - 1));
            for (int x = inner_l; x < inner_r; ++x) blend(pixels[x], rgb, a);
            blend_covered(std::max(cover.min, inner_r), cover.max);
            mark(y, std::min(cover.min, inner_l), std::max(cover.max + 1, inner_r));
        } else {
            blend_covered(cover.min, cover.max);
            mark(y, cover.min, cover.max + 1);
        }
    }
}

int OverlayCanvas::text(int x, int y, std::string_view text, const HudFont &font, uint32_t color,
                        HudAlign align, bool shadow)
{
    const int width = font.width(text);
    if (align == HudAlign::center) x -= width / 2;
    else if (align == HudAlign::right) x -= width;
    if (shadow) glyphs(x + 1, y + 1, text, font, hud_fade(kTextShadow, static_cast<float>(color >> 24) / 255.0f));
    glyphs(x, y, text, font, color);
    return width;
}

void OverlayCanvas::glyphs(int x, int y, std::string_view text, const HudFont &font, uint32_t color)
{
    const uint32_t rgb = color & 0x00ffffffu, a = color >> 24;
    for (char c : text) {
        const HudGlyph *g = font.glyph(c);
        if (!g) continue;
        const int gx = x + g->left, gy = y + g->top;
        const int col0 = std::max(0, -gx), col1 = std::min<int>(g->w, width_ - gx);
        for (int line = std::max(0, -gy); col0 < col1 && line < g->h && gy + line < height_; ++line) {
            const uint8_t *coverage = font.pixels + g->offset + static_cast<size_t>(line) * g->w;
            uint32_t *pixels = row(gy + line);
            for (int col = col0; col < col1; ++col)
                if (coverage[col]) blend(pixels[gx + col], rgb, div255(a * coverage[col]));
            mark(gy + line, gx + col0, gx + col1);
        }
        x += g->advance;
    }
}
