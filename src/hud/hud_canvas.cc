#include "hud/hud_canvas.h"

#include "common/utils_math.h"  // kPiF
#include "hud/hud_tiles.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <initializer_list>

namespace {

constexpr int kSubRows = 4;
constexpr int kSubRowWeight = 256 / kSubRows;
constexpr int kMaxCrossings = 16;
constexpr int kMaxFadeColumns = 1024;  // 열마다 흐림 알파를 미리 세는 표(세로 패널 버퍼 폭 480이 들어간다)

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
        const float angle = static_cast<float>(i) / kCornerSegments * (kPiF / 2);
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

inline uint32_t div255(uint32_t x) { return (x + 128 + ((x + 128) >> 8)) >> 8; }

// dst 위에 rgb를 알파 a(0..255)로 over 합성한다(둘 다 스트레이트 알파). 투명한 바탕(대부분의
// 화소)과 불투명한 위 색은 바로 쓴다. 분자는 255·oa를 넘지 않아 역수를 곱해도 넘치지 않는다.
// 화소 루프마다 펼쳐지도록 늘 인라인한다(그냥 두면 GCC가 호출로 남겨 화소마다 호출한다).
[[gnu::always_inline]] inline void blend(uint32_t &dst, uint32_t rgb, uint32_t a) {
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

// 화소 [from, to)를 화소마다의 알파 alpha(x)로 합성한다. 바탕 넷이 모두 투명하면(HUD 화소 대부분) 넷을 바로
// 쓴다: blend가 바탕 알파 0일 때 쓰는 값과 같고, 알파 0인 화소는 그대로 둔다.
template <typename Alpha>
[[gnu::always_inline]] inline void blend_run(uint32_t *pixels, int from, int to, uint32_t rgb, Alpha alpha) {
    int x = from;
    for (; x + 4 <= to; x += 4) {
        if ((pixels[x] | pixels[x + 1] | pixels[x + 2] | pixels[x + 3]) >> 24) {
            for (int k = x; k < x + 4; ++k) blend(pixels[k], rgb, alpha(k));
            continue;
        }
        for (int k = x; k < x + 4; ++k) {
            const uint32_t a = alpha(k);
            if (a) pixels[k] = a << 24 | rgb;
        }
    }
    for (; x < to; ++x) blend(pixels[x], rgb, alpha(x));
}

// 한 행의 부표본 커버리지 합(화소마다 0..256)과 덮인 화소 범위 [min, max]. 칸은 합성하며
// 0으로 되돌린다.
struct RowCoverage {
    uint16_t *cells;
    int width;
    int min;
    int max;

    // 부표본 행 하나의 구간 [x0, x1)을 더한다. 양 끝 화소는 덮인 비율만큼.
    [[gnu::always_inline]] void add(float x0, float x1) {
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

HudCanvas::HudCanvas(void *pixels, int width, int height, int stride_bytes, std::vector<uint16_t> &coverage,
                     std::vector<uint32_t> &damage, bool transpose)
    : base_(static_cast<uint8_t *>(pixels)),
      width_(width),
      height_(height),
      buffer_w_(transpose ? height : width),
      buffer_h_(transpose ? width : height),
      stride_(stride_bytes),
      transpose_(transpose),
      coverage_(coverage),
      damage_(damage) {
    if (static_cast<int>(coverage_.size()) < buffer_w_) coverage_.assign(buffer_w_, 0);
    if (static_cast<int>(damage_.size()) < buffer_h_) damage_.assign(buffer_h_, 0);
    while ((buffer_w_ - 1) >> tile_shift_ >= 31) ++tile_shift_;  // 칸이 31개 안에 들게(hud_tiles.h)
}

uint32_t *HudCanvas::row(int y) const { return reinterpret_cast<uint32_t *>(base_ + static_cast<size_t>(y) * stride_); }

HudPoint HudCanvas::to_buffer(HudPoint p) const { return transpose_ ? HudPoint{p.y, p.x} : p; }

void HudCanvas::mark(int y, int x0, int x1) {
    const uint32_t first = static_cast<uint32_t>(x0) >> tile_shift_;
    const uint32_t last = static_cast<uint32_t>(x1 - 1) >> tile_shift_;
    damage_[y] |= (2u << last) - (1u << first);
}

// 이어진 칸은 memset 한 번으로 지운다.
void HudCanvas::clear(bool only_damaged) {
    const uint32_t all = (2u << ((buffer_w_ - 1) >> tile_shift_)) - 1;
    for (int y = 0; y < buffer_h_; ++y) {
        uint32_t *pixels = row(y);
        for_each_tile_run(only_damaged ? damage_[y] : all, [&](int first, int end) {
            const int x0 = first << tile_shift_, x1 = std::min(buffer_w_, end << tile_shift_);
            std::memset(pixels + x0, 0, static_cast<size_t>(x1 - x0) * 4);
        });
        damage_[y] = 0;
    }
}

void HudCanvas::fill_rect(int x, int y, int w, int h, uint32_t color) {
    const HudPoint p = to_buffer({static_cast<float>(x), static_cast<float>(y)});
    const HudPoint q = to_buffer({static_cast<float>(x + w), static_cast<float>(y + h)});
    const int x0 = std::max(0, static_cast<int>(std::min(p.x, q.x))),
              x1 = std::min(buffer_w_, static_cast<int>(std::max(p.x, q.x)));
    const int y0 = std::max(0, static_cast<int>(std::min(p.y, q.y))),
              y1 = std::min(buffer_h_, static_cast<int>(std::max(p.y, q.y)));
    if (x0 >= x1) return;
    const uint32_t rgb = color & 0x00ffffffu, a = color >> 24;
    for (int yy = y0; yy < y1; ++yy) {
        blend_run(row(yy), x0, x1, rgb, [a](int) { return a; });
        mark(yy, x0, x1);
    }
}

// 모서리 넷을 시계 방향(오른쪽 위부터)으로 이은 다각형. 곧은 부분을 따로 칠할 때는 버퍼 행을 따라 자른다:
// 가로 버퍼는 위 모서리 둘과 아래 모서리 둘을, 세로 패널 버퍼(버퍼 행 = 화면 x)는 왼쪽 모서리 둘과 오른쪽
// 모서리 둘을 따로 칠하고 그 사이를 fill_rect로 채운다. 둘을 잇는 변은 버퍼에서 수평이라 커버리지에 끼지
// 않는다.
void HudCanvas::fill_round_rect(float x, float y, float w, float h, float radius, uint32_t color) {
    radius = std::clamp(radius, 0.0f, std::min(w, h) / 2);
    const HudPoint centers[] = {{x + w - radius, y + radius},
                                {x + w - radius, y + h - radius},
                                {x + radius, y + h - radius},
                                {x + radius, y + radius}};
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
    const bool integral = x == std::floor(x) && y == std::floor(y) && w == std::floor(w) && h == std::floor(h) &&
                          radius == std::floor(radius);
    if (!integral || (transpose_ ? w : h) <= 2 * radius) {
        corners({0, 1, 2, 3});
        return;
    }
    const int ix = static_cast<int>(x), iy = static_cast<int>(y), iw = static_cast<int>(w), ih = static_cast<int>(h);
    const int r = static_cast<int>(radius);
    if (transpose_) {
        corners({2, 3});
        fill_rect(ix + r, iy, iw - 2 * r, ih, color);
        corners({0, 1});
    } else {
        corners({3, 0});
        fill_rect(ix, iy + r, iw, ih - 2 * r, color);
        corners({1, 2});
    }
}

// 버퍼 행마다 부표본 행 4개가 변과 만나는 x로 짝홀 구간을 구한다. 변은 위 끝 순으로 정렬해 지금
// 걸친 변만 본다(띠는 66변 중 둘). 네 부표본 행이 모두 구간 하나면(이 HUD의 도형은 거의 다)
// 넷 다 덮는 안쪽은 바로 합성하고 양쪽 가장자리 띠만 커버리지로 쌓는다. 아니면 구간 전부를
// 쌓는다. 흐림은 화면 높이(near_y → far_y)를 따른다: 가로 버퍼에서는 행마다, 세로 패널 버퍼
// 에서는 화면 높이가 버퍼 열이라 열마다 알파를 미리 센다. near_y == far_y면 줄이지 않는다.
void HudCanvas::fill_polygon(const HudPoint *screen_points, int count, uint32_t color, float near_y, float far_y,
                             float far_alpha) {
    HudPoint points[kHudMaxPolygonPoints];
    for (int i = 0; i < count; ++i) points[i] = to_buffer(screen_points[i]);
    Edge edges[kHudMaxPolygonPoints];
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
    const int y1 = std::min(buffer_h_ - 1, static_cast<int>(std::ceil(max_y)));
    const uint32_t rgb = color & 0x00ffffffu;
    const float alpha = static_cast<float>(color >> 24);
    const bool faded = near_y != far_y;
    auto fade = [&](float screen_y) {  // 화면 높이 screen_y에서의 알파
        const float t = std::clamp((near_y - screen_y) / (near_y - far_y), 0.0f, 1.0f);
        return static_cast<uint32_t>(alpha * (1.0f - t * (1.0f - far_alpha)) + 0.5f);
    };
    const bool fade_columns = faded && transpose_;
    std::array<uint8_t, kMaxFadeColumns> column_alpha;
    if (fade_columns) {
        for (int bx = 0; bx < buffer_w_; ++bx)
            column_alpha[bx] = static_cast<uint8_t>(fade(static_cast<float>(bx) + 0.5f));
    }

    int next = 0, active[kMaxCrossings], active_count = 0;
    for (int y = y0; y <= y1; ++y) {
        RowCoverage cover{coverage_.data(), buffer_w_, buffer_w_, -1};
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
            inner_r = std::min(buffer_w_, static_cast<int>(std::floor(min_right)));
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

        const uint32_t row_a =
            !faded || fade_columns ? static_cast<uint32_t>(alpha + 0.5f) : fade(static_cast<float>(y) + 0.5f);
        auto a = [&](int x) { return fade_columns ? column_alpha[x] : row_a; };
        uint32_t *pixels = row(y);
        auto blend_covered = [&](int from, int to) {
            for (int x = from; x <= to; ++x) {
                blend(pixels[x], rgb, div255(a(x) * std::min<uint32_t>(cover.cells[x], 255)));
                cover.cells[x] = 0;
            }
        };
        if (inner_l < inner_r) {
            blend_covered(cover.min, std::min(cover.max, inner_l - 1));
            blend_run(pixels, inner_l, inner_r, rgb, a);
            blend_covered(std::max(cover.min, inner_r), cover.max);
            mark(y, std::min(cover.min, inner_l), std::max(cover.max + 1, inner_r));
        } else {
            blend_covered(cover.min, cover.max);
            mark(y, cover.min, cover.max + 1);
        }
    }
}

int HudCanvas::text(int x, int y, std::string_view text, const HudFont &font, uint32_t color, HudAlign align,
                    bool shadow) {
    const int width = font.width(text);
    if (align == HudAlign::center)
        x -= width / 2;
    else if (align == HudAlign::right)
        x -= width;
    if (shadow) glyphs(x + 1, y + 1, text, font, hud_fade(kTextShadow, static_cast<float>(color >> 24) / 255.0f));
    glyphs(x, y, text, font, color);
    return width;
}

// 글리프 화소를 버퍼로 옮겨 합성한다. 가로 버퍼에서는 글리프 줄이, 세로 패널 버퍼에서는 글리프
// 열이 버퍼 행 하나가 되도록 돌아 버퍼 쪽 쓰기가 이어진다.
void HudCanvas::glyphs(int x, int y, std::string_view text, const HudFont &font, uint32_t color) {
    const uint32_t rgb = color & 0x00ffffffu, a = color >> 24;
    const bool transpose = transpose_;
    for (char c : text) {
        const HudGlyph *g = font.glyph(c);
        if (!g) continue;
        const int gx = x + g->left, gy = y + g->top;
        const int col0 = std::max(0, -gx), col1 = std::min<int>(g->w, width_ - gx);  // 화면 안의 열·줄
        const int line0 = std::max(0, -gy), line1 = std::min<int>(g->h, height_ - gy);
        const uint8_t *bitmap = font.pixels + g->offset;
        // 바깥 고리 하나가 버퍼 행 하나: 가로면 줄, 세로 패널이면 열
        const int outer0 = transpose ? col0 : line0, outer1 = transpose ? col1 : line1;
        const int inner0 = transpose ? line0 : col0, inner1 = transpose ? line1 : col1;
        for (int outer = outer0; inner0 < inner1 && outer < outer1; ++outer) {
            const int by = transpose ? gx + outer : gy + outer;
            const int origin = transpose ? gy : gx;  // 이 버퍼 행에서 글리프가 시작하는 버퍼 열
            uint32_t *pixels = row(by);
            for (int inner = inner0; inner < inner1; ++inner) {
                const int line = transpose ? inner : outer, col = transpose ? outer : inner;
                const uint8_t coverage = bitmap[static_cast<size_t>(line) * g->w + col];
                if (coverage) blend(pixels[origin + inner], rgb, div255(a * coverage));
            }
            mark(by, origin + inner0, origin + inner1);
        }
        x += g->advance;
    }
}
