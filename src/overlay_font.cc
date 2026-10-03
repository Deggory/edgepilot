#include "overlay_font.h"

#include <array>
#include <cstddef>
#include <iterator>

namespace {

#include "overlay_font_data.inc"

// 4비트씩 두 화소를 묶은 커버리지를 8비트로 편다(컴파일 시점).
template <size_t N>
constexpr std::array<uint8_t, N * 2> unpack(const uint8_t (&packed)[N])
{
    std::array<uint8_t, N * 2> pixels{};
    for (size_t i = 0; i < N; ++i) {
        pixels[2 * i] = static_cast<uint8_t>((packed[i] >> 4) * 17);
        pixels[2 * i + 1] = static_cast<uint8_t>((packed[i] & 0x0f) * 17);
    }
    return pixels;
}

constexpr auto kSpeedPixels = unpack(kSpeedPacked);
constexpr auto kValuePixels = unpack(kValuePacked);
constexpr auto kTitlePixels = unpack(kTitlePacked);
constexpr auto kBodyPixels = unpack(kBodyPacked);
constexpr auto kCaptionPixels = unpack(kCaptionPacked);

}  // namespace

int HudFont::width(std::string_view text) const
{
    int total = 0;
    for (char c : text)
        if (const HudGlyph *g = glyph(c)) total += g->advance;
    return total;
}

const HudFont kHudSpeedFont{kSpeedGlyphs, kSpeedPixels.data(), kSpeedFirst,
                            static_cast<int>(std::size(kSpeedGlyphs)), kSpeedAscent, kSpeedDescent};
const HudFont kHudValueFont{kValueGlyphs, kValuePixels.data(), kValueFirst,
                            static_cast<int>(std::size(kValueGlyphs)), kValueAscent, kValueDescent};
const HudFont kHudTitleFont{kTitleGlyphs, kTitlePixels.data(), kTitleFirst,
                            static_cast<int>(std::size(kTitleGlyphs)), kTitleAscent, kTitleDescent};
const HudFont kHudBodyFont{kBodyGlyphs, kBodyPixels.data(), kBodyFirst,
                           static_cast<int>(std::size(kBodyGlyphs)), kBodyAscent, kBodyDescent};
const HudFont kHudCaptionFont{kCaptionGlyphs, kCaptionPixels.data(), kCaptionFirst,
                              static_cast<int>(std::size(kCaptionGlyphs)), kCaptionAscent,
                              kCaptionDescent};
