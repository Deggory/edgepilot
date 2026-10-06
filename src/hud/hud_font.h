#ifndef HUD_FONT_H
#define HUD_FONT_H

/* HUD 글꼴. tools/ui/make_hud_font.py가 구운 글리프(hud_font_data.inc, Aileron Regular CC0)를
 * 컴파일 시점에 8비트 커버리지로 펼쳐 둔다. 실행 중 글꼴 래스터화는 없다. */

#include <cstdint>
#include <string_view>

struct HudGlyph {
    uint32_t offset;   // pixels에서 이 글리프 첫 화소의 위치(행 우선, 폭 w)
    uint8_t w;
    uint8_t h;
    int8_t left;       // 펜 위치에서 글리프 왼쪽까지
    int8_t top;        // 줄 위에서 글리프 위까지
    uint8_t advance;
};

struct HudFont {
    const HudGlyph *glyphs;
    const uint8_t *pixels;
    char first;
    int count;
    int ascent;   // 줄 위에서 기준선까지
    int descent;

    // 없는 글자는 nullptr
    const HudGlyph *glyph(char c) const
    {
        const int index = static_cast<unsigned char>(c) - static_cast<unsigned char>(first);
        return index >= 0 && index < count ? &glyphs[index] : nullptr;
    }
    int width(std::string_view text) const;
};

extern const HudFont kHudSpeedFont;    // 76 px 굵은 숫자(현재 속도)
extern const HudFont kHudValueFont;    // 30 px 숫자와 기어 글자(설정 속도, 기어)
extern const HudFont kHudTitleFont;    // 22 px 세미볼드(알림 제목)
extern const HudFont kHudBodyFont;     // 18 px(칩, 값)
extern const HudFont kHudCaptionFont;  // 14 px(보조 글, 진단)

#endif
