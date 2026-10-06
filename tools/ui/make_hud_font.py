#!/usr/bin/env python3
"""Bake the HUD fonts into src/hud/overlay_font_data.inc.

  make_hud_font.py [out.inc]

The glyphs come from Pillow's default scalable font, Aileron Regular (Sora
Sagano, dotcolon.net, released as CC0), so nothing outside Pillow is needed and
the result may be committed. Heavier weights are drawn with a stroke. Coverage
is stored as 4-bit nibbles, two pixels per byte, glyph after glyph; the C++
side (overlay_font.cc) expands it to 8 bits at compile time.
"""

import sys
from pathlib import Path
from typing import NamedTuple

from PIL import Image, ImageDraw, ImageFont

PRINTABLE = "".join(map(chr, range(32, 127)))
DIGITS = " -0123456789"
GEARS = "PRNDS"  # 기어 카드
DEGREE = "\u00b0"  # Latin-1 0xB0, written as "\xb0" in C++ strings


class Spec(NamedTuple):
    name: str     # C++ identifier stem: k<Name>Glyphs / k<Name>Packed
    size: int     # pixel size
    stroke: int   # extra weight in pixels (0 = regular)
    chars: str    # the table spans min(chars)..max(chars); others in that range are empty


SPECS = [
    Spec("Speed", 76, 2, DIGITS),
    Spec("Value", 30, 1, DIGITS + GEARS),
    Spec("Title", 22, 1, PRINTABLE),
    Spec("Body", 18, 0, PRINTABLE + DEGREE),
    Spec("Caption", 14, 0, PRINTABLE + DEGREE),
]


def render(spec: Spec):
    font = ImageFont.load_default(size=spec.size)
    ascent, descent = font.getmetrics()
    first, last = min(spec.chars), max(spec.chars)
    glyphs, nibbles = [], []
    for code in range(ord(first), ord(last) + 1):
        ch = chr(code)
        advance = round(font.getlength(ch)) + 2 * spec.stroke
        box = font.getbbox(ch, stroke_width=spec.stroke) if ch in spec.chars else None
        if not box or box[2] <= box[0] or box[3] <= box[1]:
            glyphs.append((len(nibbles), 0, 0, 0, 0, advance if ch in spec.chars else 0))
            continue
        x0, y0, x1, y1 = box
        image = Image.new("L", (x1 - x0, y1 - y0), 0)
        ImageDraw.Draw(image).text((-x0, -y0), ch, font=font, fill=255,
                                   stroke_width=spec.stroke, stroke_fill=255)
        # The stroke spreads outward, so shift the glyph in by it to keep pen and line top fixed.
        glyphs.append((len(nibbles), x1 - x0, y1 - y0, x0 + spec.stroke, y0 + spec.stroke, advance))
        nibbles.extend((value + 8) // 17 for value in image.tobytes())
    if len(nibbles) % 2:
        nibbles.append(0)
    packed = bytes(nibbles[i] << 4 | nibbles[i + 1] for i in range(0, len(nibbles), 2))
    return first, last, ascent + spec.stroke, descent + spec.stroke, glyphs, packed


def hex_rows(data: bytes, per_row: int = 24) -> str:
    return "\n".join("    " + ", ".join(f"0x{b:02x}" for b in data[i:i + per_row]) + ","
                     for i in range(0, len(data), per_row))


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        Path(__file__).resolve().parents[2] / "src" / "hud" / "overlay_font_data.inc"
    parts = ["/* tools/ui/make_hud_font.py가 Pillow 기본 글꼴(Aileron Regular, CC0)로 만든 파일이다.\n"
             " * 고치지 말고 스크립트를 다시 돌린다. overlay_font.cc만 include한다. */\n"]
    for spec in SPECS:
        first, last, ascent, descent, glyphs, packed = render(spec)
        rows = ",\n".join(f"    {{{o}, {w}, {h}, {l}, {t}, {a}}}" for o, w, h, l, t, a in glyphs)
        parts.append(
            f"// {spec.name}: {spec.size} px, 획 {spec.stroke}, '{first}'..'{last}', {len(packed)}바이트\n"
            f"constexpr HudGlyph k{spec.name}Glyphs[] = {{\n{rows},\n}};\n"
            f"constexpr uint8_t k{spec.name}Packed[] = {{\n{hex_rows(packed)}\n}};\n"
            f"constexpr char k{spec.name}First = {ord(first)};\n"
            f"constexpr int k{spec.name}Ascent = {ascent};\n"
            f"constexpr int k{spec.name}Descent = {descent};\n")
    out.write_text("\n".join(parts))
    print(f"wrote {out} ({out.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
