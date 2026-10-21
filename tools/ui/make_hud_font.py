#!/usr/bin/env python3
"""HUD 글꼴을 src/hud/hud_font_data.inc로 굽는다.

  make_hud_font.py [out.inc]

글리프는 Pillow의 기본 가변 크기 글꼴인 Aileron Regular(Sora Sagano, dotcolon.net, CC0 공개)에서 가져오므로
Pillow 밖의 것이 필요 없고 결과를 커밋해도 된다. 굵은 글씨는 획을 더해 그린다. 덮임 정도는 4비트 니블로
바이트당 화소 두 개씩 글리프를 차례로 저장하고, C++ 쪽(hud_font.cc)이 컴파일할 때 8비트로 늘린다.
"""

import argparse
from pathlib import Path
from typing import NamedTuple

from PIL import Image, ImageDraw, ImageFont

PRINTABLE = "".join(map(chr, range(32, 127)))
DIGITS = " -0123456789"
GEARS = "PRNDS"  # 기어 카드
DEGREE = "\u00b0"  # Latin-1 0xB0. C++ 문자열에서는 "\xb0"으로 쓴다


class Spec(NamedTuple):
    name: str     # C++ 식별자 줄기: k<Name>Glyphs / k<Name>Packed
    size: int     # 화소 크기
    stroke: int   # 더하는 굵기(화소, 0이면 보통)
    chars: str    # 표는 min(chars)..max(chars)를 덮고, 그 범위의 다른 글자는 비어 있다


SPECS = [
    Spec("Speed", 76, 2, DIGITS),
    Spec("Value", 30, 1, DIGITS + GEARS),
    Spec("Title", 22, 1, PRINTABLE),
    Spec("Body", 18, 0, PRINTABLE + DEGREE),
    Spec("Caption", 14, 0, PRINTABLE + DEGREE),
]


def render(spec: Spec):
    font = ImageFont.load_default(size=spec.size)
    ascent, _ = font.getmetrics()
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
        # 획은 바깥으로 퍼지므로 그만큼 글리프를 안으로 옮겨 펜 위치와 줄 위쪽을 고정한다.
        glyphs.append((len(nibbles), x1 - x0, y1 - y0, x0 + spec.stroke, y0 + spec.stroke, advance))
        nibbles.extend((value + 8) // 17 for value in image.tobytes())
    if len(nibbles) % 2:
        nibbles.append(0)
    packed = bytes(nibbles[i] << 4 | nibbles[i + 1] for i in range(0, len(nibbles), 2))
    return first, last, ascent + spec.stroke, glyphs, packed


def hex_rows(data: bytes, per_row: int = 24) -> str:
    return "\n".join("    " + ", ".join(f"0x{b:02x}" for b in data[i:i + per_row]) + ","
                     for i in range(0, len(data), per_row))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", type=Path, nargs="?",
                        default=Path(__file__).resolve().parents[2] / "src" / "hud" / "hud_font_data.inc")
    out = parser.parse_args().out
    parts = ["// tools/ui/make_hud_font.py가 Pillow 기본 글꼴(Aileron Regular, CC0)로 만든 파일이다.\n"
             "// 고치지 말고 스크립트를 다시 돌린다. hud_font.cc만 include한다.\n"]
    for spec in SPECS:
        first, last, ascent, glyphs, packed = render(spec)
        rows = ",\n".join(f"    {{{o}, {w}, {h}, {l}, {t}, {a}}}" for o, w, h, l, t, a in glyphs)
        parts.append(
            f"// {spec.name}: {spec.size} px, 획 {spec.stroke}, '{first}'..'{last}', {len(packed)}바이트\n"
            f"constexpr HudGlyph k{spec.name}Glyphs[] = {{\n{rows},\n}};\n"
            f"constexpr uint8_t k{spec.name}Packed[] = {{\n{hex_rows(packed)}\n}};\n"
            f"constexpr char k{spec.name}First = {ord(first)};\n"
            f"constexpr int k{spec.name}Ascent = {ascent};\n")
    out.write_text("\n".join(parts))
    print(f"wrote {out} ({out.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
