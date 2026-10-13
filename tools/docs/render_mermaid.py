#!/usr/bin/env python3
"""문서 속 Mermaid 그림을 mermaid-cli(도커 이미지 minlag/mermaid-cli)로 그려 본다. 그리지 못한 그림과, GitHub
본문 칸(약 830 px)보다 넓어 글자가 작아지는 그림을 알려 준다. 그린 PNG는 --out 아래에 남는다(눈으로 볼 때).

사용: tools/docs/render_mermaid.py [--out build-docs] [문서.md ...]
  문서를 주지 않으면 최상위 README, docs/*.md와 폴더 README를 모두 본다.
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
IMAGE = "minlag/mermaid-cli:latest"
COLUMN_PX = 830
BLOCK = re.compile(r"^```mermaid\n(.*?)^```", re.M | re.S)


def documents():
    files = [ROOT / "README.md"] + sorted((ROOT / "docs").glob("*.md"))
    listed = subprocess.run(["git", "ls-files", "*/README.md"], cwd=ROOT, capture_output=True, text=True).stdout
    # firmware/panda는 들여온 코드지만 README는 우리 문서다
    files += [ROOT / f for f in listed.split() if not f.startswith("firmware/panda/") or f == "firmware/panda/README.md"]
    return files


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", default="build-docs", help="그림을 남길 곳(저장소 기준)")
    parser.add_argument("files", nargs="*")
    args = parser.parse_args()
    out = (ROOT / args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("*.mmd"):
        old.unlink()
    blocks = []
    for path in [Path(f).resolve() for f in args.files] or documents():
        text = path.read_text(encoding="utf-8")
        for n, match in enumerate(BLOCK.finditer(text), 1):
            line = text.count("\n", 0, match.start()) + 1
            stem = f"{path.relative_to(ROOT).as_posix().replace('/', '_').removesuffix('.md')}-{n}"
            (out / f"{stem}.mmd").write_text(match.group(1), encoding="utf-8")
            blocks.append((stem, f"{path.relative_to(ROOT)}:{line}"))
    if not blocks:
        print("no mermaid blocks")
        return 0
    # 컨테이너 하나에서 차례로 그린다. svg로 폭을 재고, png는 2배로 그려 눈으로 본다.
    script = ("cd /data && for f in *.mmd; do s=${f%.mmd}; rm -f $s.svg $s.png; "
              "mmdc -q -p /puppeteer-config.json -i $f -o $s.svg -b white >$s.log 2>&1 && mmdc -q -p /puppeteer-config.json -i $f -o $s.png -b white -s 2 >>$s.log 2>&1; "
              "done")
    subprocess.run(["docker", "run", "--rm", "--entrypoint", "sh", "-v", f"{out}:/data", IMAGE, "-c", script],
                   check=True)
    failed = wide = 0
    for stem, where in blocks:
        svg = out / f"{stem}.svg"
        if not svg.is_file():
            failed += 1
            log = (out / f"{stem}.log").read_text(encoding="utf-8", errors="replace").strip().splitlines()
            print(f"FAIL  {where}: {log[-1] if log else 'no output'}")
            continue
        m = re.search(r"max-width:\s*([\d.]+)px", svg.read_text(encoding="utf-8")[:4000])
        width = float(m.group(1)) if m else 0.0
        if width > COLUMN_PX:
            wide += 1
            print(f"WIDE  {where}: {width:.0f} px, text shrinks to {100 * COLUMN_PX / width:.0f}%  ({stem}.png)")
        else:
            print(f"ok    {where}: {width:.0f} px  ({stem}.png)")
    print(f"{len(blocks)} diagrams, {failed} failed, {wide} wider than {COLUMN_PX} px; pictures in {out}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
