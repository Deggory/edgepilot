#!/usr/bin/env python3
"""문서 형식 검사(docs/style-guide.md의 Documents·Diagrams 규칙). docs/*.md와 폴더 README를 본다:
제목이 파일 이름과 맞는지, 제목 아래 색인 링크, 마지막 절 Related, 상대 링크와 #앵커가 있는지, 산문 80칸 줄바꿈,
한글이 없는지(용어집이 있는 style-guide.md는 뺀다), Mermaid 색 정의가 공통 팔레트와 같은지, 그림 파일이 있는지.
최상위 README는 링크, 그림, Mermaid 색만 본다.

사용: tests/check_docs.py [문서.md ...]   (주지 않으면 모두)
"""
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WRAP = 80
PALETTE = {
    "pixel": "fill:#e7f5f4,stroke:#1f9e9a,color:#1f2328",
    "compute": "fill:#ebf3fe,stroke:#2f81f7,color:#1f2328",
    "proc": "fill:#f6f8fa,stroke:#8b949e,color:#1f2328",
    "external": "fill:#fff3e8,stroke:#f0883e,color:#1f2328",
    "safety": "fill:#eaf7ee,stroke:#2da44e,color:#1f2328",
    "group": "fill:#ffffff,stroke:#d1d9e0,color:#59636e",
}
HANGUL = re.compile(r"[ᄀ-ᇿ㄰-㆏가-힣]")
LINK = re.compile(r"(?<!!)\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
IMG = re.compile(r'<img[^>]*\ssrc="([^"]+)"|!\[[^\]]*\]\(([^)\s]+)\)')


def documents():
    files = sorted((ROOT / "docs").glob("*.md"))
    listed = subprocess.run(["git", "ls-files", "*/README.md"], cwd=ROOT, capture_output=True, text=True).stdout
    # firmware/panda는 들여온 코드지만 README는 우리 문서다
    files += [ROOT / f for f in listed.split() if not f.startswith("firmware/panda/") or f == "firmware/panda/README.md"]
    return files


def slug(heading: str) -> str:
    """GitHub의 제목 앵커: 소문자, 글자·숫자·공백·하이픈·밑줄만 남기고 공백을 하이픈으로."""
    text = re.sub(r"`|\*\*|\*", "", heading.strip().lower())
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"[^\w\- ]", "", text)
    return text.replace(" ", "-")


def anchors(path: Path):
    seen, out = {}, set()
    in_code = False
    for line in path.read_text(encoding="utf-8").split("\n"):
        if line.startswith("```"):
            in_code = not in_code
        if in_code:
            continue
        m = re.match(r"^#{1,6} (.+)", line)
        if m:
            s = slug(m.group(1))
            n = seen.get(s, 0)
            out.add(s if n == 0 else f"{s}-{n}")
            seen[s] = n + 1
    return out


def prose_lines(text: str):
    """(줄 번호, 줄): 코드 블록, 표, HTML, 제목 밖의 산문 줄."""
    in_code = False
    for i, line in enumerate(text.split("\n"), 1):
        if line.lstrip().startswith("```"):
            in_code = not in_code
            continue
        if in_code or not line.strip():
            continue
        s = line.lstrip()
        if s.startswith(("|", "<", "#")) or re.match(r"^\[[^\]]+\]:\s", s):
            continue
        yield i, line


def check_file(path: Path, root_readme: bool = False):
    problems = []
    rel = path.relative_to(ROOT)
    text = path.read_text(encoding="utf-8")
    lines = text.split("\n")

    def problem(line, message):
        problems.append(f"{rel}:{line}: {message}")

    # 링크, 앵커, 그림
    in_code = False
    for i, line in enumerate(lines, 1):
        if line.lstrip().startswith("```"):
            in_code = not in_code
        if in_code:
            continue
        line = re.sub(r"`[^`]*`", "", line)  # 인라인 코드 안의 예시는 링크가 아니다
        for target in LINK.findall(line):
            if re.match(r"^[a-z]+:", target) or target.startswith("//"):
                continue
            file, _, anchor = target.partition("#")
            dest = (path.parent / file).resolve() if file else path
            if file and not dest.exists():
                if file.startswith("../../"):  # GitHub 저장소 경로(예: ../../tree/k230)
                    continue
                problem(i, f"link to a missing file: {target}")
                continue
            if anchor and dest.suffix == ".md" and anchor not in anchors(dest):
                problem(i, f"link to a missing anchor: {target}")
        for a, b in IMG.findall(line):
            src = a or b
            if not re.match(r"^[a-z]+:", src) and not (path.parent / src).exists():
                problem(i, f"missing image: {src}")
    if root_readme:
        return problems + check_mermaid(rel, lines)

    # 제목과 색인 링크
    title = lines[0][2:].strip() if lines and lines[0].startswith("# ") else ""
    if not title:
        problem(1, "the first line must be the title (# ...)")
    if path.parent == ROOT / "docs":
        if path.name == "README.md":
            expected_back = "[← edgepilot](../README.md)"
        else:
            expected_back = "[← Documentation](README.md)"
            name = path.stem.replace("-", "")
            if re.sub(r"[\s\-]", "", title.lower()) != name:
                problem(1, f"the title must be the file name in sentence case (got {title!r})")
            if title and title[0].islower() and not title.startswith("`"):
                problem(1, "the title starts with a capital letter")
    else:
        back = Path("../" * (len(rel.parts) - 1)) / "docs" / "README.md"
        expected_back = f"[← Documentation]({back.as_posix()})"
    if len(lines) < 3 or lines[2].strip() != expected_back:
        problem(3, f"the line under the title must be {expected_back}")

    # 절 제목: 문장형, 마지막은 Related(색인은 빼고)
    headings = [(i, line) for i, line in enumerate(lines, 1) if re.match(r"^#{2,4} ", line)]
    in_code = False
    real = []
    for i, line in enumerate(lines, 1):
        if line.lstrip().startswith("```"):
            in_code = not in_code
        elif not in_code and re.match(r"^#{2,4} ", line):
            real.append((i, line))
    headings = real
    for i, line in headings:
        words = re.sub(r"`[^`]*`", "", line.split(" ", 1)[1]).split()
        caps = [w for w in words[1:] if re.match(r"^[A-Z][a-z]{3,}$", w)]
        if len(caps) >= 2 and len(caps) >= len(words) // 2:
            problem(i, f"headings are in sentence case: {line.strip()}")
    h2 = [line for _, line in headings if line.startswith("## ")]
    if path.name != "README.md" or path.parent != ROOT / "docs":
        if not h2 or h2[-1].strip() != "## Related":
            problem(len(lines), "the last section must be ## Related")

    # 산문 줄바꿈, 언어
    for i, line in prose_lines(text):
        if len(line) > WRAP:
            cut = line.rfind(" ", 0, WRAP + 1)
            if cut > len(line) - len(line.lstrip()) and len(line) - cut - 1 <= WRAP:
                problem(i, f"prose is wrapped at {WRAP} columns ({len(line)})")
    # 화면에 보이는 한국어 문구는 인라인 코드로만 인용한다(웹 콘솔 탭 이름 등)
    if path.name != "style-guide.md":
        for i, line in enumerate(lines, 1):
            if HANGUL.search(re.sub(r"`[^`]*`", "", line)):
                problem(i, "documents are English (quote on-screen Korean text as `code`)")
                break

    return problems + check_mermaid(rel, lines)


def check_mermaid(rel, lines):
    """Mermaid 색: classDef는 공통 팔레트의 것만 쓴다."""
    problems = []
    in_mermaid = False
    for i, line in enumerate(lines, 1):
        if line.strip() == "```mermaid":
            in_mermaid = True
        elif line.strip().startswith("```"):
            in_mermaid = False
        elif in_mermaid:
            m = re.match(r"^\s*classDef\s+(\w+)\s+(\S+)", line)
            if m and PALETTE.get(m.group(1)) != m.group(2):
                problems.append(f"{rel}:{i}: classDef {m.group(1)} is not one of the shared classes "
                                "(docs/style-guide.md#diagrams)")
    return problems


def main():
    files = [Path(f).resolve() for f in sys.argv[1:]] or documents() + [ROOT / "README.md"]
    problems = []
    for path in files:
        problems += check_file(path, root_readme=path == ROOT / "README.md")
    for p in problems:
        print(p)
    print(f"{len(files)} documents, {len(problems)} problems")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
