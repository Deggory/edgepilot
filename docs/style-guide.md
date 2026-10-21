# Style guide

[← Documentation](README.md)

How code, comments and documents are written in this repository. Read it
before a change that adds a file, a document or a diagram.

## Languages

- Code comments and docstrings are Korean, in the plain written style (`~다`),
  in every language of the repository: C++, Python, JavaScript, shell, CMake.
- Identifiers, log and error messages, and commit messages are English.
  Commit subjects read `<area>: <what changed>`.
- Documents are English: the root README, `docs/` and the READMEs in folders.
  They name the web console's tabs and buttons by its English labels. Text
  that appears on a screen only in Korean is quoted as inline code next to
  its English name.
- Text on the web console's screen is in its text catalogues, in Korean and
  English ([JavaScript](#javascript)); the code holds keys, not text.
- Some parts of a comment stay as they are: identifiers and field lists
  (`dlc=8, bus=2`), formulas and ranges, the command after `사용:`, file paths,
  linter directives (`# noqa`) and shebangs. Comments inside GLSL shader
  strings stay ASCII, because older WebGL drivers reject shader source with
  other characters.

Use the same Korean word for the same thing:

| Term | Meaning |
| --- | --- |
| 자차 | the ego car |
| 앞차, 앞차 추적 | the lead car, the lead track |
| 차선선 | a lane line (차선 alone is the lane) |
| 광류 | optical flow |
| 확대율 | expansion rate (of the lead in the image) |
| 출발 알림 | departure alert |
| 결합, 해제 | engage, disengage |
| 보정, 보정기 | calibration, the calibrator |
| 워프 | the model input warp |
| 학습기 | a learner (paramsd, torqued, lagd) |
| 녹화, 재생, 리허설 | recording, replay, rehearsal |

## C++

`.clang-format` holds the format: Google style with these changes.

- 4-space indent, 4 more for a continued line.
- 120 columns. A Hangul syllable counts as two.
- `int *p`: the `*` and `&` go with the name.
- `public:` and `private:` line up with `class`.
- Includes are sorted within each group of lines that has no blank line in it.

Format every first-party file, or check that it is already formatted, with the
Xcode command line tools' clang-format (Apple clang-format 21). Another
clang-format version may change a few lines.

```sh
clang-format -i $(git ls-files '*.cc' '*.h' | grep -v '^firmware/')
clang-format --dry-run --Werror $(git ls-files '*.cc' '*.h' | grep -v '^firmware/')
```

`firmware/panda/` is vendored from commaai/panda and keeps its own format
(`firmware/.clang-format` turns formatting off there).

- Comments are `//` only, as Google style uses them, including the comment at
  the top of a file that says what it is for. Do not write `/* */`.
- A header starts with `#pragma once`, then that comment, then the includes.
  A `.cc` file starts with the comment.
- An `enum class` names its members in PascalCase (`LaneChangeState::Starting`).
  The text a log or the screen shows comes from a `<enum>_name()` function
  next to the enum.
- A function called once per tick or per frame is `update()`. Functions that
  take one input as it arrives are `on_<input>()`.
- `// clang-format off` and `// clang-format on` are only for layouts that
  carry meaning: the row-major 3x3 matrices, the audio stub macro table in
  `maix_shim.cc`, and golden tables that a Python check reads back.
- Sometimes an include has to come before another. Put the later one in its
  own group after a blank line, with a comment saying why, so sorting cannot
  swap them. For example, `system_monitor.cc` includes `<net/if.h>` before
  `<linux/wireless.h>`.

## Python

PEP 8 with 4-space indent and lines up to 120 columns. A module starts with a
docstring that says what it is for.

## JavaScript

The web console's page is plain ES modules with no build step. It uses 2-space
indent and `//` comments. No formatter is applied, so follow the code around
the change. `static/three/` is vendored.

Every text on screen goes through `t(key, values)` from `static/i18n.js`, and
lives in `static/i18n/ko.json` and `en.json` under the same key:

- Keys are dotted, by where they show: `app.*`, `params.*`, `vehicle.*`,
  `calibration.*`, `panda.*`, `bev.*`. The parameters' texts are
  `param.<group>.<key>.label`, `.description`, `.increase`, `.decrease` and
  `.learned_role`, next to `group.*`, `section.*`, `unit.*` and `caution.*`.
  `unused.*` and `role.*` are the notes a value gets while another value
  leaves it unused or changes what it does (`when` in `param_metadata.py`).
  `param_metadata.py` holds only their structure.
- Values fill `{name}` placeholders. An English text that changes with a count
  is `{"one": ..., "other": ...}` and is picked by `{count}`.
- The server sends codes, never text: Panda reasons and steps, and a refusal
  (`refusal.py`) is a catalogue key the page turns into text.
- Add the Korean and the English text in the same change.
  `check_web_console.py` fails when the files differ in keys or placeholders,
  when a key is missing or unused, or when Korean is left in the code outside
  comments. A line that must keep it, such as a language's own name, carries
  an `i18n-allow` comment.

## Documents

Every document has the same shape, so a reader knows where to look.

- **Files.** `docs/<topic>.md`, in lowercase words joined by hyphens, named for
  the subject rather than a tool. A folder's own document is its `README.md`.
- **Title.** `#` and the file name in sentence case: `lane-line-filter.md` is
  `# Lane-line filter`. A folder README is titled by what the folder holds
  (`# Host tests`).
- **Back link.** The line under the title is `[← Documentation](README.md)`;
  a folder README links to `docs/README.md` by its relative path.
- **Opening.** One paragraph of one to three sentences: what this is and when
  you need it. The overview diagram, if there is one, comes next.
- **Body.** Shaped by what the document is for:

  | Kind | For | Body |
  | --- | --- | --- |
  | Guide | doing something | numbered steps (`## 1. Fetch the SDK`), then `## Check` and `## Troubleshooting` where useful |
  | Explanation | how a part works | the parts in the order data passes through them |
  | Design note | why a part is the way it is | `## Why`, `## How`, `## Evidence`, `## Limits`, `## Next` |
  | Reference | looking a value up | tables |

- **Related.** The last section, `## Related`, lists the linked documents, each
  with a few words on what it holds.
- **Headings** are in sentence case. Code names are in backticks.
- **Prose** is wrapped at 80 columns. Tables, HTML, code and diagrams are not.
- **Spelling** is British in prose (centre, colour, behaviour). Code names keep
  their own spelling.
- **Numbers** carry their unit after a space (`50 ms`, `20 Hz`, `0.5 m/s²`).
  Dates are written `2026-10-12`, and routes by their name.
- **Warnings.** `> [!WARNING]` is only for what can hurt the car, the board or
  the driver. `> [!NOTE]` is for the rare remark a reader must not miss.

`tests/check_docs.py`, run by `ctest`, checks the title, the back link, links
and anchors, the wrapping and the language of every document.

## Diagrams

Every diagram, in the root README, `docs/` and the folder READMEs, is a Mermaid
block inside the Markdown, so it changes with the text and GitHub draws it.
`docs/images/` holds only photos and screenshots: the drive clips and the HUD
renders.

- **Type.** `flowchart` for data paths and steps, `sequenceDiagram` for
  messages between processes, `stateDiagram-v2` for state machines, `gantt` for
  timelines, and `xychart-beta` for a handful of measured values.
- **Context.** A sentence before the diagram says what it shows. The diagram
  has no title of its own.
- **Size.** GitHub scales a diagram down to the width of the page column
  (about 830 px), so a wide diagram gets small text. Keep to about a dozen
  nodes, labels of a few words with detail on a second line after `<br>`, and
  run long chains top to bottom (`flowchart TB`). Mermaid wraps a label line
  longer than about 200 px; the local renderer's font wraps at about 130 px,
  so a label that fits there fits on GitHub. Where the wrapping breaks a label
  badly, front matter `config: flowchart: wrappingWidth: 400` keeps the lines
  as written (the root README's flowcharts do). A `sequenceDiagram` that is too
  wide can narrow its actors in front matter
  (`config: sequence: actorMargin: 30, width: 130`).
- **Layout.** An edge back to an earlier node (a loop) scrambles the layout;
  say in the text that a step repeats instead. A `subgraph` that links to
  nodes outside it ignores its own `direction`.
- **Charts.** An `xychart-beta` starts with this front matter, which sets its
  height and its colours (one per series, in order):

  ```text
  ---
  config:
    xyChart:
      height: 260
    themeVariables:
      xyChart:
        plotColorPalette: "#2f81f7, #f0883e, #1f9e9a"
  ---
  ```

  A `gantt` timeline in seconds uses `dateFormat x`, `axisFormat %-S s` and
  `todayMarker off`, and front matter that fixes its width: GitHub otherwise
  draws a gantt wider than the column and shrinks its text.

  ```text
  ---
  config:
    gantt:
      useWidth: 820
      fontSize: 13
      sectionFontSize: 13
      barHeight: 24
  ---
  ```
- **Colours.** Nodes are coloured by role with the same classes everywhere.
  Copy the classes a diagram uses from this block:

  ```text
  classDef pixel fill:#e7f5f4,stroke:#1f9e9a,color:#1f2328
  classDef compute fill:#ebf3fe,stroke:#2f81f7,color:#1f2328
  classDef proc fill:#f6f8fa,stroke:#8b949e,color:#1f2328
  classDef external fill:#fff3e8,stroke:#f0883e,color:#1f2328
  classDef safety fill:#eaf7ee,stroke:#2da44e,color:#1f2328
  classDef group fill:#ffffff,stroke:#d1d9e0,color:#59636e
  ```

  `pixel` is the camera's pixel path through hardware buffers (`camerad`, the
  frame ring, `overlayd`), `compute` is the model, control and localization,
  `proc` is any other process or file, `external` is outside the board (the
  Panda, the car), and `safety` is a limit. `group` is for a `subgraph` box
  (`class board group`). A dotted edge (`-.->`) is optional, learned, or
  indirect (a header read without linking). `stateDiagram-v2`, `gantt` and
  `sequenceDiagram` keep Mermaid's own colours.
- **Checking.** `tools/docs/render_mermaid.py` draws every diagram in the given
  files with mermaid-cli in Docker, and reports any that fail or come out wider
  than the column. It leaves a PNG of each in `build-docs/`; look at them, since
  a diagram can render and still be tangled. The image version is pinned
  (12.0.1, what the diagrams were checked with), so a new Mermaid release does
  not change the verdicts; when you raise it, draw every diagram again.

## Brand

The icon is a pilot's wings with the road running through them: the green
perspective road is the HUD's lane-mode path colour (`#30d158`) on the web
console's bar colour (`#16191c`). The wordmark is drawn, not set in a font:
round-capped strokes of the icon's weight, with the `i` of `pilot` as the same
green road under its vanishing point. The sources are in `docs/images/brand/`:

| File | Use |
| --- | --- |
| `edgepilot-icon.svg` | the icon from about 48 px up |
| `edgepilot-icon-small.svg` | 16 and 32 px: two wing strokes, thicker, so they stay apart (the web console favicon, `static/icon.svg`) |
| `edgepilot-wordmark.svg`, `-dark.svg` | the name alone, for light and dark backgrounds |
| `edgepilot-lockup.svg`, `-dark.svg` | icon and name, as at the top of the README |
| `edgepilot-social.png` | GitHub's social preview, 1280x640, uploaded in the repository settings |

The PNGs are screenshots of the SVGs in headless Chrome, for example
`chrome --headless=new --force-device-scale-factor=1 --window-size=180,180
--screenshot=out.png page.html`. The web console's `apple-touch-icon.png`
(180x180) is the large icon without its rounded corners, which the phone draws
itself.

## Versions

- No file carries its own version or change log. Git history is the record.
  Releases are tags `vX.Y.Z`.
- Every board binary carries the `git describe` of the tree it was built from
  ([Build version](build-and-deploy.md#build-version)).
- A data format keeps its version as a constant next to the layout and changes
  it with the layout. Examples are `kRecordingVersion`, `kFrameRingVersion`,
  `kIpcVersion`, and the `version` field of a saved JSON file.

## Blame across the reformat

The commit that brought in this style reformatted nearly every C++ line. To see
who last changed a line before it, pass that commit to `git blame --ignore-rev`.

## Related

- [Source layout](source-layout.md): where code goes and how files are named
- [Host tests](../tests/README.md): adding a test next to a change
