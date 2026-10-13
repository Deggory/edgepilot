# Code style

[← Documentation index](../README.md)

## Languages

- Code comments and docstrings are Korean, in the plain written style (`~다`),
  in every language of the repository: C++, Python, JavaScript, shell, CMake.
- Identifiers, log and error messages, and commit messages are English.
  Commit subjects read `<area>: <what changed>`.
- `docs/` and the root README are English. READMEs inside folders keep the
  language they are written in.
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
- `// clang-format off` and `// clang-format on` are only for layouts that
  carry meaning: the row-major 3x3 matrices, and the audio stub macro table in
  `maix_shim.cc`.
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
