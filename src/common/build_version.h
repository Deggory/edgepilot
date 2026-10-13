#ifndef BUILD_VERSION_H
#define BUILD_VERSION_H

// 빌드한 소스의 버전. cmake/build_version.cmake가 빌드할 때마다 git describe(태그-커밋 수-g해시, 커밋하지
// 않은 변경이 있으면 -dirty)를 새긴다. git이 없으면 "unknown"이다. 보드 실행 파일은 모두 이것을 링크한다
// (CMakeLists.txt configure_board_target).

#include <cstdio>

// 실행 파일 안에서 버전 바로 앞에 붙는 표식. 웹 콘솔(process_status.py VERSION_MARKER)이 같은 값으로 찾는다.
#define EDGEPILOT_VERSION_MARKER_PREFIX "EDGEPILOT_VERSION="

extern const char kBuildVersionMarker[];  // 표식 + 버전
const char *edgepilot_version();

// 시작 로그 한 줄: "<name>: edgepilot <버전>"
inline void log_build_version(const char *name) {
    std::fprintf(stderr, "%s: edgepilot %s\n", name, edgepilot_version());
}

#endif
