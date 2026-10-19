#!/usr/bin/env bash
# 호스트에서 도는 테스트를 전부 빌드하고 ctest로 돌린다. 보드도 교차 툴체인도 필요 없다.
# C++ 테스트는 googletest(처음 configure 때 내려받는다), check_web_console.py·check_manager.py는 stdlib만 쓴다.
# check_recording_reader.py는 numpy가 있어야 돌고, 없으면 ctest가 건너뛴 것으로 센다.
# 하나만 돌리려면 ctest --test-dir build-host -R <이름>, 또는 build-host/bin/<테스트> --gtest_filter=<패턴>
# 사용: scripts/run_host_tests.sh [--linux]
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

build_dir="${EDGEPILOT_HOST_BUILD_DIR:-build-host}"
jobs="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)}"

# --linux: 보드와 같은 컴파일러(GCC 11, 도커 이미지 tools/docker_ax630)로 build-linux-test에서 돈다. macOS에는
# futex가 없어 IPC 대기 코드가 Mac에서는 컴파일되지 않고, clang과 GCC는 부동소수 축약(FMA)이 달라 재생 도구의
# 다이제스트도 다르다(docs/diagnostics.md). 보드 바이너리에 대한 '동작 변화 없음'은 이 빌드로 확인한다.
if [ "${1:-}" = "--linux" ]; then
  docker build -q -t edgepilot-ax630-build tools/docker_ax630 >/dev/null
  gtest_source=""
  gtest_dir=build-host/_deps/googletest-src
  [ -d "${gtest_dir}" ] && gtest_source="-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/src/${gtest_dir}"
  version="$(git describe --tags --always --dirty --match 'v[0-9]*' 2>/dev/null || echo unknown)"
  exec docker run --rm -e EDGEPILOT_VERSION="${version}" -v "${repo_dir}":/src -w /src edgepilot-ax630-build bash -c "
    cmake -S . -B build-linux-test -DCMAKE_BUILD_TYPE=Release -DEDGEPILOT_BUILD_RUNTIME=OFF \
          -DEDGEPILOT_BUILD_DIAGNOSTICS=ON ${gtest_source} >/dev/null &&
    cmake --build build-linux-test -j\$(nproc) >/dev/null &&
    ctest --test-dir build-linux-test -j\$(nproc) --output-on-failure"
fi

cmake -S . -B "${build_dir}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DEDGEPILOT_BUILD_RUNTIME=OFF \
  -DEDGEPILOT_BUILD_DIAGNOSTICS=ON >/dev/null
cmake --build "${build_dir}" -j"${jobs}" --target host_tests >/dev/null
ctest --test-dir "${build_dir}" -j"${jobs}" --output-on-failure
