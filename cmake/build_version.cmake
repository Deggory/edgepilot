# 빌드할 때마다 도는 스크립트(src/common/CMakeLists.txt의 build_version_stamp). 소스의 git describe를 읽어
# OUTPUT(build_version.cc)에 새기고, 내용이 같으면 파일을 그대로 두어 다시 컴파일하지 않는다.
# EDGEPILOT_VERSION 환경 변수가 있으면 git 대신 그 값을 쓴다. 도커 보드 빌드는 호스트에서 읽어 넘기고
# (tools/docker_ax630/build.sh), 소스만 바꾼 두 빌드의 바이너리를 비교할 때는 같은 값으로 고정한다.
# 사용: cmake -DSOURCE_DIR=<저장소> -DOUTPUT=<build_version.cc> -P cmake/build_version.cmake
if(NOT "$ENV{EDGEPILOT_VERSION}" STREQUAL "")
  set(version "$ENV{EDGEPILOT_VERSION}")
else()
  # 태그-커밋 수-g해시, 커밋하지 않은 변경이 있으면 -dirty. 저장소 주인이 다른 사용자여도 읽는다.
  execute_process(
    COMMAND git -c safe.directory=* describe --tags --always --dirty --match "v[0-9]*"
    WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE version
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE result
    ERROR_QUIET)
  if(NOT result EQUAL 0 OR version STREQUAL "")
    set(version "unknown")
  endif()
endif()
# C 문자열과 웹 콘솔이 그대로 보여 줄 수 있는 글자만 남긴다.
string(REGEX REPLACE "[^A-Za-z0-9._+-]" "_" version "${version}")
string(SUBSTRING "${version}" 0 64 version)
configure_file("${CMAKE_CURRENT_LIST_DIR}/build_version.cc.in" "${OUTPUT}" @ONLY)
