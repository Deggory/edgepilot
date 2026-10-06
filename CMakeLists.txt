cmake_minimum_required(VERSION 3.16)

project(edgepilot LANGUAGES C CXX)

option(EDGEPILOT_BUILD_DIAGNOSTICS "Build the host unit tests and diagnostic tools" OFF)
option(EDGEPILOT_BUILD_PANDA "Build optional panda USB/CAN bridge" ON)
option(EDGEPILOT_BUILD_RUNTIME "Build AX630C runtime binaries" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
foreach(config_type DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_type} "${CMAKE_BINARY_DIR}/bin")
endforeach()

# MaixCAM2(AX630C) 보드 이미지의 NPU 런타임. 엔진 헤더는 MSP SDK에 없어 src/model/ax_engine_api.h로 선언한다.
set(AX_LIB_DIR "/opt/lib" CACHE PATH "AX630C runtime library directory to link against")
set(AX_RPATH "/opt/lib" CACHE PATH "where the board keeps libax_engine/libax_sys at run time")

set(PROJECT_WARNINGS -Wall -Wextra)
set(PROJECT_LOCAL_INCLUDES
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

# 카메라 보정 캡처(camcal)만 쓴다. 보드 빌드는 보드의 OpenCV 4.11을
# deps/ax630(include/opencv4, opencv_lib)에 받아 그대로 링크한다(ABI를 맞춘다).
set(AX_OPENCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ax630" CACHE PATH
    "board OpenCV copy: include/opencv4 and opencv_lib/libopencv_*.so")
find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs)
# 보드 OpenCV는 aarch64 Linux 바이너리라 macOS 호스트 빌드에서는 링크할 수 없다.
if(NOT OpenCV_FOUND AND CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
   EXISTS "${AX_OPENCV_DIR}/include/opencv4/opencv2/core.hpp")
  set(OpenCV_FOUND TRUE)
  set(OpenCV_INCLUDE_DIRS "${AX_OPENCV_DIR}/include/opencv4")
  set(OpenCV_LIBS
    "${AX_OPENCV_DIR}/opencv_lib/libopencv_imgcodecs.so"
    "${AX_OPENCV_DIR}/opencv_lib/libopencv_imgproc.so"
    "${AX_OPENCV_DIR}/opencv_lib/libopencv_core.so")
  message(STATUS "OpenCV from ${AX_OPENCV_DIR}")
endif()


# 라이브러리: 로컬 헤더(src/ 기준 경로로 부른다)와 경고. 폴더마다 src/<폴더>/CMakeLists.txt가 정의한다.
function(add_edgepilot_library target)
  add_library(${target} STATIC ${ARGN})
  target_include_directories(${target} PUBLIC ${PROJECT_LOCAL_INCLUDES})
  target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
endfunction()

# 호스트 검사·도구: 로컬 헤더와 경고만 붙인 일반 실행 파일.
function(add_host_tool target)
  add_executable(${target} ${ARGN})
  target_include_directories(${target} PRIVATE ${PROJECT_LOCAL_INCLUDES})
  target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
endfunction()

if(EDGEPILOT_BUILD_RUNTIME)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "camcal needs the board OpenCV in ${AX_OPENCV_DIR} "
                        "(scripts/fetch_maixcam2_sdk.sh copies it)")
  endif()
  find_package(PkgConfig QUIET)
  if(EDGEPILOT_BUILD_PANDA)
    if(PkgConfig_FOUND)
      pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    else()
      message(FATAL_ERROR "PkgConfig is required when EDGEPILOT_BUILD_PANDA=ON")
    endif()
  endif()

  find_library(AX_ENGINE_LIB ax_engine PATHS "${AX_LIB_DIR}" NO_DEFAULT_PATH)
  find_library(AX_SYS_LIB ax_sys PATHS "${AX_LIB_DIR}" NO_DEFAULT_PATH)
  if(NOT AX_ENGINE_LIB OR NOT AX_SYS_LIB)
    message(FATAL_ERROR "libax_engine/libax_sys not found in ${AX_LIB_DIR} (set AX_LIB_DIR)")
  endif()

  # MaixCAM2 카메라(VI)·화면(VO). scripts/fetch_maixcam2_sdk.sh가 받은 MSP SDK와
  # MaixCDK 헤더로 보드의 libmaixcam_lib(ax_middleware)를 직접 쓴다.
  set(MAIX_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/deps/ax630")
  set(MAIX_MSP_OVERRIDE "${MAIX_DEPS}/MaixCDK/components/3rd_party/maixcam2_msp/msp/sample/common")
  if(NOT EXISTS "${MAIX_DEPS}/MaixCDK/components/maixcam_lib/include/maixcam2/ax_middleware.hpp")
    message(FATAL_ERROR "MaixCAM2 SDK missing: run scripts/fetch_maixcam2_sdk.sh")
  endif()
  file(GLOB_RECURSE _msp_component_headers "${MAIX_DEPS}/maixcam2_msp/component/*/include/*.h")
  set(MAIX_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/platform/maixcam2/stub"
    "${MAIX_DEPS}/MaixCDK/components/maixcam_lib/include/maixcam2"
    "${MAIX_DEPS}/MaixCDK/components/basic/include"
    "${MAIX_DEPS}/MaixCDK/components/vision/include"
    "${MAIX_MSP_OVERRIDE}"
    "${MAIX_DEPS}/maixcam2_msp/out/arm64_glibc/include"
    "${MAIX_DEPS}/maixcam2_msp/sample/common")
  foreach(_h IN LISTS _msp_component_headers)
    get_filename_component(_d "${_h}" DIRECTORY)
    list(APPEND MAIX_INCLUDES "${_d}")
  endforeach()
  list(REMOVE_DUPLICATES MAIX_INCLUDES)
  file(GLOB AX_ALL_LIBS "${AX_LIB_DIR}/libax_*.so")

  # 보드 SDK 래퍼(maixcam2_media·gdc·venc·vdec). 헤더는 platform/ 기준 경로로 부른다
  # (#include "maixcam2/maix_display.h").
  add_subdirectory(platform/maixcam2)

  # 보드 실행 파일: 로컬 헤더, 경고, 공용 시스템 라이브러리. 폴더마다 자기 프로세스를 정의한다.
  function(configure_board_target target)
    target_include_directories(${target} PRIVATE ${PROJECT_LOCAL_INCLUDES})
    target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
    target_link_options(${target} PRIVATE "-Wl,-rpath,${AX_RPATH}" "-Wl,-rpath-link,${AX_LIB_DIR}")
    target_link_libraries(${target} PRIVATE ${ARGN} pthread dl rt)
  endfunction()
endif()

# 런타임 코드(src/<하위 시스템>/): 호스트에서도 빌드되는 라이브러리와, 보드에서만 빌드되는 프로세스.
add_subdirectory(src)

if(EDGEPILOT_BUILD_DIAGNOSTICS)
  # 단위 테스트는 호스트에서만 빌드·실행한다. 교차 빌드에는 gtest를 링크할 pthread도,
  # 테스트를 돌릴 곳도 없다. enable_testing()은 루트에 있어야 ctest가 하위 폴더를 찾는다.
  if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_subdirectory(gtest)
  endif()
  add_subdirectory(diagnostics)
endif()
