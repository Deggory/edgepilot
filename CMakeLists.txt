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

# MaixCAM2(AX630C) 보드 이미지의 NPU 런타임. 엔진 헤더는 MSP SDK에 없어 src/ax_engine_api.h로 선언한다.
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

# 호스트에서도 컴파일되는 서브시스템 라이브러리. 각 소스는 정확히 한 곳에만 나열한다.
function(add_edgepilot_library target)
  add_library(${target} STATIC ${ARGN})
  target_include_directories(${target} PUBLIC ${PROJECT_LOCAL_INCLUDES})
  target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
endfunction()

# 프로세스 공용 코드. glibc는 shm_open이 librt에 있고 macOS는 libc에 있다.
add_edgepilot_library(common
  src/app_config.cc
  src/ipc_channels.cc
  src/model_output.cc
  src/projection.cc
)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(common PUBLIC rt pthread)
endif()

add_edgepilot_library(utils_json src/utils_json.cc)

add_edgepilot_library(control_core
  src/adaptive_cruise.cc
  src/control_holds.cc
  src/control_params.cc
  src/departure_alert.cc
  src/hyundai_can.cc
  src/lateral_controller.cc
  src/lateral_learners.cc
  src/torque_estimator.cc
  src/vehicle_params_learner.cc
  src/lateral_path.cc
  src/lateral_torque.cc
  src/vehicle_can.cc
)
target_link_libraries(control_core PUBLIC utils_json)

add_edgepilot_library(planning
  src/desire_helper.cc
  src/lateral_mpc.cc
  src/lateral_planner.cc
)
target_link_libraries(planning PUBLIC control_core common)

# controlsd 한 틱의 로직(ControlsTick). 공유 메모리 없이 돌아 호스트 검사와 재생 도구가 같은 코드를 쓴다.
add_edgepilot_library(controls src/controls_tick.cc)
target_link_libraries(controls PUBLIC planning)

add_edgepilot_library(perception
  src/calibration_service.cc
  src/model_input_transform.cc
  src/calibration_online.cc
  src/model_state_fill.cc
)
target_link_libraries(perception PUBLIC common utils_json)

add_edgepilot_library(panda_codec src/panda_can_codec.cc)

# locationd·lagd 이식. 하드웨어에 의존하지 않아 호스트에서 녹화를 재생해 검사한다.
add_edgepilot_library(localization src/location_estimator.cc src/lateral_lag.cc src/localization_pipeline.cc)
target_link_libraries(localization PUBLIC utils_json common)

# 기록 라이터는 POSIX만 써서 호스트 검사가 디스크 포맷을 본다. 인코더(VENC)는 recordd에만.
add_edgepilot_library(recording_writer src/recording_writer.cc)
target_link_libraries(recording_writer PUBLIC common)

# 오버레이 상태 매핑·알림 정책과 렌더러(자체 캔버스·구운 글꼴)는 외부 라이브러리 없이
# 호스트에서도 빌드돼 검사와 hud_snapshot이 보드와 같은 그림을 그린다.
add_edgepilot_library(overlay_state src/overlay_state.cc)
target_link_libraries(overlay_state PUBLIC common)

add_edgepilot_library(overlay src/overlay_canvas.cc src/overlay_font.cc src/overlay_renderer.cc)
target_link_libraries(overlay PUBLIC overlay_state)

# 알림음 합성(재생은 보드의 alert_sound.cc). 호스트에서 alert_sound_preview가 같은 소리를 낸다.
add_edgepilot_library(alert_tones src/alert_tones.cc)

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

  # 재생 소스는 POSIX만 쓴다.
  add_edgepilot_library(replay src/replay_source.cc)

  # MaixCAM2 카메라(VI)·화면(VO). scripts/fetch_maixcam2_sdk.sh가 받은 MSP SDK와
  # MaixCDK 헤더로 보드의 libmaixcam_lib(ax_middleware)를 직접 쓴다.
  set(MAIX_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/deps/ax630")
  set(MAIX_MSP_OVERRIDE "${MAIX_DEPS}/MaixCDK/components/3rd_party/maixcam2_msp/msp/sample/common")
  if(NOT EXISTS "${MAIX_DEPS}/MaixCDK/components/maixcam_lib/include/maixcam2/ax_middleware.hpp")
    message(FATAL_ERROR "MaixCAM2 SDK missing: run scripts/fetch_maixcam2_sdk.sh")
  endif()
  file(GLOB_RECURSE _msp_component_headers "${MAIX_DEPS}/maixcam2_msp/component/*/include/*.h")
  set(MAIX_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/platform/maixcam2"
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
  add_library(maixcam2_media STATIC
    platform/maixcam2/maix_camera.cc
    platform/maixcam2/maix_display.cc
    platform/maixcam2/maix_touch.cc
    platform/maixcam2/maix_shim.cc
    platform/maixcam2/maix_cmm.cc
    "${MAIX_MSP_OVERRIDE}/common_vin.c"
    "${MAIX_MSP_OVERRIDE}/common_isp.c"
    "${MAIX_MSP_OVERRIDE}/common_sys.c"
    "${MAIX_MSP_OVERRIDE}/common_hw.c")
  target_include_directories(maixcam2_media SYSTEM PRIVATE ${MAIX_INCLUDES})
  target_include_directories(maixcam2_media PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/platform/maixcam2")
  set_source_files_properties(
    "${MAIX_MSP_OVERRIDE}/common_vin.c" "${MAIX_MSP_OVERRIDE}/common_isp.c"
    "${MAIX_MSP_OVERRIDE}/common_sys.c" "${MAIX_MSP_OVERRIDE}/common_hw.c"
    PROPERTIES COMPILE_OPTIONS "-w")
  target_link_directories(maixcam2_media PUBLIC "${MAIX_DEPS}/maix")
  target_link_libraries(maixcam2_media PUBLIC
    -Wl,--as-needed maixcam_lib samplerate ${AX_ALL_LIBS} pthread dl)

  # 보드 실행 파일: 로컬 헤더, 경고, 공용 시스템 라이브러리.
  function(configure_board_target target)
    target_include_directories(${target} PRIVATE ${PROJECT_LOCAL_INCLUDES})
    target_compile_options(${target} PRIVATE ${PROJECT_WARNINGS})
    target_link_options(${target} PRIVATE "-Wl,-rpath,${AX_RPATH}" "-Wl,-rpath-link,${AX_LIB_DIR}")
    target_link_libraries(${target} PRIVATE ${ARGN} pthread dl rt)
  endfunction()

  add_executable(overlayd
    src/overlayd.cc
    src/alert_sound.cc
    src/system_monitor.cc
  )
  configure_board_target(overlayd common overlay alert_tones maixcam2_media utils_json)

  add_executable(camerad src/camerad.cc)
  configure_board_target(camerad common maixcam2_media)

  # 보드 IMU(LSM6DSOW)를 읽어 locationd에 넘기고 기록한다. i2c-dev만 쓴다.
  add_executable(imud src/imud.cc)
  configure_board_target(imud common)

  # 자세 칼만 필터 + 조향 지연 추정(상류 locationd·lagd). paramsd·torqued 입력과 경로 지연에 쓴다.
  add_executable(locationd src/locationd.cc)
  configure_board_target(locationd common localization)

  # 카메라 내부 파라미터 측정용 스틸 캡처(docs/camcal.md).
  add_executable(camcal src/camcal.cc src/alert_sound.cc)
  target_include_directories(camcal SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  configure_board_target(camcal alert_tones maixcam2_media ${OpenCV_LIBS})
  target_link_options(camcal PRIVATE "-Wl,-rpath-link,${AX_OPENCV_DIR}/opencv_lib")

  # 모델 입력 워프용 GDC. MSP SDK 헤더만 쓰고 미들웨어는 쓰지 않는다(NPU 세션과 같은
  # 프로세스에서 AX SYS를 두 번 초기화하지 않도록).
  add_library(maixcam2_gdc STATIC platform/maixcam2/maix_gdc_warp.cc platform/maixcam2/maix_cmm.cc)
  target_include_directories(maixcam2_gdc PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/platform/maixcam2")
  target_include_directories(maixcam2_gdc SYSTEM PRIVATE "${MAIX_DEPS}/maixcam2_msp/out/arm64_glibc/include")
  target_compile_options(maixcam2_gdc PRIVATE ${PROJECT_WARNINGS})
  target_link_libraries(maixcam2_gdc PUBLIC "${AX_LIB_DIR}/libax_ivps.so" ${AX_SYS_LIB})

  # 녹화용 하드웨어 H.264/HEVC 인코더. GDC와 같이 MSP SDK를 직접 쓴다.
  add_library(maixcam2_venc STATIC platform/maixcam2/maix_venc.cc)
  target_include_directories(maixcam2_venc PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/platform/maixcam2")
  target_include_directories(maixcam2_venc SYSTEM PRIVATE "${MAIX_DEPS}/maixcam2_msp/out/arm64_glibc/include")
  target_compile_options(maixcam2_venc PRIVATE ${PROJECT_WARNINGS})
  # libax_venc은 VIN 연동 심볼(AX_VIN_PRIV_*)을 libax_proton에서 가져온다.
  target_link_libraries(maixcam2_venc PUBLIC "${AX_LIB_DIR}/libax_venc.so" "${AX_LIB_DIR}/libax_proton.so"
                        "${AX_LIB_DIR}/libax_ivps.so" ${AX_SYS_LIB})

  # 리허설(replayd)용 하드웨어 디코더.
  add_library(maixcam2_vdec STATIC platform/maixcam2/maix_vdec.cc)
  target_include_directories(maixcam2_vdec PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/platform/maixcam2")
  target_include_directories(maixcam2_vdec SYSTEM PRIVATE "${MAIX_DEPS}/maixcam2_msp/out/arm64_glibc/include")
  target_compile_options(maixcam2_vdec PRIVATE ${PROJECT_WARNINGS})
  target_link_libraries(maixcam2_vdec PUBLIC "${AX_LIB_DIR}/libax_vdec.so" "${AX_LIB_DIR}/libax_ivps.so" ${AX_SYS_LIB})

  add_executable(replayd src/replayd.cc)
  configure_board_target(replayd common maixcam2_vdec maixcam2_gdc)

  add_executable(recordd src/recordd.cc)
  configure_board_target(recordd common recording_writer utils_json maixcam2_venc)

  add_executable(modeld
    src/modeld.cc
    src/supercombo_model.cc
    src/ax_engine_session.cc
  )
  configure_board_target(modeld common perception replay maixcam2_gdc ${AX_ENGINE_LIB} ${AX_SYS_LIB})

  if(EDGEPILOT_BUILD_PANDA)
    add_executable(pandad
      src/pandad.cc
      src/panda_client.cc
    )
    target_include_directories(pandad SYSTEM PRIVATE ${LIBUSB_INCLUDE_DIRS})
    configure_board_target(pandad common panda_codec ${LIBUSB_LIBRARIES})
  endif()

  add_executable(controlsd src/controlsd.cc)
  configure_board_target(controlsd common control_core planning controls)
endif()

if(EDGEPILOT_BUILD_DIAGNOSTICS)
  # 단위 테스트는 호스트에서만 빌드·실행한다. 교차 빌드에는 gtest를 링크할 pthread도,
  # 테스트를 돌릴 곳도 없다. enable_testing()은 루트에 있어야 ctest가 하위 폴더를 찾는다.
  if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_subdirectory(gtest)
  endif()
  add_subdirectory(diagnostics)
endif()
