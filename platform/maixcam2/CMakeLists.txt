# MaixCAM2 보드 SDK 래퍼. 헤더는 platform/ 기준 경로로 부른다(#include "maixcam2/maix_display.h").
# MAIX_DEPS·MAIX_INCLUDES·AX_ALL_LIBS는 루트 CMakeLists.txt가 정한다.

# 카메라(VI)·화면(VO)·터치. 보드의 libmaixcam_lib(ax_middleware)를 직접 쓴다.
add_library(maixcam2_media STATIC
  maix_camera.cc
  maix_display.cc
  maix_touch.cc
  maix_shim.cc
  maix_cmm.cc
  "${MAIX_MSP_OVERRIDE}/common_vin.c"
  "${MAIX_MSP_OVERRIDE}/common_isp.c"
  "${MAIX_MSP_OVERRIDE}/common_sys.c"
  "${MAIX_MSP_OVERRIDE}/common_hw.c")
target_include_directories(maixcam2_media SYSTEM PRIVATE ${MAIX_INCLUDES})
target_include_directories(maixcam2_media PUBLIC "${PROJECT_SOURCE_DIR}/platform")
set_source_files_properties(
  "${MAIX_MSP_OVERRIDE}/common_vin.c" "${MAIX_MSP_OVERRIDE}/common_isp.c"
  "${MAIX_MSP_OVERRIDE}/common_sys.c" "${MAIX_MSP_OVERRIDE}/common_hw.c"
  PROPERTIES COMPILE_OPTIONS "-w")
target_link_directories(maixcam2_media PUBLIC "${MAIX_DEPS}/maix")
target_link_libraries(maixcam2_media PUBLIC
  -Wl,--as-needed maixcam_lib samplerate ${AX_ALL_LIBS} pthread dl)

# 모델 입력 워프용 GDC. MSP SDK 헤더만 쓰고 미들웨어는 쓰지 않는다(NPU 세션과 같은
# 프로세스에서 AX SYS를 두 번 초기화하지 않도록).
add_library(maixcam2_gdc STATIC maix_gdc_warp.cc maix_cmm.cc)
target_include_directories(maixcam2_gdc PUBLIC "${PROJECT_SOURCE_DIR}/platform")
target_include_directories(maixcam2_gdc SYSTEM PRIVATE "${MAIX_DEPS}/maixcam2_msp/out/arm64_glibc/include")
target_compile_options(maixcam2_gdc PRIVATE ${PROJECT_WARNINGS})
target_link_libraries(maixcam2_gdc PUBLIC "${AX_LIB_DIR}/libax_ivps.so" ${AX_SYS_LIB})

# 녹화용 하드웨어 H.264 인코더. GDC와 같이 MSP SDK를 직접 쓴다.
add_library(maixcam2_venc STATIC maix_venc.cc)
target_include_directories(maixcam2_venc PUBLIC "${PROJECT_SOURCE_DIR}/platform")
target_include_directories(maixcam2_venc SYSTEM PRIVATE "${MAIX_DEPS}/maixcam2_msp/out/arm64_glibc/include")
target_compile_options(maixcam2_venc PRIVATE ${PROJECT_WARNINGS})
# libax_venc은 VIN 연동 심볼(AX_VIN_PRIV_*)을 libax_proton에서 가져온다.
target_link_libraries(maixcam2_venc PUBLIC "${AX_LIB_DIR}/libax_venc.so" "${AX_LIB_DIR}/libax_proton.so"
                      "${AX_LIB_DIR}/libax_ivps.so" ${AX_SYS_LIB})

# 리허설(replayd)용 하드웨어 디코더.
add_library(maixcam2_vdec STATIC maix_vdec.cc)
target_include_directories(maixcam2_vdec PUBLIC "${PROJECT_SOURCE_DIR}/platform")
target_include_directories(maixcam2_vdec SYSTEM PRIVATE "${MAIX_DEPS}/maixcam2_msp/out/arm64_glibc/include")
target_compile_options(maixcam2_vdec PRIVATE ${PROJECT_WARNINGS})
target_link_libraries(maixcam2_vdec PUBLIC "${AX_LIB_DIR}/libax_vdec.so" "${AX_LIB_DIR}/libax_ivps.so" ${AX_SYS_LIB})
