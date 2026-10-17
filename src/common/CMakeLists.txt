# 프로세스 공용 코드: 설정, 공유 메모리 채널과 메시지, 모델 출력 형식, 투영, 유틸.
# glibc는 shm_open이 librt에 있고 macOS는 libc에 있다.
add_edgepilot_library(common
  app_config.cc
  ipc_channels.cc
  model_output.cc
  projection.cc
)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(common PUBLIC rt pthread)
endif()

add_edgepilot_library(utils_json utils_json.cc json_value.cc lateral_nn_path.cc)

# 빌드한 소스의 버전(common/build_version.h). 빌드마다 cmake/build_version.cmake가 git describe를 다시 읽고,
# 바뀌었을 때만 다시 컴파일한다. OBJECT 라이브러리라 링크한 실행 파일마다 웹 콘솔이 읽는 표식이 들어간다.
set(BUILD_VERSION_CC "${CMAKE_BINARY_DIR}/generated/build_version.cc")
add_custom_target(build_version_stamp
  COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${PROJECT_SOURCE_DIR} -DOUTPUT=${BUILD_VERSION_CC}
          -P ${PROJECT_SOURCE_DIR}/cmake/build_version.cmake
  BYPRODUCTS ${BUILD_VERSION_CC}
  VERBATIM)
add_library(build_version OBJECT ${BUILD_VERSION_CC})
add_dependencies(build_version build_version_stamp)
target_include_directories(build_version PUBLIC ${PROJECT_LOCAL_INCLUDES})
