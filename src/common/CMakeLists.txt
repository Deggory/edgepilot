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

add_edgepilot_library(utils_json utils_json.cc)
