# 녹화. 이벤트 로그 형식과 쓰기, 상태 채널 기록, 리허설 route 읽기는 POSIX만 써서 호스트 검사가 디스크
# 형식을 본다. 인코더(VENC)는 recordd에만, 디코더(VDEC)는 replayd에만.
add_edgepilot_library(recording recording_writer.cc replay_route.cc state_recorder.cc)
target_link_libraries(recording PUBLIC common)

if(EDGEPILOT_BUILD_RUNTIME)
  add_executable(recordd recordd.cc)
  configure_board_target(recordd common recording utils_json maixcam2_venc)

  add_executable(replayd replayd.cc)
  configure_board_target(replayd common recording maixcam2_vdec maixcam2_gdc)
endif()
