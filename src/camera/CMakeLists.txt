# camerad와 카메라 내부 파라미터 측정용 스틸 캡처(camcal, docs/camcal.md). 보드에서만 빌드한다.
if(EDGEPILOT_BUILD_RUNTIME)
  add_executable(camerad camerad.cc)
  configure_board_target(camerad common maixcam2_media)

  add_executable(camcal camcal.cc)
  target_include_directories(camcal SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  configure_board_target(camcal alert_sound maixcam2_media ${OpenCV_LIBS})
  target_link_options(camcal PRIVATE "-Wl,-rpath-link,${AX_OPENCV_DIR}/opencv_lib")
endif()
