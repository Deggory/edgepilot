# panda USB 브리지: CAN 패킹은 호스트에서도 빌드하고, pandad는 보드에서만.
add_edgepilot_library(panda panda_can_codec.cc)

if(EDGEPILOT_BUILD_RUNTIME AND EDGEPILOT_BUILD_PANDA)
  add_executable(pandad
    pandad.cc
    panda_client.cc
  )
  target_include_directories(pandad SYSTEM PRIVATE ${LIBUSB_INCLUDE_DIRS})
  configure_board_target(pandad common panda ${LIBUSB_LIBRARIES})
endif()
