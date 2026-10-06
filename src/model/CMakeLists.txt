# modeld. 모델 입력 워프, 온라인 보정, ModelState 채우기는 호스트에서도 빌드하고, NPU 세션과 재생 소스는
# 보드에서만.
add_edgepilot_library(model
  calibration_service.cc
  model_input_transform.cc
  calibration_online.cc
  model_state_fill.cc
)
target_link_libraries(model PUBLIC common utils_json)

if(EDGEPILOT_BUILD_RUNTIME)
  # 재생 소스는 POSIX만 쓴다.
  add_edgepilot_library(replay replay_source.cc)

  add_executable(modeld
    modeld.cc
    supercombo_model.cc
    ax_engine_session.cc
  )
  configure_board_target(modeld common model replay maixcam2_gdc ${AX_ENGINE_LIB} ${AX_SYS_LIB})
endif()
