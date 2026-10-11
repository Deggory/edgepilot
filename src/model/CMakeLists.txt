# modeld. 모델 입력 워프, 온라인 보정, 광류 보정 풀이, ModelState 채우기는 호스트에서도 빌드하고, NPU 세션,
# 재생 소스, 광류 보정기의 IVE 작업 스레드는 보드에서만.
add_edgepilot_library(model
  calibration_service.cc
  calibration_source.cc
  model_input_transform.cc
  calibration_online.cc
  flow_calibration.cc
  model_state_fill.cc
)
target_link_libraries(model PUBLIC common utils_json)

if(EDGEPILOT_BUILD_RUNTIME)
  # 재생 소스는 POSIX만 쓴다.
  add_edgepilot_library(replay replay_source.cc)

  add_executable(modeld
    modeld.cc
    flow_calibration_worker.cc
    supercombo_model.cc
    ax_engine_session.cc
  )
  configure_board_target(modeld common model replay maixcam2_gdc maixcam2_ive ${AX_ENGINE_LIB} ${AX_SYS_LIB})
endif()
