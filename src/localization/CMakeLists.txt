# locationd·lagd 이식과 보드 IMU. 하드웨어에 의존하지 않는 부분은 호스트에서 녹화를 재생해 검사한다.
add_edgepilot_library(localization
  location_estimator.cc
  lateral_lag.cc
  localization_pipeline.cc
)
target_link_libraries(localization PUBLIC utils_json common)

if(EDGEPILOT_BUILD_RUNTIME)
  # 보드 IMU(LSM6DSOW)를 읽어 locationd에 넘기고 기록한다. i2c-dev만 쓴다.
  add_executable(imud imud.cc)
  configure_board_target(imud common)

  # 자세 칼만 필터 + 조향 지연 추정(상류 locationd·lagd). paramsd·torqued 입력과 경로 지연에 쓴다.
  add_executable(locationd locationd.cc)
  configure_board_target(locationd common localization)
endif()
