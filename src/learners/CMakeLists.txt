# paramsd·torqued 이식. controlsd 안에서 돌고, 재생 도구가 녹화로 다시 돌린다.
add_edgepilot_library(learners
  lateral_learners.cc
  torque_estimator.cc
  vehicle_params_learner.cc
)
target_link_libraries(learners PUBLIC control_core)
