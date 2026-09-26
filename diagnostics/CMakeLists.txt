# 분석·재생·벤치 도구. 호스트 도구는 보드 라이브러리 없이 빌드된다.
add_host_tool(extract_lateral_dataset extract_lateral_dataset.cc)
target_link_libraries(extract_lateral_dataset PRIVATE control_core)

add_host_tool(replay_closed_loop replay_closed_loop.cc)
target_link_libraries(replay_closed_loop PRIVATE planning)

add_host_tool(replay_lateral_learners replay_lateral_learners.cc)
target_link_libraries(replay_lateral_learners PRIVATE control_core)

add_host_tool(replay_planner replay_planner.cc)
target_link_libraries(replay_planner PRIVATE planning)

# HUD 스냅샷·타이밍 도구. overlay 라이브러리와 같은 OpenCV 조건을 따른다.
if(OpenCV_FOUND)
  add_host_tool(hud_snapshot hud_snapshot.cc)
  target_include_directories(hud_snapshot SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(hud_snapshot PRIVATE overlay)
endif()
