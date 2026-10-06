# 런타임 코드. 폴더마다 자기 라이브러리(호스트에서도 빌드)와 보드 프로세스를 정의한다. 헤더는 src/ 기준
# 경로로 부른다(#include "controls/adaptive_cruise.h"). 라이브러리 사이 의존은 아래에서 위로만 간다:
# common ← car ← controls(control_core) ← planning·learners ← controls(controls).
add_subdirectory(common)
add_subdirectory(car)
add_subdirectory(panda)
add_subdirectory(controls)
add_subdirectory(planning)
add_subdirectory(learners)
add_subdirectory(localization)
add_subdirectory(model)
add_subdirectory(camera)
add_subdirectory(hud)
add_subdirectory(recording)
