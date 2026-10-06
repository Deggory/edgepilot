# K7 YG HEV CAN: 주소·버스 표(can_frame.h), 받은 프레임 해석(vehicle_can), 보낼 프레임 구성(hyundai_can).
add_edgepilot_library(car
  hyundai_can.cc
  vehicle_can.cc
)
