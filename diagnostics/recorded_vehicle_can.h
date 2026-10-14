#ifndef RECORDED_VEHICLE_CAN_H
#define RECORDED_VEHICLE_CAN_H

// 녹화한 CanRx 페이로드를 vehicle_can 상태에 반영한다(진단 도구 공용). controlsd와 같은 규칙
// (is_vehicle_rx_frame)으로 CAN-FD 길이, Panda가 되돌려 준 송신 프레임(flags != 0)과 가상 버스(src > 7)를
// 건너뛴다.

#include "car/vehicle_can.h"
#include "recording/recorded_can.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

inline void apply_recorded_can(const void *payload, size_t size, double now_s, VehicleCanState *vehicle) {
    for_each_recorded_can_frame(payload, size, [&](const RecordedCanFrame &frame) {
        // controlsd와 같이 송신 에코는 뺀다: 비전 크루즈가 보낸 CLU11 버튼이 차의 CLU11로 읽히지 않게
        if (!is_vehicle_rx_frame(frame.flags, frame.data_len, frame.src)) return;
        std::array<uint8_t, 8> data{};
        std::memcpy(data.data(), frame.data, frame.data_len);
        update_vehicle_can_state(vehicle, frame.address, data, static_cast<uint8_t>(frame.data_len),
                                 static_cast<uint8_t>(frame.src), now_s);
    });
}

#endif
