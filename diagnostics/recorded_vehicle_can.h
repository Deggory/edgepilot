#ifndef RECORDED_VEHICLE_CAN_H
#define RECORDED_VEHICLE_CAN_H

/* 녹화한 CanRx 페이로드를 vehicle_can 상태에 반영한다(진단 도구 공용). 64바이트 CAN-FD 프레임은
 * 이 차와 무관해 건너뛴다. */

#include "recorded_can.h"
#include "vehicle_can.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

inline void apply_recorded_can(const void *payload, size_t size, double now_s, VehicleCanState *vehicle)
{
    for_each_recorded_can_frame(payload, size, [&](const RecordedCanFrame &frame) {
        if (frame.data_len > 8) return;
        std::array<uint8_t, 8> data{};
        std::memcpy(data.data(), frame.data, frame.data_len);
        update_vehicle_can_state(vehicle, frame.address, data, static_cast<uint8_t>(frame.data_len),
                                 static_cast<uint8_t>(frame.src), now_s);
    });
}

#endif
