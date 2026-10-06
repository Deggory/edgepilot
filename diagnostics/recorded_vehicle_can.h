#ifndef RECORDED_VEHICLE_CAN_H
#define RECORDED_VEHICLE_CAN_H

/* 녹화한 CanRx 페이로드를 vehicle_can 상태에 반영한다(진단 도구 공용). 64바이트 CAN-FD 프레임은
 * 이 차와 무관해 건너뛴다. skip_echoes면 controlsd처럼 Panda가 되돌려 준 송신 프레임(flags != 0)과
 * 가상 버스(src > 7)도 건너뛴다. */

#include "recording/recorded_can.h"
#include "car/vehicle_can.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

inline void apply_recorded_can(const void *payload, size_t size, double now_s, VehicleCanState *vehicle,
                               bool skip_echoes = false)
{
    for_each_recorded_can_frame(payload, size, [&](const RecordedCanFrame &frame) {
        if (frame.data_len > 8) return;
        if (skip_echoes && (frame.flags != 0 || frame.src > 7)) return;
        std::array<uint8_t, 8> data{};
        std::memcpy(data.data(), frame.data, frame.data_len);
        update_vehicle_can_state(vehicle, frame.address, data, static_cast<uint8_t>(frame.data_len),
                                 static_cast<uint8_t>(frame.src), now_s);
    });
}

#endif
