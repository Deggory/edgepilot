#ifndef RECORDED_CAN_H
#define RECORDED_CAN_H

/* 녹화된 CanRx/CanTx 페이로드: RecordedCanBatchHeader 뒤에 RecordedCanFrame이 count개.
 * recordd가 쓰고 replayd와 진단 도구가 읽는다. 읽을 때는 페이로드에 다 들어 있는 프레임만 본다
 * (끊긴 꼬리는 버린다). */

#include "ipc_messages.h"
#include "recording_format.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

// 배치를 디스크 형식으로 직렬화한다. 프레임은 kCanBatchMaxFrames개까지만 쓴다.
inline std::vector<uint8_t> encode_recorded_can(const CanBatch &batch)
{
    const uint32_t count = std::min<uint32_t>(batch.count, kCanBatchMaxFrames);
    std::vector<uint8_t> payload(sizeof(RecordedCanBatchHeader) + count * sizeof(RecordedCanFrame));
    const RecordedCanBatchHeader header{count, batch.dropped};
    std::memcpy(payload.data(), &header, sizeof(header));
    uint8_t *out = payload.data() + sizeof(header);
    for (uint32_t index = 0; index < count; ++index, out += sizeof(RecordedCanFrame)) {
        const IpcCanFrame &source = batch.frames[index];
        RecordedCanFrame recorded;
        recorded.address = source.address;
        recorded.src = source.src;
        recorded.bus_time = source.bus_time;
        recorded.data_len = source.data_len;
        recorded.flags = source.flags;
        std::memcpy(recorded.data, source.data, sizeof(recorded.data));
        std::memcpy(out, &recorded, sizeof(recorded));
    }
    return payload;
}

/* 페이로드의 프레임을 순서대로 fn(const RecordedCanFrame &)에 넘기고 넘긴 수를 돌려준다.
 * 머리보다 짧으면 아무것도 넘기지 않는다. 머리의 dropped가 필요하면 dropped에 받는다. */
template <typename Fn>
uint32_t for_each_recorded_can_frame(const void *payload, size_t size, Fn &&fn,
                                     uint32_t *dropped = nullptr)
{
    RecordedCanBatchHeader header;
    if (!payload || size < sizeof(header)) return 0;
    const uint8_t *bytes = static_cast<const uint8_t *>(payload);
    std::memcpy(&header, bytes, sizeof(header));
    if (dropped) *dropped = header.dropped;
    const uint32_t count = std::min<uint32_t>(
        header.count, static_cast<uint32_t>((size - sizeof(header)) / sizeof(RecordedCanFrame)));
    for (uint32_t index = 0; index < count; ++index) {
        RecordedCanFrame frame;
        std::memcpy(&frame, bytes + sizeof(header) + index * sizeof(frame), sizeof(frame));
        fn(frame);
    }
    return count;
}

// 페이로드를 CanBatch로 되돌린다(replayd). 프레임은 kCanBatchMaxFrames개에서도 자른다.
inline CanBatch decode_recorded_can(const void *payload, size_t size, uint64_t timestamp_ns)
{
    CanBatch batch;
    if (!payload || size < sizeof(RecordedCanBatchHeader)) return batch;
    batch.timestamp_ns = timestamp_ns;
    batch.valid = 1;
    for_each_recorded_can_frame(payload, size, [&batch](const RecordedCanFrame &frame) {
        if (batch.count >= kCanBatchMaxFrames) return;
        IpcCanFrame &out = batch.frames[batch.count++];
        out.address = frame.address;
        out.src = frame.src;
        out.bus_time = frame.bus_time;
        out.data_len = frame.data_len;
        out.flags = frame.flags;
        std::memcpy(out.data, frame.data, sizeof(out.data));
    }, &batch.dropped);
    return batch;
}

#endif
