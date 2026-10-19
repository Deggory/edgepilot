#ifndef RECORDED_CAN_H
#define RECORDED_CAN_H

// 녹화된 CanRx/CanTx 페이로드: RecordedCanBatchHeader 뒤에 RecordedCanFrame이 count개.
// recordd가 쓰고 replayd와 진단 도구가 읽는다. 읽을 때는 페이로드에 다 들어 있는 프레임만 본다
// (끊긴 꼬리는 버린다). 디스크의 프레임은 IpcCanFrame과 같은 바이트라 통째로 복사한다.

#include "common/ipc_messages.h"
#include "recording/recording_format.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

static_assert(sizeof(RecordedCanFrame) == sizeof(IpcCanFrame) &&
                  offsetof(RecordedCanFrame, flags) == offsetof(IpcCanFrame, flags) &&
                  offsetof(RecordedCanFrame, data) == offsetof(IpcCanFrame, data),
              "recorded CAN frames are IpcCanFrame bytes");

// 배치를 디스크 형식으로 직렬화한다. 프레임은 kCanBatchMaxFrames개까지만 쓴다.
inline std::vector<uint8_t> encode_recorded_can(const CanBatch &batch) {
    const uint32_t count = std::min<uint32_t>(batch.count, kCanBatchMaxFrames);
    std::vector<uint8_t> payload(sizeof(RecordedCanBatchHeader) + count * sizeof(RecordedCanFrame));
    const RecordedCanBatchHeader header{count, batch.dropped};
    std::memcpy(payload.data(), &header, sizeof(header));
    std::memcpy(payload.data() + sizeof(header), batch.frames, count * sizeof(RecordedCanFrame));
    return payload;
}

// 페이로드의 프레임을 순서대로 fn(const RecordedCanFrame &)에 넘기고 넘긴 수를 돌려준다.
// 머리보다 짧으면 아무것도 넘기지 않는다. 머리의 dropped가 필요하면 dropped에 받는다.
template <typename Fn>
uint32_t for_each_recorded_can_frame(const void *payload, size_t size, Fn &&fn, uint32_t *dropped = nullptr) {
    RecordedCanBatchHeader header;
    if (size < sizeof(header)) return 0;
    const uint8_t *bytes = static_cast<const uint8_t *>(payload);
    std::memcpy(&header, bytes, sizeof(header));
    if (dropped) *dropped = header.dropped;
    const uint32_t count =
        std::min<uint32_t>(header.count, static_cast<uint32_t>((size - sizeof(header)) / sizeof(RecordedCanFrame)));
    for (uint32_t index = 0; index < count; ++index) {
        RecordedCanFrame frame;
        std::memcpy(&frame, bytes + sizeof(header) + index * sizeof(frame), sizeof(frame));
        fn(frame);
    }
    return count;
}

// 페이로드를 out에 되돌린다(replayd, replay_controls). 머리와 count개 프레임만 쓰므로(받는 쪽은 count까지만
// 읽는다) 한 배치를 계속 다시 쓸 수 있다. 프레임은 kCanBatchMaxFrames개에서도 자른다. 머리보다 짧으면 false.
inline bool decode_recorded_can(const void *payload, size_t size, uint64_t timestamp_ns, CanBatch *out) {
    RecordedCanBatchHeader header;
    if (size < sizeof(header)) return false;
    std::memcpy(&header, payload, sizeof(header));
    const size_t whole = (size - sizeof(header)) / sizeof(RecordedCanFrame);
    out->timestamp_ns = timestamp_ns;
    out->valid = 1;
    out->count = static_cast<uint32_t>(std::min<size_t>({header.count, whole, kCanBatchMaxFrames}));
    out->dropped = header.dropped;
    std::memcpy(out->frames, static_cast<const uint8_t *>(payload) + sizeof(header), out->count * sizeof(IpcCanFrame));
    return true;
}

#endif
