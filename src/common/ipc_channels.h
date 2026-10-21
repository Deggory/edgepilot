#pragma once

// /dev/shm 채널 구현: 최신값 채널, CAN 큐, NV12 프레임 링. 레이아웃은
// ipc_messages.h에 있다.

#include "common/ipc_messages.h"

#include <cstddef>
#include <cstdint>

// shm_open한 이름 하나와 그 매핑. 세 채널이 열기·크기 조정·매핑·닫기 순서를 공유하고,
// 크기 정책(만들 때 늘릴지, 붙을 때 얼마를 요구할지)만 각자 정한다.
class ShmRegion {
public:
    ShmRegion() = default;
    ~ShmRegion() { close(); }
    ShmRegion(const ShmRegion &) = delete;
    ShmRegion &operator=(const ShmRegion &) = delete;

    bool open(const char *name, bool create);
    bool file_size(size_t *size) const;
    bool resize(size_t size);
    bool map(size_t size);
    void close();
    void *data() const { return map_; }
    size_t size() const { return size_; }

private:
    int fd_ = -1;
    void *map_ = nullptr;
    size_t size_ = 0;
};

class LatestChannel {
public:
    LatestChannel() = default;
    ~LatestChannel();

    bool open(const char *name, size_t payload_capacity, bool create);
    void close();
    bool publish(const void *payload, size_t payload_size);
    bool read(void *payload, size_t payload_capacity, uint64_t *seq = nullptr) const;
    // last_seq와 다른 새 스냅샷이면 읽고 true. 없으면 timeout_ms까지 다음 발행을 기다린다(리눅스는 futex, 그
    // 밖은 1 ms 간격 폴링). 0이면 기다리지 않고, 음수면 끝없이 기다린다. 열리지 않은 채널이면 곧바로 false다.
    bool read_new(uint64_t *last_seq, void *payload, size_t payload_capacity, int timeout_ms) const;
    bool valid() const { return header_ != nullptr; }

private:
    // seen_seq 다음 발행까지(또는 timeout_ns) 잔다.
    void wait_publish(uint64_t seen_seq, uint64_t timeout_ns) const;

    ShmRegion region_;
    IpcHeader *header_ = nullptr;
    uint8_t *payload_ = nullptr;
};

// LatestChannel 하나의 타입 있는 구독: 마지막으로 받은 스냅샷과 그 seq. poll()은 새 스냅샷을 끝까지
// 읽었을 때만 latest()를 바꾼다(찢어진 읽기는 버리고 전 값을 둔다).
template <typename T>
class Subscription {
public:
    bool open(const char *topic, bool create = true) { return channel_.open(topic, sizeof(T), create); }
    // 아직 열리지 않았으면 생산자가 만든 채널에 붙어 본다(생산자가 늦게 뜨거나 없어도 되는 구독).
    // 열려 있으면 true.
    bool attach(const char *topic) { return channel_.valid() || channel_.open(topic, sizeof(T), false); }
    // 새 스냅샷이면 latest()에 두고 true. timeout_ms까지 다음 발행을 기다릴 수 있다(read_new와 같다).
    bool poll(int timeout_ms = 0) {
        T candidate;
        uint64_t seq = seq_;
        if (!channel_.read_new(&seq, &candidate, sizeof(candidate), timeout_ms)) return false;
        latest_ = candidate;
        seq_ = seq;
        return true;
    }
    const T &latest() const { return latest_; }

private:
    LatestChannel channel_;
    uint64_t seq_ = 0;
    T latest_{};
};

class CanQueue {
public:
    CanQueue() = default;
    ~CanQueue();

    bool open(const char *name, unsigned slot_count = kCanQueueSlots, bool create = true);
    void close();
    void reset();
    bool push(const CanBatch &batch);
    // 생길 수 없는 인덱스(쓰기 < 읽기, 칸 수를 넘는 차이)를 보면 읽기 위치를 쓰기 위치에 맞춰 큐를 비운다.
    bool pop(CanBatch *batch);
    // 비어 있으면 push가 깨울 때까지(최대 timeout_ns) 기다리며 꺼낸다. 소비자는 하나여야 한다.
    bool pop_wait(CanBatch *batch, uint64_t timeout_ns);
    uint64_t depth() const;
    bool valid() const { return header_ != nullptr; }

private:
    ShmRegion region_;
    CanQueueHeader *header_ = nullptr;
    CanBatch *slots_ = nullptr;
};

// 카메라 프레임 링. shm에는 헤더만 있고, 슬롯 픽셀은 생산자(camerad)가 잡은 CMM
// 블록에 있다(헤더에 물리 주소). 카메라·GDC·IVPS가 물리 주소로 직접 읽고 쓰며(CPU는 슬롯 매핑으로 읽는다), 각
// 슬롯의 seqlock(slot_seq, 쓰는 중이면 홀수)이 덮어쓰기를 알린다.
class FrameRing {
public:
    FrameRing() = default;
    ~FrameRing();

    // create면 헤더를 새로 초기화한다(생산자). 아니면 생산자가 만든 링에 붙는다. name은 시험이 바꾼다.
    bool open(bool create, unsigned width = kAiWidth, unsigned height = kAiHeight, unsigned slots = kFrameSlots,
              const char *name = kRoadAiFrameRing);
    void close();
    uint64_t slot_phys(unsigned index) const;
    void set_slot_phys(unsigned index, uint64_t phys);
    // 하드웨어가 슬롯에 직접 쓸 때: begin_write .. (쓰기) .. end_write.
    void begin_write(unsigned index);
    void end_write(unsigned index, uint64_t frame_id);
    // 하드웨어나 CPU가 슬롯을 읽을 때: read_begin이 안정된 seq를 주고, 다 읽은 뒤
    // read_still_valid로 그동안 덮어써지지 않았는지 본다.
    bool read_begin(unsigned index, uint64_t frame_id, uint64_t *seq) const;
    bool read_still_valid(unsigned index, uint64_t frame_id, uint64_t seq) const;

    unsigned slot_count() const { return header_ ? header_->slot_count : 0; }
    unsigned frame_bytes() const { return header_ ? header_->frame_bytes : 0; }
    unsigned width() const { return header_ ? header_->width : 0; }
    unsigned height() const { return header_ ? header_->height : 0; }
    bool valid() const { return header_ != nullptr; }

private:
    ShmRegion region_;
    FrameRingHeader *header_ = nullptr;
};
