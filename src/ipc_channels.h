#ifndef IPC_CHANNELS_H
#define IPC_CHANNELS_H

/* /dev/shm 채널 구현: 최신값 채널, CAN 큐, NV12 프레임 링. 레이아웃은
 * ipc_messages.h에 있다. */

#include "ipc_messages.h"

#include <cstddef>
#include <cstdint>
#include <string>

/* shm_open한 이름 하나와 그 매핑. 세 채널이 열기·크기 조정·매핑·닫기 순서를 공유하고,
 * 크기 정책(만들 때 늘릴지, 붙을 때 얼마를 요구할지)만 각자 정한다. */
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

class K230LatestChannel {
public:
    K230LatestChannel() = default;
    ~K230LatestChannel();

    bool open(const char *name, size_t payload_capacity, bool create);
    void close();
    bool publish(const void *payload, size_t payload_size);
    bool read(void *payload, size_t payload_capacity, uint64_t *seq = nullptr) const;
    bool read_new(uint64_t *last_seq, void *payload, size_t payload_capacity, int timeout_ms) const;
    bool valid() const { return header_ != nullptr; }

private:
    std::string name_;
    ShmRegion region_;
    K230IpcHeader *header_ = nullptr;
    uint8_t *payload_ = nullptr;
};

class K230CanQueue {
public:
    K230CanQueue() = default;
    ~K230CanQueue();

    bool open(const char *name, unsigned slot_count = kK230CanQueueSlots,
              bool create = true);
    void close();
    void reset();
    bool push(const K230CanBatch &batch);
    bool pop(K230CanBatch *batch);
    uint64_t depth() const;
    bool valid() const { return header_ != nullptr; }

private:
    std::string name_;
    ShmRegion region_;
    K230CanQueueHeader *header_ = nullptr;
    K230CanBatch *slots_ = nullptr;
};

/* 카메라 프레임 링. shm에는 헤더만 있고, 슬롯 픽셀은 생산자(camerad)가 잡은 CMM
 * 블록에 있다(헤더에 물리 주소). 카메라·GDC·IVPS가 물리 주소로 직접 읽고 쓰며, 각
 * 슬롯의 seqlock(slot_seq, 쓰는 중이면 홀수)이 덮어쓰기를 알린다. */
class K230FrameRing {
public:
    K230FrameRing() = default;
    ~K230FrameRing();

    // create면 헤더를 새로 초기화한다(생산자). 아니면 생산자가 만든 링에 붙는다.
    bool open(bool create, unsigned width = kK230AiWidth, unsigned height = kK230AiHeight,
              unsigned slots = kK230FrameSlots);
    void close();
    /* CPU로 읽을 때: attach_slot으로 붙인 슬롯 매핑에서 seqlock으로 복사한다. */
    bool copy_slot(unsigned index, uint64_t frame_id, uint8_t *destination,
                   size_t size) const;
    uint64_t slot_phys(unsigned index) const;
    void set_slot_phys(unsigned index, uint64_t phys);
    void attach_slot(unsigned index, uint8_t *virt);
    /* 하드웨어가 슬롯에 직접 쓸 때: begin_write .. (쓰기) .. end_write. */
    void begin_write(unsigned index);
    void end_write(unsigned index, uint64_t frame_id);
    /* 하드웨어가 슬롯을 직접 읽을 때: read_begin이 안정된 seq를 주고, 다 읽은 뒤
     * read_still_valid로 그동안 덮어써지지 않았는지 본다. */
    bool read_begin(unsigned index, uint64_t frame_id, uint64_t *seq) const;
    bool read_still_valid(unsigned index, uint64_t frame_id, uint64_t seq) const;

    unsigned slot_count() const { return header_ ? header_->slot_count : 0; }
    unsigned frame_bytes() const { return header_ ? header_->frame_bytes : 0; }
    unsigned width() const { return header_ ? header_->width : 0; }
    unsigned height() const { return header_ ? header_->height : 0; }
    bool valid() const { return header_ != nullptr; }

private:
    ShmRegion region_;
    K230FrameRingHeader *header_ = nullptr;
    uint8_t *slot_virt_[kK230FrameSlots] = {};
};

#endif
