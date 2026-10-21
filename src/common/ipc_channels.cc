#include "common/ipc_channels.h"
#include "common/utils_time.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#if defined(__linux__)
#include <linux/futex.h>
#include <sys/syscall.h>
#endif

#include <algorithm>
#include <climits>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace {

// futex 대기 한 번의 상한. 끝없이 기다리는 호출도 이만큼마다 깨어 다시 본다.
constexpr uint64_t kMaxFutexWaitNs = 100'000'000ULL;

// 기다리는 쪽을 깨운다. 기다리는 이가 없어도 FUTEX_WAKE를 부른다(syscall 하나, 약 1 µs).
void wake_all(std::atomic<uint32_t> *word) {
    word->fetch_add(1);
#if defined(__linux__)
    syscall(SYS_futex, reinterpret_cast<uint32_t *>(word), FUTEX_WAKE, INT_MAX, nullptr, nullptr, 0);
#endif
}

// still_waiting()이 참이면 word가 바뀌거나 timeout_ns(상한 100 ms)가 지날 때까지 잔다. 깨움 단어를 조건보다 먼저
// 읽으므로 그 사이의 발행은 FUTEX_WAIT가 바로 돌아오게 한다. 가짜로 깰 수 있어 호출자가 다시 본다. 리눅스 밖(호스트
// 시험)은 1 ms 잔다.
template <class StillWaiting>
void wait_wake(std::atomic<uint32_t> *word, uint64_t timeout_ns, StillWaiting still_waiting) {
#if defined(__linux__)
    const uint32_t before = word->load();
    if (!still_waiting()) return;
    const uint64_t ns = std::min(timeout_ns, kMaxFutexWaitNs);
    timespec relative{static_cast<time_t>(ns / 1'000'000'000ULL), static_cast<long>(ns % 1'000'000'000ULL)};
    // 공유 메모리라 FUTEX_PRIVATE_FLAG는 쓰지 않는다. 깨움·시간 초과·신호 어느 것이든 돌아가 호출자가 다시 본다.
    syscall(SYS_futex, reinterpret_cast<uint32_t *>(word), FUTEX_WAIT, before, &relative, nullptr, 0);
#else
    (void)word;
    (void)timeout_ns;
    (void)still_waiting;
    usleep(1000);
#endif
}

}  // namespace

// ---- ShmRegion ----

bool ShmRegion::open(const char *name, bool create) {
    close();
    fd_ = shm_open(name, O_RDWR | (create ? O_CREAT : 0), 0664);
    return fd_ >= 0;
}

bool ShmRegion::file_size(size_t *size) const {
    struct stat st{};
    if (fd_ < 0 || fstat(fd_, &st) != 0) return false;
    *size = static_cast<size_t>(st.st_size);
    return true;
}

bool ShmRegion::resize(size_t size) { return fd_ >= 0 && ftruncate(fd_, static_cast<off_t>(size)) == 0; }

bool ShmRegion::map(size_t size) {
    if (fd_ < 0 || map_) return false;
    void *mapped = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapped == MAP_FAILED) return false;
    map_ = mapped;
    size_ = size;
    return true;
}

void ShmRegion::close() {
    if (map_) {
        munmap(map_, size_);
        map_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    size_ = 0;
}

// ---- FrameRing ----

FrameRing::~FrameRing() { close(); }

bool FrameRing::open(bool create, unsigned width, unsigned height, unsigned slots, const char *name) {
    close();
    if (!region_.open(name, create)) return false;
    if (create && !region_.resize(sizeof(FrameRingHeader))) {
        std::perror("ipc: ftruncate frame ring");
        close();
        return false;
    }
    size_t size = 0;
    if (!region_.file_size(&size) || size < sizeof(FrameRingHeader) || !region_.map(sizeof(FrameRingHeader))) {
        close();
        return false;
    }

    header_ = static_cast<FrameRingHeader *>(region_.data());
    if (create) {
        // 이전 실행의 seq·frame_id·물리 주소가 남아 있으면 소비자가 없는 슬롯을
        // 유효하다고 읽으므로, 만들 때마다 전부 초기화한다. 물리 주소는 생산자가
        // 슬롯 CMM을 잡은 뒤 set_slot_phys로 채운다.
        header_->magic = kFrameRingMagic;
        header_->version = kFrameRingVersion;
        header_->slot_count = slots;
        header_->width = width;
        header_->height = height;
        header_->frame_bytes = static_cast<uint32_t>(width * height * 3 / 2);
        header_->reserved0 = 0;
        header_->reserved1 = 0;
        for (unsigned index = 0; index < kFrameSlots; ++index) {
            header_->slot_seq[index].store(0, std::memory_order_relaxed);
            header_->slot_frame_id[index].store(UINT64_MAX, std::memory_order_relaxed);
            header_->slot_phys[index] = 0;
        }
        std::atomic_thread_fence(std::memory_order_release);
    }
    const bool valid = header_->magic == kFrameRingMagic && header_->version == kFrameRingVersion &&
                       header_->slot_count > 0 && header_->slot_count <= kFrameSlots && header_->frame_bytes > 0;
    if (!valid) close();
    return valid;
}

void FrameRing::close() {
    region_.close();
    header_ = nullptr;
}

uint64_t FrameRing::slot_phys(unsigned index) const {
    return header_ && index < kFrameSlots ? header_->slot_phys[index] : 0;
}

void FrameRing::set_slot_phys(unsigned index, uint64_t phys) {
    if (header_ && index < kFrameSlots) header_->slot_phys[index] = phys;
}

void FrameRing::begin_write(unsigned index) {
    std::atomic<uint64_t> &sequence = header_->slot_seq[index];
    uint64_t next = sequence.load(std::memory_order_relaxed);
    if (next & 1ULL) ++next;
    sequence.store(next + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_seq_cst);
}

void FrameRing::end_write(unsigned index, uint64_t frame_id) {
    std::atomic<uint64_t> &sequence = header_->slot_seq[index];
    header_->slot_frame_id[index].store(frame_id, std::memory_order_release);
    sequence.store(sequence.load(std::memory_order_relaxed) + 1, std::memory_order_release);
}

bool FrameRing::read_begin(unsigned index, uint64_t frame_id, uint64_t *seq) const {
    if (!header_ || index >= header_->slot_count) return false;
    const uint64_t before = header_->slot_seq[index].load(std::memory_order_acquire);
    if (before == 0 || (before & 1ULL) != 0) return false;
    if (header_->slot_frame_id[index].load(std::memory_order_acquire) != frame_id) return false;
    *seq = before;
    return true;
}

bool FrameRing::read_still_valid(unsigned index, uint64_t frame_id, uint64_t seq) const {
    // CPU가 슬롯 매핑에서 읽은 것(앞차 광류)이 아래 seq 확인보다 늦게 보이지 않게 한다(ARM은 약한 순서)
    std::atomic_thread_fence(std::memory_order_acquire);
    return header_->slot_seq[index].load(std::memory_order_acquire) == seq &&
           header_->slot_frame_id[index].load(std::memory_order_acquire) == frame_id;
}

LatestChannel::~LatestChannel() { close(); }

bool LatestChannel::open(const char *name, size_t payload_capacity, bool create) {
    close();
    if (!region_.open(name, create)) return false;

    const size_t map_size = sizeof(IpcHeader) + payload_capacity;
    if (create) {
        if (!region_.resize(map_size)) {
            std::perror("ipc: ftruncate ipc channel");
            close();
            return false;
        }
    } else {
        // 생산자가 O_CREAT 직후 ftruncate 전이면 shm이 요청보다 작을 수 있다.
        // 그대로 mmap하면 header를 읽는 순간 SIGBUS가 난다.
        size_t actual = 0;
        if (!region_.file_size(&actual) || actual < map_size) {
            std::fprintf(stderr, "ipc: ipc channel size mismatch name=%s actual=%zu expected=%zu\n", name, actual,
                         map_size);
            close();
            return false;
        }
    }
    if (!region_.map(map_size)) {
        std::perror("ipc: mmap ipc channel");
        close();
        return false;
    }

    header_ = static_cast<IpcHeader *>(region_.data());
    payload_ = reinterpret_cast<uint8_t *>(header_) + sizeof(IpcHeader);
    if (create && (header_->magic != kIpcMagic || header_->version != kIpcVersion ||
                   header_->payload_capacity != payload_capacity)) {
        header_->magic = kIpcMagic;
        header_->version = kIpcVersion;
        header_->payload_capacity = static_cast<uint32_t>(payload_capacity);
        header_->wake.store(0);
        header_->seq.store(0, std::memory_order_release);
        header_->timestamp_ns.store(0, std::memory_order_release);
        header_->payload_size.store(0, std::memory_order_release);
        std::memset(payload_, 0, payload_capacity);
    }
    // 다른 빌드의 레이아웃이 남은 채널은 매핑도 풀어 valid()가 거짓이게 한다(CanQueue·FrameRing과 같다).
    const bool valid =
        header_->magic == kIpcMagic && header_->version == kIpcVersion && header_->payload_capacity >= payload_capacity;
    if (!valid) close();
    return valid;
}

void LatestChannel::close() {
    region_.close();
    header_ = nullptr;
    payload_ = nullptr;
}

bool LatestChannel::publish(const void *payload, size_t payload_size) {
    if (!header_ || payload_size > header_->payload_capacity) return false;

    uint64_t seq = header_->seq.load(std::memory_order_relaxed);
    if ((seq & 1ULL) != 0) ++seq;
    header_->seq.store(seq + 1, std::memory_order_relaxed);
    // 홀수 seq가 아래 페이로드 쓰기보다 먼저 보이게 한다. release 저장만으로는 뒤의 쓰기가 앞당겨질 수 있다(ARM은
    // 약한 순서). FrameRing::begin_write도 같은 이유로 울타리를 둔다(그쪽은 seq_cst).
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(payload_, payload, payload_size);
    header_->payload_size.store(static_cast<uint32_t>(payload_size), std::memory_order_release);
    header_->timestamp_ns.store(monotonic_now_ns(), std::memory_order_release);
    header_->seq.store(seq + 2, std::memory_order_release);
    // 새 seq를 쓴 뒤 깨운다. 기다리는 쪽은 깨움 단어를 읽은 뒤 seq를 다시 보고 잠들므로 이 순서면 깨움을 놓치지 않는다.
    wake_all(&header_->wake);
    return true;
}

void LatestChannel::wait_publish(uint64_t seen_seq, uint64_t timeout_ns) const {
    // 발행이 진행 중(홀수 seq)이어도 기다린다. 다 쓴 뒤 깨운다.
    wait_wake(&header_->wake, timeout_ns, [&] {
        const uint64_t seq = header_->seq.load();
        return seq == seen_seq || (seq & 1ULL) != 0;
    });
}

bool LatestChannel::read(void *payload, size_t payload_capacity, uint64_t *seq) const {
    if (!header_) return false;
    for (int attempt = 0; attempt < 4; ++attempt) {
        const uint64_t before = header_->seq.load(std::memory_order_acquire);
        if (before == 0 || (before & 1ULL) != 0) return false;
        const uint32_t payload_size = header_->payload_size.load(std::memory_order_acquire);
        if (payload_size == 0 || payload_size > payload_capacity) return false;
        std::memcpy(payload, payload_, payload_size);
        // 복사한 읽기가 뒤의 seq 확인보다 늦게 끝나지 않게 한다(FrameRing::read_still_valid와 같다).
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint64_t after = header_->seq.load(std::memory_order_relaxed);
        if (before == after && (after & 1ULL) == 0) {
            if (seq) *seq = after;
            return true;
        }
    }
    return false;
}

bool LatestChannel::read_new(uint64_t *last_seq, void *payload, size_t payload_capacity, int timeout_ms) const {
    if (!header_) return false;
    const uint64_t timeout_ns = timeout_ms < 0 ? UINT64_MAX : static_cast<uint64_t>(timeout_ms) * 1'000'000ULL;
    uint64_t start = 0;  // 기다려야 할 때만 시계를 읽는다(CLOCK_BOOTTIME은 이 보드에서 진짜 syscall이다)
    while (true) {
        // seq가 그대로면 페이로드를 복사하지 않는다(폴링하는 쪽이 대부분 이 경우다)
        const uint64_t current = header_->seq.load(std::memory_order_acquire);
        uint64_t seq = 0;
        if (current != *last_seq && read(payload, payload_capacity, &seq) && seq != *last_seq) {
            *last_seq = seq;
            return true;
        }
        if (timeout_ms == 0) return false;
        const uint64_t now = monotonic_now_ns();
        if (start == 0) start = now;
        const uint64_t elapsed = now - start;
        if (timeout_ms > 0 && elapsed >= timeout_ns) return false;
        wait_publish(current, timeout_ms > 0 ? timeout_ns - elapsed : kMaxFutexWaitNs);
    }
}

namespace {

size_t queue_map_size(unsigned slot_count) {
    return sizeof(CanQueueHeader) + static_cast<size_t>(slot_count) * sizeof(CanBatch);
}

}  // namespace

CanQueue::~CanQueue() { close(); }

bool CanQueue::open(const char *name, unsigned slot_count, bool create) {
    close();
    if (!region_.open(name, create)) {
        std::perror("ipc: shm_open CAN queue");
        return false;
    }

    // 만들 때는 작으면 늘리고(기존 큐는 유지), 붙을 때는 요청 크기 이상을 요구한다.
    const size_t map_size = queue_map_size(slot_count);
    size_t actual = 0;
    if (!region_.file_size(&actual)) {
        std::perror("ipc: fstat CAN queue");
        close();
        return false;
    }
    if (create) {
        if (actual < map_size && !region_.resize(map_size)) {
            std::fprintf(stderr, "ipc: CAN queue resize failed name=%s actual=%zu expected=%zu\n", name, actual,
                         map_size);
            std::perror("ipc: ftruncate CAN queue");
            close();
            return false;
        }
    } else if (actual < map_size) {
        std::fprintf(stderr, "ipc: CAN queue size mismatch name=%s actual=%zu expected=%zu\n", name, actual, map_size);
        close();
        return false;
    }
    if (!region_.map(map_size)) {
        std::perror("ipc: mmap CAN queue");
        close();
        return false;
    }

    void *map = region_.data();
    header_ = static_cast<CanQueueHeader *>(map);
    slots_ = reinterpret_cast<CanBatch *>(reinterpret_cast<uint8_t *>(map) + sizeof(CanQueueHeader));
    if (create && (header_->magic != kCanQueueMagic || header_->version != kCanQueueVersion ||
                   header_->slot_count != slot_count)) {
        std::memset(map, 0, region_.size());
        header_->magic = kCanQueueMagic;
        header_->version = kCanQueueVersion;
        header_->slot_count = slot_count;
        header_->write_seq.store(0, std::memory_order_release);
        header_->read_seq.store(0, std::memory_order_release);
    }

    if (header_->magic != kCanQueueMagic || header_->version != kCanQueueVersion || header_->slot_count != slot_count) {
        std::fprintf(stderr, "ipc: CAN queue header mismatch name=%s magic=0x%x version=%u slots=%u\n", name,
                     header_->magic, header_->version, header_->slot_count);
        close();
        return false;
    }
    return true;
}

void CanQueue::close() {
    region_.close();
    header_ = nullptr;
    slots_ = nullptr;
}

void CanQueue::reset() {
    if (!header_) return;
    header_->read_seq.store(0, std::memory_order_release);
    header_->write_seq.store(0, std::memory_order_release);
    for (unsigned i = 0; i < header_->slot_count; ++i) {
        slots_[i] = CanBatch{};
    }
}

namespace {

// 머리와 채운 프레임만 옮긴다. 배치 하나가 21.5 KB(프레임 256개)라 통째로 복사하면 10 ms마다 몇십 개를 위해
// 다 옮기게 된다. 받는 쪽은 count까지만 읽는다(넘는 count는 자른다).
void copy_can_batch(CanBatch *to, const CanBatch &from) {
    const uint32_t count = std::min<uint32_t>(from.count, kCanBatchMaxFrames);
    to->timestamp_ns = from.timestamp_ns;
    to->valid = from.valid;
    to->count = count;
    to->dropped = from.dropped;
    to->reserved = from.reserved;
    std::memcpy(to->frames, from.frames, count * sizeof(IpcCanFrame));
}

}  // namespace

bool CanQueue::push(const CanBatch &batch) {
    if (!header_) return false;
    const uint64_t write_seq = header_->write_seq.load(std::memory_order_relaxed);
    const uint64_t read_seq = header_->read_seq.load(std::memory_order_acquire);
    if (write_seq - read_seq >= header_->slot_count) return false;

    copy_can_batch(&slots_[write_seq % header_->slot_count], batch);
    header_->write_seq.store(write_seq + 1, std::memory_order_release);
    wake_all(&header_->wake);  // 기다리는 소비자(pandad 송신 스레드)를 깨운다
    return true;
}

bool CanQueue::pop(CanBatch *batch) {
    if (!header_) return false;
    const uint64_t read_seq = header_->read_seq.load(std::memory_order_relaxed);
    const uint64_t write_seq = header_->write_seq.load(std::memory_order_acquire);
    if (read_seq == write_seq) return false;
    if (write_seq < read_seq || write_seq - read_seq > header_->slot_count) {
        // 정상이면 생길 수 없는 인덱스(생산자 reset과 꺼내기가 겹친 경우 등). 소비자 몫인 읽기 위치를 쓰기 위치에 맞춰
        // 큐를 비운다. 그대로 두면 push도 계속 가득 찼다고 실패한다.
        header_->read_seq.store(write_seq, std::memory_order_release);
        return false;
    }

    copy_can_batch(batch, slots_[read_seq % header_->slot_count]);
    header_->read_seq.store(read_seq + 1, std::memory_order_release);
    return true;
}

bool CanQueue::pop_wait(CanBatch *batch, uint64_t timeout_ns) {
    if (pop(batch)) return true;
    if (!header_) return false;
    const uint64_t start = monotonic_now_ns();
    while (true) {
        const uint64_t elapsed = monotonic_now_ns() - start;
        if (elapsed >= timeout_ns) return false;
        wait_wake(&header_->wake, timeout_ns - elapsed,
                  [&] { return header_->write_seq.load() == header_->read_seq.load(); });
        if (pop(batch)) return true;
    }
}

uint64_t CanQueue::depth() const {
    if (!header_) return 0;
    const uint64_t write_seq = header_->write_seq.load(std::memory_order_acquire);
    const uint64_t read_seq = header_->read_seq.load(std::memory_order_acquire);
    if (write_seq < read_seq) return 0;
    return std::min<uint64_t>(write_seq - read_seq, header_->slot_count);
}
