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
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace {

// futex 대기 한 번의 상한. 깨우지 않는 생산자라도 이 안에 데이터를 보고, 이 시간 초과로 그런 생산자를 알아챈다.
constexpr uint64_t kMaxFutexWaitNs = 100000000ULL;

// 기다리는 쪽을 깨운다. waiters 셈은 보지 않고 늘 FUTEX_WAKE를 부른다: 다른 빌드가 머리를 다시 초기화(memset)하면
// 기다리는 중의 셈이 어긋나(0 밑으로 감겨) 깨움을 영영 빠뜨릴 수 있다. 기다리는 이가 없으면 syscall 하나(약 1 µs)다.
void wake_all(std::atomic<uint32_t> *word) {
    word->fetch_add(1);
#if defined(__linux__)
    syscall(SYS_futex, reinterpret_cast<uint32_t *>(word), FUTEX_WAKE, INT_MAX, nullptr, nullptr, 0);
#endif
}

}  // namespace

template <class StillWaiting>
void WakeWaiter::wait(std::atomic<uint32_t> *word, std::atomic<uint32_t> *waiters, uint64_t timeout_ns,
                      StillWaiting still_waiting, const std::string &name) {
#if defined(__linux__)
    if (polling_) {
        if (word->load() == polled_word_) {
            usleep(1000);
            return;
        }
        polling_ = false;  // 생산자가 다시 깨운다(새 빌드로 다시 떴다)
        std::fprintf(stderr, "ipc: %s producer wakes again, waiting on futex\n", name.c_str());
    }
    // waiters는 이 방식 전 빌드의 생산자(waiters가 0이면 깨우지 않았다)를 위해 계속 센다.
    waiters->fetch_add(1);
    const uint32_t before = word->load();
    bool timed_out = false;
    if (still_waiting()) {
        const uint64_t ns = std::min(timeout_ns, kMaxFutexWaitNs);
        timespec relative{static_cast<time_t>(ns / 1000000000ULL), static_cast<long>(ns % 1000000000ULL)};
        // 공유 메모리라 FUTEX_PRIVATE_FLAG는 쓰지 않는다. 깨움·시간 초과·신호 어느 것이든 돌아가 호출자가 다시 본다.
        timed_out =
            syscall(SYS_futex, reinterpret_cast<uint32_t *>(word), FUTEX_WAIT, before, &relative, nullptr, 0) != 0 &&
            errno == ETIMEDOUT;
    }
    waiters->fetch_sub(1);
    // 시간 초과로 깼는데 그사이 데이터가 왔고 깨움 단어가 그대로면 생산자가 깨우지 않는 빌드다. 막 발행하던(데이터를 쓰고
    // 깨우기 직전인) 새 생산자를 잘못 보지 않게 1 ms 뒤 한 번 더 본다.
    if (timed_out && !still_waiting() && word->load() == before) {
        usleep(1000);
        if (word->load() == before) {
            polling_ = true;
            polled_word_ = before;
            std::fprintf(stderr, "ipc: %s producer does not wake (older build?), polling every 1 ms\n", name.c_str());
        }
    }
#else
    (void)word;
    (void)waiters;
    (void)timeout_ns;
    (void)still_waiting;
    (void)name;
    (void)polling_;
    (void)polled_word_;
    usleep(1000);
#endif
}

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

bool FrameRing::open(bool create, unsigned width, unsigned height, unsigned slots) {
    close();
    if (width == 0 || height == 0 || slots == 0 || slots > kFrameSlots) return false;

    if (!region_.open(kRoadAiFrameRing, create)) return false;
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
    for (auto &virt : slot_virt_) virt = nullptr;
}

uint64_t FrameRing::slot_phys(unsigned index) const {
    return header_ && index < kFrameSlots ? header_->slot_phys[index] : 0;
}

void FrameRing::set_slot_phys(unsigned index, uint64_t phys) {
    if (header_ && index < kFrameSlots) header_->slot_phys[index] = phys;
}

void FrameRing::attach_slot(unsigned index, uint8_t *virt) {
    if (index < kFrameSlots) slot_virt_[index] = virt;
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
    return header_->slot_seq[index].load(std::memory_order_acquire) == seq &&
           header_->slot_frame_id[index].load(std::memory_order_acquire) == frame_id;
}

namespace {

// seqlock 재시도 껍데기. copy는 슬롯이 안정적일 때만 호출되고, 복사 도중
// 생산자가 슬롯을 덮었으면 결과를 버리고 다시 시도한다.
template <typename Copy>
bool copy_slot_guarded(const FrameRingHeader &header, unsigned index, uint64_t frame_id, const uint8_t *source,
                       Copy copy) {
    constexpr unsigned kCopyAttempts = 8;
    const std::atomic<uint64_t> &sequence = header.slot_seq[index];
    const std::atomic<uint64_t> &stored_frame_id = header.slot_frame_id[index];
    for (unsigned attempt = 0; attempt < kCopyAttempts; ++attempt) {
        const uint64_t before = sequence.load(std::memory_order_acquire);
        if (before == 0 || (before & 1ULL) != 0) continue;
        const uint64_t before_frame_id = stored_frame_id.load(std::memory_order_acquire);
        if (before_frame_id != frame_id) {
            if (before_frame_id > frame_id) return false;
            continue;
        }

        copy(source);

        // 복사한 읽기가 뒤의 seq 확인보다 늦게 보이지 않게 한다(ARM은 약한 순서).
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint64_t after = sequence.load(std::memory_order_acquire);
        const uint64_t after_frame_id = stored_frame_id.load(std::memory_order_acquire);
        if (before == after && (after & 1ULL) == 0 && after_frame_id == frame_id) return true;
    }
    return false;
}

}  // namespace

bool FrameRing::copy_slot(unsigned index, uint64_t frame_id, uint8_t *destination, size_t size) const {
    if (!header_ || !destination || index >= header_->slot_count || size != header_->frame_bytes) return false;

    const uint8_t *source = slot_virt_[index];
    if (!source) return false;
    return copy_slot_guarded(*header_, index, frame_id, source,
                             [&](const uint8_t *from) { std::memcpy(destination, from, size); });
}

LatestChannel::~LatestChannel() { close(); }

bool LatestChannel::open(const char *name, size_t payload_capacity, bool create) {
    close();
    name_ = name ? name : "";
    if (!region_.open(name_.c_str(), create)) return false;

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
            std::fprintf(stderr, "ipc: ipc channel size mismatch name=%s actual=%zu expected=%zu\n", name_.c_str(),
                         actual, map_size);
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
        header_->waiters.store(0);
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
    if (!header_ || !payload || payload_size > header_->payload_capacity) return false;

    uint64_t seq = header_->seq.load(std::memory_order_relaxed);
    if ((seq & 1ULL) != 0) ++seq;
    header_->seq.store(seq + 1, std::memory_order_relaxed);
    // 홀수 seq가 아래 페이로드 쓰기보다 먼저 보이게 한다. release 저장만으로는 뒤의 쓰기가 앞당겨질 수 있다(ARM은
    // 약한 순서). FrameRing::begin_write와 같다.
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
    if (!header_) {
        usleep(1000);
        return;
    }
    // 발행이 진행 중(홀수 seq)이어도 기다린다. 다 쓴 뒤 깨우므로, 그 사이의 시간 초과를 깨우지 않는 생산자로 보지 않는다.
    waiter_.wait(
        &header_->wake, &header_->waiters, timeout_ns,
        [&] {
            const uint64_t seq = header_->seq.load();
            return seq == seen_seq || (seq & 1ULL) != 0;
        },
        name_);
}

bool LatestChannel::read(void *payload, size_t payload_capacity, uint64_t *seq) const {
    if (!header_ || !payload) return false;
    for (int attempt = 0; attempt < 4; ++attempt) {
        const uint64_t before = header_->seq.load(std::memory_order_acquire);
        if (before == 0 || (before & 1ULL) != 0) return false;
        const uint32_t payload_size = header_->payload_size.load(std::memory_order_acquire);
        if (payload_size == 0 || payload_size > payload_capacity) return false;
        std::memcpy(payload, payload_, payload_size);
        // 복사한 읽기가 뒤의 seq 확인보다 늦게 끝나지 않게 한다(FrameRing::read와 같다).
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
    const uint64_t timeout_ns = timeout_ms < 0 ? UINT64_MAX : static_cast<uint64_t>(timeout_ms) * 1000000ULL;
    uint64_t start = 0;  // 기다려야 할 때만 시계를 읽는다(CLOCK_BOOTTIME은 이 보드에서 진짜 syscall이다)
    while (true) {
        // seq가 그대로면 페이로드를 복사하지 않는다(폴링하는 쪽이 대부분 이 경우다)
        const uint64_t current = header_ ? header_->seq.load(std::memory_order_acquire) : 0;
        const bool unchanged = header_ && last_seq && current == *last_seq;
        uint64_t seq = 0;
        if (!unchanged && read(payload, payload_capacity, &seq) && (!last_seq || seq != *last_seq)) {
            if (last_seq) *last_seq = seq;
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
    if (!name || name[0] == '\0' || slot_count == 0) return false;

    name_ = name;
    if (!region_.open(name_.c_str(), create)) {
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
            std::fprintf(stderr, "ipc: CAN queue resize failed name=%s actual=%zu expected=%zu\n", name_.c_str(),
                         actual, map_size);
            std::perror("ipc: ftruncate CAN queue");
            close();
            return false;
        }
    } else if (actual < map_size) {
        std::fprintf(stderr, "ipc: CAN queue size mismatch name=%s actual=%zu expected=%zu\n", name_.c_str(), actual,
                     map_size);
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
        std::fprintf(stderr, "ipc: CAN queue header mismatch name=%s magic=0x%x version=%u slots=%u\n", name_.c_str(),
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
    if (!header_ || !slots_) return false;
    const uint64_t write_seq = header_->write_seq.load(std::memory_order_relaxed);
    const uint64_t read_seq = header_->read_seq.load(std::memory_order_acquire);
    if (write_seq - read_seq >= header_->slot_count) return false;

    copy_can_batch(&slots_[write_seq % header_->slot_count], batch);
    header_->write_seq.store(write_seq + 1, std::memory_order_release);
    wake_all(&header_->wake);  // 기다리는 소비자(pandad 송신 스레드)를 깨운다
    return true;
}

bool CanQueue::pop(CanBatch *batch) {
    if (!header_ || !slots_ || !batch) return false;
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
        waiter_.wait(
            &header_->wake, &header_->waiters, timeout_ns - elapsed,
            [&] { return header_->write_seq.load() == header_->read_seq.load(); }, name_);
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
