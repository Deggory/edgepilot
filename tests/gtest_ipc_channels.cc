// CanQueue: 공유 메모리 CAN 링 큐. 가득 참, 순서, 생산자 재열기·reset, 신선도 판정, 생길 수 없는 인덱스 복구.
// futex 대기: 발행되면 깨고, 깨우지 않는 예전 빌드 생산자에는 폴링으로 내려갔다가 깨우는 생산자가 오면 돌아온다.
// Subscription: 헤더가 맞지 않는 채널에는 붙지 않는다.
#include "common/ipc_channels.h"
#include "common/utils_time.h"

#include <gtest/gtest.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

namespace {

// 테스트가 예전 빌드의 생산자처럼 머리를 직접 쓰려고 shm을 따로 매핑한다.
struct RawMap {
    void *data = MAP_FAILED;
    size_t size = 0;
    RawMap(const std::string &name, size_t bytes) : size(bytes) {
        const int fd = shm_open(name.c_str(), O_RDWR, 0);
        if (fd >= 0) {
            data = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            ::close(fd);
        }
    }
    ~RawMap() {
        if (data != MAP_FAILED) munmap(data, size);
    }
    bool ok() const { return data != MAP_FAILED; }
};

CanBatch batch_for(uint32_t value) {
    CanBatch batch;
    batch.timestamp_ns = value;
    batch.valid = 1;
    batch.count = 1;
    batch.frames[0].address = value;
    return batch;
}

// read_new는 다음 발행까지 기다리다(리눅스는 futex) 발행되면 곧바로 돌아오고, 발행이 없으면 시간 초과로 돌아온다.
TEST(LatestChannel, ReadNewWakesOnPublishAndTimesOut) {
    const std::string name = "/ep_latest_wait_" + std::to_string(getpid());
    struct Unlink {
        const std::string &name;
        ~Unlink() { shm_unlink(name.c_str()); }
    } cleanup{name};
    LatestChannel publisher, subscriber;
    ASSERT_TRUE(publisher.open(name.c_str(), sizeof(uint64_t), true));
    ASSERT_TRUE(subscriber.open(name.c_str(), sizeof(uint64_t), false));
    using Clock = std::chrono::steady_clock;
    uint64_t seq = 0, value = 0;
    const auto start = Clock::now();
    EXPECT_FALSE(subscriber.read_new(&seq, &value, sizeof(value), 100));
    const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
    EXPECT_GE(waited, 90);
    EXPECT_LT(waited, 600);

    // 발행에서 깨기까지를 잰다. 깨움이 없으면 대기 상한(100 ms)에야 돌아오므로 50 ms 늦는다.
    std::atomic<uint64_t> published_ns{0};
    std::thread writer([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const uint64_t v = 42;
        published_ns = monotonic_now_ns();
        publisher.publish(&v, sizeof(v));
    });
    EXPECT_TRUE(subscriber.read_new(&seq, &value, sizeof(value), 3000));
    const uint64_t woke_ns = monotonic_now_ns() - published_ns.load();
    writer.join();
    EXPECT_EQ(value, 42u);
    EXPECT_LT(woke_ns, 25'000'000ULL) << "발행하면 곧바로 깬다";
    EXPECT_FALSE(subscriber.read_new(&seq, &value, sizeof(value), 0)) << "같은 seq는 새 스냅샷이 아니다";
}

// pop_wait는 비어 있으면 시간 초과까지 기다리고, 그 사이 push가 오면 바로 꺼낸다(pandad 송신 스레드).
TEST(CanQueue, PopWaitWakesOnPushAndTimesOut) {
    const std::string name = "/ep_canq_wait_" + std::to_string(getpid());
    struct Unlink {
        const std::string &name;
        ~Unlink() { shm_unlink(name.c_str()); }
    } cleanup{name};
    CanQueue producer, consumer;
    ASSERT_TRUE(producer.open(name.c_str(), 4, true));
    producer.reset();
    ASSERT_TRUE(consumer.open(name.c_str(), 4, false));
    using Clock = std::chrono::steady_clock;
    const std::unique_ptr<CanBatch> got(new CanBatch);
    const auto start = Clock::now();
    EXPECT_FALSE(consumer.pop_wait(got.get(), 100'000'000ULL));
    const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
    EXPECT_GE(waited, 90);
    EXPECT_LT(waited, 600);

    // push에서 깨기까지를 잰다. 깨움이 없으면 대기 상한(100 ms)에야 돌아오므로 50 ms 늦는다.
    std::atomic<uint64_t> pushed_ns{0};
    std::thread writer([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const std::unique_ptr<CanBatch> sent(new CanBatch);
        sent->valid = 1;
        sent->count = 1;
        sent->frames[0].address = 0x340;
        pushed_ns = monotonic_now_ns();
        producer.push(*sent);
    });
    EXPECT_TRUE(consumer.pop_wait(got.get(), 3'000'000'000ULL));
    const uint64_t woke_ns = monotonic_now_ns() - pushed_ns.load();
    writer.join();
    EXPECT_EQ(got->frames[0].address, 0x340u);
    EXPECT_LT(woke_ns, 25'000'000ULL) << "push하면 곧바로 깬다";
}

// 깨우지 않는 예전 빌드의 발행자(seq만 바꾼다)라도 read_new는 대기 상한(100 ms) 안에 받고, 그 뒤로는 폴링으로 바로 받는다.
// 마지막 값만 남는 채널이라 발행자는 읽는 쪽이 다 받을 때까지 30 ms마다 계속 쓴다.
TEST(LatestChannel, WriterThatDoesNotWakeFallsBackToPolling) {
    const std::string name = "/ep_latest_old_" + std::to_string(getpid());
    struct Unlink {
        const std::string &name;
        ~Unlink() { shm_unlink(name.c_str()); }
    } cleanup{name};
    LatestChannel publisher, subscriber;
    ASSERT_TRUE(publisher.open(name.c_str(), sizeof(uint64_t), true));
    ASSERT_TRUE(subscriber.open(name.c_str(), sizeof(uint64_t), false));
    RawMap raw(name, sizeof(IpcHeader) + sizeof(uint64_t));
    ASSERT_TRUE(raw.ok());
    auto *header = static_cast<IpcHeader *>(raw.data);
    auto *payload = reinterpret_cast<uint8_t *>(header + 1);

    std::atomic<bool> stop{false};
    struct Joiner {
        std::atomic<bool> &stop;
        std::thread thread;
        ~Joiner() {
            stop = true;
            if (thread.joinable()) thread.join();
        }
    } writer{stop, std::thread([&] {
                 while (!stop) {
                     std::this_thread::sleep_for(std::chrono::milliseconds(30));
                     const uint64_t stamp = monotonic_now_ns();
                     const uint64_t seq = header->seq.load();
                     header->seq.store(seq + 1);
                     std::memcpy(payload, &stamp, sizeof(stamp));
                     header->payload_size.store(sizeof(stamp));
                     header->seq.store(seq + 2);  // wake는 그대로, FUTEX_WAKE도 없다
                 }
             })};
    uint64_t seq = 0, stamp = 0, worst_after_first = 0;
    for (int i = 0; i < 8; ++i) {
        const bool got = subscriber.read_new(&seq, &stamp, sizeof(stamp), 3000);
        EXPECT_TRUE(got) << i;
        if (!got) break;
        const uint64_t latency = monotonic_now_ns() - stamp;
        EXPECT_LT(latency, 300'000'000ULL) << "frame " << i << " waited past the 100 ms cap";
        if (i >= 2) worst_after_first = std::max(worst_after_first, latency);
    }
    EXPECT_LT(worst_after_first, 20'000'000ULL) << "after the first timeout the reader polls";

    // 깨우는 발행자로 바뀌면(새 빌드로 다시 뜸) 폴링을 그만두고 futex 대기로 돌아간다.
    stop = true;
    writer.thread.join();
    subscriber.read_new(&seq, &stamp, sizeof(stamp), 0);  // 멈추기 전에 쓴 값은 버린다
    testing::internal::CaptureStderr();
    std::thread waker([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        const uint64_t v = 7;
        publisher.publish(&v, sizeof(v));
    });
    EXPECT_TRUE(subscriber.read_new(&seq, &stamp, sizeof(stamp), 3000));
    waker.join();
    EXPECT_EQ(stamp, 7u);
    // 다음 대기가 깨움 단어가 움직인 것을 보고 futex로 돌아간다.
    EXPECT_FALSE(subscriber.read_new(&seq, &stamp, sizeof(stamp), 20));
    const std::string log = testing::internal::GetCapturedStderr();
#if defined(__linux__)
    EXPECT_NE(log.find("producer wakes again"), std::string::npos) << log;
#endif
}

// 깨우지 않는 예전 빌드의 생산자(write_seq만 올린다)라도 pop_wait는 상한 안에 받고 그 뒤로는 폴링으로 바로 받는다.
TEST(CanQueue, ProducerThatDoesNotWakeFallsBackToPolling) {
    const std::string name = "/ep_canq_old_" + std::to_string(getpid());
    struct Unlink {
        const std::string &name;
        ~Unlink() { shm_unlink(name.c_str()); }
    } cleanup{name};
    CanQueue owner, consumer;
    ASSERT_TRUE(owner.open(name.c_str(), 4, true));
    owner.reset();
    ASSERT_TRUE(consumer.open(name.c_str(), 4, false));
    RawMap raw(name, sizeof(CanQueueHeader) + 4 * sizeof(CanBatch));
    ASSERT_TRUE(raw.ok());
    auto *header = static_cast<CanQueueHeader *>(raw.data);
    auto *slots = reinterpret_cast<CanBatch *>(header + 1);

    constexpr int kBatches = 8;
    std::thread writer([&] {
        for (int i = 0; i < kBatches; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            const uint64_t w = header->write_seq.load();
            slots[w % 4].timestamp_ns = monotonic_now_ns();
            slots[w % 4].valid = 1;
            slots[w % 4].count = 0;
            header->write_seq.store(w + 1);  // wake는 그대로
        }
    });
    const std::unique_ptr<CanBatch> got(new CanBatch);
    uint64_t worst_after_first = 0;
    for (int i = 0; i < kBatches; ++i) {
        const bool popped = consumer.pop_wait(got.get(), 3'000'000'000ULL);
        EXPECT_TRUE(popped) << i;
        if (!popped) break;
        const uint64_t latency = monotonic_now_ns() - got->timestamp_ns;
        EXPECT_LT(latency, 300'000'000ULL) << "batch " << i << " waited past the 100 ms cap";
        if (i >= 2) worst_after_first = std::max(worst_after_first, latency);
    }
    writer.join();
    EXPECT_LT(worst_after_first, 20'000'000ULL) << "after the first timeout the consumer polls";
}

// 생길 수 없는 인덱스(읽기 > 쓰기, 칸 수보다 큰 차이)를 본 pop은 큐를 비워 생산자가 다시 넣을 수 있게 한다. 그대로 두면
// push가 계속 실패하고 pop_wait는 잠들지 못한다.
TEST(CanQueue, PopClearsImpossibleIndices) {
    const std::string name = "/ep_canq_heal_" + std::to_string(getpid());
    struct Unlink {
        const std::string &name;
        ~Unlink() { shm_unlink(name.c_str()); }
    } cleanup{name};
    CanQueue producer, consumer;
    ASSERT_TRUE(producer.open(name.c_str(), 4, true));
    producer.reset();
    ASSERT_TRUE(consumer.open(name.c_str(), 4, false));
    RawMap raw(name, sizeof(CanQueueHeader));
    ASSERT_TRUE(raw.ok());
    auto *header = static_cast<CanQueueHeader *>(raw.data);
    const std::unique_ptr<CanBatch> got(new CanBatch);

    header->read_seq.store(7);
    header->write_seq.store(3);
    EXPECT_FALSE(producer.push(batch_for(1))) << "an impossible queue looks full to the producer";
    EXPECT_FALSE(consumer.pop(got.get()));
    ASSERT_TRUE(producer.push(batch_for(2))) << "pop cleared the queue";
    ASSERT_TRUE(consumer.pop(got.get()));
    EXPECT_EQ(got->frames[0].address, 2u);

    header->write_seq.store(header->read_seq.load() + 10);
    EXPECT_FALSE(consumer.pop(got.get()));
    EXPECT_EQ(consumer.depth(), 0u);
    ASSERT_TRUE(producer.push(batch_for(3)));
    ASSERT_TRUE(consumer.pop(got.get()));
    EXPECT_EQ(got->frames[0].address, 3u);
}

// 큐는 머리와 채운 프레임만 옮긴다: 받는 배치의 앞 count개가 보낸 것과 같고, 넘치는 count는 자른다.
TEST(CanQueue, CopiesTheHeaderAndTheFilledFrames) {
    const std::string name = "/ep_canq_copy_" + std::to_string(getpid());
    struct Unlink {
        const std::string &name;
        ~Unlink() { shm_unlink(name.c_str()); }
    } cleanup{name};
    CanQueue producer, consumer;
    ASSERT_TRUE(producer.open(name.c_str(), 4, true));
    producer.reset();
    ASSERT_TRUE(consumer.open(name.c_str(), 4, false));

    CanBatch sent;
    sent.timestamp_ns = 77;
    sent.valid = 1;
    sent.dropped = 2;
    sent.count = 3;
    for (uint32_t i = 0; i < sent.count; ++i) {
        sent.frames[i].address = 0x100 + i;
        sent.frames[i].src = i;
        sent.frames[i].data_len = 8;
        sent.frames[i].data[7] = static_cast<uint8_t>(i + 1);
    }
    ASSERT_TRUE(producer.push(sent));
    CanBatch received;
    received.frames[5].address = 0xdead;  // count 뒤는 건드리지 않는다
    ASSERT_TRUE(consumer.pop(&received));
    EXPECT_EQ(received.timestamp_ns, 77u);
    EXPECT_EQ(received.valid, 1u);
    EXPECT_EQ(received.dropped, 2u);
    ASSERT_EQ(received.count, 3u);
    EXPECT_EQ(std::memcmp(received.frames, sent.frames, 3 * sizeof(IpcCanFrame)), 0);
    EXPECT_EQ(received.frames[5].address, 0xdeadu);

    sent.count = kCanBatchMaxFrames + 10;  // 깨진 count
    ASSERT_TRUE(producer.push(sent));
    ASSERT_TRUE(consumer.pop(&received));
    EXPECT_EQ(received.count, kCanBatchMaxFrames);
}

TEST(CanQueue, SharedMemoryQueue) {
    const std::string name = "/edgepilot_can_queue_test_" + std::to_string(getpid());
    // ASSERT가 중간에 빠져나가도 공유 메모리를 지운다.
    struct Unlink {
        const std::string &name;
        ~Unlink() { shm_unlink(name.c_str()); }
    } cleanup{name};
    CanQueue producer;
    CanQueue consumer;
    ASSERT_TRUE(producer.open(name.c_str(), 8, true)) << "생산자 열기";
    producer.reset();
    ASSERT_TRUE(consumer.open(name.c_str(), 8, false)) << "소비자 열기";

    for (uint32_t i = 0; i < 8; ++i) {
        ASSERT_TRUE(producer.push(batch_for(i))) << "큐 채우기";
    }
    ASSERT_EQ(producer.depth(), 8) << "가득 찬 큐의 깊이";
    ASSERT_FALSE(producer.push(batch_for(8))) << "가득 찬 큐는 덮어쓰지 않는다";

    for (uint32_t i = 0; i < 8; ++i) {
        CanBatch batch;
        ASSERT_TRUE(consumer.pop(&batch)) << "큐 비우기";
        // 넣은 순서대로 나온다
        ASSERT_TRUE(batch.valid);
        ASSERT_EQ(batch.timestamp_ns, i);
        ASSERT_EQ(batch.frames[0].address, i);
    }
    ASSERT_FALSE(consumer.pop(nullptr)) << "null로 꺼내기";
    CanBatch empty;
    ASSERT_FALSE(consumer.pop(&empty)) << "빈 큐";

    ASSERT_TRUE(producer.push(batch_for(41))) << "생산자를 다시 열기 전에 넣기";
    producer.close();
    ASSERT_TRUE(producer.open(name.c_str(), 8, true)) << "생산자 다시 열기";
    CanBatch after_reopen;
    ASSERT_TRUE(consumer.pop(&after_reopen)) << "생산자를 다시 열어도 큐가 남는다";
    ASSERT_EQ(after_reopen.frames[0].address, 41) << "다시 연 생산자는 쌓인 데이터를 보존한다";

    ASSERT_TRUE(producer.push(batch_for(42))) << "생산자 reset 전에 넣기";
    producer.reset();
    ASSERT_FALSE(consumer.pop(&empty)) << "생산자 reset은 이전 세대를 버린다";

    for (uint32_t i = 0; i < 10000; ++i) {
        ASSERT_TRUE(producer.push(batch_for(i))) << "번갈아 넣기";
        CanBatch batch;
        ASSERT_TRUE(consumer.pop(&batch)) << "번갈아 꺼내기";
        ASSERT_EQ(batch.timestamp_ns, i) << "번갈아 넣고 꺼내도 순서가 유지된다";
    }
    ASSERT_EQ(producer.depth(), 0) << "빈 큐의 깊이";
    CanBatch fresh = batch_for(1);
    fresh.timestamp_ns = 1000;
    ASSERT_TRUE(can_batch_is_fresh(fresh, 1050, 100)) << "신선한 묶음";
    ASSERT_FALSE(can_batch_is_fresh(fresh, 1101, 100)) << "낡은 묶음";
    ASSERT_FALSE(can_batch_is_fresh(fresh, 999, 100)) << "미래 시각의 묶음";

    consumer.close();
    producer.close();
}

TEST(Subscription, StaleHeaderIsNotAttached) {
    struct Payload {
        uint64_t value;
    };
    const std::string name = "/edgepilot_latest_test_" + std::to_string(getpid() % 100000);
    struct Unlink {
        const std::string &name;
        ~Unlink() { shm_unlink(name.c_str()); }
    } cleanup{name};
    LatestChannel producer;
    ASSERT_TRUE(producer.open(name.c_str(), sizeof(Payload), true)) << "생산자 열기";
    const Payload stale{7};
    ASSERT_TRUE(producer.publish(&stale, sizeof(stale)));

    // 다른 빌드가 남긴 채널: 헤더 버전이 다르다
    ShmRegion raw;
    ASSERT_TRUE(raw.open(name.c_str(), false));
    ASSERT_TRUE(raw.map(sizeof(IpcHeader)));
    IpcHeader *header = static_cast<IpcHeader *>(raw.data());
    header->version = kIpcVersion + 1;

    Subscription<Payload> subscription;
    ASSERT_FALSE(subscription.attach(name.c_str())) << "버전이 다른 채널에 붙기";
    ASSERT_FALSE(subscription.attach(name.c_str())) << "다시 시도해도 붙지 않는다";
    ASSERT_FALSE(subscription.poll()) << "붙지 않은 채널은 읽지 않는다";

    // 생산자가 채널을 다시 초기화하면 붙는다
    header->version = kIpcVersion;
    const Payload fresh{9};
    ASSERT_TRUE(producer.publish(&fresh, sizeof(fresh)));
    ASSERT_TRUE(subscription.attach(name.c_str())) << "초기화된 채널에 붙기";
    ASSERT_TRUE(subscription.poll()) << "새 스냅샷";
    ASSERT_EQ(subscription.latest().value, 9u);
}

}  // namespace
