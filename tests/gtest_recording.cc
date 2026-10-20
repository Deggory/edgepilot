// RecordingWriter가 디스크에 남기는 것: 60초 청크 이벤트 로그(EDGELOG1), 세그먼트
// 프레임 인덱스(EDGEIDX1), 매니페스트, params 스냅샷, 그리고 tmpfs 스테이징 →
// 최종 경로 이동. 기록한 CAN 페이로드를 쓰고 읽는 recorded_can.h, 상태 채널을 이벤트 로그로
// 옮기는 StateRecorder, 그 route를 다시 읽는 replayd의 ReplayRoute도 같이 본다. 보드·인코더 없이
// 합성 레코드로 검사한다.
#include "recording/event_log_reader.h"
#include "recording/recorded_can.h"
#include "recording/recording_format.h"
#include "recording/recording_writer.h"
#include "recording/replay_route.h"
#include "recording/state_recorder.h"
#include "recording/time_ordered_events.h"

#include <dirent.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

namespace {

std::vector<uint8_t> read_file(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        ADD_FAILURE() << "열기 실패 " << path;
        return {};
    }
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::vector<std::string> list_dir(const std::string &path) {
    std::vector<std::string> names;
    if (DIR *dir = opendir(path.c_str())) {
        while (dirent *entry = readdir(dir)) {
            const std::string name = entry->d_name;
            if (name != "." && name != "..") names.push_back(name);
        }
        closedir(dir);
    }
    return names;
}

template <class T>
T read_at(const std::vector<uint8_t> &bytes, size_t offset) {
    if (offset + sizeof(T) > bytes.size()) {
        ADD_FAILURE() << "레코드가 파일 끝을 넘는다";
        return T{};
    }
    T value;
    std::memcpy(&value, bytes.data() + offset, sizeof(T));
    return value;
}

CanBatch synthetic_batch(uint64_t timestamp_ns, uint32_t count, uint32_t dropped) {
    CanBatch batch;
    batch.timestamp_ns = timestamp_ns;
    batch.valid = 1;
    batch.count = count;
    batch.dropped = dropped;
    for (uint32_t i = 0; i < std::min<uint32_t>(count, kCanBatchMaxFrames); ++i) {
        IpcCanFrame &frame = batch.frames[i];
        frame.address = 0x340 + i;
        frame.src = i % 3;
        frame.bus_time = 1000 + i;
        frame.data_len = 8;
        frame.flags = i == 1 ? 0x2 : 0;
        for (int b = 0; b < 8; ++b) frame.data[b] = static_cast<uint8_t>(i * 8 + b);
    }
    return batch;
}

// route를 시작하면 params·모델 사본과 첫 manifest를 바로 SD로 보낸다. 닫을 때까지 tmpfs에만 있으면 시동을 끄는
// 보통의 끝에 통째로 사라졌다.
TEST(RecordingWriter, RouteMetadataReachesTheCardAtStart) {
    char root_template[] = "/tmp/gtest_recording_meta_XXXXXX";
    const std::string root = mkdtemp(root_template);
    const std::string params = root + "/params";
    const std::string recordings = root + "/recordings";
    mkdir(params.c_str(), 0775);
    std::ofstream(params + "/steering.json") << "{\"steer_max\": 384}\n";
    setenv("EDGEPILOT_RECORD_STAGING", (root + "/staging").c_str(), 1);
    {
        RecordingWriter writer(recordings, params, 1280, 720, 20, 8000000);
        writer.set_enabled(true, 5'000'000'000ULL);
        bool arrived = false;
        // 옮기기는 fdatasync를 거치므로 병렬 시험의 부하에서는 2초를 넘기도 했다. 도착하면 곧바로 나온다. 아직 없는
        // 파일은 실패가 아니라 기다릴 일이라 read_file(없으면 실패를 남긴다) 대신 크기만 본다.
        const auto has_content = [](const std::string &path) {
            std::error_code error;
            const auto size = std::filesystem::file_size(path, error);
            return !error && size > 0;
        };
        for (int i = 0; i < 1000 && !arrived; ++i) {
            const std::vector<std::string> routes = list_dir(recordings);
            arrived = routes.size() == 1 && has_content(recordings + "/" + routes[0] + "/params/steering.json") &&
                      has_content(recordings + "/" + routes[0] + "/manifest.json");
            if (!arrived) usleep(10'000);
        }
        EXPECT_TRUE(arrived) << "녹화 중에 params와 manifest가 SD 쪽에 있다";
        EXPECT_TRUE(writer.active());
        writer.close();
    }
    const std::vector<std::string> routes = list_dir(recordings);
    ASSERT_EQ(routes.size(), 1u);
    const std::vector<uint8_t> manifest = read_file(recordings + "/" + routes[0] + "/manifest.json");
    EXPECT_NE(std::string(manifest.begin(), manifest.end()).find("\"complete\": true"), std::string::npos)
        << "닫을 때 쓴 manifest가 처음 것을 덮는다";
    std::filesystem::remove_all(root);
}

// 종료(stop) 뒤에는 속도 제한 없이 옮긴다. 3 MiB/s로는 열린 세그먼트를 다 옮기기 전에 매니저가 SIGKILL했다.
TEST(StagingMover, StopMovesWithoutTheRateLimit) {
    char root_template[] = "/tmp/gtest_recording_move_XXXXXX";
    const std::string root = mkdtemp(root_template);
    mkdir((root + "/from").c_str(), 0775);
    std::ofstream(root + "/from/big.bin") << std::string(9 << 20, 'x');  // 3 MiB/s면 3초
    const auto start = std::chrono::steady_clock::now();
    {
        StagingMover mover;
        mover.enqueue_tree(root + "/from", root + "/to");
        mover.stop();
    }
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    EXPECT_LT(seconds, 1.5);
    EXPECT_EQ(read_file(root + "/to/big.bin").size(), static_cast<size_t>(9 << 20));
    std::filesystem::remove_all(root);
}

// 키프레임마다 영상과 프레임 인덱스를 스테이징에 내보낸다. recordd가 비정상으로 끝나면(닫지 못함) 회수한 마지막
// 세그먼트가 그 키프레임까지 읽힌다. 예전에는 frames.bin이 세그먼트 내내 stdio 버퍼에 있어 0바이트로 남았다.
TEST(RecordingWriter, KeyframesFlushTheFrameIndex) {
    char root_template[] = "/tmp/gtest_recording_flush_XXXXXX";
    const std::string root = mkdtemp(root_template);
    const std::string staging = root + "/staging";
    setenv("EDGEPILOT_RECORD_STAGING", staging.c_str(), 1);
    const uint64_t t0 = 5'000'000'000ULL;
    RecordingWriter writer(root + "/recordings", root + "/params", 1280, 720, 20, 8000000);
    writer.set_storage_check([](uint64_t) { return true; });
    writer.set_enabled(true, t0);
    const uint8_t config[] = {'C', 'F', 'G', 0x01};
    writer.set_codec_config(config, sizeof(config));
    for (uint64_t i = 0; i < 5; ++i) {  // 키프레임은 0번과 3번
        RoadAiFrame frame;
        frame.frame_id = i;
        frame.timestamp_ns = t0 + i * 50'000'000ULL;
        const std::vector<uint8_t> packet(32, static_cast<uint8_t>(0xB0 + i));
        writer.write_encoded_frame(frame, packet.data(), packet.size(), i == 0 || i == 3);
    }
    std::string segment;
    FrameIndex index;
    for (int i = 0; i < 2000 && index.records.size() < 4; ++i) {
        const std::vector<std::string> routes = list_dir(staging);
        if (routes.size() == 1) segment = staging + "/" + routes[0] + "/segments/000";
        if (segment.empty() || !read_frame_index(segment, &index)) index.records.clear();
        if (index.records.size() < 4) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(index.records.size(), 4u) << "두 번째 키프레임까지(닫지 않은 세그먼트)";
    EXPECT_EQ(index.records[3].flags, 1u);
    const std::vector<uint8_t> video = read_file(segment + "/road.h264");
    EXPECT_GE(video.size(), index.records[3].file_offset + index.records[3].packet_size) << "인덱스가 가리키는 영상";
    writer.close();
    std::filesystem::remove_all(root);
}

TEST(RecordingWriter, RouteOnDisk) {
    char root_template[] = "/tmp/gtest_recording_XXXXXX";
    const std::string root = mkdtemp(root_template);
    const std::string staging = root + "/staging";
    const std::string params = root + "/params";
    const std::string recordings = root + "/recordings";
    mkdir(params.c_str(), 0775);
    std::ofstream(params + "/steering.json") << "{\"steer_max\": 384}\n";
    std::ofstream(params + "/lateral_nn.json") << "{\"model\": \"k7_hev_v3\"}\n";  // 레포 루트에서 돈다
    // controlsd의 상태 파일이 없는 자리: route에 남길 모델은 고르기를 따른다(보드의 /dev/shm 파일을 읽지 않는다)
    setenv("EDGEPILOT_LATERAL_NN_STATUS", (root + "/no_status.json").c_str(), 1);
    std::ofstream(params + "/notes.txt") << "not a param file\n";
    std::ofstream(params + "/live_torque_parameters.bin") << "EPTQC001";  // torqued 캐시도 남긴다
    setenv("EDGEPILOT_RECORD_STAGING", staging.c_str(), 1);

    const uint64_t t0 = 5'000'000'000ULL;
    {
        RecordingWriter writer(recordings, params, 1280, 720, 20, 8000000);
        writer.set_enabled(true, t0);
        const uint8_t codec_config[] = {'C', 'F', 'G', 0x01};
        writer.set_codec_config(codec_config, sizeof(codec_config));
        for (uint64_t i = 0; i < 3; ++i) {
            RoadAiFrame frame;
            frame.frame_id = 100 + i;
            frame.timestamp_ns = t0 + i * 50'000'000ULL;
            std::vector<uint8_t> packet(64 + i * 16, static_cast<uint8_t>(0xA0 + i));
            writer.write_encoded_frame(frame, packet.data(), packet.size(), i == 0);
        }
        writer.write_can(RecordType::CanRx, synthetic_batch(t0 + 10'000'000ULL, 3, 2));
        writer.write_can(RecordType::CanTx, synthetic_batch(t0 + 20'000'000ULL, 300, 0));
        std::vector<uint8_t> control(240);
        for (size_t i = 0; i < control.size(); ++i) control[i] = static_cast<uint8_t>(i);
        writer.write_state(RecordType::ControlState, t0 + 30'000'000ULL, control.data(), control.size());
        std::vector<uint8_t> panda(96, 0x5A);
        writer.write_state(RecordType::PandaState, t0 + 40'000'000ULL, panda.data(), panda.size());
        writer.set_enabled(false, t0 + 50'000'000ULL);
        writer.close();
        // route를 끝낸 뒤 writer 카운터
        ASSERT_FALSE(writer.active());
        ASSERT_EQ(writer.video_frames(), 3);
        ASSERT_EQ(writer.queue_drops(), 0);
    }

    const std::vector<std::string> route_dirs = list_dir(recordings);
    ASSERT_EQ(route_dirs.size(), 1) << "route가 정확히 하나 생긴다";
    ASSERT_TRUE(list_dir(staging).empty()) << "close 뒤 스테이징 디렉터리는 빈다";
    const std::string route = recordings + "/" + route_dirs[0];

    // events/000.bin: 헤더 + CanRx(3) + CanTx(256으로 잘림) + 상태 2개
    const std::vector<uint8_t> events = read_file(route + "/events/000.bin");
    ASSERT_EQ(std::memcmp(events.data(), kEventLogMagic, 8), 0) << "이벤트 로그 magic";
    ASSERT_EQ(read_at<uint32_t>(events, 8), kRecordingVersion) << "이벤트 로그 버전";
    const uint32_t header_size = read_at<uint32_t>(events, 12);
    // 이벤트 로그 헤더 크기와 route 시작 시각
    ASSERT_EQ(header_size, sizeof(EventFileHeader));
    ASSERT_EQ(read_at<uint64_t>(events, 16), t0);
    size_t offset = header_size;
    struct Expected {
        uint16_t type;
        uint64_t ts;
        uint32_t payload;
    };
    const size_t frame_bytes = sizeof(RecordedCanFrame);
    const size_t batch_header = sizeof(RecordedCanBatchHeader);
    const Expected expected[] = {
        {1, t0 + 10'000'000ULL, static_cast<uint32_t>(batch_header + 3 * frame_bytes)},
        {2, t0 + 20'000'000ULL, static_cast<uint32_t>(batch_header + 256 * frame_bytes)},
        {4, t0 + 30'000'000ULL, 240},
        {5, t0 + 40'000'000ULL, 96},
    };
    for (const Expected &record : expected) {
        const auto header = read_at<EventRecordHeader>(events, offset);
        // 이벤트 레코드 헤더 순서
        ASSERT_EQ(header.type, record.type);
        ASSERT_EQ(header.timestamp_ns, record.ts);
        ASSERT_EQ(header.payload_size, record.payload);
        offset += sizeof(header);
        if (record.type == 1) {
            const auto batch = read_at<RecordedCanBatchHeader>(events, offset);
            // CanRx 묶음 헤더
            ASSERT_EQ(batch.count, 3);
            ASSERT_EQ(batch.dropped, 2);
            const auto frame1 = read_at<RecordedCanFrame>(events, offset + batch_header + frame_bytes);
            // 기록된 CAN 프레임 필드
            ASSERT_EQ(frame1.address, 0x341);
            ASSERT_EQ(frame1.src, 1);
            ASSERT_EQ(frame1.bus_time, 1001);
            ASSERT_EQ(frame1.data_len, 8);
            ASSERT_EQ(frame1.flags, 0x2);
            ASSERT_EQ(frame1.data[3], 11);
        } else if (record.type == 2) {
            const auto batch = read_at<RecordedCanBatchHeader>(events, offset);
            // 256프레임을 넘는 묶음은 기록 형식의 최대치로 잘린다
            ASSERT_EQ(batch.count, 256);
            ASSERT_EQ(batch.dropped, 0);
        } else if (record.type == 4) {
            ASSERT_EQ(events[offset + 17], 17) << "상태 페이로드는 그대로 저장된다";
        }
        offset += record.payload;
    }
    ASSERT_EQ(offset, events.size()) << "마지막 레코드 뒤에 남는 바이트가 없다";
    {
        // 진단 도구와 replayd가 쓰는 리더도 같은 레코드를 읽는다
        EventLogReader reader(route + "/events/000.bin");
        ASSERT_TRUE(reader.ok());
        EventRecordHeader header{};
        std::vector<char> payload;
        for (const Expected &record : expected) {
            ASSERT_TRUE(reader.next(&header, &payload));
            ASSERT_EQ(header.type, record.type);
            ASSERT_EQ(header.timestamp_ns, record.ts);
            ASSERT_EQ(payload.size(), record.payload);
            if (record.type == 1) {
                // replayd처럼 되돌리면 써 넣은 묶음이 그대로 나온다
                const CanBatch written = synthetic_batch(t0 + 10'000'000ULL, 3, 2);
                CanBatch decoded;
                ASSERT_TRUE(decode_recorded_can(payload.data(), payload.size(), 7, &decoded));
                ASSERT_EQ(decoded.count, 3);
                ASSERT_EQ(decoded.dropped, 2);
                for (uint32_t i = 0; i < decoded.count; ++i) {
                    ASSERT_EQ(decoded.frames[i].address, written.frames[i].address);
                    ASSERT_EQ(decoded.frames[i].flags, written.frames[i].flags);
                    ASSERT_EQ(std::memcmp(decoded.frames[i].data, written.frames[i].data, 8), 0);
                }
            }
        }
        ASSERT_FALSE(reader.next(&header, &payload));
        ASSERT_FALSE(reader.truncated()) << "온전한 파일 끝은 끊김이 아니다";
    }

    // segments/000: 코덱 설정 + 패킷 3개, 인덱스 오프셋은 누적
    const std::vector<uint8_t> video = read_file(route + "/segments/000/road.h264");
    // 영상 스트림은 코덱 설정 뒤에 패킷이 이어진다
    ASSERT_EQ(video.size(), 4 + 64 + 80 + 96);
    ASSERT_EQ(std::memcmp(video.data(), "CFG", 3), 0);
    const std::vector<uint8_t> index = read_file(route + "/segments/000/frames.bin");
    const auto index_header = read_at<FrameIndexHeader>(index, 0);
    // 프레임 인덱스 헤더
    ASSERT_EQ(std::memcmp(index_header.magic, kFrameIndexMagic, 8), 0);
    ASSERT_EQ(index_header.width, 1280);
    ASSERT_EQ(index_header.height, 720);
    ASSERT_EQ(index_header.fps, 20);
    ASSERT_EQ(index_header.record_size, sizeof(FrameIndexRecord));
    ASSERT_EQ(index_header.segment_start_ns, t0);
    ASSERT_EQ(index.size(), sizeof(FrameIndexHeader) + 3 * sizeof(FrameIndexRecord)) << "패킷마다 인덱스 레코드 하나";
    const auto second = read_at<FrameIndexRecord>(index, sizeof(FrameIndexHeader) + sizeof(FrameIndexRecord));
    // 프레임 인덱스 레코드의 오프셋이 스트림을 따른다
    ASSERT_EQ(second.frame_id, 101);
    ASSERT_EQ(second.encode_index, 1);
    ASSERT_EQ(second.file_offset, 4 + 64);
    ASSERT_EQ(second.packet_size, 80);
    ASSERT_EQ(second.flags, 0);

    const std::vector<uint8_t> manifest = read_file(route + "/manifest.json");
    const std::string text(manifest.begin(), manifest.end());
    // 매니페스트의 개수
    ASSERT_NE(text.find("\"complete\": true"), std::string::npos);
    ASSERT_NE(text.find("\"video_frames\": 3"), std::string::npos);
    ASSERT_NE(text.find("\"event_records\": 4"), std::string::npos);
    auto snapshot = list_dir(route + "/params");
    std::sort(snapshot.begin(), snapshot.end());
    // params 스냅샷은 json 파일과 torqued 캐시만 복사한다
    ASSERT_EQ(snapshot, (std::vector<std::string>{"lateral_nn.json", "live_torque_parameters.bin", "steering.json"}));
    // 고른 학습 토크 모델 파일도 route에 남는다(나중에 같은 모델로 재생)
    ASSERT_EQ(read_file(route + "/models/lateral_nn.json"), read_file("models/lateral_nn/k7_hev_v3.json"));

    std::system(("rm -rf '" + root + "'").c_str());
}

// 저장 공간 예비분(5 GiB): 모자라면 거절하고 STORAGE FULL을 띄운다. 녹화를 끄면 경고가 지워진다. 켜 둔 채
// 공간이 예비분보다 1 GiB 넘게 늘면 recordd의 poll_storage가 다시 시작하고, 녹화 중 모자라지면 멈췄다가 같은
// 방법으로 다시 시작한다. 여유 공간은 set_storage_check로 흉내 낸다.
TEST(RecordingWriter, StorageReserveRefusesClearsAndRetries) {
    char root_template[] = "/tmp/gtest_recording_XXXXXX";
    const std::string root = mkdtemp(root_template);
    setenv("EDGEPILOT_RECORD_STAGING", (root + "/staging").c_str(), 1);
    constexpr uint64_t kGiB = 1024ULL * 1024 * 1024;
    constexpr uint64_t kSecond = 1'000'000'000ULL;
    std::atomic<uint64_t> free_bytes{4 * kGiB};
    // 워커 스레드가 처리할 때까지 기다린다(최대 2초).
    const auto settle = [](const auto &done) {
        for (int i = 0; i < 2000 && !done(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return done();
    };

    RecordingWriter writer(root + "/recordings", root + "/params", 1280, 720, 20, 8000000);
    writer.set_storage_check([&](uint64_t extra) { return free_bytes.load() >= 5 * kGiB + extra; });
    uint64_t now = 10 * kSecond;

    writer.set_enabled(true, now);
    ASSERT_TRUE(settle([&] { return writer.storage_blocked() && !writer.requested_enabled(); }))
        << "4 GiB: 거절하고 STORAGE FULL";
    EXPECT_EQ(writer.storage_blocked(), kStorageBlockedSdReserve);
    EXPECT_FALSE(writer.active());
    writer.set_enabled(false, now);
    EXPECT_FALSE(writer.storage_blocked()) << "끄면 이미 거절된 뒤라도 경고가 지워진다";
    writer.poll_storage(now += 10 * kSecond);
    EXPECT_FALSE(writer.requested_enabled()) << "꺼 둔 동안은 다시 시작하지 않는다";

    writer.set_enabled(true, now);
    ASSERT_TRUE(settle([&] { return writer.storage_blocked() && !writer.requested_enabled(); }));
    free_bytes = 5 * kGiB + kGiB / 2;
    writer.poll_storage(now += 10 * kSecond);
    EXPECT_TRUE(writer.storage_blocked()) << "예비분은 넘었지만 1 GiB 여유가 없다";
    EXPECT_FALSE(writer.requested_enabled());
    free_bytes = 7 * kGiB;
    writer.poll_storage(now + kSecond);
    EXPECT_FALSE(writer.requested_enabled()) << "공간은 5초마다만 본다";
    writer.poll_storage(now += 10 * kSecond);
    EXPECT_TRUE(writer.requested_enabled()) << "공간이 돌아오면 다시 녹화한다";
    EXPECT_FALSE(writer.storage_blocked());
    ASSERT_TRUE(settle([&] { return writer.active(); }));

    // 녹화 중 모자라지면 다음 확인(5초 뒤 프레임)에서 멈춘다.
    const uint8_t codec_config[] = {'C', 'F', 'G', 0x01};
    writer.set_codec_config(codec_config, sizeof(codec_config));
    std::vector<uint8_t> packet(64, 0xA0);
    RoadAiFrame frame;
    frame.frame_id = 1;
    frame.timestamp_ns = now;
    writer.write_encoded_frame(frame, packet.data(), packet.size(), true);
    free_bytes = 4 * kGiB;
    frame.frame_id = 2;
    frame.timestamp_ns = now + 6 * kSecond;
    writer.write_encoded_frame(frame, packet.data(), packet.size(), false);
    ASSERT_TRUE(settle([&] { return writer.storage_blocked() && !writer.active(); })) << "녹화 중 공간 부족";
    EXPECT_FALSE(writer.requested_enabled());
    free_bytes = 8 * kGiB;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));  // route 이름은 밀리초 단위 시각이다
    writer.poll_storage(now += 20 * kSecond);
    EXPECT_TRUE(writer.requested_enabled());
    ASSERT_TRUE(settle([&] { return writer.active(); }));
    writer.set_enabled(false, now);
    writer.close();
    EXPECT_EQ(list_dir(root + "/recordings").size(), 2u) << "멈춘 route와 다시 시작한 route";
}

// tmpfs 스테이징 상한(160 MiB): 넘으면 거절하고 SD가 느리다고 알린다(kStorageBlockedStaging, HUD SD CARD SLOW). 상한
// 밑으로 내려와도 절반(80 MiB) 밑이 될 때까지는 다시 시작하지 않고, 녹화 중 넘으면 멈춘다. SD 여유는 넉넉하게 두고,
// 스테이징에 쌓인 크기는 sparse 파일로 흉내 낸다(크기는 st_size로 잰다).
TEST(RecordingWriter, StagingCapStopsAndResumesBelowHalf) {
    char root_template[] = "/tmp/gtest_recording_XXXXXX";
    const std::string root = mkdtemp(root_template);
    const std::string staging = root + "/staging";
    setenv("EDGEPILOT_RECORD_STAGING", staging.c_str(), 1);
    constexpr uint64_t kMiB = 1024ULL * 1024;
    constexpr uint64_t kSecond = 1'000'000'000ULL;
    const auto settle = [](const auto &done) {
        for (int i = 0; i < 2000 && !done(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return done();
    };
    ASSERT_EQ(mkdir(staging.c_str(), 0755), 0);
    const std::string backlog = staging + "/backlog.bin";
    const auto stage = [&](uint64_t bytes) {
        const int fd = ::open(backlog.c_str(), O_CREAT | O_WRONLY, 0644);
        const bool ok = fd >= 0 && ftruncate(fd, static_cast<off_t>(bytes)) == 0;
        if (fd >= 0) ::close(fd);
        return ok;
    };

    RecordingWriter writer(root + "/recordings", root + "/params", 1280, 720, 20, 8000000);
    writer.set_storage_check([](uint64_t) { return true; });
    uint64_t now = 10 * kSecond;
    ASSERT_TRUE(stage(170 * kMiB));
    writer.set_enabled(true, now);
    ASSERT_TRUE(settle([&] { return writer.storage_blocked() && !writer.requested_enabled(); })) << "170 MiB: 거절";
    EXPECT_EQ(writer.storage_blocked(), kStorageBlockedStaging);

    ASSERT_TRUE(stage(100 * kMiB));
    writer.poll_storage(now += 10 * kSecond);
    EXPECT_FALSE(writer.requested_enabled()) << "상한 밑이지만 절반을 넘는다";
    EXPECT_EQ(writer.storage_blocked(), kStorageBlockedStaging);
    ASSERT_TRUE(stage(70 * kMiB));
    writer.poll_storage(now += 10 * kSecond);
    EXPECT_TRUE(writer.requested_enabled()) << "절반 밑이면 다시 녹화한다";
    EXPECT_EQ(writer.storage_blocked(), 0u);
    ASSERT_TRUE(settle([&] { return writer.active(); }));

    // 녹화 중 상한을 넘으면 다음 확인(5초 뒤 프레임)에서 멈춘다.
    const uint8_t codec_config[] = {'C', 'F', 'G', 0x01};
    writer.set_codec_config(codec_config, sizeof(codec_config));
    std::vector<uint8_t> packet(64, 0xA0);
    RoadAiFrame frame;
    frame.frame_id = 1;
    frame.timestamp_ns = now;
    writer.write_encoded_frame(frame, packet.data(), packet.size(), true);
    ASSERT_TRUE(stage(170 * kMiB));
    frame.frame_id = 2;
    frame.timestamp_ns = now + 6 * kSecond;
    writer.write_encoded_frame(frame, packet.data(), packet.size(), false);
    ASSERT_TRUE(settle([&] { return writer.storage_blocked() && !writer.active(); })) << "녹화 중 스테이징 상한";
    EXPECT_EQ(writer.storage_blocked(), kStorageBlockedStaging);
    EXPECT_FALSE(writer.requested_enabled());
    writer.set_enabled(false, now);
    EXPECT_EQ(writer.storage_blocked(), 0u) << "끄면 경고를 지운다";
    writer.close();
    std::system(("rm -rf '" + root + "'").c_str());
}

// recordd는 100 Hz ControlState를 발행마다 기다려 쓰는 스레드로 남긴다. 루프마다 한 번 읽으면 한 바퀴가 10 ms를
// 넘을 때 최신값 채널의 덮어쓴 스냅샷이 빠졌다(녹화의 약 4 %).
TEST(StateRecorder, ControlThreadKeepsEveryPublish) {
    char root_template[] = "/tmp/gtest_state_control_XXXXXX";
    const std::string root = mkdtemp(root_template);
    setenv("EDGEPILOT_RECORD_STAGING", (root + "/staging").c_str(), 1);
    const std::string suffix = "_c" + std::to_string(getpid() % 100000);
    const std::string topic = kControlStateTopic + suffix;
    struct Unlink {
        const std::string &name;
        ~Unlink() { shm_unlink(name.c_str()); }
    } cleanup{topic};
    const uint64_t t0 = 5'000'000'000ULL;
    constexpr int kPublishes = 50;
    {
        RecordingWriter writer(root + "/recordings", root + "/params", 1280, 720, 20, 8000000);
        writer.set_enabled(true, t0);
        StateRecorder recorder(suffix);
        LatestChannel pub;
        ASSERT_TRUE(pub.open(topic.c_str(), sizeof(ControlState), true));
        std::atomic<bool> stop{false};
        std::thread thread([&] {
            while (!stop) recorder.record_control(writer, 20);
        });
        usleep(50'000);  // 스레드가 채널에 붙도록
        ControlState control;
        for (int i = 0; i < kPublishes; ++i) {
            control.timestamp_ns = t0 + 1 + static_cast<uint64_t>(i);
            pub.publish(&control, sizeof(control));
            usleep(5'000);
        }
        usleep(50'000);
        stop = true;
        thread.join();
        writer.set_enabled(false, t0 + 1'000'000'000ULL);
        writer.close();
    }
    const std::vector<std::string> routes = list_dir(root + "/recordings");
    ASSERT_EQ(routes.size(), 1u);
    EventLogReader reader(root + "/recordings/" + routes[0] + "/events/000.bin");
    ASSERT_TRUE(reader.ok());
    EventRecordHeader header{};
    std::vector<char> payload;
    int controls = 0;
    while (reader.next(&header, &payload)) controls += header.type == static_cast<uint16_t>(RecordType::ControlState);
    EXPECT_EQ(controls, kPublishes);
    std::filesystem::remove_all(root);
}

TEST(StateRecorder, WritesEachNewSnapshotOnce) {
    char root_template[] = "/tmp/gtest_state_recorder_XXXXXX";
    const std::string root = mkdtemp(root_template);
    setenv("EDGEPILOT_RECORD_STAGING", (root + "/staging").c_str(), 1);
    // macOS는 shm 이름을 31자까지 받는다: 가장 긴 토픽(24자) 뒤에 7자까지
    const std::string suffix = "_" + std::to_string(getpid() % 100000);
    const std::string topics[] = {kModelStateTopic + suffix, kControlStateTopic + suffix,
                                  kPandaStateTopic + suffix, kLearnerStateTopic + suffix,
                                  kImuTopic + suffix,        kLocalizationStateTopic + suffix};
    // ASSERT가 중간에 빠져나가도 공유 메모리를 지운다.
    struct Unlink {
        const std::string (&names)[6];
        ~Unlink() {
            for (const std::string &name : names) shm_unlink(name.c_str());
        }
    } cleanup{topics};

    const uint64_t t0 = 5'000'000'000ULL;
    ModelState model;
    ControlState control;
    PandaState panda;
    LearnerState learner;
    ImuBatch imu;
    LocalizationState localization;
    {
        RecordingWriter writer(root + "/recordings", root + "/params", 1280, 720, 20, 8000000);
        writer.set_enabled(true, t0);
        StateRecorder recorder(suffix);
        recorder.record(writer);  // 생산자가 아직 없다

        LatestChannel pubs[6];
        const size_t sizes[] = {sizeof(model),   sizeof(control), sizeof(panda),
                                sizeof(learner), sizeof(imu),     sizeof(localization)};
        for (int i = 0; i < 6; ++i) ASSERT_TRUE(pubs[i].open(topics[i].c_str(), sizes[i], true)) << topics[i];
        model.model_timestamp_ns = t0 + 1;
        control.timestamp_ns = t0 + 2;
        panda.timestamp_ns = t0 + 3;
        learner.timestamp_ns = t0 + 4;
        imu.timestamp_ns = t0 + 5;
        imu.count = 2;
        localization.timestamp_ns = t0 + 6;
        const void *payloads[] = {&model, &control, &panda, &learner, &imu, &localization};
        for (int i = 0; i < 6; ++i) pubs[i].publish(payloads[i], sizes[i]);
        recorder.record(writer);
        recorder.record(writer);  // 새 스냅샷이 없으면 쓰지 않는다

        control.timestamp_ns = t0 + 7;
        pubs[1].publish(&control, sizeof(control));
        imu.timestamp_ns = t0 + 8;
        imu.count = 0;  // 빈 IMU 묶음은 남기지 않는다
        pubs[4].publish(&imu, sizeof(imu));
        recorder.record(writer);
        writer.set_enabled(false, t0 + 9);
        writer.close();
    }

    const std::vector<std::string> routes = list_dir(root + "/recordings");
    ASSERT_EQ(routes.size(), 1);
    EventLogReader reader(root + "/recordings/" + routes[0] + "/events/000.bin");
    ASSERT_TRUE(reader.ok());
    struct Expected {
        RecordType type;
        uint64_t ts;
        size_t payload;
    };
    const Expected expected[] = {
        {RecordType::ModelState, t0 + 1, sizeof(ModelState)},
        {RecordType::ControlState, t0 + 2, sizeof(ControlState)},
        {RecordType::PandaState, t0 + 3, sizeof(PandaState)},
        {RecordType::LearnerState, t0 + 4, sizeof(LearnerState)},
        {RecordType::Imu, t0 + 5, offsetof(ImuBatch, samples) + 2 * sizeof(ImuSample)},
        {RecordType::Localization, t0 + 6, sizeof(LocalizationState)},
        {RecordType::ControlState, t0 + 7, sizeof(ControlState)},
    };
    EventRecordHeader header{};
    std::vector<char> payload;
    for (const Expected &record : expected) {
        ASSERT_TRUE(reader.next(&header, &payload));
        // 채널 순서대로 새 스냅샷 하나씩, IMU는 채운 샘플까지
        ASSERT_EQ(header.type, static_cast<uint16_t>(record.type));
        ASSERT_EQ(header.timestamp_ns, record.ts);
        ASSERT_EQ(payload.size(), record.payload);
    }
    ASSERT_FALSE(reader.next(&header, &payload)) << "같은 스냅샷을 두 번 쓰지 않는다";

    std::system(("rm -rf '" + root + "'").c_str());
}

TEST(ReplayRoute, ReadsTheWrittenRouteBack) {
    char root_template[] = "/tmp/gtest_replay_route_XXXXXX";
    const std::string root = mkdtemp(root_template);
    setenv("EDGEPILOT_RECORD_STAGING", (root + "/staging").c_str(), 1);
    const uint64_t t0 = 5'000'000'000ULL, kMinute = 60'000'000'000ULL;
    const uint8_t config[] = {'C', 'F', 'G', 0x01};
    struct Written {
        uint64_t ts;
        bool key;
        uint8_t fill;
        size_t size;
    };
    const Written written[] = {{t0, true, 0xA0, 40},
                               {t0 + 50'000'000ULL, false, 0xA1, 24},
                               {t0 + 100'000'000ULL, true, 0xA2, 32},  // 세그먼트 가운데 키프레임
                               {t0 + kMinute, true, 0xA3, 48},         // 60초 뒤 키프레임: 다음 세그먼트
                               {t0 + kMinute + 50'000'000ULL, false, 0xA4, 16}};
    {
        RecordingWriter writer(root + "/recordings", root + "/params", 1280, 720, 20, 8000000);
        writer.set_enabled(true, t0);
        writer.set_codec_config(config, sizeof(config));
        for (size_t i = 0; i < std::size(written); ++i) {
            RoadAiFrame frame;
            frame.frame_id = i;
            frame.timestamp_ns = written[i].ts;
            const std::vector<uint8_t> packet(written[i].size, written[i].fill);
            writer.write_encoded_frame(frame, packet.data(), packet.size(), written[i].key);
        }
        const std::vector<uint8_t> panda(sizeof(PandaState), 0x5A), control(sizeof(ControlState), 1);
        writer.write_can(RecordType::CanRx, synthetic_batch(t0 + 30'000'000ULL, 2, 0));
        writer.write_state(RecordType::PandaState, t0 + 20'000'000ULL, panda.data(),
                           panda.size());  // 앞선 시각을 나중에
        writer.write_state(RecordType::ControlState, t0 + 25'000'000ULL, control.data(), control.size());
        writer.write_can(RecordType::CanTx, synthetic_batch(t0 + 26'000'000ULL, 1, 0));
        writer.write_state(RecordType::PandaState, t0 + kMinute + 10'000'000ULL, panda.data(),
                           panda.size());  // 다음 청크
        // 프레임 0과 2의 모델 출력(프레임 1은 없음): 모델 시각 − 촬영 시각 − 실행 시간이 녹화의 카메라 지연
        ModelState model;
        model.capture_timestamp_ns = written[0].ts;
        model.model_timestamp_ns = written[0].ts + 62'000'000ULL;
        model.model_execution_ms = 18.5f;
        writer.write_state(RecordType::ModelState, model.model_timestamp_ns, &model, sizeof(model));
        model.capture_timestamp_ns = written[2].ts;
        model.model_timestamp_ns = written[2].ts + 70'000'000ULL;
        model.model_execution_ms = 20.0f;
        writer.write_state(RecordType::ModelState, model.model_timestamp_ns, &model, sizeof(model));
        writer.set_enabled(false, t0 + 2 * kMinute);
        writer.close();
    }
    const std::vector<std::string> routes = list_dir(root + "/recordings");
    ASSERT_EQ(routes.size(), 1);
    ReplayRoute route(root + "/recordings/" + routes[0]);

    // 세그먼트 둘, 프레임 다섯
    ASSERT_EQ(route.segments.size(), 2);
    ASSERT_EQ(route.frames.size(), std::size(written));
    ASSERT_EQ(route.width, 1280);
    ASSERT_EQ(route.height, 720);
    for (size_t i = 0; i < std::size(written); ++i) {
        // 인덱스의 시각·키프레임·세그먼트
        ASSERT_EQ(route.frames[i].capture_ns, written[i].ts);
        ASSERT_EQ(route.frames[i].keyframe, written[i].key);
        ASSERT_EQ(route.frames[i].segment, i < 3 ? 0u : 1u);
        ASSERT_EQ(route.frames[i].segment_first, i == 0 || i == 3);
    }
    ASSERT_EQ(route.segments[1].header_bytes, sizeof(config)) << "세그먼트마다 앞에 코덱 설정";

    const auto packet = [&](size_t i) { return std::vector<uint8_t>(written[i].size, written[i].fill); };
    const auto with_config = [&](size_t i) {
        std::vector<uint8_t> bytes(config, config + sizeof(config));
        const std::vector<uint8_t> tail = packet(i);
        bytes.insert(bytes.end(), tail.begin(), tail.end());
        return bytes;
    };
    std::vector<uint8_t> out;
    ASSERT_TRUE(route.read_frame(route.frames[0], &out));
    ASSERT_EQ(out, with_config(0)) << "세그먼트 첫 키프레임은 파일 처음부터";
    ASSERT_TRUE(route.read_frame(route.frames[1], &out));
    ASSERT_EQ(out, packet(1)) << "키프레임이 아니면 패킷만";
    ASSERT_TRUE(route.read_frame(route.frames[2], &out));
    ASSERT_EQ(out, with_config(2)) << "가운데 키프레임에도 코덱 설정을 붙인다";
    ASSERT_TRUE(route.read_frame(route.frames[3], &out));
    ASSERT_EQ(out, with_config(3));
    ASSERT_TRUE(route.read_frame(route.frames[4], &out));
    ASSERT_EQ(out, packet(4));

    // 이벤트는 고른 종류(CanRx와 PandaState)만, 청크를 넘어 시각 순으로 흘려 읽는다
    const auto read_events = [&](uint64_t from_ns, size_t *chunks) {
        ReplayEvents events(root + "/recordings/" + routes[0], from_ns, {RecordType::CanRx, RecordType::PandaState});
        *chunks = events.chunks();
        std::vector<ReplayEvents::Event> out;
        for (const ReplayEvents::Event *e = events.peek(); e; e = events.peek()) {
            out.push_back(*e);
            events.pop();
        }
        return out;
    };
    size_t chunks = 0;
    std::vector<ReplayEvents::Event> events = read_events(t0, &chunks);
    EXPECT_EQ(chunks, 2u);
    ASSERT_EQ(events.size(), 3);
    EXPECT_EQ(events[0].header.type, static_cast<uint16_t>(RecordType::PandaState));
    EXPECT_EQ(events[0].header.timestamp_ns, t0 + 20'000'000ULL);
    EXPECT_EQ(events[1].header.type, static_cast<uint16_t>(RecordType::CanRx));
    EXPECT_EQ(events[1].header.timestamp_ns, t0 + 30'000'000ULL);
    CanBatch batch;
    ASSERT_TRUE(decode_recorded_can(events[1].payload.data(), events[1].payload.size(), 0, &batch));
    EXPECT_EQ(batch.count, 2);
    EXPECT_EQ(events[2].header.timestamp_ns, t0 + kMinute + 10'000'000ULL);
    // 녹화 카메라 지연은 그 프레임의 모델 출력에서, 모델 출력이 없는 프레임은 직전 값
    RecordedCameraDelay delay(root + "/recordings/" + routes[0], t0);
    EXPECT_EQ(delay.at(written[0].ts), 43'500'000ULL);
    EXPECT_EQ(delay.at(written[1].ts), 43'500'000ULL);
    EXPECT_EQ(delay.at(written[2].ts), 50'000'000ULL);
    EXPECT_EQ(delay.at(written[3].ts), 50'000'000ULL);
    // 중간에서 시작하면 그 앞의 이벤트는 내지 않고, 2초 넘게 앞선 청크는 열지 않는다
    events = read_events(t0 + kMinute + 5'000'000ULL, &chunks);
    EXPECT_EQ(chunks, 2u) << "다음 청크가 시작보다 2초 넘게 앞서지 않는다";
    ASSERT_EQ(events.size(), 1);
    EXPECT_EQ(events[0].header.timestamp_ns, t0 + kMinute + 10'000'000ULL);
    events = read_events(t0 + kMinute + 3'000'000'000ULL, &chunks);
    EXPECT_EQ(chunks, 1u) << "첫 청크는 건너뛴다";
    EXPECT_TRUE(events.empty());
    // 끊긴 꼬리(0으로 채워진 구간)는 그 앞까지 내고 stderr에 알린다
    const std::string last_chunk = root + "/recordings/" + routes[0] + "/events/001.bin";
    std::ofstream(last_chunk, std::ios::binary | std::ios::app) << std::string(64, '\0');
    testing::internal::CaptureStderr();
    events = read_events(t0, &chunks);
    const std::string log = testing::internal::GetCapturedStderr();
    EXPECT_EQ(events.size(), 3u);
    EXPECT_NE(log.find("replayd: " + last_chunk + ": truncated at"), std::string::npos) << log;

    // 다른 녹화 버전의 인덱스는 읽지 않는다(그 세그먼트만 건너뛴다)
    const std::string first_index = root + "/recordings/" + routes[0] + "/segments/000/frames.bin";
    const std::vector<uint8_t> original = read_file(first_index);
    std::vector<uint8_t> older = original;
    FrameIndexHeader older_header = read_at<FrameIndexHeader>(older, 0);
    older_header.version = kRecordingVersion - 1;
    std::memcpy(older.data(), &older_header, sizeof(older_header));
    std::ofstream(first_index, std::ios::binary)
        .write(reinterpret_cast<const char *>(older.data()), static_cast<std::streamsize>(older.size()));
    ASSERT_EQ(ReplayRoute(root + "/recordings/" + routes[0]).segments.size(), 1);
    std::ofstream(first_index, std::ios::binary)
        .write(reinterpret_cast<const char *>(original.data()), static_cast<std::streamsize>(original.size()));
    ASSERT_EQ(ReplayRoute(root + "/recordings/" + routes[0]).frames.size(), std::size(written));

    // header_size가 파일보다 큰 인덱스(깨진 머리)는 그 세그먼트만 건너뛴다
    const std::string index_path = root + "/recordings/" + routes[0] + "/segments/001/frames.bin";
    std::vector<uint8_t> index = read_file(index_path);
    FrameIndexHeader index_header = read_at<FrameIndexHeader>(index, 0);
    index_header.header_size = 0xFFFFFFF0u;
    std::memcpy(index.data(), &index_header, sizeof(index_header));
    std::ofstream(index_path, std::ios::binary)
        .write(reinterpret_cast<const char *>(index.data()), static_cast<std::streamsize>(index.size()));
    ReplayRoute broken(root + "/recordings/" + routes[0]);
    ASSERT_EQ(broken.segments.size(), 1);
    ASSERT_EQ(broken.frames.size(), 3);

    std::system(("rm -rf '" + root + "'").c_str());
}

// 기록한 CAN 페이로드(recorded_can.h): 64바이트 CAN-FD 데이터까지 그대로 되돌리고, 페이로드 밖으로
// 나간 프레임은 읽지 않으며, 쓸 때와 replayd로 되돌릴 때 기록 형식의 최대 프레임 수로 자른다.
TEST(RecordedCan, RoundTripAndTruncation) {
    CanBatch batch = synthetic_batch(1, 3, 5);
    batch.frames[2].data_len = 64;
    for (int b = 0; b < 64; ++b) batch.frames[2].data[b] = static_cast<uint8_t>(0xC0 ^ b);
    const std::vector<uint8_t> payload = encode_recorded_can(batch);
    ASSERT_EQ(payload.size(), sizeof(RecordedCanBatchHeader) + 3 * sizeof(RecordedCanFrame));

    CanBatch decoded;
    ASSERT_TRUE(decode_recorded_can(payload.data(), payload.size(), 42, &decoded));
    ASSERT_EQ(decoded.valid, 1);
    ASSERT_EQ(decoded.timestamp_ns, 42);
    ASSERT_EQ(decoded.count, 3);
    ASSERT_EQ(decoded.dropped, 5);
    for (uint32_t i = 0; i < 3; ++i) {
        const IpcCanFrame &in = batch.frames[i];
        const IpcCanFrame &out = decoded.frames[i];
        ASSERT_EQ(out.address, in.address);
        ASSERT_EQ(out.src, in.src);
        ASSERT_EQ(out.bus_time, in.bus_time);
        ASSERT_EQ(out.data_len, in.data_len);
        ASSERT_EQ(out.flags, in.flags);
        ASSERT_EQ(std::memcmp(out.data, in.data, sizeof(in.data)), 0) << "프레임 " << i;
    }

    const auto ignore = [](const RecordedCanFrame &) {};
    ASSERT_EQ(for_each_recorded_can_frame(payload.data(), payload.size() - 1, ignore), 2)
        << "마지막 프레임 중간에서 끊기면 온전한 프레임만 읽는다";
    ASSERT_TRUE(decode_recorded_can(payload.data(), payload.size() - 1, 0, &decoded));
    ASSERT_EQ(decoded.count, 2);
    ASSERT_EQ(for_each_recorded_can_frame(payload.data(), sizeof(RecordedCanBatchHeader) - 1, ignore), 0);
    ASSERT_FALSE(decode_recorded_can(payload.data(), sizeof(RecordedCanBatchHeader) - 1, 0, &decoded))
        << "머리보다 짧으면 false";

    ASSERT_EQ(encode_recorded_can(synthetic_batch(1, 300, 0)).size(),
              sizeof(RecordedCanBatchHeader) + kCanBatchMaxFrames * sizeof(RecordedCanFrame))
        << "쓸 때 기록 형식의 최대 프레임 수로 자른다";
    std::vector<uint8_t> oversized(sizeof(RecordedCanBatchHeader) + 300 * sizeof(RecordedCanFrame));
    const RecordedCanBatchHeader header{300, 0};
    std::memcpy(oversized.data(), &header, sizeof(header));
    ASSERT_EQ(for_each_recorded_can_frame(oversized.data(), oversized.size(), ignore), 300);
    ASSERT_TRUE(decode_recorded_can(oversized.data(), oversized.size(), 0, &decoded));
    ASSERT_EQ(decoded.count, kCanBatchMaxFrames) << "CanBatch로 되돌릴 때도 최대치로 자른다";
}

// 파일 순서가 시각 순이 아닌 녹화(recordd는 쌓인 CAN을 먼저, 상태 스냅샷을 뒤에 쓴다): 창 안에서 시각 순으로 내고,
// 같은 시각은 파일 순서를 지키며, 파일을 넘어서도 이어 붙인다. 열지 못한 파일은 알리고 건너뛴다.
TEST(TimeOrderedEvents, ReordersAcrossFiles) {
    char root_template[] = "/tmp/gtest_ordered_XXXXXX";
    const std::string root = mkdtemp(root_template);
    using Entry = std::pair<uint64_t, uint16_t>;
    const auto write_log = [&](const std::string &name, const std::vector<Entry> &records) {
        std::ofstream out(root + "/" + name, std::ios::binary);
        const EventFileHeader file_header;
        out.write(reinterpret_cast<const char *>(&file_header), sizeof(file_header));
        for (const auto &[timestamp_ns, type] : records) {
            EventRecordHeader record;
            record.timestamp_ns = timestamp_ns;
            record.type = type;
            record.payload_size = 1;
            out.write(reinterpret_cast<const char *>(&record), sizeof(record));
            out.put('x');
        }
        return root + "/" + name;
    };
    const auto can = static_cast<uint16_t>(RecordType::CanRx);
    const auto control = static_cast<uint16_t>(RecordType::ControlState);
    // ms: CAN 10·20 뒤에 늦게 쓴 ControlState 15·20, 다음 파일의 CAN 18·30
    const std::string a =
        write_log("000.bin", {{10'000'000, can}, {20'000'000, can}, {15'000'000, control}, {20'000'000, control}});
    const std::string b = write_log("001.bin", {{18'000'000, can}, {30'000'000, can}});
    TimeOrderedEvents events({a, root + "/missing.bin", b}, 100'000'000);
    std::vector<std::string> ends;
    events.on_file_end = [&](const std::string &path, const EventLogReader &reader) {
        ends.push_back(path + (reader.ok() ? "" : " bad"));
    };
    std::vector<Entry> got;
    TimeOrderedEvents::Record record;
    while (events.next(&record)) got.emplace_back(record.header.timestamp_ns, record.header.type);
    const std::vector<Entry> want = {{10'000'000, can}, {15'000'000, control}, {18'000'000, can},
                                     {20'000'000, can}, {20'000'000, control}, {30'000'000, can}};
    EXPECT_EQ(got, want);
    EXPECT_EQ(ends, (std::vector<std::string>{a, root + "/missing.bin bad", b}));
}

// 끊긴 이벤트 로그: tmpfs가 차서 0으로 채워진 꼬리, 모자란 페이로드·레코드 머리, 1 MiB를 넘는
// 길이. 리더는 재동기화 없이 거기서 끝내고 truncated()로 알린다. header_size가 구조체보다 큰
// (뒤에 필드가 붙은) 머리는 header_size만큼 건너뛴다.
TEST(EventLogReader, StopsAtTruncatedTail) {
    char root_template[] = "/tmp/gtest_event_log_XXXXXX";
    const std::string root = mkdtemp(root_template);
    const auto write_log = [&](const std::string &name, uint32_t extra_header, uint32_t claimed_payload,
                               const std::string &tail) {
        EventFileHeader file_header;
        file_header.header_size = sizeof(EventFileHeader) + extra_header;
        EventRecordHeader record;
        record.timestamp_ns = 123;
        record.type = static_cast<uint16_t>(RecordType::ControlState);
        record.payload_size = claimed_payload;
        std::ofstream out(root + "/" + name, std::ios::binary);
        out.write(reinterpret_cast<const char *>(&file_header), sizeof(file_header));
        out << std::string(extra_header, '\x7f');
        out.write(reinterpret_cast<const char *>(&record), sizeof(record));
        out << tail;
        return root + "/" + name;
    };
    const long long first_record = sizeof(EventFileHeader);
    EventRecordHeader header;
    std::vector<char> payload;

    {
        // 레코드 하나 + 0으로 채워진 꼬리. 큰 머리를 건너뛴다
        EventLogReader reader(write_log("zero_tail.bin", 8, 4, std::string(4, '\x11') + std::string(32, '\0')));
        ASSERT_TRUE(reader.ok());
        ASSERT_TRUE(reader.next(&header, &payload));
        ASSERT_EQ(header.timestamp_ns, 123);
        ASSERT_EQ(payload, std::vector<char>(4, '\x11'));
        ASSERT_FALSE(reader.next(&header, &payload));
        ASSERT_TRUE(reader.truncated());
        ASSERT_EQ(reader.truncated_at(), first_record + 8 + static_cast<long long>(sizeof(EventRecordHeader)) + 4);
        ASSERT_FALSE(reader.next(&header, &payload)) << "끝난 뒤에는 계속 false";
    }
    {
        // 레코드 하나로 깨끗하게 끝난다
        EventLogReader reader(write_log("clean.bin", 0, 4, std::string(4, '\x11')));
        ASSERT_TRUE(reader.next(&header, &payload));
        ASSERT_FALSE(reader.next(&header, &payload));
        ASSERT_FALSE(reader.truncated());
    }
    {
        // 다른 녹화 버전은 읽지 않는다(옛 녹화는 모두 지금 버전으로 다시 썼다)
        const std::string path = write_log("older.bin", 0, 4, std::string(4, '\x11'));
        std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
        const uint32_t version = kRecordingVersion - 1;
        file.seekp(8);
        file.write(reinterpret_cast<const char *>(&version), sizeof(version));
        file.close();
        EventLogReader reader(path);
        ASSERT_FALSE(reader.ok()) << "옛 버전";
        ASSERT_FALSE(reader.next(&header, &payload));
    }
    // 페이로드가 모자람, 길이가 1 MiB 초과, 다음 레코드 머리가 반만 있음
    const struct {
        const char *name;
        uint32_t claimed;
        std::string tail;
        int records;
    } cases[] = {
        {"short_payload.bin", 100, std::string(10, '\x11'), 0},
        {"huge_payload.bin", 2U << 20, std::string(4, '\x11'), 0},
        {"partial_header.bin", 4, std::string(4, '\x11') + std::string(5, '\x22'), 1},
    };
    for (const auto &c : cases) {
        EventLogReader reader(write_log(c.name, 0, c.claimed, c.tail));
        int records = 0;
        while (reader.next(&header, &payload)) ++records;
        EXPECT_EQ(records, c.records) << c.name;
        EXPECT_TRUE(reader.truncated()) << c.name;
        EXPECT_EQ(reader.truncated_at(),
                  first_record + c.records * static_cast<long long>(sizeof(EventRecordHeader) + 4))
            << c.name;
    }
    {
        std::ofstream(root + "/not_a_log.bin", std::ios::binary) << std::string(64, 'x');
        EventLogReader reader(root + "/not_a_log.bin");
        ASSERT_FALSE(reader.ok());
        ASSERT_FALSE(reader.next(&header, &payload));
        ASSERT_FALSE(EventLogReader(root + "/missing.bin").ok());
    }

    std::system(("rm -rf '" + root + "'").c_str());
}

}  // namespace
