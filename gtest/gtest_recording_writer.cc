/* RecordingWriter가 디스크에 남기는 것: 60초 청크 이벤트 로그(K230LOG1), 세그먼트
 * 프레임 인덱스(K230IDX1), 매니페스트, params 스냅샷, 그리고 tmpfs 스테이징 →
 * 최종 경로 이동. 기록한 CAN 페이로드를 쓰고 읽는 recorded_can.h도 같이 본다. 보드·인코더 없이
 * 합성 레코드로 검사한다. */
#include "event_log_reader.h"
#include "recorded_can.h"
#include "recorded_model_state.h"
#include "recording_format.h"
#include "recording_writer.h"

#include <gtest/gtest.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

std::vector<uint8_t> read_file(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    ADD_FAILURE() << "열기 실패 " << path;
    return {};
  }
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
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

TEST(RecordingWriter, RouteOnDisk) {
  char root_template[] = "/tmp/gtest_recording_XXXXXX";
  const std::string root = mkdtemp(root_template);
  const std::string staging = root + "/staging";
  const std::string params = root + "/params";
  const std::string recordings = root + "/recordings";
  mkdir(params.c_str(), 0775);
  std::ofstream(params + "/steering.json") << "{\"steer_max\": 384}\n";
  std::ofstream(params + "/notes.txt") << "not a param file\n";
  setenv("EDGEPILOT_RECORD_STAGING", staging.c_str(), 1);

  const uint64_t t0 = 5'000'000'000ULL;
  {
    RecordingWriter writer(recordings, params, 1280, 720, 20, 8000000, VideoCodec::H264);
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
    writer.write_can(RecordType::ModelState, synthetic_batch(t0, 1, 0));  // 무시돼야 한다
    std::vector<uint8_t> control(240);
    for (size_t i = 0; i < control.size(); ++i) control[i] = static_cast<uint8_t>(i);
    writer.write_state(RecordType::ControlState, t0 + 30'000'000ULL, control.data(),
                       control.size());
    std::vector<uint8_t> panda(96, 0x5A);
    writer.write_state(RecordType::PandaState, t0 + 40'000'000ULL, panda.data(),
                       panda.size());
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
  ASSERT_EQ(std::memcmp(events.data(), "K230LOG1", 8), 0) << "이벤트 로그 magic";
  ASSERT_EQ(read_at<uint32_t>(events, 8), kRecordingVersion) << "이벤트 로그 버전";
  const uint32_t header_size = read_at<uint32_t>(events, 12);
  // 이벤트 로그 헤더 크기와 route 시작 시각
  ASSERT_EQ(header_size, sizeof(EventFileHeader));
  ASSERT_EQ(read_at<uint64_t>(events, 16), t0);
  size_t offset = header_size;
  struct Expected { uint16_t type; uint64_t ts; uint32_t payload; };
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
    ASSERT_EQ(reader.version(), kRecordingVersion);
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
        const CanBatch decoded = decode_recorded_can(payload.data(), payload.size(), 7);
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
  ASSERT_EQ(std::memcmp(index_header.magic, "K230IDX1", 8), 0);
  ASSERT_EQ(index_header.width, 1280);
  ASSERT_EQ(index_header.height, 720);
  ASSERT_EQ(index_header.fps, 20);
  ASSERT_EQ(index_header.record_size, sizeof(FrameIndexRecord));
  ASSERT_EQ(index_header.segment_start_ns, t0);
  ASSERT_EQ(index.size(), sizeof(FrameIndexHeader) + 3 * sizeof(FrameIndexRecord))
      << "패킷마다 인덱스 레코드 하나";
  const auto second = read_at<FrameIndexRecord>(
      index, sizeof(FrameIndexHeader) + sizeof(FrameIndexRecord));
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
  const auto snapshot = list_dir(route + "/params");
  // params 스냅샷은 json 파일만 복사한다
  ASSERT_EQ(snapshot.size(), 1);
  ASSERT_EQ(snapshot[0], "steering.json");

  std::system(("rm -rf '" + root + "'").c_str());
}

/* 기록한 CAN 페이로드(recorded_can.h): 64바이트 CAN-FD 데이터까지 그대로 되돌리고, 페이로드 밖으로
 * 나간 프레임은 읽지 않으며, 쓸 때와 replayd로 되돌릴 때 기록 형식의 최대 프레임 수로 자른다. */
TEST(RecordedCan, RoundTripAndTruncation) {
  CanBatch batch = synthetic_batch(1, 3, 5);
  batch.frames[2].data_len = 64;
  for (int b = 0; b < 64; ++b) batch.frames[2].data[b] = static_cast<uint8_t>(0xC0 ^ b);
  const std::vector<uint8_t> payload = encode_recorded_can(batch);
  ASSERT_EQ(payload.size(), sizeof(RecordedCanBatchHeader) + 3 * sizeof(RecordedCanFrame));

  const CanBatch decoded = decode_recorded_can(payload.data(), payload.size(), 42);
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
  ASSERT_EQ(decode_recorded_can(payload.data(), payload.size() - 1, 0).count, 2);
  ASSERT_EQ(for_each_recorded_can_frame(payload.data(), sizeof(RecordedCanBatchHeader) - 1, ignore), 0);
  ASSERT_EQ(decode_recorded_can(payload.data(), sizeof(RecordedCanBatchHeader) - 1, 0).valid, 0)
      << "머리보다 짧으면 빈 묶음";

  ASSERT_EQ(encode_recorded_can(synthetic_batch(1, 300, 0)).size(),
            sizeof(RecordedCanBatchHeader) + kCanBatchMaxFrames * sizeof(RecordedCanFrame))
      << "쓸 때 기록 형식의 최대 프레임 수로 자른다";
  std::vector<uint8_t> oversized(sizeof(RecordedCanBatchHeader) + 300 * sizeof(RecordedCanFrame));
  const RecordedCanBatchHeader header{300, 0};
  std::memcpy(oversized.data(), &header, sizeof(header));
  ASSERT_EQ(for_each_recorded_can_frame(oversized.data(), oversized.size(), ignore), 300);
  ASSERT_EQ(decode_recorded_can(oversized.data(), oversized.size(), 0).count, kCanBatchMaxFrames)
      << "CanBatch로 되돌릴 때도 최대치로 자른다";
}

/* 끊긴 이벤트 로그: tmpfs가 차서 0으로 채워진 꼬리, 모자란 페이로드·레코드 머리, 1 MiB를 넘는
 * 길이. 리더는 재동기화 없이 거기서 끝내고 truncated()로 알린다. header_size가 구조체보다 큰
 * (뒤에 필드가 붙은) 머리는 header_size만큼 건너뛴다. */
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

/* 진단 도구가 옛 녹화의 ModelState를 현재 구조체로 읽는다: v7은 카메라 장착까지, v6은 그 앞까지
 * (장착은 0), v5는 plan_yaw 앞까지. */
TEST(RecordedModelState, DecodesV5ToV7) {
  ModelState src;
  src.frame_id = 42;
  src.calibration.yaw = 0.01f;
  src.plan_yaw[3] = 0.2f;
  src.camera_offset_m = 0.15f;
  src.camera_height_m = 1.3f;
  const char *bytes = reinterpret_cast<const char *>(&src);
  ModelState out;
  ASSERT_TRUE(decode_recorded_model_state(bytes, sizeof(ModelState), 7, &out));
  EXPECT_EQ(out.frame_id, 42u);
  EXPECT_FLOAT_EQ(out.camera_offset_m, 0.15f);
  EXPECT_FLOAT_EQ(out.camera_height_m, 1.3f);
  ASSERT_TRUE(decode_recorded_model_state(bytes, offsetof(ModelState, camera_offset_m), 6, &out));
  EXPECT_FLOAT_EQ(out.plan_yaw[3], 0.2f);
  EXPECT_FLOAT_EQ(out.camera_offset_m, 0.0f);
  ASSERT_TRUE(decode_recorded_model_state(bytes, offsetof(ModelState, plan_yaw), 5, &out));
  EXPECT_FLOAT_EQ(out.calibration.yaw, 0.01f);
  EXPECT_FLOAT_EQ(out.plan_yaw[3], 0.0f);
  EXPECT_FALSE(decode_recorded_model_state(bytes, offsetof(ModelState, camera_offset_m), 7, &out))
      << "v7이라면서 짧으면 거부";
}

}  // namespace
