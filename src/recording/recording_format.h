#pragma once

#include <cstdint>
#include <cstring>

/* v2: ModelState에서 소비자가 없던 lateral_target/lateral_plan(308 B)을
 * 제거해 페이로드가 4384 -> 4076 B로 줄었다. v1 녹화와 호환되지 않는다.
 * v3: 이벤트 로그를 route 단위 events.bin 하나에서 60초 청크 events/NNN.bin
 * 으로 분할했다. route 단위 파일은 CAN 로깅(~0.5 MB/s)만으로 30분이면 988 MB
 * tmpfs 스테이징을 가득 채워 남은 주행의 기록을 전부 죽였다. 레코드/헤더
 * 레이아웃은 v2와 동일하고 각 청크가 EventFileHeader로 시작한다.
 * v4: openpilot v0.9.4 모델로 옮기면서 그 모델이 내지 않는 stop_line(28 B)을
 * ModelState에서 뺐다.
 * v5: 소비자가 없던 plan_position_stds/plan_orientations(792 B)를
 * ModelState에서 뺐다. 페이로드가 4048 -> 3256 B로 줄어 v4 이하 녹화와
 * 호환되지 않는다. LearnerState(6)는 v5에 더한 타입이라 모르는 리더는 건너뛴다.
 * v6: ModelState 끝에 plan_yaw/plan_yaw_rate(264 B).
 * v7: ModelState 끝에 camera_offset_m/camera_height_m(8 B, 워프에 쓴 카메라 장착).
 * v8: ModelState 끝에 gas_press_probs/brake_press_probs(48 B, 모델 meta의 페달 예측).
 * v9: 파일 머리 매직을 EDGELOG1(이벤트 로그)·EDGEIDX1(프레임 인덱스)로 바꿨다. 레이아웃은 v8과
 *     같고, v8 이하 녹화는 예전 매직 그대로 읽는다. */
constexpr uint32_t kRecordingVersion = 9;

/* 파일 머리 매직. 리더는 v8 이하 녹화의 예전 매직(kLegacy*)도 받는다. */
constexpr char kEventLogMagic[8] = {'E', 'D', 'G', 'E', 'L', 'O', 'G', '1'};
constexpr char kFrameIndexMagic[8] = {'E', 'D', 'G', 'E', 'I', 'D', 'X', '1'};
constexpr char kLegacyEventLogMagic[8] = {'K', '2', '3', '0', 'L', 'O', 'G', '1'};
constexpr char kLegacyFrameIndexMagic[8] = {'K', '2', '3', '0', 'I', 'D', 'X', '1'};

inline bool is_event_log_magic(const char *magic)
{
  return std::memcmp(magic, kEventLogMagic, 8) == 0 || std::memcmp(magic, kLegacyEventLogMagic, 8) == 0;
}

inline bool is_frame_index_magic(const char *magic)
{
  return std::memcmp(magic, kFrameIndexMagic, 8) == 0 || std::memcmp(magic, kLegacyFrameIndexMagic, 8) == 0;
}

/* 세그먼트 영상은 H.264(segments/NNN/road.h264, manifest의 video_codec "h264")다. 보드 디코더가
 * H.264를 풀어 녹화를 그대로 리허설에 쓴다. */

enum class RecordType : uint16_t {
  CanRx = 1,
  CanTx = 2,
  ModelState = 3,
  ControlState = 4,
  PandaState = 5,
  LearnerState = 6,
  Imu = 7,
  Localization = 8,
};
constexpr uint16_t kLastRecordType = static_cast<uint16_t>(RecordType::Localization);

#pragma pack(push, 1)
struct EventFileHeader {
  char magic[8] = {'E', 'D', 'G', 'E', 'L', 'O', 'G', '1'};  // kEventLogMagic
  uint32_t version = kRecordingVersion;
  uint32_t header_size = sizeof(EventFileHeader);
  uint64_t route_start_ns = 0;
  uint64_t reserved = 0;
};

struct EventRecordHeader {
  uint64_t timestamp_ns = 0;
  uint16_t type = 0;
  uint16_t flags = 0;
  uint32_t payload_size = 0;
};

struct RecordedCanBatchHeader {
  uint32_t count = 0;
  uint32_t dropped = 0;
};

struct RecordedCanFrame {
  uint32_t address = 0;
  uint32_t src = 0;
  uint32_t bus_time = 0;
  uint32_t data_len = 0;
  uint32_t flags = 0;
  uint8_t data[64] = {};
};

struct FrameIndexHeader {
  char magic[8] = {'E', 'D', 'G', 'E', 'I', 'D', 'X', '1'};  // kFrameIndexMagic
  uint32_t version = kRecordingVersion;
  uint32_t header_size = sizeof(FrameIndexHeader);
  uint32_t record_size = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t fps = 0;
  uint64_t segment_start_ns = 0;
  uint64_t reserved = 0;
};

struct FrameIndexRecord {
  uint64_t frame_id = 0;
  uint64_t capture_timestamp_ns = 0;
  uint64_t encode_index = 0;
  uint64_t file_offset = 0;
  uint32_t packet_size = 0;
  uint32_t flags = 0;
};
#pragma pack(pop)

static_assert(sizeof(EventRecordHeader) == 16);
static_assert(sizeof(RecordedCanFrame) == 84);
static_assert(sizeof(FrameIndexRecord) == 40);
