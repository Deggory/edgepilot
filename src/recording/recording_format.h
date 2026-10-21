#pragma once

// 녹화 형식(recordd가 쓰고 재생 도구와 Python 리더가 읽는다): 이벤트 로그와 프레임 색인의 머리·레코드, 레코드 타입.

#include <cstdint>
#include <cstring>

// v14(2026-10-19): ModelState에서 고정 격자(model_t, 차선·도로 경계의 x)와 plan에서 나오는 lane_t, 상수였던
// best_plan·plan_probability, 읽는 곳이 없던 brake_press_probs를 뺐다.
// 그 전 녹화(v5~v13)는 모두 이 형식으로 다시 썼으므로(docs/recording.md) 리더는 이 버전만 읽는다.
constexpr uint32_t kRecordingVersion = 14;

constexpr char kEventLogMagic[8] = {'E', 'D', 'G', 'E', 'L', 'O', 'G', '1'};
constexpr char kFrameIndexMagic[8] = {'E', 'D', 'G', 'E', 'I', 'D', 'X', '1'};
constexpr uint32_t kFrameIndexKeyframe = 1u << 0;  // FrameIndexRecord.flags: 키프레임(IDR)

// 세그먼트 영상은 H.264(segments/NNN/road.h264, manifest의 video_codec "h264")다. 보드 디코더가
// H.264를 풀어 녹화를 그대로 리허설에 쓴다.

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
    uint32_t flags = 0;  // kFrameIndexKeyframe
};
#pragma pack(pop)

static_assert(sizeof(EventFileHeader) == 32);
static_assert(sizeof(EventRecordHeader) == 16);
static_assert(sizeof(FrameIndexHeader) == 48);
static_assert(sizeof(RecordedCanFrame) == 84);
static_assert(sizeof(FrameIndexRecord) == 40);
