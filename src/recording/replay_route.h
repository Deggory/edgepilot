#pragma once

// 녹화 route 읽기(replayd 리허설): 세그먼트마다 프레임 인덱스(frames.bin)로 영상(road.h264) 프레임
// 목록을 만들고(ReplayRoute), 이벤트 로그에서 고른 레코드를 시각 순으로 흘려 읽는다(ReplayEvents). 녹화의 카메라
// 지연은 ModelState에서 앞서 읽는다(RecordedCameraDelay).

#include "recording/recording_format.h"
#include "recording/time_ordered_events.h"

#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

// 세그먼트 하나의 프레임 색인(frames.bin): 영상 크기와 프레임마다 road.h264 안의 위치·크기·시각.
struct FrameIndex {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<FrameIndexRecord> records;
};

// seg_dir/frames.bin을 읽는다. 없거나 머리가 맞지 않거나(깨진 header_size 포함) 레코드가 없으면 false.
// 리허설(ReplayRoute)과 광류 보정 재생 도구가 같이 쓴다.
bool read_frame_index(const std::string &seg_dir, FrameIndex *out);

// 세그먼트 영상(road.h264)에서 프레임 하나를 디코더에 넣을 패킷으로 읽는다. 세그먼트 첫 프레임은 파일 처음(코덱
// 설정 포함)부터 읽고, 다른 키프레임 앞에는 파일 앞 header_bytes(첫 프레임 앞의 코덱 설정)를 붙인다.
bool read_h264_frame(FILE *file, uint64_t header_bytes, uint64_t offset, uint32_t size, bool keyframe,
                     bool segment_first, std::vector<uint8_t> *out);

class ReplayRoute {
public:
    // 영상 프레임 하나: 세그먼트 파일에서 읽을 위치와 녹화 시각.
    struct VideoFrame {
        size_t segment;
        uint64_t offset;
        uint32_t size;
        uint64_t capture_ns;
        bool keyframe;
        bool segment_first;
    };

    struct Segment {
        std::string video_path;
        uint64_t header_bytes;  // 첫 프레임 앞의 파라미터 세트(VPS/SPS/PPS)
        FILE *file = nullptr;
    };

    explicit ReplayRoute(const std::string &dir);
    ~ReplayRoute();
    ReplayRoute(const ReplayRoute &) = delete;
    ReplayRoute &operator=(const ReplayRoute &) = delete;

    // 프레임 하나를 디코더에 넣을 분량으로 읽는다. 키프레임에는 세그먼트 앞의 파라미터 세트를
    // 붙인다(세그먼트 첫 프레임은 파일 처음부터 읽어 이미 붙어 있다).
    bool read_frame(const VideoFrame &frame, std::vector<uint8_t> *out);

    unsigned width = 0, height = 0;
    std::vector<Segment> segments;
    std::vector<VideoFrame> frames;
};

// route에서 types 레코드를 from_ns부터 기록 시각 순으로 흘려 읽는다(TimeOrderedEvents, 0.5초 창). route 전체를
// 메모리에 올리지 않는다: 51분 route면 약 1.7 GB였고 보드 RAM은 447 MB라 클립을 손으로 잘라야 했다. from_ns보다
// kLeadNs 넘게 앞서 시작하는 청크 가운데 마지막 것부터 연다(그 앞 청크는 읽지 않는다). 끊긴 꼬리와 읽지 못한 청크는
// stderr에 알린다.
class ReplayEvents {
public:
    using Event = TimeOrderedEvents::Record;
    static constexpr uint64_t kLeadNs = 2'000'000'000ULL;

    ReplayEvents(const std::string &route_dir, uint64_t from_ns, std::initializer_list<RecordType> types);

    // 다음 이벤트. 없으면 nullptr. pop()까지 같은 이벤트다.
    const Event *peek();
    void pop() { have_ = false; }
    size_t chunks() const { return chunks_; }  // 읽을 청크 수

private:
    std::unique_ptr<TimeOrderedEvents> stream_;
    uint64_t from_ns_;
    uint32_t types_ = 0;  // 1 << RecordType
    size_t chunks_ = 0;
    Event current_;  // 스트림에서 옮겨 받는다(페이로드를 복사하지 않는다)
    bool have_ = false;
};

// 녹화에서 프레임이 센서 시각부터 프레임 링에 들어오기까지 걸린 시간. 낮에 약 24 ms, 밤에 약 43 ms(노출이
// 길다)로 프레임마다 다르다. modeld는 프레임을 받자마자 돌리므로 ModelState의 model_timestamp_ns − 촬영 시각 −
// model_execution_ms가 그 값이다(2026-10-19 보드에서 링 발행 시각과 0.5 ms 안으로 같았다). ModelState는 프레임보다
// 늦게 기록되므로 replayd의 이벤트와 따로 앞서 읽는다.
class RecordedCameraDelay {
public:
    RecordedCameraDelay(const std::string &route_dir, uint64_t from_ns)
        : models_(route_dir, from_ns, {RecordType::ModelState}) {}

    // 촬영 시각이 capture_ns인 프레임의 지연. 프레임은 촬영 시각 순으로 묻는다. 그 프레임의 ModelState가 없으면
    // 직전 값(처음이면 0)이다.
    uint64_t at(uint64_t capture_ns);

private:
    ReplayEvents models_;
    uint64_t delay_ns_ = 0;
};
