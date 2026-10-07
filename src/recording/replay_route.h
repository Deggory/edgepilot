#pragma once

/* 녹화 route 읽기(replayd 리허설): 세그먼트마다 프레임 인덱스(frames.bin)로 영상(road.h264) 프레임
 * 목록을 만들고, 이벤트 로그에서 CanRx와 PandaState를 시각 순으로 모은다. */

#include "recording/recording_format.h"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

class ReplayRoute {
public:
    /* 영상 프레임 하나: 세그먼트 파일에서 읽을 위치와 녹화 시각. */
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

    struct Event {
        uint64_t timestamp_ns;
        RecordType type;
        std::vector<uint8_t> payload;
    };

    explicit ReplayRoute(const std::string &dir);
    ~ReplayRoute();
    ReplayRoute(const ReplayRoute &) = delete;
    ReplayRoute &operator=(const ReplayRoute &) = delete;

    /* 프레임 하나를 디코더에 넣을 분량으로 읽는다. 키프레임에는 세그먼트 앞의 파라미터 세트를
     * 붙인다(세그먼트 첫 프레임은 파일 처음부터 읽어 이미 붙어 있다). */
    bool read_frame(const VideoFrame &frame, std::vector<uint8_t> *out);

    unsigned width = 0, height = 0;
    std::vector<Segment> segments;
    std::vector<VideoFrame> frames;
    std::vector<Event> events;
};
