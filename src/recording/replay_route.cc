#include "recording/replay_route.h"

#include "common/ipc_messages.h"
#include "common/utils_file.h"
#include "recording/event_log_reader.h"

#include <dirent.h>

#include <algorithm>
#include <cctype>
#include <cstring>

namespace {

std::vector<std::string> numbered_entries(const std::string &dir, const char *suffix) {
    std::vector<std::string> names;
    if (DIR *d = opendir(dir.c_str())) {
        while (dirent *e = readdir(d)) {
            const std::string name = e->d_name;
            if (name.size() >= 3 && std::isdigit(static_cast<unsigned char>(name[0])) &&
                (!suffix || (name.size() > std::strlen(suffix) &&
                             name.compare(name.size() - std::strlen(suffix), std::string::npos, suffix) == 0)))
                names.push_back(dir + "/" + name);
        }
        closedir(d);
    }
    std::sort(names.begin(), names.end());
    return names;
}

}  // namespace

bool read_frame_index(const std::string &seg_dir, FrameIndex *out) {
    const std::string index = read_text_file(seg_dir + "/frames.bin");
    if (index.size() < sizeof(FrameIndexHeader)) return false;
    FrameIndexHeader header;
    std::memcpy(&header, index.data(), sizeof(header));
    // header_size가 파일보다 크면(깨진 머리) 아래 레코드 수 계산이 size_t로 넘친다
    if (std::memcmp(header.magic, kFrameIndexMagic, 8) != 0 || header.version != kRecordingVersion ||
        header.record_size != sizeof(FrameIndexRecord) || header.header_size < sizeof(FrameIndexHeader) ||
        header.header_size > index.size())
        return false;
    const size_t count = (index.size() - header.header_size) / sizeof(FrameIndexRecord);
    if (count == 0) return false;
    out->width = header.width;
    out->height = header.height;
    out->records.resize(count);
    std::memcpy(out->records.data(), index.data() + header.header_size, count * sizeof(FrameIndexRecord));
    return true;
}

ReplayRoute::ReplayRoute(const std::string &dir) {
    for (const std::string &seg_dir : numbered_entries(dir + "/segments", nullptr)) {
        FrameIndex index;
        if (!read_frame_index(seg_dir, &index)) continue;
        width = index.width;
        height = index.height;
        Segment segment;
        segment.video_path = seg_dir + "/road.h264";
        for (size_t i = 0; i < index.records.size(); ++i) {
            const FrameIndexRecord &r = index.records[i];
            if (i == 0) segment.header_bytes = r.file_offset;
            frames.push_back({segments.size(), r.file_offset, r.packet_size, r.capture_timestamp_ns,
                              (r.flags & kFrameIndexKeyframe) != 0, i == 0});
        }
        segments.push_back(segment);
    }
}
ReplayRoute::~ReplayRoute() {
    for (Segment &s : segments)
        if (s.file) std::fclose(s.file);
}

ReplayEvents::ReplayEvents(const std::string &route_dir, uint64_t from_ns, std::initializer_list<RecordType> types)
    : from_ns_(from_ns) {
    for (const RecordType type : types) types_ |= 1u << static_cast<unsigned>(type);
    std::vector<std::string> paths = numbered_entries(route_dir + "/events", ".bin");
    // 청크는 시각 순이다. 첫 레코드가 from_ns - kLeadNs 이전인 마지막 청크부터 읽는다: 레코드는 파일에 늦어도
    // 0.5초 안에 쓰이므로 그 앞 청크에 from_ns 이후의 레코드는 없다.
    size_t first = 0;
    EventRecordHeader header{};
    std::vector<char> payload;
    for (size_t i = 0; i < paths.size(); ++i) {
        EventLogReader reader(paths[i]);
        if (!reader.next(&header, &payload)) continue;
        if (header.timestamp_ns + kLeadNs > from_ns) break;
        first = i;
    }
    paths.erase(paths.begin(), paths.begin() + static_cast<std::ptrdiff_t>(first));
    chunks_ = paths.size();
    stream_ = std::make_unique<TimeOrderedEvents>(std::move(paths));
    // 끊긴 꼬리와 읽지 못한 청크를 알린다(진행 표시 줄이 \r로 끝나므로 새 줄에서)
    stream_->on_file_end = [](const std::string &path, const EventLogReader &reader) {
        report_event_file_end(path, reader, "\nreplayd: ");
    };
}

const ReplayEvents::Event *ReplayEvents::peek() {
    while (!have_ && stream_->next(&current_)) {
        have_ = (types_ >> current_.header.type & 1u) && current_.header.timestamp_ns >= from_ns_;
    }
    return have_ ? &current_ : nullptr;
}

uint64_t RecordedCameraDelay::at(uint64_t capture_ns) {
    for (const ReplayEvents::Event *e = models_.peek(); e; e = models_.peek()) {
        ModelState model;
        if (read_record(e->payload, &model)) {
            if (model.capture_timestamp_ns > capture_ns) break;
            // 촬영 시각을 링에 넣을 때 찍던 2026-09-25~27 녹화는 0에 가깝다(가장 작은 값 0.03 ms)
            if (model.capture_timestamp_ns == capture_ns)
                delay_ns_ = model.model_timestamp_ns - model.capture_timestamp_ns -
                            static_cast<uint64_t>(model.model_execution_ms * 1e6);
        }
        models_.pop();
    }
    return delay_ns_;
}

bool read_h264_frame(FILE *file, uint64_t header_bytes, uint64_t offset, uint32_t size, bool keyframe,
                     bool segment_first, std::vector<uint8_t> *out) {
    const uint64_t begin = keyframe && segment_first ? 0 : offset;
    const uint64_t prefix = keyframe && !segment_first ? header_bytes : 0;
    out->resize(prefix + (offset + size - begin));
    if (prefix) {
        std::fseek(file, 0, SEEK_SET);
        if (std::fread(out->data(), 1, prefix, file) != prefix) return false;
    }
    std::fseek(file, static_cast<long>(begin), SEEK_SET);
    return std::fread(out->data() + prefix, 1, out->size() - prefix, file) == out->size() - prefix;
}

bool ReplayRoute::read_frame(const VideoFrame &frame, std::vector<uint8_t> *out) {
    Segment &s = segments[frame.segment];
    if (!s.file && !(s.file = std::fopen(s.video_path.c_str(), "rb"))) return false;
    return read_h264_frame(s.file, s.header_bytes, frame.offset, frame.size, frame.keyframe, frame.segment_first, out);
}
