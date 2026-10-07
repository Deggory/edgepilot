#include "recording/replay_route.h"

#include "recording/event_log_reader.h"

#include <dirent.h>

#include <algorithm>
#include <cctype>
#include <cstring>

namespace {

std::vector<std::string> numbered_entries(const std::string &dir, const char *suffix)
{
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

std::vector<uint8_t> read_file(const std::string &path)
{
    std::vector<uint8_t> data;
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) return data;
    std::fseek(f, 0, SEEK_END);
    data.resize(static_cast<size_t>(std::ftell(f)));
    std::fseek(f, 0, SEEK_SET);
    if (std::fread(data.data(), 1, data.size(), f) != data.size()) data.clear();
    std::fclose(f);
    return data;
}

}  // namespace

ReplayRoute::ReplayRoute(const std::string &dir)
{
    for (const std::string &seg_dir : numbered_entries(dir + "/segments", nullptr)) {
        const std::vector<uint8_t> index = read_file(seg_dir + "/frames.bin");
        if (index.size() < sizeof(FrameIndexHeader)) continue;
        FrameIndexHeader header;
        std::memcpy(&header, index.data(), sizeof(header));
        // header_size가 파일보다 크면(깨진 머리) 아래 레코드 수 계산이 size_t로 넘친다
        if (!is_frame_index_magic(header.magic) || header.record_size != sizeof(FrameIndexRecord) ||
            header.header_size < sizeof(FrameIndexHeader) || header.header_size > index.size())
            continue;
        width = header.width;
        height = header.height;
        const size_t count = (index.size() - header.header_size) / header.record_size;
        if (count == 0) continue;
        Segment segment;
        segment.video_path = seg_dir + "/road.h264";
        for (size_t i = 0; i < count; ++i) {
            FrameIndexRecord r;
            std::memcpy(&r, index.data() + header.header_size + i * sizeof(r), sizeof(r));
            if (i == 0) segment.header_bytes = r.file_offset;
            frames.push_back({segments.size(), r.file_offset, r.packet_size, r.capture_timestamp_ns,
                              (r.flags & 1U) != 0, i == 0});
        }
        segments.push_back(segment);
    }
    std::vector<char> payload;
    for (const std::string &path : numbered_entries(dir + "/events", ".bin")) {
        EventLogReader reader(path);
        EventRecordHeader h{};
        while (reader.next(&h, &payload)) {
            const auto type = static_cast<RecordType>(h.type);
            if (type == RecordType::CanRx || type == RecordType::PandaState)
                events.push_back({h.timestamp_ns, type, std::vector<uint8_t>(payload.begin(), payload.end())});
        }
    }
    std::stable_sort(events.begin(), events.end(),
                     [](const Event &a, const Event &b) { return a.timestamp_ns < b.timestamp_ns; });
}
ReplayRoute::~ReplayRoute()
{
    for (Segment &s : segments)
        if (s.file) std::fclose(s.file);
}

bool ReplayRoute::read_frame(const VideoFrame &frame, std::vector<uint8_t> *out)
{
    Segment &s = segments[frame.segment];
    if (!s.file && !(s.file = std::fopen(s.video_path.c_str(), "rb"))) return false;
    const uint64_t begin = frame.keyframe ? (frame.segment_first ? 0 : frame.offset) : frame.offset;
    const uint64_t prefix = frame.keyframe && !frame.segment_first ? s.header_bytes : 0;
    out->resize(prefix + (frame.offset + frame.size - begin));
    if (prefix) {
        std::fseek(s.file, 0, SEEK_SET);
        if (std::fread(out->data(), 1, prefix, s.file) != prefix) return false;
    }
    std::fseek(s.file, static_cast<long>(begin), SEEK_SET);
    return std::fread(out->data() + prefix, 1, out->size() - prefix, s.file) == out->size() - prefix;
}
