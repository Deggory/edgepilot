/* 실차 전 리허설: 녹화 route를 카메라와 판다처럼 실시간으로 재생한다. camerad·pandad
 * 자리에서 돌고(manager.py EDGEPILOT_REPLAY_ROUTE), modeld·controlsd·overlayd·recordd는
 * 차에서와 똑같이 돈다.
 *  - 영상: segments/NNN/road.h264(AX630C VDEC는 H.264만 디코딩한다; 녹화의 HEVC는
 *    tools/rehearsal/transcode_route.py로 바꿔 둔다)를 프레임 인덱스로 한 프레임씩 VDEC에
 *    넣고, 디코딩된 프레임을 IVPS로 프레임 링 슬롯(CMM)에 복사해 roadAiFrame을 낸다.
 *  - CAN·판다 상태: events/NNN.bin의 CanRx와 PandaState를 같은 시간축으로 판다 채널에
 *    낸다(pandad처럼 CAN은 녹화용 로그 큐에도 넣는다). controlsd의 송신 요청은 보내지
 *    않고 송신 로그로만 넘긴다(recordd가 기록한다).
 * 타임스탬프는 모두 지금 시각으로 바꾼다. 녹화 시각은 재생 간격을 정하는 데만 쓴다.
 * 사용: replayd <route> [start_s] [duration_s]   (duration 0 = 끝까지) */
#include "app_config.h"
#include "event_log_reader.h"
#include "ipc_channels.h"
#include "ipc_messages.h"
#include "maix_cmm.h"
#include "maix_vdec.h"
#include "recording_format.h"
#include "utils_process.h"
#include "utils_time.h"

#include <dirent.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

volatile sig_atomic_t g_stop = 0;

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

class Route {
public:
    explicit Route(const std::string &dir)
    {
        for (const std::string &seg_dir : numbered_entries(dir + "/segments", nullptr)) {
            const std::vector<uint8_t> index = read_file(seg_dir + "/frames.bin");
            if (index.size() < sizeof(FrameIndexHeader)) continue;
            FrameIndexHeader header;
            std::memcpy(&header, index.data(), sizeof(header));
            if (std::memcmp(header.magic, "K230IDX1", 8) != 0 || header.record_size != sizeof(FrameIndexRecord))
                continue;
            width = header.width;
            height = header.height;
            const size_t count = (index.size() - header.header_size) / header.record_size;
            if (count == 0) continue;
            Segment segment;
            // AX630C 디코더는 H.264만 받으므로 tools/rehearsal/transcode_route.py가 만든
            // road.h264를 먼저 쓴다. 없으면 원본 road.hevc(디코더가 지원하면).
            segment.video_path = seg_dir + "/road.h264";
            if (access(segment.video_path.c_str(), R_OK) == 0) {
                h264 = true;
            } else {
                segment.video_path = seg_dir + "/road.hevc";
            }
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
    ~Route()
    {
        for (Segment &s : segments)
            if (s.file) std::fclose(s.file);
    }

    /* 세그먼트 첫 프레임이면 파라미터 세트를 앞에 붙여 한 프레임 분량을 읽는다. */
    bool read_frame(const VideoFrame &frame, std::vector<uint8_t> *out)
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

    unsigned width = 0, height = 0;
    bool h264 = false;
    std::vector<Segment> segments;
    std::vector<VideoFrame> frames;
    std::vector<Event> events;
};

CanBatch can_batch_from_record(const std::vector<uint8_t> &payload, uint64_t now_ns)
{
    CanBatch batch;
    if (payload.size() < sizeof(RecordedCanBatchHeader)) return batch;
    RecordedCanBatchHeader header;
    std::memcpy(&header, payload.data(), sizeof(header));
    const uint32_t count = std::min<uint32_t>(
        {header.count, kCanBatchMaxFrames,
         static_cast<uint32_t>((payload.size() - sizeof(header)) / sizeof(RecordedCanFrame))});
    batch.timestamp_ns = now_ns;
    batch.valid = 1;
    batch.count = count;
    batch.dropped = header.dropped;
    for (uint32_t i = 0; i < count; ++i) {
        RecordedCanFrame f;
        std::memcpy(&f, payload.data() + sizeof(header) + i * sizeof(f), sizeof(f));
        IpcCanFrame &out = batch.frames[i];
        out.address = f.address;
        out.src = f.src;
        out.bus_time = f.bus_time;
        out.data_len = f.data_len;
        out.flags = f.flags;
        std::memcpy(out.data, f.data, sizeof(out.data));
    }
    return batch;
}

} // namespace

int main(int argc, char *argv[])
{
    install_stop_signal_handlers(&g_stop);
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <route> [start_s] [duration_s]\n", argv[0]);
        return 2;
    }
    try {
        const std::string route_dir = argv[1];
        const double start_s = argc > 2 ? std::atof(argv[2]) : 0.0;
        const double duration_s = argc > 3 ? std::atof(argv[3]) : 0.0;
        Route route(route_dir);
        if (route.frames.empty()) throw std::runtime_error("no video frames in " + route_dir);
        const uint64_t route_start = route.frames.front().capture_ns;
        const uint64_t want = route_start + static_cast<uint64_t>(start_s * 1e9);
        size_t first = 0;
        while (first < route.frames.size() &&
               (route.frames[first].capture_ns < want || !route.frames[first].keyframe))
            ++first;
        if (first == route.frames.size()) throw std::runtime_error("no keyframe after start");
        const uint64_t t0 = route.frames[first].capture_ns;
        const uint64_t t_end = duration_s > 0 ? t0 + static_cast<uint64_t>(duration_s * 1e9) : UINT64_MAX;
        size_t next_event = 0;
        while (next_event < route.events.size() && route.events[next_event].timestamp_ns < t0) ++next_event;

        LatestChannel frame_pub, panda_pub;
        FrameRing ring;
        CanQueue can_pub, can_log_pub, sendcan_sub, sendcan_log_pub;
        if (!frame_pub.open(kRoadAiFrameTopic, sizeof(RoadAiFrame), true) ||
            !panda_pub.open(kPandaStateTopic, sizeof(PandaState), true) ||
            !can_pub.open(kCanTopic, kCanQueueSlots, true) ||
            !can_log_pub.open(kCanLogTopic, kCanQueueSlots, true) ||
            !sendcan_sub.open(kSendCanTopic, kCanQueueSlots, true) ||
            !sendcan_log_pub.open(kSendCanLogTopic, kCanQueueSlots, true))
            throw std::runtime_error("open ipc failed");
        if (!ring.open(true, route.width, route.height))
            throw std::runtime_error("open frame ring failed");

        VideoDecoder decoder(route.h264 ? VideoDecoder::Codec::H264 : VideoDecoder::Codec::HEVC,
                             static_cast<int>(route.width),
                             static_cast<int>(route.height));
        std::vector<CmmBlock> slots(ring.slot_count());
        for (unsigned i = 0; i < slots.size(); ++i) {
            if (!cmm_alloc(&slots[i], ring.frame_bytes(), "replay_ring"))
                throw std::runtime_error("frame ring CMM alloc failed");
            ring.set_slot_phys(i, slots[i].phys);
            ring.attach_slot(i, slots[i].virt);
        }
        std::fprintf(stderr, "replayd: %s from %.1f s (%zu frames, %zu CAN/panda events), %ux%u\n",
                     route_dir.c_str(), (t0 - route_start) / 1e9, route.frames.size() - first,
                     route.events.size() - next_event, route.width, route.height);

        const uint64_t base = monotonic_now_ns() + 200000000ULL;  // 소비자가 붙을 여유
        auto due = [&](uint64_t recorded_ns) { return base + (recorded_ns - t0); };
        std::vector<uint8_t> packet;
        uint64_t frame_id = 0, sent = 0, decode_errors = 0, can_batches = 0, tx_batches = 0;
        uint64_t dropped_outputs = 0, last_output_ns = monotonic_now_ns();
        size_t next_frame = first;
        uint64_t window = monotonic_now_ns(), window_frames = 0;
        while (!g_stop) {
            const uint64_t now = monotonic_now_ns();
            // 영상: 녹화 간격대로 한 프레임씩 넣고, 나온 프레임을 곧바로 링에 싣는다.
            if (next_frame < route.frames.size() && route.frames[next_frame].capture_ns < t_end &&
                now >= due(route.frames[next_frame].capture_ns)) {
                const VideoFrame &f = route.frames[next_frame++];
                if (route.read_frame(f, &packet) && decoder.send(packet.data(), packet.size(), f.capture_ns))
                    ++sent;
                else
                    ++decode_errors;
            }
            // 넣은 프레임이 아직 다 나오지 않았으면 다음 슬롯에 받는다.
            if (sent > frame_id + dropped_outputs) {
                const unsigned slot = static_cast<unsigned>(frame_id % ring.slot_count());
                ring.begin_write(slot);
                uint64_t pts = 0;
                if (decoder.receive_to(slots[slot].phys, &pts, 0)) {
                    ring.end_write(slot, frame_id);
                    RoadAiFrame meta;
                    meta.frame_id = frame_id++;
                    meta.timestamp_ns = monotonic_now_ns();
                    meta.slot = slot;
                    meta.width = route.width;
                    meta.height = route.height;
                    meta.format = V4L2_PIX_FMT_NV12;
                    frame_pub.publish(&meta, sizeof(meta));
                    ++window_frames;
                    last_output_ns = now;
                } else {
                    ring.end_write(slot, UINT64_MAX);
                    if (now - last_output_ns > 1000000000ULL) {  // 디코더가 삼킨 프레임
                        ++dropped_outputs;
                        last_output_ns = now;
                    }
                }
            }
            // CAN·판다 상태
            while (next_event < route.events.size() && route.events[next_event].timestamp_ns < t_end &&
                   now >= due(route.events[next_event].timestamp_ns)) {
                const Event &e = route.events[next_event++];
                if (e.type == RecordType::CanRx) {
                    const CanBatch batch = can_batch_from_record(e.payload, now);
                    can_pub.push(batch);
                    can_log_pub.push(batch);
                    ++can_batches;
                } else if (e.payload.size() == sizeof(PandaState)) {
                    PandaState state;
                    std::memcpy(&state, e.payload.data(), sizeof(state));
                    state.timestamp_ns = now;
                    panda_pub.publish(&state, sizeof(state));
                }
            }
            // controlsd의 송신 요청: 보내지 않고 송신 로그로만.
            CanBatch tx;
            while (sendcan_sub.pop(&tx)) {
                tx.timestamp_ns = now;
                sendcan_log_pub.push(tx);
                ++tx_batches;
            }
            const bool done = (next_frame >= route.frames.size() || route.frames[next_frame].capture_ns >= t_end) &&
                              sent <= frame_id + dropped_outputs;
            if (now - window >= 1000000000ULL) {
                std::fprintf(stderr, "replayd: fps=%.2f t=%.1fs frames=%llu errors=%llu can=%llu tx=%llu          \r",
                             window_frames * 1e9 / (now - window), (now - base) / 1e9,
                             static_cast<unsigned long long>(frame_id),
                             static_cast<unsigned long long>(decode_errors),
                             static_cast<unsigned long long>(can_batches),
                             static_cast<unsigned long long>(tx_batches));
                std::fflush(stderr);
                window = now;
                window_frames = 0;
            }
            if (done) break;
            usleep(1000);
        }
        std::fprintf(stderr, "\nreplayd: done frames=%llu errors=%llu can=%llu tx=%llu\n",
                     static_cast<unsigned long long>(frame_id), static_cast<unsigned long long>(decode_errors),
                     static_cast<unsigned long long>(can_batches), static_cast<unsigned long long>(tx_batches));
        for (unsigned i = 0; i < slots.size(); ++i) {
            ring.begin_write(i);
            ring.set_slot_phys(i, 0);
        }
        for (auto &block : slots) cmm_free(&block);
        // 끝나면 매니저가 다시 띄우지 않도록 기다린다(리허설 결과를 볼 수 있게).
        while (!g_stop) usleep(200000);
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "replayd error: %s\n", e.what());
        return 1;
    }
}
