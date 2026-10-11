/* 광류 보정기를 녹화 구간 하나에 돌린다(보드 전용). VDEC로 디코딩한 Y 평면을 CMM에 실어 modeld와 같은
 * FlowWindowTracker(IVE 피라미드·코너·LK, 풀이, 블록 집계)에 넘기고, 받아들인 창마다 한 줄과 끝의 추정을
 * 출력한다. IVPS를 쓰지 않으므로 edgepilot이 도는 중에도 된다.
 *
 * 사용: flow_calibration_replay <segment_dir> [--open frames.txt]
 *   --open  조건이 열린 프레임 번호(frames.bin 순서) 목록. 없으면 모든 프레임을 곧은 길로 본다.
 * 출력: "window <끝 프레임> <yaw°> <pitch°> <회전°> <점> <맞는 점> <추정 yaw°> <추정 pitch°> <블록>" 줄들과
 *       "summary ..." 한 줄. */
#include "common/app_config.h"
#include "maixcam2/maix_cmm.h"
#include "maixcam2/maix_vdec.h"
#include "model/flow_calibration_worker.h"
#include "recording/recording_format.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

namespace {

constexpr double kDeg = 57.29577951308232;

std::vector<uint8_t> read_file(const std::string &path)
{
    std::vector<uint8_t> d;
    if (FILE *f = std::fopen(path.c_str(), "rb")) {
        std::fseek(f, 0, SEEK_END);
        d.resize(static_cast<size_t>(std::ftell(f)));
        std::fseek(f, 0, SEEK_SET);
        if (std::fread(d.data(), 1, d.size(), f) != d.size()) d.clear();
        std::fclose(f);
    }
    return d;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <segment_dir> [--open frames.txt]\n", argv[0]);
        return 2;
    }
    const std::string seg = argv[1];
    std::set<long> open_frames;
    bool all_open = true;
    for (int i = 2; i + 1 < argc; i += 2) {
        if (std::string(argv[i]) == "--open") {
            all_open = false;
            if (FILE *f = std::fopen(argv[i + 1], "r")) {
                long v;
                while (std::fscanf(f, "%ld", &v) == 1) open_frames.insert(v);
                std::fclose(f);
            }
        }
    }
    const std::vector<uint8_t> index = read_file(seg + "/frames.bin");
    FrameIndexHeader header;
    if (index.size() < sizeof(header)) {
        std::fprintf(stderr, "no frames.bin in %s\n", seg.c_str());
        return 1;
    }
    std::memcpy(&header, index.data(), sizeof(header));
    if (!is_frame_index_magic(header.magic) || header.record_size != sizeof(FrameIndexRecord) ||
        header.header_size > index.size()) {
        std::fprintf(stderr, "bad frames.bin\n");
        return 1;
    }
    const size_t count = (index.size() - header.header_size) / sizeof(FrameIndexRecord);
    std::vector<FrameIndexRecord> recs(count);
    std::memcpy(recs.data(), index.data() + header.header_size, count * sizeof(FrameIndexRecord));
    FILE *video = std::fopen((seg + "/road.h264").c_str(), "rb");
    if (!video || count == 0) {
        std::fprintf(stderr, "no road.h264\n");
        return 1;
    }
    const int w = static_cast<int>(header.width), h = static_cast<int>(header.height);
    const uint64_t prefix_bytes = recs[0].file_offset;  // 첫 프레임 앞의 파라미터 세트

    VideoDecoder decoder(w, h);  // AX_SYS_Init
    const AppConfig config = AppConfig::from_env_defaults();
    const FlowIntrinsics k{config.input_warp_fx, config.input_warp_fy, config.input_warp_cx, config.input_warp_cy};
    FlowWindowTracker tracker(k, w, h);
    /* 원본 단은 복사하지 않으므로(LK가 이전 프레임을 그대로 읽는다) 링처럼 버퍼 둘을 번갈아 쓴다. NV12로 잡고
     * UV는 회색으로 둔다(VPP가 NV12를 읽는다). */
    CmmBlock slots[2];
    for (CmmBlock &b : slots) {
        if (!cmm_alloc(&b, static_cast<size_t>(w) * h * 3 / 2, "flow_replay")) {
            std::fprintf(stderr, "CMM alloc failed\n");
            return 1;
        }
        std::memset(b.virt + static_cast<size_t>(w) * h, 128, static_cast<size_t>(w) * h / 2);
    }

    std::vector<uint8_t> packet;
    size_t sent = 0, got = 0;
    uint32_t windows = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (got < count) {
        bool have = false;
        const auto wait0 = std::chrono::steady_clock::now();
        while (!have) {
            if (sent < count && sent < got + 24) {
                const FrameIndexRecord &r = recs[sent];
                const bool key = (r.flags & 1U) != 0;
                const uint64_t begin = key ? (sent == 0 ? 0 : r.file_offset) : r.file_offset;
                const uint64_t prefix = key && sent != 0 ? prefix_bytes : 0;
                packet.resize(prefix + (r.file_offset + r.packet_size - begin));
                bool ok = true;
                if (prefix) ok = std::fseek(video, 0, SEEK_SET) == 0 && std::fread(packet.data(), 1, prefix, video) == prefix;
                ok = ok && std::fseek(video, static_cast<long>(begin), SEEK_SET) == 0 &&
                     std::fread(packet.data() + prefix, 1, packet.size() - prefix, video) == packet.size() - prefix;
                if (!ok || !decoder.send(packet.data(), packet.size(), r.capture_timestamp_ns, 200)) {
                    std::fprintf(stderr, "feeding frame %zu failed\n", sent);
                    return 1;
                }
                ++sent;
            }
            have = decoder.receive_luma(slots[got % 2].virt, w, nullptr, sent < count ? 5 : 200);
            if (!have && std::chrono::steady_clock::now() - wait0 > std::chrono::seconds(5)) {
                std::fprintf(stderr, "decoder stalled at frame %zu of %zu\n", got, count);
                return 1;
            }
        }
        const long frame = static_cast<long>(got++);
        FlowGateInput gate;  // 녹화의 ControlState 대신: 열린 프레임은 곧은 고속 주행으로 둔다
        if (all_open || open_frames.count(frame)) {
            gate.v_ego = 25.0f;
            gate.vehicle_fresh = true;
        }
        tracker.process(slots[frame % 2].phys, w, static_cast<uint64_t>(frame), gate, nullptr);
        const FlowCalibrationState st = tracker.calibrator().state();
        if (st.windows != windows) {
            windows = st.windows;
            std::printf("window %llu %+.4f %+.4f %.4f %u %u %+.4f %+.4f %u\n",
                        static_cast<unsigned long long>(st.window_frame_id), st.window_yaw * kDeg,
                        st.window_pitch * kDeg, st.window_rotation * kDeg, st.window_tracks, st.window_inliers,
                        st.yaw * kDeg, st.pitch * kDeg, st.blocks);
        }
    }
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const FlowWindowTracker::Stats &s = tracker.stats();
    const FlowCalibrationState st = tracker.calibrator().state();
    const double frames = s.frames ? static_cast<double>(s.frames) : 1.0;
    std::printf("summary frames %zu ive_frames %llu windows %u done %llu lost %llu rejected %llu | %.2f ms wall %.3f ms cpu "
                "per IVE frame, pyramid %s | estimate status %u blocks %u yaw %+.4f pitch %+.4f | %.1f s\n",
                count, static_cast<unsigned long long>(s.frames), st.windows,
                static_cast<unsigned long long>(s.windows_done), static_cast<unsigned long long>(s.windows_lost),
                static_cast<unsigned long long>(s.rejected), s.wall_ms / frames, s.cpu_ms / frames,
                tracker.pyramid_engine(), st.status,
                st.blocks, st.yaw * kDeg, st.pitch * kDeg, seconds);
    for (CmmBlock &b : slots) cmm_free(&b);
    std::fclose(video);
    return 0;
}
