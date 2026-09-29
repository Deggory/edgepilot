#include "app_config.h"
#include "ipc_channels.h"
#include "maix_camera.h"
#include "maix_cmm.h"
#include "utils_process.h"
#include "utils_time.h"

#include <signal.h>

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <vector>

/* MaixCAM2 카메라를 프레임 링에 싣는다. 링 슬롯은 CMM 블록이고(헤더에 물리 주소),
 * 카메라 프레임은 IVPS 하드웨어 복사로 슬롯에 들어가므로 CPU가 픽셀을 만지지 않는다.
 * modeld(GDC)와 overlayd(IVPS)도 슬롯을 직접 읽는다. 센서 자체가 20 fps 자동 노출로
 * 돌아 모델이 기대하는 50 ms 간격 그대로 모든 프레임을 링에 쓴다. */

namespace {

volatile sig_atomic_t g_stop = 0;

} // namespace

int main()
{
    install_stop_signal_handlers(&g_stop);

    try {
        AppConfig config = AppConfig::from_env_defaults();
        LatestChannel frame_pub;
        FrameRing frame_ring;

        if (!frame_pub.open(kRoadAiFrameTopic, sizeof(RoadAiFrame), true))
            throw std::runtime_error("open roadAiFrame ipc failed");
        if (!frame_ring.open(true, config.nv12_width, config.nv12_height))
            throw std::runtime_error("open road ai frame ring failed");

        // 카메라가 AX SYS를 초기화한 뒤에 슬롯 CMM을 잡는다.
        MaixCamera camera(static_cast<int>(config.nv12_width), static_cast<int>(config.nv12_height),
                          20, true);
        std::vector<CmmBlock> slots(frame_ring.slot_count());
        for (unsigned i = 0; i < slots.size(); ++i) {
            if (!cmm_alloc(&slots[i], frame_ring.frame_bytes(), "frame_ring"))
                throw std::runtime_error("frame ring CMM alloc failed");
            frame_ring.set_slot_phys(i, slots[i].phys);
            frame_ring.attach_slot(i, slots[i].virt);
        }
        uint64_t frame_id = 0;
        unsigned errors = 0;
        uint64_t window_start = monotonic_now_ns();
        uint64_t window_frames = 0;
        /* 센서 캡처(VI 하드웨어 PTS)부터 프레임이 링에 들어오기까지의 지연. 캡처 시각을
         * 이만큼 앞당겨 발행한다. 10초마다 분포를 남긴다. */
        std::vector<uint64_t> ages_us, pop_ages_us;
        constexpr uint64_t kMaxCaptureAgeUs = 200000;  // 이보다 오래됐다고 나오면 PTS를 믿지 않는다
        uint64_t age_window_start = window_start;
        std::fprintf(stderr, "camerad: MaixCAM2 VI %ux%u NV12, 20 fps sensor, CMM ring\n",
                     config.nv12_width, config.nv12_height);

        while (!g_stop) {
            const unsigned slot = static_cast<unsigned>(frame_id % frame_ring.slot_count());
            frame_ring.begin_write(slot);
            uint64_t age_us = 0, pop_age_us = 0;
            const bool got = camera.read_to(slots[slot].phys, 1000, &age_us, &pop_age_us);
            /* 캡처 시각은 센서가 찍은 시각(VI 하드웨어 PTS)으로 둔다. openpilot처럼 plan 나이를
             * 센서 기준으로 재야 지연 보정이 맞는다. 링에 들어오기까지 약 24 ms가 걸려서, 여기서
             * 현재 시각을 찍으면 그만큼 plan이 새것으로 보였다. PTS를 못 읽으면 현재 시각이다. */
            uint64_t capture_ns = monotonic_now_ns();
            if (got && age_us > 0 && age_us < kMaxCaptureAgeUs) {
                capture_ns -= age_us * 1000ULL;
                ages_us.push_back(age_us);
                pop_ages_us.push_back(pop_age_us);
            }
            frame_ring.end_write(slot, got ? frame_id : UINT64_MAX);
            if (!got) {
                ++errors;
                continue;
            }
            RoadAiFrame msg;
            msg.frame_id = frame_id;
            msg.timestamp_ns = capture_ns;
            msg.slot = slot;
            msg.width = config.nv12_width;
            msg.height = config.nv12_height;
            msg.format = camera.fourcc();
            msg.crop_x = config.nv12_crop_x;
            msg.crop_y = config.nv12_crop_y;
            msg.crop_width = config.nv12_crop_width;
            msg.crop_height = config.nv12_crop_height;
            if (!frame_pub.publish(&msg, sizeof(msg))) ++errors;

            ++frame_id;
            ++window_frames;
            if (config.max_frames > 0 && frame_id >= config.max_frames) break;

            const uint64_t now = monotonic_now_ns();
            if (now - window_start >= 1000000000ULL) {
                std::fprintf(stderr, "camerad: fps=%.2f frames=%llu errors=%u          \r",
                             window_frames * 1e9 / (now - window_start),
                             static_cast<unsigned long long>(frame_id), errors);
                std::fflush(stderr);
                window_start = now;
                window_frames = 0;
            }
            if (now - age_window_start >= 10000000000ULL && !ages_us.empty()) {
                auto pct = [](std::vector<uint64_t> v, double q) {
                    std::sort(v.begin(), v.end());
                    return v[std::min(v.size() - 1, static_cast<size_t>(q * v.size()))] / 1000.0;
                };
                std::fprintf(stderr,
                             "\ncamerad: capture latency ms (HW PTS -> ring) p50 %.1f p95 %.1f max %.1f"
                             " | at VI pop p50 %.1f p95 %.1f (n=%zu)\n",
                             pct(ages_us, 0.5), pct(ages_us, 0.95), pct(ages_us, 1.0),
                             pct(pop_ages_us, 0.5), pct(pop_ages_us, 0.95), ages_us.size());
                ages_us.clear();
                pop_ages_us.clear();
                age_window_start = now;
            }
        }
        std::fprintf(stderr, "\ncamerad done frames=%llu errors=%u\n",
                     static_cast<unsigned long long>(frame_id), errors);
        // 소비자가 읽기 시작한 슬롯도 read_still_valid에서 걸리도록 무효로 표시한 뒤 푼다.
        for (unsigned i = 0; i < slots.size(); ++i) {
            frame_ring.begin_write(i);
            frame_ring.set_slot_phys(i, 0);
        }
        for (auto &block : slots) cmm_free(&block);
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "camerad error: %s\n", e.what());
        return 1;
    }
}
