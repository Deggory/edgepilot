#include "app_config.h"
#include "ipc_channels.h"
#include "maix_camera.h"
#include "maix_cmm.h"
#include "utils_process.h"
#include "utils_time.h"

#include <signal.h>

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
        std::fprintf(stderr, "camerad: MaixCAM2 VI %ux%u NV12, 20 fps sensor, CMM ring\n",
                     config.nv12_width, config.nv12_height);

        while (!g_stop) {
            const unsigned slot = static_cast<unsigned>(frame_id % frame_ring.slot_count());
            frame_ring.begin_write(slot);
            const bool got = camera.read_to(slots[slot].phys);
            const uint64_t capture_ns = monotonic_now_ns();
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
