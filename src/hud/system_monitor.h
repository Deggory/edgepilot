#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

/* /proc CPU/메모리/저장소, thermal zone 온도, 네트워크 상태를 읽어
 * OverlayHudState에 채운다. overlayd가 1 Hz로 호출한다. */

#include "hud/overlay_state.h"

#include <cstdint>

class SystemMonitor {
public:
    void sample(OverlayHudState *hud);

private:
    void sample_cpu(float *percent);
    void sample_memory(float *percent);
    void sample_storage(float *percent);
    void sample_temperature(float *temperature_c);
    void sample_network(OverlayHudState *hud);

    uint64_t previous_total_ = 0;
    uint64_t previous_idle_ = 0;
};

#endif  // SYSTEM_MONITOR_H
