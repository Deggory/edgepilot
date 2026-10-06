#include "hud/system_monitor.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <linux/wireless.h>  // net/if.h 다음이어야 struct ifreq가 겹치지 않는다
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

/* 접속한 와이파이 이름. 무선 확장 ioctl(SIOCGIWESSID)이라 wpa_cli를 띄우지 않는다. HUD 글꼴에
 * 없는 바이트(UTF-8 등)는 '?'로 바꾼다. */
void read_ssid(const char *interface_name, char *ssid, size_t size)
{
    ssid[0] = '\0';
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return;
    char essid[IW_ESSID_MAX_SIZE + 1] = {};
    iwreq request {};
    std::snprintf(request.ifr_ifrn.ifrn_name, sizeof(request.ifr_ifrn.ifrn_name), "%s", interface_name);
    request.u.essid.pointer = essid;
    request.u.essid.length = IW_ESSID_MAX_SIZE;
    if (ioctl(fd, SIOCGIWESSID, &request) == 0) {
        const size_t length = std::min<size_t>(request.u.essid.length, size - 1);
        for (size_t i = 0; i < length; ++i)
            ssid[i] = essid[i] >= 0x20 && essid[i] < 0x7f ? essid[i] : '?';
        ssid[length] = '\0';
    }
    close(fd);
}

}  // namespace

void SystemMonitor::sample(OverlayHudState *hud)
{
    if (!hud) return;
    sample_cpu(&hud->cpu_percent);
    sample_memory(&hud->memory_percent);
    sample_storage(&hud->storage_percent);
    sample_temperature(&hud->cpu_temp_c);
    sample_network(hud);
}

void SystemMonitor::sample_cpu(float *percent)
{
    FILE *file = std::fopen("/proc/stat", "r");
    if (!file) return;
    unsigned long long user = 0, nice = 0, system = 0, idle = 0;
    unsigned long long iowait = 0, irq = 0, softirq = 0, steal = 0;
    const int count = std::fscanf(file, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                                  &user, &nice, &system, &idle, &iowait,
                                  &irq, &softirq, &steal);
    std::fclose(file);
    if (count < 4) return;

    const uint64_t idle_total = idle + iowait;
    const uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
    if (previous_total_ != 0 && total > previous_total_) {
        const uint64_t total_delta = total - previous_total_;
        const uint64_t idle_delta = idle_total - previous_idle_;
        *percent = static_cast<float>(100.0 *
            (1.0 - static_cast<double>(std::min(idle_delta, total_delta)) / total_delta));
    }
    previous_total_ = total;
    previous_idle_ = idle_total;
}

void SystemMonitor::sample_memory(float *percent)
{
    FILE *file = std::fopen("/proc/meminfo", "r");
    if (!file) return;
    unsigned long long total_kb = 0;
    unsigned long long available_kb = 0;
    char line[128];
    while (std::fgets(line, sizeof(line), file)) {
        std::sscanf(line, "MemTotal: %llu kB", &total_kb);
        std::sscanf(line, "MemAvailable: %llu kB", &available_kb);
    }
    std::fclose(file);
    if (total_kb > 0) {
        *percent = static_cast<float>(100.0 *
            (1.0 - static_cast<double>(std::min(available_kb, total_kb)) / total_kb));
    }
}

void SystemMonitor::sample_storage(float *percent)
{
    struct statvfs space {};
    if (statvfs("/", &space) != 0 || space.f_blocks == 0) return;
    const uint64_t available = space.f_bavail;
    const uint64_t total = space.f_blocks;
    *percent = static_cast<float>(100.0 *
        (1.0 - static_cast<double>(std::min(available, total)) / total));
}

void SystemMonitor::sample_temperature(float *temperature_c)
{
    float maximum = 0.0f;
    for (int i = 0; i < 16; ++i) {
        char path[96];
        std::snprintf(path, sizeof(path), "/sys/class/thermal/thermal_zone%d/temp", i);
        FILE *file = std::fopen(path, "r");
        if (!file) continue;
        float value = 0.0f;
        const bool read = std::fscanf(file, "%f", &value) == 1;
        std::fclose(file);
        if (!read) continue;
        if (value > 1000.0f) value /= 1000.0f;  // 밀리도
        if (value > 0.0f && value < 200.0f) maximum = std::max(maximum, value);
    }
    *temperature_c = maximum;
}

/* 와이파이(wlanN)와 그 밖의 링크(usb0 같은 USB 가상 이더넷)를 따로 본다. HUD의 연결 표시는
 * 와이파이만이고, 주소가 있으면서 SSID가 잡혀야(AP에 붙어야) 연결로 친다. USB 링크는 늘 주소가
 * 있어 예전에는 와이파이가 끊겨도 연결로 보였다(2026-10-04 실차). */
void SystemMonitor::sample_network(OverlayHudState *hud)
{
    hud->network_connected = false;
    hud->wifi_signal_dbm = 0;
    hud->network_interface[0] = '\0';
    hud->network_ipv4[0] = '\0';
    hud->network_ssid[0] = '\0';
    hud->wired_interface[0] = '\0';
    hud->wired_ipv4[0] = '\0';

    ifaddrs *addresses = nullptr;
    if (getifaddrs(&addresses) != 0) return;
    for (const ifaddrs *address = addresses; address; address = address->ifa_next) {
        if (!address->ifa_addr || address->ifa_addr->sa_family != AF_INET) continue;
        if ((address->ifa_flags & IFF_UP) == 0 || (address->ifa_flags & IFF_LOOPBACK) != 0) continue;
        const char *name = address->ifa_name ? address->ifa_name : "";
        char ipv4[INET_ADDRSTRLEN] = {};
        const sockaddr_in *socket_address = reinterpret_cast<const sockaddr_in *>(address->ifa_addr);
        if (!inet_ntop(AF_INET, &socket_address->sin_addr, ipv4, sizeof(ipv4))) continue;
        const bool wifi = std::strncmp(name, "wlan", 4) == 0;
        char *interface = wifi ? hud->network_interface : hud->wired_interface;
        char *ip = wifi ? hud->network_ipv4 : hud->wired_ipv4;
        // 와이파이는 wlan0을 먼저, 그 밖의 링크는 처음 본 것
        if (interface[0] != '\0' && !(wifi && std::strcmp(name, "wlan0") == 0)) continue;
        std::snprintf(interface, sizeof(hud->network_interface), "%s", name);
        std::snprintf(ip, sizeof(hud->network_ipv4), "%s", ipv4);
    }
    freeifaddrs(addresses);

    if (hud->network_interface[0] == '\0') return;
    read_ssid(hud->network_interface, hud->network_ssid, sizeof(hud->network_ssid));
    hud->network_connected = hud->network_ssid[0] != '\0';
    if (!hud->network_connected) return;

    FILE *file = std::fopen("/proc/net/wireless", "r");
    if (!file) return;
    char line[160];
    while (std::fgets(line, sizeof(line), file)) {
        char interface_name[16] = {};
        unsigned status = 0;
        float link = 0.0f;
        float level = 0.0f;
        float noise = 0.0f;
        if (std::sscanf(line, " %15[^:]: %x %f %f %f",
                        interface_name, &status, &link, &level, &noise) == 5 &&
            std::strcmp(interface_name, hud->network_interface) == 0) {
            hud->wifi_signal_dbm = static_cast<int>(std::lround(level));
            break;
        }
    }
    std::fclose(file);
}
