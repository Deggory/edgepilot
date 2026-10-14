#include "common/build_version.h"
#include "common/ipc_channels.h"
#include "common/ipc_messages.h"
#include "common/utils_process.h"
#include "common/utils_time.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

// 보드 IMU(ST LSM6DSOW)를 읽어 공유 메모리(/edgepilot_imu)에 발행한다. locationd의 입력이고
// 녹화된다. IMU가 없거나 죽으면 locationd가 발행을 멈추고 controlsd 학습기는 ESP12 값으로
// 돌아가므로 주행은 계속된다.
//
// MaixCAM2의 IMU는 i2c-1의 0x6B다(WHO_AM_I 0x6C). libmaixcam_lib에는 IMU 클래스가 없고
// MaixPy 모듈은 불러올 때 UART4 핀을 바꾸는 부수효과가 있어, i2c-dev로 직접 읽는다.
// 가속도·자이로 모두 104 Hz, ±4 g / ±250 dps, 블록 업데이트(BDU)로 한 샘플의 바이트가
// 섞이지 않게 한다. 값은 칩 좌표 그대로 내고 바이어스도 빼지 않는다.
//
// i2c-1에는 터치·RTC·배터리 게이지가 같이 있다. 버스를 붙잡지 않도록 전송은 짧은 결합
// 전송(레지스터 주소 쓰기 + 읽기) 하나씩이고, 열기에 실패하면 10초마다 다시 시도한다
// (manager가 1초마다 재시작하는 루프를 만들지 않는다).
//
// 환경: EDGEPILOT_IMU_DEV(기본 /dev/i2c-1), EDGEPILOT_IMU_ADDR(기본 0x6B).

namespace {

volatile sig_atomic_t g_stop = 0;

constexpr uint8_t kRegWhoAmI = 0x0F;
constexpr uint8_t kWhoAmIValue = 0x6C;
constexpr uint8_t kRegCtrl1Xl = 0x10;
constexpr uint8_t kRegCtrl2G = 0x11;
constexpr uint8_t kRegCtrl3C = 0x12;
constexpr uint8_t kRegStatus = 0x1E;
constexpr uint8_t kRegOutTempL = 0x20;  // 0x20 온도
constexpr uint8_t kRegOutGyroL = 0x22;  // 0x22 자이로 xyz, 0x28 가속도 xyz

constexpr uint8_t kCtrl3SwReset = 0x01;
constexpr uint8_t kCtrl3BduIfInc = 0x44;   // BDU | IF_INC
constexpr uint8_t kCtrl1Xl104Hz4g = 0x48;  // ODR 104 Hz, FS ±4 g
constexpr uint8_t kCtrl2G104Hz250 = 0x40;  // ODR 104 Hz, FS ±250 dps

constexpr float kGravity = 9.80665f;
constexpr float kAccelScale = 0.122e-3f * kGravity;                  // m/s^2 / LSB at ±4 g
constexpr float kGyroScale = 8.75e-3f * 3.14159265358979f / 180.0f;  // ±250 dps에서 LSB당 rad/s
constexpr uint64_t kPublishIntervalNs = 100'000'000ULL;
constexpr uint64_t kSamplePeriodNs = 9'615'000ULL;  // 104 Hz
constexpr uint64_t kTemperatureIntervalNs = 1'000'000'000ULL;
// 샘플을 읽고 kFirstPollNs 뒤에 깨어나고, 준비되지 않았으면 kPollRetryNs마다 본다. 읽은 시각에서 재므로 다음
// 샘플(실제 간격 약 9.9 ms, 101 Hz)보다 늘 먼저 깨어나 건너뛰지 않는다. 받은 간격으로 깨어날 때를 잡으면 늦게
// 읽을수록 더 늦게 깨어나 샘플을 건너뛴다(2026-10-13 보드에서 85 Hz까지 떨어졌다).
constexpr uint64_t kFirstPollNs = kSamplePeriodNs * 5 / 6;  // 약 8 ms
constexpr uint64_t kPollRetryNs = 1'000'000ULL;
// 샘플이 다섯 주기 넘게 없으면 10 ms마다만 보고, 0.5 s 넘게 없으면 센서를 다시 연다(설정을 다시 쓴다). 전원 글리치로
// 센서가 power-down(ODR 0)으로 돌아가면 상태 읽기는 되지만 데이터가 오지 않아, 그대로면 1 ms마다 I2C를 읽기만 한다.
constexpr uint64_t kSlowRetryAfterNs = kSamplePeriodNs * 5;
constexpr uint64_t kSlowPollRetryNs = 10'000'000ULL;
constexpr uint64_t kNoSampleReopenNs = 500'000'000ULL;

class Lsm6dso {
public:
    ~Lsm6dso() { close(); }

    bool open(const std::string &dev, int addr, std::string *error) {
        close();
        fd_ = ::open(dev.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0) {
            *error = "open " + dev + ": " + std::strerror(errno);
            return false;
        }
        addr_ = addr;
        uint8_t who = 0;
        if (!read(kRegWhoAmI, &who, 1)) {
            *error = "WHO_AM_I read failed: " + std::string(std::strerror(errno));
            close();
            return false;
        }
        if (who != kWhoAmIValue) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "WHO_AM_I 0x%02x, expected 0x%02x", who, kWhoAmIValue);
            *error = buf;
            close();
            return false;
        }
        // 다른 앱(MaixPy 샘플)이 남긴 설정을 지우고 우리 설정만 둔다.
        if (!write(kRegCtrl3C, kCtrl3SwReset)) return fail(error, "reset");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (!write(kRegCtrl3C, kCtrl3BduIfInc) || !write(kRegCtrl1Xl, kCtrl1Xl104Hz4g) ||
            !write(kRegCtrl2G, kCtrl2G104Hz250))
            return fail(error, "configure");
        return true;
    }

    void close() {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }

    // 가속도·자이로가 모두 새로 준비됐으면 한 샘플을 읽는다.
    // 1: 샘플, 0: 아직 없음, -1: I2C 오류.
    int poll(ImuSample *out) {
        uint8_t status = 0;
        if (!read(kRegStatus, &status, 1)) return -1;
        if ((status & 0x03) != 0x03) return 0;
        // 자이로·가속도 12바이트만 읽는다. 이 버스(i2c_designware)는 여러 바이트를 읽으면 바이트마다 인터럽트가
        // 16번쯤 걸려(1바이트 읽기는 4번, 2026-10-13 보드 실측), 샘플마다 읽던 온도 2바이트도 인터럽트를 14% 늘린다.
        uint8_t raw[12];
        if (!read(kRegOutGyroL, raw, sizeof(raw))) return -1;
        auto s16 = [](const uint8_t *p) { return static_cast<int16_t>(p[0] | (p[1] << 8)); };
        out->timestamp_ns = monotonic_now_ns();
        for (int k = 0; k < 3; ++k) {
            out->gyro_rad_s[k] = s16(raw + 2 * k) * kGyroScale;
            out->accel_mps2[k] = s16(raw + 6 + 2 * k) * kAccelScale;
        }
        // 온도는 기록만 하고 쓰는 곳이 없어 1초에 한 번 읽는다
        if (temperature_ns_ == 0 || out->timestamp_ns - temperature_ns_ >= kTemperatureIntervalNs) {
            uint8_t t[2];
            if (read(kRegOutTempL, t, sizeof(t))) temperature_c_ = 25.0f + s16(t) / 256.0f;
            temperature_ns_ = out->timestamp_ns;
        }
        out->temperature_c = temperature_c_;
        return 1;
    }

private:
    bool fail(std::string *error, const char *what) {
        *error = std::string(what) + " failed: " + std::strerror(errno);
        close();
        return false;
    }

    bool read(uint8_t reg, uint8_t *buf, uint16_t len) {
        i2c_msg msgs[2] = {
            {static_cast<uint16_t>(addr_), 0, 1, &reg},
            {static_cast<uint16_t>(addr_), I2C_M_RD, len, buf},
        };
        i2c_rdwr_ioctl_data xfer = {msgs, 2};
        return ioctl(fd_, I2C_RDWR, &xfer) == 2;
    }

    bool write(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        i2c_msg msg = {static_cast<uint16_t>(addr_), 0, 2, buf};
        i2c_rdwr_ioctl_data xfer = {&msg, 1};
        return ioctl(fd_, I2C_RDWR, &xfer) == 1;
    }

    int fd_ = -1;
    int addr_ = 0;
    float temperature_c_ = 0.0f;
    uint64_t temperature_ns_ = 0;
};

}  // namespace

int main() {
    install_stop_signal_handlers(&g_stop);
    log_build_version("imud");
    const std::string dev = env_string("EDGEPILOT_IMU_DEV", "/dev/i2c-1");
    const int addr = static_cast<int>(std::strtol(env_string("EDGEPILOT_IMU_ADDR", "0x6B").c_str(), nullptr, 0));

    LatestChannel pub;
    if (!pub.open(kImuTopic, sizeof(ImuBatch), true)) {
        std::fprintf(stderr, "imud: open %s failed\n", kImuTopic);
        return 1;
    }

    Lsm6dso imu;
    std::string last_error;
    ImuBatch batch;
    uint64_t samples = 0, errors = 0, dropped = 0, last_sample_ns = 0;
    uint64_t batch_start = monotonic_now_ns(), window_start = batch_start;
    bool opened = false;
    uint64_t opened_ns = 0;
    int consecutive_errors = 0;
    uint64_t next_poll_ns = 0;  // 다음에 깨어날 시각(kFirstPollNs, kPollRetryNs)

    while (!g_stop) {
        if (!opened) {
            std::string error;
            opened = imu.open(dev, addr, &error);
            if (opened) {
                std::fprintf(stderr, "imud: LSM6DSOW on %s 0x%02X, 104 Hz, +-4 g, +-250 dps\n", dev.c_str(), addr);
                last_error.clear();
                consecutive_errors = 0;
                last_sample_ns = 0;
                opened_ns = monotonic_now_ns();
            } else {
                if (error != last_error) std::fprintf(stderr, "imud: %s (retrying every 10 s)\n", error.c_str());
                last_error = error;
                for (int i = 0; i < 100 && !g_stop; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }

        ImuSample sample;
        const int got = imu.poll(&sample);
        if (got < 0) {
            ++errors;
            if (++consecutive_errors >= 50) {
                std::fprintf(stderr, "imud: %d consecutive I2C errors, reopening\n", consecutive_errors);
                imu.close();
                opened = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        consecutive_errors = 0;
        if (got > 0) {
            // 샘플 간격이 1.5주기를 넘으면 그 사이 준비됐던 샘플을 놓친 것이다.
            const uint64_t interval = last_sample_ns ? sample.timestamp_ns - last_sample_ns : 0;
            if (interval > kSamplePeriodNs * 3 / 2) dropped += (interval + kSamplePeriodNs / 2) / kSamplePeriodNs - 1;
            last_sample_ns = sample.timestamp_ns;
            if (batch.count < kImuBatchMaxSamples) batch.samples[batch.count++] = sample;
            ++samples;
        }

        const uint64_t now = monotonic_now_ns();
        if (batch.count && (now - batch_start >= kPublishIntervalNs || batch.count == kImuBatchMaxSamples)) {
            batch.timestamp_ns = now;
            batch.dropped = static_cast<uint32_t>(dropped);
            pub.publish(&batch, sizeof(batch));
            batch.count = 0;
            batch_start = now;
        }
        if (now - window_start >= 10'000'000'000ULL) {
            std::fprintf(stderr, "imud: rate=%.1f Hz samples=%llu dropped=%llu errors=%llu\n",
                         samples * 1e9 / (now - window_start), static_cast<unsigned long long>(samples),
                         static_cast<unsigned long long>(dropped), static_cast<unsigned long long>(errors));
            window_start = now;
            samples = 0;
        }
        // 다음 샘플 직전까지 자고, 그 뒤로는 1 ms마다 본다. 3 ms마다 보던 때보다 상태 읽기와 깨어남이 줄고
        // 타임스탬프(읽은 시각) 오차가 1 ms 안이다. 시각은 위에서 읽은 now를 다시 쓴다(CLOCK_BOOTTIME은 syscall이다).
        const uint64_t since_sample = now - (last_sample_ns ? last_sample_ns : opened_ns);
        if (got == 0 && since_sample > kNoSampleReopenNs) {
            std::fprintf(stderr, "imud: no sample for %llu ms, reopening\n",
                         static_cast<unsigned long long>(since_sample / 1000000ULL));
            imu.close();
            opened = false;
            continue;
        }
        if (got > 0)
            next_poll_ns = sample.timestamp_ns + kFirstPollNs;
        else
            next_poll_ns = now + (since_sample > kSlowRetryAfterNs ? kSlowPollRetryNs : kPollRetryNs);
        if (next_poll_ns > now) std::this_thread::sleep_for(std::chrono::nanoseconds(next_poll_ns - now));
    }
    std::fprintf(stderr, "imud: done\n");
    return 0;
}
