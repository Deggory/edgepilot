#include "ipc_channels.h"
#include "ipc_messages.h"
#include "utils_process.h"
#include "utils_time.h"

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

/* 보드 IMU(ST LSM6DSOW)를 읽어 공유 메모리(/edgepilot_imu)에 발행한다. 기록·검증용이다:
 * 제어 경로는 이 데몬을 쓰지 않으므로 IMU가 없거나 죽어도 주행에는 영향이 없다.
 *
 * MaixCAM2의 IMU는 i2c-1의 0x6B다(WHO_AM_I 0x6C). libmaixcam_lib에는 IMU 클래스가 없고
 * MaixPy 모듈은 불러올 때 UART4 핀을 바꾸는 부수효과가 있어, i2c-dev로 직접 읽는다.
 * 가속도·자이로 모두 104 Hz, ±4 g / ±250 dps, 블록 업데이트(BDU)로 한 샘플의 바이트가
 * 섞이지 않게 한다. 값은 칩 좌표 그대로 내고 바이어스도 빼지 않는다.
 *
 * i2c-1에는 터치·RTC·배터리 게이지가 같이 있다. 버스를 붙잡지 않도록 전송은 짧은 결합
 * 전송(레지스터 주소 쓰기 + 읽기) 하나씩이고, 열기에 실패하면 10초마다 다시 시도한다
 * (manager가 1초마다 재시작하는 루프를 만들지 않는다).
 *
 * 환경: EDGEPILOT_IMU_DEV(기본 /dev/i2c-1), EDGEPILOT_IMU_ADDR(기본 0x6B). */

namespace {

volatile sig_atomic_t g_stop = 0;

constexpr uint8_t kRegWhoAmI = 0x0F;
constexpr uint8_t kWhoAmIValue = 0x6C;
constexpr uint8_t kRegCtrl1Xl = 0x10;
constexpr uint8_t kRegCtrl2G = 0x11;
constexpr uint8_t kRegCtrl3C = 0x12;
constexpr uint8_t kRegStatus = 0x1E;
constexpr uint8_t kRegOutTempL = 0x20;  // 0x20 온도, 0x22 자이로 xyz, 0x28 가속도 xyz

constexpr uint8_t kCtrl3SwReset = 0x01;
constexpr uint8_t kCtrl3BduIfInc = 0x44;  // BDU | IF_INC
constexpr uint8_t kCtrl1Xl104Hz4g = 0x48;  // ODR 104 Hz, FS ±4 g
constexpr uint8_t kCtrl2G104Hz250 = 0x40;  // ODR 104 Hz, FS ±250 dps

constexpr float kGravity = 9.80665f;
constexpr float kAccelScale = 0.122e-3f * kGravity;              // m/s^2 / LSB at ±4 g
constexpr float kGyroScale = 8.75e-3f * 3.14159265358979f / 180.0f;  // rad/s / LSB at ±250 dps
constexpr uint64_t kPublishIntervalNs = 100'000'000ULL;
constexpr uint64_t kSamplePeriodNs = 9'615'000ULL;  // 104 Hz

class Lsm6dso {
public:
    ~Lsm6dso() { close(); }

    bool open(const std::string &dev, int addr, std::string *error)
    {
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

    void close()
    {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }

    // 가속도·자이로가 모두 새로 준비됐으면 한 샘플을 읽는다.
    // 1: 샘플, 0: 아직 없음, -1: I2C 오류.
    int poll(ImuSample *out)
    {
        uint8_t status = 0;
        if (!read(kRegStatus, &status, 1)) return -1;
        if ((status & 0x03) != 0x03) return 0;
        uint8_t raw[14];
        if (!read(kRegOutTempL, raw, sizeof(raw))) return -1;
        auto s16 = [&](int i) { return static_cast<int16_t>(raw[i] | (raw[i + 1] << 8)); };
        out->timestamp_ns = monotonic_now_ns();
        out->temperature_c = 25.0f + s16(0) / 256.0f;
        for (int k = 0; k < 3; ++k) {
            out->gyro_rad_s[k] = s16(2 + 2 * k) * kGyroScale;
            out->accel_mps2[k] = s16(8 + 2 * k) * kAccelScale;
        }
        return 1;
    }

private:
    bool fail(std::string *error, const char *what)
    {
        *error = std::string(what) + " failed: " + std::strerror(errno);
        close();
        return false;
    }

    bool read(uint8_t reg, uint8_t *buf, uint16_t len)
    {
        i2c_msg msgs[2] = {
            {static_cast<uint16_t>(addr_), 0, 1, &reg},
            {static_cast<uint16_t>(addr_), I2C_M_RD, len, buf},
        };
        i2c_rdwr_ioctl_data xfer = {msgs, 2};
        return ioctl(fd_, I2C_RDWR, &xfer) == 2;
    }

    bool write(uint8_t reg, uint8_t value)
    {
        uint8_t buf[2] = {reg, value};
        i2c_msg msg = {static_cast<uint16_t>(addr_), 0, 2, buf};
        i2c_rdwr_ioctl_data xfer = {&msg, 1};
        return ioctl(fd_, I2C_RDWR, &xfer) == 1;
    }

    int fd_ = -1;
    int addr_ = 0;
};

} // namespace

int main()
{
    install_stop_signal_handlers(&g_stop);
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
    int consecutive_errors = 0;

    while (!g_stop) {
        if (!opened) {
            std::string error;
            opened = imu.open(dev, addr, &error);
            if (opened) {
                std::fprintf(stderr, "imud: LSM6DSOW on %s 0x%02X, 104 Hz, +-4 g, +-250 dps\n", dev.c_str(), addr);
                last_error.clear();
                consecutive_errors = 0;
                last_sample_ns = 0;
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
            if (last_sample_ns && sample.timestamp_ns - last_sample_ns > kSamplePeriodNs * 3 / 2)
                dropped += (sample.timestamp_ns - last_sample_ns + kSamplePeriodNs / 2) / kSamplePeriodNs - 1;
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
        // 104 Hz(9.6 ms) 샘플을 놓치지 않을 만큼만 자주 본다.
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    std::fprintf(stderr, "imud: done\n");
    return 0;
}
