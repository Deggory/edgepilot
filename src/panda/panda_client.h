#pragma once

// Panda USB 연결(PandaClient): 접속, safety 설정, 하트비트, 상태 조회, CAN 송수신. pandad가 쓴다.

#include "panda/panda_can_codec.h"
#include "panda/panda_protocol.h"

#include <libusb.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

// send()의 결과.
enum class PandaSendResult {
    Sent,
    NotReady,  // 끊겼거나 safety 설정(set_configured) 전이다
    Expired,   // USB 잠금을 얻었을 때 이미 not_after_ns를 넘겼다
    Failed,    // USB 전송 실패
};

// Panda USB 연결. pandad는 수신 루프(메인 스레드)와 송신 스레드가 한 객체를 나눠 쓴다: USB를 건드리는
// 메서드는 모두 내부 잠금(상류 pandad의 hw_lock과 같은 재귀 잠금)으로 한 번에 하나씩 돈다. 송신 스레드는
// send()와 ready()만 부르고, 접속·설정은 메인 스레드만 한다. 송신 가능 여부(접속 + safety 설정 확인)와 신선도는
// send()가 잠금을 얻은 뒤에 보므로, 재접속 직후 설정 전에 보내거나 잠금을 기다리는 사이 낡은 묶음을 보내지 않는다.
class PandaClient {
public:
    PandaClient() = default;
    ~PandaClient();

    bool connect(const std::string &serial = "");
    void close();
    bool connected() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return dev_handle_ != nullptr;
    }
    bool comms_healthy() const { return comms_healthy_.load(); }
    // 메인 스레드가 safety 모델 설정을 확인한 뒤 부른다. close()와 connect()가 지운다.
    void set_configured() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        configured_ = dev_handle_ != nullptr;
    }
    // 접속돼 있고 safety 설정이 확인됐다.
    bool ready() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return configured_ && dev_handle_ != nullptr;
    }
    // 아래 접속 정보는 송신 스레드의 USB 오류(장치 사라짐)가 close로 비울 수 있어 잠금 아래에서 복사해 준다.
    std::string usb_serial() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return usb_serial_;
    }
    // 접속할 때 읽은 펌웨어 버전 문자열(0xd6). 못 읽었으면 빈 문자열이다.
    std::string firmware_version() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return firmware_version_;
    }
    uint8_t hw_type() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return hw_type_;
    }
    uint8_t health_packet_version() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return health_packet_version_;
    }
    uint8_t can_packet_version() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return can_packet_version_;
    }
    uint32_t usb_tx_timeouts() const { return usb_tx_timeouts_; }
    uint32_t usb_tx_retries() const { return usb_tx_retries_; }
    uint32_t malformed_rx_batches() const { return malformed_rx_batches_; }

    bool set_safety_model(uint16_t safety_model, uint16_t safety_param);
    bool send_heartbeat(bool engaged);
    bool get_health(PandaHealthPacket *health);
    bool receive(std::vector<PandaCanFrame> *frames, int timeout_ms = 10);
    // 잠금을 얻은 뒤 ready()가 아니면 NotReady, 지금이 not_after_ns(monotonic_now_ns)보다 늦으면 Expired로 보내지
    // 않는다. started_ns에는 잠금을 얻어 USB 전송을 시작한 시각을 쓴다.
    PandaSendResult send(const std::vector<PandaCanFrame> &frames, uint64_t not_after_ns,
                         uint64_t *started_ns = nullptr);

private:
    bool control_write(uint8_t request, uint16_t value, uint16_t index, unsigned timeout_ms = 1000);
    bool control_read(uint8_t request, uint16_t value, uint16_t index, void *data, uint16_t length,
                      unsigned timeout_ms = 1000);
    // timed_out: 시간 초과로 끝났다(옮긴 바이트가 있으면 전송 중간에서 끊긴 것이다)
    int bulk_read(uint8_t endpoint, uint8_t *data, int length, unsigned timeout_ms, bool *timed_out = nullptr);
    int bulk_write(uint8_t endpoint, const uint8_t *data, int length, unsigned timeout_ms);
    void mark_usb_error(int err, const char *where);
    bool unpack_can_buffer(const uint8_t *data, int size, bool complete, std::vector<PandaCanFrame> *frames);

    mutable std::recursive_mutex mutex_;  // USB와 dev_handle_(mark_usb_error가 안에서 close를 부른다)
    libusb_context *ctx_ = nullptr;
    libusb_device_handle *dev_handle_ = nullptr;
    bool configured_ = false;  // set_configured 뒤로 송신을 허용한다
    std::string usb_serial_;
    std::string firmware_version_;
    uint8_t hw_type_ = 0;
    uint8_t health_packet_version_ = 0;
    uint8_t can_packet_version_ = 0;
    std::atomic<bool> comms_healthy_{true};
    std::atomic<uint32_t> usb_tx_timeouts_{0};
    std::atomic<uint32_t> usb_tx_retries_{0};
    std::atomic<uint32_t> malformed_rx_batches_{0};
    uint32_t consecutive_malformed_rx_ = 0;
    PandaRxStream rx_stream_;  // 시간 초과로 잘린 수신을 다음 읽기로 잇는다
};
