#include "panda/panda_can_codec.h"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace {

constexpr int kUsbPacketMaxSize = 0x40;
constexpr int kCanPacketHeadSize = 5;

constexpr uint8_t kDlcToLen[16] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64,
};

void set_error(std::string *error, const char *message) {
    if (error) *error = message;
}

uint8_t len_to_dlc(uint8_t len) {
    for (uint8_t dlc = 0; dlc < 16; ++dlc) {
        if (kDlcToLen[dlc] == len) return dlc;
    }
    return 0xff;
}

uint32_t read_le32(const uint8_t *data) {
    return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

void write_le32(uint8_t *data, uint32_t value) {
    data[0] = static_cast<uint8_t>(value & 0xffU);
    data[1] = static_cast<uint8_t>((value >> 8) & 0xffU);
    data[2] = static_cast<uint8_t>((value >> 16) & 0xffU);
    data[3] = static_cast<uint8_t>((value >> 24) & 0xffU);
}

void append_packet_bytes(std::vector<uint8_t> *dest, const uint8_t *src, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        if ((dest->size() % kUsbPacketMaxSize) == 0) {
            dest->push_back(static_cast<uint8_t>(dest->size() / kUsbPacketMaxSize));
        }
        dest->push_back(src[i]);
    }
}

}  // namespace

void panda_can_pack_buffer(const std::vector<PandaCanFrame> &frames, std::vector<uint8_t> *out) {
    out->clear();
    for (const PandaCanFrame &frame : frames) {
        const uint8_t dlc = len_to_dlc(frame.data_len);
        uint8_t header[kCanPacketHeadSize] = {};
        const uint8_t extended = frame.address >= 0x800U ? 1U : 0U;
        const uint32_t word_4b = (frame.address << 3) | (extended << 2);
        header[0] = static_cast<uint8_t>((dlc << 4) | (frame.bus << 1));
        write_le32(&header[1], word_4b);

        append_packet_bytes(out, header, sizeof(header));
        append_packet_bytes(out, frame.data, frame.data_len);
    }
}

bool panda_can_unpack_stream(const uint8_t *data, int size, bool complete, PandaRxStream *stream,
                             std::vector<PandaCanFrame> *frames, std::string *error) {
    frames->clear();
    std::vector<uint8_t> buffer = std::move(stream->pending);
    stream->pending.clear();
    for (int i = 0; i < size; i += kUsbPacketMaxSize) {
        if (data[i] != stream->next_counter) {
            set_error(error, "malformed CAN USB packet counter");
            *stream = PandaRxStream{};
            return false;
        }
        ++stream->next_counter;
        const int chunk_len = std::min(kUsbPacketMaxSize, size - i);
        buffer.insert(buffer.end(), &data[i + 1], &data[i + chunk_len]);
    }

    size_t pos = 0;
    while (pos < buffer.size()) {
        const size_t left = buffer.size() - pos;
        const uint8_t header0 = buffer[pos];
        const uint8_t data_len = kDlcToLen[(header0 >> 4) & 0xfU];
        if (left < static_cast<size_t>(kCanPacketHeadSize) + data_len) {
            if (!complete) {
                // 잘린 읽기: 걸쳐 있는 CAN 패킷의 뒤쪽은 다음 읽기의 첫 패킷에 온다
                stream->pending.assign(buffer.begin() + static_cast<std::ptrdiff_t>(pos), buffer.end());
                break;
            }
            set_error(error, left < static_cast<size_t>(kCanPacketHeadSize) ? "truncated CAN packet header"
                                                                            : "truncated CAN packet payload");
            *stream = PandaRxStream{};
            return false;
        }
        const uint32_t word_4b = read_le32(buffer.data() + pos + 1);
        PandaCanFrame frame;
        frame.bus = (header0 >> 1) & 0x7U;
        frame.address = word_4b >> 3;
        frame.rejected = (word_4b & 0x1U) != 0;
        frame.returned = (word_4b & 0x2U) != 0;
        frame.data_len = data_len;
        std::copy_n(buffer.data() + pos + kCanPacketHeadSize, data_len, frame.data);
        frames->push_back(frame);

        pos += kCanPacketHeadSize + data_len;
    }
    // 전송이 끝났으면 다음 읽기는 새 전송이다(펌웨어가 카운터를 0으로 되돌린다)
    if (complete) stream->next_counter = 0;
    return true;
}
