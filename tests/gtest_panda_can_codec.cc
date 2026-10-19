// panda USB CAN 버퍼 코덱: panda python의 pack_can_buffer와 같은 바이트로 싸는지,
// 소프트 청크(256 B)를 넘는 스트림까지 왕복하는지 본다.
#include "panda/panda_can_codec.h"

#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <string>
#include <vector>

namespace {

PandaCanFrame make_frame(uint32_t address, uint8_t bus, const std::vector<uint8_t> &data) {
    PandaCanFrame frame;
    frame.address = address;
    frame.bus = bus;
    frame.data_len = static_cast<uint8_t>(data.size());
    if (!data.empty()) std::memcpy(frame.data, data.data(), data.size());
    return frame;
}

// 전송 하나를 끝까지 받은 읽기
bool unpack_transfer(const std::vector<uint8_t> &bytes, std::vector<PandaCanFrame> *frames, std::string *error) {
    PandaRxStream stream;
    return panda_can_unpack_stream(bytes.data(), static_cast<int>(bytes.size()), true, &stream, frames, error);
}

bool equal_frame(const PandaCanFrame &a, const PandaCanFrame &b) {
    return a.address == b.address && a.bus == b.bus && a.data_len == b.data_len && a.returned == b.returned &&
           a.rejected == b.rejected && std::memcmp(a.data, b.data, a.data_len) == 0;
}

TEST(PandaCanCodec, PackUnpackRoundTrip) {
    std::vector<PandaCanFrame> frames = {
        make_frame(0x123, 2, {0, 1, 2, 3, 4, 5, 6, 7}),
        make_frame(0x18da10f1U, 1, {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x10, 0x20}),
    };

    std::vector<uint8_t> packed;
    std::string error;
    panda_can_pack_buffer(frames, &packed);
    EXPECT_GE(packed.size(), 27u) << "CAN 패킷 두 개와 USB 카운터";

    const std::array<uint8_t, 14> expected_first = {
        0x00,                    // USB 카운터
        0x84,                    // dlc=8, bus=2
        0x18, 0x09, 0x00, 0x00,  // 0x123 << 3
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    };
    EXPECT_TRUE(std::equal(expected_first.begin(), expected_first.end(), packed.begin()))
        << "첫 헤더가 panda python의 pack_can_buffer와 같다";

    std::vector<PandaCanFrame> unpacked;
    EXPECT_TRUE(unpack_transfer(packed, &unpacked, &error)) << error;
    EXPECT_EQ(unpacked.size(), frames.size());
    for (size_t i = 0; i < unpacked.size() && i < frames.size(); ++i) {
        EXPECT_TRUE(equal_frame(unpacked[i], frames[i])) << "왕복한 프레임 " << i;
    }

    std::vector<PandaCanFrame> many;
    for (int i = 0; i < 40; ++i) {
        many.push_back(make_frame(0x340 + static_cast<uint32_t>(i), i % 4, {0, 1, 2, 3, 4, 5, 6, 7}));
    }
    panda_can_pack_buffer(many, &packed);
    EXPECT_GT(packed.size(), 256u) << "긴 스트림은 panda 소프트 청크 크기를 넘는다";
    EXPECT_TRUE(unpack_transfer(packed, &unpacked, &error)) << error;
    EXPECT_EQ(unpacked.size(), many.size());

    std::vector<uint8_t> flagged = {
        0x00,
        0x80,                    // dlc=8, bus=0
        0x1b, 0x09, 0x00, 0x00,  // (0x123 << 3) | returned | rejected
        1,    2,    3,    4,    5, 6, 7, 8,
    };
    EXPECT_TRUE(unpack_transfer(flagged, &unpacked, &error)) << error;
    ASSERT_EQ(unpacked.size(), 1u);
    EXPECT_TRUE(unpacked[0].returned);
    EXPECT_TRUE(unpacked[0].rejected);

    panda_can_pack_buffer({}, &packed);
    EXPECT_TRUE(packed.empty()) << "빈 TX 묶음은 바이트를 만들지 않는다";
}

// 펌웨어 usb_cb_ep1_in(firmware/panda/board/usb_protocol.h)을 옮긴 것: CAN 큐에서 IN 패킷 하나(64바이트까지)를 만든다.
// 0이면 0바이트 패킷, 64보다 짧으면 짧은 패킷이고 둘 다 전송의 끝이다.
struct FakePandaUsb {
    static constexpr int kMaxChunk = 16256;   // MAX_EP1_CHUNK_PER_BULK_TRANSFER
    std::vector<std::vector<uint8_t>> queue;  // 머리(5바이트)와 데이터
    size_t next = 0;
    int ptr = 0;
    uint8_t tail[72] = {};
    uint8_t counter = 0;
    int total_rx_size = 0;

    int ep1_in(uint8_t *usb, int len = 64) {
        int pos = 1;
        usb[0] = counter;
        if (ptr > 0) {
            if (ptr <= 63) {
                std::memcpy(&usb[pos], tail, static_cast<size_t>(ptr));
                pos += ptr;
                ptr = 0;
            } else {
                std::memcpy(&usb[pos], tail, 63);
                ptr -= 63;
                std::memmove(tail, &tail[63], static_cast<size_t>(ptr));
                pos += 63;
            }
        }
        if (total_rx_size > kMaxChunk) {
            total_rx_size = 0;
            counter = 0;
        } else {
            while (pos < len && next < queue.size()) {
                const std::vector<uint8_t> &packet = queue[next++];
                const int packet_len = static_cast<int>(packet.size());
                if (pos + packet_len <= len) {
                    std::memcpy(&usb[pos], packet.data(), packet.size());
                    pos += packet_len;
                } else {
                    std::memcpy(&usb[pos], packet.data(), static_cast<size_t>(len - pos));
                    ptr = packet_len - (len - pos);
                    std::memcpy(tail, &packet[static_cast<size_t>(len - pos)], static_cast<size_t>(ptr));
                    pos = len;
                }
            }
            ++counter;
            total_rx_size += pos;
        }
        if (pos != len) {
            counter = 0;
            total_rx_size = 0;
        }
        if (pos <= 1) pos = 0;
        return pos;
    }

    // 호스트의 bulk 읽기 한 번: 짧은 패킷이 오거나 cut 패킷을 받으면(시간 초과) 멈춘다. complete는 전송 끝까지
    // 받았는지.
    std::vector<uint8_t> read(int cut, bool *complete) {
        std::vector<uint8_t> out;
        *complete = false;
        for (int i = 0; i < cut; ++i) {
            uint8_t usb[64];
            const int n = ep1_in(usb);
            out.insert(out.end(), usb, usb + n);
            if (n < 64) {
                *complete = true;
                break;
            }
        }
        return out;
    }
};

std::vector<uint8_t> raw_can_packet(const PandaCanFrame &frame) {
    static constexpr uint8_t kLenToDlc[65] = {0, 1, 2,  3, 4,  5, 6, 7, 8, 0, 0,  0, 9, 0, 0, 0, 10, 0, 0, 0, 11, 0,
                                              0, 0, 12, 0, 0,  0, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  0,
                                              0, 0, 0,  0, 14, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,  0, 0, 0, 15};
    std::vector<uint8_t> packet(5 + frame.data_len);
    packet[0] = static_cast<uint8_t>((kLenToDlc[frame.data_len] << 4) | (frame.bus << 1));
    const uint32_t word = (frame.address << 3) | (frame.address >= 0x800U ? 4U : 0U);
    for (int i = 0; i < 4; ++i) packet[1 + i] = static_cast<uint8_t>(word >> (8 * i));
    std::memcpy(&packet[5], frame.data, frame.data_len);
    return packet;
}

// 읽기가 시간 초과로 전송 중간에서 끊겨도(밀린 큐를 10 ms 안에 다 받지 못할 때) 잘린 꼬리와 카운터를 다음 읽기로
// 넘겨 프레임을 하나도 잃지 않는다. 예전에는 잘린 묶음을 통째로 버리고 다음 읽기도 카운터 오류로 버려, 세 번이면
// 재접속했다(재접속은 Panda의 controls_allowed를 지운다).
TEST(PandaCanCodec, CutReadsContinueTheTransfer) {
    std::vector<PandaCanFrame> sent;
    FakePandaUsb usb;
    for (int i = 0; i < 3000; ++i) {
        const uint8_t len = i % 97 == 0 ? 64 : static_cast<uint8_t>(i % 9);  // 가끔 FD 64바이트(꼬리가 두 패킷)
        std::vector<uint8_t> data(len);
        for (uint8_t k = 0; k < len; ++k) data[k] = static_cast<uint8_t>(i + k);
        sent.push_back(make_frame(0x100U + static_cast<uint32_t>(i % 700), static_cast<uint8_t>(i % 3), data));
        usb.queue.push_back(raw_can_packet(sent.back()));
    }
    PandaRxStream stream;
    std::vector<PandaCanFrame> received;
    int cut_reads = 0, stateless_failures = 0;
    for (int read = 0; read < 10000 && received.size() < sent.size(); ++read) {
        const int cut = 1 + (read * 37) % 23;  // 1~23패킷에서 끊는다
        bool complete = false;
        const std::vector<uint8_t> bytes = usb.read(cut, &complete);
        cut_reads += complete ? 0 : 1;
        std::vector<PandaCanFrame> frames;
        std::string error;
        ASSERT_TRUE(
            panda_can_unpack_stream(bytes.data(), static_cast<int>(bytes.size()), complete, &stream, &frames, &error))
            << "읽기 " << read << ": " << error;
        received.insert(received.end(), frames.begin(), frames.end());
        std::vector<PandaCanFrame> ignored;
        if (!unpack_transfer(bytes, &ignored, nullptr)) ++stateless_failures;
    }
    EXPECT_GT(cut_reads, 20) << "잘린 읽기를 실제로 거쳤다";
    EXPECT_GT(stateless_failures, 20) << "읽기마다 따로 풀면(예전 방식) 잘린 읽기와 그 다음 읽기를 버린다";
    ASSERT_EQ(received.size(), sent.size());
    for (size_t i = 0; i < sent.size(); ++i) ASSERT_TRUE(equal_frame(received[i], sent[i])) << "프레임 " << i;
}

// 카운터가 어긋나면(전송을 놓친 경우) 그 묶음을 버리고 처음 상태로 돌아간다
TEST(PandaCanCodec, StreamResetsOnACounterMismatch) {
    PandaRxStream stream;
    stream.next_counter = 3;
    stream.pending = {0x84};
    uint8_t packet[64] = {};
    packet[0] = 7;
    std::vector<PandaCanFrame> frames;
    std::string error;
    EXPECT_FALSE(panda_can_unpack_stream(packet, 10, true, &stream, &frames, &error));
    EXPECT_EQ(error, "malformed CAN USB packet counter");
    EXPECT_EQ(stream.next_counter, 0);
    EXPECT_TRUE(stream.pending.empty());
}

}  // namespace
