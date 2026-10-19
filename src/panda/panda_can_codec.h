#ifndef PANDA_CAN_CODEC_H
#define PANDA_CAN_CODEC_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

constexpr uint8_t kPandaCanMaxDataLen = 64U;

struct PandaCanFrame {
    uint32_t address = 0;
    uint8_t bus = 0;
    uint8_t data_len = 0;
    uint8_t data[kPandaCanMaxDataLen] = {};
    bool returned = false;
    bool rejected = false;
};

// 보낼 프레임을 USB 버퍼로 싼다. 프레임은 controlsd가 만든 것뿐이다(LKAS11·CLU11·MDPS12, 버스 0~2, 4·8바이트).
void panda_can_pack_buffer(const std::vector<PandaCanFrame> &frames, std::vector<uint8_t> *out);

// USB IN 읽기 사이에 남는 상태. 펌웨어의 한 전송은 카운터가 0부터 붙는 64바이트 패킷들이고 짧은 패킷(또는 0바이트
// 패킷)으로 끝난다. 읽기가 시간 초과로 전송 중간에서 끊기면 펌웨어는 다음 읽기에서 같은 전송을 이어 보낸다: 카운터가
// 이어지고, 걸쳐 있던 CAN 패킷의 뒤쪽이 다음 패킷에 온다.
struct PandaRxStream {
    uint8_t next_counter = 0;      // 다음 패킷에 기대하는 카운터
    std::vector<uint8_t> pending;  // 아직 다 오지 않은 CAN 패킷의 앞부분
};

// 한 번의 USB 읽기를 푼다. complete는 이 읽기가 전송 끝까지 받았다는 뜻이다(시간 초과로 잘리지 않았다). 잘린 읽기면
// 남은 앞부분과 카운터를 stream에 두고 다음 읽기에서 잇는다. 실패하면 stream을 처음으로 되돌린다. size는 0보다 크다.
bool panda_can_unpack_stream(const uint8_t *data, int size, bool complete, PandaRxStream *stream,
                             std::vector<PandaCanFrame> *frames, std::string *error = nullptr);

#endif
