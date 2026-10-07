#ifndef MAIXCAM2_VENC_H
#define MAIXCAM2_VENC_H

/* AX620E 하드웨어 비디오 인코더(VENC, H.264). 보드 디코더(VDEC)도 H.264를 풀어서, 녹화한
 * 주행을 그대로 리허설(replayd)에 쓸 수 있다.
 * 프레임 링 슬롯(CMM)을 IVPS로 자체 풀 블록에 복사한 뒤(인코더는 비동기로 읽으므로 camerad가 슬롯을 덮어써도 안전하다) 인코더에
 * 넣고, 나온 스트림을 Annex B 패킷으로 돌려준다. 파라미터 세트(SPS/PPS)는 codec config로 따로 내고
 * 프레임 패킷에서는 뺀다(RecordingWriter가 세그먼트 앞에 붙인다). MSP SDK를 직접
 * 쓰며 AX 헤더는 이 파일 밖으로 새지 않는다. AX_SYS_Init은 생성자가 한다. */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

class VideoEncoder {
public:
    struct Packet {
        const uint8_t *data;
        size_t size;
        uint64_t frame_id;  // submit에 준 값
        bool keyframe;
    };

    VideoEncoder(int width, int height, int fps, unsigned bitrate);
    ~VideoEncoder();
    VideoEncoder(const VideoEncoder &) = delete;
    VideoEncoder &operator=(const VideoEncoder &) = delete;

    /* src_phys(NV12 CMM)를 복사해 인코더에 넣는다. 복사 뒤 source_still_valid()가
     * 거짓이면(그동안 덮어써짐) 넣지 않고 false. 인코더 입력 큐가 차 있어도 false. */
    bool submit(unsigned long long src_phys, uint64_t frame_id,
                const std::function<bool()> &source_still_valid);
    /* 나온 패킷을 모두 on_packet으로 넘긴다. 첫 IDR에서 파라미터 세트를 모으면 on_config를
     * 한 번 부른다. 넘긴 패킷 수를 돌려준다. */
    unsigned drain(const std::function<void(const uint8_t *, size_t)> &on_config,
                   const std::function<void(const Packet &)> &on_packet, int timeout_ms = 0);

    uint64_t submitted() const { return submitted_; }
    uint64_t encoded() const { return encoded_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    uint64_t submitted_ = 0;
    uint64_t encoded_ = 0;
};

#endif
