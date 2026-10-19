#pragma once

// 녹화 이벤트 로그(events/NNN.bin) 읽기. 진단 도구와 replayd가 같이 쓴다. 파일 머리(EDGELOG1, 지금 녹화 버전)를
// 확인하고 header_size 뒤부터 레코드를 차례로 돌려준다. 끊긴 꼬리(tmpfs가 차서 0으로 채워진 구간,
// 전원이 끊긴 마지막 청크)를 만나면 재동기화하지 않고 거기서 끝낸다: 타입이 범위 밖이거나 길이가
// 1 MiB를 넘거나 레코드 머리·페이로드가 모자라면 truncated()가 참이다.

#include "recording/recording_format.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

class EventLogReader {
public:
    explicit EventLogReader(const std::string &path) : file_(path, std::ios::binary) {
        file_.read(reinterpret_cast<char *>(&header_), sizeof(header_));
        ok_ = static_cast<bool>(file_) && std::memcmp(header_.magic, kEventLogMagic, 8) == 0 &&
              header_.version == kRecordingVersion && header_.header_size >= sizeof(EventFileHeader);
        offset_ = header_.header_size;
        if (ok_) file_.seekg(offset_);
    }

    bool ok() const { return ok_; }  // 이벤트 로그 머리가 맞다(지금 녹화 버전)

    // 다음 레코드와 페이로드. 끝났거나 끊겼으면 false.
    bool next(EventRecordHeader *record, std::vector<char> *payload) {
        if (!ok_ || done_) return false;
        // 레코드 머리를 다 못 읽었으면: 남은 바이트가 없으면 정상 끝, 있으면 중간에 끊긴 것
        if (!file_.read(reinterpret_cast<char *>(record), sizeof(*record))) return finish(file_.gcount() > 0);
        if (record->type < 1 || record->type > kLastRecordType || record->payload_size > kMaxPayloadBytes)
            return finish(true);
        payload->resize(record->payload_size);
        if (!file_.read(payload->data(), record->payload_size)) return finish(true);
        offset_ += sizeof(*record) + record->payload_size;
        return true;
    }

    bool truncated() const { return truncated_; }
    // 끊긴 레코드의 시작 바이트(truncated()일 때)
    long long truncated_at() const { return truncated_at_; }

private:
    static constexpr uint32_t kMaxPayloadBytes = 1U << 20;

    // tellg는 macOS libc++에서 레코드 읽기보다 비싸서(경로 하나에 0.75초) 오프셋을 직접 센다.
    bool finish(bool truncated) {
        done_ = true;
        truncated_ = truncated;
        truncated_at_ = static_cast<long long>(offset_);
        return false;
    }

    std::ifstream file_;
    EventFileHeader header_{};
    bool ok_ = false;
    bool done_ = false;
    bool truncated_ = false;
    uint64_t offset_ = 0;  // 다음 레코드의 시작 바이트
    long long truncated_at_ = -1;
};

// 고정 크기 상태 레코드(ModelState·ControlState·PandaState·LearnerState·LocalizationState)를 out에 옮긴다. 크기가 다르면 false.
template <typename T>
bool read_record(const std::vector<char> &payload, T *out) {
    if (payload.size() != sizeof(T)) return false;
    std::memcpy(out, payload.data(), sizeof(T));
    return true;
}

// 다 읽은(또는 열지 못한) 이벤트 파일의 문제를 stderr에 알린다: 이벤트 파일이 아니거나 꼬리가 끊겼다. prefix는
// 줄 앞에 붙인다. 읽지 못한 파일이면 false.
inline bool report_event_file_end(const std::string &path, const EventLogReader &reader, const char *prefix = "") {
    if (!reader.ok()) {
        std::fprintf(stderr, "%scannot read %s: not a version %u event log\n", prefix, path.c_str(), kRecordingVersion);
        return false;
    }
    if (reader.truncated())
        std::fprintf(stderr, "%s%s: truncated at %lld bytes\n", prefix, path.c_str(), reader.truncated_at());
    return true;
}
