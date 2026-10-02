#pragma once

/* 녹화 이벤트 로그(events/NNN.bin) 읽기. 진단 도구와 replayd가 같이 쓴다. 파일 머리(K230LOG1)를
 * 확인하고 header_size 뒤부터 레코드를 차례로 돌려준다. 끊긴 꼬리(tmpfs가 차서 0으로 채워진 구간,
 * 전원이 끊긴 마지막 청크)를 만나면 재동기화하지 않고 거기서 끝낸다: 타입이 범위 밖이거나 길이가
 * 1 MiB를 넘거나 레코드 머리·페이로드가 모자라면 truncated()가 참이다. */

#include "recording_format.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

class EventLogReader {
public:
  explicit EventLogReader(const std::string &path) : file_(path, std::ios::binary) {
    file_.read(reinterpret_cast<char *>(&header_), sizeof(header_));
    ok_ = static_cast<bool>(file_) && std::memcmp(header_.magic, "K230LOG1", 8) == 0 &&
          header_.header_size >= sizeof(EventFileHeader);
    if (ok_) file_.seekg(header_.header_size);
  }

  bool ok() const { return ok_; }  // 이벤트 로그 머리가 맞다
  uint32_t version() const { return header_.version; }
  const EventFileHeader &header() const { return header_; }

  /* 다음 레코드와 페이로드. 끝났거나 끊겼으면 false. */
  bool next(EventRecordHeader *record, std::vector<char> *payload) {
    if (!ok_ || done_) return false;
    const std::streamoff at = file_.tellg();
    // 레코드 머리를 다 못 읽었으면: 남은 바이트가 없으면 정상 끝, 있으면 중간에 끊긴 것
    if (!file_.read(reinterpret_cast<char *>(record), sizeof(*record))) return finish(file_.gcount() > 0, at);
    if (record->type < 1 || record->type > kLastRecordType || record->payload_size > kMaxPayloadBytes)
      return finish(true, at);
    payload->resize(record->payload_size);
    if (!file_.read(payload->data(), record->payload_size)) return finish(true, at);
    return true;
  }

  bool truncated() const { return truncated_; }
  // 끊긴 레코드의 시작 바이트(truncated()일 때)
  long long truncated_at() const { return truncated_at_; }

private:
  static constexpr uint32_t kMaxPayloadBytes = 1U << 20;

  bool finish(bool truncated, std::streamoff at) {
    done_ = true;
    truncated_ = truncated;
    truncated_at_ = static_cast<long long>(at);
    return false;
  }

  std::ifstream file_;
  EventFileHeader header_{};
  bool ok_ = false;
  bool done_ = false;
  bool truncated_ = false;
  long long truncated_at_ = -1;
};
