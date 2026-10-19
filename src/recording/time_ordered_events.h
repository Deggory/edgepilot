#pragma once

// 녹화 이벤트 로그 파일들(events/NNN.bin, 순서대로)을 기록 시각 순으로 읽는다. recordd는 쌓인 CAN을 먼저 쓰고 상태
// 스냅샷(각자 발행 시각)을 그 뒤에 써서 파일 순서는 시각 순이 아니다(2026-10-15 경로에서 ControlState가 최대 14 ms,
// ModelState가 21 ms, Localization이 39 ms 늦게 나온다). window_ns 안에서만 다시 늘어놓아 경로 전체를 메모리에
// 올리지 않는다. 시각이 같으면 파일 순서를 지킨다.

#include "recording/event_log_reader.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class TimeOrderedEvents {
public:
    struct Record {
        EventRecordHeader header{};
        std::vector<char> payload;
        size_t file = 0;  // 그 파일이 paths에서 몇 번째인가
    };

    explicit TimeOrderedEvents(std::vector<std::string> paths, uint64_t window_ns = 500'000'000ULL)
        : paths_(std::move(paths)), window_ns_(window_ns) {}

    // 파일을 다 읽었거나 열지 못했을 때 부른다(경로와 그 리더: ok(), truncated()로 알린다).
    std::function<void(const std::string &, const EventLogReader &)> on_file_end;

    // 다음 레코드(시각 순). 끝났으면 false.
    bool next(Record *out) {
        // 가장 이른 레코드는 가장 늦게 읽은 시각보다 창만큼 앞서야(뒤에 더 이른 것이 올 수 없어야) 낸다
        while (heap_.empty() || heap_.front().timestamp_ns + window_ns_ > newest_ns_) {
            if (!read_one()) break;
        }
        if (heap_.empty()) return false;
        std::pop_heap(heap_.begin(), heap_.end(), Later{});
        *out = std::move(heap_.back().record);
        heap_.pop_back();
        return true;
    }

private:
    struct Pending {
        uint64_t timestamp_ns = 0;
        uint64_t sequence = 0;
        Record record;
    };
    struct Later {
        bool operator()(const Pending &a, const Pending &b) const {
            return a.timestamp_ns != b.timestamp_ns ? a.timestamp_ns > b.timestamp_ns : a.sequence > b.sequence;
        }
    };

    // 레코드 하나를 더 읽어 쌓는다. 모든 파일이 끝났으면 false.
    bool read_one() {
        while (true) {
            if (!reader_) {
                if (file_ >= paths_.size()) return false;
                reader_ = std::make_unique<EventLogReader>(paths_[file_]);
            }
            Pending pending;
            if (reader_->ok() && reader_->next(&pending.record.header, &pending.record.payload)) {
                pending.record.file = file_;
                pending.timestamp_ns = pending.record.header.timestamp_ns;
                pending.sequence = sequence_++;
                newest_ns_ = std::max(newest_ns_, pending.timestamp_ns);
                heap_.push_back(std::move(pending));
                std::push_heap(heap_.begin(), heap_.end(), Later{});
                return true;
            }
            if (on_file_end) on_file_end(paths_[file_], *reader_);
            reader_.reset();
            ++file_;
        }
    }

    std::vector<std::string> paths_;
    uint64_t window_ns_;
    size_t file_ = 0;
    std::unique_ptr<EventLogReader> reader_;
    std::vector<Pending> heap_;
    uint64_t newest_ns_ = 0;
    uint64_t sequence_ = 0;
};
