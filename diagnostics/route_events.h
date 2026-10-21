#pragma once

// 재생 도구 공용: 녹화 이벤트 파일들(events/NNN.bin, 순서대로)을 기록 시각 순으로 흘려 읽는다(TimeOrderedEvents).
// 이벤트 파일이 아닌 경로와 끊긴 꼬리를 알리고, 못 연 파일이 있으면 exit_code()가 1이다. 예전에는 도구마다 파일
// 순서로 읽는 루프를 따로 두어, 경로에 오타가 있어도 'n=0 rmse=nan'을 찍고 성공으로 끝났고 PandaState를 최대
// 330 ms 늦게 읽었으며(10-16 route), replay_controls는 route 전체를 메모리에 올렸다(4.6 GB route에서 RSS 2.8 GB).

#include "recording/time_ordered_events.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

// 레코드 머리의 타입과 기록 시각(s). 도구마다 캐스트를 되풀이하지 않게 둔다.
inline RecordType record_type(const EventRecordHeader &header) { return static_cast<RecordType>(header.type); }
inline double record_time_s(const EventRecordHeader &header) { return static_cast<double>(header.timestamp_ns) * 1e-9; }

class RouteEvents {
public:
    // window_ns: 파일 순서와 기록 시각이 이만큼까지 어긋난 레코드를 바로잡는다(recordd는 상태 스냅샷을 쌓인 CAN 뒤에
    // 쓴다). route의 첫 PandaState가 1초쯤 앞서 있는 녹화가 있어 넉넉히 둔다.
    explicit RouteEvents(std::vector<std::string> paths, uint64_t window_ns = 2'000'000'000ULL)
        : events_(std::move(paths), window_ns) {
        events_.on_file_end = [this](const std::string &path, const EventLogReader &reader) {
            if (!report_event_file_end(path, reader)) ++unreadable_;
        };
    }
    RouteEvents(const RouteEvents &) = delete;
    RouteEvents &operator=(const RouteEvents &) = delete;

    // 다음 레코드(기록 시각 순, 같은 시각이면 파일 순서). 끝났으면 false.
    bool next(TimeOrderedEvents::Record *out) { return events_.next(out); }
    // 도구의 종료 코드: 못 연 파일이 있으면 1
    int exit_code() const { return unreadable_ > 0 ? 1 : 0; }

private:
    TimeOrderedEvents events_;
    int unreadable_ = 0;
};
