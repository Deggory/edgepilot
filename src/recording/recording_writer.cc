#include "recording/recording_writer.h"

#include "common/lateral_nn_path.h"
#include "common/utils_process.h"
#include "recording/recorded_can.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>
#if defined(__linux__)
#include <sys/syscall.h>
#endif

#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr uint64_t kSegmentDurationNs = 60ULL * 1'000'000'000ULL;
constexpr uint64_t kStorageCheckIntervalNs = 5ULL * 1'000'000'000ULL;
constexpr uint64_t kMinimumFreeBytes = 5ULL * 1024 * 1024 * 1024;
constexpr unsigned kMinimumFreePercent = 10;
// 공간 부족으로 멈춘 뒤 다시 시작하려면 예비분보다 이만큼 더 비어 있어야 한다. 멈춘 route의 파일이 tmpfs에서
// SD로 옮겨지며 공간을 더 쓰므로, 기준에서 바로 다시 시작하면 시작과 정지를 되풀이한다.
constexpr uint64_t kRetryMarginBytes = 1ULL * 1024 * 1024 * 1024;
constexpr size_t kFileBufferBytes = 1024 * 1024;
// 스테이징(tmpfs, 곧 RAM)이 이보다 커지면 SD가 기록을 따라오지 못하는 것이다. 기록을 멈춰 RAM을 지킨다: 보드의
// /tmp는 224 MB이고 스왑이 없다. 8 Mbit/s에서는 닫힌 세그먼트를 옮기는 동안 최대 약 106 MiB다(옮긴 파일은 다 복사한
// 뒤에야 지운다). 그래서 recordd의 비트레이트 상한은 10 Mbit/s다.
constexpr uint64_t kMaxStagingBytes = 160ULL << 20;
// 스테이징 때문에 멈춘 뒤에는 상한의 절반 밑으로 내려와야 다시 시작한다(SD 쪽 kRetryMarginBytes와 같은 이유: 상한
// 바로 밑에서 다시 시작하면 몇 초 만에 다시 멈추며 짧은 route를 되풀이해 만든다).
constexpr uint64_t kStagingResumeBytes = kMaxStagingBytes / 2;

// 디렉토리 아래 파일들의 크기 합(스테이징은 route/segments/NNN/와 events/의 파일 수십 개).
uint64_t tree_bytes(const std::string &path) {
    struct stat st{};
    if (lstat(path.c_str(), &st) != 0) return 0;
    if (!S_ISDIR(st.st_mode)) return S_ISREG(st.st_mode) ? static_cast<uint64_t>(st.st_size) : 0;
    uint64_t total = 0;
    if (DIR *dir = opendir(path.c_str())) {
        while (const dirent *entry = readdir(dir)) {
            if (std::strcmp(entry->d_name, ".") == 0 || std::strcmp(entry->d_name, "..") == 0) continue;
            total += tree_bytes(path + "/" + entry->d_name);
        }
        closedir(dir);
    }
    return total;
}

bool make_directories(const std::string &path) {
    if (path.empty()) return false;
    std::string current;
    if (path.front() == '/') current = "/";
    size_t start = path.front() == '/' ? 1 : 0;
    while (start <= path.size()) {
        const size_t slash = path.find('/', start);
        const std::string component = path.substr(start, slash - start);
        if (!component.empty()) {
            if (!current.empty() && current.back() != '/') current += '/';
            current += component;
            if (mkdir(current.c_str(), 0775) != 0 && errno != EEXIST) return false;
        }
        if (slash == std::string::npos) break;
        start = slash + 1;
    }
    return true;
}

// 보드에는 tz 데이터가 없어 localtime이 UTC로 떨어진다. route 이름은 주행을
// 되짚는 사람이 읽는 값이므로 KST(DST 없음)로 고정한다.
constexpr std::chrono::hours kKoreaStandardTimeOffset{9};

std::string route_name() {
    const auto now = std::chrono::system_clock::now();
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    const std::time_t wall = std::chrono::system_clock::to_time_t(now + kKoreaStandardTimeOffset);
    std::tm korea{};
    gmtime_r(&wall, &korea);
    std::ostringstream name;
    name << std::put_time(&korea, "%Y-%m-%d--%H-%M-%S") << '-' << std::setw(3) << std::setfill('0')
         << milliseconds.count();
    return name.str();
}

// 닫힌 세그먼트를 SD로 옮긴다. 한 번에 복사하면 더티 페이지가 쌓였다가 한꺼번에 쓰이면서
// SD(저가 카드는 쓰기 수 MB/s)를 몇 초씩 붙잡아 다른 프로세스의 IO가 멈췄다(보드에서 부하
// 10, SSH 한 번에 36초). 1 MB씩 쓰고 바로 fdatasync하며 초당 kMoveBytesPerSecond로
// 제한한다. 녹화 평균(8 Mbps 영상 + CAN·상태 로그 ≈ 1.6 MB/s)보다 넉넉히 빠르다.
// unthrottled가 서 있으면(종료 중) 제한 없이 SD가 내는 속도로 옮긴다.
constexpr size_t kMoveChunkBytes = 1 << 20;
constexpr double kMoveBytesPerSecond = 3.0 * (1 << 20);

bool copy_file(const std::string &source, const std::string &destination,
               const std::atomic<bool> *unthrottled = nullptr) {
    const int in = ::open(source.c_str(), O_RDONLY);
    if (in < 0) return false;
    const int out = ::open(destination.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (out < 0) {
        ::close(in);
        return false;
    }
    std::vector<char> buffer(kMoveChunkBytes);
    bool ok = true;
    const auto start = std::chrono::steady_clock::now();
    size_t copied = 0;
    while (ok) {
        const ssize_t n = ::read(in, buffer.data(), buffer.size());
        if (n <= 0) {
            ok = n == 0;
            break;
        }
        for (ssize_t done = 0; ok && done < n;) {
            const ssize_t w = ::write(out, buffer.data() + done, static_cast<size_t>(n - done));
            ok = w > 0;
            if (ok) done += w;
        }
#if defined(__linux__)
        if (ok) ::fdatasync(out);
#endif
        copied += static_cast<size_t>(n);
        if (unthrottled && *unthrottled) continue;
        const auto due = start + std::chrono::duration<double>(copied / kMoveBytesPerSecond);
        std::this_thread::sleep_until(due);
    }
    ok = ::close(out) == 0 && ok;
    ::close(in);
    return ok;
}

}  // namespace

// ---- StagingMover ----

void StagingMover::recover(const std::string &staging_root, const std::string &root) {
    DIR *stale = opendir(staging_root.c_str());
    if (!stale) return;
    while (dirent *entry = readdir(stale)) {
        const std::string name = entry->d_name;
        if (name == "." || name == "..") continue;
        std::fprintf(stderr, "recordd: recovering staged route %s\n", name.c_str());
        enqueue_tree(staging_root + "/" + name, root + "/" + name);
    }
    closedir(stale);
}

void StagingMover::enqueue_file(std::string from, std::string to) {
    enqueue(Job{false, false, std::move(from), std::move(to)});
}

void StagingMover::enqueue_tree(std::string from, std::string to) {
    enqueue(Job{true, false, std::move(from), std::move(to)});
}

void StagingMover::enqueue_copy(std::string from, std::string to) {
    enqueue(Job{false, true, std::move(from), std::move(to)});
}

void StagingMover::enqueue(Job &&job) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(std::move(job));
        pending_.store(queue_.size());
    }
    cv_.notify_one();
}

void StagingMover::stop() {
    // 남은 파일은 속도 제한 없이 옮긴다. 종료는 기다려 주는 시간이 짧다(매니저 8초, systemd 10초) — 3 MiB/s로는
    // 열린 세그먼트(최대 약 96 MB)를 다 옮기지 못하고 tmpfs에 남은 것은 전원이 꺼지면 사라진다.
    unthrottled_ = true;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_one();
    if (thread_.joinable()) thread_.join();
}

void StagingMover::loop() {
#if defined(__linux__)
    // 옮기기는 급하지 않으니 가장 낮은 IO 우선순위(idle)로 한다: 다른 IO가 없을 때만 쓴다.
    constexpr int kIoprioWhoProcess = 1, kIoprioClassIdle = 3, kIoprioClassShift = 13;
    syscall(SYS_ioprio_set, kIoprioWhoProcess, static_cast<int>(syscall(SYS_gettid)),
            kIoprioClassIdle << kIoprioClassShift);
#endif
    while (true) {
        Job job;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
            if (queue_.empty()) {
                if (stop_) return;
                continue;
            }
            job = std::move(queue_.front());
            queue_.pop_front();
            pending_.store(queue_.size());
        }
        if (job.tree) {
            move_tree(job.from, job.to);
        } else {
            move_file(job.from, job.to, job.keep);
        }
    }
}

void StagingMover::move_file(const std::string &from, const std::string &to, bool keep) {
    const size_t slash = to.rfind('/');
    if (slash != std::string::npos) make_directories(to.substr(0, slash));
    if (!copy_file(from, to, &unthrottled_)) {
        std::fprintf(stderr, "recordd: move failed %s -> %s: %s\n", from.c_str(), to.c_str(), std::strerror(errno));
        return;
    }
    if (!keep) unlink(from.c_str());
}

void StagingMover::move_tree(const std::string &from, const std::string &to) {
    DIR *directory = opendir(from.c_str());
    if (!directory) return;
    while (dirent *entry = readdir(directory)) {
        const std::string name = entry->d_name;
        if (name == "." || name == "..") continue;
        const std::string source = from + "/" + name;
        struct stat info = {};
        if (stat(source.c_str(), &info) != 0) continue;
        if (S_ISDIR(info.st_mode)) {
            move_tree(source, to + "/" + name);
        } else {
            move_file(source, to + "/" + name);
        }
    }
    closedir(directory);
    rmdir(from.c_str());
}

// ---- RecordingWriter ----

RecordingWriter::RecordingWriter(std::string root, std::string params_directory, unsigned width, unsigned height,
                                 unsigned fps, unsigned bitrate)
    : root_(std::move(root)),
      params_directory_(std::move(params_directory)),
      width_(width),
      height_(height),
      fps_(fps),
      bitrate_(bitrate) {
    staging_root_ = env_string("EDGEPILOT_RECORD_STAGING", "/tmp/record_staging");
    // 이전 세션이 route 도중 죽었으면 스테이징 잔여가 tmpfs(램)를 계속
    // 점유한다. 시작할 때 남아 있는 route를 SD로 회수한다.
    mover_.recover(staging_root_, root_);
}

RecordingWriter::~RecordingWriter() { close(); }

void RecordingWriter::enqueue(PendingWrite &&write, bool force) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!force && queue_.size() >= kMaximumPendingWrites) {
            queue_drops_.fetch_add(1);
            return;
        }
        queue_.push_back(std::move(write));
    }
    queue_cv_.notify_one();
}

void RecordingWriter::worker_loop() {
    while (true) {
        PendingWrite write;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !queue_.empty(); });
            write = std::move(queue_.front());
            queue_.pop_front();
        }
        const bool stop = write.kind == PendingWrite::Kind::Stop;
        process(std::move(write));
        if (stop) return;
    }
}

void RecordingWriter::process(PendingWrite &&write) {
    switch (write.kind) {
        case PendingWrite::Kind::Enable:
            if (!start_route(write.timestamp_ns)) {
                requested_enabled_.store(false);
            }
            break;
        case PendingWrite::Kind::Disable:
            close_route(true);
            // 바로 앞의 Enable이 공간 부족으로 거절됐어도 끈 뒤에는 경고를 남기지 않는다.
            storage_blocked_.store(0);
            break;
        case PendingWrite::Kind::CodecConfig:
            codec_config_ = std::move(write.data);
            break;
        case PendingWrite::Kind::EncodedFrame:
            write_encoded_frame_impl(write.frame, write.data.data(), write.data.size(), write.keyframe);
            break;
        case PendingWrite::Kind::Can:
        case PendingWrite::Kind::State:
            write_record_impl(write.record_type, write.timestamp_ns, write.data.data(), write.data.size());
            break;
        case PendingWrite::Kind::Stop:
            close_route(true);
            break;
    }
}

FILE *RecordingWriter::open_buffered(const std::string &path) {
    FILE *file = std::fopen(path.c_str(), "wb");
    if (file) setvbuf(file, nullptr, _IOFBF, kFileBufferBytes);
    return file;
}

uint32_t RecordingWriter::storage_problem(bool resuming) const {
    const uint64_t staged = tree_bytes(staging_root_);
    if (staged > (resuming ? kStagingResumeBytes : kMaxStagingBytes)) {
        if (!resuming)
            std::fprintf(stderr, "recordd: staging holds %llu MiB, the SD card is not keeping up\n",
                         static_cast<unsigned long long>(staged >> 20));
        return kStorageBlockedStaging;
    }
    const uint64_t extra_bytes = resuming ? kRetryMarginBytes : 0;
    if (storage_check_) return storage_check_(extra_bytes) ? 0 : kStorageBlockedSdReserve;
    struct statvfs space{};
    if (statvfs(root_.c_str(), &space) != 0 || space.f_blocks == 0) return kStorageBlockedSdReserve;
    const uint64_t block_size = space.f_frsize ? space.f_frsize : space.f_bsize;
    const uint64_t free_bytes = block_size * space.f_bavail;
    const uint64_t total_bytes = block_size * space.f_blocks;
    const bool reserve =
        free_bytes >= kMinimumFreeBytes + extra_bytes && free_bytes * 100ULL >= total_bytes * kMinimumFreePercent;
    return reserve ? 0 : kStorageBlockedSdReserve;
}

void RecordingWriter::set_enabled(bool enabled, uint64_t now_ns) {
    config_enabled_ = enabled;
    if (!enabled) {
        // 공간 부족으로 거절된 뒤라 이미 꺼져 있어도 경고는 지운다.
        storage_blocked_.store(0);
        if (!requested_enabled_.exchange(false)) return;
        PendingWrite write;
        write.kind = PendingWrite::Kind::Disable;
        enqueue(std::move(write), true);
        return;
    }
    if (requested_enabled_.exchange(true)) return;
    storage_blocked_.store(0);
    PendingWrite write;
    write.kind = PendingWrite::Kind::Enable;
    write.timestamp_ns = now_ns;
    enqueue(std::move(write), true);
}

void RecordingWriter::poll_storage(uint64_t now_ns) {
    if (!config_enabled_ || requested_enabled_.load() || storage_blocked_.load() == 0) return;
    if (now_ns < next_storage_retry_ns_) return;
    next_storage_retry_ns_ = now_ns + kStorageCheckIntervalNs;
    if (const uint32_t problem = storage_problem(true)) {
        storage_blocked_.store(problem);  // 막는 까닭이 바뀌었을 수 있다(스테이징이 빠진 뒤 SD 여유)
        return;
    }
    std::fprintf(stderr, "recordd: storage reserve is back, recording again\n");
    requested_enabled_.store(true);
    storage_blocked_.store(0);
    PendingWrite write;
    write.kind = PendingWrite::Kind::Enable;
    write.timestamp_ns = now_ns;
    enqueue(std::move(write), true);
}

bool RecordingWriter::start_route(uint64_t now_ns) {
    const uint32_t problem = make_directories(root_) ? storage_problem(false) : kStorageBlockedSdReserve;
    if (problem != 0) {
        std::fprintf(stderr, "recordd: recording refused: %s\n",
                     problem == kStorageBlockedStaging ? "tmpfs staging full" : "SD reserve below 5 GiB/10%");
        storage_blocked_.store(problem);
        active_.store(false);
        return false;
    }
    // 활성 route는 tmpfs에 쓰고 mover가 닫힌 파일을 SD로 옮긴다.
    const std::string name = route_name();
    route_path_ = staging_root_ + "/" + name;
    final_route_path_ = root_ + "/" + name;
    if (!make_directories(route_path_ + "/segments")) {
        std::fprintf(stderr, "recordd: create route failed path=%s error=%s\n", route_path_.c_str(),
                     std::strerror(errno));
        return false;
    }
    segment_index_ = 0;
    total_video_frames_.store(0);
    event_records_ = 0;
    route_start_ns_ = now_ns;
    event_chunk_index_ = 0;
    // 이벤트 로그는 route 단위 단일 파일이 아니라 60초 청크다. CAN 로깅만으로
    // ~0.5 MB/s가 쌓이므로 route 단위 파일은 긴 주행에서 tmpfs 스테이징을
    // 가득 채운다(988 MB / 30분). 닫힌 청크는 세그먼트처럼 mover가 옮긴다.
    if (!make_directories(route_path_ + "/events") || !open_event_chunk(now_ns)) {
        std::fprintf(stderr, "recordd: open event log failed: %s\n", std::strerror(errno));
        close_route(false);
        return false;
    }
    snapshot_params();
    write_manifest(false);
    // params·모델 사본과 첫 manifest는 바로 SD로 보낸다. route를 닫을 때까지 tmpfs에만 두면 시동을 끄는 보통의
    // 끝(전원이 바로 끊긴다)에 통째로 사라졌다(2026-09-25~10-14 route 31개 가운데 5개). manifest는 닫을 때 다시 쓰고
    // route째 옮기므로 여기서는 복사만 한다: 옮기기(복사 뒤 지우기)가 늦어 그사이 닫히면 새로 쓴 최종 manifest를
    // 지웠다(빨리 끈 route는 SD에 complete=false가 남았다).
    mover_.enqueue_tree(route_path_ + "/params", final_route_path_ + "/params");
    mover_.enqueue_tree(route_path_ + "/models", final_route_path_ + "/models");
    mover_.enqueue_copy(route_path_ + "/manifest.json", final_route_path_ + "/manifest.json");
    next_storage_check_ns_ = now_ns + kStorageCheckIntervalNs;
    active_.store(true);
    std::fprintf(stderr, "recordd: recording started route=%s staging=%s\n", final_route_path_.c_str(),
                 route_path_.c_str());
    return true;
}

void RecordingWriter::set_codec_config(const uint8_t *data, size_t size) {
    if (!data || size == 0) return;
    PendingWrite write;
    write.kind = PendingWrite::Kind::CodecConfig;
    write.data.assign(data, data + size);
    enqueue(std::move(write));
}

bool RecordingWriter::open_segment(const RoadAiFrame &frame) {
    std::ostringstream number;
    number << std::setw(3) << std::setfill('0') << segment_index_;
    segment_relative_ = "segments/" + number.str();
    const std::string directory = route_path_ + "/" + segment_relative_;
    if (!make_directories(directory)) return false;
    video_file_ = open_buffered(directory + "/road.h264");
    index_file_ = open_buffered(directory + "/frames.bin");
    if (!video_file_ || !index_file_) {
        close_segment();
        return false;
    }
    segment_start_ns_ = frame.timestamp_ns;
    video_offset_ = 0;
    if (!codec_config_.empty()) {
        std::fwrite(codec_config_.data(), 1, codec_config_.size(), video_file_);
        video_offset_ += codec_config_.size();
    }
    FrameIndexHeader header;
    header.record_size = sizeof(FrameIndexRecord);
    header.width = width_;
    header.height = height_;
    header.fps = fps_;
    header.segment_start_ns = frame.timestamp_ns;
    if (std::fwrite(&header, sizeof(header), 1, index_file_) != 1) {
        close_segment();
        return false;
    }
    return true;
}

void RecordingWriter::write_encoded_frame(const RoadAiFrame &frame, const uint8_t *data, size_t size, bool keyframe) {
    if (!requested_enabled_.load() || !data || size == 0) return;
    PendingWrite write;
    write.kind = PendingWrite::Kind::EncodedFrame;
    write.frame = frame;
    write.keyframe = keyframe;
    write.data.assign(data, data + size);
    enqueue(std::move(write));
}

void RecordingWriter::write_encoded_frame_impl(const RoadAiFrame &frame, const uint8_t *data, size_t size,
                                               bool keyframe) {
    if (!event_file_) return;
    if (frame.timestamp_ns >= next_storage_check_ns_) {
        next_storage_check_ns_ = frame.timestamp_ns + kStorageCheckIntervalNs;
        if (const uint32_t problem = storage_problem(false)) {
            storage_blocked_.store(problem);
            requested_enabled_.store(false);
            close_route(true);
            std::fprintf(stderr, "recordd: recording stopped: %s\n",
                         problem == kStorageBlockedStaging ? "tmpfs staging full" : "SD reserve below 5 GiB/10%");
            return;
        }
    }
    if (!video_file_) {
        if (!keyframe || codec_config_.empty() || !open_segment(frame)) return;
    } else if (keyframe && frame.timestamp_ns - segment_start_ns_ >= kSegmentDurationNs) {
        close_segment();
        ++segment_index_;
        if (!open_segment(frame)) return;
    }

    const uint64_t offset = video_offset_;
    if (std::fwrite(data, 1, size, video_file_) != size) {
        std::fprintf(stderr, "recordd: video write failed: %s\n", std::strerror(errno));
        return;
    }
    FrameIndexRecord index;
    index.frame_id = frame.frame_id;
    index.capture_timestamp_ns = frame.timestamp_ns;
    index.encode_index = total_video_frames_.load();
    index.file_offset = offset;
    index.packet_size = static_cast<uint32_t>(size);
    index.flags = keyframe ? kFrameIndexKeyframe : 0U;
    if (std::fwrite(&index, sizeof(index), 1, index_file_) != 1) {
        std::fprintf(stderr, "recordd: frame index write failed: %s\n", std::strerror(errno));
        return;
    }
    video_offset_ += size;
    total_video_frames_.fetch_add(1);
    // 키프레임(1초)마다 영상과 인덱스를 스테이징에 내보낸다(영상 먼저: 인덱스가 아직 없는 바이트를 가리키지 않게).
    // 인덱스(세그먼트에 약 48 KB)는 1 MiB stdio 버퍼를 채우지 못해, recordd가 비정상으로 끝나면 회수한 마지막
    // 세그먼트의 frames.bin이 0바이트로 남아 그 영상을 어떤 리더도 읽지 못했다.
    if (keyframe) {
        std::fflush(video_file_);
        std::fflush(index_file_);
    }
}

bool RecordingWriter::open_event_chunk(uint64_t now_ns) {
    std::ostringstream number;
    number << std::setw(3) << std::setfill('0') << event_chunk_index_;
    event_file_ = open_buffered(route_path_ + "/events/" + number.str() + ".bin");
    if (!event_file_) return false;
    event_chunk_start_ns_ = now_ns;
    EventFileHeader header;
    header.route_start_ns = route_start_ns_;
    if (std::fwrite(&header, sizeof(header), 1, event_file_) != 1) {
        std::fclose(event_file_);
        event_file_ = nullptr;
        return false;
    }
    return true;
}

void RecordingWriter::close_event_chunk() {
    if (!event_file_) return;
    std::fclose(event_file_);
    event_file_ = nullptr;
    std::ostringstream number;
    number << std::setw(3) << std::setfill('0') << event_chunk_index_;
    mover_.enqueue_file(route_path_ + "/events/" + number.str() + ".bin",
                        final_route_path_ + "/events/" + number.str() + ".bin");
}

bool RecordingWriter::write_event_header(RecordType type, uint64_t timestamp_ns, uint32_t payload_size) {
    if (!event_file_) return false;
    if (timestamp_ns >= event_chunk_start_ns_ && timestamp_ns - event_chunk_start_ns_ >= kSegmentDurationNs) {
        close_event_chunk();
        ++event_chunk_index_;
        if (!open_event_chunk(timestamp_ns)) {
            std::fprintf(stderr, "recordd: open event chunk failed: %s\n", std::strerror(errno));
            return false;
        }
    }
    EventRecordHeader header;
    header.timestamp_ns = timestamp_ns;
    header.type = static_cast<uint16_t>(type);
    header.payload_size = payload_size;
    return std::fwrite(&header, sizeof(header), 1, event_file_) == 1;
}

// 배치를 큐에 넣기 전에 디스크 형식(recorded_can.h)으로 직렬화한다. 이후 경로는
// 상태 스냅샷과 같다.
void RecordingWriter::write_can(RecordType type, const CanBatch &batch) {
    if (!requested_enabled_.load()) return;
    PendingWrite write;
    write.kind = PendingWrite::Kind::Can;
    write.record_type = type;
    write.timestamp_ns = batch.timestamp_ns;
    write.data = encode_recorded_can(batch);
    enqueue(std::move(write));
}

void RecordingWriter::write_state(RecordType type, uint64_t timestamp_ns, const void *data, size_t size) {
    if (!requested_enabled_.load()) return;
    PendingWrite write;
    write.kind = PendingWrite::Kind::State;
    write.record_type = type;
    write.timestamp_ns = timestamp_ns;
    write.data.assign(static_cast<const uint8_t *>(data), static_cast<const uint8_t *>(data) + size);
    enqueue(std::move(write));
}

void RecordingWriter::write_record_impl(RecordType type, uint64_t timestamp_ns, const void *data, size_t size) {
    if (!event_file_) return;
    if (!write_event_header(type, timestamp_ns, static_cast<uint32_t>(size))) return;
    if (std::fwrite(data, 1, size, event_file_) == size) ++event_records_;
}

void RecordingWriter::close_segment() {
    const bool had_files = video_file_ != nullptr || index_file_ != nullptr;
    if (video_file_) std::fclose(video_file_);
    if (index_file_) std::fclose(index_file_);
    video_file_ = nullptr;
    index_file_ = nullptr;
    segment_start_ns_ = 0;
    video_offset_ = 0;
    if (had_files && !segment_relative_.empty()) {
        const std::string video = "/road.h264";
        for (const std::string &file : {video, std::string("/frames.bin")}) {
            mover_.enqueue_file(route_path_ + "/" + segment_relative_ + file,
                                final_route_path_ + "/" + segment_relative_ + file);
        }
    }
    segment_relative_.clear();
}

void RecordingWriter::write_manifest(bool complete) const {
    std::ofstream manifest(route_path_ + "/manifest.json", std::ios::trunc);
    manifest << "{\n"
             << "  \"version\": " << kRecordingVersion << ",\n"
             << "  \"complete\": " << (complete ? "true" : "false") << ",\n"
             << "  \"video_codec\": \"h264\",\n"
             << "  \"width\": " << width_ << ",\n"
             << "  \"height\": " << height_ << ",\n"
             << "  \"fps\": " << fps_ << ",\n"
             << "  \"bitrate\": " << bitrate_ << ",\n"
             << "  \"segment_seconds\": " << kSegmentDurationNs / 1'000'000'000ULL << ",\n"
             << "  \"video_frames\": " << total_video_frames_.load() << ",\n"
             << "  \"event_records\": " << event_records_ << "\n"
             << "}\n";
}

void RecordingWriter::snapshot_params() const {
    DIR *directory = opendir(params_directory_.c_str());
    if (!directory) return;
    const std::string destination = route_path_ + "/params";
    make_directories(destination);
    while (dirent *entry = readdir(directory)) {
        const std::string name = entry->d_name;
        if (name.empty() || name[0] == '.') continue;
        // 설정(.json)과 torqued 캐시(.bin, 약 88 KB). 캐시가 있어야 재생 도구가 보드와 같은 torqued에서 시작한다.
        const bool json = name.size() >= 5 && name.substr(name.size() - 5) == ".json";
        if (!json && name != "live_torque_parameters.bin") continue;
        copy_file(params_directory_ + "/" + name, destination + "/" + name);
    }
    closedir(directory);
    // controlsd가 쓰는 학습한 토크 모델(고른 것)도 남긴다. 다시 학습한 뒤에도 이 주행을 그때 모델로 재생한다
    // (replay_controls --lateral-nn route/models/lateral_nn.json). 고르기 자체는 위의 params/lateral_nn.json이다.
    const std::string model = lateral_nn_choice(params_directory_).path;
    if (!model.empty() && access(model.c_str(), R_OK) == 0 && make_directories(route_path_ + "/models"))
        copy_file(model, route_path_ + "/models/lateral_nn.json");
}

void RecordingWriter::close_route(bool complete) {
    if (route_path_.empty()) return;  // 이벤트 청크는 route 안에서만 열린다
    close_segment();
    close_event_chunk();
    write_manifest(complete);
    std::fprintf(stderr, "recordd: recording stopped route=%s frames=%llu events=%llu\n", final_route_path_.c_str(),
                 static_cast<unsigned long long>(total_video_frames_.load()),
                 static_cast<unsigned long long>(event_records_));
    // 남은 route 파일(이벤트 청크, manifest, params, 마지막 세그먼트)을 전부 SD로 옮긴다. mover는 순서대로 처리하므로
    // 앞선 파일 이동이 끝난 뒤 잔여만 쓸어 담는다.
    mover_.enqueue_tree(route_path_, final_route_path_);
    route_path_.clear();
    final_route_path_.clear();
    segment_index_ = 0;
    active_.store(false);
}

void RecordingWriter::close() {
    if (!worker_.joinable()) return;
    requested_enabled_.store(false);
    PendingWrite write;
    write.kind = PendingWrite::Kind::Stop;
    enqueue(std::move(write), true);
    worker_.join();
    mover_.stop();
}
