#include "utils_json.h"
#include "utils_process.h"
#include "utils_time.h"
#include "ipc_channels.h"
#include "maix_venc.h"
#include "recording_writer.h"

#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

/* 주행 녹화기. modeld가 실제로 쓴 프레임(recordFrame)을 하드웨어 H.264 인코더(VENC)로
 * 압축하고, CAN과 모델·제어·판다·학습기 상태를 함께 RecordingWriter 형식(세그먼트별
 * .h264 + 인덱스 + 이벤트 로그)으로 남긴다. 링 슬롯은 IVPS로 인코더 전용 버퍼에 복사한 뒤
 * 슬롯이 그동안 덮어써지지 않았을 때만 인코더에 넣는다(maix_venc.h). 녹화 on/off는
 * params/recording.json을 따른다. */

namespace {

volatile sig_atomic_t g_stop = 0;
constexpr unsigned kRecordingFps = 20;
// recording.json 의 bitrate_bps 가 없을 때의 기본값.
constexpr unsigned kRecordingBitrate = 8000000;
constexpr uint64_t kConfigPollIntervalNs = 250000000ULL;
constexpr uint64_t kMaximumFrameAgeNs = 100000000ULL;

bool read_recording_enabled(const std::string &path, bool fallback) {
  std::ifstream file(path);
  if (!file) return fallback;
  const std::string text((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
  bool enabled = fallback;
  if (!parse_json_bool_value(text, "enabled", &enabled)) {
    throw std::runtime_error("recording config has no 'enabled' value");
  }
  return enabled;
}

/* 인코더는 시작할 때 한 번만 열리므로 비트레이트는 기동 시점 값이다.
 * 웹에서 바꾸면 recordd 재시작부터 적용된다. */
unsigned read_recording_bitrate(const std::string &path, unsigned fallback) {
  std::ifstream file(path);
  if (!file) return fallback;
  const std::string text((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
  int bitrate = static_cast<int>(fallback);
  parse_json_optional_int(text, "bitrate_bps", 1000000, 20000000, &bitrate);
  return static_cast<unsigned>(bitrate);
}

uint64_t file_revision(const std::string &path) {
  struct stat status {};
  if (stat(path.c_str(), &status) != 0) return 0;
#if defined(__APPLE__)
  const uint64_t nanoseconds = status.st_mtimespec.tv_nsec;
#else
  const uint64_t nanoseconds = status.st_mtim.tv_nsec;
#endif
  return static_cast<uint64_t>(status.st_ino) ^
      (static_cast<uint64_t>(status.st_mtime) << 24) ^ nanoseconds ^
      static_cast<uint64_t>(status.st_size);
}

void open_optional_channel(K230LatestChannel &channel, bool *opened,
                           const char *name, size_t size) {
  if (!*opened) *opened = channel.open(name, size, false);
}

}  // namespace

int main() {
  install_stop_signal_handlers(&g_stop);

  try {
    const std::string config_path = k230_param_path("recording.json");
    const std::string recording_root = env_string("K230_RECORD_ROOT", "recordings");
    const unsigned recording_bitrate =
        read_recording_bitrate(config_path, kRecordingBitrate);

    K230FrameRing frame_ring;
    while (!g_stop && !frame_ring.open(false)) {
      std::fprintf(stderr, "recordd: waiting for road AI frame ring\n");
      usleep(250000);
    }
    if (!frame_ring.valid()) return 1;

    K230LatestChannel record_frame_sub;
    if (!record_frame_sub.open(kK230RecordFrameTopic, sizeof(K230RoadAiFrame), true))
      throw std::runtime_error("open recordFrame IPC failed");

    K230CanQueue can_log_sub;
    K230CanQueue sendcan_log_sub;
    if (!can_log_sub.open(kK230CanLogTopic, kK230CanQueueSlots, true) ||
        !sendcan_log_sub.open(kK230SendCanLogTopic, kK230CanQueueSlots, true)) {
      throw std::runtime_error("open CAN recording queues failed");
    }

    VideoEncoder encoder(VideoEncoder::Codec::H264, static_cast<int>(frame_ring.width()),
                         static_cast<int>(frame_ring.height()), kRecordingFps, recording_bitrate);
    RecordingWriter writer(recording_root, k230_params_dir(), frame_ring.width(),
                           frame_ring.height(), kRecordingFps, recording_bitrate,
                           K230VideoCodec::H264);

    /* 인코더는 넣은 순서대로 내놓는다. 패킷의 frame_id로 넣을 때의 메타데이터를 찾는다. */
    std::array<K230RoadAiFrame, 16> in_flight{};
    auto on_config = [&writer](const uint8_t *data, size_t size) {
      writer.set_codec_config(data, size);
    };
    auto on_packet = [&writer, &in_flight](const VideoEncoder::Packet &packet) {
      const K230RoadAiFrame &frame = in_flight[packet.frame_id % in_flight.size()];
      if (frame.frame_id != packet.frame_id) return;
      writer.write_encoded_frame(frame, packet.data, packet.size, packet.keyframe);
    };
    auto drain_encoder = [&]() {
      const uint64_t deadline_ns = k230_now_ns() + 500000000ULL;
      while (encoder.submitted() != encoder.encoded() && k230_now_ns() < deadline_ns)
        encoder.drain(on_config, on_packet, 10);
    };

    K230LatestChannel model_sub;
    K230LatestChannel control_sub;
    K230LatestChannel panda_sub;
    K230LatestChannel learner_sub;
    bool model_open = false;
    bool control_open = false;
    bool panda_open = false;
    bool learner_open = false;
    uint64_t model_seq = 0;
    uint64_t control_seq = 0;
    uint64_t panda_seq = 0;
    uint64_t learner_seq = 0;
    uint64_t frame_seq = 0;
    uint64_t config_revision = UINT64_MAX;
    uint64_t next_config_poll_ns = 0;
    uint64_t next_log_ns = k230_now_ns() + 1000000000ULL;
    uint64_t selected_frames = 0;
    uint64_t dropped_frames = 0;
    uint64_t stale_frames = 0;
    uint64_t frame_sync_failures = 0;
    bool warmed = false;

    while (!g_stop) {
      const uint64_t now_ns = k230_now_ns();
      if (now_ns >= next_config_poll_ns) {
        next_config_poll_ns = now_ns + kConfigPollIntervalNs;
        const uint64_t revision = file_revision(config_path);
        if (revision != config_revision) {
          config_revision = revision;
          try {
            const bool enabled = read_recording_enabled(config_path, false);
            if (!enabled && writer.requested_enabled()) drain_encoder();
            writer.set_enabled(enabled, now_ns);
          } catch (const std::exception &error) {
            std::fprintf(stderr, "recordd: config error: %s\n", error.what());
          }
        }
      }

      const bool need_video_frame = !warmed || writer.requested_enabled();
      K230RoadAiFrame frame;
      if (need_video_frame &&
          record_frame_sub.read_new(&frame_seq, &frame, sizeof(frame), 10)) {
        ++selected_frames;

        const uint64_t frame_now_ns = k230_now_ns();
        const uint64_t age_ns = frame_now_ns >= frame.timestamp_ns
            ? frame_now_ns - frame.timestamp_ns : UINT64_MAX;
        if (frame.slot < frame_ring.slot_count() &&
            frame.width == frame_ring.width() && frame.height == frame_ring.height()) {
          if (age_ns <= kMaximumFrameAgeNs) {
            // 슬롯(CMM)을 IVPS로 인코더 버퍼에 복사한다. CPU는 픽셀을 만지지 않는다.
            uint64_t slot_seq = 0;
            const unsigned long long phys = frame_ring.slot_phys(frame.slot);
            if (phys == 0 || !frame_ring.read_begin(frame.slot, frame.frame_id, &slot_seq)) {
              ++frame_sync_failures;
            } else {
              in_flight[frame.frame_id % in_flight.size()] = frame;
              bool torn = false;
              const bool submitted = encoder.submit(phys, frame.frame_id, [&] {
                torn = !frame_ring.read_still_valid(frame.slot, frame.frame_id, slot_seq);
                return !torn;
              });
              if (submitted) warmed = true;
              else if (torn) ++frame_sync_failures;
              else ++dropped_frames;
            }
          } else {
            ++stale_frames;
          }
        }
      } else if (!need_video_frame) {
        usleep(10000);
      }
      if (encoder.submitted() != encoder.encoded()) encoder.drain(on_config, on_packet);

      K230CanBatch batch;
      while (can_log_sub.pop(&batch)) writer.write_can(K230RecordType::CanRx, batch);
      while (sendcan_log_sub.pop(&batch)) writer.write_can(K230RecordType::CanTx, batch);

      if (writer.requested_enabled()) {
        open_optional_channel(model_sub, &model_open, kK230ModelStateTopic,
                              sizeof(K230ModelState));
        open_optional_channel(control_sub, &control_open, kK230ControlStateTopic,
                              sizeof(K230ControlState));
        open_optional_channel(panda_sub, &panda_open, kK230PandaStateTopic,
                              sizeof(K230PandaState));
        open_optional_channel(learner_sub, &learner_open, kK230LearnerStateTopic,
                              sizeof(K230LearnerState));
        K230ModelState model_state;
        if (model_open && model_sub.read_new(&model_seq, &model_state,
                                             sizeof(model_state), 0)) {
          writer.write_state(K230RecordType::ModelState, model_state.model_timestamp_ns,
                             &model_state, sizeof(model_state));
        }
        K230ControlState control_state;
        if (control_open && control_sub.read_new(&control_seq, &control_state,
                                                 sizeof(control_state), 0)) {
          writer.write_state(K230RecordType::ControlState, control_state.timestamp_ns,
                             &control_state, sizeof(control_state));
        }
        K230PandaState panda_state;
        if (panda_open && panda_sub.read_new(&panda_seq, &panda_state,
                                             sizeof(panda_state), 0)) {
          writer.write_state(K230RecordType::PandaState, panda_state.timestamp_ns,
                             &panda_state, sizeof(panda_state));
        }
        K230LearnerState learner_state;
        if (learner_open && learner_sub.read_new(&learner_seq, &learner_state,
                                                 sizeof(learner_state), 0)) {
          writer.write_state(K230RecordType::LearnerState, learner_state.timestamp_ns,
                             &learner_state, sizeof(learner_state));
        }
      }

      if (now_ns >= next_log_ns) {
        next_log_ns = now_ns + 1000000000ULL;
        std::fprintf(stderr,
                     "recordd: enabled=%u active=%u warmed=%u selected=%llu "
                     "submitted=%llu encoded=%llu dropped=%llu stale=%llu sync=%llu queues=%llu/%llu "
                     "frames=%llu write_queue_drops=%llu moves=%llu%s\n",
                     writer.requested_enabled() ? 1 : 0, writer.active() ? 1 : 0,
                     warmed ? 1 : 0,
                     static_cast<unsigned long long>(selected_frames),
                     static_cast<unsigned long long>(encoder.submitted()),
                     static_cast<unsigned long long>(encoder.encoded()),
                     static_cast<unsigned long long>(dropped_frames),
                     static_cast<unsigned long long>(stale_frames),
                     static_cast<unsigned long long>(frame_sync_failures),
                     static_cast<unsigned long long>(can_log_sub.depth()),
                     static_cast<unsigned long long>(sendcan_log_sub.depth()),
                     static_cast<unsigned long long>(writer.video_frames()),
                     static_cast<unsigned long long>(writer.queue_drops()),
                     static_cast<unsigned long long>(writer.pending_moves()),
                     writer.blocked_for_space() ? " storage-blocked" : "");
      }
    }

    drain_encoder();
    writer.close();
    std::fprintf(stderr, "recordd: stopped\n");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "recordd error: %s\n", error.what());
    return 1;
  }
}
