#include "model/calibration_service.h"

#include "common/utils_file.h"
#include "common/utils_json.h"
#include "common/utils_math.h"
#include "common/utils_process.h"

#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace {

constexpr int kPersistVersion = 1;
constexpr auto kPersistInterval = std::chrono::seconds(60);
constexpr float kPersistDeltaRad = 0.01f * kPiF / 180.0f;
constexpr auto kResetPollInterval = std::chrono::seconds(1);

struct StoredCalibration {
    float rpy[3] = {};
    float spread[3] = {};
    int valid_blocks = 0;
    float height_m = OnlineCalibrator::kHeightInit;  // 없으면(예전 파일) HEIGHT_INIT
};

// 이 코드만 원자적으로(fsync) 쓰는 파일이라 형식이 틀린 값은 생기지 않는다(json_float가 던지면 modeld가 끝난다).
bool load_stored_calibration(const std::string &path, StoredCalibration *stored, std::string *error) {
    const std::string text = read_text_file(path);
    JsonValue root;
    if (text.empty() || !parse_json_object(text, &root, error)) return false;
    float version = 0.0f;
    float valid_blocks = 0.0f;
    std::array<float, 3> rpy{};
    std::array<float, 3> spread{};
    if (!json_float(root, "version", &version) || static_cast<int>(std::lround(version)) != kPersistVersion ||
        !json_float_array(root, "rpy_rad", &rpy) || !json_float(root, "valid_blocks", &valid_blocks)) {
        if (error) *error = "missing or unsupported fields";
        return false;
    }
    json_float_array(root, "spread_rad", &spread);
    json_float(root, "height_m", &stored->height_m);  // 저장소 기본 파일에는 없다
    for (int i = 0; i < 3; ++i) {
        stored->rpy[i] = rpy[static_cast<size_t>(i)];
        stored->spread[i] = spread[static_cast<size_t>(i)];
    }
    stored->valid_blocks = std::max(0, static_cast<int>(std::lround(valid_blocks)));
    return true;
}

std::string calibration_json(const float rpy[3], const OnlineCalibrator::Snapshot &snapshot) {
    std::ostringstream json;
    json << std::setprecision(9) << "{\n"
         << "  \"version\": " << kPersistVersion << ",\n"
         << "  \"rpy_rad\": [" << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << "],\n"
         << "  \"spread_rad\": [" << snapshot.spread[0] << ", " << snapshot.spread[1] << ", " << snapshot.spread[2]
         << "],\n"
         << "  \"valid_blocks\": " << snapshot.valid_blocks << ",\n"
         << "  \"height_m\": " << snapshot.height_m << "\n"
         << "}\n";
    return json.str();
}

// 광류 블록(flow_calibration.json). 블록 중앙값을 오래된 것부터 담는다. 범위·유한성은 FlowCalibrator::restore가
// 다시 본다.
constexpr int kFlowPersistVersion = 1;
constexpr size_t kFlowMaxStoredBlocks = 64;

bool load_stored_flow(const std::string &path, FlowBlocks *blocks, std::string *error) {
    const std::string text = read_text_file(path);
    JsonValue root;
    if (text.empty() || !parse_json_object(text, &root, error)) return false;
    float version = 0.0f, windows = 0.0f, mount_changes = 0.0f;
    FlowBlocks out;
    if (!json_float(root, "version", &version) || static_cast<int>(std::lround(version)) != kFlowPersistVersion ||
        !json_float_list(root, "yaw_rad", kFlowMaxStoredBlocks, &out.yaw) ||
        !json_float_list(root, "pitch_rad", kFlowMaxStoredBlocks, &out.pitch)) {
        if (error) *error = "missing or unsupported fields";
        return false;
    }
    if (json_float(root, "windows", &windows)) out.windows = static_cast<uint32_t>(std::lround(windows));
    if (json_float(root, "mount_changes", &mount_changes))
        out.mount_changes = static_cast<uint32_t>(std::lround(mount_changes));
    *blocks = std::move(out);
    return true;
}

std::string flow_json(const FlowBlocks &blocks) {
    std::ostringstream json;
    json << std::setprecision(9) << "{\n  \"version\": " << kFlowPersistVersion << ",\n  \"yaw_rad\": [";
    for (size_t i = 0; i < blocks.yaw.size(); ++i) json << (i ? ", " : "") << blocks.yaw[i];
    json << "],\n  \"pitch_rad\": [";
    for (size_t i = 0; i < blocks.pitch.size(); ++i) json << (i ? ", " : "") << blocks.pitch[i];
    json << "],\n  \"windows\": " << blocks.windows << ",\n  \"mount_changes\": " << blocks.mount_changes << "\n}\n";
    return json.str();
}

float max_rpy_delta(const float a[3], const float b[3]) {
    return std::max(std::fabs(a[0] - b[0]), std::max(std::fabs(a[1] - b[1]), std::fabs(a[2] - b[2])));
}

void copy_rpy(float dst[3], const float src[3]) {
    for (int i = 0; i < 3; ++i) dst[i] = src[i];
}

}  // namespace

CalibrationService::CalibrationService(const AppConfig &config, bool use_flow)
    : auto_enabled_(config.calibration_auto),
      calibration_path_(param_path("calibration.json")),
      flow_path_(param_path("flow_calibration.json")),
      reset_request_path_(env_string("EDGEPILOT_CALIBRATION_RESET_PATH", "/dev/shm/edgepilot_calibration_reset")),
      use_flow_(use_flow) {
    StoredCalibration stored;
    std::string load_error;
    const bool loaded = load_stored_calibration(calibration_path_, &stored, &load_error);
    if (!loaded && !load_error.empty())
        std::fprintf(stderr, "calibration: ignoring %s: %s\n", calibration_path_.c_str(), load_error.c_str());

    const bool restored =
        loaded && calibrator_.restore(stored.rpy, stored.valid_blocks, stored.spread, stored.height_m);
    last_snapshot_ = calibrator_.snapshot();
    if (restored) {
        copy_rpy(persisted_rpy_, stored.rpy);
        has_persisted_ = true;
        last_persist_ = std::chrono::steady_clock::now();
        if (auto_enabled_) {
            calibrator_.output_rpy(online_rpy_);
            projection_ = make_projection_state(online_rpy_[0], online_rpy_[1], online_rpy_[2]);
        } else {
            copy_rpy(fixed_rpy_, stored.rpy);
            set_fixed_projection();
        }
    } else {
        set_fixed_projection();
    }

    // 광류: 저장해 둔 블록과 시작 설정으로 첫 프레임 워프부터 출처를 정한다. 측정원이 없어도 저장된 추정은
    // 장착의 성질이라 그대로 쓸 수 있다.
    std::string flow_error;
    if (load_stored_flow(flow_path_, &stored_flow_, &flow_error)) {
        FlowCalibrator stored_calibrator;
        if (stored_calibrator.restore(stored_flow_)) {
            flow_state_ = stored_calibrator.state();
            std::fprintf(stderr, "calibration: flow blocks restored n=%u yaw=%.3f pitch=%.3f deg (%s)\n",
                         flow_state_.estimate_blocks, rad_to_deg(flow_state_.yaw), rad_to_deg(flow_state_.pitch),
                         flow_path_.c_str());
        } else {
            std::fprintf(stderr, "calibration: ignoring %s: invalid blocks\n", flow_path_.c_str());
            stored_flow_ = FlowBlocks{};
        }
    } else if (!flow_error.empty()) {
        std::fprintf(stderr, "calibration: ignoring %s: %s\n", flow_path_.c_str(), flow_error.c_str());
    }
    apply_selection(0.0f);

    const char *mode =
        auto_enabled_ ? (restored ? "auto-pose-restored" : "auto-pose-live") : (restored ? "stored" : "fixed");
    std::fprintf(stderr, "calibration mode=%s\n", mode);
    const ProjectionState current = projection_;
    std::fprintf(stderr, "projection calibration deg roll=%.3f pitch=%.3f yaw=%.3f params=%s source=%s (setting %s)\n",
                 rad_to_deg(current.roll), rad_to_deg(current.pitch), rad_to_deg(current.yaw),
                 calibration_path_.c_str(), selector_.flow_applied() ? "flow" : "pose", use_flow_ ? "flow" : "pose");
    logged_flow_applied_ = selector_.flow_applied();
}

CalibrationService::~CalibrationService() {
    maybe_persist_flow(std::chrono::steady_clock::now(), true);  // 마지막 광류 블록을 남긴다(writer_가 마저 쓴다)
}

void CalibrationService::set_fixed_projection() {
    projection_ = make_projection_state(fixed_rpy_[0], fixed_rpy_[1], fixed_rpy_[2]);
}

void CalibrationService::input_rpy(float rpy[3]) const {
    const float *src = auto_enabled_ ? applied_rpy_ : fixed_rpy_;
    rpy[0] = src[0];
    rpy[1] = src[1];
    rpy[2] = src[2];
}

// 워프에 쓸 값을 정한다. 자동 보정이 꺼져 있으면 저장된 값 그대로, 아니면 출처 선택기가 포즈 보정기 출력과 광류 추정과
// 설정으로 정한다(설정이 포즈면 포즈 보정기 출력 그대로). snapshot의 모든 필드는 그 출처의 것이다
// (ipc_messages.h CalibrationState).
void CalibrationService::apply_selection(float dt_s) {
    if (!auto_enabled_) {
        copy_rpy(applied_rpy_, fixed_rpy_);
        applied_snapshot_ = last_snapshot_;
        return;
    }
    selector_.update(online_rpy_, flow_state_, use_flow_, engaged_, dt_s);
    selector_.rpy(applied_rpy_);
    projection_ = make_projection_state(applied_rpy_[0], applied_rpy_[1], applied_rpy_[2]);
    applied_snapshot_ = last_snapshot_;
    copy_rpy(applied_snapshot_.rpy, applied_rpy_);
    if (selector_.flow_applied()) {
        // 광류는 3블록 이상, 블록끼리 맞고 범위 안일 때만 쓴다. 결합 조건(controlsd)도 이 상태를 본다.
        applied_snapshot_.status = CalibrationStatus::Calibrated;
        applied_snapshot_.valid_blocks = static_cast<int>(flow_state_.estimate_blocks);
        applied_snapshot_.spread[1] = flow_state_.sd_pitch;
        applied_snapshot_.spread[2] = flow_state_.sd_yaw;
    }
    if (selector_.flow_applied() != logged_flow_applied_) {
        logged_flow_applied_ = selector_.flow_applied();
        std::fprintf(stderr,
                     "\ncalibration: warp source %s target pitch=%.3f yaw=%.3f deg (pose %.3f %.3f, flow %.3f %.3f)\n",
                     logged_flow_applied_ ? "flow" : "pose",
                     rad_to_deg(logged_flow_applied_ ? flow_state_.pitch : online_rpy_[1]),
                     rad_to_deg(logged_flow_applied_ ? flow_state_.yaw : online_rpy_[2]), rad_to_deg(online_rpy_[1]),
                     rad_to_deg(online_rpy_[2]), rad_to_deg(flow_state_.pitch), rad_to_deg(flow_state_.yaw));
    }
    const bool pending = (selector_.flags() & kCalibrationSourcePending) != 0;
    if (pending != logged_pending_) {
        logged_pending_ = pending;
        if (pending)
            std::fprintf(stderr, "\ncalibration: switching to %s waits for disengage\n", use_flow_ ? "flow" : "pose");
    }
}

CalibrationSourceState CalibrationService::source_state() const {
    CalibrationSourceState out;
    const bool online = auto_enabled_;
    out.source = online && selector_.flow_applied() ? kCalibrationFromFlow : kCalibrationFromPose;
    out.requested = use_flow_ ? 1u : 0u;
    out.flags = online ? selector_.flags() : 0u;
    out.pose_status = static_cast<uint32_t>(last_snapshot_.status);
    out.pose_valid_blocks = last_snapshot_.valid_blocks;
    const float *pose = online ? online_rpy_ : fixed_rpy_;
    out.pose_roll = pose[0];
    out.pose_pitch = pose[1];
    out.pose_yaw = pose[2];
    return out;
}

// 광류 측정원과 주고받는다: 초기화 요청을 보내고, 상태를 읽고(초기화를 처리하기 전의 상태와 블록은 버린다),
// 끝난 블록을 받아 저장할 거리로 둔다.
void CalibrationService::poll_flow_source(std::chrono::steady_clock::time_point now) {
    if (flow_source_) {
        while (flow_resets_sent_ < flow_resets_requested_) {
            flow_source_->request_reset();
            ++flow_resets_sent_;
        }
        const FlowCalibrationState state = flow_source_->state(&flow_resets_seen_);
        if (flow_resets_seen_ >= flow_resets_requested_) {
            flow_state_ = state;
            FlowBlocks blocks;
            if (flow_source_->take_blocks(&blocks)) {
                latest_flow_blocks_ = std::move(blocks);
                flow_blocks_dirty_ = true;
            }
        }
    }
    maybe_persist_flow(now, false);
}

void CalibrationService::maybe_persist_flow(std::chrono::steady_clock::time_point now, bool flush) {
    if (!flow_blocks_dirty_ || latest_flow_blocks_.yaw.empty()) return;
    const bool first = !has_flow_persisted_;
    const bool mount_changed = latest_flow_blocks_.mount_changes != persisted_mount_changes_;
    if (!flush && !first && !mount_changed && now - last_flow_persist_ < kPersistInterval) return;
    writer_.write(flow_path_, flow_json(latest_flow_blocks_));
    std::fprintf(stderr, "calibration: save %s blocks=%zu windows=%u mount_changes=%u\n", flow_path_.c_str(),
                 latest_flow_blocks_.yaw.size(), latest_flow_blocks_.windows, latest_flow_blocks_.mount_changes);
    has_flow_persisted_ = true;
    persisted_mount_changes_ = latest_flow_blocks_.mount_changes;
    last_flow_persist_ = now;
    flow_blocks_dirty_ = false;
}

// 웹 콘솔의 "보정 초기화"는 요청 파일을 만들기만 한다. 마운트를
// 바꾼 뒤 SSH 없이 처음부터 다시 수렴시키려는 것이다. 모델 루프에서 1초에 한 번 tmpfs의
// 파일 하나를 확인할 뿐이라 비용은 없다. 주행 중 초기화는 웹 콘솔이 해제 상태에서만
// 받는다(openpilot의 Reset Calibration처럼).
void CalibrationService::poll_reset_request() {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_reset_poll_ < kResetPollInterval) return;
    last_reset_poll_ = now;
    if (access(reset_request_path_.c_str(), F_OK) != 0) {
        logged_reset_deferred_ = false;
        return;
    }
    if (engaged_) {
        // 웹 콘솔은 해제 상태에서만 요청하지만, 요청과 이 확인 사이에 결합했을 수 있다. 워프가 그 자리에서 0으로
        // 뛰지 않게 해제될 때까지 요청 파일을 둔다.
        if (!logged_reset_deferred_) std::fprintf(stderr, "calibration: reset request waits for disengage\n");
        logged_reset_deferred_ = true;
        return;
    }
    logged_reset_deferred_ = false;
    if (unlink(reset_request_path_.c_str()) != 0 && errno != ENOENT) {
        std::fprintf(stderr, "calibration: unlink %s failed: %s\n", reset_request_path_.c_str(), std::strerror(errno));
        return;
    }
    if (!auto_enabled_) {
        std::fprintf(stderr, "calibration: reset request ignored (EDGEPILOT_CALIB_AUTO=0)\n");
        return;
    }
    reset_online();
}

void CalibrationService::reset_online() {
    calibrator_ = OnlineCalibrator();
    calibrator_.output_rpy(online_rpy_);
    last_snapshot_ = calibrator_.snapshot();
    has_persisted_ = false;
    copy_rpy(persisted_rpy_, online_rpy_);
    // 아직 쓰지 않은 저장은 삭제로 바뀐다. 새 값은 다시 수렴하면 저장된다. 장착을 바꿨다는 뜻이므로 광류 블록도
    // 버린다.
    writer_.remove(calibration_path_);
    writer_.remove(flow_path_);
    flow_state_ = FlowCalibrationState{};
    flow_state_.status = kFlowCalibrationCollecting;
    stored_flow_ = FlowBlocks{};
    latest_flow_blocks_ = FlowBlocks{};
    flow_blocks_dirty_ = false;
    has_flow_persisted_ = false;
    persisted_mount_changes_ = 0;
    ++flow_resets_requested_;
    selector_.snap_to(online_rpy_);
    apply_selection(0.0f);
    std::fprintf(stderr, "calibration: reset requested, recalibrating from scratch (pose and flow)\n");
}

void CalibrationService::update(const ParsedModelOutput &output, float v_ego, bool engaged, bool use_flow) {
    engaged_ = engaged;
    use_flow_ = use_flow;
    poll_reset_request();
    const auto now = std::chrono::steady_clock::now();
    const float dt_s =
        last_update_.time_since_epoch().count() == 0 ? 0.0f : std::chrono::duration<float>(now - last_update_).count();
    last_update_ = now;
    poll_flow_source(now);
    if (!auto_enabled_) {
        apply_selection(dt_s);
        return;
    }

    // 포즈 보정기는 upstream처럼 모델이 자기 값으로 워프된 영상을 봤다고 보고 관측을 자기 값 위에 더한다. 워프가
    // 그 값과 다를 때(광류를 쓰거나 포즈로 돌아오는 중)는 그 가정이 깨지니 멈춰 둔다. 그동안 자기 추정과 저장
    // 파일은 그대로다.
    const bool warp_is_pose = !selector_.flow_applied() && (selector_.flags() & kCalibrationSourceRamping) == 0;
    if (warp_is_pose) {
        const OnlineCalibrator::UpdateResult result = calibrator_.update(output.pose, v_ego);
        last_snapshot_ = result.snapshot;
        calibrator_.output_rpy(online_rpy_);
        maybe_persist(result);
    }
    apply_selection(dt_s);
}

void CalibrationService::maybe_persist(const OnlineCalibrator::UpdateResult &result) {
    if (!result.block_completed || result.snapshot.status != CalibrationStatus::Calibrated) return;

    const auto now = std::chrono::steady_clock::now();
    if (has_persisted_) {
        if (now - last_persist_ < kPersistInterval) return;
        if (max_rpy_delta(persisted_rpy_, online_rpy_) < kPersistDeltaRad) return;
    }
    // 초기화가 남긴 삭제가 아직 쓰이지 않았으면 이 쓰기가 대신한다(BackgroundWriter는 경로마다 마지막 일만 한다).
    writer_.write(calibration_path_, calibration_json(online_rpy_, result.snapshot));
    std::fprintf(stderr, "calibration: save %s rpy_deg=(%.3f %.3f %.3f) height=%.3f m validBlocks=%d\n",
                 calibration_path_.c_str(), rad_to_deg(online_rpy_[0]), rad_to_deg(online_rpy_[1]),
                 rad_to_deg(online_rpy_[2]), result.snapshot.height_m, result.snapshot.valid_blocks);
    copy_rpy(persisted_rpy_, online_rpy_);
    has_persisted_ = true;
    last_persist_ = now;
}
