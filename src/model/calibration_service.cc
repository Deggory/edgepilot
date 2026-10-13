#include "model/calibration_service.h"

#include "common/utils_file.h"
#include "common/utils_json.h"
#include "common/utils_math.h"
#include "common/utils_process.h"

#include <sys/stat.h>
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
constexpr float kPersistDeltaRad = 0.01f * 3.14159265358979323846f / 180.0f;
constexpr auto kResetPollInterval = std::chrono::seconds(1);

struct StoredCalibration {
    float rpy[3] = {};
    float spread[3] = {};
    int valid_blocks = 0;
    float height_m = OnlineCalibrator::kHeightInit;  // 없으면(예전 파일) HEIGHT_INIT
};

bool load_stored_calibration(const std::string &path, StoredCalibration *stored, std::string *error) {
    if (!stored) return false;
    const std::string text = read_text_file(path);
    if (text.empty()) return false;
    try {
        float version = 0.0f;
        float valid_blocks = 0.0f;
        std::array<float, 3> rpy{};
        std::array<float, 3> spread{};
        if (!parse_json_float_value(text, "version", &version) ||
            static_cast<int>(std::lround(version)) != kPersistVersion ||
            !parse_json_float_array(text, "rpy_rad", &rpy) ||
            !parse_json_float_value(text, "valid_blocks", &valid_blocks)) {
            if (error) *error = "missing or unsupported fields";
            return false;
        }
        parse_json_float_array(text, "spread_rad", &spread);
        float height = OnlineCalibrator::kHeightInit;
        if (parse_json_float_value(text, "height_m", &height) && std::isfinite(height)) stored->height_m = height;
        for (int i = 0; i < 3; ++i) {
            stored->rpy[i] = rpy[static_cast<size_t>(i)];
            stored->spread[i] = spread[static_cast<size_t>(i)];
        }
        stored->valid_blocks = std::max(0, static_cast<int>(std::lround(valid_blocks)));
        return true;
    } catch (const std::exception &exc) {
        if (error) *error = exc.what();
        return false;
    }
}

bool ensure_params_dir(const std::string &path) {
    if (mkdir(path.c_str(), 0755) == 0 || errno == EEXIST) return true;
    std::fprintf(stderr, "calibration: mkdir %s failed: %s\n", path.c_str(), std::strerror(errno));
    return false;
}

bool save_stored_calibration(const std::string &params_dir, const std::string &path, const float rpy[3],
                             const OnlineCalibrator::Snapshot &snapshot) {
    if (!ensure_params_dir(params_dir)) return false;
    std::ostringstream json;
    json << std::setprecision(9) << "{\n"
         << "  \"version\": " << kPersistVersion << ",\n"
         << "  \"rpy_rad\": [" << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << "],\n"
         << "  \"spread_rad\": [" << snapshot.spread[0] << ", " << snapshot.spread[1] << ", " << snapshot.spread[2]
         << "],\n"
         << "  \"valid_blocks\": " << snapshot.valid_blocks << ",\n"
         << "  \"height_m\": " << snapshot.height_m << "\n"
         << "}\n";
    if (!write_file_atomic(path, json.str())) {
        std::fprintf(stderr, "calibration: write %s failed: %s\n", path.c_str(), std::strerror(errno));
        return false;
    }
    return true;
}

// 광류 블록(flow_calibration.json). 블록 중앙값을 오래된 것부터 담는다. 범위·유한성은 FlowCalibrator::restore가
// 다시 본다.
constexpr int kFlowPersistVersion = 1;
constexpr size_t kFlowMaxStoredBlocks = 64;

bool load_stored_flow(const std::string &path, FlowBlocks *blocks, std::string *error) {
    const std::string text = read_text_file(path);
    if (text.empty()) return false;
    try {
        float version = 0.0f, windows = 0.0f, mount_changes = 0.0f;
        FlowBlocks out;
        if (!parse_json_float_value(text, "version", &version) ||
            static_cast<int>(std::lround(version)) != kFlowPersistVersion ||
            !parse_json_float_list(text, "yaw_rad", kFlowMaxStoredBlocks, &out.yaw) ||
            !parse_json_float_list(text, "pitch_rad", kFlowMaxStoredBlocks, &out.pitch) ||
            out.yaw.size() != out.pitch.size()) {
            if (error) *error = "missing or unsupported fields";
            return false;
        }
        if (parse_json_float_value(text, "windows", &windows) && windows >= 0.0f && std::isfinite(windows))
            out.windows = static_cast<uint32_t>(std::lround(windows));
        if (parse_json_float_value(text, "mount_changes", &mount_changes) && mount_changes >= 0.0f &&
            std::isfinite(mount_changes))
            out.mount_changes = static_cast<uint32_t>(std::lround(mount_changes));
        *blocks = std::move(out);
        return true;
    } catch (const std::exception &exc) {
        if (error) *error = exc.what();
        return false;
    }
}

bool save_stored_flow(const std::string &params_dir, const std::string &path, const FlowBlocks &blocks) {
    if (!ensure_params_dir(params_dir)) return false;
    std::ostringstream json;
    json << std::setprecision(9) << "{\n  \"version\": " << kFlowPersistVersion << ",\n  \"yaw_rad\": [";
    for (size_t i = 0; i < blocks.yaw.size(); ++i) json << (i ? ", " : "") << blocks.yaw[i];
    json << "],\n  \"pitch_rad\": [";
    for (size_t i = 0; i < blocks.pitch.size(); ++i) json << (i ? ", " : "") << blocks.pitch[i];
    json << "],\n  \"windows\": " << blocks.windows << ",\n  \"mount_changes\": " << blocks.mount_changes << "\n}\n";
    if (!write_file_atomic(path, json.str())) {
        std::fprintf(stderr, "calibration: write %s failed: %s\n", path.c_str(), std::strerror(errno));
        return false;
    }
    return true;
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
      manual_override_(config.manual_calibration),
      log_enabled_(config.log_calibration),
      params_dir_(params_dir()),
      calibration_path_(param_path("calibration.json")),
      flow_path_(param_path("flow_calibration.json")),
      reset_request_path_(env_string("EDGEPILOT_CALIBRATION_RESET_PATH", "/dev/shm/edgepilot_calibration_reset")),
      use_flow_(use_flow) {
    fixed_rpy_[0] = config.manual_roll;
    fixed_rpy_[1] = config.manual_pitch;
    fixed_rpy_[2] = config.manual_yaw;

    StoredCalibration stored;
    std::string load_error;
    const bool loaded = load_stored_calibration(calibration_path_, &stored, &load_error);
    if (!loaded && !load_error.empty())
        std::fprintf(stderr, "calibration: ignoring %s: %s\n", calibration_path_.c_str(), load_error.c_str());

    if (manual_override_) {
        restored_ = calibrator_.restore(fixed_rpy_, 5);
        last_snapshot_ = calibrator_.snapshot();
        set_fixed_projection();
        if (restored_ && (!loaded || max_rpy_delta(stored.rpy, fixed_rpy_) > 1e-7f) &&
            save_stored_calibration(params_dir_, calibration_path_, fixed_rpy_, last_snapshot_)) {
            copy_rpy(persisted_rpy_, fixed_rpy_);
            has_persisted_ = true;
            last_persist_ = std::chrono::steady_clock::now();
        } else if (loaded) {
            copy_rpy(persisted_rpy_, stored.rpy);
            has_persisted_ = true;
        }
    } else if (loaded && calibrator_.restore(stored.rpy, stored.valid_blocks, stored.spread, stored.height_m)) {
        restored_ = true;
        copy_rpy(persisted_rpy_, stored.rpy);
        has_persisted_ = true;
        last_persist_ = std::chrono::steady_clock::now();
        if (auto_enabled_) {
            calibrator_.output_rpy(online_rpy_);
            last_snapshot_ = calibrator_.snapshot();
            projection_ = make_projection_state(online_rpy_[0], online_rpy_[1], online_rpy_[2]);
        } else {
            copy_rpy(fixed_rpy_, stored.rpy);
            last_snapshot_ = calibrator_.snapshot();
            set_fixed_projection();
        }
    } else {
        set_fixed_projection();
        last_snapshot_ = calibrator_.snapshot();
    }

    // 광류: 저장해 둔 블록과 시작 설정으로 첫 프레임 워프부터 출처를 정한다. 측정원이 없어도 저장된 추정은
    // 장착의 성질이라 그대로 쓸 수 있다.
    std::string flow_error;
    if (load_stored_flow(flow_path_, &stored_flow_, &flow_error)) {
        FlowCalibrator restored;
        if (restored.restore(stored_flow_)) {
            flow_state_ = restored.state();
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

    persist_thread_ = std::thread(&CalibrationService::persist_loop, this);

    const char *mode = manual_override_ ? "manual"
                                        : (can_apply_online() ? (restored_ ? "auto-pose-restored" : "auto-pose-live")
                                                              : (restored_ ? "stored" : "fixed"));
    std::fprintf(stderr, "calibration mode=%s overlay_auto=%d manual_override=%d\n", mode, can_apply_online() ? 1 : 0,
                 manual_override_ ? 1 : 0);
    const ProjectionState current = projection_;
    std::fprintf(stderr, "projection calibration deg roll=%.3f pitch=%.3f yaw=%.3f params=%s source=%s (setting %s)\n",
                 rad_to_deg(current.roll), rad_to_deg(current.pitch), rad_to_deg(current.yaw),
                 calibration_path_.c_str(), selector_.flow_applied() ? "flow" : "pose", use_flow_ ? "flow" : "pose");
    logged_flow_applied_ = selector_.flow_applied();
}

CalibrationService::~CalibrationService() {
    maybe_persist_flow(std::chrono::steady_clock::now(), true);  // 마지막 광류 블록을 남긴다
    {
        std::lock_guard<std::mutex> lock(persist_mutex_);
        persist_stop_ = true;
    }
    persist_cv_.notify_one();
    if (persist_thread_.joinable()) persist_thread_.join();
}

void CalibrationService::persist_loop() {
    std::unique_lock<std::mutex> lock(persist_mutex_);
    while (true) {
        persist_cv_.wait(lock, [this] { return persist_pending_ || flow_persist_pending_ || persist_stop_; });
        if (!persist_pending_ && !flow_persist_pending_) return;  // 멈출 때 남은 저장은 마저 한다
        if (flow_persist_pending_) {
            const FlowPersistJob flow_job = flow_job_;
            flow_persist_pending_ = false;
            lock.unlock();
            if (flow_job.remove) {
                if (std::remove(flow_path_.c_str()) == 0 || errno == ENOENT)
                    std::fprintf(stderr, "calibration: removed %s\n", flow_path_.c_str());
                else
                    std::fprintf(stderr, "calibration: remove %s failed: %s\n", flow_path_.c_str(),
                                 std::strerror(errno));
            } else if (save_stored_flow(params_dir_, flow_path_, flow_job.blocks)) {
                std::fprintf(stderr, "calibration: saved %s blocks=%zu windows=%u mount_changes=%u\n",
                             flow_path_.c_str(), flow_job.blocks.yaw.size(), flow_job.blocks.windows,
                             flow_job.blocks.mount_changes);
            }
            lock.lock();
            continue;
        }
        const PersistJob job = persist_job_;
        persist_pending_ = false;
        lock.unlock();
        if (job.remove) {
            if (std::remove(calibration_path_.c_str()) == 0 || errno == ENOENT)
                std::fprintf(stderr, "calibration: removed %s\n", calibration_path_.c_str());
            else
                std::fprintf(stderr, "calibration: remove %s failed: %s\n", calibration_path_.c_str(),
                             std::strerror(errno));
        } else if (save_stored_calibration(params_dir_, calibration_path_, job.rpy, job.snapshot))
            std::fprintf(stderr, "calibration: saved %s rpy_deg=(%.3f %.3f %.3f) height=%.3f m validBlocks=%d\n",
                         calibration_path_.c_str(), rad_to_deg(job.rpy[0]), rad_to_deg(job.rpy[1]),
                         rad_to_deg(job.rpy[2]), job.snapshot.height_m, job.snapshot.valid_blocks);
        lock.lock();
    }
}

void CalibrationService::set_fixed_projection() {
    projection_ = make_projection_state(fixed_rpy_[0], fixed_rpy_[1], fixed_rpy_[2]);
}

void CalibrationService::input_rpy(float rpy[3]) const {
    const float *src = can_apply_online() ? applied_rpy_ : fixed_rpy_;
    rpy[0] = src[0];
    rpy[1] = src[1];
    rpy[2] = src[2];
}

// 워프에 쓸 값을 정한다. 수동 보정이면 그 값 그대로, 아니면 출처 선택기가 포즈 보정기 출력과 광류 추정과
// 설정으로 정한다(설정이 포즈면 포즈 보정기 출력 그대로). snapshot의 모든 필드는 그 출처의 것이다
// (ipc_messages.h CalibrationState).
void CalibrationService::apply_selection(float dt_s) {
    if (!can_apply_online()) {
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
    const bool online = can_apply_online();
    out.source = online && selector_.flow_applied() ? 1u : 0u;
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
    {
        std::lock_guard<std::mutex> lock(persist_mutex_);
        flow_job_.blocks = latest_flow_blocks_;
        flow_job_.remove = false;
        flow_persist_pending_ = true;
    }
    persist_cv_.notify_one();
    has_flow_persisted_ = true;
    persisted_mount_changes_ = latest_flow_blocks_.mount_changes;
    last_flow_persist_ = now;
    flow_blocks_dirty_ = false;
}

const char *CalibrationService::mode_name(const OnlineCalibrator::Snapshot &snapshot) const {
    if (!can_apply_online()) return "manual";
    return snapshot.status == CalibrationStatus::Calibrated ? "auto-pose-valid" : "auto-pose-unvalid";
}

// 웹 콘솔의 "캘리브레이션 초기화"는 요청 파일을 만들기만 한다. 마운트를
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
    if (!can_apply_online()) {
        std::fprintf(stderr, "calibration: reset request ignored (manual calibration)\n");
        return;
    }
    reset_online();
}

void CalibrationService::reset_online() {
    calibrator_ = OnlineCalibrator();
    calibrator_.output_rpy(online_rpy_);
    last_snapshot_ = calibrator_.snapshot();
    restored_ = false;
    has_persisted_ = false;
    copy_rpy(persisted_rpy_, online_rpy_);
    last_valid_blocks_ = -1;
    {
        // 아직 쓰지 않은 저장이 있으면 버리고, 파일을 지운다. 새 값은 다시 수렴하면 저장된다.
        std::lock_guard<std::mutex> lock(persist_mutex_);
        persist_job_ = PersistJob{};
        persist_job_.remove = true;
        persist_pending_ = true;
        // 장착을 바꿨다는 뜻이므로 광류 블록도 버린다
        flow_job_ = FlowPersistJob{};
        flow_job_.remove = true;
        flow_persist_pending_ = true;
    }
    persist_cv_.notify_one();
    flow_state_ = FlowCalibrationState{};
    flow_state_.status = 1;
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
    if (!can_apply_online()) {
        apply_selection(dt_s);
        return;
    }

    // 포즈 보정기는 upstream처럼 모델이 자기 값으로 워프된 영상을 봤다고 보고 관측을 자기 값 위에 더한다. 워프가
    // 그 값과 다를 때(광류를 쓰거나 포즈로 돌아오는 중)는 그 가정이 깨지니 멈춰 둔다. 그동안 자기 추정과 저장
    // 파일은 그대로다.
    const bool warp_is_pose = !selector_.flow_applied() && (selector_.flags() & kCalibrationSourceRamping) == 0;
    if (output.has_pose && warp_is_pose) {
        const OnlineCalibrator::UpdateResult result = calibrator_.update(output.pose, v_ego);
        last_snapshot_ = result.snapshot;
        calibrator_.output_rpy(online_rpy_);
        maybe_persist(result);
        maybe_log(result);
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
    {
        std::lock_guard<std::mutex> lock(persist_mutex_);
        copy_rpy(persist_job_.rpy, online_rpy_);
        persist_job_.snapshot = result.snapshot;
        // 초기화가 남긴 삭제 표시를 지운다. 두면 초기화 뒤 저장이 매번 파일 삭제가 되어,
        // 재시작하면 저장소 기본값(upload_to_board.sh가 빈 자리에 채움)으로 돌아간다.
        persist_job_.remove = false;
        persist_pending_ = true;
    }
    persist_cv_.notify_one();
    copy_rpy(persisted_rpy_, online_rpy_);
    has_persisted_ = true;
    last_persist_ = now;
}

void CalibrationService::maybe_log(const OnlineCalibrator::UpdateResult &result) {
    if (!log_enabled_) return;

    const auto now = std::chrono::steady_clock::now();
    const bool first = last_log_.time_since_epoch().count() == 0;
    const bool elapsed = first || now - last_log_ >= std::chrono::seconds(1);
    const bool status_changed =
        result.snapshot.status != last_status_ || result.snapshot.valid_blocks != last_valid_blocks_;
    if (!elapsed && !result.block_completed && !status_changed) return;

    last_log_ = now;
    last_status_ = result.snapshot.status;
    last_valid_blocks_ = result.snapshot.valid_blocks;

    std::fprintf(stderr,
                 "\ncalib mode=%s status=%s accepted=%llu rejected=%llu "
                 "validBlocks=%d blockSamples=%d last=%s rpy_deg=(%.3f %.3f %.3f) "
                 "spread_deg=(%.3f %.3f %.3f) height=%.3f m\n",
                 mode_name(result.snapshot), calibration_status_name(result.snapshot.status),
                 static_cast<unsigned long long>(result.snapshot.accepted_samples),
                 static_cast<unsigned long long>(result.snapshot.rejected_samples), result.snapshot.valid_blocks,
                 result.snapshot.block_sample_count, result.accepted ? "accepted" : "rejected",
                 rad_to_deg(result.snapshot.rpy[0]), rad_to_deg(result.snapshot.rpy[1]),
                 rad_to_deg(result.snapshot.rpy[2]), rad_to_deg(result.snapshot.spread[0]),
                 rad_to_deg(result.snapshot.spread[1]), rad_to_deg(result.snapshot.spread[2]),
                 result.snapshot.height_m);
}
