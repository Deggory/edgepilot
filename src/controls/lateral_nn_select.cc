#include "controls/lateral_nn_select.h"

#include "common/utils_process.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <utility>

namespace {

constexpr uint64_t kCheckIntervalNs = 1'000'000'000ULL;

std::string json_string(const std::string &text) {
    std::string out = "\"";
    for (char c : text) {
        if (c == '"' || c == '\\') out += '\\';
        if (static_cast<unsigned char>(c) < 0x20) continue;
        out += c;
    }
    return out + "\"";
}

}  // namespace

LateralNnSelector::LateralNnSelector(std::string status_path, Choose choose)
    : status_path_(std::move(status_path)), choose_(std::move(choose)) {}

LateralNnSelector::~LateralNnSelector() {
    if (pending_.valid()) pending_.wait();
}

std::string LateralNnSelector::default_status_path() {
    return env_string("EDGEPILOT_LATERAL_NN_STATUS", "/dev/shm/edgepilot_lateral_nn_status.json");
}

LateralNnSelector::Attempt LateralNnSelector::load(LateralNnChoice choice) {
    Attempt attempt;
    attempt.choice = std::move(choice);
    if (attempt.choice.path.empty()) {
        attempt.error = attempt.choice.error;
        return attempt;
    }
    attempt.stamp = file_stamp(attempt.choice.path);
    auto model = std::make_shared<LateralNnModel>();
    if (model->load(attempt.choice.path, &attempt.error))
        attempt.model = std::move(model);
    else
        attempt.error = attempt.choice.name + ": " + attempt.error;
    return attempt;
}

bool LateralNnSelector::same_as_wanted(const LateralNnChoice &choice, const FileStamp &stamp) const {
    return choice.path == wanted_path_ && !(stamp != wanted_stamp_);
}

std::shared_ptr<const LateralNnModel> LateralNnSelector::load_now() {
    const Attempt attempt = load(choose_());
    finish(attempt, true);
    return attempt.model;
}

bool LateralNnSelector::poll(uint64_t now_ns, std::shared_ptr<const LateralNnModel> *model) {
    if (pending_.valid()) {
        if (pending_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return false;
        const Attempt attempt = pending_.get();
        finish(attempt, false);
        if (!attempt.model) return false;
        *model = attempt.model;
        return true;
    }
    if (now_ns < next_check_ns_) return false;
    next_check_ns_ = now_ns + kCheckIntervalNs;
    LateralNnChoice choice = choose_();
    const FileStamp stamp = choice.path.empty() ? FileStamp{} : file_stamp(choice.path);
    if (same_as_wanted(choice, stamp)) return false;
    wanted_path_ = choice.path;
    wanted_stamp_ = stamp;
    pending_ = std::async(std::launch::async, &LateralNnSelector::load, std::move(choice));
    return false;
}

void LateralNnSelector::finish(const Attempt &attempt, bool startup) {
    wanted_path_ = attempt.choice.path;
    wanted_stamp_ = attempt.stamp;
    selected_name_ = attempt.choice.name;
    if (attempt.model) {
        loaded_name_ = attempt.choice.name;
        loaded_path_ = attempt.choice.path;
        loaded_inputs_ = attempt.model->input_count();
        last_error_.clear();
        if (!startup)
            std::fprintf(stderr, "controlsd: lateral nn switched to %s (%s, %zu inputs)\n", loaded_name_.c_str(),
                         loaded_path_.c_str(), loaded_inputs_);
    } else {
        last_error_ = attempt.error;
        if (!startup)
            std::fprintf(stderr, "controlsd: lateral nn %s refused: %s; keeping %s\n", attempt.choice.name.c_str(),
                         attempt.error.c_str(), loaded_name_.empty() ? "none" : loaded_name_.c_str());
    }
    write_status();
}

void LateralNnSelector::write_status() const {
    if (status_path_.empty()) return;
    char tail[96];
    std::snprintf(tail, sizeof(tail), ", \"inputs\": %zu, \"unix_time\": %lld}\n", loaded_inputs_,
                  static_cast<long long>(std::time(nullptr)));
    const std::string text = "{\"selected\": " + json_string(selected_name_) +
                             ", \"loaded\": " + json_string(loaded_name_) + ", \"path\": " + json_string(loaded_path_) +
                             ", \"error\": " + json_string(last_error_) + tail;
    if (!write_file_atomic(status_path_, text))
        std::fprintf(stderr, "controlsd: cannot write %s\n", status_path_.c_str());
}
