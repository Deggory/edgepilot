#include "model/lane_plugin.h"

#include "common/utils_math.h"

#include <dlfcn.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fstream>
#include <utility>

namespace {

bool finite_in(float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; }

// 점 하나가 쓸 만한지. 화소는 프레임 밖으로 조금 나갈 수 있지만(외삽), LaneState의 1/4 화소 int16에 들어가야 한다.
bool point_ok(const EpLanePoint &p) {
    constexpr float kLimit = 8000.0f;
    return finite_in(p.x, -kLimit, kLimit) && finite_in(p.y, -kLimit, kLimit) && finite_in(p.width, 0.0f, kLimit) &&
           finite_in(p.edge_x, -kLimit, kLimit);
}

bool points_ok(const EpLanePoint *points, uint32_t count) {
    if (count > EP_LANE_MAX_POINTS) return false;
    for (uint32_t i = 0; i < count; ++i)
        if (!point_ok(points[i])) return false;
    return true;
}

// 구조를 보고 신뢰도는 0..1로 자른다. 어긋나면 false.
bool check_result(EpLaneResult *r) {
    if (r->line_count > EP_LANE_MAX_LINES || r->boundary_count > EP_LANE_MAX_BOUNDARIES) return false;
    for (uint32_t i = 0; i < r->line_count; ++i) {
        EpLaneLine &l = r->lines[i];
        if (l.slot < EP_LANE_EGO_LEFT || l.slot > EP_LANE_RIGHT_NEXT || l.shape < EP_LANE_SOLID ||
            l.shape > EP_LANE_BOTTS || l.double_line < EP_LANE_SINGLE || l.double_line > EP_LANE_DOUBLE_BOTTS ||
            l.color < EP_LANE_WHITE || l.color > EP_LANE_COLOR_UNKNOWN || !std::isfinite(l.confidence) ||
            !points_ok(l.points, l.point_count))
            return false;
        l.confidence = clamp_float(l.confidence, 0.0f, 1.0f);
    }
    for (uint32_t i = 0; i < r->boundary_count; ++i) {
        EpLaneBoundary &b = r->boundaries[i];
        if (b.slot < EP_BOUNDARY_NEAR_LEFT || b.slot > EP_BOUNDARY_FAR_RIGHT || b.type < EP_BOUNDARY_WALL ||
            b.type > EP_BOUNDARY_FLAT || !std::isfinite(b.confidence) || !points_ok(b.points, b.point_count))
            return false;
        b.confidence = clamp_float(b.confidence, 0.0f, 1.0f);
    }
    if (!std::isfinite(r->vanishing_x) || !std::isfinite(r->vanishing_y) || !std::isfinite(r->vanishing_confidence))
        return false;
    r->vanishing_confidence = clamp_float(r->vanishing_confidence, 0.0f, 1.0f);
    return true;
}

}  // namespace

LanePlugin::LanePlugin(void *handle, const EpLanePlugin *api, std::string dir)
    : handle_(handle), api_(api), dir_(std::move(dir)) {}

LanePlugin::~LanePlugin() {
    if (handle_) dlclose(handle_);
}

std::unique_ptr<LanePlugin> LanePlugin::open(const std::string &path, std::string *why) {
    auto fail = [why](const std::string &reason) {
        if (why) *why = reason;
        return std::unique_ptr<LanePlugin>();
    };
    void *handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!handle) {
        const char *error = dlerror();
        return fail(error ? error : "dlopen failed: " + path);
    }
    auto entry = reinterpret_cast<EpLanePluginEntry>(dlsym(handle, EP_LANE_PLUGIN_ENTRY));
    // 계약은 예외를 밖으로 던지지 않는 것이지만, 던지면 modeld가 내려가며 크래시 가드 기록까지 지우므로 여기서 받는다
    const EpLanePlugin *api = nullptr;
    std::string reason;
    try {
        api = entry ? entry() : nullptr;
    } catch (...) {
        reason = "the plugin entry threw";
    }
    if (!reason.empty()) {
    } else if (!entry) {
        reason = std::string("no ") + EP_LANE_PLUGIN_ENTRY + " in " + path;
    } else if (!api) {
        reason = "the plugin entry returned no table";
    } else if (api->abi_version != EP_LANE_PLUGIN_ABI_VERSION) {
        reason = "plugin ABI " + std::to_string(api->abi_version) + ", modeld expects " +
                 std::to_string(EP_LANE_PLUGIN_ABI_VERSION);
    } else if (!api->decode || !api->name || !api->model_file || !api->model_file[0] ||
               std::strchr(api->model_file, '/')) {
        reason = "the plugin table lacks decode, name or a plain model file name";
    } else if (!finite_in(api->roi[0], 0.0f, 1.0f) || !finite_in(api->roi[1], 0.0f, 1.0f) ||
               !finite_in(api->roi[2], 0.0f, 1.0f) || !finite_in(api->roi[3], 0.0f, 1.0f) ||
               api->roi[2] <= api->roi[0] || api->roi[3] <= api->roi[1]) {
        reason = "the plugin ROI is not inside the frame";
    }
    if (!reason.empty()) {
        dlclose(handle);
        return fail(reason);
    }
    const size_t slash = path.find_last_of('/');
    std::string dir = slash == std::string::npos ? "." : path.substr(0, slash);
    if (dir.empty()) dir = "/";
    return std::unique_ptr<LanePlugin>(new LanePlugin(handle, api, std::move(dir)));
}

EpLaneGeometry LanePlugin::geometry(int image_width, int image_height, int net_width, int net_height) const {
    auto even = [](long v) { return static_cast<int32_t>(v & ~1L); };
    const float *roi = api_->roi;
    const int32_t right = std::min(even(std::lround(image_width * roi[2])), even(image_width));
    const int32_t bottom = std::min(even(std::lround(image_height * roi[3])), even(image_height));
    EpLaneGeometry g{};
    g.image_width = image_width;
    g.image_height = image_height;
    g.crop_x = even(std::lround(image_width * roi[0]));
    g.crop_y = even(std::lround(image_height * roi[1]));
    g.crop_width = std::max(0, right - g.crop_x);
    g.crop_height = std::max(0, bottom - g.crop_y);
    g.net_width = net_width;
    g.net_height = net_height;
    return g;
}

bool LanePlugin::decode(const EpTensor *outputs, uint32_t output_count, const EpLaneGeometry &geometry,
                        EpLaneResult *result, int32_t *code) const {
    std::memset(result, 0, sizeof(*result));
    int32_t rc = kThrew;
    try {
        rc = api_->decode(outputs, output_count, &geometry, result);
    } catch (...) {
        rc = kThrew;  // 해석 실패로 센다(잇달아 실패하면 단계가 꺼진다)
    }
    if (rc != 0 || !check_result(result)) {
        std::memset(result, 0, sizeof(*result));
        if (code) *code = rc != 0 ? rc : kInvalidResult;
        return false;
    }
    if (code) *code = 0;
    return true;
}

LanePluginGuard::LanePluginGuard(std::string path, int max_starts) : path_(std::move(path)), max_starts_(max_starts) {}

LanePluginGuard::~LanePluginGuard() {
    // 예외로 풀려 내려가는 중이면 정상 종료가 아니다: 기록을 남겨 다음 시작이 센다
    if (armed_ && std::uncaught_exceptions() == 0) healthy();
}

int LanePluginGuard::starts() const {
    std::ifstream in(path_);
    int n = 0;
    if (!(in >> n) || n < 0) return 0;
    return n;
}

bool LanePluginGuard::begin() {
    const int n = starts();
    if (n >= max_starts_) return false;
    std::ofstream out(path_, std::ios::trunc);
    out << n + 1 << "\n";
    out.flush();
    armed_ = static_cast<bool>(out);
    return true;
}

void LanePluginGuard::healthy() {
    std::remove(path_.c_str());
    armed_ = false;
}
