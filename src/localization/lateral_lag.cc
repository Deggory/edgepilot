#include "localization/lateral_lag.h"

#include "common/utils_json.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace {

constexpr int kCorrBorderOffset = 5;
constexpr double kLagCandidateCorrThreshold = 0.9;
constexpr int kSmoothK = 5;
constexpr double kSmoothSigma = 1.0;
constexpr double kMaxYawRateSanityCheck = 1.0;
constexpr int kCacheVersion = 1;

double parabolic_peak_interp(const std::vector<double> &r, size_t i)
{
    if (i == 0 || i + 1 >= r.size()) return static_cast<double>(i);
    const double ym1 = r[i - 1], y0 = r[i], yp1 = r[i + 1];
    const double denom = 2 * y0 - yp1 - ym1;
    if (denom == 0.0) return static_cast<double>(i);
    return static_cast<double>(i) + 0.5 * (yp1 - ym1) / denom;
}

}  // namespace

LateralLagEstimator::LateralLagEstimator(const LateralLagConfig &config) : config_(config)
{
    reset(config_.initial_lag, 0);
}

void LateralLagEstimator::reset(double initial_lag, int valid_blocks)
{
    config_.initial_lag = initial_lag;
    // upstream Points는 창 길이만큼 0(무효)으로 채워 시작한다.
    points_.assign(static_cast<size_t>(std::lround(config_.window_s / config_.dt)), Point{0.0, 0.0, 0.0, false});
    okay_count_ = 0;
    blocks_.init(config_.block_count, config_.block_size, valid_blocks, initial_lag);
    last_estimate_t_ = 0.0;
}

void LateralLagEstimator::Blocks::init(int n, int size, int valid, double initial)
{
    num_blocks = n;
    block_size = size;
    valid_blocks = std::clamp(valid, 0, n);
    block_idx = valid_blocks % n;
    idx = 0;
    values.assign(static_cast<size_t>(n), initial);
}

void LateralLagEstimator::Blocks::update(double v)
{
    double &cur = values[static_cast<size_t>(block_idx)];
    cur = (idx * cur + v) / (idx + 1);
    idx = (idx + 1) % block_size;
    if (idx == 0) {
        block_idx = (block_idx + 1) % num_blocks;
        valid_blocks = std::min(valid_blocks + 1, num_blocks);
    }
}

void LateralLagEstimator::Blocks::get(double *valid_mean, double *valid_std, double *current_mean,
                                      double *current_std) const
{
    auto stats = [&](bool include_current, double *mean, double *std) {
        std::vector<double> v;
        for (int i = 0; i < valid_blocks; ++i)
            if (i != block_idx) v.push_back(values[static_cast<size_t>(i)]);
        if (include_current && idx > 0) v.push_back(values[static_cast<size_t>(block_idx)]);
        if (v.empty()) {
            *mean = *std = std::numeric_limits<double>::quiet_NaN();
            return;
        }
        double m = 0.0;
        for (double x : v) m += x;
        m /= static_cast<double>(v.size());
        double s = 0.0;
        for (double x : v) s += (x - m) * (x - m);
        *mean = m;
        *std = std::sqrt(s / static_cast<double>(v.size()));
    };
    stats(false, valid_mean, valid_std);
    stats(true, current_mean, current_std);
}

void LateralLagEstimator::update_points(const LateralLagInput &in)
{
    t_ = in.t;
    const double la_desired = in.desired_curvature * in.v_ego * in.v_ego;
    const double la_actual = in.yaw_rate * in.v_ego;
    const bool fast = in.v_ego > config_.min_vego;
    const bool sensors_valid = in.pose_valid && std::fabs(in.yaw_rate) < kMaxYawRateSanityCheck &&
                               in.yaw_rate_std < kMaxYawRateSanityCheck;
    const bool la_valid = std::fabs(la_actual) <= config_.max_lat_accel &&
                          std::fabs(la_desired - la_actual) <= config_.max_lat_accel_diff;
    if (!in.lat_active) last_lat_inactive_t_ = t_;
    if (in.steering_pressed) last_steering_pressed_t_ = t_;
    if (in.saturated) last_saturated_t_ = t_;
    if (!sensors_valid || !la_valid) last_pose_invalid_t_ = t_;
    const double r = config_.min_recovery_buffer_s;
    const bool recovered = t_ - last_lat_inactive_t_ >= r && t_ - last_steering_pressed_t_ >= r &&
                           t_ - last_saturated_t_ >= r && t_ - last_pose_invalid_t_ >= r;
    const bool okay = in.lat_active && !in.steering_pressed && !in.saturated && fast && recovered &&
                      in.calib_valid && sensors_valid && la_valid;
    if (points_.front().okay) --okay_count_;
    points_.pop_front();
    points_.push_back({t_, la_desired, la_actual, okay});
    if (okay) ++okay_count_;
}

std::vector<double> LateralLagEstimator::masked_smooth(const std::vector<double> &x, const std::vector<bool> &mask)
{
    // upstream masked_symmetric_moving_average: 가우시안 가중, 양끝은 가장자리 값으로 채움
    const int pad = kSmoothK / 2;
    double w[kSmoothK];
    double wsum = 0.0;
    for (int i = 0; i < kSmoothK; ++i) {
        const double d = (i - pad) / kSmoothSigma;
        w[i] = std::exp(-0.5 * d * d);
        wsum += w[i];
    }
    for (double &v : w) v /= wsum;
    const int n = static_cast<int>(x.size());
    std::vector<double> out(x.size(), std::numeric_limits<double>::quiet_NaN());
    for (int i = 0; i < n; ++i) {
        double num = 0.0, den = 0.0;
        for (int k = 0; k < kSmoothK; ++k) {
            const int j = std::clamp(i + k - pad, 0, n - 1);
            const double m = mask[static_cast<size_t>(j)] ? 1.0 : 0.0;
            num += w[k] * x[static_cast<size_t>(j)] * m;
            den += w[k] * m;
        }
        if (den != 0.0) out[static_cast<size_t>(i)] = num / den;
    }
    return out;
}

LateralLagEstimator::DelayResult LateralLagEstimator::actuator_delay(const std::vector<double> &expected,
                                                                     const std::vector<double> &actual,
                                                                     const std::vector<bool> &mask, double dt,
                                                                     double min_lag, double max_lag)
{
    const int n = static_cast<int>(expected.size());
    const int min_s = static_cast<int>(std::lround(min_lag / dt));
    const int max_s = static_cast<int>(std::lround(max_lag / dt));
    const int one_s = static_cast<int>(std::lround(1.0 / dt));
    // Padfield 마스크 정규화 상호상관. lag k: actual[i]와 expected[i − k]
    auto ncc_at = [&](int k) {
        double nn = 0, sa = 0, se = 0, sea = 0, saa = 0, see = 0;
        for (int i = std::max(0, k); i < std::min(n, n + k); ++i) {
            const int j = i - k;
            if (!mask[static_cast<size_t>(i)] || !mask[static_cast<size_t>(j)]) continue;
            const double a = actual[static_cast<size_t>(i)], e = expected[static_cast<size_t>(j)];
            nn += 1;
            sa += a;
            se += e;
            sea += e * a;
            saa += a * a;
            see += e * e;
        }
        nn = std::max(nn, std::numeric_limits<double>::epsilon());
        const double num = sea - sa * se / nn;
        const double da = std::max(0.0, saa - sa * sa / nn);
        const double de = std::max(0.0, see - se * se / nn);
        const double den = std::sqrt(da * de);
        return std::make_pair(num, den);
    };
    const int lo = -kCorrBorderOffset, hi = one_s + kCorrBorderOffset;  // [lo, hi)
    std::vector<double> num(static_cast<size_t>(hi - lo)), den(num.size());
    double max_den = 0.0;
    for (int k = lo; k < hi; ++k) {
        const auto nd = ncc_at(k);
        num[static_cast<size_t>(k - lo)] = nd.first;
        den[static_cast<size_t>(k - lo)] = nd.second;
        max_den = std::max(max_den, std::fabs(nd.second));
    }
    // upstream은 전체 상관 배열의 최대 분모 기준으로 아주 작은 분모를 0으로 둔다(여기선 계산한 범위 기준).
    const double tol = 1e3 * std::numeric_limits<double>::epsilon() * max_den;
    auto ncc = [&](int k) {
        const size_t i = static_cast<size_t>(k - lo);
        return den[i] > tol ? std::clamp(num[i] / den[i], -1.0, 1.0) : 0.0;
    };
    std::vector<double> roi;
    for (int k = min_s; k < max_s; ++k) roi.push_back(ncc(k));
    const size_t best = static_cast<size_t>(std::max_element(roi.begin(), roi.end()) - roi.begin());
    DelayResult out;
    out.corr = roi[best];
    out.lag = parabolic_peak_interp(roi, best) * dt + min_lag;
    double tmin = 1e9, tmax = -1e9;
    for (int k = 0; k < one_s; ++k) {
        tmin = std::min(tmin, ncc(k));
        tmax = std::max(tmax, ncc(k));
    }
    const double thresh = (tmax - tmin) * kLagCandidateCorrThreshold + tmin;
    std::vector<int> good;
    for (int k = lo; k < hi; ++k) good.push_back(ncc(k) >= thresh ? 1 : 0);
    // 연속 구간 시작·끝, upstream과 같이 roi 인덱스 + 테두리로 구간을 고른다
    std::vector<int> starts, ends;
    for (size_t i = 0; i < good.size(); ++i) {
        const int prev = i == 0 ? 0 : good[i - 1];
        if (good[i] && !prev) starts.push_back(static_cast<int>(i));
        const int next = i + 1 < good.size() ? good[i + 1] : 0;
        if (good[i] && !next) ends.push_back(static_cast<int>(i));
    }
    const int key = static_cast<int>(best) + kCorrBorderOffset;
    const int run = static_cast<int>(std::upper_bound(starts.begin(), starts.end(), key) - starts.begin()) - 1;
    const double width = run >= 0 ? ends[static_cast<size_t>(run)] - starts[static_cast<size_t>(run)] + 1
                                  : static_cast<double>(good.size());
    out.confidence = std::clamp(1.0 - width * dt, 0.0, 1.0);
    return out;
}

bool LateralLagEstimator::update_estimate()
{
    const size_t okay_needed = static_cast<size_t>(std::lround(config_.okay_window_s / config_.dt));
    if (points_.size() < okay_needed) return false;
    double amin = 1e18, amax = -1e18;
    for (const Point &p : points_) {
        amin = std::min(amin, p.actual);
        amax = std::max(amax, p.actual);
    }
    bool is_valid = okay_count_ >= okay_needed && amax - amin >= config_.min_lat_accel_range;
    if (last_estimate_t_ != 0.0 && points_.front().t <= last_estimate_t_) {
        // 마지막 추정 뒤로 새 점이 있고, 그중 하나는 쓸 수 있어야 한다
        size_t newer = 0;
        for (auto it = points_.rbegin(); it != points_.rend() && it->t > last_estimate_t_; ++it) ++newer;
        bool any_okay = false;
        for (size_t i = points_.size() - newer; i < points_.size(); ++i) any_okay = any_okay || points_[i].okay;
        is_valid = is_valid && newer > 0 && any_okay;
    }
    if (!is_valid) return false;
    std::vector<double> desired, actual;
    std::vector<bool> mask;
    desired.reserve(points_.size());
    for (const Point &p : points_) {
        desired.push_back(p.desired);
        actual.push_back(p.actual);
        mask.push_back(p.okay);
    }
    std::vector<double> ds = masked_smooth(desired, mask), as = masked_smooth(actual, mask);
    for (size_t i = 0; i < ds.size(); ++i) {
        if (!std::isfinite(ds[i])) ds[i] = 0.0;
        if (!std::isfinite(as[i])) as[i] = 0.0;
    }
    const DelayResult r = actuator_delay(ds, as, mask, config_.dt, config_.min_lag, config_.max_lag);
    if (r.corr < config_.min_ncc || r.confidence < config_.min_confidence) return false;
    blocks_.update(r.lag);
    last_estimate_t_ = t_;
    return true;
}

LateralLagOutput LateralLagEstimator::output() const
{
    LateralLagOutput out;
    double vm, vs, cm, cs;
    blocks_.get(&vm, &vs, &cm, &cs);
    if (blocks_.valid_blocks >= config_.min_valid_block_count && std::isfinite(vm) && std::isfinite(vs))
        out.status = vs > config_.max_lag_std ? LateralLagStatus::Invalid : LateralLagStatus::Estimated;
    out.lateral_delay = out.status == LateralLagStatus::Estimated
        ? std::clamp(vm, config_.min_lag, config_.max_lag) : config_.initial_lag;
    out.estimate = std::isfinite(cm) ? cm : config_.initial_lag;
    out.estimate_std = std::isfinite(cs) ? cs : 0.0;
    out.valid_blocks = blocks_.valid_blocks;
    out.cal_perc = std::min(100, 100 * (blocks_.valid_blocks * config_.block_size + blocks_.idx) /
                                     (config_.min_valid_block_count * config_.block_size));
    return out;
}

std::string LateralLagEstimator::cache_json() const
{
    const LateralLagOutput o = output();
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "{\n  \"version\": %d,\n  \"initial_lag\": %.6f,\n  \"estimate\": %.6f,\n"
                  "  \"valid_blocks\": %d,\n  \"status\": %u\n}\n",
                  kCacheVersion, config_.initial_lag, o.estimate, o.valid_blocks, static_cast<unsigned>(o.status));
    return buf;
}

bool LateralLagEstimator::restore(const std::string &json)
{
    // upstream retrieve_initial_lag: 같은 버전·차량(단일 차종이라 버전만)이고 무효가 아니면
    // 추정값과 유효 블록 수로 다시 시작한다. 초기값(steer_actuator_delay)이 바뀌어도 측정한 블록은
    // 차의 지연이라 그대로 쓴다(상류도 steerActuatorDelay로 거르지 않는다).
    float version = 0, initial = 0, estimate = 0, blocks = 0, status = 0;
    if (!parse_json_float_value(json, "version", &version) || static_cast<int>(version) != kCacheVersion ||
        !parse_json_float_value(json, "initial_lag", &initial) ||
        !parse_json_float_value(json, "estimate", &estimate) ||
        !parse_json_float_value(json, "valid_blocks", &blocks) || !parse_json_float_value(json, "status", &status))
        return false;
    if (static_cast<int>(status) == static_cast<int>(LateralLagStatus::Invalid) || blocks < 0 ||
        blocks > config_.block_count || !std::isfinite(estimate))
        return false;
    const double keep_initial = config_.initial_lag;
    reset(estimate, static_cast<int>(blocks));
    config_.initial_lag = keep_initial;
    return true;
}
