#include "torque_estimator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

constexpr double kGravity = 9.81;
constexpr double kDtMdl = 0.05;

double clip(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

}  // namespace

// ---------------------------------------------------------------- TorqueEstimator

namespace {

// torqued.py
constexpr int kMinPointsTotal = 4000;
constexpr int kFitPointsTotal = 2000;
constexpr double kMinVel = 15.0;
constexpr double kFrictionFactor = 1.5;
constexpr double kFactorSanity = 0.3;
constexpr double kFrictionSanity = 0.5;
constexpr double kSteerMinThreshold = 0.02;
constexpr double kMinFilterDecay = 50.0;
constexpr double kMaxFilterDecay = 250.0;
constexpr double kLatAccThreshold = 1.0;
constexpr double kMinEngageBuffer = 2.0;
constexpr int kTorqueVersion = 2;  // 2: 요레이트·롤 출처 바이트(1은 출처 없이 ESP12)
constexpr double kSteerBucketBounds[TorqueEstimator::kBuckets][2] = {
    {-0.5, -0.3}, {-0.3, -0.2}, {-0.2, -0.1}, {-0.1, 0.0},
    {0.0, 0.1},   {0.1, 0.2},   {0.2, 0.3},   {0.3, 0.5}};
constexpr int kMinBucketPoints[TorqueEstimator::kBuckets] = {100, 300, 500, 500,
                                                             500, 500, 300, 100};
constexpr char kTorqueCacheMagic[8] = {'K', '2', '3', '0', 'T', 'Q', 'C', '1'};

uint64_t splitmix64(uint64_t *state) {
  uint64_t z = (*state += 0x9e3779b97f4a7c15ULL);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

/* 대칭 3×3의 최소 고유값 고유벡터(순환 야코비). [x,1,y]의 최소 특이벡터와 같다. */
std::array<double, 3> smallest_eigenvector(std::array<std::array<double, 3>, 3> a) {
  std::array<std::array<double, 3>, 3> v{};
  for (int i = 0; i < 3; ++i) v[i][i] = 1.0;
  for (int sweep = 0; sweep < 50; ++sweep) {
    if (a[0][1] == 0.0 && a[0][2] == 0.0 && a[1][2] == 0.0) break;
    for (int p = 0; p < 2; ++p) {
      for (int q = p + 1; q < 3; ++q) {
        if (a[p][q] == 0.0) continue;
        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        const double t = (theta >= 0.0 ? 1.0 : -1.0) /
                         (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
        for (int k = 0; k < 3; ++k) {
          const double akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {
          const double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        a[p][q] = a[q][p] = 0.0;
        for (int k = 0; k < 3; ++k) {
          const double vkp = v[k][p], vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  int m = 0;
  for (int i = 1; i < 3; ++i)
    if (a[i][i] < a[m][m]) m = i;
  return {v[0][m], v[1][m], v[2][m]};
}

template <class T> void put(std::string *out, const T &v) {
  out->append(reinterpret_cast<const char *>(&v), sizeof(v));
}
template <class T> bool take(const std::string &in, size_t *pos, T *v) {
  if (*pos + sizeof(T) > in.size()) return false;
  std::memcpy(v, in.data() + *pos, sizeof(T));
  *pos += sizeof(T);
  return true;
}

}  // namespace

TorqueEstimator::TorqueEstimator(const TorqueTuning &offline, double lag_s, uint64_t seed,
                                 const std::string &cache, bool localizer_source)
    : offline_(offline), localizer_source_(localizer_source), lag_s_(lag_s), rng_(seed) {
  fit_points_.reserve(kBuckets * kPointsPerBucket);
  reset();
  decay_ = kMinFilterDecay;
  min_factor_ = (1.0 - kFactorSanity) * offline_.lat_accel_factor;
  max_factor_ = (1.0 + kFactorSanity) * offline_.lat_accel_factor;
  min_friction_ = (1.0 - kFrictionSanity) * offline_.friction;
  max_friction_ = (1.0 + kFrictionSanity) * offline_.friction;
  double factor = offline_.lat_accel_factor, offset = 0.0, friction = offline_.friction;
  if (!cache.empty()) restore_ = restore(cache, &factor, &offset, &friction);
  const double alpha = kDtMdl / (decay_ + kDtMdl);
  factor_f_ = {factor, alpha};
  offset_f_ = {offset, alpha};
  friction_f_ = {friction, alpha};
}

void TorqueEstimator::reset() {
  resets_ += 1.0;
  decay_ = kMinFilterDecay;
  raw_head_ = raw_count_ = 0;
  for (Bucket &b : buckets_) b.head = b.count = 0;
}

void TorqueEstimator::add_point(double x, double y) {
  for (int i = 0; i < kBuckets; ++i) {
    if (x >= kSteerBucketBounds[i][0] && x < kSteerBucketBounds[i][1]) {
      Bucket &b = buckets_[i];
      if (b.count < kPointsPerBucket) {
        b.points[(b.head + b.count) % kPointsPerBucket] = {x, y};
        ++b.count;
      } else {
        b.points[b.head] = {x, y};
        b.head = (b.head + 1) % kPointsPerBucket;
      }
      break;
    }
  }
}

int TorqueEstimator::total_points() const {
  int n = 0;
  for (const Bucket &b : buckets_) n += b.count;
  return n;
}

std::vector<std::array<double, 2>> TorqueEstimator::points() const {
  std::vector<std::array<double, 2>> out;
  out.reserve(total_points());
  for (const Bucket &b : buckets_)
    for (int i = 0; i < b.count; ++i) out.push_back(b.at(i));
  return out;
}

bool TorqueEstimator::is_calculable() const {
  for (const Bucket &b : buckets_)
    if (b.count == 0) return false;
  return true;
}

bool TorqueEstimator::is_valid() const {
  for (int i = 0; i < kBuckets; ++i)
    if (buckets_[i].count < kMinBucketPoints[i]) return false;
  return total_points() >= kMinPointsTotal;
}

int TorqueEstimator::valid_percent() const {
  const double total = std::min(static_cast<double>(total_points()) / kMinPointsTotal * 100.0, 100.0);
  double individual = 1e300;
  for (int i = 0; i < kBuckets; ++i)
    individual = std::min(individual,
                          static_cast<double>(buckets_[i].count) / kMinBucketPoints[i] * 100.0);
  individual = std::min(individual, 100.0);
  return static_cast<int>((total + individual) / 2.0);
}

void TorqueEstimator::estimate_params(double *slope, double *offset, double *friction) {
  fit_points_.clear();  // 예약해 둔 버퍼를 재사용한다
  for (const Bucket &b : buckets_)
    for (int i = 0; i < b.count; ++i) fit_points_.push_back(b.at(i));
  const int total = static_cast<int>(fit_points_.size());
  int n = total;
  if (!fit_all_points_ && total > kFitPointsTotal) {  // rng.choice(replace=False): 부분 셔플
    n = kFitPointsTotal;
    for (int i = 0; i < n; ++i) {
      const int j = i + static_cast<int>(splitmix64(&rng_) % static_cast<uint64_t>(total - i));
      std::swap(fit_points_[i], fit_points_[j]);
    }
  }
  // 총최소제곱: [x, 1, y]의 최소 특이벡터 n에서 y = −(n0·x + n1)/n2
  std::array<std::array<double, 3>, 3> ata{};
  for (int i = 0; i < n; ++i) {
    const double r[3] = {fit_points_[i][0], 1.0, fit_points_[i][1]};
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b) ata[a][b] += r[a] * r[b];
  }
  const std::array<double, 3> v = smallest_eigenvector(ata);
  *slope = -v[0] / v[2];
  *offset = -v[1] / v[2];
  // slope2rot은 |기울기|로 돌린다(상류 그대로). 흩어짐 = 직선에 수직인 성분
  const double sin = std::sqrt(*slope * *slope / (*slope * *slope + 1.0));
  const double cos = std::sqrt(1.0 / (*slope * *slope + 1.0));
  double mean = 0.0;
  for (int i = 0; i < n; ++i) mean += -fit_points_[i][0] * sin + fit_points_[i][1] * cos;
  mean /= n;
  double var = 0.0;
  for (int i = 0; i < n; ++i) {
    const double d = -fit_points_[i][0] * sin + fit_points_[i][1] * cos - mean;
    var += d * d;
  }
  *friction = std::sqrt(var / n) * kFrictionFactor;
}

void TorqueEstimator::update_params(double factor, double offset, double friction) {
  decay_ = std::min(decay_ + kDtMdl, kMaxFilterDecay);
  const double alpha = kDtMdl / (decay_ + kDtMdl);
  Filter *filters[3] = {&factor_f_, &offset_f_, &friction_f_};
  const double values[3] = {factor, offset, friction};
  for (int i = 0; i < 3; ++i) {
    filters[i]->x = (1.0 - filters[i]->alpha) * filters[i]->x + filters[i]->alpha * values[i];
    filters[i]->alpha = alpha;
  }
}

double TorqueEstimator::interp_raw(double x, double RawSample::*field) const {  // np.interp
  auto at = [&](int i) -> const RawSample & { return raw_[(raw_head_ + i) % kHistLen]; };
  const int n = raw_count_;
  if (x < at(0).t) return at(0).*field;
  if (x >= at(n - 1).t) return at(n - 1).*field;
  int lo = 0, hi = n - 1;  // at(lo).t <= x < at(hi).t
  while (hi - lo > 1) {
    const int mid = (lo + hi) / 2;
    if (at(mid).t <= x) lo = mid;
    else hi = mid;
  }
  const RawSample &a = at(lo), &b = at(hi);
  if (a.t == x) return a.*field;
  const double slope = (b.*field - a.*field) / (b.t - a.t);
  return slope * (x - a.t) + a.*field;
}

void TorqueEstimator::handle_device_motion(const TorqueEstimatorInput &in) {
  if (raw_count_ < kHistLen || !in.pose_valid) return;
  const double t = in.t_s;
  // np.arange(t − MIN_ENGAGE_BUFFER, t + lag, DT_MDL): 이력은 지연만큼 밀려 있으니 지금까지의 활성
  const double start = t - kMinEngageBuffer;
  const long n = static_cast<long>(std::ceil((t + lag_s_ - start) / kDtMdl));
  const double delta = (start + kDtMdl) - start;
  bool all_active = true, any_override = false;
  for (long i = 0; i < n; ++i) {
    const double ti = i == 0 ? start : i == 1 ? start + kDtMdl : start + i * delta;
    all_active = all_active && interp_raw(ti, &RawSample::lat_active) != 0.0;
    any_override = any_override || interp_raw(ti, &RawSample::steer_override) != 0.0;
  }
  const double vego = interp_raw(t, &RawSample::vego);
  const double steer = interp_raw(t, &RawSample::steer);
  const double lateral_acc = vego * in.yaw_rate_rad_s - std::sin(in.roll_rad) * kGravity;
  if (all_active && !any_override && vego > kMinVel && std::fabs(steer) > kSteerMinThreshold &&
      std::fabs(lateral_acc) <= kLatAccThreshold)
    add_point(steer, lateral_acc);
}

bool TorqueEstimator::update(const TorqueEstimatorInput &in) {
  persist_due_ = false;
  const bool motion_due = !has_motion_t_ || in.t_s - last_motion_t_ >= kDtMdl - 1e-3;
  if (!motion_due) return false;
  has_motion_t_ = true;
  last_motion_t_ = in.t_s;
  ++frame_;
  if (in.inputs_fresh) {
    // 상류는 conflate된 carControl·carOutput·carState 최신값을 deviceMotion보다 먼저 넣는다
    // lag가 줄면 시각이 뒤로 갈 수 있다. 보간(np.interp)은 증가하는 시각을 가정한다
    double sample_t = in.t_s + lag_s_;
    if (raw_count_ > 0) {
      const double last_t = raw_[(raw_head_ + raw_count_ - 1) % kHistLen].t;
      sample_t = std::max(sample_t, last_t + 1e-3);
    }
    RawSample &s = raw_[(raw_head_ + raw_count_) % kHistLen];
    s = {sample_t, in.lat_active ? 1.0 : 0.0, in.steer_torque, in.speed_mps,
         in.steer_override ? 1.0 : 0.0};
    if (raw_count_ < kHistLen) ++raw_count_;
    else raw_head_ = (raw_head_ + 1) % kHistLen;
    handle_device_motion(in);
  }
  const bool publish = frame_ % 5 == 0;
  if (publish) last_ = get_msg(in.inputs_fresh);
  if (frame_ % 240 == 0) {
    cache_ = serialize(get_msg(in.inputs_fresh));
    persist_due_ = true;
  }
  return publish;
}

TorqueParams TorqueEstimator::get_msg(bool inputs_ok) {
  TorqueParams m;
  m.inputs_ok = inputs_ok;
  m.use_params = true;  // 현대 + 토크 제어
  if (is_calculable()) {
    double factor = 0.0, offset = 0.0, friction = 0.0;
    estimate_params(&factor, &offset, &friction);
    m.lat_accel_factor_raw = factor;
    m.lat_accel_offset_raw = offset;
    m.friction_raw = friction;
    if (is_valid()) {
      if (std::isnan(factor) || std::isnan(offset) || std::isnan(friction)) {
        std::fprintf(stderr, "torque params: NaN in estimate, resetting\n");
        m.valid = false;
        reset();
      } else {
        m.valid = true;
        update_params(clip(factor, min_factor_, max_factor_), offset,
                      clip(friction, min_friction_, max_friction_));
      }
    }
  }
  m.lat_accel_factor = factor_f_.x;
  m.lat_accel_offset = offset_f_.x;
  m.friction = friction_f_.x;
  m.total_bucket_points = total_points();
  m.cal_perc = valid_percent();
  m.decay = decay_;
  m.max_resets = resets_;
  return m;
}

/* 상류 LiveTorqueParameters(capnp Float32)와 같은 정밀도로 남긴다. 키는 상류 get_restore_key의
 * 튜닝값·버전(지문과 제어 종류는 단일 차종·토크 제어라 고정). */
std::string TorqueEstimator::serialize(const TorqueParams &p) const {
  std::string out(kTorqueCacheMagic, sizeof(kTorqueCacheMagic));
  put(&out, static_cast<int32_t>(kTorqueVersion));
  put(&out, static_cast<float>(offline_.friction));
  put(&out, static_cast<float>(offline_.lat_accel_factor));
  put(&out, static_cast<uint8_t>(localizer_source_ ? 1 : 0));
  put(&out, static_cast<uint8_t>(p.valid ? 1 : 0));
  put(&out, static_cast<float>(p.lat_accel_factor));
  put(&out, static_cast<float>(p.lat_accel_offset));
  put(&out, static_cast<float>(p.friction));
  put(&out, static_cast<float>(p.decay));
  put(&out, static_cast<uint32_t>(total_points()));
  for (const Bucket &b : buckets_) {
    for (int i = 0; i < b.count; ++i) {
      put(&out, static_cast<float>(b.at(i)[0]));
      put(&out, static_cast<float>(b.at(i)[1]));
    }
  }
  return out;
}

TorqueRestore TorqueEstimator::restore(const std::string &cache, double *factor, double *offset,
                                       double *friction) {
  size_t pos = sizeof(kTorqueCacheMagic);
  int32_t version = 0;
  float key_friction = 0, key_factor = 0, f_factor = 0, f_offset = 0, f_friction = 0, decay = 0;
  uint8_t source = 0, valid = 0;
  uint32_t n = 0;
  if (cache.size() < pos || std::memcmp(cache.data(), kTorqueCacheMagic, pos) != 0 ||
      !take(cache, &pos, &version) || !take(cache, &pos, &key_friction) ||
      !take(cache, &pos, &key_factor) || (version >= 2 && !take(cache, &pos, &source)) ||
      !take(cache, &pos, &valid) || !take(cache, &pos, &f_factor) || !take(cache, &pos, &f_offset) ||
      !take(cache, &pos, &f_friction) || !take(cache, &pos, &decay) || !take(cache, &pos, &n) ||
      cache.size() != pos + static_cast<size_t>(n) * 2 * sizeof(float))
    return TorqueRestore::Corrupt;
  if ((version != kTorqueVersion && version != 1) || key_friction != static_cast<float>(offline_.friction) ||
      key_factor != static_cast<float>(offline_.lat_accel_factor))
    return TorqueRestore::KeyMismatch;
  if ((source != 0) != localizer_source_) {
    if (valid) {
      *factor = f_factor;
      *friction = f_friction;
    }
    return TorqueRestore::SourceChanged;
  }
  if (valid) {
    *factor = f_factor;
    *offset = f_offset;
    *friction = f_friction;
  }
  for (uint32_t i = 0; i < n; ++i) {
    float x = 0, y = 0;
    take(cache, &pos, &x);
    take(cache, &pos, &y);
    add_point(x, y);
  }
  decay_ = decay;
  return TorqueRestore::Restored;
}
