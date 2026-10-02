#pragma once

/* openpilot torqued(TorqueEstimator) 이식. 점의 요레이트·롤 출처(ESP12/locationd)를 캐시에 남기고
 * 출처가 바뀐 캐시는 배율·마찰만 이어 쓴다. */

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>


/* CP.lateralTuning.torque. 사전값이자 허용 폭(배율 ±30%, 마찰 ±50%)과 캐시 키의 기준이다. */
struct TorqueTuning {
  double lat_accel_factor = 0.0;  // 정규화 토크 1.0이 내는 횡가속도 [m/s²]
  double friction = 0.0;
};

/* 한 제어 틱의 입력. 토크·요레이트는 상류 torqued처럼 우측 양수다(컨트롤러 곡률과 같은
 * 관례라 학습 절편을 FF에 그대로 뺄 수 있다). 롤은 paramsd 관례. */
struct TorqueEstimatorInput {
  double t_s = 0.0;
  bool inputs_fresh = false;  // 상류 sm.all_checks()
  bool lat_active = false;
  double steer_torque = 0.0;  // 실제 보낸 토크 / steer_max, 우측 양수
  double speed_mps = 0.0;
  bool steer_override = false;
  bool pose_valid = false;      // 요레이트·롤 출처가 유효
  double yaw_rate_rad_s = 0.0;  // 바이어스 제거 후, 우측 양수
  double roll_rad = 0.0;        // 양수 = 오른쪽이 낮다(paramsd·상류 NED 관례)
};

/* lateralTorqueParameters. 제어는 필터값을 쓴다. */
struct TorqueParams {
  bool inputs_ok = false;  // 봉투 valid
  bool valid = false;      // 필드 valid
  bool use_params = true;
  double lat_accel_factor_raw = 0.0;
  double lat_accel_offset_raw = 0.0;
  double friction_raw = 0.0;
  double lat_accel_factor = 0.0;
  double lat_accel_offset = 0.0;
  double friction = 0.0;
  int total_bucket_points = 0;
  int cal_perc = 0;
  double decay = 0.0;
  double max_resets = 0.0;
};

/* SourceChanged: 다른 요레이트·롤 출처(ESP12 ↔ locationd)로 쌓은 캐시. 점의 횡가속도가 롤 출처
 * 차이(g·Δroll, 0.1~0.25 m/s²)만큼 어긋나 섞으면 절편이 몇 시간 끌려가므로 점과 절편은 버리고
 * 출처와 무관한 배율·마찰만 이어 쓴다. */
enum class TorqueRestore { None, Restored, KeyMismatch, Corrupt, SourceChanged };

class TorqueEstimator {
public:
  static constexpr int kBuckets = 8;
  static constexpr int kPointsPerBucket = 1500;
  static constexpr int kHistLen = 100;  // 5초 × 20 Hz

  /* lag_s는 상류 lateralDelay 자리. cache가 비어 있지 않으면 상류처럼 먼저 복원한다.
   * Corrupt면 호출자가 캐시를 지운다(상류 params.remove). 키 불일치는 두기만 한다.
   * localizer_source: 점의 요레이트·롤 출처(캐시에 남기고 복원 때 대조한다). */
  TorqueEstimator(const TorqueTuning &offline, double lag_s, uint64_t seed,
                  const std::string &cache = std::string(), bool localizer_source = false);

  /* 제어 틱마다 부른다. 4 Hz 출력 틱이면 true이고 params()가 갱신된다. */
  bool update(const TorqueEstimatorInput &in);
  const TorqueParams &params() const { return last_; }
  /* 20 Hz 240번(12초)마다, 첫 틱 포함. 상류처럼 get_msg를 한 번 더 돌린 결과라
   * 그 틱에는 필터가 두 번 갱신된다. */
  bool persist_due() const { return persist_due_; }
  const std::string &cache() const { return cache_; }
  TorqueRestore restore_status() const { return restore_; }
  const TorqueTuning &tuning() const { return offline_; }
  bool localizer_source() const { return localizer_source_; }
  /* 상류 torqued는 lateralDelay 메시지(lagd)로 lag를 바꾼다. 이후 들어오는 점부터 적용한다. */
  void set_lag(double lag_s) {
    if (std::isfinite(lag_s) && lag_s > 0.0) lag_s_ = lag_s;
  }
  double lag() const { return lag_s_; }

  /* 적합에 점을 전부 쓴다(상류는 2000점 무작위 추출). 참조 구현 대조용. */
  void set_fit_all_points(bool all) { fit_all_points_ = all; }
  int bucket_size(int i) const { return buckets_[i].count; }
  int total_points() const;
  /* 상류 get_points()[:, [0, 2]]: 버킷 순서, 버킷 안은 오래된 것부터. */
  std::vector<std::array<double, 2>> points() const;

private:
  struct RawSample {
    double t = 0.0;  // 지연을 더한 시각
    double lat_active = 0.0;
    double steer = 0.0;
    double vego = 0.0;
    double steer_override = 0.0;
  };
  struct Bucket {  // NPQueue: 가득 차면 가장 오래된 점을 민다
    std::vector<std::array<double, 2>> points =
        std::vector<std::array<double, 2>>(kPointsPerBucket);
    int head = 0;
    int count = 0;
    const std::array<double, 2> &at(int i) const {
      return points[(head + i) % kPointsPerBucket];
    }
  };
  struct Filter {  // FirstOrderFilter(dt = DT_MDL)
    double x = 0.0;
    double alpha = 0.0;
  };

  void reset();
  void add_point(double x, double y);
  bool is_calculable() const;
  bool is_valid() const;
  int valid_percent() const;
  void estimate_params(double *slope, double *offset, double *friction);
  void update_params(double factor, double offset, double friction);
  void handle_device_motion(const TorqueEstimatorInput &in);
  double interp_raw(double t, double RawSample::*field) const;
  TorqueParams get_msg(bool inputs_ok);
  std::string serialize(const TorqueParams &p) const;
  TorqueRestore restore(const std::string &cache, double *factor, double *offset,
                        double *friction);

  TorqueTuning offline_;
  bool localizer_source_ = false;
  double lag_s_ = 0.0;
  uint64_t rng_ = 0;
  bool fit_all_points_ = false;
  double min_factor_ = 0.0, max_factor_ = 0.0, min_friction_ = 0.0, max_friction_ = 0.0;
  double resets_ = 0.0;
  double decay_ = 0.0;
  std::array<RawSample, kHistLen> raw_{};
  int raw_head_ = 0;  // 가장 오래된 표본
  int raw_count_ = 0;
  std::array<Bucket, kBuckets> buckets_{};
  std::vector<std::array<double, 2>> fit_points_;
  Filter factor_f_, offset_f_, friction_f_;
  long frame_ = -1;
  bool has_motion_t_ = false;
  double last_motion_t_ = 0.0;
  bool persist_due_ = false;
  TorqueRestore restore_ = TorqueRestore::None;
  std::string cache_;
  TorqueParams last_{};
};
