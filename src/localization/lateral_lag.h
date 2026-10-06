#ifndef LATERAL_LAG_H
#define LATERAL_LAG_H

/* openpilot lagd(selfdrive/locationd/lagd.py)의 C++ 이식: 조향 지연을 주행 중에 추정한다.
 *
 * 20 Hz로 목표 횡가속(desired curvature · v²)과 실제 횡가속(요레이트 · v)을 60초 창에 모으고,
 * 조건(조향 활성, 핸들 비조작, 비포화, 고속, 센서·횡가속 정상, 2초 회복)을 만족하는 점만
 * 마스크해 정규화 상호상관의 봉우리를 찾는다. 추정값은 100개씩 블록 평균을 내고, 5블록이
 * 모이면 유효하다(블록 사이 표준편차 0.1 s를 넘으면 무효). upstream은 전체 상관을 FFT로 구하지만
 * 쓰는 지연 범위(−0.25~1.25 s)만 직접 계산해도 값은 같다. 하드웨어에 의존하지 않는다. */

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

struct LateralLagConfig {
    double dt = 0.05;               // 점 주기(20 Hz)
    int block_count = 50;
    int min_valid_block_count = 5;
    int block_size = 100;
    double window_s = 60.0;
    double okay_window_s = 25.0;
    double min_recovery_buffer_s = 2.0;
    // upstream은 50 mph. 우리 주행은 80 km/h 이상이 드물어(2026-09-27 39분 중 7분) 40 km/h로 둔다.
    double min_vego = 40.0 / 3.6;
    double min_ncc = 0.95;
    double max_lag = 0.65;
    double min_lag = 0.15;
    double max_lag_std = 0.1;
    double max_lat_accel = 2.0;
    double max_lat_accel_diff = 0.6;
    double min_lat_accel_range = 0.5;
    double min_confidence = 0.7;
    double initial_lag = 0.42;      // steering.json steer_actuator_delay
};

struct LateralLagInput {
    double t = 0.0;
    bool lat_active = false;
    bool steering_pressed = false;
    bool saturated = false;
    double desired_curvature = 0.0;  // 오른쪽 양수(우리 제어 관례)
    double v_ego = 0.0;
    double yaw_rate = 0.0;           // 같은 관례(오른쪽 회전 양수), rad/s
    double yaw_rate_std = 0.0;
    bool pose_valid = false;
    bool calib_valid = false;
};

enum class LateralLagStatus : uint32_t { Unestimated = 0, Estimated = 1, Invalid = 2 };

struct LateralLagOutput {
    double lateral_delay = 0.0;          // 쓸 값: 추정되면 추정, 아니면 초기값
    double estimate = 0.0;               // 진행 중 블록 포함 평균
    double estimate_std = 0.0;
    int valid_blocks = 0;
    int cal_perc = 0;
    LateralLagStatus status = LateralLagStatus::Unestimated;
};

class LateralLagEstimator {
public:
    explicit LateralLagEstimator(const LateralLagConfig &config = LateralLagConfig());

    // 20 Hz로 부른다.
    void update_points(const LateralLagInput &in);
    // 4 Hz로 부른다. 새 추정을 블록에 넣었으면 true.
    bool update_estimate();
    LateralLagOutput output() const;

    // 저장·복원(재시작 사이에 추정을 이어 간다). 초기값이 다르면 복원하지 않는다.
    std::string cache_json() const;
    bool restore(const std::string &json);
    void reset(double initial_lag, int valid_blocks);

    const LateralLagConfig &config() const { return config_; }
    size_t okay_points() const { return okay_count_; }

    // 검사용: 마스크 정규화 상호상관으로 지연을 구한다(upstream actuator_delay).
    struct DelayResult {
        double lag;
        double corr;
        double confidence;
    };
    static DelayResult actuator_delay(const std::vector<double> &expected, const std::vector<double> &actual,
                                      const std::vector<bool> &mask, double dt, double min_lag, double max_lag);
    static std::vector<double> masked_smooth(const std::vector<double> &x, const std::vector<bool> &mask);

private:
    struct Point {
        double t;
        double desired;
        double actual;
        bool okay;
    };
    struct Blocks {
        int num_blocks = 0, block_size = 0, block_idx = 0, idx = 0, valid_blocks = 0;
        std::vector<double> values;
        void init(int n, int size, int valid, double initial);
        void update(double v);
        void get(double *valid_mean, double *valid_std, double *current_mean, double *current_std) const;
    };

    LateralLagConfig config_;
    std::deque<Point> points_;
    size_t okay_count_ = 0;
    Blocks blocks_;
    double t_ = 0.0;
    double last_lat_inactive_t_ = 0.0, last_steering_pressed_t_ = 0.0, last_saturated_t_ = 0.0,
           last_pose_invalid_t_ = 0.0, last_estimate_t_ = 0.0;
};

#endif
