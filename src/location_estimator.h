#ifndef LOCATION_ESTIMATOR_H
#define LOCATION_ESTIMATOR_H

/* openpilot locationd(selfdrive/locationd/locationd.py + models/pose_kf.py)의 C++ 이식.
 *
 * 보드 IMU(자이로·가속도)와 모델의 카메라 주행거리(pose: 평행이동·회전, 보정 좌표계)를
 * 18상태 EKF로 합쳐 기기 좌표계(x 앞, y 오른쪽, z 아래)의 자세·속도·각속도·가속도를 낸다.
 * paramsd·torqued·lagd가 쓰는 요레이트와 도로 롤을 CAN 대신 여기서 얻을 수 있다.
 *
 * rednose의 EKF_sym과 같이 예측은 x ← f(x, dt), P ← F P Fᵀ + dt·Q, 갱신은 Joseph 형태다.
 * 야코비안은 수치 미분으로 구한다(18상태라 비용이 작다). 카메라 주행거리는 모델 문맥 때문에
 * 0.1초 과거 시점의 관측이라, 필터를 그 시각으로 되감아 넣고 이후 관측을 다시 적용한다
 * (최대 0.8초). 하드웨어에 의존하지 않아 호스트에서 녹화를 재생해 검사한다. */

#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>

class PoseKalman {
public:
    static constexpr int kN = 18;
    using Vec = std::array<double, kN>;
    using Mat = std::array<std::array<double, kN>, kN>;
    using Vec3 = std::array<double, 3>;

    // 상태 배치(pose_kf.States)
    static constexpr int kOrientation = 0;      // NED 기준 roll, pitch, yaw (rad)
    static constexpr int kVelocity = 3;         // 기기 좌표계 속도 (m/s)
    static constexpr int kAngularVelocity = 6;  // 기기 좌표계 각속도 (rad/s)
    static constexpr int kGyroBias = 9;
    static constexpr int kAcceleration = 12;    // 기기 좌표계 가속도 (m/s²)
    static constexpr int kAccelBias = 15;

    enum class Kind { Gyro, Accel, CameraTranslation, CameraRotation };

    static Vec initial_x();
    static Mat initial_p();
    static Mat q();
    static Vec3 default_noise(Kind kind);  // 관측 분산(대각)

    explicit PoseKalman(double max_rewind_s = 0.8) : max_rewind_s_(max_rewind_s) { init(initial_x(), initial_p()); }
    // t가 NaN이면 첫 관측 시각에서 시작한다.
    void init(const Vec &x, const Mat &p, double t = std::numeric_limits<double>::quiet_NaN());
    /* t 시각의 관측을 넣는다. 필터 시각보다 max_rewind_s 넘게 과거면 false.
     * 과거 관측은 되감아 넣고 뒤의 관측을 다시 적용한다. */
    bool predict_and_observe(double t, Kind kind, const Vec3 &z, const Vec3 &r);

    const Vec &x() const { return x_; }
    const Mat &p() const { return p_; }
    double t() const { return t_; }

    // 모델 식(검사용으로 공개)
    static Vec f(const Vec &x, double dt);
    static Vec3 h(Kind kind, const Vec &x);

private:
    struct Entry {
        double t;
        Kind kind;
        Vec3 z;
        Vec3 r;
        // 이 관측을 적용하기 직전의 필터 상태
        double t_before;
        Vec x_before;
        Mat p_before;
    };

    void predict(double dt);
    void update(Kind kind, const Vec3 &z, const Vec3 &r);
    void apply(double t, Kind kind, const Vec3 &z, const Vec3 &r);

    double max_rewind_s_;
    Vec x_{};
    Mat p_{};
    double t_ = std::numeric_limits<double>::quiet_NaN();
    std::deque<Entry> history_;
};

// locationd의 deviceMotion(livePose)에 해당하는 출력. 모두 기기 좌표계, 표준편차 포함.
struct LivePoseEstimate {
    double t = 0.0;
    std::array<double, 3> orientation_ned{}, orientation_ned_std{};
    std::array<double, 3> velocity_device{}, velocity_device_std{};
    std::array<double, 3> angular_velocity_device{}, angular_velocity_device_std{};
    std::array<double, 3> acceleration_device{}, acceleration_device_std{};
    bool filter_valid = false;
    bool inputs_ok = false;
    bool posenet_ok = false;
    bool sensors_ok = false;
};

/* 입력별 받음·거부 누적 수(사유별). locationd 로그와 재생 도구가 어느 입력이 inputs_ok를
 * 떨어뜨리는지 보는 데 쓴다. */
struct LocationInputCounters {
    uint64_t accel_ok = 0, accel_timestamp = 0, accel_sanity = 0, accel_filter = 0;
    uint64_t gyro_ok = 0, gyro_timestamp = 0, gyro_sanity = 0, gyro_cross_check = 0, gyro_filter = 0;
    uint64_t camera_ok = 0, camera_timestamp = 0, camera_sanity = 0, camera_filter = 0, camera_speed_guard = 0;
};

/* locationd.LocationEstimator + main 루프의 입력 검사. 입력은 시각 순서로 넣어야 한다
 * (카메라 주행거리는 캡처 시각을 주면 내부에서 0.1초 당겨 되감아 넣는다). */
class LocationEstimator {
public:
    LocationEstimator();

    /* 보드 IMU 한 샘플(칩 좌표 그대로). 칩 축(y 위, z 뒤, x 오른쪽)을 기기 좌표계로 돌린다. */
    void handle_imu(double t, const float accel_chip[3], const float gyro_chip[3]);
    /* 차속(CAN). valid면 카메라 주행거리를 차속과 대조한다(상류에 없는 가드: 정차 중 모델이
     * 가짜 움직임을 내면 자이로 교차검사가 자이로를 버리고 자세·바이어스가 틀어진다). */
    void handle_car_speed(double t, double speed_mps, bool valid)
    {
        car_speed_ = std::fabs(speed_mps);
        car_speed_t_ = t;
        car_speed_valid_ = valid;
    }
    // 온라인 캘리브레이션 rpy(보정 → 기기 회전)
    void handle_calibration(const double rpy[3]);
    /* 모델 pose(보정 좌표계). t_capture는 근거 프레임의 센서 캡처 시각. */
    void handle_camera_odometry(double t_capture, const float trans[3], const float rot[3],
                                const float trans_std[3], const float rot_std[3]);

    LivePoseEstimate estimate(double now) const;
    const PoseKalman &filter() const { return kf_; }
    const LocationInputCounters &counters() const { return counters_; }
    // 0 가속도, 1 자이로, 2 카메라: 거부 누적이 한도를 넘었으면 해당 비트
    uint32_t invalid_service_mask() const;
    bool camera_guarded() const { return camera_guarded_; }

    static constexpr double kCamOdoPoseDelay = 0.1;

private:
    enum Service { kAccel, kGyro, kCamera, kServiceCount };
    void note_result(Service service, bool success);
    bool timestamp_ok(double t) const;
    void finite_check(double t);

    PoseKalman kf_;
    bool car_speed_ok(double t) const;

    double car_speed_ = 0.0;
    double car_speed_t_ = -1.0;
    bool car_speed_valid_ = false;
    bool camera_guarded_ = false;  // 마지막 카메라 관측이 차속 가드로 빠졌다
    LocationInputCounters counters_;
    std::array<double, 9> device_from_calib_{1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::array<double, 2> camodo_yawrate_{0.0, 10.0};  // 평균, 표준편차
    std::array<double, 40> posenet_stds_{};
    std::array<double, kServiceCount> invalid_{};
    std::array<double, kServiceCount> invalid_threshold_{};
    std::array<double, kServiceCount> invalid_decay_{};
    double last_imu_t_ = -1.0;
    bool seen_camera_ = false;
    bool seen_imu_ = false;
};

/* PoseCalibrator: 기기 좌표계 추정을 보정(차량) 좌표계로 옮긴다. 요레이트는 z(아래)축이라
 * 오른쪽 회전이 양수다. 롤은 오른쪽이 아래로 기울 때 양수. */
struct CalibratedPose {
    std::array<double, 3> orientation{};   // roll, pitch, yaw
    std::array<double, 3> angular_velocity{}, angular_velocity_std{};
    std::array<double, 3> acceleration{};
};
CalibratedPose calibrate_pose(const LivePoseEstimate &pose, const double calib_rpy[3]);

#endif
