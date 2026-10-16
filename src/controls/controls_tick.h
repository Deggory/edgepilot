#pragma once

// controlsd 한 틱(100 Hz)의 로직. 공유 메모리·파일을 다루지 않고, controlsd main이 읽어 온 입력을
// 받아 보낼 CAN과 상태를 만든다. 호스트 검사와 재생 도구가 같은 코드를 돌린다.
//
// 한 틱의 순서(main이 지킨다):
//   apply_params(바뀌었으면) → on_can_batch(...) → on_model(새 모델이면) → on_panda → on_localization →
//   step → [main: ControlState 발행, CAN 송신] → update_learners → [main: 학습 저장, LearnerState 발행]
// - 플래너는 on_model에서 이번 틱 CAN이 반영된 차량 상태와 직전 틱의 실제 곡률·active를 받아 그 자리에서
//   계산하고, 같은 틱의 step이 그 결과를 쓴다(평균 약 0.1 ms, MPC는 반복 없는 고정 연산; 2026-10-13 보드).
//   예전에는 작업 스레드가 계산해 새 플랜이 한 틱(10 ms) 늦게 쓰였고, 재생 도구·검사와도 그만큼 달랐다.
// - step의 now_ns는 공유 상태를 다 읽은 직후의 시각이다(그 사이 발행된 모델도 신선하게 본다).
// - 학습기 갱신은 송신 뒤라 적합·직렬화 틱이 CAN 송신을 늦추지 않고, 결과는 다음 틱의 컨트롤러가
//   쓴다.

#include "car/vehicle_can.h"
#include "common/ipc_messages.h"
#include "common/utils_file.h"
#include "controls/adaptive_cruise.h"
#include "controls/control_holds.h"
#include "controls/control_params.h"
#include "controls/departure_alert.h"
#include "controls/lateral_controller.h"
#include "learners/lateral_learners.h"
#include "planning/lane_state_selector.h"
#include "planning/lateral_planner.h"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

// controlsd의 런타임 파라미터 두 파일(steering/adaptive_cruise.json)과 코드 고정값(timing). 파일은 한꺼번에
// 읽고 한꺼번에 바꾼다.
struct ControlParams {
    SteeringParams steering;
    ControlTiming timing;
    AdaptiveCruiseConfig cruise;
};

struct ControlParamPaths {
    std::string steering;
    std::string cruise;
};

// 둘 다 읽어야 true. 하나라도 거부되면 params는 그대로이고 error에 사유를 쓴다.
bool load_control_params(const ControlParamPaths &paths, ControlParams *params, std::string *error);
// 시작·재적용 로그의 꼬리: MDPS 속도 바꿔치기와 비전 크루즈 설정.
std::string control_params_summary(const ControlParams &params);

// 두 파일을 stat으로 감시하고, 바뀌었거나 SIGHUP이 오면 둘을 다시 읽는다. 하나라도 거부되면 둘 다
// 이전 값을 유지한다.
class ControlParamsWatcher {
public:
    ControlParamsWatcher(ControlParamPaths paths, std::chrono::steady_clock::time_point now);

    // 새로 적용할 값. 바뀐 게 없거나 거부됐으면(로그를 남긴다) 비어 있다.
    std::optional<ControlParams> poll(std::chrono::steady_clock::time_point now, bool reload_requested,
                                      const ControlParams &current);
    unsigned generation() const { return generation_; }

private:
    void stamp();

    ControlParamPaths paths_;
    FileStamp steering_stamp_;
    FileStamp cruise_stamp_;
    std::chrono::steady_clock::time_point next_check_;
    unsigned generation_ = 1;
};

// engage/disengage/거부 이벤트 id와 전이 로그. id는 0을 건너뛰어 HUD가 새 이벤트를 구분한다.
struct EngageEvents {
    uint32_t engage_id = 0;
    uint32_t disengage_id = 0;
    uint32_t reject_id = 0;
    char reject_block[32] = {};
    bool have_previous = false;
    bool previous_engaged = false;
    bool previous_active = false;

    void update(const LateralControlResult &result, const VehicleCanState &vehicle, const PandaGateOutput &panda,
                const PandaState &panda_state, const PathHoldOutput &held, const ModelState &model, uint64_t now_ns);
};

// 이번 틱에 읽은 locationd 상태. 채널이 아직 없으면 open이 거짓이다.
struct LocalizationRead {
    bool open = false;
    bool read = false;     // 쓰기와 겹쳐 읽지 못한 틱은 거짓(state는 읽다 만 값일 수 있다)
    uint64_t read_ns = 0;  // 읽은 직후 시각(monotonic_now_ns): 2초 신선도
    LocalizationState state;
};

// update_learners의 결과. 저장할 내용과 발행할 학습 상태(timestamp_ns는 main이 찍는다).
struct LearnerOutputs {
    std::optional<std::string> vehicle_json;
    std::optional<std::string> torque_cache;
    std::optional<LearnerState> learner_state;
};

class ControlsTick {
public:
    // force_engaged는 EDGEPILOT_FORCE_ENGAGED(버튼 없이 결합), vehicle_learn_json·torque_learn_cache는
    // 학습 저장 파일 내용(없으면 빈 문자열), seed는 학습기 난수 시드.
    ControlsTick(const ControlParams &params, bool force_engaged, const std::string &vehicle_learn_json,
                 const std::string &torque_learn_cache, uint64_t seed);

    // 다시 읽은 파라미터를 컨트롤러·플래너·크루즈에 넘긴다.
    void apply_params(const ControlParams &params);
    // 수신 CAN 묶음. 100 ms보다 오래된 묶음은 버리고 false다.
    bool on_can_batch(const CanBatch &batch, uint64_t can_now_ns, double now_s);
    // 새로 읽은 차선선 플러그인 결과(LaneState). 같은 틱의 on_model보다 먼저 부른다.
    void on_lane_state(const LaneState &lane) { lane_states_.offer(lane); }
    // 새 모델 상태. 이번 틱 차량 상태와 직전 틱 결과(vEgo·실제 곡률·active)로 플래너를 돌린다. Lanebow 차선은
    // 그 프레임에 고른 LaneState(planning/lane_state_selector)에서 얻는다.
    void on_model(const ModelState &model);
    void on_panda(const PandaState &panda_state) { panda_state_ = panda_state; }
    // lagd 지연과 paramsd·torqued의 locationd 입력.
    void on_localization(const LocalizationRead &localization, uint64_t can_now_ns, double now_s);

    // 컨트롤러·알림·크루즈를 돌려 이번 틱의 ControlState(timestamp_ns 제외)를 만든다. 보낼 프레임은
    // result().frames, 보낼지는 result().should_send다.
    ControlState step(double now_s, uint64_t now_ns);
    // 송신 뒤: 학습기 갱신과 다음 틱 컨트롤러에 학습값 반영.
    // 저장 주기가 됐고 내용이 마지막 저장과 다르면 저장할 내용을, 새 LearnerState가 나왔으면 그것도 담아 돌려준다.
    LearnerOutputs update_learners(double now_s);
    // 학습 파일 쓰기가 실패했다(BackgroundWriter::failures가 늘었다). kLearnerRetryS 뒤 두 파일을 내용이 같아도 다시
    // 내놓는다. 그대로 두면 주차 중처럼 내용이 바뀌지 않는 동안 실패한 파일을 다시 쓰지 않는다.
    void learner_write_failed(double now_s);

    const LateralControlResult &result() const { return last_result_; }
    const PandaGateOutput &panda() const { return panda_; }
    const LateralTarget &target() const { return lateral_target_; }
    const VehicleCanState &vehicle() const { return vehicle_; }
    const AdaptiveCruiseOutput &adaptive_cruise() const { return adaptive_cruise_; }
    const DepartureAlertInput &alert_input() const { return alert_input_; }
    const AdaptiveCruiseInput &cruise_input() const { return cruise_input_; }
    const LateralLearners &learners() const { return learners_; }
    LateralController &controller() { return controller_; }
    const LateralController &controller() const { return controller_; }
    // 재생 도구의 --lane-filter(LateralPlanner::set_lane_filter_weight)
    LateralPlanner &planner() { return planner_; }

private:
    LateralControllerConfig config_;  // 조향 파라미터와 제어 타이밍은 여기에 있다
    AdaptiveCruiseConfig cruise_;
    LateralPlanner planner_;
    LateralController controller_;
    LateralLearners learners_;
    AdaptiveCruiseController adaptive_cruise_controller_;
    DepartureAlertDetector departure_alert_detector_;
    PandaHealthGate panda_gate_;
    PathHoldGate path_gate_;
    EngageEvents events_;
    VehicleCanState vehicle_;
    ModelState model_;
    PandaState panda_state_;
    LateralTarget lateral_target_;
    LaneStateSelector lane_states_;
    AdaptiveCruiseOutput adaptive_cruise_;
    LateralControlResult last_result_;
    PandaGateOutput panda_;
    DepartureAlertInput alert_input_;
    AdaptiveCruiseInput cruise_input_;
    // 마지막으로 온전히 읽은 locationd 상태. 쓰기와 겹쳐 읽지 못한 틱은 상류 SubMaster처럼 이 값을 쓴다.
    LocalizationState localization_{};
    bool localization_seen_ = false;
    bool model_updated_ = false;
    int control_frame_ = 0;
    uint32_t last_logged_alert_event_id_ = 0;
    // 마지막으로 저장하라고 넘긴(처음에는 시작 때 읽은) 학습 파일 내용. 같은 내용은 다시 쓰지 않는다: 주차 중에도
    // torqued가 88 KB를 12초마다 SD에 다시 쓰고 있었다(상류는 주행 중에만 돈다). 쓰기가 실패하면
    // learner_write_failed가 비우고 다시 내놓게 한다(locationd의 lagd 저장과 같다).
    std::string saved_vehicle_json_;
    std::string saved_torque_cache_;
    double learner_retry_at_s_ = 0.0;  // 0이 아니면 이 시각에 두 학습 파일을 다시 내놓는다
};
