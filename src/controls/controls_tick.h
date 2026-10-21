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
// - step의 now_ns는 공유 상태를 다 읽은 직후의 시각이다(그 사이 발행된 모델도 신선하게 본다). 컨트롤러는 plan 나이도
//   이 시각으로 잰다.
// - 학습기 갱신은 송신 뒤라 적합·직렬화 틱이 CAN 송신을 늦추지 않고, 결과는 다음 틱의 컨트롤러가
//   쓴다.

#include "car/vehicle_can.h"
#include "common/ipc_messages.h"
#include "controls/adaptive_cruise.h"
#include "controls/control_holds.h"
#include "controls/control_params.h"
#include "controls/departure_alert.h"
#include "controls/lateral_controller.h"
#include "learners/lateral_learners.h"
#include "planning/lateral_planner.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

// 결합·해제·거부 이벤트 id(ControlsTick::update_events가 갱신하고 전이를 로그에 남긴다). id는 0을 건너뛰어 HUD가 새
// 이벤트를 구분한다.
struct EngageEvents {
    uint32_t engage_id = 0;
    uint32_t disengage_id = 0;
    uint32_t reject_id = 0;
    char reject_block[32] = {};
    bool have_previous = false;
    bool previous_engaged = false;
    bool previous_active = false;
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
    // 학습 저장 파일 내용(없으면 빈 문자열).
    ControlsTick(const ControlParams &params, bool force_engaged, const std::string &vehicle_learn_json,
                 const std::string &torque_learn_cache);

    // 다시 읽은 파라미터를 컨트롤러·플래너·크루즈에 넘긴다.
    void apply_params(const ControlParams &params);
    // 학습한 토크 모델(docs/lateral-nn.md). controlsd와 재생 도구가 시작할 때 한 번 준다.
    void set_lateral_nn(std::shared_ptr<const LateralNnModel> model) { controller_.set_lateral_nn(std::move(model)); }
    // 수신 CAN 묶음. now_ns는 묶음을 큐에서 꺼낸 직후의 시각이다. 100 ms보다 오래됐거나 now_ns보다 늦은 시각의
    // 묶음은 버리고 false다(틱 시작 시각으로 재면 그 틱에 들어온 새 묶음이 미래 시각이 되어 버려진다).
    bool on_can_batch(const CanBatch &batch, uint64_t now_ns, double now_s);
    // 새 모델 상태. 이번 틱 차량 상태와 직전 틱 결과(vEgo·실제 곡률·active)로 플래너를 돌린다.
    void on_model(const ModelState &model);
    void on_panda(const PandaState &panda_state) { panda_state_ = panda_state; }
    // lagd 지연과 paramsd·torqued의 locationd 입력. 나이는 상태를 읽은 직후의 시각(read_ns)으로 잰다.
    void on_localization(const LocalizationRead &localization, double now_s);

    // 컨트롤러·알림·크루즈를 돌려 이번 틱의 ControlState(timestamp_ns 제외)를 만든다. 보낼 프레임은
    // result().frames, 보낼지는 result().should_send다.
    ControlState step(double now_s, uint64_t now_ns);
    // 송신 뒤: 학습기 갱신과 다음 틱 컨트롤러에 학습값 반영. 같은 틱의 step 다음에 부른다(컨트롤러의 신선도 판정을
    // 쓴다). 저장 주기가 됐고 내용이 마지막 저장과 다르면 저장할 내용을, 새 LearnerState가 나왔으면 그것도 담아
    // 돌려준다. 실패한 쓰기는 BackgroundWriter가 다시 쓴다.
    LearnerOutputs update_learners(double now_s);
    // 끝낼 때: 마지막 저장 뒤 바뀐 학습값(paramsd 60초·torqued 12초 주기 사이의 것). locationd처럼 끝낼 때도 남긴다.
    LearnerOutputs final_learner_outputs();

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
    // step의 단계들. 입력은 멤버(이번 틱 상태)에서 읽고, 인자는 step의 지역값이다.
    void update_events(const PathHoldOutput &held, uint64_t now_ns);
    // overlayd/recordd가 읽는 100 Hz 스냅샷(timestamp_ns 제외)
    ControlState make_control_state(const DepartureAlertOutput &departure_alert, bool radar_lead_fresh,
                                    float ego_speed_kph, double now_s) const;
    // 학습기·컨트롤러 상태를 HUD와 웹 콘솔이 읽는 LearnerState로(timestamp_ns 제외)
    LearnerState make_learner_state() const;
    const SteeringParams &steering() const { return controller_.config().steering_params; }

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
    uint64_t logged_nonfinite_ticks_ = 0;  // 로그에 남긴 토크 0(유한하지 않은 입력) 틱 수
    // 마지막으로 저장하라고 넘긴(처음에는 시작 때 읽은) 학습 파일 내용. 같은 내용은 다시 쓰지 않는다: 주차 중에도
    // torqued가 88 KB를 12초마다 SD에 다시 쓰고 있었다(상류는 주행 중에만 돈다).
    std::string saved_vehicle_json_;
    std::string saved_torque_cache_;
};
