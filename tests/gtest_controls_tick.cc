// controlsd 한 틱(ControlsTick)을 controlsd main처럼 10 ms마다 돌린다. K7 수신 CAN(신호 배치대로 채운
// 바이트), 20 Hz 직선 도로 모델, Panda 상태를 넣고 ControlState와 보낼 CAN을 본다: SET으로 결합해
// LKAS11을 보내고 문이 열리면 해제한다, Panda가 허가하지 않으면 1초 유예 뒤 거부한다, 브레이크
// 페달(AHB1)이 고정형 크루즈 추정을 끈다, 휠 속도가 끊기면 크루즈·발행 속도는 0.5초 뒤 비운다,
// locationd를 못 읽은 틱은 마지막 lagd 지연을 쓴다, 확률 문턱 밑의 모델 앞차는 쓰지 않는다, 새 모델의
// 플랜은 그 모델을 받은 틱에 쓴다.
#include "car/can_frame.h"
#include "common/ipc_messages.h"
#include "common/model_output.h"
#include "controls/control_holds.h"
#include "controls/controls_tick.h"
#include "localization/lateral_lag.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace {

struct K7Inputs {
    float speed_kph = 60.0f;
    int cruise_button = 0;  // CLU11 CF_Clu_CruiseSwState(kCruiseButton*)
    bool door_open = false;
    float brake_stroke_mm = 0.0f;
    bool wheel_speed = true;  // WHL_SPD11을 보낼지
};

void add_frame(CanBatch *batch, uint32_t address, uint8_t bus, const std::array<uint8_t, 8> &data,
               uint32_t length = 8) {
    IpcCanFrame &frame = batch->frames[batch->count++];
    frame.address = address;
    frame.src = bus;
    frame.data_len = length;
    std::copy(data.begin(), data.begin() + length, frame.data);
}

// 한 틱의 K7 수신 프레임: 컨트롤러가 신선해야 하는 메시지 전부와 AHB1.
CanBatch k7_batch(const K7Inputs &in, uint64_t now_ns) {
    CanBatch batch;
    batch.valid = 1;
    batch.timestamp_ns = now_ns;
    std::array<uint8_t, 8> d{};
    add_frame(&batch, kHyundaiLkas11Address, kCameraBus, d);
    d = {};
    set_signal_le(&d, 0, 3, static_cast<uint32_t>(in.cruise_button));
    set_signal_le(&d, 8, 9, static_cast<uint32_t>(std::lround(in.speed_kph * 2.0f)));
    add_frame(&batch, kHyundaiClu11Address, kPowertrainBus, d, 4);
    d = {};
    add_frame(&batch, kHyundaiSas11Address, kMdpsBus, d);
    d = {};
    set_signal_le(&d, 0, 11, 1024);  // 운전자 토크 0
    add_frame(&batch, kHyundaiMdps12Address, kMdpsBus, d);
    d = {};
    set_signal_le(&d, 0, 11, 1023);   // 횡가속 0
    set_signal_le(&d, 40, 13, 4095);  // 요레이트 0
    add_frame(&batch, kHyundaiEsp12Address, kPowertrainBus, d);
    if (in.wheel_speed) {
        d = {};
        for (const int start : {0, 16, 32, 48})
            set_signal_le(&d, start, 14, static_cast<uint32_t>(std::lround(in.speed_kph / 0.03125f)));
        add_frame(&batch, kHyundaiWhlSpd11Address, kPowertrainBus, d);
    }
    d = {};
    add_frame(&batch, kHyundaiTcs13Address, kPowertrainBus, d);
    add_frame(&batch, kHyundaiTcs15Address, kPowertrainBus, d);
    add_frame(&batch, kHyundaiEEms11Address, kPowertrainBus, d);
    add_frame(&batch, kHyundaiCgw2Address, kPowertrainBus, d);
    set_signal_le(&d, 16, 4, kGearDrive);
    add_frame(&batch, kHyundaiElectGearAddress, kPowertrainBus, d);
    d = {};
    set_signal_le(&d, 10, 2, 1);  // 안전벨트 착용
    if (in.door_open) set_signal_le(&d, 8, 2, 1);
    add_frame(&batch, kHyundaiCgw1Address, kPowertrainBus, d);
    d = {};
    set_signal_le(&d, 8, 16, static_cast<uint32_t>(std::lround(in.brake_stroke_mm * 10.0f)));
    add_frame(&batch, kHyundaiAhb1Address, kPowertrainBus, d);
    return batch;
}

// 66 m 앞까지 뻗은 직선 plan, 보정 완료.
ModelState straight_model(uint64_t now_ns) {
    ModelState model;
    model.valid = 1;
    model.capture_timestamp_ns = now_ns;
    model.model_timestamp_ns = now_ns;
    model.plan_probability = 0.9f;
    model.calibration.status = 1;
    for (int i = 0; i < kTrajectorySize; ++i) model.plan[i].x = 2.0f * static_cast<float>(i + 1);
    return model;
}

PandaState panda_state(uint64_t now_ns, bool controls_allowed) {
    PandaState state;
    state.timestamp_ns = now_ns;
    state.connected = state.comms_healthy = state.tx_enabled = 1;
    state.controls_allowed = controls_allowed ? 1U : 0U;
    state.safety_mode = kExpectedPandaSafetyModel;
    state.safety_param = kExpectedPandaSafetyParam;
    return state;
}

// controlsd main의 순서대로 틱을 돈다(Laneless: 차선 없이 plan으로 경로를 만든다).
class Drive {
public:
    explicit Drive(const ControlParams &params = laneless_params())
        : tick_(params, false, std::string(), std::string(), 1) {
        tick_.controller().set_clock([this] { return now_ns_; });
    }

    static ControlParams laneless_params() {
        ControlParams params;
        params.steering.laneless_mode = true;
        return params;
    }

    // seconds 동안 틱을 돌리고 마지막 ControlState를 돌려준다.
    ControlState run(double seconds, const K7Inputs &in, bool panda_allows = true) {
        ControlState state;
        for (int i = 0; i < static_cast<int>(std::lround(seconds * 100.0)); ++i) {
            now_ns_ += 10'000'000ULL;
            const double now_s = static_cast<double>(now_ns_) * 1e-9;
            tick_.on_can_batch(k7_batch(in, now_ns_), now_ns_, now_s);
            if (ticks_++ % 5 == 0) tick_.on_model(straight_model(now_ns_));
            tick_.on_panda(panda_state(now_ns_, panda_allows));
            tick_.on_localization(LocalizationRead{}, now_ns_, now_s);
            state = tick_.step(now_s, now_ns_);
            tick_.update_learners(now_s);
        }
        return state;
    }

    // 버튼을 0.05 s 누르고 뗀다.
    ControlState press(int button, K7Inputs in, bool panda_allows = true) {
        in.cruise_button = button;
        run(0.05, in, panda_allows);
        in.cruise_button = 0;
        return run(0.01, in, panda_allows);
    }

    const ControlsTick &tick() const { return tick_; }

private:
    uint64_t now_ns_ = 1'000'000'000ULL;
    unsigned ticks_ = 0;
    ControlsTick tick_;
};

bool sends(const ControlsTick &tick, uint32_t address, uint8_t bus) {
    const LateralControlResult &result = tick.result();
    if (!result.should_send) return false;
    return std::any_of(result.frames.begin(), result.frames.end(),
                       [&](const CanFrame &frame) { return frame.address == address && frame.bus == bus; });
}

// 학습 파일은 내용이 바뀔 때만 쓴다: CAN이 없어 배울 것이 없으면 처음 한 번 뒤로는 다시 쓰지 않는다.
TEST(ControlsTick, LearnerFilesAreWrittenOnlyWhenTheyChange) {
    const ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string(), 1);
    int vehicle_writes = 0, torque_writes = 0;
    for (int i = 0; i < 13000; ++i) {  // 100 Hz로 130 s: paramsd 저장 주기 2번, torqued 10번
        const LearnerOutputs out = tick.update_learners(1.0 + i * 0.01);
        vehicle_writes += out.vehicle_json ? 1 : 0;
        torque_writes += out.torque_cache ? 1 : 0;
    }
    EXPECT_EQ(vehicle_writes, 1);
    EXPECT_EQ(torque_writes, 1);

    // 시작 때 읽은 내용과 같으면 첫 저장도 하지 않는다
    ControlsTick restarted(params, false, tick.learners().vehicle_persist_json(), tick.learners().torque_cache(), 1);
    torque_writes = 0;
    for (int i = 0; i < 3000; ++i) torque_writes += restarted.update_learners(1.0 + i * 0.01).torque_cache ? 1 : 0;
    EXPECT_EQ(torque_writes, 0);
}

// 학습 파일 쓰기가 실패하면(BackgroundWriter::failures가 늘면) 내용이 같아도 두 파일을 다시 내놓는다: 다음 저장 주기나
// 늦어도 12 s 뒤. 주차 중처럼 내용이 바뀌지 않으면 그렇지 않고는 다시 쓸 일이 없다.
TEST(ControlsTick, FailedLearnerWritesAreRetried) {
    const ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string(), 1);
    double now_s = 1.0;
    for (int i = 0; i < 3000; ++i, now_s += 0.01) tick.update_learners(now_s);  // 첫 저장
    int vehicle_writes = 0, torque_writes = 0;
    for (int i = 0; i < 3000; ++i, now_s += 0.01) {
        const LearnerOutputs out = tick.update_learners(now_s);
        vehicle_writes += out.vehicle_json ? 1 : 0;
        torque_writes += out.torque_cache ? 1 : 0;
    }
    ASSERT_EQ(vehicle_writes + torque_writes, 0) << "nothing changed, nothing written";

    const double failed_at = now_s;
    tick.learner_write_failed(failed_at);
    double torque_retried_at = 0.0;
    for (int i = 0; i < 3000; ++i, now_s += 0.01) {
        const LearnerOutputs out = tick.update_learners(now_s);
        if (out.torque_cache && torque_retried_at == 0.0) torque_retried_at = now_s;
        vehicle_writes += out.vehicle_json ? 1 : 0;
        torque_writes += out.torque_cache ? 1 : 0;
    }
    EXPECT_EQ(vehicle_writes, 1);
    EXPECT_EQ(torque_writes, 1);
    EXPECT_LE(torque_retried_at, failed_at + 12.0 + 0.02) << "retried within 12 s";
}

TEST(ControlsTick, EngagesOnSetAndDisengagesWhenADoorOpens) {
    Drive drive;
    K7Inputs in;
    ControlState state = drive.run(1.0, in);
    ASSERT_EQ(state.engaged, 0);
    ASSERT_STREQ(state.active_block, "not_engaged");
    ASSERT_EQ(state.vehicle_fresh, 1) << "수신 프레임이 컨트롤러가 보는 메시지를 다 채운다";

    state = drive.press(kCruiseButtonSet, in);
    ASSERT_EQ(state.engaged, 1);
    ASSERT_EQ(state.engage_event_id, 1);
    state = drive.run(1.0, in);  // 경로 유효 0.5 s 디바운스를 넘긴다
    ASSERT_EQ(state.active, 1) << state.active_block;
    ASSERT_TRUE(sends(drive.tick(), kHyundaiLkas11Address, kPowertrainBus));
    ASSERT_TRUE(sends(drive.tick(), kHyundaiLkas11Address, kMdpsBus)) << "MDPS 버스에도 LKAS11을 보낸다";

    in.door_open = true;
    state = drive.run(0.05, in);
    ASSERT_EQ(state.engaged, 0) << "문 열림은 즉시 해제한다";
    ASSERT_EQ(state.active, 0);
    ASSERT_EQ(state.disengage_event_id, 1);
}

TEST(ControlsTick, PandaRefusalRejectsEngageAfterTheGrace) {
    Drive drive;
    K7Inputs in;
    drive.run(1.0, in, false);
    ControlState state = drive.press(kCruiseButtonSet, in, false);
    ASSERT_EQ(state.engage_reject_event_id, 0) << "Panda 허가를 1초 기다린다";
    state = drive.run(1.1, in, false);
    ASSERT_EQ(state.engaged, 0);
    ASSERT_EQ(state.engage_reject_event_id, 1);
    ASSERT_STREQ(state.engage_reject_block, "panda_controls_off");
}

TEST(ControlsTick, BrakePedalCancelsTheCruiseEstimate) {
    Drive drive;
    K7Inputs in;
    drive.run(1.0, in);
    ControlState state = drive.press(kCruiseButtonSet, in);
    ASSERT_EQ(state.cruise_active, 1) << "SET은 고정형 크루즈 추정을 켠다";
    in.brake_stroke_mm = 15.0f;
    state = drive.run(0.05, in);
    ASSERT_EQ(state.cruise_active, 0) << "AHB1 페달 15 mm는 크루즈 추정을 끈다";
    ASSERT_EQ(state.engaged, 1) << "브레이크는 횡제어를 해제하지 않는다";
}

TEST(ControlsTick, CruiseSpeedGoesStaleAfterHalfASecond) {
    ControlParams params = Drive::laneless_params();
    params.timing.vehicle_state_timeout_ms = 1000;
    Drive drive(params);
    K7Inputs in;
    ControlState state = drive.run(1.0, in);
    ASSERT_FLOAT_EQ(state.ego_speed_kph, 60.0f);
    in.wheel_speed = false;
    state = drive.run(0.7, in);
    ASSERT_TRUE(std::isfinite(drive.tick().result().control_speed_kph))
        << "조향은 설정한 1 s까지 마지막 휠 속도를 쓴다";
    ASSERT_TRUE(std::isnan(state.ego_speed_kph)) << "크루즈·경보·발행 속도는 0.5 s가 지나면 쓰지 않는다";
}

// locationd 상태를 읽다가 쓰기와 겹친 틱(read 거짓, state는 읽다 만 값)은 상류 SubMaster처럼 마지막으로
// 온전히 읽은 값을 쓴다. 그 틱에 lagd 지연을 버리면 토크 요청 버퍼를 읽는 위치가 바뀌어 토크가 튄다.
TEST(ControlsTick, TornLocalizationReadKeepsTheLastLagEstimate) {
    ControlParams params;
    params.steering.use_live_delay = true;
    ControlsTick tick(params, false, std::string(), std::string(), 1);
    LocalizationRead good;
    good.open = good.read = true;
    good.read_ns = 10'000'000'000ULL;
    good.state.timestamp_ns = good.read_ns - 50'000'000ULL;
    good.state.lateral_delay_s = 0.3f;
    good.state.lag_status = static_cast<uint32_t>(LateralLagStatus::Estimated);
    tick.on_localization(good, good.read_ns, 10.0);
    ASSERT_FLOAT_EQ(tick.controller().plan_delay_s(), 0.3f);

    LocalizationRead torn;
    torn.open = true;
    torn.read_ns = good.read_ns + 10'000'000ULL;
    torn.state.lateral_delay_s = 0.6f;  // 읽다 만 값
    tick.on_localization(torn, torn.read_ns, 10.01);
    ASSERT_FLOAT_EQ(tick.controller().plan_delay_s(), 0.3f) << "못 읽은 틱은 마지막 값을 쓴다";

    torn.read_ns = good.state.timestamp_ns + 2'100'000'000ULL;
    tick.on_localization(torn, torn.read_ns, 12.1);
    ASSERT_FLOAT_EQ(tick.controller().plan_delay_s(), params.steering.steer_actuator_delay)
        << "마지막 값도 2초가 지나면 낡았다";
}

// 모델 앞차는 openpilot radard처럼 고정한 확률(0.5) 밑이면 쓰지 않는다. 출발 알림과 비전 크루즈가 같은
// 판정(observe_vision_lead)을 받는다.
TEST(ControlsTick, VisionLeadNeedsTheFixedProbability) {
    ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string(), 1);
    const uint64_t now_ns = 2'000'000'000ULL;
    ModelState model = straight_model(now_ns);
    model.lead.valid = 1;
    model.lead.x = 30.0f;
    model.lead.velocity = 15.0f;
    model.lead.probability = kLeadProbabilityThreshold - 0.05f;
    tick.on_model(model);
    tick.step(2.0, now_ns);
    ASSERT_FALSE(tick.alert_input().lead_valid);
    model.lead.probability = kLeadProbabilityThreshold + 0.05f;
    tick.on_model(model);
    tick.step(2.0, now_ns);
    ASSERT_TRUE(tick.alert_input().lead_valid);
}

// 비전 크루즈는 modeld의 앞차 추적(모델 거리와 광류를 섞은 값)이 있으면 그 거리와 상대 속도를 받고, 출발
// 알림은 모델 앞차를 그대로 받는다(늦으면 안 된다). 추적이 없으면 크루즈도 모델 앞차다.
TEST(ControlsTick, CruiseTakesTheLeadTrackAndTheAlertTheModelLead) {
    ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string(), 1);
    const uint64_t now_ns = 2'000'000'000ULL;
    ModelState model = straight_model(now_ns);
    model.lead.valid = 1;
    model.lead.x = 30.0f;
    model.lead.velocity = 15.0f;
    model.lead.probability = 0.9f;
    model.lead_track.valid = 1;
    model.lead_track.distance_m = 32.0f;
    model.lead_track.relative_speed_mps = -2.0f;
    tick.on_model(model);
    tick.step(2.0, now_ns);
    ASSERT_TRUE(tick.cruise_input().vision_lead_valid);
    EXPECT_FLOAT_EQ(tick.cruise_input().vision_lead_distance_m, 32.0f - kRadarToCameraDistanceM);
    EXPECT_FLOAT_EQ(tick.cruise_input().vision_lead_relative_speed_mps, -2.0f);
    ASSERT_TRUE(tick.alert_input().lead_valid);
    EXPECT_FLOAT_EQ(tick.alert_input().lead_distance_m, 30.0f - kRadarToCameraDistanceM) << "출발 알림은 모델 앞차";

    model.lead_track.valid = 0;  // 추적이 없으면 크루즈도 모델 앞차
    tick.on_model(model);
    tick.step(2.0, now_ns);
    EXPECT_FLOAT_EQ(tick.cruise_input().vision_lead_distance_m, 30.0f - kRadarToCameraDistanceM);

    model.lead_track.valid = 1;  // 모델 앞차가 확률 문턱 밑이면 추적이 있어도 쓰지 않는다
    model.lead.probability = kLeadProbabilityThreshold - 0.05f;
    tick.on_model(model);
    tick.step(2.0, now_ns);
    EXPECT_FALSE(tick.cruise_input().vision_lead_valid);
}

// Lane 모드 모델: 차 양옆 1.75 m의 차선 둘과 직진 plan.
ModelState lane_model(uint64_t now_ns) {
    const float v = 20.0f;
    ModelState model;
    model.valid = 1;
    model.capture_timestamp_ns = model.model_timestamp_ns = now_ns;
    model.plan_probability = 0.9f;
    model.calibration.status = 1;
    for (int i = 0; i < kTrajectorySize; ++i) {
        const float t = model_t_idx(i);
        model.model_t[i] = model.lane_t[i] = t;
        model.plan[i] = {v * t, 0.0f, 0.0f};
        model.lanes[1][i] = {v * t, -1.75f, 0.0f};
        model.lanes[2][i] = {v * t, 1.75f, 0.0f};
    }
    model.lane_probabilities[1] = model.lane_probabilities[2] = 0.99f;
    model.lane_stds[1] = model.lane_stds[2] = 0.05f;
    model.desire_state[0] = 1.0f;
    return model;
}

// 웹 콘솔·SIGHUP으로 다시 읽은 파라미터는 다음 모델 프레임부터 플래너에 들어간다(apply_params → 플래너).
TEST(ControlsTick, ReloadedParamsReachThePlanner) {
    ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string(), 1);
    uint64_t now_ns = 1'000'000'000ULL;
    for (int frame = 0; frame < 40; ++frame, now_ns += 50'000'000ULL) {
        tick.on_model(lane_model(now_ns));
        tick.step(static_cast<double>(now_ns) * 1e-9, now_ns);
    }
    ASSERT_FALSE(tick.target().laneless_mode) << "settled lanes give a lane-mode plan";
    params.steering.laneless_mode = true;
    tick.apply_params(params);
    tick.on_model(lane_model(now_ns));
    tick.step(static_cast<double>(now_ns) * 1e-9, now_ns);
    EXPECT_TRUE(tick.target().laneless_mode) << "the first frame after the reload plans laneless";
}

// 새 모델의 플랜은 그 모델을 받은 틱의 step이 바로 쓴다(예전 작업 스레드는 한 틱 늦어, 이 틱에는 직전
// 프레임의 플랜이 나왔다).
TEST(ControlsTick, UsesTheNewPlanInTheSameTick) {
    const ControlParams params;
    ControlsTick tick(params, false, std::string(), std::string(), 1);
    uint64_t now_ns = 1'000'000'000ULL;
    for (int frame = 0; frame < 20; ++frame, now_ns += 50'000'000ULL) {
        tick.on_model(lane_model(now_ns));
        tick.step(static_cast<double>(now_ns) * 1e-9, now_ns);
        ASSERT_TRUE(tick.target().valid) << frame;
        ASSERT_EQ(tick.target().capture_timestamp_ns, now_ns) << "프레임 " << frame << "의 플랜을 같은 틱에 써야 한다";
    }
}

}  // namespace
