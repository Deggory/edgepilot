#pragma once

// 녹화 중에 상태 스냅샷(모델, 제어, 판다, 학습기, IMU, locationd)을 이벤트 로그로 옮긴다. 생산자가
// 늦게 뜨거나 없어도 되므로 채널은 생긴 뒤에 붙고, 새 스냅샷만 한 번씩 쓴다. IMU 묶음은 채운
// 샘플까지만 남긴다(100 ms마다 약 10개, 초당 약 4 KB). recordd가 녹화를 켠 동안 루프마다 부른다.
// 100 Hz ControlState는 recordd가 따로 스레드에서 record_control로 발행마다 기다려 쓴다: 루프 한 바퀴가
// 프레임 처리로 10 ms를 넘기면 최신값 채널의 덮어쓴 스냅샷이 빠졌다(녹화의 약 4 %).

#include "common/ipc_channels.h"
#include "common/ipc_messages.h"

#include <string>

class RecordingWriter;

class StateRecorder {
public:
    // topic_suffix는 채널 이름 뒤에 붙는다(시험이 실제 채널과 겹치지 않게).
    explicit StateRecorder(const std::string &topic_suffix = {});
    // include_control이 거짓이면 ControlState는 건너뛴다(record_control을 쓰는 스레드가 맡는다).
    void record(RecordingWriter &writer, bool include_control = true);
    // 다음 ControlState 발행을 timeout_ms까지 기다려 쓴다. 채널이 아직 없으면 timeout_ms 동안 쉰다. 쓴 것이 있으면
    // 참이다. record(include_control = false)와 다른 스레드에서 불러도 된다.
    bool record_control(RecordingWriter &writer, int timeout_ms);

private:
    const std::string model_topic_, control_topic_, panda_topic_, learner_topic_, imu_topic_, localization_topic_;
    Subscription<ModelState> model_;
    Subscription<ControlState> control_;
    Subscription<PandaState> panda_;
    Subscription<LearnerState> learner_;
    Subscription<ImuBatch> imu_;
    Subscription<LocalizationState> localization_;
};
