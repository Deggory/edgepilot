#pragma once

/* 녹화 중에 상태 스냅샷(모델, 제어, 판다, 학습기, IMU, locationd)을 이벤트 로그로 옮긴다. 생산자가
 * 늦게 뜨거나 없어도 되므로 채널은 생긴 뒤에 붙고, 새 스냅샷만 한 번씩 쓴다. IMU 묶음은 채운
 * 샘플까지만 남긴다(100 ms마다 약 10개, 초당 약 4 KB). recordd가 녹화를 켠 동안 루프마다 부른다. */

#include "common/ipc_channels.h"
#include "common/ipc_messages.h"

#include <string>

class RecordingWriter;

class StateRecorder {
public:
    // topic_suffix는 채널 이름 뒤에 붙는다(시험이 실제 채널과 겹치지 않게).
    explicit StateRecorder(const std::string &topic_suffix = {});
    void record(RecordingWriter &writer);

private:
    const std::string model_topic_, control_topic_, panda_topic_, learner_topic_, imu_topic_,
        localization_topic_;
    Subscription<ModelState> model_;
    Subscription<ControlState> control_;
    Subscription<PandaState> panda_;
    Subscription<LearnerState> learner_;
    Subscription<ImuBatch> imu_;
    Subscription<LocalizationState> localization_;
};
