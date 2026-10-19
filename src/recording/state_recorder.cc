#include "recording/state_recorder.h"

#include "recording/recording_format.h"
#include "recording/recording_writer.h"

#include <chrono>
#include <cstddef>
#include <thread>

// 기록 버전과 그대로 저장하는 구조체 크기를 한 줄에 묶어, 둘 중 하나만 바꾸면 컴파일이 깨진다.
static_assert(kRecordingVersion == 14 && sizeof(ModelState) == 2648 && sizeof(ControlState) == 240 &&
                  sizeof(PandaState) == 96 && sizeof(LearnerState) == 128 && sizeof(ImuSample) == 40 &&
                  offsetof(ImuBatch, samples) == 16 && sizeof(LocalizationState) == 128,
              "recording v14 pins these payloads; bump kRecordingVersion together");

StateRecorder::StateRecorder(const std::string &topic_suffix)
    : model_topic_(kModelStateTopic + topic_suffix),
      control_topic_(kControlStateTopic + topic_suffix),
      panda_topic_(kPandaStateTopic + topic_suffix),
      learner_topic_(kLearnerStateTopic + topic_suffix),
      imu_topic_(kImuTopic + topic_suffix),
      localization_topic_(kLocalizationStateTopic + topic_suffix) {}

void StateRecorder::record(RecordingWriter &writer, bool include_control) {
    if (model_.attach(model_topic_.c_str()) && model_.poll()) {
        const ModelState &model = model_.latest();
        writer.write_state(RecordType::ModelState, model.model_timestamp_ns, &model, sizeof(model));
    }
    if (include_control && control_.attach(control_topic_.c_str()) && control_.poll()) {
        const ControlState &control = control_.latest();
        writer.write_state(RecordType::ControlState, control.timestamp_ns, &control, sizeof(control));
    }
    if (panda_.attach(panda_topic_.c_str()) && panda_.poll()) {
        const PandaState &panda = panda_.latest();
        writer.write_state(RecordType::PandaState, panda.timestamp_ns, &panda, sizeof(panda));
    }
    if (learner_.attach(learner_topic_.c_str()) && learner_.poll()) {
        const LearnerState &learner = learner_.latest();
        writer.write_state(RecordType::LearnerState, learner.timestamp_ns, &learner, sizeof(learner));
    }
    if (imu_.attach(imu_topic_.c_str()) && imu_.poll()) {
        const ImuBatch &imu = imu_.latest();
        if (imu.count > 0 && imu.count <= kImuBatchMaxSamples)
            writer.write_state(RecordType::Imu, imu.timestamp_ns, &imu,
                               offsetof(ImuBatch, samples) + imu.count * sizeof(ImuSample));
    }
    if (localization_.attach(localization_topic_.c_str()) && localization_.poll()) {
        const LocalizationState &localization = localization_.latest();
        writer.write_state(RecordType::Localization, localization.timestamp_ns, &localization, sizeof(localization));
    }
}

bool StateRecorder::record_control(RecordingWriter &writer, int timeout_ms) {
    if (!control_.attach(control_topic_.c_str())) {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
        return false;
    }
    if (!control_.poll(timeout_ms)) return false;
    const ControlState &control = control_.latest();
    writer.write_state(RecordType::ControlState, control.timestamp_ns, &control, sizeof(control));
    return true;
}
