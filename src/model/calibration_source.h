#ifndef CALIBRATION_SOURCE_H
#define CALIBRATION_SOURCE_H

// 워프 보정의 출처를 고른다: 모델 포즈 보정기(model/calibration_online)와 광류 보정기
// (model/flow_calibration). 웹 기기 설정이 광류이고 광류 추정이 쓸 만하면 pitch·yaw를 광류에서, roll은
// 포즈 보정기에서 가져온다(광류로는 roll을 잴 수 없다). 아니면 포즈 보정기 값 그대로다.
//
// - 포즈에서 광류로 바꾸는 것은 해제 상태에서만 한다: 설정이 바뀌었을 때도, 광류 추정이 새로 쓸 만해졌을 때도
//   결합 중이면 해제될 때까지 기다린다(웹 콘솔도 결합 중에는 설정을 받지 않는다). 광류가 더는 쓸 만하지 않으면
//   결합 중이어도 포즈로 돌아간다. 쓰고 있는 광류 추정이 움직이는 것(새 블록, 장착 변경)은 결합 중에도 따라간다.
// - 쓸 만함에는 문턱을 둘 둔다: 블록 중앙값 표준편차가 kMaxFlowSpreadRad 안이어야 쓰기 시작하고,
//   kMaxFlowSpreadStayRad를 넘어야 그만 쓴다(경계에서 오가지 않게).
// - 출처가 바뀌거나 광류 추정이 움직이면 워프를 천천히 옮긴다: 남은 차이의 1/kRampTimeS를 초당 옮기되
//   kRampRadPerS보다 느리지는 않게. 모델은 t−4·t 두 프레임과 4.8초 특징 이력을 보므로 한 번에 바꾸면
//   경로가 튄다(카메라 오프셋도 같은 이유로 옮긴다). 0.3° 전환은 6초쯤, 2° 장착 변경은 10초쯤 걸린다.
// - 포즈로 돌아와 다 따라잡은 뒤에는 포즈 값을 그대로 쓴다. 광류 설정이 꺼져 있으면 지금 동작과 같다.
// docs/flow-calibration.md.

#include "common/ipc_messages.h"  // FlowCalibrationState, kCalibrationSource*

#include <cstdint>

class CalibrationSourceSelector {
public:
    static constexpr float kRampRadPerS = 0.05f * 3.14159265358979323846f / 180.0f;
    static constexpr float kRampTimeS = 5.0f;
    static constexpr float kMaxDtS = 0.2f;  // 이보다 긴 간격(멈췄다 다시 온 프레임)은 이만큼만 옮긴다
    // 블록 중앙값들의 표준편차가 이 안이어야 쓰기 시작한다(블록 하나의 잡음은 0.2° 안팎)
    static constexpr float kMaxFlowSpreadRad = 0.5f * 3.14159265358979323846f / 180.0f;
    // 쓰는 중에는 이보다 커져야 그만 쓴다
    static constexpr float kMaxFlowSpreadStayRad = 0.8f * 3.14159265358979323846f / 180.0f;

    // 워프에 쓸 만한 광류 추정인지: 3블록 이상, 블록끼리 맞고(표준편차가 max_spread 안), 포즈 보정기와 같은 각도
    // 한계 안.
    static bool flow_usable(const FlowCalibrationState &flow, float max_spread = kMaxFlowSpreadRad);

    // 모델 프레임마다. pose_rpy는 포즈 보정기 출력(rad), requested_flow는 설정, dt_s는 지난 호출부터의 시간.
    // 첫 호출은 설정과 목표를 그대로 받는다(시작할 때는 차가 서 있다).
    void update(const float pose_rpy[3], const FlowCalibrationState &flow, bool requested_flow, bool engaged,
                float dt_s);
    // 램프 없이 이 값으로 맞춘다(보정 초기화). 출처는 다음 update가 다시 정한다.
    void snap_to(const float rpy[3]);

    void rpy(float out[3]) const;
    bool flow_applied() const { return flow_applied_; }
    bool requested() const { return requested_; }
    uint32_t flags() const { return flags_; }

private:
    bool initialized_ = false;
    bool requested_ = false;     // 마지막으로 본 설정
    bool active_ = false;        // 지금 따르는 설정(결합 중 바뀐 설정은 해제 뒤에 온다)
    bool flow_applied_ = false;  // 지금 목표가 광류다
    bool transition_ = false;    // 광류에서 포즈로 돌아오는 중
    uint32_t flags_ = 0;
    float rpy_[3] = {0.0f, 0.0f, 0.0f};
};

#endif
