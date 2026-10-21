#pragma once

// 운전자 조향 토크로 '운전자가 핸들을 돌린다'를 판정한다(openpilot update_steering_pressed와 같은 카운터). 기준을
// 넘는 틱마다 +1, 아니면 -1(0~11)이고, kMinCount를 넘으면(6틱 연속) 돌린다고 본다. 컨트롤러와 학습기 재생 도구가
// 같이 쓴다.

#include <algorithm>
#include <cstdlib>

class SteeringPressedFilter {
public:
    static constexpr int kMinCount = 5;

    bool update(int driver_torque, int threshold) {
        counter_ = std::clamp(counter_ + (std::abs(driver_torque) > threshold ? 1 : -1), 0, kMinCount * 2 + 1);
        return counter_ > kMinCount;
    }

private:
    int counter_ = 0;
};
