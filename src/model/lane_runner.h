#ifndef MODEL_LANE_RUNNER_H
#define MODEL_LANE_RUNNER_H

// modeld 안의 차선선 플러그인 실행(보드). 프레임 링 슬롯(NV12, 물리 주소)의 플러그인 영역을 VPP로 잘라 플러그인
// 모델의 입력 텐서에 바로 쓰고(CPU가 화소를 만지지 않는다), NPU를 돌린 뒤 플러그인이 해석한 결과를 LaneState로
// 채운다. NPU는 supercombo와 같은 주 스레드에서 차례로 쓴다(사용자 프로세스가 둘이면 AI-ISP 곁에서 출력이
// 깨진다). docs/lane-line-plugin.md.

#include "common/ipc_messages.h"
#include "model/ax_engine_session.h"
#include "model/lane_plugin.h"
#include "model/lane_state_fill.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class LaneRunner {
public:
    // 플러그인의 모델을 열고 입출력 약속(NV12 u8 입력 하나, fp32 출력)을 본다. 맞지 않으면 예외.
    explicit LaneRunner(std::unique_ptr<LanePlugin> plugin);
    LaneRunner(const LaneRunner &) = delete;
    LaneRunner &operator=(const LaneRunner &) = delete;

    const char *name() const { return plugin_->name(); }
    int net_width() const { return net_w_; }
    int net_height() const { return net_h_; }

    // 프레임 하나. slot_phys는 width x height NV12(chroma_vu면 NV21) 슬롯이고, still_valid는 VPP가 다 읽은 뒤에도
    // 슬롯이 그대로인지 본다. profile은 같은 프레임 supercombo의 노면 높이. 성공이면 true와 out(선·경계·소실점·
    // 도로면 블록, valid = 1). 실패면 false와 why.
    bool run(uint64_t slot_phys, int width, int height, bool chroma_vu, const std::function<bool()> &still_valid,
             const LaneCamera &camera, const LaneRoadProfile &profile, LaneState *out, std::string *why);

private:
    std::unique_ptr<LanePlugin> plugin_;
    AxEngineSession session_;
    int net_w_ = 0, net_h_ = 0;
    std::vector<EpTensor> outputs_;
    std::unique_ptr<EpLaneResult> result_;  // 약 12 KB라 힙에 둔다
    EpLaneGeometry geometry_{};
};

#endif
