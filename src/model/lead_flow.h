#pragma once

// 앞차 확대율. 모델 앞차를 원본 영상에 투영한 상자 안의 점을 1프레임씩 따라가고(IVE, model/flow_worker가
// 광류 보정과 한 번에 부르고 앞차 점만 거꾸로 한 번 더 부른다), kLeadWindowSteps 프레임 전 위치와 지금 위치에
// 유사변환을 맞춰 크기 비 s를 낸다. 확대율 r = ln(s) / 창 길이이면 상대 속도는 -x·r, 충돌까지 시간은 1/r이다
// (LeadFlowState).
//
// 1프레임씩 이어 붙인 위치만 쓰면 걸음마다의 오차가 쌓이고(창 10걸음이면 √10배), 불빛·번호판 둘레에서
// 미끄러지는 점이 남는다. 그래서 세 가지를 더한다(2026-10-11, OpenCV로 이 알고리즘을 흉내 내 고름).
//  - 걸음마다 앞뒤 검사: 되돌려 따라간 위치가 직전 위치에서 0.3 px 안이어야 산다.
//  - 살아 있는 점이 kLeadTopUpPoints보다 적으면 상자에서 새로 골라 채운다(창은 끊지 않는다).
//  - 창이 찬 점은 10프레임 전 그 자리의 템플릿(9x9)을 지금 프레임에 맞춰 위치를 다듬는다. 쌓인 오차가 없어진다.
// 보드 재생에서 모델 거리 추세(±2초)와의 상관(25~40 m)은 밤 0.86, 낮 0.69였다(이어 붙이기만 0.29/0.49, 모델
// 앞차 속도 0.39/0.43, docs/lead-flow.md). 이 파일은 하드웨어와 무관한 부분(투영, 점 고르기, 맞춤, 창)이다.

#include "common/ipc_messages.h"     // LeadFlowState
#include "common/model_output.h"     // kLeadProbabilityThreshold
#include "model/flow_calibration.h"  // FlowIntrinsics

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

constexpr int kLeadMaxPoints = 64;    // 따라가는 점
constexpr int kLeadTopUpPoints = 32;  // 살아 있는 점이 이보다 적으면 새로 골라 채운다
constexpr int kLeadWindowSteps = 10;  // 창 길이(프레임). 20 fps에서 0.5초
constexpr float kLeadMinDistanceM = 8.0f;
constexpr float kLeadMaxDistanceM = 80.0f;
constexpr double kLeadInlierPx = 0.7;  // 유사변환 잔차 문턱
constexpr int kLeadTemplateHalf = 4;   // 다듬기 템플릿 9x9

// 원본 영상의 화소 상자 [x0, x1) x [y0, y1)
struct LeadBox {
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    bool empty() const { return x1 <= x0 || y1 <= y0; }
    int width() const { return x1 - x0; }
    int height() const { return y1 - y0; }
};

// 모델 앞차(x 앞, y 오른쪽이 +, 카메라 기준 m)의 뒷면(폭 1.8 m, 도로 위 0.3~1.4 m)을 원본 영상에 투영하고
// 배경이 덜 들게 줄인 상자. rpy는 워프에 쓴 보정(rad), camera_height_m는 도로에서 카메라까지,
// camera_offset_m는 장착 좌우 위치(웹 기기 설정과 같은 부호: 모델은 실제 점을 y - offset으로 본다).
// 상자가 너무 작거나 영상 밖이면 false.
bool project_lead_box(const FlowIntrinsics &k, const float rpy[3], float camera_height_m, float camera_offset_m,
                      float lead_x, float lead_y, int width, int height, LeadBox *box);

// 상자 안에서 Shi-Tomasi 최소 고유값(3x3 Sobel, 3x3 창)이 큰 점을 원본 해상도로 고른다. 축소 단의 코너
// 응답(광류 보정용)은 40 m 밖 상자(25 px 안팎)에 점이 몇 개 안 남는다. 최댓값의 1%보다 약하거나 이미 고른
// 점과 3화소 안인 점은 건너뛴다. y는 Y 평면(행 간격 stride), 상자 둘레 2화소까지 영상 안이어야 한다.
// 고른 수를 돌려준다.
int pick_lead_corners(const uint8_t *y, int stride, const LeadBox &box, int max_points, float *xy);

// 점 쌍 n개(a에서 b로)에 유사변환(크기·회전·이동)을 RANSAC으로 맞추고 맞는 점으로 다시 푼다. 상자 안에 든
// 도로·배경 점은 앞차와 다르게 움직여 빠진다.
struct ScaleFit {
    bool valid = false;
    double scale = 1.0;
    int inliers = 0;
};
ScaleFit fit_similarity_scale(const float *a, const float *b, int n, double inlier_px);

// 이 프레임의 모델 앞차와 투영에 필요한 값
struct LeadFlowInput {
    bool valid = false;
    float probability = 0.0f;
    float x_m = 0.0f;   // 카메라 기준
    float y_m = 0.0f;   // 오른쪽이 +
    float rpy[3] = {};  // 워프에 쓴 보정
    float camera_height_m = 0.0f;
    float camera_offset_m = 0.0f;
};

// 따라갈 만한 앞차인지: 있고, 확률 문턱(controlsd와 같다) 이상이고, 거리가 범위 안이다
inline bool lead_flow_wanted(const LeadFlowInput &lead) {
    return lead.valid && lead.probability >= kLeadProbabilityThreshold && lead.x_m >= kLeadMinDistanceM &&
           lead.x_m <= kLeadMaxDistanceM;
}

// 앞차 점의 창. 프레임마다 begin → (점이 있으면) points를 LK에 넣고, 그 결과를 거꾸로 LK에 넣어 둘 다
// advance에 → top_up 순으로 부른다. 점마다 최근 kLeadWindowSteps+1 프레임의 위치와 템플릿을 기억해, 창 내내
// 살아 있는 점으로 매 프레임 확대율을 낸다. 앞차가 바뀌거나(거리·옆 위치가 튄다) 프레임이 이어지지 않으면
// 점을 버린다.
class LeadFlowTracker {
public:
    LeadFlowTracker(const FlowIntrinsics &k, int width, int height);

    // 프레임 시작. y는 이 프레임의 Y 평면(CPU 매핑, 행 간격 stride)으로, 상자 둘레를 한 번 복사해 두고 점
    // 고르기·템플릿·다듬기에 쓴다(호출자는 그동안 버퍼가 그대로였는지 확인한다). invalidate가 있으면 복사할 행
    // 범위(y에서의 바이트 오프셋, 길이)를 읽기 전에 넘긴다(캐시 매핑). consecutive가 false면(프레임을 건너뛰었다)
    // 점을 버린다. 앞차를 따라갈 수 없으면 점과 확대율을 버리고 false. 돌려준 뒤 points()가 이번에 따라갈 점이다.
    bool begin(const LeadFlowInput &lead, uint64_t frame_id, double t_s, bool consecutive, const uint8_t *y, int stride,
               const std::function<void(size_t, size_t)> &invalidate = {});
    // 따라갈 점(원본 좌표, 직전 프레임 위치)을 xy에 쓰고 수를 돌려준다
    int points(float *xy);
    // points()를 LK로 이 프레임까지 따라간 결과(xy, ok)와, 그 위치를 직전 프레임으로 되돌려 따라간 결과
    // (back, back_ok). 앞뒤가 맞지 않거나 상자를 벗어난 점을 버리고 확대율을 낸다.
    void advance(const float *xy, const uint8_t *ok, const float *back, const uint8_t *back_ok);
    // 살아 있는 점이 kLeadTopUpPoints보다 적으면 이 프레임의 상자에서 새 점을 골라 빈자리를 채운다
    void top_up();
    void reset();
    const LeadBox &box() const { return box_; }
    int alive_points() const;
    LeadFlowState state() const { return state_; }

private:
    static constexpr int kSlots = kLeadWindowSteps + 1;
    static constexpr int kTemplateSide = 2 * kLeadTemplateHalf + 3;  // 기울기를 낼 테두리 1화소까지
    static constexpr int kTemplateSize = kTemplateSide * kTemplateSide;

    void drop_points();
    void copy_roi(const uint8_t *y, int stride, const std::function<void(size_t, size_t)> &invalidate);
    bool sample_patch(double x, double y, int half, float *out) const;
    bool refine(const float *tmpl, float *xy) const;
    int slot(int back) const { return (head_ - back + kSlots) % kSlots; }
    float *pos(int point, int back) { return &hist_[(static_cast<size_t>(point) * kSlots + slot(back)) * 2]; }
    float *tmpl(int point, int back) {
        return &tmpl_[(static_cast<size_t>(point) * kSlots + slot(back)) * kTemplateSize];
    }

    FlowIntrinsics k_;
    int width_, height_;
    LeadBox box_;  // 이 프레임의 앞차 상자. 비어 있으면 앞차가 없다
    float last_x_ = 0.0f, last_y_ = 0.0f;
    int head_ = 0;             // 지금 프레임의 슬롯
    std::vector<float> hist_;  // 점 x 슬롯 x (x, y)
    std::vector<float> tmpl_;  // 점 x 슬롯 x 템플릿(그 프레임에서 그 위치 둘레)
    std::vector<int> age_;     // 점마다 따라간 걸음 수
    std::vector<uint8_t> alive_;
    double times_[kSlots] = {};  // 슬롯의 프레임 시각(s)
    std::vector<uint8_t> roi_;   // 이 프레임의 상자 둘레 복사본
    int roi_x0_ = 0, roi_y0_ = 0, roi_w_ = 0, roi_h_ = 0;
    std::vector<int> live_;  // points()가 내보낸 점의 번호
    std::vector<float> fit_a_, fit_b_;
    std::vector<float> corners_;  // top_up이 상자에서 고른 점(복사본 좌표)
    LeadFlowState state_{};
};
