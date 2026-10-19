#ifndef SUPERCOMBO_MODEL_H
#define SUPERCOMBO_MODEL_H

#include "common/app_config.h"
#include "common/model_output.h"
#include "maixcam2/maix_gdc_warp.h"
#include "model/ax_engine_session.h"
#include "model/model_input_transform.h"
#include "model/model_output_assembly.h"
#include "model/model_temporal.h"

#include <cstdint>
#include <functional>
#include <vector>

// openpilot master supercombo 코어(axmodel)를 AX630C NPU에서 돌린다. 공개 ONNX가
// 그래프 안에 갖던 이미지·desire·특징 큐는 여기서 관리한다(model_temporal.h).
// 입력: input_imgs/big_input_imgs uint8 [1,12,128,256], desire [1,25,8],
// features_buffer [1,24,512], traffic_convention [1,2]. 출력은 헤드를 나눈 출력들(split_outputs.py)을
// [1,2576] 제자리에 모은다. 입력 워프는 GDC(maix_gdc_warp.h)가 config의 nv12 크기 NV12 소스에서 한다.
class SupercomboModel {
public:
    SupercomboModel(const char *model_file, const AppConfig &config);

    // 재생: NV12 프레임(nv12 크기)을 GDC 소스 버퍼에 복사해 실행한다.
    bool run_frame_nv12(const uint8_t *nv12, std::vector<float> &raw_output);
    // 소스 NV12가 다른 CMM 블록(프레임 링 슬롯)에 있을 때 GDC가 직접 읽는다. 워프 뒤
    // source_still_valid()가 거짓이면(읽는 동안 덮어써짐) 모델 상태를 바꾸지 않고 false.
    bool run_frame_phys(unsigned long long src_phys, std::vector<float> &raw_output,
                        const std::function<bool()> &source_still_valid);
    void set_input_calibration(const float rpy[3]);
    void set_camera_mount(float offset_m, float height_m);
    void set_desire(int desire);

private:
    bool infer(uint64_t t0, uint64_t t1, std::vector<float> &raw_output);
    // EDGEPILOT_PROFILE일 때만 시계를 읽는다(CLOCK_BOOTTIME은 이 보드에서 진짜 syscall이다).
    uint64_t profile_stamp() const;

    enum Input { kRoad, kWide, kDesire, kFeatures, kTraffic, kInputCount };

    AxEngineSession session_;
    int index_[kInputCount] = {};
    int output_parts_[model_output_assembly::kPartCount] = {};  // kParts 순서대로 세션 출력 번호
    bool profile_ = false;                                      // EDGEPILOT_PROFILE: 30프레임마다 단계별 평균 ms
    ModelInputTransform input_transform_;
    ModelInputTransform big_input_transform_;
    GdcWarp gdc_;  // session_ 뒤에 만든다(AX_SYS_Init을 세션이 한다)
    ModelImageHistory road_history_;
    ModelImageHistory wide_history_;
    SupercomboTemporalState temporal_;
    double profile_ms_[4] = {};
    unsigned profile_frames_ = 0;
};

#endif
