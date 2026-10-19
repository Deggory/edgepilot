#include "model/model_input_transform.h"

#include "common/projection.h"
#include "common/utils_math.h"

ModelInputTransform::ModelInputTransform(const AppConfig &config, ModelFrame model_frame)
    : fx_(config.input_warp_fx),
      fy_(config.input_warp_fy),
      cx_(config.input_warp_cx),
      cy_(config.input_warp_cy),
      model_frame_(model_frame) {}

void ModelInputTransform::set_calibration(float roll, float pitch, float yaw) {
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
}

void ModelInputTransform::set_camera_mount(float offset_m, float height_m) {
    camera_offset_m_ = offset_m;
    camera_height_m_ = height_m;
}

void ModelInputTransform::projection_matrix(float *projection) const {
    // openpilot get_warp_matrix(common/transformations/model.py, 0.9.4 modeld도 같다):
    //   camera_from_calib = K · view_from_device · device_from_calib(rpy)
    //   calib_from_model  = inv(K_model · view_from_device)
    // 카메라 중심에서의 순수 회전이다. 예전 구현은 openpilot 0.8 이하의 지면 homography(카메라
    // 높이 1.22 m 아래 지면 점을 축으로 회전)라 pitch·roll이 있으면 가까운 노면이 어긋났다
    // (pitch 1°에 최대 1.3 px, roll 1°에 3.6 px, 모델 이미지 기준). rpy가 0이면 둘은 같다.
    const bool big = model_frame_ == ModelFrame::SmallBigModel;
    const float model_f = big ? 455.0f : 910.0f;                   // sbigmodel_fl / medmodel_fl
    const float model_cx = 256.0f;                                 // 0.5 * 512
    const float model_cy = big ? 0.5f * (256.0f + 47.6f) : 47.6f;  // MEDMODEL_CY = 47.6
    // clang-format off
    const float k[9] = {
        fx_, 0.0f, cx_,
        0.0f, fy_, cy_,
        0.0f, 0.0f, 1.0f,
    };
    // clang-format on
    // view_from_device: 행이 (0,1,0), (0,0,1), (1,0,0). 역행렬은 전치다.
    const float view[9] = {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f};
    const float view_t[9] = {0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    // clang-format off
    const float model_k_inv[9] = {
        1.0f / model_f, 0.0f, -model_cx / model_f,
        0.0f, 1.0f / model_f, -model_cy / model_f,
        0.0f, 0.0f, 1.0f,
    };
    // clang-format on
    float rot[9];
    rotation_from_rpy(roll_, pitch_, yaw_, rot);

    // 가상 카메라 이동(set_camera_mount): 보정 좌표계(x 앞, y 오른쪽, z 아래)에서 가상 카메라가
    // c = (0, offset, 0)에 있으면, 높이 h 아래 도로면 점 P_v(z = h)는 실제 카메라에서
    // P = P_v + c·(z/h) = (I + c·nᵀ/h)·P_v, n = (0,0,1)이다. 도로면 위 물체는 기울어진다
    // (sunnypilot은 ±0.35 m로 제한). offset 0이면 단위행렬.
    const float shift[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, camera_offset_m_ / camera_height_m_, 0.0f, 0.0f, 1.0f};
    float kv[9], kvr[9], kvrs[9], kvrvt[9];
    matmul3(k, view, kv);
    matmul3(kv, rot, kvr);
    matmul3(kvr, shift, kvrs);
    matmul3(kvrs, view_t, kvrvt);
    matmul3(kvrvt, model_k_inv, projection);
}
