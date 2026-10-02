#ifndef PROJECTION_H
#define PROJECTION_H

struct ProjectionState {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    /* 모델 출력 좌표계의 원점(가상 카메라, set_camera_mount)이 실제 카메라에서 오른쪽(+)으로
     * 떨어진 거리. 화면에 그릴 때 모델 점을 실제 카메라 기준으로 되돌린다. */
    float lateral_offset_m = 0.0f;
    float view_from_calib[9] = {
        0.0f, -1.0f, 0.0f,
        0.0f, 0.0f, -1.0f,
        1.0f, 0.0f, 0.0f,
    };
};

ProjectionState make_projection_state(float roll, float pitch, float yaw);
/* 모델 출력 좌표(가상 카메라 기준, x 앞·y 오른쪽·z 아래, 모델 점을 그대로 넘긴다)의 점을
 * 화면 픽셀로. */
bool project_point(const ProjectionState &projection, float x_forward, float y_right, float z_down,
                   int width, int height, int *px, int *py);
/* project_point가 쓰는 카메라 내부 파라미터(1920x1080 기준). 기본은 MaixCAM2 카메라
 * (app_config.h kCamera*)이고, overlayd가 EDGEPILOT_CAMERA_INTRINSICS를 따라 바꾼다
 * (예: K230 녹화 리허설에서 차선이 영상과 맞게). */
void projection_set_camera_intrinsics(float fx, float fy, float cx, float cy);

// RPY(rad) -> 3x3 회전행렬(row-major), Rz*Ry*Rx.
void rotation_from_rpy(float roll, float pitch, float yaw, float *rot);

#endif
