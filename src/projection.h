#ifndef PROJECTION_H
#define PROJECTION_H

struct ProjectionState {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float view_from_calib[9] = {
        0.0f, -1.0f, 0.0f,
        0.0f, 0.0f, -1.0f,
        1.0f, 0.0f, 0.0f,
    };
};

ProjectionState make_projection_state(float roll, float pitch, float yaw);
bool project_point(const ProjectionState &projection, float x_forward, float y_left, float z_up,
                   int width, int height, int *px, int *py);
/* project_point가 쓰는 카메라 내부 파라미터(1920x1080 기준). 기본은 MaixCAM2 카메라
 * (app_config.h kCamera*)이고, overlayd가 SUPERCOMBO_CAMERA_INTRINSICS를 따라 바꾼다
 * (예: K230 녹화 리허설에서 차선이 영상과 맞게). */
void projection_set_camera_intrinsics(float fx, float fy, float cx, float cy);

// RPY(rad) -> 3x3 회전행렬(row-major), Rz*Ry*Rx.
void rotation_from_rpy(float roll, float pitch, float yaw, float *rot);

#endif
