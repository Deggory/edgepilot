#include "model_input_transform.h"
#include "utils_math.h"

#include "projection.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

void projection_scale_buffer(const float *in, float scale, float *out)
{
    const float transform_out[9] = {
        1.0f / scale, 0.0f, 0.5f,
        0.0f, 1.0f / scale, 0.5f,
        0.0f, 0.0f, 1.0f,
    };
    const float transform_in[9] = {
        scale, 0.0f, -0.5f * scale,
        0.0f, scale, -0.5f * scale,
        0.0f, 0.0f, 1.0f,
    };

    float tmp[9];
    matmul3(in, transform_out, tmp);
    matmul3(transform_in, tmp, out);
}

void ModelInputTransform::SampleMap::resize(size_t size)
{
    offset.resize(size);
    x_step.resize(size);
    y_step.resize(size);
    for (auto &weights : weight)
        weights.resize(size);
}

ModelInputTransform::ModelInputTransform(const AppConfig &config, ModelFrame model_frame)
    : fx_(config.input_warp_fx),
      fy_(config.input_warp_fy),
      cx_(config.input_warp_cx),
      cy_(config.input_warp_cy),
      height_(config.input_warp_height),
      roll_(config.manual_roll),
      pitch_(config.manual_pitch),
      yaw_(config.manual_yaw),
      model_frame_(model_frame)
{
}

void ModelInputTransform::set_calibration(float roll, float pitch, float yaw)
{
    constexpr float kEpsilon = 1e-7f;
    if (std::fabs(roll_ - roll) < kEpsilon &&
        std::fabs(pitch_ - pitch) < kEpsilon &&
        std::fabs(yaw_ - yaw) < kEpsilon) {
        return;
    }

    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    map_valid_ = false;
}

uint8_t ModelInputTransform::sample(const uint8_t *base, const SampleMap &map,
                                    size_t index, int channel)
{
    const uint32_t offset = map.offset[index] + static_cast<uint32_t>(channel);
    const uint32_t x_step = map.x_step[index];
    const uint32_t y_step = map.y_step[index];
    uint32_t sum = 0;
    sum += static_cast<uint32_t>(base[offset]) * map.weight[0][index];
    sum += static_cast<uint32_t>(base[offset + x_step]) * map.weight[1][index];
    sum += static_cast<uint32_t>(base[offset + y_step]) * map.weight[2][index];
    sum += static_cast<uint32_t>(base[offset + y_step + x_step]) * map.weight[3][index];
    const int rounded = (sum + (kWeightScale / 2)) >> kWeightBits;
    return static_cast<uint8_t>(std::min(255, std::max(0, rounded)));
}

void ModelInputTransform::projection_matrix(float *projection) const
{
    // Same model-frame inverses used by openpilot modeld.update_calibration().
    const float ground_from_medmodel_frame[9] = {
        0.00000000e+00f, 0.00000000e+00f, 1.00000000e+00f,
       -1.09890110e-03f, 0.00000000e+00f, 2.81318681e-01f,
       -1.84808520e-20f, 9.00738606e-04f, -4.28751576e-02f,
    };
    const float ground_from_sbigmodel_frame[9] = {
        0.00000000e+00f,  7.31372216e-19f,  1.00000000e+00f,
       -2.19780220e-03f,  4.11497335e-19f,  5.62637363e-01f,
       -5.46146580e-20f,  1.80147721e-03f, -2.73464241e-01f,
    };
    const float k[9] = {
        fx_, 0.0f, cx_,
        0.0f, fy_, cy_,
        0.0f, 0.0f, 1.0f,
    };

    float rot[9];
    rotation_from_rpy(roll_, pitch_, yaw_, rot);

    float device_from_road[9];
    for (int row = 0; row < 3; ++row) {
        device_from_road[row * 3 + 0] = rot[row * 3 + 0];
        device_from_road[row * 3 + 1] = -rot[row * 3 + 1];
        device_from_road[row * 3 + 2] = -rot[row * 3 + 2];
    }

    // view_from_device = [[0,1,0],[0,0,1],[1,0,0]]
    float view_from_road[9];
    for (int col = 0; col < 3; ++col) {
        view_from_road[0 * 3 + col] = device_from_road[1 * 3 + col];
        view_from_road[1 * 3 + col] = device_from_road[2 * 3 + col];
        view_from_road[2 * 3 + col] = device_from_road[0 * 3 + col];
    }

    float extrinsic[12] = {
        view_from_road[0], view_from_road[1], view_from_road[2], 0.0f,
        view_from_road[3], view_from_road[4], view_from_road[5], height_,
        view_from_road[6], view_from_road[7], view_from_road[8], 0.0f,
    };

    float camera_frame_from_road[12];
    matmul34(k, extrinsic, camera_frame_from_road);

    float camera_frame_from_ground[9];
    for (int row = 0; row < 3; ++row) {
        camera_frame_from_ground[row * 3 + 0] = camera_frame_from_road[row * 4 + 0];
        camera_frame_from_ground[row * 3 + 1] = camera_frame_from_road[row * 4 + 1];
        camera_frame_from_ground[row * 3 + 2] = camera_frame_from_road[row * 4 + 3];
    }

    const float *ground_from_model_frame = model_frame_ == ModelFrame::SmallBigModel
        ? ground_from_sbigmodel_frame
        : ground_from_medmodel_frame;
    matmul3(camera_frame_from_ground, ground_from_model_frame, projection);
}

void ModelInputTransform::build_sample_map(const float *projection, int src_w, int src_h,
                                           int dst_w, int dst_h, int src_stride_pixels,
                                           int bytes_per_pixel, int dst_scale,
                                           int dst_x_offset, int dst_y_offset,
                                           SampleMap &map) const
{
    const size_t sample_count = static_cast<size_t>(dst_w) * dst_h;
    map.resize(sample_count);
    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; ++x) {
            const int dst_x = x * dst_scale + dst_x_offset;
            const int dst_y = y * dst_scale + dst_y_offset;
            const float x0 = projection[0] * dst_x + projection[1] * dst_y + projection[2];
            const float y0 = projection[3] * dst_x + projection[4] * dst_y + projection[5];
            const float w0 = projection[6] * dst_x + projection[7] * dst_y + projection[8];
            const size_t index = static_cast<size_t>(y) * dst_w + x;
            int ix = -2;
            int iy = -2;
            if (std::fabs(w0) > 1e-6f) {
                const float sx = x0 / w0;
                const float sy = y0 / w0;
                ix = static_cast<int>(std::floor(sx));
                iy = static_cast<int>(std::floor(sy));
                const float ax = sx - ix;
                const float ay = sy - iy;
                const float weights_f[4] = {
                    (1.0f - ax) * (1.0f - ay),
                    ax * (1.0f - ay),
                    (1.0f - ax) * ay,
                    ax * ay,
                };
                const int xs[4] = {ix, ix + 1, ix, ix + 1};
                const int ys[4] = {iy, iy, iy + 1, iy + 1};
                for (int i = 0; i < 4; ++i) {
                    if (xs[i] >= 0 && xs[i] < src_w && ys[i] >= 0 && ys[i] < src_h) {
                        map.weight[i][index] = static_cast<uint16_t>(
                            std::max(0.0f, std::min(static_cast<float>(kWeightScale), std::round(weights_f[i] * kWeightScale))));
                    } else {
                        map.weight[i][index] = 0;
                    }
                }
            } else {
                for (auto &weights : map.weight)
                    weights[index] = 0;
            }

            const int base_x = std::max(0, std::min(src_w - 1, ix));
            const int base_y = std::max(0, std::min(src_h - 1, iy));
            map.offset[index] = static_cast<uint32_t>(
                (base_y * src_stride_pixels + base_x) * bytes_per_pixel);
            map.x_step[index] = ix >= 0 && ix + 1 < src_w
                ? static_cast<uint16_t>(bytes_per_pixel)
                : 0;
            map.y_step[index] = iy >= 0 && iy + 1 < src_h
                ? static_cast<uint16_t>(src_stride_pixels * bytes_per_pixel)
                : 0;
        }
    }
}

void ModelInputTransform::rebuild_maps(int src_w, int src_h)
{
    if (src_w <= 0 || src_h <= 0 || (src_w & 1) || (src_h & 1))
        throw std::runtime_error("input warp requires positive even NV12 dimensions");

    float projection_y[9];
    float projection_uv[9];
    projection_matrix(projection_y);
    projection_scale_buffer(projection_y, 0.5f, projection_uv);

    constexpr int x_offsets[4] = {0, 0, 1, 1};
    constexpr int y_offsets[4] = {0, 1, 0, 1};
    for (int plane = 0; plane < 4; ++plane) {
        build_sample_map(projection_y, src_w, src_h, kHalfW, kHalfH, src_w, 1,
                         2, x_offsets[plane], y_offsets[plane], y_maps_[plane]);
    }
    build_sample_map(projection_uv, src_w / 2, src_h / 2, kHalfW, kHalfH,
                     src_w / 2, 2, 1, 0, 0, uv_map_);

    map_src_w_ = src_w;
    map_src_h_ = src_h;
    map_valid_ = true;

    std::fprintf(stderr,
                 "input warp=on frame=%s source=%dx%d intrinsics=(fx=%.2f fy=%.2f cx=%.2f cy=%.2f) "
                 "rpy_deg=(%.3f %.3f %.3f)\n",
                 model_frame_ == ModelFrame::SmallBigModel ? "sbigmodel" : "medmodel",
                 src_w, src_h, fx_, fy_, cx_, cy_,
                 rad_to_deg(roll_), rad_to_deg(pitch_), rad_to_deg(yaw_));
}

template <typename OutT>
void ModelInputTransform::warp(const uint8_t *nv12, int src_w, int src_h, OutT *out)
{
    if (!map_valid_ || src_w != map_src_w_ || src_h != map_src_h_)
        rebuild_maps(src_w, src_h);
    const uint8_t *y_src = nv12;
    const uint8_t *uv_src = nv12 + src_w * src_h;
    const int plane_size = kHalfW * kHalfH;

    for (int plane = 0; plane < 4; ++plane) {
        OutT *dst = out + plane * plane_size;
        const SampleMap &map = y_maps_[plane];
        for (size_t i = 0; i < map.size(); ++i)
            dst[i] = static_cast<OutT>(sample(y_src, map, i, 0));
    }

    OutT *u_plane = out + 4 * plane_size;
    OutT *v_plane = out + 5 * plane_size;
    const int u_channel = chroma_vu_ ? 1 : 0;
    for (size_t i = 0; i < uv_map_.size(); ++i) {
        u_plane[i] = static_cast<OutT>(sample(uv_src, uv_map_, i, u_channel));
        v_plane[i] = static_cast<OutT>(sample(uv_src, uv_map_, i, 1 - u_channel));
    }
}

void ModelInputTransform::nv12_to_yuv6_warped(const uint8_t *nv12, int src_w, int src_h, float *out)
{
    warp(nv12, src_w, src_h, out);
}

void ModelInputTransform::nv12_to_yuv6_warped(const uint8_t *nv12, int src_w, int src_h,
                                              std::vector<float> &out)
{
    if (out.size() < static_cast<size_t>(6 * kHalfW * kHalfH))
        out.resize(6 * kHalfW * kHalfH);
    warp(nv12, src_w, src_h, out.data());
}

void ModelInputTransform::nv12_to_yuv6_warped(const uint8_t *nv12, int src_w, int src_h, uint8_t *out)
{
    warp(nv12, src_w, src_h, out);
}

