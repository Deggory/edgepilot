#include "common/app_config.h"
#include "common/utils_process.h"
#include "common/utils_math.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

std::string AppConfig::usage(const char *program_name)
{
    return std::string("Usage: ") + (program_name ? program_name : "modeld") +
        " <supercombo.axmodel>";
}

void AppConfig::set_warp_source(unsigned width, unsigned height)
{
    // default_input_warp_*와 같은 연산 순서라 기본값에서는 비트까지 같다.
    const float w = static_cast<float>(width), h = static_cast<float>(height);
    input_warp_fx = camera_fx * w / static_cast<float>(kDefaultSensorWidth);
    input_warp_fy = camera_fy * h / static_cast<float>(kDefaultSensorHeight);
    input_warp_cx = camera_cx * w / static_cast<float>(kDefaultSensorWidth);
    input_warp_cy = camera_cy * h / static_cast<float>(kDefaultSensorHeight);
}

AppConfig AppConfig::from_env_defaults()
{
    AppConfig config;

    const std::string intrinsics = env_string("EDGEPILOT_CAMERA_INTRINSICS");
    if (!intrinsics.empty()) {
        float v[4];
        if (std::sscanf(intrinsics.c_str(), "%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3]) != 4 ||
            v[0] <= 0.0f || v[1] <= 0.0f)
            throw std::runtime_error("EDGEPILOT_CAMERA_INTRINSICS must be fx,fy,cx,cy at 1920x1080");
        config.camera_fx = v[0];
        config.camera_fy = v[1];
        config.camera_cx = v[2];
        config.camera_cy = v[3];
    }
    config.set_warp_source(config.nv12_width, config.nv12_height);

    config.max_frames = env_unsigned("EDGEPILOT_MAX_FRAMES", 0);

    config.replay_nv12_path = env_string("EDGEPILOT_REPLAY_NV12");

    config.manual_calibration = env_present("EDGEPILOT_CALIB_ROLL_DEG") ||
        env_present("EDGEPILOT_CALIB_PITCH_DEG") ||
        env_present("EDGEPILOT_CALIB_YAW_DEG");
    config.calibration_auto = env_flag("EDGEPILOT_CALIB_AUTO", true);
    config.manual_roll = deg_to_rad(env_float("EDGEPILOT_CALIB_ROLL_DEG", 0.0f));
    config.manual_pitch = deg_to_rad(env_float("EDGEPILOT_CALIB_PITCH_DEG", 0.0f));
    config.manual_yaw = deg_to_rad(env_float("EDGEPILOT_CALIB_YAW_DEG", 0.0f));
    config.log_calibration = env_flag("EDGEPILOT_LOG_CALIB");
    config.profile = env_flag("EDGEPILOT_PROFILE");
    return config;
}

AppConfig AppConfig::from_env(int argc, char *argv[])
{
    if (argc != 2)
        throw std::invalid_argument(usage(argc > 0 ? argv[0] : "modeld"));

    AppConfig config = from_env_defaults();
    config.axmodel_path = argv[1];
    return config;
}
