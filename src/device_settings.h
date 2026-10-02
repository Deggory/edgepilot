#ifndef DEVICE_SETTINGS_H
#define DEVICE_SETTINGS_H

/* 웹 기기 설정(params/display.json) 중 런타임이 읽는 값. modeld(카메라 장착)와 overlayd(알림음,
 * HUD 투영)가 각자 메인 루프에서 poll()을 부른다. 1초에 한 번만 stat하고 수정 시각이 바뀌었을 때만
 * 다시 읽는다. */

#include "model_output.h"
#include "utils_json.h"
#include "utils_process.h"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>

struct DeviceSettings {
    static constexpr float kMaxCameraOffsetM = 0.35f;  // sunnypilot과 같은 한계(넘으면 물체가 기운다)

    float alert_volume_percent = NAN;  // 없으면 NAN(시작 크기 유지)
    float camera_offset_m = 0.0f;      // 가상 카메라를 오른쪽(+)으로: 차가 왼쪽으로 간다
    float camera_height_m = kModelHeight;
};

class DeviceSettingsFile {
public:
    DeviceSettingsFile() : path_(param_path("display.json")) {}

    // 새로 읽었으면 true. 첫 호출은 파일이 있으면 항상 읽는다.
    bool poll(uint64_t now_ns, DeviceSettings *out)
    {
        if (now_ns < next_check_ns_) return false;
        next_check_ns_ = now_ns + 1'000'000'000ULL;
        struct stat st {};
        if (stat(path_.c_str(), &st) != 0) return false;
        if (read_once_ && st.st_mtim.tv_sec == mtime_.tv_sec && st.st_mtim.tv_nsec == mtime_.tv_nsec) return false;
        mtime_ = st.st_mtim;
        read_once_ = true;
        std::ifstream file(path_);
        const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        DeviceSettings s;
        float value = 0.0f;
        if (parse_json_float_value(text, "alert_volume_percent", &value) && std::isfinite(value))
            s.alert_volume_percent = value;
        if (parse_json_float_value(text, "camera_offset_m", &value) && std::isfinite(value))
            s.camera_offset_m = std::clamp(value, -DeviceSettings::kMaxCameraOffsetM, DeviceSettings::kMaxCameraOffsetM);
        if (parse_json_float_value(text, "camera_height_m", &value) && std::isfinite(value))
            s.camera_height_m = std::clamp(value, 0.8f, 2.0f);
        *out = s;
        return true;
    }

private:
    std::string path_;
    struct timespec mtime_ = {};
    bool read_once_ = false;
    uint64_t next_check_ns_ = 0;
};

#endif
