/* 웹 기기 설정(display.json) 읽기: 범위 클램프, 잘못된 값은 프로세스를 죽이지 않고 이전 값을
 * 유지(modeld가 죽으면 횡제어가 멈춘다), 파일이 바뀔 때만 다시 읽기. */
#include "device_settings.h"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

struct TempFile {
  std::filesystem::path path;
  TempFile() {
    path = std::filesystem::temp_directory_path() /
           ("edgepilot_display_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
            ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".json");
  }
  ~TempFile() { std::filesystem::remove(path); }
  void write(const std::string &text) const {
    const std::filesystem::path tmp = path.string() + ".tmp";
    std::ofstream(tmp) << text;
    std::filesystem::rename(tmp, path);  // 웹 서버처럼 바꿔치기(inode가 바뀐다)
  }
};

TEST(DeviceSettings, ReadsAndClampsLikeTheEditor) {
  TempFile f;
  f.write(R"({"enabled": true, "alert_volume_percent": 70, "camera_offset_m": 0.5, "camera_height_m": 1.30})");
  DeviceSettingsFile file(f.path.string());
  DeviceSettings s;
  ASSERT_TRUE(file.poll(1, &s));
  EXPECT_FLOAT_EQ(s.alert_volume_percent, 70.0f);
  EXPECT_FLOAT_EQ(s.camera_offset_m, 0.35f) << "±0.35 m로 클램프";
  EXPECT_FLOAT_EQ(s.camera_height_m, 1.30f);
  EXPECT_FALSE(file.poll(2'000'000'000ULL, &s)) << "바뀌지 않았으면 다시 읽지 않는다";

  f.write(R"({"brightness_percent": 40})");
  ASSERT_TRUE(file.poll(3'000'000'000ULL, &s));
  EXPECT_TRUE(std::isnan(s.alert_volume_percent)) << "없는 키는 기본값";
  EXPECT_FLOAT_EQ(s.camera_offset_m, 0.0f);
  EXPECT_FLOAT_EQ(s.camera_height_m, kModelHeight);
}

TEST(DeviceSettings, MalformedValueKeepsPreviousSettings) {
  TempFile f;
  f.write(R"({"camera_offset_m": 0.12, "camera_height_m": 1.25})");
  DeviceSettingsFile file(f.path.string());
  DeviceSettings s;
  ASSERT_TRUE(file.poll(1, &s));
  uint64_t now = 1;
  for (const char *bad : {R"({"camera_offset_m": null})", R"({"camera_offset_m": "0.3"})",
                          R"({"camera_offset_m": NaN})", R"({"alert_volume_percent": true})",
                          R"({"camera_height_m":)"}) {
    SCOPED_TRACE(bad);
    f.write(bad);
    now += 1'000'000'000ULL;
    EXPECT_NO_THROW(EXPECT_FALSE(file.poll(now, &s)));
    EXPECT_FLOAT_EQ(s.camera_offset_m, 0.12f) << "이전 값을 유지한다";
    EXPECT_FLOAT_EQ(s.camera_height_m, 1.25f);
  }
  f.write(R"({"camera_offset_m": -0.05})");
  now += 1'000'000'000ULL;
  ASSERT_TRUE(file.poll(now, &s)) << "고쳐지면 다시 읽는다";
  EXPECT_FLOAT_EQ(s.camera_offset_m, -0.05f);
}

TEST(DeviceSettings, MissingFileIsIgnoredUntilItAppears) {
  TempFile f;
  DeviceSettingsFile file(f.path.string());
  DeviceSettings s;
  s.camera_offset_m = 0.2f;
  EXPECT_FALSE(file.poll(1, &s));
  EXPECT_FLOAT_EQ(s.camera_offset_m, 0.2f);
  f.write(R"({"camera_offset_m": 0.1})");
  EXPECT_FALSE(file.poll(500'000'000ULL, &s)) << "1초에 한 번만 본다";
  ASSERT_TRUE(file.poll(1'000'000'001ULL, &s));
  EXPECT_FLOAT_EQ(s.camera_offset_m, 0.1f);
}

}  // namespace
