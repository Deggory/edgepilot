// 차선선 플러그인 불러오기(model/lane_plugin): 가짜 플러그인으로 열기, ABI 검사, 결과 검사, 자를 영역, 연속 크래시
// 가드를 본다.
#include "model/lane_plugin.h"

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {

// LanePlugin이 연 것과 같은 라이브러리를 열어(참조 수만 는다) 가짜 플러그인의 모드를 바꾼다.
void set_mode(int mode) {
    void *handle = dlopen(FAKE_LANE_PLUGIN_PATH, RTLD_NOW | RTLD_LOCAL);
    ASSERT_NE(handle, nullptr) << dlerror();
    auto set = reinterpret_cast<void (*)(int)>(dlsym(handle, "fake_lane_plugin_set_mode"));
    ASSERT_NE(set, nullptr);
    set(mode);
    dlclose(handle);
}

std::string temp_path(const char *name) {
    return std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") + "/" + name + "_" +
           std::to_string(getpid());
}

TEST(LanePlugin, OpensAndDecodes) {
    std::string why;
    const auto plugin = LanePlugin::open(FAKE_LANE_PLUGIN_PATH, &why);
    ASSERT_NE(plugin, nullptr) << why;
    set_mode(0);
    EXPECT_STREQ(plugin->name(), "fake lane plugin");
    const std::string path = FAKE_LANE_PLUGIN_PATH;
    EXPECT_EQ(plugin->model_path(), path.substr(0, path.find_last_of('/')) + "/fake.axmodel")
        << "모델 파일은 플러그인과 같은 디렉터리에서 찾는다";

    const EpLaneGeometry g = plugin->geometry(1280, 720, 640, 320);
    EXPECT_EQ(g.crop_x, 0);
    EXPECT_EQ(g.crop_y, 288);
    EXPECT_EQ(g.crop_width, 1280);
    EXPECT_EQ(g.crop_height, 288);
    EXPECT_EQ(g.net_width, 640);
    EXPECT_EQ(g.net_height, 320);

    const float line_value = 600.0f;
    const EpTensor outputs[] = {{"other", nullptr, 0}, {"line", &line_value, 1}};
    EpLaneResult result;
    int32_t code = -1;
    ASSERT_TRUE(plugin->decode(outputs, 2, g, &result, &code));
    EXPECT_EQ(code, 0);
    ASSERT_EQ(result.line_count, 1u);
    EXPECT_FLOAT_EQ(result.lines[0].points[0].x, 600.0f) << "텐서가 이름과 함께 넘어간다";
    EXPECT_FLOAT_EQ(result.lines[0].points[0].y, 576.0f);
    EXPECT_EQ(result.lines[0].color, EP_LANE_YELLOW);
    ASSERT_EQ(result.boundary_count, 1u);
    EXPECT_EQ(result.boundaries[0].type, EP_BOUNDARY_GUARDRAIL);
}

TEST(LanePlugin, RejectsBrokenResults) {
    std::string why;
    const auto plugin = LanePlugin::open(FAKE_LANE_PLUGIN_PATH, &why);
    ASSERT_NE(plugin, nullptr) << why;
    const EpLaneGeometry g = plugin->geometry(1280, 720, 640, 320);
    const float line_value = 600.0f;
    const EpTensor outputs[] = {{"line", &line_value, 1}};
    EpLaneResult result;
    int32_t code = 0;

    set_mode(1);
    EXPECT_FALSE(plugin->decode(outputs, 1, g, &result, &code));
    EXPECT_EQ(code, 7) << "플러그인의 실패 코드를 그대로 넘긴다";
    EXPECT_EQ(result.line_count, 0u);

    set_mode(2);
    EXPECT_FALSE(plugin->decode(outputs, 1, g, &result, &code));
    EXPECT_EQ(code, LanePlugin::kInvalidResult) << "열거값 밖의 색은 결과 전체를 버린다";
    EXPECT_EQ(result.line_count, 0u);

    set_mode(3);
    EXPECT_FALSE(plugin->decode(outputs, 1, g, &result, &code));
    EXPECT_EQ(code, LanePlugin::kInvalidResult) << "유한하지 않은 점은 결과 전체를 버린다";

    set_mode(4);
    ASSERT_TRUE(plugin->decode(outputs, 1, g, &result, &code));
    EXPECT_FLOAT_EQ(result.lines[0].confidence, 1.0f) << "신뢰도는 0..1로 자른다";

    set_mode(5);
    EXPECT_FALSE(plugin->decode(outputs, 1, g, &result, &code))
        << "던진 예외는 해석 실패로 받는다(modeld가 내려가지 않게)";
    EXPECT_EQ(code, LanePlugin::kThrew);
    EXPECT_EQ(result.line_count, 0u);
    set_mode(0);
}

TEST(LanePlugin, RefusesOtherAbiAndMissingFiles) {
    std::string why;
    EXPECT_EQ(LanePlugin::open(FAKE_LANE_PLUGIN_BAD_ABI_PATH, &why), nullptr);
    EXPECT_NE(why.find("ABI 99"), std::string::npos) << why;
    why.clear();
    EXPECT_EQ(LanePlugin::open("/nonexistent/liblane_plugin.so", &why), nullptr);
    EXPECT_FALSE(why.empty());
}

TEST(LanePlugin, CropIsEvenAndInsideTheFrame) {
    std::string why;
    const auto plugin = LanePlugin::open(FAKE_LANE_PLUGIN_PATH, &why);
    ASSERT_NE(plugin, nullptr) << why;
    // roi [0, 0.4, 1, 0.8] on 1001x501: 홀수 프레임에서도 NV12 짝수 경계 안에 둔다
    const EpLaneGeometry g = plugin->geometry(1001, 501, 640, 320);
    EXPECT_EQ(g.crop_x, 0);
    EXPECT_EQ(g.crop_y, 200);
    EXPECT_EQ(g.crop_width, 1000);
    EXPECT_EQ(g.crop_height, 200);
}

TEST(LanePluginGuard, StopsAfterStartsThatNeverFinished) {
    const std::string path = temp_path("gtest_lane_plugin_guard");
    std::remove(path.c_str());
    {
        LanePluginGuard guard(path, 2);
        EXPECT_TRUE(guard.begin());
        EXPECT_EQ(guard.starts(), 1);
    }
    EXPECT_EQ(LanePluginGuard(path).starts(), 0) << "정상 종료는 기록을 지운다";

    // 두 번 죽은 것처럼: 기록이 지워지지 않고 남았다
    std::ofstream(path) << "2\n";
    {
        LanePluginGuard guard(path, 2);
        EXPECT_FALSE(guard.begin()) << "지우지 못한 시작이 두 번이면 열지 않는다";
    }
    EXPECT_EQ(LanePluginGuard(path).starts(), 2) << "열지 않았으면 기록을 그대로 둔다";

    std::ofstream(path) << "1\n";
    {
        LanePluginGuard guard(path, 2);
        EXPECT_TRUE(guard.begin());
        EXPECT_EQ(guard.starts(), 2);
        guard.healthy();
        EXPECT_EQ(guard.starts(), 0) << "충분히 돌면 지운다";
    }

    // 예외로 풀려 내려가는 소멸은 정상 종료가 아니다: 기록을 남긴다
    std::remove(path.c_str());
    try {
        LanePluginGuard guard(path, 2);
        ASSERT_TRUE(guard.begin());
        throw std::runtime_error("modeld unwinds");
    } catch (const std::runtime_error &) {
    }
    EXPECT_EQ(LanePluginGuard(path).starts(), 1);
    std::remove(path.c_str());
}

}  // namespace
