// 학습 토크 모델 고르기(docs/lateral-nn.md): 고르기 파일 해석(이름 규칙, 잘못된 파일, 환경 변수 우선)과
// LateralNnSelector가 고르기·모델 파일이 바뀌면 백그라운드에서 읽어 갈아 끼우고, 못 읽으면 지금 모델을 두고,
// 웹 콘솔이 읽는 상태 파일을 쓰는지.
#include "common/lateral_nn_path.h"
#include "common/utils_file.h"
#include "controls/lateral_nn_select.h"

#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;

struct ScratchDir {
    ScratchDir() {
        char path[] = "/tmp/gtest_lateral_nn_select_XXXXXX";
        root = mkdtemp(path);
    }
    ~ScratchDir() { std::system(("rm -rf '" + root + "'").c_str()); }
    std::string root;
};

void write(const std::string &path, const std::string &text) { std::ofstream(path) << text; }

// 읽기가 끝날 때까지(최대 5초) 1초 넘게 간격을 두고 poll한다. 새 모델을 받으면 true.
bool poll_until_done(LateralNnSelector &selector, Clock::time_point *now,
                     std::shared_ptr<const LateralNnModel> *model) {
    *now += std::chrono::seconds(2);
    for (int i = 0; i < 5000; ++i) {
        if (selector.poll(*now, model)) return true;
        if (i > 0 && !selector.loading()) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

}  // namespace

TEST(LateralNnPath, NameRule) {
    EXPECT_TRUE(valid_lateral_nn_name("k7_hev_v3"));
    EXPECT_TRUE(valid_lateral_nn_name("Car-2.1"));
    EXPECT_FALSE(valid_lateral_nn_name(""));
    EXPECT_FALSE(valid_lateral_nn_name(".hidden"));
    EXPECT_FALSE(valid_lateral_nn_name("../steering"));
    EXPECT_FALSE(valid_lateral_nn_name("a/b"));
    EXPECT_FALSE(valid_lateral_nn_name(std::string(65, 'a')));
}

TEST(LateralNnPath, ChoiceFromTheSelectionFile) {
    const LateralNnChoice good = lateral_nn_choice_from(R"({"model": "k7_hev_v3"})", "models/lateral_nn");
    EXPECT_EQ(good.name, "k7_hev_v3");
    EXPECT_EQ(good.path, "models/lateral_nn/k7_hev_v3.json");
    EXPECT_TRUE(good.error.empty());
    for (const char *bad : {"", "{", R"({"other": 1})", R"({"model": 3})", R"({"model": "../x"})"}) {
        const LateralNnChoice choice = lateral_nn_choice_from(bad, "models/lateral_nn");
        EXPECT_TRUE(choice.path.empty()) << bad;
        EXPECT_FALSE(choice.error.empty()) << bad;
    }
}

TEST(LateralNnPath, EnvironmentOverridesTheSelection) {
    ScratchDir dir;
    write(dir.root + "/lateral_nn.json", R"({"model": "k7_hev_v2"})");
    EXPECT_EQ(lateral_nn_choice(dir.root).path, "models/lateral_nn/k7_hev_v2.json");
    setenv("EDGEPILOT_LATERAL_NN_MODEL", "/elsewhere/model.json", 1);
    EXPECT_EQ(lateral_nn_choice(dir.root).path, "/elsewhere/model.json");
    unsetenv("EDGEPILOT_LATERAL_NN_MODEL");
}

TEST(LateralNnSelector, SwitchesKeepsAndReports) {
    ScratchDir dir;
    const std::string status = dir.root + "/status.json";
    LateralNnChoice choice{"k7_hev_v3", "models/lateral_nn/k7_hev_v3.json", ""};
    LateralNnSelector selector(status, [&] { return choice; });

    const auto first = selector.load_now();
    ASSERT_TRUE(first);
    EXPECT_EQ(selector.loaded_name(), "k7_hev_v3");
    EXPECT_NE(read_text_file(status).find(R"("loaded": "k7_hev_v3")"), std::string::npos);

    Clock::time_point now{};
    std::shared_ptr<const LateralNnModel> next;
    EXPECT_FALSE(poll_until_done(selector, &now, &next)) << "그대로면 다시 읽지 않는다";

    choice = {"k7_hev_v2", "models/lateral_nn/k7_hev_v2.json", ""};
    ASSERT_TRUE(poll_until_done(selector, &now, &next)) << "고르기가 바뀌면 읽어서 넘긴다";
    EXPECT_NE(next, first);
    EXPECT_EQ(selector.loaded_name(), "k7_hev_v2");

    // 깨진 모델 파일: 거부하고 지금 모델을 둔다. 실패 이유가 상태에 남는다.
    const std::string broken = dir.root + "/broken.json";
    write(broken, "{\"format\": \"edgepilot-lateral-nn\"}");
    choice = {"broken", broken, ""};
    EXPECT_FALSE(poll_until_done(selector, &now, &next));
    EXPECT_EQ(selector.loaded_name(), "k7_hev_v2");
    EXPECT_FALSE(selector.last_error().empty());
    const std::string text = read_text_file(status);
    EXPECT_NE(text.find(R"("selected": "broken")"), std::string::npos) << text;
    EXPECT_NE(text.find(R"("loaded": "k7_hev_v2")"), std::string::npos) << text;
    EXPECT_FALSE(poll_until_done(selector, &now, &next)) << "같은 깨진 파일은 다시 시도하지 않는다";

    // 그 파일을 고치면(파일 시각이 바뀌면) 다시 읽는다
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    write(broken, read_text_file("models/lateral_nn/k7_hev_v1.json"));
    ASSERT_TRUE(poll_until_done(selector, &now, &next));
    EXPECT_EQ(selector.loaded_name(), "broken");
    EXPECT_TRUE(selector.last_error().empty());
}

TEST(LateralNnSelector, NoSelectionMeansNoModel) {
    ScratchDir dir;
    LateralNnSelector selector(dir.root + "/status.json",
                               [] { return lateral_nn_choice_from("", "models/lateral_nn"); });
    EXPECT_FALSE(selector.load_now());
    EXPECT_FALSE(selector.last_error().empty());
    EXPECT_NE(read_text_file(dir.root + "/status.json").find(R"("loaded": "")"), std::string::npos);
}
