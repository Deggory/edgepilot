// 공용 도우미: write_file_atomic(임시 파일을 남기지 않고 바꿔 넣는다), FileWatch(주기마다 보고 바뀐 확인에만 참,
// 지운 것도 바뀐 것), StatsWindow(창이 끝날 때만 닫고 그 창의 초당 개수), 단순 JSON 배열 읽기(고정 길이는 개수가
// 정확해야 한다).
#include "common/utils_file.h"
#include "common/utils_json.h"
#include "common/utils_time.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

TEST(WriteFileAtomic, ReplacesTheFileAndLeavesNoTemporary) {
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / ("edgepilot_atomic_" + std::to_string(getpid()));
    std::filesystem::create_directories(directory);
    const std::string path = (directory / "state.json").string();
    ASSERT_TRUE(write_file_atomic(path, "old"));
    ASSERT_TRUE(write_file_atomic(path, "new content"));
    EXPECT_EQ(read_text_file(path), "new content");
    size_t files = 0;
    for (const auto &entry : std::filesystem::directory_iterator(directory)) {
        ++files;
        EXPECT_EQ(entry.path().filename(), "state.json") << "임시 파일이 남았다";
    }
    EXPECT_EQ(files, 1u);
    EXPECT_FALSE(write_file_atomic((directory / "missing" / "x.json").string(), "x")) << "없는 폴더";
    std::filesystem::remove_all(directory);
}

TEST(FileWatch, ReportsEachChangeOnceAtItsInterval) {
    const std::string path = std::filesystem::temp_directory_path() / ("edgepilot_watch_" + std::to_string(getpid()));
    std::filesystem::remove(path);
    FileWatch watch(path, 1'000'000'000ULL);
    EXPECT_FALSE(watch.changed(0)) << "없는 파일은 바뀐 것이 아니다";
    ASSERT_TRUE(write_file_atomic(path, "a"));
    EXPECT_FALSE(watch.changed(500'000'000ULL)) << "주기 전에는 보지 않는다";
    EXPECT_TRUE(watch.changed(1'000'000'000ULL)) << "생긴 파일";
    EXPECT_FALSE(watch.changed(2'000'000'000ULL)) << "그대로";
    ASSERT_TRUE(write_file_atomic(path, "bb"));
    EXPECT_TRUE(watch.changed(3'000'000'000ULL)) << "rename으로 바꾼 파일";
    std::filesystem::remove(path);
    EXPECT_TRUE(watch.changed(4'000'000'000ULL)) << "지운 것도 바뀐 것이다";
    EXPECT_FALSE(watch.changed(5'000'000'000ULL));

    ASSERT_TRUE(write_file_atomic(path, "c"));
    FileWatch seen(path, 1'000'000'000ULL);
    seen.mark_seen(10'000'000'000ULL);
    EXPECT_FALSE(seen.changed(11'000'000'000ULL)) << "시작 때 읽은 파일은 다시 읽지 않는다";
    std::filesystem::remove(path);
}

TEST(StatsWindow, ClosesOnlyWhenDueWithThatWindowsRate) {
    StatsWindow window(0);
    for (int i = 0; i < 20; ++i) window.count();
    EXPECT_FALSE(window.close_if_due(999'999'999ULL));
    ASSERT_TRUE(window.close_if_due(1'250'000'000ULL));
    EXPECT_DOUBLE_EQ(window.seconds(), 1.25);
    EXPECT_DOUBLE_EQ(window.rate(), 16.0);
    window.count();
    EXPECT_FALSE(window.close_if_due(2'000'000'000ULL)) << "새 창은 닫은 시각에서 시작한다";
    ASSERT_TRUE(window.close_if_due(2'250'000'000ULL));
    EXPECT_DOUBLE_EQ(window.rate(), 1.0);
    EXPECT_DOUBLE_EQ(window.total_rate(45, 2'250'000'000ULL), 20.0);
}

TEST(JsonFloatArray, ReadsExactlyTheCount) {
    const auto parse = [](const char *text) {
        JsonValue root;
        EXPECT_TRUE(parse_json_object(text, &root));
        return root;
    };
    std::array<float, 3> rpy{};
    ASSERT_TRUE(json_float_array(parse(R"({"rpy_rad": [0.1, -0.2, 3e-3]})"), "rpy_rad", &rpy));
    EXPECT_FLOAT_EQ(rpy[1], -0.2f);
    EXPECT_FALSE(json_float_array(parse(R"({"other": [1, 2, 3]})"), "rpy_rad", &rpy)) << "키가 없다";
    EXPECT_THROW(json_float_array(parse(R"({"rpy_rad": [1, 2]})"), "rpy_rad", &rpy), std::runtime_error)
        << "개수가 다르다";
    EXPECT_THROW(json_float_array(parse(R"({"rpy_rad": [1, 2, 3, 4]})"), "rpy_rad", &rpy), std::runtime_error);
    EXPECT_THROW(json_float_array(parse(R"({"rpy_rad": [1, "2", 3]})"), "rpy_rad", &rpy), std::runtime_error)
        << "수가 아니다";
    std::vector<float> list;
    ASSERT_TRUE(json_float_list(parse(R"({"yaw_rad" : [ ]})"), "yaw_rad", 4, &list));
    EXPECT_TRUE(list.empty());
    float value = 0.0f;
    EXPECT_THROW(json_float(parse(R"({"x": 1e39})"), "x", &value), std::runtime_error) << "float로 유한하지 않다";
    JsonValue root;
    EXPECT_FALSE(parse_json_object("[1, 2]", &root)) << "객체가 아니다";
}

}  // namespace
