// 공용 도우미: write_file_atomic(임시 파일을 남기지 않고 바꿔 넣는다), 단순 JSON 배열 읽기(고정 길이는 개수가
// 정확해야 한다).
#include "common/utils_file.h"
#include "common/utils_json.h"

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
