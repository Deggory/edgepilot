// 작은 JSON 읽기(JsonValue): 모든 값 종류, 문자열 이스케이프, 중첩, 그리고 잘못된 문서를 위치와 함께 거부하는지.
#include "common/json_value.h"

#include <gtest/gtest.h>
#include <string>

namespace {

TEST(JsonValue, ParsesEveryKind) {
    JsonValue v;
    std::string error;
    ASSERT_TRUE(
        JsonValue::parse(R"( {"a": 1.5e2, "b": [true, false, null, -0.25], "c": {"d": "x"}, "e": []} )", &v, &error))
        << error;
    ASSERT_TRUE(v.is_object());
    EXPECT_DOUBLE_EQ(v.find("a")->number(), 150.0);
    const auto &b = v.find("b")->array();
    ASSERT_EQ(b.size(), 4U);
    EXPECT_TRUE(b[0].is_bool() && b[0].boolean());
    EXPECT_TRUE(b[1].is_bool() && !b[1].boolean());
    EXPECT_TRUE(b[2].is_null());
    EXPECT_DOUBLE_EQ(b[3].number(), -0.25);
    EXPECT_EQ(v.find("c")->find("d")->string(), "x");
    EXPECT_TRUE(v.find("e")->is_array() && v.find("e")->array().empty());
    EXPECT_EQ(v.find("missing"), nullptr);
    EXPECT_EQ(b[0].find("a"), nullptr) << "객체가 아니면 find는 nullptr";
}

TEST(JsonValue, DecodesStringEscapes) {
    JsonValue v;
    std::string error;
    ASSERT_TRUE(JsonValue::parse(R"("q\" s\\ n\n t\t u\u00e9 k\ud55c sp\ud83d\ude00")", &v, &error)) << error;
    EXPECT_EQ(v.string(), "q\" s\\ n\n t\t u\xC3\xA9 k\xED\x95\x9C sp\xF0\x9F\x98\x80");
}

TEST(JsonValue, RejectsMalformedDocuments) {
    // 마지막 둘은 짝 없는 대리쌍(낮은 쪽 혼자, 높은 쪽 뒤에 \u가 없음)
    const char *bad[] = {"",        "{",         "[1,]",       "{\"a\" 1}",   "{\"a\":}",    "01",
                         "1.",      "-",         "tru",        "\"abc",       "[1] x",       "{\"a\":1,}",
                         "\"\\x\"", "\"\\u12\"", "[\"a\nb\"]", "\"\\udc00\"", "\"\\ud800x\""};
    for (const char *text : bad) {
        JsonValue v;
        std::string error;
        EXPECT_FALSE(JsonValue::parse(text, &v, &error)) << text;
        EXPECT_NE(error.find("at byte"), std::string::npos) << text << ": " << error;
    }
}

TEST(JsonValue, LimitsNesting) {
    std::string deep(65, '[');
    deep += std::string(65, ']');
    JsonValue v;
    std::string error;
    EXPECT_FALSE(JsonValue::parse(deep, &v, &error));
    EXPECT_NE(error.find("nesting"), std::string::npos) << error;
    std::string ok(60, '[');
    ok += std::string(60, ']');
    EXPECT_TRUE(JsonValue::parse(ok, &v, &error)) << error;
}

}  // namespace
