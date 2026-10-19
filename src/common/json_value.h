#pragma once

// 작은 JSON 읽기. 런타임이 읽는 JSON 파일은 모두 이것으로 푼다(파라미터·학습 파일은 utils_json의 헬퍼가 그 위에서
// 키를 읽는다). 쓰기는 하지 않는다.
// - 객체, 배열, 숫자(double), 문자열(\uXXXX는 UTF-8로), true/false/null.
// - 깊이 64까지. 오류는 위치(바이트)와 함께 돌려준다.

#include <string>
#include <utility>
#include <vector>

class JsonValue {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    // text 전체가 JSON 값 하나여야 한다(앞뒤 공백 허용). 실패하면 false와 error.
    static bool parse(const std::string &text, JsonValue *out, std::string *error);

    Type type() const { return type_; }
    bool is_null() const { return type_ == Type::Null; }
    bool is_bool() const { return type_ == Type::Bool; }
    bool is_number() const { return type_ == Type::Number; }
    bool is_string() const { return type_ == Type::String; }
    bool is_array() const { return type_ == Type::Array; }
    bool is_object() const { return type_ == Type::Object; }

    bool boolean() const { return bool_; }
    double number() const { return number_; }
    const std::string &string() const { return string_; }
    const std::vector<JsonValue> &array() const { return array_; }
    const std::vector<std::pair<std::string, JsonValue>> &object() const { return object_; }

    // 객체의 멤버(같은 키가 여럿이면 처음 것). 객체가 아니거나 없으면 nullptr.
    const JsonValue *find(const std::string &key) const;

private:
    friend class JsonParser;
    Type type_ = Type::Null;
    bool bool_ = false;
    double number_ = 0.0;
    std::string string_;
    std::vector<JsonValue> array_;
    std::vector<std::pair<std::string, JsonValue>> object_;
};
