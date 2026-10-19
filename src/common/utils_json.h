#pragma once

// 파라미터·학습 파일(평평한 JSON 객체)을 JsonValue로 읽는 헬퍼. 값 읽기는 키가 없으면 false이고, 있는데 형식이
// 틀리면(불리언이 아님, float로 유한한 수가 아님, 배열 개수) 예외를 던진다. load_json_param_file이 그 예외를 error로
// 바꾼다.

#include "common/json_value.h"
#include "common/utils_math.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

// text가 JSON 객체 하나면 out에 풀고 true. 아니면 false와 error.
bool parse_json_object(const std::string &text, JsonValue *out, std::string *error = nullptr);

bool json_bool(const JsonValue &object, const std::string &key, bool *out);
bool json_float(const JsonValue &object, const std::string &key, float *out);
// 길이가 정해지지 않은 float 배열(최대 max_count개).
bool json_float_list(const JsonValue &object, const std::string &key, size_t max_count, std::vector<float> *out);

// 고정 길이 float 배열(json_float_list와 같고 개수가 정확히 N이어야 한다).
template <size_t N>
bool json_float_array(const JsonValue &object, const std::string &key, std::array<float, N> *out) {
    std::vector<float> values;
    if (!json_float_list(object, key, N, &values)) return false;
    if (values.size() != N)
        throw std::runtime_error("json key '" + key + "' must contain exactly " + std::to_string(N) + " floats");
    std::copy(values.begin(), values.end(), out->begin());
    return true;
}

// {키, 하한, 상한, 멤버} 표로 파라미터 구조체를 채운다. 없는 키는 기본값을 남기고 범위 밖은 클램프한다. 표가 곧
// 파라미터 명세다.
template <class T>
struct JsonBoolField {
    const char *key;
    bool T::*member;
};
template <class T>
struct JsonIntField {
    const char *key;
    int lo;
    int hi;
    int T::*member;
};
template <class T>
struct JsonFloatField {
    const char *key;
    float lo;
    float hi;
    float T::*member;
};

template <class T, size_t N>
void parse_json_fields(const JsonValue &root, const JsonBoolField<T> (&fields)[N], T *out) {
    for (const auto &f : fields) json_bool(root, f.key, &(out->*f.member));
}
template <class T, size_t N>
void parse_json_fields(const JsonValue &root, const JsonIntField<T> (&fields)[N], T *out) {
    float value = 0.0f;
    for (const auto &f : fields)
        if (json_float(root, f.key, &value)) out->*f.member = clamp_int(value, f.lo, f.hi);
}
template <class T, size_t N>
void parse_json_fields(const JsonValue &root, const JsonFloatField<T> (&fields)[N], T *out) {
    float value = 0.0f;
    for (const auto &f : fields)
        if (json_float(root, f.key, &value)) out->*f.member = clamp_float(value, f.lo, f.hi);
}

// 파라미터 파일 로더 공통 뼈대. 파일을 JSON 객체로 풀어 apply(root)를 부르고, 형식 예외를 error 문자열로 바꾼다.
// 로더는 키 목록만 쓰면 된다.
bool load_json_param_file(const std::string &path, const std::function<void(const JsonValue &)> &apply,
                          std::string *error);
