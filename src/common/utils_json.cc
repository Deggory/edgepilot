#include "common/utils_json.h"

#include <cmath>
#include <fstream>
#include <iterator>

namespace {

float finite_float(const JsonValue &value, const std::string &key) {
    const float parsed = static_cast<float>(value.number());  // 수가 아니면 0이고 아래에서 거른다
    if (!value.is_number() || !std::isfinite(parsed))
        throw std::runtime_error("json key '" + key + "' is not a finite float");
    return parsed;
}

}  // namespace

bool parse_json_object(const std::string &text, JsonValue *out, std::string *error) {
    std::string parse_error;
    if (!JsonValue::parse(text, out, &parse_error)) {
        if (error) *error = parse_error;
        return false;
    }
    if (!out->is_object()) {
        if (error) *error = "not a JSON object";
        return false;
    }
    return true;
}

bool json_bool(const JsonValue &object, const std::string &key, bool *out) {
    const JsonValue *value = object.find(key);
    if (!value) return false;
    if (!value->is_bool()) throw std::runtime_error("json key '" + key + "' is not a boolean");
    *out = value->boolean();
    return true;
}

bool json_float(const JsonValue &object, const std::string &key, float *out) {
    const JsonValue *value = object.find(key);
    if (!value) return false;
    *out = finite_float(*value, key);
    return true;
}

bool json_float_list(const JsonValue &object, const std::string &key, size_t max_count, std::vector<float> *out) {
    const JsonValue *value = object.find(key);
    if (!value) return false;
    if (!value->is_array()) throw std::runtime_error("json key '" + key + "' is not an array");
    if (value->array().size() > max_count)
        throw std::runtime_error("json key '" + key + "' has more than " + std::to_string(max_count) + " floats");
    out->clear();
    for (const JsonValue &item : value->array()) out->push_back(finite_float(item, key));
    return true;
}

bool load_json_param_file(const std::string &path, const std::function<void(const JsonValue &)> &apply,
                          std::string *error) {
    std::ifstream file(path);
    if (!file.is_open()) {
        if (error) *error = "open failed";
        return false;
    }
    JsonValue root;
    if (!parse_json_object(std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()), &root,
                           error))
        return false;
    try {
        apply(root);
    } catch (const std::exception &exc) {
        if (error) *error = exc.what();
        return false;
    }
    return true;
}
