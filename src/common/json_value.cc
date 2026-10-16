#include "common/json_value.h"

#include <cstdlib>
#include <cstring>

namespace {

constexpr int kMaxDepth = 64;

void append_utf8(unsigned code, std::string *out) {
    if (code < 0x80) {
        out->push_back(static_cast<char>(code));
    } else if (code < 0x800) {
        out->push_back(static_cast<char>(0xC0 | (code >> 6)));
        out->push_back(static_cast<char>(0x80 | (code & 0x3F)));
    } else if (code < 0x10000) {
        out->push_back(static_cast<char>(0xE0 | (code >> 12)));
        out->push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
        out->push_back(static_cast<char>(0x80 | (code & 0x3F)));
    } else {
        out->push_back(static_cast<char>(0xF0 | (code >> 18)));
        out->push_back(static_cast<char>(0x80 | ((code >> 12) & 0x3F)));
        out->push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
        out->push_back(static_cast<char>(0x80 | (code & 0x3F)));
    }
}

}  // namespace

// 재귀 하강 파서. JsonValue의 내부를 채우려고 friend다.
class JsonParser {
public:
    explicit JsonParser(const std::string &text) : s_(text) {}

    bool parse_document(JsonValue *out, std::string *error) {
        skip_ws();
        if (!value(out, 0)) return fail(error);
        skip_ws();
        if (pos_ != s_.size()) {
            message_ = "trailing characters";
            return fail(error);
        }
        return true;
    }

private:
    bool fail(std::string *error) const {
        if (error) *error = message_ + " at byte " + std::to_string(pos_);
        return false;
    }

    void skip_ws() {
        while (pos_ < s_.size() && (s_[pos_] == ' ' || s_[pos_] == '\n' || s_[pos_] == '\r' || s_[pos_] == '\t'))
            ++pos_;
    }

    bool literal(const char *word) {
        const size_t n = std::strlen(word);
        if (s_.compare(pos_, n, word) != 0) {
            message_ = "invalid literal";
            return false;
        }
        pos_ += n;
        return true;
    }

    bool value(JsonValue *out, int depth) {
        if (depth >= kMaxDepth) {  // 깊이 0부터라 64단계까지
            message_ = "nesting too deep";
            return false;
        }
        if (pos_ >= s_.size()) {
            message_ = "unexpected end";
            return false;
        }
        const char c = s_[pos_];
        if (c == '{') return object(out, depth);
        if (c == '[') return array(out, depth);
        if (c == '"') {
            out->type_ = JsonValue::Type::String;
            return string(&out->string_);
        }
        if (c == 't' || c == 'f') {
            out->type_ = JsonValue::Type::Bool;
            out->bool_ = c == 't';
            return literal(c == 't' ? "true" : "false");
        }
        if (c == 'n') {
            out->type_ = JsonValue::Type::Null;
            return literal("null");
        }
        return number(out);
    }

    bool number(JsonValue *out) {
        // JSON 문법: -? (0 | [1-9][0-9]*) (.[0-9]+)? ([eE][+-]?[0-9]+)?
        const size_t start = pos_;
        if (pos_ < s_.size() && s_[pos_] == '-') ++pos_;
        const auto digits = [this] {
            const size_t from = pos_;
            while (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9') ++pos_;
            return pos_ > from;
        };
        if (pos_ < s_.size() && s_[pos_] == '0') {
            ++pos_;
        } else if (!digits()) {
            message_ = "invalid value";
            return false;
        }
        if (pos_ < s_.size() && s_[pos_] == '.') {
            ++pos_;
            if (!digits()) {
                message_ = "invalid number";
                return false;
            }
        }
        if (pos_ < s_.size() && (s_[pos_] == 'e' || s_[pos_] == 'E')) {
            ++pos_;
            if (pos_ < s_.size() && (s_[pos_] == '+' || s_[pos_] == '-')) ++pos_;
            if (!digits()) {
                message_ = "invalid number";
                return false;
            }
        }
        out->type_ = JsonValue::Type::Number;
        // 문법은 위에서 확인했으니 strtod는 같은 곳에서 멈춘다(숫자마다 문자열을 새로 만들지 않는다)
        out->number_ = std::strtod(s_.c_str() + start, nullptr);
        return true;
    }

    bool hex4(unsigned *code) {
        if (pos_ + 4 > s_.size()) return false;
        unsigned v = 0;
        for (int i = 0; i < 4; ++i) {
            const char h = s_[pos_ + i];
            v <<= 4;
            if (h >= '0' && h <= '9')
                v |= static_cast<unsigned>(h - '0');
            else if (h >= 'a' && h <= 'f')
                v |= static_cast<unsigned>(h - 'a' + 10);
            else if (h >= 'A' && h <= 'F')
                v |= static_cast<unsigned>(h - 'A' + 10);
            else
                return false;
        }
        pos_ += 4;
        *code = v;
        return true;
    }

    bool string(std::string *out) {
        ++pos_;  // 여는 따옴표
        out->clear();
        while (pos_ < s_.size()) {
            const char c = s_[pos_++];
            if (c == '"') return true;
            if (static_cast<unsigned char>(c) < 0x20) {
                message_ = "control character in string";
                return false;
            }
            if (c != '\\') {
                out->push_back(c);
                continue;
            }
            if (pos_ >= s_.size()) break;
            const char e = s_[pos_++];
            switch (e) {
                case '"':
                    out->push_back('"');
                    break;
                case '\\':
                    out->push_back('\\');
                    break;
                case '/':
                    out->push_back('/');
                    break;
                case 'b':
                    out->push_back('\b');
                    break;
                case 'f':
                    out->push_back('\f');
                    break;
                case 'n':
                    out->push_back('\n');
                    break;
                case 'r':
                    out->push_back('\r');
                    break;
                case 't':
                    out->push_back('\t');
                    break;
                case 'u': {
                    unsigned code = 0;
                    if (!hex4(&code)) {
                        message_ = "invalid \\u escape";
                        return false;
                    }
                    // 대리쌍: 높은 쪽 뒤에는 \u 낮은 쪽이 와야 하고, 낮은 쪽 혼자는 안 된다(UTF-8로 낼 수 없다)
                    if (code >= 0xDC00 && code < 0xE000) {
                        message_ = "invalid surrogate pair";
                        return false;
                    }
                    if (code >= 0xD800 && code < 0xDC00) {
                        unsigned low = 0;
                        const bool escaped = s_.compare(pos_, 2, "\\u") == 0;
                        if (escaped) pos_ += 2;
                        if (!escaped || !hex4(&low) || low < 0xDC00 || low >= 0xE000) {
                            message_ = "invalid surrogate pair";
                            return false;
                        }
                        code = 0x10000 + ((code - 0xD800) << 10) + (low - 0xDC00);
                    }
                    append_utf8(code, out);
                    break;
                }
                default:
                    message_ = "invalid escape";
                    return false;
            }
        }
        message_ = "unterminated string";
        return false;
    }

    bool array(JsonValue *out, int depth) {
        ++pos_;
        out->type_ = JsonValue::Type::Array;
        skip_ws();
        if (pos_ < s_.size() && s_[pos_] == ']') {
            ++pos_;
            return true;
        }
        while (true) {
            skip_ws();
            out->array_.emplace_back();
            if (!value(&out->array_.back(), depth + 1)) return false;
            skip_ws();
            if (pos_ < s_.size() && s_[pos_] == ',') {
                ++pos_;
                continue;
            }
            if (pos_ < s_.size() && s_[pos_] == ']') {
                ++pos_;
                return true;
            }
            message_ = "expected , or ]";
            return false;
        }
    }

    bool object(JsonValue *out, int depth) {
        ++pos_;
        out->type_ = JsonValue::Type::Object;
        skip_ws();
        if (pos_ < s_.size() && s_[pos_] == '}') {
            ++pos_;
            return true;
        }
        while (true) {
            skip_ws();
            if (pos_ >= s_.size() || s_[pos_] != '"') {
                message_ = "expected key";
                return false;
            }
            std::string key;
            if (!string(&key)) return false;
            skip_ws();
            if (pos_ >= s_.size() || s_[pos_] != ':') {
                message_ = "expected :";
                return false;
            }
            ++pos_;
            skip_ws();
            out->object_.emplace_back(std::move(key), JsonValue{});
            if (!value(&out->object_.back().second, depth + 1)) return false;
            skip_ws();
            if (pos_ < s_.size() && s_[pos_] == ',') {
                ++pos_;
                continue;
            }
            if (pos_ < s_.size() && s_[pos_] == '}') {
                ++pos_;
                return true;
            }
            message_ = "expected , or }";
            return false;
        }
    }

    const std::string &s_;
    size_t pos_ = 0;
    std::string message_;
};

bool JsonValue::parse(const std::string &text, JsonValue *out, std::string *error) {
    JsonValue parsed;
    JsonParser parser(text);
    if (!parser.parse_document(&parsed, error)) return false;
    *out = std::move(parsed);
    return true;
}

const JsonValue *JsonValue::find(const std::string &key) const {
    if (type_ != Type::Object) return nullptr;
    for (const auto &member : object_) {
        if (member.first == key) return &member.second;
    }
    return nullptr;
}
