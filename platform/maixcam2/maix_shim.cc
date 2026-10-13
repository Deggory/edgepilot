// ax_middleware.hpp의 인라인 코드가 부르는 MaixCDK 런타임 조각(log, err, 보드 설정 조회). libmaixcam_lib는
// 미들웨어 클래스는 내보내지만 이것들은 내보내지 않는다. 보드 이미지에 tinyalsa가 없어 오디오 경로는
// 닿으면 중단한다.
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include "maix_basic.hpp"
extern "C" {
#include <stddef.h>
}
namespace maix {
namespace log {
static void vlog(const char *tag, const char *fmt, va_list ap) {
    std::fprintf(stderr, "[maix %s] ", tag);
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
}
void info(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vlog("I", fmt, ap);
    va_end(ap);
}
void warn(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vlog("W", fmt, ap);
    va_end(ap);
}
void error(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vlog("E", fmt, ap);
    va_end(ap);
}
void debug(const char *, ...) {}
}  // namespace log
namespace err {
void check_null_raise(void *ptr, const std::string &msg) {
    if (!ptr) throw Exception(msg, ERR_NO_MEM);
}
Exception::Exception(const std::string &msg, err::Err code) : _msg(msg), _code(code) {}
Exception::Exception(const err::Err code, const std::string &msg) : _msg(msg), _code(code) {}
const char *Exception::what() const throw() { return _msg.c_str(); }
err::Err Exception::code() const { return _code; }
}  // namespace err
}  // namespace maix
namespace maix {
namespace app {
// MaixCDK는 보드 설정을 /boot/configs에서 maix_<item>_<key>=value 형식으로 읽는다.
std::string get_sys_config_kv(const std::string &item, const std::string &key, const std::string &value, bool) {
    FILE *f = std::fopen("/boot/configs", "r");
    if (!f) return value;
    const std::string want = "maix_" + item + "_" + key + "=";
    char line[256];
    std::string result = value;
    while (std::fgets(line, sizeof(line), f)) {
        std::string l(line);
        while (!l.empty() && (l.back() == '\n' || l.back() == '\r')) l.pop_back();
        if (l.compare(0, want.size(), want) == 0) result = l.substr(want.size());
    }
    std::fclose(f);
    return result;
}
}  // namespace app
}  // namespace maix
// 보드 이미지에 tinyalsa가 없다. 헤더의 오디오 경로는 여기서 쓰지 않는다.
extern "C" {
// clang-format off
#define NO_AUDIO(name) void *name() { std::fprintf(stderr, #name ": audio not available\n"); std::abort(); }
NO_AUDIO(pcm_open) NO_AUDIO(pcm_writei) NO_AUDIO(pcm_wait) NO_AUDIO(pcm_state) NO_AUDIO(pcm_readi)
NO_AUDIO(pcm_is_ready) NO_AUDIO(pcm_get_error) NO_AUDIO(pcm_close) NO_AUDIO(pcm_prepare) NO_AUDIO(pcm_start)
NO_AUDIO(pcm_get_buffer_size) NO_AUDIO(pcm_frames_to_bytes) NO_AUDIO(pcm_bytes_to_frames) NO_AUDIO(pcm_stop)
NO_AUDIO(pcm_get_delay) NO_AUDIO(mixer_open) NO_AUDIO(mixer_close) NO_AUDIO(mixer_get_ctl_by_name)
NO_AUDIO(mixer_ctl_set_value) NO_AUDIO(mixer_ctl_set_array) NO_AUDIO(mixer_ctl_get_value) NO_AUDIO(mixer_ctl_get_type)
// clang-format on
}
