/* MaixCDK runtime pieces referenced by ax_middleware.hpp inline code (log, err,
 * board config lookup). libmaixcam_lib exports the middleware classes but not these,
 * and tinyalsa is not on the board image, so its audio paths abort if ever reached. */
#include "maix_basic.hpp"
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
extern "C" {
#include <stddef.h>
}
namespace maix {
namespace log {
static void vlog(const char *tag, const char *fmt, va_list ap) { std::fprintf(stderr, "[maix %s] ", tag); std::vfprintf(stderr, fmt, ap); std::fputc('\n', stderr); }
void info(const char *fmt, ...) { va_list ap; va_start(ap, fmt); vlog("I", fmt, ap); va_end(ap); }
void warn(const char *fmt, ...) { va_list ap; va_start(ap, fmt); vlog("W", fmt, ap); va_end(ap); }
void error(const char *fmt, ...) { va_list ap; va_start(ap, fmt); vlog("E", fmt, ap); va_end(ap); }
void debug(const char *, ...) {}
}
namespace err {
void check_null_raise(void *ptr, const std::string &msg) { if (!ptr) throw Exception(msg, ERR_NO_MEM); }
Exception::Exception(const std::string &msg, err::Err code) : _msg(msg), _code(code) {}
Exception::Exception(const err::Err code, const std::string &msg) : _msg(msg), _code(code) {}
const char *Exception::what() const throw() { return _msg.c_str(); }
err::Err Exception::code() const { return _code; }
}
}
namespace maix { namespace app {
/* MaixCDK reads board settings from /boot/configs as maix_<item>_<key>=value. */
std::string get_sys_config_kv(const std::string &item, const std::string &key, const std::string &value, bool)
{
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
} }
/* tinyalsa is not on the board image; audio paths in the header are never used here. */
extern "C" {
#define NO_AUDIO(name) void *name() { std::fprintf(stderr, #name ": audio not available\n"); std::abort(); }
NO_AUDIO(pcm_open) NO_AUDIO(pcm_writei) NO_AUDIO(pcm_wait) NO_AUDIO(pcm_state) NO_AUDIO(pcm_readi)
NO_AUDIO(pcm_is_ready) NO_AUDIO(pcm_get_error) NO_AUDIO(pcm_close) NO_AUDIO(pcm_prepare) NO_AUDIO(pcm_start)
NO_AUDIO(pcm_get_buffer_size) NO_AUDIO(pcm_frames_to_bytes) NO_AUDIO(pcm_bytes_to_frames) NO_AUDIO(pcm_stop)
NO_AUDIO(pcm_get_delay) NO_AUDIO(mixer_open) NO_AUDIO(mixer_close) NO_AUDIO(mixer_get_ctl_by_name)
NO_AUDIO(mixer_ctl_set_value) NO_AUDIO(mixer_ctl_set_array) NO_AUDIO(mixer_ctl_get_value) NO_AUDIO(mixer_ctl_get_type)
}
