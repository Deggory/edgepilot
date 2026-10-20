#ifndef REPLAY_SOURCE_H
#define REPLAY_SOURCE_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>

// SCNV12R1 재생 파일의 NV12 프레임 소스. modeld의 헤드리스 재생용.
class ReplayNv12Source {
public:
    explicit ReplayNv12Source(const std::string &path);

    // 다음 프레임(width*height*3/2 바이트)을 dst(GDC 소스 CMM)에 바로 읽는다. 끝이면 false.
    bool read(uint8_t *dst);
    unsigned frame_count() const { return frame_count_; }
    unsigned width() const { return width_; }
    unsigned height() const { return height_; }

private:
    void read_exact(char *dst, size_t size, const char *label);

    std::string path_;
    std::ifstream file_;
    unsigned width_ = 0;
    unsigned height_ = 0;
    unsigned frame_count_ = 0;
    size_t frame_bytes_ = 0;
    unsigned frames_read_ = 0;
};

#endif
