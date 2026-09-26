#ifndef REPLAY_SOURCE_H
#define REPLAY_SOURCE_H

#include <cstdint>
#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

struct Nv12Frame {
    unsigned width = 0;
    unsigned height = 0;
    std::vector<uint8_t> data;
};

/* SCNV12R1 리플레이 파일의 NV12 프레임 소스. k230_modeld의 헤드리스 재생용. */
class ReplayNv12Source {
public:
    explicit ReplayNv12Source(const std::string &path);

    bool read(Nv12Frame &frame);
    bool eof() const { return frames_read_ >= frame_count_; }
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
