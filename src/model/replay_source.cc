#include "model/replay_source.h"

#include <cstring>
#include <stdexcept>

ReplayNv12Source::ReplayNv12Source(const std::string &path) : path_(path) {
    file_.open(path_, std::ios::binary);
    if (!file_) throw std::runtime_error("open replay file failed: " + path_);

    char magic[8]{};
    read_exact(magic, sizeof(magic), "magic");
    if (std::memcmp(magic, "SCNV12R1", sizeof(magic)) != 0) throw std::runtime_error("bad replay magic: " + path_);

    read_exact(reinterpret_cast<char *>(&width_), sizeof(width_), "width");
    read_exact(reinterpret_cast<char *>(&height_), sizeof(height_), "height");
    read_exact(reinterpret_cast<char *>(&frame_count_), sizeof(frame_count_), "frame count");

    if (width_ == 0 || height_ == 0 || (width_ & 1) || (height_ & 1))
        throw std::runtime_error("bad replay dimensions: " + path_);
    frame_bytes_ = static_cast<size_t>(width_) * height_ * 3 / 2;
}

void ReplayNv12Source::read_exact(char *dst, size_t size, const char *label) {
    file_.read(dst, static_cast<std::streamsize>(size));
    if (file_.gcount() != static_cast<std::streamsize>(size))
        throw std::runtime_error(std::string("short replay header read: ") + label + ": " + path_);
}

bool ReplayNv12Source::read(uint8_t *dst) {
    if (frames_read_ >= frame_count_) return false;
    file_.read(reinterpret_cast<char *>(dst), static_cast<std::streamsize>(frame_bytes_));
    if (file_.gcount() != static_cast<std::streamsize>(frame_bytes_))
        throw std::runtime_error("short replay frame read: " + path_);
    ++frames_read_;
    return true;
}
