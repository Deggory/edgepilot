#pragma once

/* 파라미터 파일이 바뀌었는지 보는 stat 지문. 웹 서버는 임시 파일 + rename으로 쓰므로 inode도
 * 본다. macOS(호스트 빌드)와 Linux의 수정 시각 필드 이름이 달라 여기서만 나눈다. */

#include <sys/stat.h>

#include <string>

struct FileStamp {
  bool valid = false;
  unsigned long long device = 0;
  unsigned long long inode = 0;
  unsigned long long size = 0;
  long long modified_sec = 0;
  long long modified_nsec = 0;
};

inline bool operator!=(const FileStamp &left, const FileStamp &right) {
  return left.valid != right.valid ||
         left.device != right.device ||
         left.inode != right.inode ||
         left.size != right.size ||
         left.modified_sec != right.modified_sec ||
         left.modified_nsec != right.modified_nsec;
}

inline FileStamp file_stamp(const std::string &path) {
  struct stat info = {};
  FileStamp stamp;
  if (stat(path.c_str(), &info) != 0) return stamp;
  stamp.valid = true;
  stamp.device = static_cast<unsigned long long>(info.st_dev);
  stamp.inode = static_cast<unsigned long long>(info.st_ino);
  stamp.size = static_cast<unsigned long long>(info.st_size);
#if defined(__APPLE__)
  stamp.modified_sec = static_cast<long long>(info.st_mtimespec.tv_sec);
  stamp.modified_nsec = static_cast<long long>(info.st_mtimespec.tv_nsec);
#else
  stamp.modified_sec = static_cast<long long>(info.st_mtim.tv_sec);
  stamp.modified_nsec = static_cast<long long>(info.st_mtim.tv_nsec);
#endif
  return stamp;
}
