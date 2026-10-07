/* BackgroundWriter: controlsd와 locationd가 SD 쓰기를 루프 밖으로 넘기는 스레드. 같은 경로에 쌓인 쓰기는
 * 최신 내용만 남고, 지우기도 같은 순서를 따르며, flush는 넘긴 일이 끝날 때까지 기다리고, 없앨 때 남은
 * 일을 마치고, 실패는 세어 둔다. */
#include "common/background_writer.h"

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <string>

namespace {

TEST(BackgroundWriter, LatestContentWinsAndFailuresAreCounted) {
  char root_template[] = "/tmp/gtest_background_writer_XXXXXX";
  const std::string root = mkdtemp(root_template);
  const std::string path = root + "/live_delay.json";
  const std::string removed = root + "/removed.json";
  write_file_atomic(removed, "old");
  {
    BackgroundWriter writer("gtest: write");
    for (int i = 0; i < 100; ++i) writer.write(path, "content " + std::to_string(i));
    writer.remove(removed);
    writer.write(root + "/missing/dir/file.json", "x");  // 디렉터리가 없어 실패한다
    writer.flush();
    ASSERT_EQ(writer.failures(), 1u) << "flush가 돌아오면 실패한 쓰기 하나가 세어져 있다";
    ASSERT_EQ(read_text_file(path), "content 99") << "flush가 돌아오면 넘긴 쓰기가 끝나 있다";
    writer.write(path, "final");
  }  // 없앨 때 남은 쓰기를 마친다
  ASSERT_EQ(read_text_file(path), "final") << "같은 경로는 최신 내용이 남는다";
  struct stat st {};
  ASSERT_NE(stat(removed.c_str(), &st), 0) << "지우기 요청도 처리된다";
  ASSERT_NE(stat((path + ".tmp").c_str(), &st), 0) << "임시 파일은 남지 않는다";
  std::system(("rm -rf '" + root + "'").c_str());
}

}  // namespace
