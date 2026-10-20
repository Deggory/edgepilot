// BackgroundWriter: controlsd와 locationd가 SD 쓰기를 루프 밖으로 넘기는 스레드. 같은 경로에 쌓인 쓰기는
// 최신 내용만 남고, 지우기도 같은 순서를 따르며, flush는 넘긴 일이 끝날 때까지 기다리고, 없앨 때 남은
// 일을 마치고, 실패한 쓰기는 정한 시간 뒤에 다시 쓴다(그 사이 같은 경로에 새 일이 오면 그것만).
#include "common/background_writer.h"

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

namespace {

// path의 내용이 expected가 될 때까지 최대 2초 기다린다(다시 쓰기는 writer 스레드가 정한 시각에 한다)
bool wait_for_content(const std::string &path, const std::string &expected) {
    for (int i = 0; i < 200; ++i) {
        if (read_text_file(path) == expected) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

TEST(BackgroundWriter, LatestContentWins) {
    char root_template[] = "/tmp/gtest_background_writer_XXXXXX";
    const std::string root = mkdtemp(root_template);
    const std::string path = root + "/live_delay.json";
    const std::string removed = root + "/removed.json";
    write_file_atomic(removed, "old");
    {
        BackgroundWriter writer("gtest: write");
        for (int i = 0; i < 100; ++i) writer.write(path, "content " + std::to_string(i));
        writer.remove(removed);
        writer.flush();
        ASSERT_EQ(read_text_file(path), "content 99") << "flush가 돌아오면 넘긴 쓰기가 끝나 있다";
        writer.write(path, "final");
    }  // 없앨 때 남은 쓰기를 마친다
    ASSERT_EQ(read_text_file(path), "final") << "같은 경로는 최신 내용이 남는다";
    struct stat st{};
    ASSERT_NE(stat(removed.c_str(), &st), 0) << "지우기 요청도 처리된다";
    ASSERT_NE(stat((path + ".tmp").c_str(), &st), 0) << "임시 파일은 남지 않는다";
    std::system(("rm -rf '" + root + "'").c_str());
}

// 실패한 쓰기(SD 오류 등)는 넘긴 쪽이 다시 넘기지 않아도 retry_after 뒤에 다시 쓴다: 주차 중처럼 학습값이 바뀌지 않으면
// 넘긴 쪽은 같은 내용을 다시 넘기지 않는다. 그 사이 같은 경로에 새 내용이 오면 옛 내용은 다시 쓰지 않는다.
TEST(BackgroundWriter, FailedWritesAreRetried) {
    char root_template[] = "/tmp/gtest_background_writer_XXXXXX";
    const std::string root = mkdtemp(root_template);
    const std::string dir = root + "/params";
    const std::string retried = dir + "/live_parameters.json";
    const std::string replaced = dir + "/live_torque_parameters.bin";
    {
        BackgroundWriter writer("gtest: write", std::chrono::milliseconds(50));
        writer.write(retried, "learned");
        writer.write(replaced, "old");
        writer.flush();  // 디렉터리가 없어 둘 다 실패한다
        ASSERT_EQ(mkdir(dir.c_str(), 0755), 0);
        writer.write(replaced, "new");
        writer.flush();
        EXPECT_TRUE(wait_for_content(retried, "learned")) << "실패한 쓰기를 다시 쓴다";
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        EXPECT_EQ(read_text_file(replaced), "new") << "새 내용이 온 경로는 실패한 옛 내용을 다시 쓰지 않는다";
    }
    std::system(("rm -rf '" + root + "'").c_str());
}

}  // namespace
