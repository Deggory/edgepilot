#pragma once

/* 파일 쓰기를 맡는 스레드. 루프는 내용만 넘기고, 이 스레드가 임시 파일에 쓴 뒤 rename한다
 * (write_file_atomic). SD가 바빠 쓰기가 막혀도 제어·센서 루프는 멈추지 않는다. 같은 경로에 쌓인 일은
 * 최신 것만 남기고, 없앨 때 남은 일을 마친다. controlsd(학습값)와 locationd(조향 지연)가 쓴다. */

#include "common/utils_file.h"

#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

class BackgroundWriter {
public:
  // log_prefix는 실패 로그의 앞머리다("controlsd: learner write" → "... <경로> failed: <사유>").
  explicit BackgroundWriter(std::string log_prefix)
      : log_prefix_(std::move(log_prefix)), thread_([this] { run(); }) {}
  ~BackgroundWriter() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    condition_.notify_one();
    thread_.join();
  }
  BackgroundWriter(const BackgroundWriter &) = delete;
  BackgroundWriter &operator=(const BackgroundWriter &) = delete;

  void write(const std::string &path, const std::string &content) { submit(path, true, content); }
  void remove(const std::string &path) { submit(path, false, std::string()); }
  // 지금까지 실패한 쓰기 수. 넘긴 쪽이 다시 넘길지 정할 때 본다.
  uint64_t failures() const { return failures_.load(); }

private:
  struct Job {
    bool write = false;
    std::string content;
  };
  void submit(const std::string &path, bool write, const std::string &content) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_[path] = Job{write, content};
    }
    condition_.notify_one();
  }
  void run() {
    while (true) {
      std::map<std::string, Job> jobs;
      bool stop = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return stop_ || !pending_.empty(); });
        jobs.swap(pending_);
        stop = stop_;
      }
      for (const auto &[path, job] : jobs) {
        if (!job.write) {
          std::remove(path.c_str());
          continue;
        }
        if (!write_file_atomic(path, job.content)) {
          std::fprintf(stderr, "%s %s failed: %s\n", log_prefix_.c_str(), path.c_str(), std::strerror(errno));
          failures_.fetch_add(1);
        }
      }
      if (stop) return;
    }
  }

  const std::string log_prefix_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::map<std::string, Job> pending_;
  bool stop_ = false;
  std::atomic<uint64_t> failures_{0};
  std::thread thread_;  // 마지막 멤버: 위가 다 준비된 뒤 시작한다
};
