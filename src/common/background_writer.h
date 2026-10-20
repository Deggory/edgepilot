#pragma once

// 파일 쓰기를 맡는 스레드. 루프는 내용만 넘기고, 이 스레드가 임시 파일에 쓴 뒤 rename한다
// (write_file_atomic). SD가 바빠 쓰기가 막혀도 제어·센서 루프는 멈추지 않는다. 같은 경로에 쌓인 일은
// 최신 것만 남기고, 실패한 쓰기는 retry_after(기본 12초, paramsd·torqued 저장 주기) 뒤에 다시 쓴다. 그 사이 같은
// 경로에 새 일이 오면 그것만 쓴다. 넘긴 쪽은 실패를 따로 챙기지 않는다: 주차 중처럼 내용이 바뀌지 않으면 다시 넘길
// 일이 없다. 없앨 때 남은 일과 다시 쓸 일을 마친다. controlsd(학습값)와 locationd(조향 지연)가 쓴다.

#include "common/utils_file.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <condition_variable>
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
    explicit BackgroundWriter(std::string log_prefix,
                              std::chrono::steady_clock::duration retry_after = std::chrono::seconds(12))
        : log_prefix_(std::move(log_prefix)), retry_after_(retry_after), thread_([this] { run(); }) {}
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

    void write(const std::string &path, std::string content) { submit(path, true, std::move(content)); }
    void remove(const std::string &path) { submit(path, false, std::string()); }
    // 넘긴 일을 다 마칠 때까지 기다린다(실패해 다시 쓸 일은 기다리지 않는다).
    void flush() {
        std::unique_lock<std::mutex> lock(mutex_);
        idle_.wait(lock, [this] { return pending_.empty() && !busy_; });
    }

private:
    using Clock = std::chrono::steady_clock;
    struct Job {
        bool write = false;
        std::string content;
    };
    struct Retry {
        Clock::time_point at;
        std::string content;
    };
    void submit(const std::string &path, bool write, std::string content) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_[path] = Job{write, std::move(content)};
            retries_.erase(path);  // 새 일이 실패한 옛 내용을 대신한다
        }
        condition_.notify_one();
    }
    // 쌓인 일과 때가 된 재시도(멈출 때는 전부)를 꺼낸다. mutex_를 쥐고 부른다.
    std::map<std::string, Job> take_jobs() {
        std::map<std::string, Job> jobs;
        jobs.swap(pending_);
        const Clock::time_point now = Clock::now();
        for (auto it = retries_.begin(); it != retries_.end();) {
            if (!stop_ && it->second.at > now) {
                ++it;
                continue;
            }
            jobs.emplace(it->first, Job{true, std::move(it->second.content)});
            it = retries_.erase(it);
        }
        return jobs;
    }
    void run() {
        while (true) {
            std::map<std::string, Job> jobs;
            bool stop = false;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                const auto has_work = [this] { return stop_ || !pending_.empty(); };
                if (retries_.empty()) {
                    condition_.wait(lock, has_work);
                } else {
                    Clock::time_point next = retries_.begin()->second.at;
                    for (const auto &entry : retries_) next = std::min(next, entry.second.at);
                    condition_.wait_until(lock, next, has_work);
                }
                jobs = take_jobs();
                stop = stop_;
                busy_ = true;
            }
            for (auto &[path, job] : jobs) {
                if (!job.write) {
                    std::remove(path.c_str());
                    continue;
                }
                if (write_file_atomic(path, job.content)) continue;
                std::fprintf(stderr, "%s %s failed: %s\n", log_prefix_.c_str(), path.c_str(), std::strerror(errno));
                std::lock_guard<std::mutex> lock(mutex_);
                // 그사이 같은 경로에 새 일이 왔으면 그것이 이긴다. 멈출 때의 실패는 다시 쓰지 않는다
                if (!stop && pending_.count(path) == 0)
                    retries_[path] = Retry{Clock::now() + retry_after_, std::move(job.content)};
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                busy_ = false;
            }
            idle_.notify_all();
            if (stop) return;
        }
    }

    const std::string log_prefix_;
    const Clock::duration retry_after_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::condition_variable idle_;  // flush(): 쌓인 일도, 하던 일도 없다
    std::map<std::string, Job> pending_;
    std::map<std::string, Retry> retries_;  // 실패해 at에 다시 쓸 내용(경로마다 하나)
    bool busy_ = false;
    bool stop_ = false;
    std::thread thread_;  // 마지막 멤버: 위가 다 준비된 뒤 시작한다
};
