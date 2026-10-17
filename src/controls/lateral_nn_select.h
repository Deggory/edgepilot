#pragma once

// 고른 학습 토크 모델을 읽고, 고르기가 바뀌면 controlsd를 다시 띄우지 않고 갈아 끼운다(docs/lateral-nn.md).
// - 고르기(common/lateral_nn_path.h)와 그 모델 파일을 1초마다 본다. 바뀌었으면 백그라운드 스레드가 읽는다(JSON 파싱과
//   시험 벡터 계산이 보드에서 수십 ms라 제어 틱에서 하지 않는다). 제어 루프는 다 읽은 틱에 받기만 한다.
// - 새 모델을 못 읽으면 지금 모델을 그대로 쓴다.
// - 읽은 결과는 상태 파일(EDGEPILOT_LATERAL_NN_STATUS, 기본 /dev/shm/edgepilot_lateral_nn_status.json)에 남겨 웹
//   콘솔이 보인다: 고른 이름, 쓰는 이름, 입력 수, 마지막 실패 이유.

#include "common/lateral_nn_path.h"
#include "common/utils_file.h"
#include "controls/lateral_nn.h"

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

class LateralNnSelector {
public:
    using Choose = std::function<LateralNnChoice()>;

    explicit LateralNnSelector(
        std::string status_path = default_status_path(), Choose choose = [] { return lateral_nn_choice(); });
    ~LateralNnSelector();

    // 시작 때: 지금 고른 모델을 이 자리에서 읽는다. 못 읽으면 nullptr(선형 피드포워드만).
    std::shared_ptr<const LateralNnModel> load_now();
    // 매 틱: 고르기를 다시 볼 때가 됐으면 보고, 바뀌었으면 읽기를 시작한다. 새 모델을 다 읽은 틱에만 true와 그 모델.
    bool poll(std::chrono::steady_clock::time_point now, std::shared_ptr<const LateralNnModel> *model);

    const std::string &loaded_name() const { return loaded_name_; }
    bool loading() const { return pending_.valid(); }
    const std::string &last_error() const { return last_error_; }
    static std::string default_status_path();

private:
    struct Attempt {
        LateralNnChoice choice;
        FileStamp stamp;
        std::shared_ptr<LateralNnModel> model;  // 못 읽었으면 nullptr
        std::string error;
    };
    static Attempt load(LateralNnChoice choice);
    bool same_as_wanted(const LateralNnChoice &choice, const FileStamp &stamp) const;
    void finish(const Attempt &attempt, bool startup);
    void write_status() const;

    std::string status_path_;
    Choose choose_;
    std::chrono::steady_clock::time_point next_check_{};
    std::future<Attempt> pending_;
    // 마지막으로 읽기를 시도한 고르기(성공이든 실패든 다시 시도하지 않는다; 파일이 바뀌면 다시 본다)
    std::string wanted_path_;
    FileStamp wanted_stamp_;
    std::string selected_name_;
    std::string loaded_name_;
    std::string loaded_path_;
    size_t loaded_inputs_ = 0;
    std::string last_error_;
};
