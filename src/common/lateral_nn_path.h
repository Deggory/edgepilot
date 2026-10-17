#pragma once

// 학습한 토크 모델 고르기(docs/lateral-nn.md). 모델 파일은 설치 디렉터리의 models/lateral_nn/<이름>.json이고 이름은
// <차종>_v<버전>(예: k7_hev_v3)이다. 학습 값은 그 차에만 맞으므로 차종과 버전을 이름에 둔다. 무엇을 쓸지는 파라미터
// 디렉터리의 lateral_nn.json {"model": "<이름>"}이 정하고 웹 콘솔이 바꾼다. EDGEPILOT_LATERAL_NN_MODEL(파일 경로)이
// 있으면 고르기를 무시하고 그 파일을 쓴다(재생·시험용).

#include <string>

constexpr const char *kLateralNnModelDir = "models/lateral_nn";
constexpr const char *kLateralNnSelectionFile = "lateral_nn.json";

struct LateralNnChoice {
    std::string name;   // 고른 모델 이름(환경 변수로 정했으면 그 경로)
    std::string path;   // 읽을 파일. 고르기가 없거나 잘못이면 빈 문자열
    std::string error;  // path가 비었을 때 이유
};

// 이름에 쓸 수 있는 글자: 영문·숫자·'_'·'-'·'.'(첫 글자는 '.'이 아니다). 고른 이름이 디렉터리를 벗어나지 못한다.
bool valid_lateral_nn_name(const std::string &name);
// 고르기 파일(파라미터 디렉터리의 lateral_nn.json)
std::string lateral_nn_selection_path(const std::string &params_directory);
// 지금 고른 모델: 환경 변수, 없으면 고르기 파일. 인자 없으면 파라미터 디렉터리는 params_dir().
LateralNnChoice lateral_nn_choice();
LateralNnChoice lateral_nn_choice(const std::string &params_directory);
// 고르기 파일 내용으로 고른다(lateral_nn_choice가 쓰고, 시험이 부른다).
LateralNnChoice lateral_nn_choice_from(const std::string &selection_text, const std::string &model_dir);
