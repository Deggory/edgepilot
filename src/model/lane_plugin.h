#ifndef MODEL_LANE_PLUGIN_H
#define MODEL_LANE_PLUGIN_H

// 차선선 플러그인(공유 라이브러리) 불러오기, 결과 검사, 연속 크래시 가드. 플러그인 모델의 NPU 실행과 VPP 입력은
// 보드 쪽 model/lane_runner가 한다. 플러그인과의 약속은 model/lane_plugin_abi.h와 docs/lane-line-plugin.md에 있다.

#include "model/lane_plugin_abi.h"

#include <cstdint>
#include <memory>
#include <string>

class LanePlugin {
public:
    // 플러그인은 성공이라 했지만 결과가 약속(개수, 열거값, 유한값)을 어겼을 때의 코드와, decode가 예외를 던졌을 때의 코드.
    static constexpr int32_t kInvalidResult = -10000;
    static constexpr int32_t kThrew = -10001;

    // path의 공유 라이브러리를 열고 진입점, ABI 버전, 필수 항목을 본다. 쓸 수 없으면 nullptr과 이유(why).
    static std::unique_ptr<LanePlugin> open(const std::string &path, std::string *why);
    ~LanePlugin();
    LanePlugin(const LanePlugin &) = delete;
    LanePlugin &operator=(const LanePlugin &) = delete;

    const char *name() const { return api_->name; }
    // 플러그인 모델 파일: 플러그인과 같은 디렉터리의 model_file.
    std::string model_path() const { return dir_ + "/" + api_->model_file; }
    // image_width x image_height 프레임에서 roi를 잘라 net_width x net_height 입력으로 넣을 때의 관계. NV12라 자를
    // 영역의 위치와 크기를 짝수로 맞춘다. 영역이 비면 crop_width나 crop_height가 0이다.
    EpLaneGeometry geometry(int image_width, int image_height, int net_width, int net_height) const;
    // 해석한 뒤 결과를 검사한다. 신뢰도는 0..1로 자르고, 구조가 어긋나면(개수, 열거값, 유한하지 않은 값) 버린다.
    // 실패면 false와 code(플러그인의 코드, kInvalidResult 또는 kThrew). result는 실패해도 빈 결과로 남는다.
    bool decode(const EpTensor *outputs, uint32_t output_count, const EpLaneGeometry &geometry, EpLaneResult *result,
                int32_t *code) const;

private:
    LanePlugin(void *handle, const EpLanePlugin *api, std::string dir);

    void *handle_ = nullptr;
    const EpLanePlugin *api_ = nullptr;
    std::string dir_;
};

// 연속 크래시 가드. 플러그인 코드가 modeld를 죽이면 매니저가 modeld를 다시 띄우고 같은 플러그인을 다시 연다. 열 때
// 파일에 시작 횟수를 하나 올리고, 충분히 돈 뒤(healthy)나 정상 종료 때 지운다(예외로 풀려 내려가는 소멸은 정상
// 종료로 치지 않는다). 지우지 못한 시작이 max_starts번 쌓이면 다음부터는 열지 않는다. 파일은 tmpfs(/run)에 두므로
// 다시 부팅하면 다시 시도한다.
class LanePluginGuard {
public:
    explicit LanePluginGuard(std::string path, int max_starts = 2);
    ~LanePluginGuard();
    LanePluginGuard(const LanePluginGuard &) = delete;
    LanePluginGuard &operator=(const LanePluginGuard &) = delete;

    // 열어도 되면 true(시작 횟수를 올린다). 이미 max_starts번 쌓였으면 false.
    bool begin();
    // 충분히 돌았다: 기록을 지운다.
    void healthy();
    // 지우지 못하고 쌓인 시작 횟수(파일이 없으면 0).
    int starts() const;

private:
    std::string path_;
    int max_starts_ = 2;
    bool armed_ = false;  // begin이 올린 기록을 아직 지우지 않았다
};

#endif
