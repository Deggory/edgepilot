#pragma once

// axmodel 하나를 AX620E NPU에서 돌리는 최소 세션. 입력 전부와 출력 전부를 각각 캐시 가능한 CMM
// 블록 하나에 페이지 경계로 나눠 잡아 두고(따로 할당할 때와 같은 시작·여유), 호출자는 input(i)/output(i)에
// 직접 읽고 쓴다. run()이 입력 블록 캐시를 밀어내고 실행한 뒤 출력 블록 캐시를 무효화한다. ax_engine 심볼은
// 이 클래스 안에만 둔다.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class AxEngineSession {
public:
    enum class DataType { Unknown, UInt8, UInt16, Float32, Other };

    struct Tensor {
        std::string name;
        std::vector<int> shape;
        DataType dtype = DataType::Unknown;
        size_t bytes = 0;
        uint64_t phys = 0;
        void *virt = nullptr;

        size_t count() const;
    };

    explicit AxEngineSession(const std::string &model_path);
    ~AxEngineSession();
    AxEngineSession(const AxEngineSession &) = delete;
    AxEngineSession &operator=(const AxEngineSession &) = delete;

    const std::vector<Tensor> &inputs() const { return inputs_; }
    const std::vector<Tensor> &outputs() const { return outputs_; }
    // 이름으로 찾는다. 없으면 -1.
    int input_index(const std::string &name) const;

    template <class T>
    T *input(size_t i) {
        return static_cast<T *>(inputs_[i].virt);
    }
    template <class T>
    const T *output(size_t i) const {
        return static_cast<const T *>(outputs_[i].virt);
    }

    bool run();
    const std::string &tools_version() const { return tools_version_; }

private:
    struct Block {
        uint64_t phys = 0;
        void *virt = nullptr;
        size_t bytes = 0;
    };
    // CMM은 페이지 단위로 할당하므로 텐서마다 따로 잡을 때도 페이지 경계에서 시작했다. 같게 둔다.
    static constexpr size_t kTensorAlign = 4096;

    void allocate(Block &block, std::vector<Tensor> &tensors, const void *metas, unsigned count);

    void *model_map_ = nullptr;  // 모델 파일의 읽기 전용 매핑(세션 동안 둔다)
    size_t model_size_ = 0;
    uint64_t *handle_ = nullptr;
    uint64_t *context_ = nullptr;
    Block input_block_;
    Block output_block_;
    std::vector<Tensor> inputs_;
    std::vector<Tensor> outputs_;
    std::vector<uint8_t> io_storage_;  // AX_ENGINE_IO_T와 버퍼 배열
    std::string tools_version_;
};
