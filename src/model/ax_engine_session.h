#ifndef AX_ENGINE_SESSION_H
#define AX_ENGINE_SESSION_H

/* axmodel 하나를 AX620E NPU에서 돌리는 최소 세션. 입출력마다 캐시 가능한 CMM
 * 버퍼를 한 번 잡아 두고, 호출자는 input(i)/output(i)에 직접 읽고 쓴다. run()이
 * 입력 캐시를 밀어내고 실행한 뒤 출력 캐시를 무효화한다. ax_engine 심볼은
 * 이 클래스 안에만 둔다. */

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class AxEngineSession
{
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

    template <class T> T *input(size_t i) { return static_cast<T *>(inputs_[i].virt); }
    template <class T> const T *output(size_t i) const { return static_cast<const T *>(outputs_[i].virt); }

    bool run();
    const std::string &tools_version() const { return tools_version_; }

private:
    void allocate(std::vector<Tensor> &tensors, const void *metas, unsigned count);
    void release();

    std::vector<uint8_t> model_data_;
    uint64_t *handle_ = nullptr;
    uint64_t *context_ = nullptr;
    std::vector<Tensor> inputs_;
    std::vector<Tensor> outputs_;
    std::vector<uint8_t> io_storage_;  // AX_ENGINE_IO_T와 버퍼 배열
    std::string tools_version_;
};

#endif
