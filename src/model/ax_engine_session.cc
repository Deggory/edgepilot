#include "model/ax_engine_session.h"
#include "model/ax_engine_api.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>

namespace {

// AX_SYS/AX_ENGINE 초기화는 프로세스당 한 번. 세션을 여러 개 만들어도 공유한다.
void init_runtime_once() {
    static std::once_flag once;
    static int status = 0;
    std::call_once(once, [] {
        if (AX_SYS_Init() != 0) {
            status = 1;
            return;
        }
        // AI-ISP(camerad)가 NPU 한 코어를 쓰므로 NPU는 늘 가상 분할로 부팅되고
        // (/boot/configs maix_npu_ai_isp=1), 모델은 코어 하나용(NPU1)으로 컴파일돼 있다.
        // NPU 모드는 시스템 전역이라 같은 모드로 초기화해야 한다.
        AX_ENGINE_NPU_ATTR_T attr;
        std::memset(&attr, 0, sizeof(attr));
        attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_ENABLE;
        if (AX_ENGINE_Init(&attr) != 0) status = 2;
    });
    if (status == 1) throw std::runtime_error("AX_SYS_Init failed");
    if (status == 2)
        throw std::runtime_error("AX_ENGINE_Init (virtual NPU) failed; is maix_npu_ai_isp=1 in /boot/configs?");
}

AxEngineSession::DataType to_dtype(AX_ENGINE_DATA_TYPE_T dt) {
    switch (dt) {
        case AX_ENGINE_DT_UINT8:
            return AxEngineSession::DataType::UInt8;
        case AX_ENGINE_DT_UINT16:
            return AxEngineSession::DataType::UInt16;
        case AX_ENGINE_DT_FLOAT32:
            return AxEngineSession::DataType::Float32;
        case AX_ENGINE_DT_UNKNOWN:
            return AxEngineSession::DataType::Unknown;
        default:
            return AxEngineSession::DataType::Other;
    }
}

}  // namespace

size_t AxEngineSession::Tensor::count() const {
    return std::accumulate(shape.begin(), shape.end(), size_t{1}, std::multiplies<size_t>());
}

AxEngineSession::AxEngineSession(const std::string &model_path, const char *cmm_tag) : cmm_tag_(cmm_tag) {
    // 모델 파일(34 MB)은 힙에 읽어 두지 않고 읽기 전용으로 매핑한다. 엔진은 핸들을 만들며 가중치를 CMM에 옮기므로,
    // 그 뒤에는 페이지를 놓아 준다(MADV_DONTNEED). 엔진이 포인터를 다시 읽어도 파일에서 다시 올라온다.
    const int fd = ::open(model_path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) throw std::runtime_error("cannot open axmodel: " + model_path);
    struct stat st = {};
    if (::fstat(fd, &st) != 0 || st.st_size <= 0) {
        ::close(fd);
        throw std::runtime_error("empty axmodel: " + model_path);
    }
    model_size_ = static_cast<size_t>(st.st_size);
    void *map = ::mmap(nullptr, model_size_, PROT_READ, MAP_PRIVATE, fd, 0);
    ::close(fd);
    if (map == MAP_FAILED) throw std::runtime_error("cannot map axmodel: " + model_path);
    model_map_ = map;

    init_runtime_once();

    AX_ENGINE_HANDLE_EXTRA_T extra;
    std::memset(&extra, 0, sizeof(extra));
    if (AX_ENGINE_CreateHandleV2(&handle_, model_map_, static_cast<AX_U32>(model_size_), &extra) != 0) {
        release();
        throw std::runtime_error("AX_ENGINE_CreateHandleV2 failed: " + model_path);
    }
    if (AX_ENGINE_CreateContextV2(handle_, &context_) != 0) {
        release();
        throw std::runtime_error("AX_ENGINE_CreateContextV2 failed");
    }
    ::madvise(model_map_, model_size_, MADV_DONTNEED);
    if (const char *v = AX_ENGINE_GetModelToolsVersion(handle_)) tools_version_ = v;

    AX_ENGINE_IO_INFO_T *info = nullptr;
    if (AX_ENGINE_GetIOInfo(handle_, &info) != 0 || !info) {
        release();
        throw std::runtime_error("AX_ENGINE_GetIOInfo failed");
    }
    try {
        allocate(input_block_, inputs_, info->pInputs, info->nInputSize);
        allocate(output_block_, outputs_, info->pOutputs, info->nOutputSize);
    } catch (...) {
        release();
        throw;
    }

    // AX_ENGINE_IO_T 한 개 + 입력·출력 버퍼 배열을 한 덩어리로 들고 다닌다.
    io_storage_.assign(sizeof(AX_ENGINE_IO_T) + sizeof(AX_ENGINE_IO_BUFFER_T) * (inputs_.size() + outputs_.size()), 0);
    auto *io = reinterpret_cast<AX_ENGINE_IO_T *>(io_storage_.data());
    auto *bufs = reinterpret_cast<AX_ENGINE_IO_BUFFER_T *>(io + 1);
    for (size_t i = 0; i < inputs_.size(); ++i) {
        bufs[i].phyAddr = inputs_[i].phys;
        bufs[i].pVirAddr = inputs_[i].virt;
        bufs[i].nSize = static_cast<AX_U32>(inputs_[i].bytes);
    }
    for (size_t i = 0; i < outputs_.size(); ++i) {
        auto &b = bufs[inputs_.size() + i];
        b.phyAddr = outputs_[i].phys;
        b.pVirAddr = outputs_[i].virt;
        b.nSize = static_cast<AX_U32>(outputs_[i].bytes);
    }
    io->pInputs = bufs;
    io->nInputSize = static_cast<AX_U32>(inputs_.size());
    io->pOutputs = bufs + inputs_.size();
    io->nOutputSize = static_cast<AX_U32>(outputs_.size());
}

AxEngineSession::~AxEngineSession() { release(); }

void AxEngineSession::allocate(Block &block, std::vector<Tensor> &tensors, const void *metas_ptr, unsigned count) {
    const auto *metas = static_cast<const AX_ENGINE_IO_META_T *>(metas_ptr);
    std::vector<size_t> offsets(count);
    size_t total = 0;
    for (unsigned i = 0; i < count; ++i) {
        offsets[i] = total;
        total += (metas[i].nSize + kTensorAlign - 1) / kTensorAlign * kTensorAlign;
    }
    if (total == 0) return;
    void *virt = nullptr;
    AX_U64 phys = 0;
    if (AX_SYS_MemAllocCached(&phys, &virt, static_cast<AX_U32>(total), 128,
                              reinterpret_cast<const AX_S8 *>(cmm_tag_)) != 0)
        throw std::runtime_error("AX_SYS_MemAllocCached failed for " + std::to_string(count) + " tensors");
    block.phys = phys;
    block.virt = virt;
    block.bytes = total;
    std::memset(virt, 0, total);
    AX_SYS_MflushCache(phys, virt, static_cast<AX_U32>(total));
    for (unsigned i = 0; i < count; ++i) {
        const AX_ENGINE_IO_META_T &m = metas[i];
        Tensor t;
        t.name = m.pName ? m.pName : "";
        t.shape.assign(m.pShape, m.pShape + m.nShapeSize);
        t.dtype = to_dtype(m.eDataType);
        t.bytes = m.nSize;
        t.phys = phys + offsets[i];
        t.virt = static_cast<uint8_t *>(virt) + offsets[i];
        tensors.push_back(std::move(t));
    }
}

void AxEngineSession::release() {
    for (Block *block : {&input_block_, &output_block_}) {
        if (block->virt) AX_SYS_MemFree(block->phys, block->virt);
        *block = Block();
    }
    inputs_.clear();
    outputs_.clear();
    if (handle_) AX_ENGINE_DestroyHandle(handle_);
    handle_ = nullptr;
    context_ = nullptr;
    if (model_map_) ::munmap(model_map_, model_size_);
    model_map_ = nullptr;
    model_size_ = 0;
}

int AxEngineSession::input_index(const std::string &name) const {
    for (size_t i = 0; i < inputs_.size(); ++i)
        if (inputs_[i].name == name) return static_cast<int>(i);
    return -1;
}

bool AxEngineSession::run() {
    // 블록마다 한 번씩. 텐서마다 부르면 출력 15개 invalidate만 프레임당 약 0.6 ms였다(범위가 바뀌는 호출은 한 번에
    // 약 40 µs, 같은 범위를 한 번에는 약 15 µs; 2026-10-13 보드 측정).
    if (input_block_.virt)
        AX_SYS_MflushCache(input_block_.phys, input_block_.virt, static_cast<AX_U32>(input_block_.bytes));
    auto *io = reinterpret_cast<AX_ENGINE_IO_T *>(io_storage_.data());
    if (AX_ENGINE_RunSyncV2(handle_, context_, io) != 0) return false;
    if (output_block_.virt)
        AX_SYS_MinvalidateCache(output_block_.phys, output_block_.virt, static_cast<AX_U32>(output_block_.bytes));
    return true;
}
