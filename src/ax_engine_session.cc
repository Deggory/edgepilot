#include "ax_engine_session.h"
#include "ax_engine_api.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>
#include <mutex>
#include <numeric>
#include <stdexcept>

namespace {

/* AX_SYS/AX_ENGINE 초기화는 프로세스당 한 번. 세션을 여러 개 만들어도 공유한다. */
void init_runtime_once()
{
    static std::once_flag once;
    static int status = 0;
    std::call_once(once, [] {
        if (AX_SYS_Init() != 0) {
            status = 1;
            return;
        }
        /* NPU 모드는 시스템 전역이다. 다른 프로세스(예: MaixCAM 앱)가 이미 다른
         * 모드(가상 NPU)로 초기화했으면 기본 모드로는 거부되므로(커널: "repeatedly
         * initialize npu with different npu attr"), pyaxengine처럼 현재 모드를
         * 먼저 읽어 같은 모드로 초기화한다. */
        AX_ENGINE_NPU_ATTR_T attr;
        std::memset(&attr, 0, sizeof(attr));
        if (AX_ENGINE_GetVNPUAttr(&attr) != 0) {
            /* 이 프로세스에서 아직 초기화 전이면 읽지 못한다. 커널이 알려 주는 현재 모드를
             * 따른다: AI-ISP(camerad)가 켜지면 NPU는 가상 분할(vnpu=enable)이다. */
            std::memset(&attr, 0, sizeof(attr));
            char mode[16] = {};
            if (FILE *f = std::fopen("/proc/ax_proc/npu/vnpu", "r")) {
                if (!std::fgets(mode, sizeof(mode), f)) mode[0] = '\0';
                std::fclose(f);
            }
            attr.eHardMode = std::strncmp(mode, "enable", 6) == 0 ? AX_ENGINE_VIRTUAL_NPU_ENABLE
                                                                   : AX_ENGINE_VIRTUAL_NPU_DISABLE;
        }
        if (AX_ENGINE_Init(&attr) != 0) status = 2;
        std::fprintf(stderr, "npu: %s mode\n",
                     attr.eHardMode == AX_ENGINE_VIRTUAL_NPU_ENABLE ? "virtual (split)" : "full");
    });
    if (status == 1) throw std::runtime_error("AX_SYS_Init failed");
    if (status == 2) throw std::runtime_error("AX_ENGINE_Init failed");
}

AxEngineSession::DataType to_dtype(AX_ENGINE_DATA_TYPE_T dt)
{
    switch (dt) {
    case AX_ENGINE_DT_UINT8: return AxEngineSession::DataType::UInt8;
    case AX_ENGINE_DT_UINT16: return AxEngineSession::DataType::UInt16;
    case AX_ENGINE_DT_FLOAT32: return AxEngineSession::DataType::Float32;
    case AX_ENGINE_DT_UNKNOWN: return AxEngineSession::DataType::Unknown;
    default: return AxEngineSession::DataType::Other;
    }
}

} // namespace

size_t AxEngineSession::Tensor::count() const
{
    return std::accumulate(shape.begin(), shape.end(), size_t{1}, std::multiplies<size_t>());
}

AxEngineSession::AxEngineSession(const std::string &model_path)
{
    std::ifstream file(model_path, std::ios::binary);
    if (!file) throw std::runtime_error("cannot open axmodel: " + model_path);
    model_data_.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    if (model_data_.empty()) throw std::runtime_error("empty axmodel: " + model_path);

    init_runtime_once();

    AX_ENGINE_HANDLE_EXTRA_T extra;
    std::memset(&extra, 0, sizeof(extra));
    if (AX_ENGINE_CreateHandleV2(&handle_, model_data_.data(),
                                 static_cast<AX_U32>(model_data_.size()), &extra) != 0)
        throw std::runtime_error("AX_ENGINE_CreateHandleV2 failed: " + model_path);
    if (AX_ENGINE_CreateContextV2(handle_, &context_) != 0) {
        release();
        throw std::runtime_error("AX_ENGINE_CreateContextV2 failed");
    }
    if (const char *v = AX_ENGINE_GetModelToolsVersion(handle_)) tools_version_ = v;

    AX_ENGINE_IO_INFO_T *info = nullptr;
    if (AX_ENGINE_GetIOInfo(handle_, &info) != 0 || !info) {
        release();
        throw std::runtime_error("AX_ENGINE_GetIOInfo failed");
    }
    try {
        allocate(inputs_, info->pInputs, info->nInputSize);
        allocate(outputs_, info->pOutputs, info->nOutputSize);
    } catch (...) {
        release();
        throw;
    }

    // AX_ENGINE_IO_T 한 개 + 입력·출력 버퍼 배열을 한 덩어리로 들고 다닌다.
    io_storage_.assign(sizeof(AX_ENGINE_IO_T) +
                           sizeof(AX_ENGINE_IO_BUFFER_T) * (inputs_.size() + outputs_.size()),
                       0);
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

AxEngineSession::~AxEngineSession()
{
    release();
}

void AxEngineSession::allocate(std::vector<Tensor> &tensors, const void *metas_ptr, unsigned count)
{
    const auto *metas = static_cast<const AX_ENGINE_IO_META_T *>(metas_ptr);
    for (unsigned i = 0; i < count; ++i) {
        const AX_ENGINE_IO_META_T &m = metas[i];
        Tensor t;
        t.name = m.pName ? m.pName : "";
        t.shape.assign(m.pShape, m.pShape + m.nShapeSize);
        t.dtype = to_dtype(m.eDataType);
        t.bytes = m.nSize;
        void *virt = nullptr;
        AX_U64 phys = 0;
        if (AX_SYS_MemAllocCached(&phys, &virt, m.nSize, 128,
                                  reinterpret_cast<const AX_S8 *>("supercombo")) != 0)
            throw std::runtime_error("AX_SYS_MemAllocCached failed for " + t.name);
        std::memset(virt, 0, m.nSize);
        AX_SYS_MflushCache(phys, virt, m.nSize);
        t.phys = phys;
        t.virt = virt;
        tensors.push_back(std::move(t));
    }
}

void AxEngineSession::release()
{
    for (auto *list : {&inputs_, &outputs_}) {
        for (auto &t : *list)
            if (t.virt) AX_SYS_MemFree(t.phys, t.virt);
        list->clear();
    }
    if (handle_) AX_ENGINE_DestroyHandle(handle_);
    handle_ = nullptr;
    context_ = nullptr;
}

int AxEngineSession::input_index(const std::string &name) const
{
    for (size_t i = 0; i < inputs_.size(); ++i)
        if (inputs_[i].name == name) return static_cast<int>(i);
    return -1;
}

bool AxEngineSession::run()
{
    for (auto &t : inputs_)
        AX_SYS_MflushCache(t.phys, t.virt, static_cast<AX_U32>(t.bytes));
    auto *io = reinterpret_cast<AX_ENGINE_IO_T *>(io_storage_.data());
    if (AX_ENGINE_RunSyncV2(handle_, context_, io) != 0) return false;
    for (auto &t : outputs_)
        AX_SYS_MinvalidateCache(t.phys, t.virt, static_cast<AX_U32>(t.bytes));
    return true;
}
