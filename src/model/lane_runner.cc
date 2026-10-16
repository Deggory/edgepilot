#include "model/lane_runner.h"

#include "maixcam2/maix_vpp.h"

#include <cstdio>
#include <stdexcept>
#include <utility>

LaneRunner::LaneRunner(std::unique_ptr<LanePlugin> plugin)
    : plugin_(std::move(plugin)), session_(plugin_->model_path(), "lane_plugin"), result_(new EpLaneResult()) {
    // 입력 하나: NV12 u8, 1 x (H * 3 / 2) x W x 1
    if (session_.inputs().size() != 1) throw std::runtime_error("lane model must have one input");
    const AxEngineSession::Tensor &in = session_.inputs()[0];
    if (in.dtype != AxEngineSession::DataType::UInt8 || in.shape.size() != 4 || in.shape[0] != 1 || in.shape[3] != 1 ||
        in.shape[1] % 3 != 0)
        throw std::runtime_error("lane model input must be NV12 u8 1 x (H*3/2) x W x 1");
    net_w_ = in.shape[2];
    net_h_ = in.shape[1] * 2 / 3;
    if (net_w_ <= 0 || net_h_ <= 0 || ((net_w_ | net_h_) & 1) != 0 ||
        in.bytes != static_cast<size_t>(net_w_) * static_cast<size_t>(net_h_) * 3 / 2)
        throw std::runtime_error("lane model input size is not an even NV12 frame");
    // 출력은 촘촘한 fp32만(플러그인은 count개를 이어진 배열로 읽는다). 세션의 출력 블록은 세션이 사는 동안 그대로라
    // 텐서 목록을 한 번 만든다.
    for (const AxEngineSession::Tensor &t : session_.outputs()) {
        if (t.dtype != AxEngineSession::DataType::Float32 || t.bytes != t.count() * sizeof(float))
            throw std::runtime_error("lane model output " + t.name + " is not dense fp32");
        outputs_.push_back({t.name.c_str(), static_cast<const float *>(t.virt), static_cast<uint64_t>(t.count())});
    }
}

bool LaneRunner::run(uint64_t slot_phys, int width, int height, bool chroma_vu,
                     const std::function<bool()> &still_valid, const LaneCamera &camera, const LaneRoadProfile &profile,
                     LaneState *out, std::string *why) {
    if (geometry_.image_width != width || geometry_.image_height != height)
        geometry_ = plugin_->geometry(width, height, net_w_, net_h_);
    if (geometry_.crop_width <= 0 || geometry_.crop_height <= 0) {
        *why = "empty plugin ROI";
        return false;
    }

    // 슬롯의 영역을 모델 입력 텐서로 바로(입력 블록은 CPU가 쓰지 않으므로 run()의 캐시 밀어내기가 VPP가 쓴
    // 내용을 덮지 않는다)
    VppNv12 src;
    src.phys = slot_phys;
    src.width = width;
    src.height = height;
    src.chroma_vu = chroma_vu;
    VppNv12 dst;
    dst.phys = session_.inputs()[0].phys;
    dst.width = net_w_;
    dst.height = net_h_;
    const int vpp =
        vpp_crop_resize_nv12(src, geometry_.crop_x, geometry_.crop_y, geometry_.crop_width, geometry_.crop_height, dst);
    if (vpp != 0) {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "VPP crop-resize 0x%x", static_cast<unsigned>(vpp));
        *why = buf;
        return false;
    }
    if (still_valid && !still_valid()) {
        *why = "ring slot overwritten";
        return false;
    }
    if (!session_.run()) {
        *why = "NPU run failed";
        return false;
    }
    int32_t code = 0;
    if (!plugin_->decode(outputs_.data(), static_cast<uint32_t>(outputs_.size()), geometry_, result_.get(), &code)) {
        *why = "plugin decode failed (" + std::to_string(code) + ")";
        return false;
    }
    fill_lane_state(*result_, camera, profile, width, height, out);
    out->valid = 1;
    return true;
}
