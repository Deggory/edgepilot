#include "maixcam2/maix_ive_flow.h"

#include "ax_ive_api.h"
#include "ax_ivps_api.h"
#include "ax_sys_api.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

namespace {

struct Buf {
    AX_U64 phys = 0;
    uint8_t *virt = nullptr;
    size_t size = 0;
};

// 캐시 CMM: CPU가 점을 쓰고 결과·응답을 읽는다(쓰기 뒤 flush, 하드웨어가 쓴 뒤 invalidate)
Buf alloc_buf(size_t size)
{
    Buf b;
    void *v = nullptr;
    if (AX_SYS_MemAllocCached(&b.phys, &v, static_cast<AX_U32>(size), 128,
                              reinterpret_cast<const AX_S8 *>("ive_flow")) != 0)
        throw std::runtime_error("IVE flow buffer alloc failed");
    b.virt = static_cast<uint8_t *>(v);
    b.size = size;
    std::memset(b.virt, 0, size);
    AX_SYS_MflushCache(b.phys, b.virt, static_cast<AX_U32>(size));
    return b;
}
void free_buf(Buf &b)
{
    if (b.virt) AX_SYS_MemFree(b.phys, b.virt);
    b = Buf{};
}
void flush(const Buf &b) { AX_SYS_MflushCache(b.phys, b.virt, static_cast<AX_U32>(b.size)); }
void invalidate(const Buf &b) { AX_SYS_MinvalidateCache(b.phys, b.virt, static_cast<AX_U32>(b.size)); }

AX_IVE_IMAGE_T image(AX_U64 phys, uint8_t *virt, int w, int h, int stride, AX_IVE_IMAGE_TYPE_E type)
{
    AX_IVE_IMAGE_T im = {};
    im.au64PhyAddr[0] = phys;
    im.au64VirAddr[0] = reinterpret_cast<uintptr_t>(virt);
    im.au32Stride[0] = static_cast<AX_U32>(stride);
    im.u32Width = static_cast<AX_U32>(w);
    im.u32Height = static_cast<AX_U32>(h);
    im.enType = type;
    return im;
}
AX_IVE_MEM_INFO_T mem(const Buf &b)
{
    AX_IVE_MEM_INFO_T m = {};
    m.u64PhyAddr = b.phys;
    m.u64VirAddr = reinterpret_cast<uintptr_t>(b.virt);
    m.u32Size = static_cast<AX_U32>(b.size);
    return m;
}
AX_IVE_DATA_T data(AX_U64 phys, uint8_t *virt, int w, int h, int stride)
{
    AX_IVE_DATA_T d = {};
    d.u64PhyAddr = phys;
    d.u64VirAddr = reinterpret_cast<uintptr_t>(virt);
    d.u32Stride = static_cast<AX_U32>(stride);
    d.u32Width = static_cast<AX_U32>(w);
    d.u32Height = static_cast<AX_U32>(h);
    return d;
}

/* 작업 하나를 넣고 끝날 때까지 기다린다. 블록 질의는 작업이 길면 몇 ms 뒤 시간 초과를 내므로 다시 묻는다
 * (AX_ERR_IVE_QUERY_TIMEOUT은 열거값 0x54가 아니라 AX_DEF_ERR로 다시 정의된 전체 코드다). */
template <class Submit> bool run(Submit submit)
{
    AX_IVE_HANDLE handle = 0;
    if (submit(&handle) != 0) return false;
    AX_BOOL finished = AX_FALSE;
    AX_S32 r = AX_IVE_Query(handle, &finished, AX_TRUE);
    for (int i = 0; i < 2000 && r == AX_ERR_IVE_QUERY_TIMEOUT; ++i) r = AX_IVE_Query(handle, &finished, AX_TRUE);
    return r == 0;
}

struct PointQ {  // AX_IVE_POINT_U11Q20_T: y가 먼저
    AX_U32 y, x;
} __attribute__((packed));

constexpr float kQ20 = 1048576.0f;

} // namespace

struct IveFlow::Impl {
    struct Frame {
        AX_U64 l0_phys = 0;     // 원본 단: 호출자의 Y 평면(링 슬롯)을 복사하지 않고 그대로 읽는다
        int l0_stride = 0;
        Buf level[kLevels];     // 1단부터. NV12 크기로 잡는다(VPP가 YUV400을 못 내면 UV까지 쓴다)
    };
    enum class Engine { VppY400, VppNv12, Ive };
    int w = 0, h = 0;
    Frame frames[2];
    int cur = -1;  // 현재 프레임 자리(push가 바꾼다), 이전 프레임은 1 - cur
    int pushed = 0;
    Engine engine = Engine::VppY400;
    Buf blur, gx, gy, resp, corners, pts, nxt, status;
    AX_IVE_FILTER_CTRL_T gauss{};
    AX_IVE_SOBEL_CTRL_T sobel_x{}, sobel_y{};
    AX_IVE_LK_OPTICAL_FLOW_PYR_CTRL_T lk{};
    bool ive = false;

    ~Impl()
    {
        for (Frame &f : frames)
            for (Buf &b : f.level) free_buf(b);
        for (Buf *b : {&blur, &gx, &gy, &resp, &corners, &pts, &nxt, &status}) free_buf(*b);
        if (ive) AX_IVE_Exit();
    }

    /* 축소 단을 VPP 한 번으로(원본에서 1/2, 1/4를 함께) 만든다. modeld는 GDC 워프 때문에 AX_IVPS_Init으로
     * TDP·VPP 작업 큐 한 쌍(보드 전체에 넷)을 이미 쥐고 있지만 쓰지는 않았다. y400이면 출력에 Y만 쓴다. */
    AX_S32 pyramid_vpp(Frame &f, AX_U64 uv_phys, bool y400)
    {
        AX_VIDEO_FRAME_T src = {};
        src.u32Width = static_cast<AX_U32>(w);
        src.u32Height = static_cast<AX_U32>(h);
        src.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;  // NV21이어도 Y는 같다(UV는 버린다)
        src.u32PicStride[0] = src.u32PicStride[1] = static_cast<AX_U32>(f.l0_stride);
        src.u64PhyAddr[0] = f.l0_phys;
        src.u64PhyAddr[1] = uv_phys;
        src.u32FrameSize = static_cast<AX_U32>(static_cast<size_t>(f.l0_stride) * h * 3 / 2);
        AX_VIDEO_FRAME_T dst[kLevels - 1];
        AX_IVPS_RECT_T box[kLevels - 1];
        for (int l = 1; l < kLevels; ++l) {
            const int lw = w >> l, lh = h >> l;
            AX_VIDEO_FRAME_T &d = dst[l - 1];
            d = AX_VIDEO_FRAME_T{};
            d.u32Width = static_cast<AX_U32>(lw);
            d.u32Height = static_cast<AX_U32>(lh);
            d.enImgFormat = y400 ? AX_FORMAT_YUV400 : AX_FORMAT_YUV420_SEMIPLANAR;
            d.u32PicStride[0] = d.u32PicStride[1] = static_cast<AX_U32>(lw);
            d.u64PhyAddr[0] = f.level[l].phys;
            d.u64PhyAddr[1] = y400 ? 0 : f.level[l].phys + static_cast<AX_U64>(lw) * lh;
            d.u32FrameSize = static_cast<AX_U32>(static_cast<size_t>(lw) * lh * (y400 ? 2 : 3) / 2);
            box[l - 1] = AX_IVPS_RECT_T{};
            box[l - 1].nW = static_cast<AX_U16>(w);
            box[l - 1].nH = static_cast<AX_U16>(h);
        }
        AX_IVPS_CROP_RESIZE_ATTR_T attr = {};
        attr.eSclType = AX_IVPS_SCL_TYPE_AUTO;
        attr.eSclInput = AX_IVPS_SCL_INPUT_SHARE;
        attr.tAspectRatio.eMode = AX_IVPS_ASPECT_RATIO_STRETCH;
        return AX_IVPS_CropResizeV2Vpp(&src, kLevels - 1, box, dst, &attr);
    }

    // VPP를 못 쓸 때: IVE 5x5 가우시안 + 두 화소·두 행마다 하나(원본 단은 호출자 버퍼에서 바로 읽는다)
    bool pyramid_ive(Frame &f)
    {
        for (int l = 1; l < kLevels; ++l) {
            const int sw = w >> (l - 1), sh = h >> (l - 1);
            const AX_U64 src_phys = l == 1 ? f.l0_phys : f.level[l - 1].phys;
            const int src_stride = l == 1 ? f.l0_stride : sw;
            AX_IVE_IMAGE_T src = image(src_phys, nullptr, sw, sh, src_stride, AX_IVE_IMAGE_TYPE_U8C1);
            AX_IVE_IMAGE_T dst = image(blur.phys, blur.virt, sw, sh, sw, AX_IVE_IMAGE_TYPE_U8C1);
            if (!run([&](AX_IVE_HANDLE *hd) { return AX_IVE_Filter(hd, &src, &dst, &gauss, AX_FALSE); })) return false;
            AX_IVE_DATA_T ds = data(blur.phys, blur.virt, sw, sh, sw);
            AX_IVE_DATA_T dd = data(f.level[l].phys, f.level[l].virt, sw / 2, sh / 2, sw / 2);
            AX_IVE_DMA_CTRL_T dc = {};
            dc.enMode = AX_IVE_DMA_MODE_INTERVAL_COPY;  // 두 화소·두 행마다 하나
            dc.u8HorSegSize = 2;
            dc.u8ElemSize = 1;
            dc.u8VerSegRows = 2;
            if (!run([&](AX_IVE_HANDLE *hd) { return AX_IVE_DMA(hd, &ds, &dd, &dc, AX_FALSE); })) return false;
        }
        return true;
    }

    bool pyramid(Frame &f, AX_U64 uv_phys)
    {
        while (engine != Engine::Ive) {
            const AX_S32 r = pyramid_vpp(f, uv_phys, engine == Engine::VppY400);
            if (r == 0) return true;
            // 처음 실패하면 YUV400 → NV12 출력 → IVE 순으로 물러난다(다시 오르지는 않는다)
            const Engine next = engine == Engine::VppY400 ? Engine::VppNv12 : Engine::Ive;
            std::fprintf(stderr, "ive_flow: VPP pyramid (%s) failed 0x%x, falling back to %s\n",
                         engine == Engine::VppY400 ? "YUV400" : "NV12", static_cast<unsigned>(r),
                         next == Engine::Ive ? "IVE" : "NV12 output");
            engine = next;
        }
        return pyramid_ive(f);
    }
};

IveFlow::IveFlow(int width, int height) : impl_(new Impl), width_(width), height_(height)
{
    Impl &m = *impl_;
    m.w = width;
    m.h = height;
    // 가장 작은 단의 폭도 IVE가 요구하는 16화소 배수, 높이는 단마다 반으로 나누어떨어져야 한다
    if (width % (16 << (kLevels - 1)) != 0 || height % (1 << (kLevels - 1)) != 0 ||
        (height >> 1) < kBandTop + kBandRows)
        throw std::runtime_error("IVE flow: frame size " + std::to_string(width) + "x" + std::to_string(height) +
                                 " does not fit the pyramid or the corner band");
    if (AX_IVE_Init() != 0) throw std::runtime_error("AX_IVE_Init failed");
    m.ive = true;
    for (Impl::Frame &f : m.frames)
        for (int l = 1; l < kLevels; ++l)
            f.level[l] = alloc_buf(static_cast<size_t>(width >> l) * (height >> l) * 3 / 2);
    const size_t band = static_cast<size_t>(width / 2) * kBandRows * 2;
    m.blur = alloc_buf(static_cast<size_t>(width) * height);
    m.gx = alloc_buf(band);
    m.gy = alloc_buf(band);
    m.resp = alloc_buf(band);
    m.corners = alloc_buf(sizeof(AX_IVE_CORNER_DET_MEM_T));
    m.pts = alloc_buf(kMaxPoints * sizeof(PointQ));
    m.nxt = alloc_buf(kMaxPoints * sizeof(PointQ));
    m.status = alloc_buf(kMaxPoints);

    // 5x5 [1 4 6 4 1]² / 256 가우시안(Q10): OpenCV pyrDown과 같은 핵
    const int k[5] = {1, 4, 6, 4, 1};
    for (int r = 0; r < 5; ++r)
        for (int c = 0; c < 5; ++c) m.gauss.as8q10Mask[r * 5 + c] = k[r] * k[c] * 4;
    // 3x3 Sobel / 8(화소 단위): 코너 응답이 대부분 S16 안에 든다
    const int s[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            m.sobel_x.as8q10Mask[(r + 1) * 5 + (c + 1)] = s[r][c] * 128;
            m.sobel_y.as8q10Mask[(c + 1) * 5 + (r + 1)] = s[r][c] * 128;
        }
    // LK: 중심 차분 기울기(화소 단위), 보정 1.0, 행렬 축소 0, 반복 15회, 0.01 px에서 멈춘다
    m.lk.enOutMode = AX_IVE_LK_OPTICAL_FLOW_PYR_OUT_MODE_STATUS;
    m.lk.bUseInitFlow = AX_FALSE;
    m.lk.u8MaxLevel = kLevels - 1;
    m.lk.u3q5PyraScale = 64;  // 2.0
    m.lk.u8MatScale = 0;
    m.lk.u8q6GradCompensation = 64;
    m.lk.s8q10FilterWeight[2 * 5 + 1] = -512;
    m.lk.s8q10FilterWeight[2 * 5 + 3] = 512;
    m.lk.u7q12FltGEpsilon = 1;
    m.lk.u0q16IterEpsilon = 655;
    m.lk.u8MaxIter = 15;
}

IveFlow::~IveFlow() = default;

bool IveFlow::push(unsigned long long y_phys, int stride, unsigned long long uv_phys)
{
    Impl &m = *impl_;
    if (stride % 16 != 0 || stride < m.w) return false;
    const int next = m.cur < 0 ? 0 : 1 - m.cur;
    Impl::Frame &f = m.frames[next];
    f.l0_phys = y_phys;
    f.l0_stride = stride;
    if (!m.pyramid(f, uv_phys ? uv_phys : y_phys + static_cast<AX_U64>(stride) * m.h)) return false;
    m.cur = next;
    ++m.pushed;
    return true;
}

const char *IveFlow::pyramid_engine() const
{
    switch (impl_->engine) {
    case Impl::Engine::VppY400:
        return "vpp";
    case Impl::Engine::VppNv12:
        return "vpp-nv12";
    case Impl::Engine::Ive:
        return "ive";
    }
    return "?";
}

const int16_t *IveFlow::corner_response()
{
    Impl &m = *impl_;
    if (m.cur < 0) return nullptr;
    const int cw = m.w / 2;
    const Buf &l1 = m.frames[m.cur].level[1];
    const size_t off = static_cast<size_t>(kBandTop) * cw;
    AX_IVE_IMAGE_T src = image(l1.phys + off, l1.virt + off, cw, kBandRows, cw, AX_IVE_IMAGE_TYPE_U8C1);
    AX_IVE_IMAGE_T ix = image(m.gx.phys, m.gx.virt, cw, kBandRows, cw, AX_IVE_IMAGE_TYPE_U16C1);
    AX_IVE_IMAGE_T iy = image(m.gy.phys, m.gy.virt, cw, kBandRows, cw, AX_IVE_IMAGE_TYPE_U16C1);
    if (!run([&](AX_IVE_HANDLE *hd) { return AX_IVE_Sobel(hd, &src, &ix, &m.sobel_x, AX_FALSE); })) return nullptr;
    if (!run([&](AX_IVE_HANDLE *hd) { return AX_IVE_Sobel(hd, &src, &iy, &m.sobel_y, AX_FALSE); })) return nullptr;
    AX_IVE_IMAGE_T cx = image(m.gx.phys, m.gx.virt, cw, kBandRows, cw, AX_IVE_IMAGE_TYPE_S16C1);
    AX_IVE_IMAGE_T cy = image(m.gy.phys, m.gy.virt, cw, kBandRows, cw, AX_IVE_IMAGE_TYPE_S16C1);
    AX_IVE_IMAGE_T cr = image(m.resp.phys, m.resp.virt, cw, kBandRows, cw, AX_IVE_IMAGE_TYPE_S16C1);
    AX_IVE_CORNER_DET_CTRL_T cc = {};
    cc.enMode = AX_IVE_CORNER_DET_MODE_ST;
    cc.u16MaxCornerNum = kMaxPoints;
    AX_IVE_MEM_INFO_T mc = mem(m.corners);
    if (!run([&](AX_IVE_HANDLE *hd) { return AX_IVE_CornerDet(hd, &cx, &cy, &mc, &cr, &cc, AX_FALSE); }))
        return nullptr;
    invalidate(m.resp);
    return reinterpret_cast<const int16_t *>(m.resp.virt);
}

bool IveFlow::track(const float *xy, int n, float *out_xy, uint8_t *ok, bool backward)
{
    Impl &m = *impl_;
    if (n <= 0) return true;
    std::fill(ok, ok + n, 0);
    if (m.pushed < 2) return false;  // 따라갈 이전 프레임이 없다
    n = std::min(n, static_cast<int>(kMaxPoints));
    const float top = static_cast<float>(1 << (kLevels - 1));
    auto *pq = reinterpret_cast<PointQ *>(m.pts.virt);
    for (int i = 0; i < n; ++i) {
        pq[i].x = static_cast<AX_U32>(std::lround(std::max(0.0f, xy[2 * i]) / top * kQ20));
        pq[i].y = static_cast<AX_U32>(std::lround(std::max(0.0f, xy[2 * i + 1]) / top * kQ20));
    }
    flush(m.pts);
    const int from = backward ? m.cur : 1 - m.cur;  // 점이 있는 프레임(앞으로면 이전, 거꾸로면 현재)
    const Impl::Frame &prev = m.frames[from], &next = m.frames[1 - from];
    AX_IVE_SRC_IMAGE_T a[kLevels], b[kLevels];
    a[0] = image(prev.l0_phys, nullptr, m.w, m.h, prev.l0_stride, AX_IVE_IMAGE_TYPE_U8C1);
    b[0] = image(next.l0_phys, nullptr, m.w, m.h, next.l0_stride, AX_IVE_IMAGE_TYPE_U8C1);
    for (int l = 1; l < kLevels; ++l) {
        const int lw = m.w >> l, lh = m.h >> l;
        a[l] = image(prev.level[l].phys, prev.level[l].virt, lw, lh, lw, AX_IVE_IMAGE_TYPE_U8C1);
        b[l] = image(next.level[l].phys, next.level[l].virt, lw, lh, lw, AX_IVE_IMAGE_TYPE_U8C1);
    }
    m.lk.u16CornerNum = static_cast<AX_U16>(n);
    AX_IVE_SRC_MEM_INFO_T mp = mem(m.pts);
    AX_IVE_MEM_INFO_T mn = mem(m.nxt);
    AX_IVE_DST_MEM_INFO_T ms = mem(m.status);
    if (!run([&](AX_IVE_HANDLE *hd) { return AX_IVE_LKOpticalFlowPyr(hd, a, b, &mp, &mn, &ms, &m.lk, AX_FALSE); }))
        return false;
    invalidate(m.nxt);
    invalidate(m.status);
    const auto *nq = reinterpret_cast<const PointQ *>(m.nxt.virt);
    for (int i = 0; i < n; ++i) {
        float dx = static_cast<float>(nq[i].x) / kQ20 - xy[2 * i];
        float dy = static_cast<float>(nq[i].y) / kQ20 - xy[2 * i + 1];
        if (dx >= 8.0f) dx -= 16.0f;  // 음수 이동은 +16 px로 나온다
        if (dy >= 8.0f) dy -= 16.0f;
        out_xy[2 * i] = xy[2 * i] + dx;
        out_xy[2 * i + 1] = xy[2 * i + 1] + dy;
        ok[i] = m.status.virt[i] != 0 && std::fabs(dx) <= kMaxStepPx && std::fabs(dy) <= kMaxStepPx;
    }
    return true;
}
