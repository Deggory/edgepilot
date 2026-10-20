#include "maixcam2/maix_ive_flow.h"

#include "ax_ive_api.h"
#include "maixcam2/maix_frames.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace {

struct Buf {
    AX_U64 phys = 0;
    uint8_t *virt = nullptr;
    size_t size = 0;
};

// CMM 블록. 캐시 가능한 것은 0으로 시작하고, CPU가 결과·응답을 읽기 전에 invalidate한다. CPU가 쓰기만 하는 것(LK
// 시작점)은 캐시 없이 잡아(maix_cmm처럼) flush가 없다. 그 버퍼는 track이 쓴 점까지만 IVE가 읽는다.
Buf alloc_buf(size_t size, bool cached = true) {
    Buf b;
    void *v = nullptr;
    const auto *token = reinterpret_cast<const AX_S8 *>("ive_flow");
    const AX_S32 r = cached ? AX_SYS_MemAllocCached(&b.phys, &v, static_cast<AX_U32>(size), 128, token)
                            : AX_SYS_MemAlloc(&b.phys, &v, static_cast<AX_U32>(size), 128, token);
    if (r != 0) throw std::runtime_error("IVE flow buffer alloc failed");
    b.virt = static_cast<uint8_t *>(v);
    b.size = size;
    if (cached) {
        std::memset(b.virt, 0, size);
        AX_SYS_MflushCache(b.phys, b.virt, static_cast<AX_U32>(size));
    }
    return b;
}
void free_buf(Buf &b) {
    if (b.virt) AX_SYS_MemFree(b.phys, b.virt);
    b = Buf{};
}
void invalidate(const Buf &b) { AX_SYS_MinvalidateCache(b.phys, b.virt, static_cast<AX_U32>(b.size)); }

AX_IVE_IMAGE_T image(AX_U64 phys, uint8_t *virt, int w, int h, int stride, AX_IVE_IMAGE_TYPE_E type) {
    AX_IVE_IMAGE_T im = {};
    im.au64PhyAddr[0] = phys;
    im.au64VirAddr[0] = reinterpret_cast<uintptr_t>(virt);
    im.au32Stride[0] = static_cast<AX_U32>(stride);
    im.u32Width = static_cast<AX_U32>(w);
    im.u32Height = static_cast<AX_U32>(h);
    im.enType = type;
    return im;
}
AX_IVE_MEM_INFO_T mem(const Buf &b) {
    AX_IVE_MEM_INFO_T m = {};
    m.u64PhyAddr = b.phys;
    m.u64VirAddr = reinterpret_cast<uintptr_t>(b.virt);
    m.u32Size = static_cast<AX_U32>(b.size);
    return m;
}
// 블록 안의 offset부터 size 바이트
Buf part(const Buf &b, size_t offset, size_t size) { return Buf{b.phys + offset, b.virt + offset, size}; }
AX_IVE_DATA_T data(AX_U64 phys, uint8_t *virt, int w, int h, int stride) {
    AX_IVE_DATA_T d = {};
    d.u64PhyAddr = phys;
    d.u64VirAddr = reinterpret_cast<uintptr_t>(virt);
    d.u32Stride = static_cast<AX_U32>(stride);
    d.u32Width = static_cast<AX_U32>(w);
    d.u32Height = static_cast<AX_U32>(h);
    return d;
}

// 작업 하나를 넣고 끝날 때까지 기다린다. 블록 질의는 작업이 길면 몇 ms 뒤 시간 초과를 내므로 다시 묻는다
// (AX_ERR_IVE_QUERY_TIMEOUT은 열거값 0x54가 아니라 AX_DEF_ERR로 다시 정의된 전체 코드다).
template <class Submit>
bool run(Submit submit) {
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

}  // namespace

struct IveFlow::Impl {
    struct Frame {
        AX_U64 l0_phys = 0;  // 원본 단: 호출자의 Y 평면(링 슬롯)을 복사하지 않고 그대로 읽는다
        int l0_stride = 0;
        Buf level[kLevels];  // 1단부터. VPP가 NV12로 내므로 UV까지 잡는다
    };
    int w = 0, h = 0;
    Frame frames[2];
    int cur = 0;      // 현재 프레임 자리(push가 바꾼다), 이전 프레임은 1 - cur
    bool vpp = true;  // 축소 단을 VPP로 만든다(실패하면 IVE로 물러나 그대로 간다)
    Buf blur;         // IVE 축소 단의 가우시안 결과(IVE로 물러날 때 잡는다)
    Buf gx, gy, resp, corners;
    Buf pts;  // LK 시작점(캐시 없음)
    // LK 출력 한 블록: 끝점(PointQ) kMaxPoints개, 이어서 128바이트 경계에서 상태 kMaxPoints바이트. invalidate가 한
    // 번이다(범위가 바뀌는 호출은 한 번에 약 40 µs).
    static constexpr size_t kNextBytes = kMaxPoints * sizeof(PointQ);
    static constexpr size_t kStatusOffset = (kNextBytes + 127) / 128 * 128;
    Buf lk_out;
    AX_IVE_FILTER_CTRL_T gauss{};
    AX_IVE_SOBEL_CTRL_T sobel_x{}, sobel_y{};
    AX_IVE_LK_OPTICAL_FLOW_PYR_CTRL_T lk{};
    bool ive = false;

    ~Impl() {
        for (Frame &f : frames)
            for (Buf &b : f.level) free_buf(b);
        for (Buf *b : {&blur, &gx, &gy, &resp, &corners, &pts, &lk_out}) free_buf(*b);
        if (ive) AX_IVE_Exit();
    }

    // 축소 단을 VPP 한 번으로(원본에서 1/2, 1/4를 함께) 만든다. modeld는 GDC 워프 때문에 AX_IVPS_Init으로
    // TDP·VPP 작업 큐 한 쌍(보드 전체에 넷)을 이미 쥐고 있지만 쓰지는 않았다.
    AX_S32 pyramid_vpp(Frame &f) {
        AX_VIDEO_FRAME_T src = nv12_frame(f.l0_phys, w, h);  // 링 슬롯(행 간격 = 폭)
        AX_VIDEO_FRAME_T dst[kLevels - 1];
        AX_IVPS_RECT_T box[kLevels - 1];
        for (int l = 1; l < kLevels; ++l) {
            dst[l - 1] = nv12_frame(f.level[l].phys, w >> l, h >> l);
            box[l - 1] = AX_IVPS_RECT_T{};
            box[l - 1].nW = static_cast<AX_U16>(w);
            box[l - 1].nH = static_cast<AX_U16>(h);
        }
        AX_IVPS_CROP_RESIZE_ATTR_T attr = stretch_resize();
        return AX_IVPS_CropResizeV2Vpp(&src, kLevels - 1, box, dst, &attr);
    }

    // VPP를 못 쓸 때: IVE 5x5 가우시안 + 두 화소·두 행마다 하나(원본 단은 호출자 버퍼에서 바로 읽는다)
    bool pyramid_ive(Frame &f) {
        if (!blur.virt) blur = alloc_buf(static_cast<size_t>(w) * h);
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

    bool pyramid(Frame &f) {
        if (vpp) {
            const AX_S32 r = pyramid_vpp(f);
            if (r == 0) return true;
            // 처음 실패하면 IVE로 물러난다(다시 오르지는 않는다)
            std::fprintf(stderr, "ive_flow: VPP pyramid failed 0x%x, falling back to IVE\n", static_cast<unsigned>(r));
            vpp = false;
        }
        return pyramid_ive(f);
    }
};

IveFlow::IveFlow(int width, int height) : impl_(new Impl), width_(width) {
    Impl &m = *impl_;
    m.w = width;
    m.h = height;
    if (AX_IVE_Init() != 0) throw std::runtime_error("AX_IVE_Init failed");
    m.ive = true;
    for (Impl::Frame &f : m.frames)
        for (int l = 1; l < kLevels; ++l)
            f.level[l] = alloc_buf(static_cast<size_t>(width >> l) * (height >> l) * 3 / 2);
    const size_t band = static_cast<size_t>(width / 2) * kBandRows * 2;
    m.gx = alloc_buf(band);
    m.gy = alloc_buf(band);
    m.resp = alloc_buf(band);
    m.corners = alloc_buf(sizeof(AX_IVE_CORNER_DET_MEM_T));
    m.pts = alloc_buf(kMaxPoints * sizeof(PointQ), false);
    m.lk_out = alloc_buf(Impl::kStatusOffset + kMaxPoints);

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

bool IveFlow::push(unsigned long long y_phys, int stride) {
    Impl &m = *impl_;
    const int next = 1 - m.cur;
    Impl::Frame &f = m.frames[next];
    f.l0_phys = y_phys;
    f.l0_stride = stride;
    if (!m.pyramid(f)) return false;
    m.cur = next;
    return true;
}

const char *IveFlow::pyramid_engine() const { return impl_->vpp ? "vpp" : "ive"; }

// FlowTracker는 push가 된 뒤에만 부른다
const int16_t *IveFlow::corner_response() {
    Impl &m = *impl_;
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

// FlowTracker는 점이 있고(n은 kMaxPoints 이하) 이전 프레임이 이어질 때만 부른다
bool IveFlow::track(const float *xy, int n, float *out_xy, uint8_t *ok, bool backward) {
    Impl &m = *impl_;
    std::fill(ok, ok + n, 0);
    const float top = static_cast<float>(1 << (kLevels - 1));
    auto *pq = reinterpret_cast<PointQ *>(m.pts.virt);  // 캐시 없는 버퍼라 flush가 필요 없다
    for (int i = 0; i < n; ++i) {
        pq[i].x = static_cast<AX_U32>(std::lround(std::max(0.0f, xy[2 * i]) / top * kQ20));
        pq[i].y = static_cast<AX_U32>(std::lround(std::max(0.0f, xy[2 * i + 1]) / top * kQ20));
    }
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
    AX_IVE_MEM_INFO_T mn = mem(part(m.lk_out, 0, Impl::kNextBytes));
    AX_IVE_DST_MEM_INFO_T ms = mem(part(m.lk_out, Impl::kStatusOffset, kMaxPoints));
    if (!run([&](AX_IVE_HANDLE *hd) { return AX_IVE_LKOpticalFlowPyr(hd, a, b, &mp, &mn, &ms, &m.lk, AX_FALSE); }))
        return false;
    invalidate(m.lk_out);
    const auto *nq = reinterpret_cast<const PointQ *>(m.lk_out.virt);
    const uint8_t *status = m.lk_out.virt + Impl::kStatusOffset;
    for (int i = 0; i < n; ++i) {
        float dx = static_cast<float>(nq[i].x) / kQ20 - xy[2 * i];
        float dy = static_cast<float>(nq[i].y) / kQ20 - xy[2 * i + 1];
        if (dx >= 8.0f) dx -= 16.0f;  // 음수 이동은 +16 px로 나온다
        if (dy >= 8.0f) dy -= 16.0f;
        out_xy[2 * i] = xy[2 * i] + dx;
        out_xy[2 * i + 1] = xy[2 * i + 1] + dy;
        ok[i] = status[i] != 0 && std::fabs(dx) <= kMaxStepPx && std::fabs(dy) <= kMaxStepPx;
    }
    return true;
}
