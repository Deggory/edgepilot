#include "model/lead_flow.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr double kLeadHalfWidthM = 0.9;
constexpr double kLeadBottomM = 0.3;
constexpr double kLeadTopM = 1.4;
constexpr double kShrinkX = 0.12;   // 상자 양옆을 폭의 이만큼 줄인다(옆 차선·배경)
constexpr double kShrinkY = 0.08;
constexpr int kBoxMinWidthPx = 8;
constexpr int kBoxMinHeightPx = 6;
constexpr int kBorderPx = 3;        // Sobel과 LK 창이 영상 밖을 읽지 않게
constexpr double kCornerQuality = 0.01;
constexpr int kCornerMinDist2 = 9;  // 고른 점끼리 3화소
constexpr double kKeepMargin = 0.4; // 점이 상자에서 폭·높이의 이만큼 벗어나면 버린다
constexpr double kMinWindowS = 0.2;
constexpr double kRefineMaxPx = 2.0;   // 다듬기가 이어 따라간 위치에서 이만큼 넘게 옮기면 버린다
constexpr int kRefineIterations = 10;
constexpr double kRefineMinDet = 1e-3; // 템플릿 기울기 행렬이 이보다 작으면(평평하다) 다듬지 않는다
// 상자 둘레 복사본의 여유: 버리는 경계 밖으로 템플릿 반폭 + 기울기 1 + 다듬기 이동 + 보간 1
constexpr int kRoiMarginPx = kLeadTemplateHalf + 1 + static_cast<int>(kRefineMaxPx) + 1;

} // namespace

bool project_lead_box(const FlowIntrinsics &k, const float rpy[3], float camera_height_m, float camera_offset_m,
                      float lead_x, float lead_y, int width, int height, LeadBox *box)
{
    if (!box) return false;
    *box = LeadBox{};
    if (!(lead_x >= kLeadMinDistanceM && lead_x <= kLeadMaxDistanceM) || !std::isfinite(lead_y) ||
        !std::isfinite(camera_height_m))
        return false;
    // tools/model/model_warp.py projection_matrix와 같다: 도로(x 앞, y 왼쪽, z 위, 원점은 카메라 밑 도로) ->
    // 장치(x 앞, y 오른쪽, z 아래, 보정 회전) -> 영상(x 오른쪽, y 아래, z 앞, 카메라 높이만큼 아래로 옮김)
    const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
    const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
    const double r[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                            {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                            {-sp, cp * sr, cp * cr}};
    // 모델은 오프셋만큼 오른쪽인 가상 카메라에서 본다(common/projection.cc와 같다)
    const double center_left = -(static_cast<double>(lead_y) + camera_offset_m);
    double u0 = 1e9, u1 = -1e9, v0 = 1e9, v1 = -1e9;
    for (const double dy : {-kLeadHalfWidthM, kLeadHalfWidthM}) {
        for (const double z : {kLeadBottomM, kLeadTopM}) {
            const double px = lead_x, py = -(center_left + dy), pz = -z;  // 장치 축으로 뒤집은 도로 점
            const double dx = r[0][0] * px + r[0][1] * py + r[0][2] * pz;
            const double dyv = r[1][0] * px + r[1][1] * py + r[1][2] * pz;
            const double dz = r[2][0] * px + r[2][1] * py + r[2][2] * pz;
            const double vx = dyv, vy = dz + camera_height_m, vz = dx;
            if (vz < 1.0) return false;
            const double u = k.fx * vx / vz + k.cx, v = k.fy * vy / vz + k.cy;
            u0 = std::min(u0, u);
            u1 = std::max(u1, u);
            v0 = std::min(v0, v);
            v1 = std::max(v1, v);
        }
    }
    const double w = u1 - u0, h = v1 - v0;
    LeadBox b;
    b.x0 = static_cast<int>(std::floor(u0 + kShrinkX * w));
    b.x1 = static_cast<int>(std::ceil(u1 - kShrinkX * w));
    b.y0 = static_cast<int>(std::floor(v0 + kShrinkY * h));
    b.y1 = static_cast<int>(std::ceil(v1 - kShrinkY * h));
    if (b.width() < kBoxMinWidthPx || b.y1 - b.y0 < kBoxMinHeightPx) return false;
    if (b.x0 < kBorderPx || b.y0 < kBorderPx || b.x1 > width - kBorderPx || b.y1 > height - kBorderPx) return false;
    *box = b;
    return true;
}

int pick_lead_corners(const uint8_t *y, int stride, const LeadBox &box, int max_points, float *xy)
{
    if (!y || box.empty() || max_points <= 0) return 0;
    // 상자와 테두리 2화소를 옮겨 둔다(링 슬롯은 캐시 없는 매핑이라 행 단위로 한 번만 읽는다)
    const int rx0 = box.x0 - 2, ry0 = box.y0 - 2, rw = box.width() + 4, rh = box.y1 - box.y0 + 4;
    std::vector<uint8_t> roi(static_cast<size_t>(rw) * rh);
    for (int r = 0; r < rh; ++r)
        std::memcpy(&roi[static_cast<size_t>(r) * rw], y + static_cast<size_t>(ry0 + r) * stride + rx0,
                    static_cast<size_t>(rw));
    auto at = [&](int r, int c) { return static_cast<int>(roi[static_cast<size_t>(r) * rw + c]); };
    std::vector<float> gxx(roi.size(), 0.0f), gyy(roi.size(), 0.0f), gxy(roi.size(), 0.0f);
    for (int r = 1; r < rh - 1; ++r) {
        for (int c = 1; c < rw - 1; ++c) {
            const int gx = (at(r - 1, c + 1) + 2 * at(r, c + 1) + at(r + 1, c + 1)) -
                           (at(r - 1, c - 1) + 2 * at(r, c - 1) + at(r + 1, c - 1));
            const int gy = (at(r + 1, c - 1) + 2 * at(r + 1, c) + at(r + 1, c + 1)) -
                           (at(r - 1, c - 1) + 2 * at(r - 1, c) + at(r - 1, c + 1));
            const size_t i = static_cast<size_t>(r) * rw + c;
            gxx[i] = static_cast<float>(gx * gx);
            gyy[i] = static_cast<float>(gy * gy);
            gxy[i] = static_cast<float>(gx * gy);
        }
    }
    // 상자 안 화소의 최소 고유값(3x3 창)
    const int bw = box.width(), bh = box.y1 - box.y0;
    std::vector<float> lambda(static_cast<size_t>(bw) * bh, 0.0f);
    float lmax = 0.0f;
    for (int r = 0; r < bh; ++r) {
        for (int c = 0; c < bw; ++c) {
            double a = 0.0, b = 0.0, d = 0.0;
            for (int dr = -1; dr <= 1; ++dr)
                for (int dc = -1; dc <= 1; ++dc) {
                    const size_t i = static_cast<size_t>(r + 2 + dr) * rw + (c + 2 + dc);
                    a += gxx[i];
                    d += gyy[i];
                    b += gxy[i];
                }
            const float l = static_cast<float>(0.5 * (a + d - std::sqrt((a - d) * (a - d) + 4.0 * b * b)));
            lambda[static_cast<size_t>(r) * bw + c] = l;
            lmax = std::max(lmax, l);
        }
    }
    if (lmax <= 0.0f) return 0;
    const float thr = static_cast<float>(kCornerQuality * lmax);
    struct Cand {
        float v;
        int r, c;
    };
    std::vector<Cand> cands;
    for (int r = 0; r < bh; ++r) {
        for (int c = 0; c < bw; ++c) {
            const float v = lambda[static_cast<size_t>(r) * bw + c];
            if (v < thr) continue;
            bool peak = true;
            for (int dr = -1; dr <= 1 && peak; ++dr)
                for (int dc = -1; dc <= 1 && peak; ++dc) {
                    const int rr = r + dr, cc = c + dc;
                    if ((dr || dc) && rr >= 0 && cc >= 0 && rr < bh && cc < bw &&
                        lambda[static_cast<size_t>(rr) * bw + cc] > v)
                        peak = false;
                }
            if (peak) cands.push_back({v, r, c});
        }
    }
    std::sort(cands.begin(), cands.end(), [](const Cand &a, const Cand &b) { return a.v > b.v; });
    int n = 0;
    for (const Cand &cd : cands) {
        bool far = true;
        for (int j = 0; j < n && far; ++j) {
            const float dx = xy[2 * j] - static_cast<float>(box.x0 + cd.c);
            const float dy = xy[2 * j + 1] - static_cast<float>(box.y0 + cd.r);
            far = dx * dx + dy * dy >= kCornerMinDist2;
        }
        if (!far) continue;
        xy[2 * n] = static_cast<float>(box.x0 + cd.c);
        xy[2 * n + 1] = static_cast<float>(box.y0 + cd.r);
        if (++n >= max_points) break;
    }
    return n;
}

namespace {

struct Similarity {
    double c = 1.0, d = 0.0, tx = 0.0, ty = 0.0;  // b = [c -d; d c] a + t
    double scale() const { return std::hypot(c, d); }
};

int count_inliers(const Similarity &s, const float *a, const float *b, int n, double thr2, std::vector<uint8_t> *mask)
{
    int count = 0;
    for (int i = 0; i < n; ++i) {
        const double ax = a[2 * i], ay = a[2 * i + 1];
        const double ex = s.c * ax - s.d * ay + s.tx - b[2 * i];
        const double ey = s.d * ax + s.c * ay + s.ty - b[2 * i + 1];
        const bool in = ex * ex + ey * ey < thr2;
        if (mask) (*mask)[static_cast<size_t>(i)] = in ? 1 : 0;
        count += in ? 1 : 0;
    }
    return count;
}

// 맞는 점으로 최소제곱 유사변환(2차원 Umeyama)
bool solve_least_squares(const float *a, const float *b, int n, const std::vector<uint8_t> &mask, Similarity *out)
{
    double max = 0.0, may = 0.0, mbx = 0.0, mby = 0.0;
    int m = 0;
    for (int i = 0; i < n; ++i) {
        if (!mask[static_cast<size_t>(i)]) continue;
        max += a[2 * i];
        may += a[2 * i + 1];
        mbx += b[2 * i];
        mby += b[2 * i + 1];
        ++m;
    }
    if (m < 2) return false;
    max /= m;
    may /= m;
    mbx /= m;
    mby /= m;
    double saa = 0.0, sdot = 0.0, scross = 0.0;
    for (int i = 0; i < n; ++i) {
        if (!mask[static_cast<size_t>(i)]) continue;
        const double ax = a[2 * i] - max, ay = a[2 * i + 1] - may;
        const double bx = b[2 * i] - mbx, by = b[2 * i + 1] - mby;
        saa += ax * ax + ay * ay;
        sdot += ax * bx + ay * by;
        scross += ax * by - ay * bx;
    }
    if (saa < 1e-6) return false;
    out->c = sdot / saa;
    out->d = scross / saa;
    out->tx = mbx - (out->c * max - out->d * may);
    out->ty = mby - (out->d * max + out->c * may);
    return true;
}

} // namespace

ScaleFit fit_similarity_scale(const float *a, const float *b, int n, double inlier_px)
{
    ScaleFit fit;
    if (!a || !b || n < 3) return fit;
    const double thr2 = inlier_px * inlier_px;
    // 같은 입력이면 같은 답이 나오게 고정한 의사난수로 두 점을 고른다
    uint32_t rng = 0x9e3779b9u;
    auto next = [&rng] {
        rng = rng * 1664525u + 1013904223u;
        return rng >> 8;
    };
    Similarity best;
    int best_count = 0;
    const int iterations = std::min(64, n * (n - 1) / 2);
    for (int it = 0; it < iterations; ++it) {
        const int i = static_cast<int>(next() % static_cast<uint32_t>(n));
        const int j = static_cast<int>(next() % static_cast<uint32_t>(n));
        if (i == j) continue;
        const double dax = a[2 * j] - a[2 * i], day = a[2 * j + 1] - a[2 * i + 1];
        const double dbx = b[2 * j] - b[2 * i], dby = b[2 * j + 1] - b[2 * i + 1];
        const double norm = dax * dax + day * day;
        if (norm < 4.0) continue;  // 너무 가까운 두 점은 크기를 잘 정하지 못한다
        Similarity s;
        s.c = (dax * dbx + day * dby) / norm;
        s.d = (dax * dby - day * dbx) / norm;
        s.tx = b[2 * i] - (s.c * a[2 * i] - s.d * a[2 * i + 1]);
        s.ty = b[2 * i + 1] - (s.d * a[2 * i] + s.c * a[2 * i + 1]);
        const int count = count_inliers(s, a, b, n, thr2, nullptr);
        if (count > best_count) {
            best_count = count;
            best = s;
        }
    }
    if (best_count < 3) return fit;
    std::vector<uint8_t> mask(static_cast<size_t>(n));
    count_inliers(best, a, b, n, thr2, &mask);
    Similarity refined = best;
    if (solve_least_squares(a, b, n, mask, &refined)) {
        const int count = count_inliers(refined, a, b, n, thr2, nullptr);
        if (count >= best_count) {
            best = refined;
            best_count = count;
        }
    }
    fit.valid = best.scale() > 0.0;
    fit.scale = best.scale();
    fit.inliers = best_count;
    return fit;
}

LeadFlowTracker::LeadFlowTracker(const FlowIntrinsics &k, int width, int height)
    : k_(k), width_(width), height_(height), hist_(static_cast<size_t>(kLeadMaxPoints) * kSlots * 2),
      tmpl_(static_cast<size_t>(kLeadMaxPoints) * kSlots * kTemplateSize), age_(kLeadMaxPoints),
      alive_(kLeadMaxPoints), seed_(2 * kLeadMaxPoints)
{
    live_.reserve(kLeadMaxPoints);
    fit_a_.reserve(2 * kLeadMaxPoints);
    fit_b_.reserve(2 * kLeadMaxPoints);
}

void LeadFlowTracker::drop_points()
{
    std::fill(alive_.begin(), alive_.end(), 0);
}

void LeadFlowTracker::reset()
{
    drop_points();
    have_lead_ = false;
    box_ = LeadBox{};
    roi_.clear();
    roi_w_ = roi_h_ = 0;
    state_ = LeadFlowState{};
}

int LeadFlowTracker::alive_points() const
{
    int count = 0;
    for (const uint8_t a : alive_) count += a ? 1 : 0;
    return count;
}

// 점이 버려지기 전까지 머무는 곳(상자와 kKeepMargin)과 템플릿·다듬기가 읽는 둘레를 행 단위로 한 번 복사한다
void LeadFlowTracker::copy_roi(const uint8_t *y, int stride)
{
    const int mx = static_cast<int>(std::ceil(kKeepMargin * box_.width())) + kRoiMarginPx;
    const int my = static_cast<int>(std::ceil(kKeepMargin * (box_.y1 - box_.y0))) + kRoiMarginPx;
    roi_x0_ = std::max(0, box_.x0 - mx);
    roi_y0_ = std::max(0, box_.y0 - my);
    roi_w_ = std::min(width_, box_.x1 + mx) - roi_x0_;
    roi_h_ = std::min(height_, box_.y1 + my) - roi_y0_;
    roi_.resize(static_cast<size_t>(roi_w_) * roi_h_);
    for (int r = 0; r < roi_h_; ++r)
        std::memcpy(&roi_[static_cast<size_t>(r) * roi_w_], y + static_cast<size_t>(roi_y0_ + r) * stride + roi_x0_,
                    static_cast<size_t>(roi_w_));
}

// (x, y) 둘레 (2·half+1)² 화소를 쌍선형 보간으로 뽑는다. 복사본을 벗어나면 false.
bool LeadFlowTracker::sample_patch(double x, double y, int half, float *out) const
{
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    const double fx = std::floor(x), fy = std::floor(y);
    if (fx - half < roi_x0_ || fy - half < roi_y0_ || fx + half + 1 >= roi_x0_ + roi_w_ ||
        fy + half + 1 >= roi_y0_ + roi_h_)
        return false;
    const int ix = static_cast<int>(fx) - roi_x0_ - half, iy = static_cast<int>(fy) - roi_y0_ - half;
    const float ax = static_cast<float>(x - fx), ay = static_cast<float>(y - fy);
    const float w00 = (1.0f - ax) * (1.0f - ay), w01 = ax * (1.0f - ay), w10 = (1.0f - ax) * ay, w11 = ax * ay;
    const int side = 2 * half + 1;
    for (int r = 0; r < side; ++r) {
        const uint8_t *row = &roi_[static_cast<size_t>(iy + r) * roi_w_ + ix];
        const uint8_t *below = row + roi_w_;
        for (int c = 0; c < side; ++c)
            out[r * side + c] = w00 * row[c] + w01 * row[c + 1] + w10 * below[c] + w11 * below[c + 1];
    }
    return true;
}

/* 10프레임 전 템플릿(그때 위치 둘레, 테두리 1화소 포함)을 이 프레임에 맞춘다. 이동만 푸는 역합성 LK로,
 * 이어 따라간 위치 xy에서 시작한다. 걸음마다 쌓인 오차가 여기서 없어진다. kRefineMaxPx 안이면 xy를 바꾼다. */
bool LeadFlowTracker::refine(const float *t, float *xy) const
{
    constexpr int h = kLeadTemplateHalf, n = 2 * h + 1, side = kTemplateSide;
    float gx[n * n], gy[n * n], inner[n * n], win[n * n];
    double hxx = 0.0, hxy = 0.0, hyy = 0.0;
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            const int i = (r + 1) * side + (c + 1), k = r * n + c;
            gx[k] = 0.5f * (t[i + 1] - t[i - 1]);
            gy[k] = 0.5f * (t[i + side] - t[i - side]);
            inner[k] = t[i];
            hxx += static_cast<double>(gx[k]) * gx[k];
            hxy += static_cast<double>(gx[k]) * gy[k];
            hyy += static_cast<double>(gy[k]) * gy[k];
        }
    }
    const double det = hxx * hyy - hxy * hxy;
    if (det < kRefineMinDet) return false;
    const double ixx = hyy / det, ixy = -hxy / det, iyy = hxx / det;
    double px = xy[0], py = xy[1];
    for (int it = 0; it < kRefineIterations; ++it) {
        if (!sample_patch(px, py, h, win)) return false;
        double bx = 0.0, by = 0.0;
        for (int k = 0; k < n * n; ++k) {
            const double e = static_cast<double>(win[k]) - inner[k];
            bx += gx[k] * e;
            by += gy[k] * e;
        }
        const double dx = ixx * bx + ixy * by, dy = ixy * bx + iyy * by;
        px -= dx;
        py -= dy;
        if (dx * dx + dy * dy < 1e-4) break;
    }
    if (std::hypot(px - xy[0], py - xy[1]) >= kRefineMaxPx) return false;
    xy[0] = static_cast<float>(px);
    xy[1] = static_cast<float>(py);
    return true;
}

bool LeadFlowTracker::begin(const LeadFlowInput &lead, uint64_t frame_id, double t_s, bool consecutive,
                            const uint8_t *y, int stride)
{
    if (!consecutive) drop_points();  // 이전 위치가 이 프레임 바로 앞이 아니다
    LeadBox box;
    const bool usable = y && lead.valid && lead.probability >= kLeadMinProbability &&
                        project_lead_box(k_, lead.rpy, lead.camera_height_m, lead.camera_offset_m, lead.x, lead.y,
                                         width_, height_, &box);
    if (!usable) {
        reset();
        state_.frame_id = frame_id;
        return false;
    }
    state_ = LeadFlowState{};
    state_.frame_id = frame_id;
    // 끼어들기·차선 변경으로 앞차가 바뀌면 거리나 옆 위치가 한 프레임에 튄다
    if (have_lead_ && (std::fabs(lead.x - last_x_) > std::max(4.0f, 0.15f * lead.x) ||
                       std::fabs(lead.y - last_y_) > 1.5f))
        drop_points();
    have_lead_ = true;
    last_x_ = lead.x;
    last_y_ = lead.y;
    state_.lead_x = lead.x;
    box_ = box;
    head_ = (head_ + 1) % kSlots;
    times_[head_] = t_s;
    copy_roi(y, stride);
    return alive_points() > 0;
}

int LeadFlowTracker::points(float *xy)
{
    live_.clear();
    for (int i = 0; i < kLeadMaxPoints; ++i) {
        if (!alive_[static_cast<size_t>(i)]) continue;
        const float *p = pos(i, 1);
        const size_t m = live_.size();
        xy[2 * m] = p[0];
        xy[2 * m + 1] = p[1];
        live_.push_back(i);
    }
    return static_cast<int>(live_.size());
}

void LeadFlowTracker::advance(const float *xy, const uint8_t *ok, const float *back, const uint8_t *back_ok)
{
    const double mx = kKeepMargin * box_.width(), my = kKeepMargin * (box_.y1 - box_.y0);
    for (size_t j = 0; j < live_.size(); ++j) {
        const int i = live_[j];
        const float *prev = pos(i, 1);
        const float px = xy[2 * j], py = xy[2 * j + 1];
        const bool inside = px >= box_.x0 - mx && px <= box_.x1 + mx && py >= box_.y0 - my && py <= box_.y1 + my;
        // 되돌려 따라간 위치가 직전 위치와 어긋나면 미끄러진 점이다(불빛 번짐, 가장자리)
        const bool tracks_back =
            back_ok[j] && std::hypot(back[2 * j] - prev[0], back[2 * j + 1] - prev[1]) < kLeadFbPx;
        // 놓쳤거나 앞차를 벗어난 점(배경)도 버린다. 산 점은 이 프레임의 템플릿을 남긴다.
        if (!ok[j] || !tracks_back || !inside || !sample_patch(px, py, kLeadTemplateHalf + 1, tmpl(i, 0))) {
            alive_[static_cast<size_t>(i)] = 0;
            continue;
        }
        float *p = pos(i, 0);
        p[0] = px;
        p[1] = py;
        ++age_[static_cast<size_t>(i)];
    }
    fit_a_.clear();
    fit_b_.clear();
    for (int i = 0; i < kLeadMaxPoints; ++i) {
        if (!alive_[static_cast<size_t>(i)] || age_[static_cast<size_t>(i)] < kLeadWindowSteps) continue;
        float b[2] = {pos(i, 0)[0], pos(i, 0)[1]};
        if (!refine(tmpl(i, kLeadWindowSteps), b)) continue;
        const float *a = pos(i, kLeadWindowSteps);
        fit_a_.push_back(a[0]);
        fit_a_.push_back(a[1]);
        fit_b_.push_back(b[0]);
        fit_b_.push_back(b[1]);
    }
    const int m = static_cast<int>(fit_a_.size() / 2);
    state_.points = static_cast<uint32_t>(m);
    if (m < kLeadMinPoints) return;
    const double window_s = times_[slot(0)] - times_[slot(kLeadWindowSteps)];
    if (window_s < kMinWindowS) return;
    const ScaleFit fit = fit_similarity_scale(fit_a_.data(), fit_b_.data(), m, kLeadInlierPx);
    state_.inliers = static_cast<uint32_t>(fit.inliers);
    if (!fit.valid || fit.inliers < kLeadMinInliers) return;
    state_.valid = 1;
    state_.window_s = static_cast<float>(window_s);
    state_.expansion_rate = static_cast<float>(std::log(fit.scale) / window_s);
}

void LeadFlowTracker::top_up()
{
    if (box_.empty() || roi_.empty()) return;
    int alive = alive_points();
    if (alive >= kLeadTopUpPoints) return;
    const LeadBox local{box_.x0 - roi_x0_, box_.y0 - roi_y0_, box_.x1 - roi_x0_, box_.y1 - roi_y0_};
    const int n = pick_lead_corners(roi_.data(), roi_w_, local, kLeadMaxPoints, seed_.data());
    int free_slot = 0;
    for (int c = 0; c < n && alive < kLeadMaxPoints; ++c) {
        const float x = seed_[2 * c] + static_cast<float>(roi_x0_), y = seed_[2 * c + 1] + static_cast<float>(roi_y0_);
        bool far = true;
        for (int i = 0; i < kLeadMaxPoints && far; ++i) {
            if (!alive_[static_cast<size_t>(i)]) continue;
            const float *p = pos(i, 0);
            far = (p[0] - x) * (p[0] - x) + (p[1] - y) * (p[1] - y) >= kCornerMinDist2;
        }
        if (!far) continue;
        while (free_slot < kLeadMaxPoints && alive_[static_cast<size_t>(free_slot)]) ++free_slot;
        if (free_slot >= kLeadMaxPoints) break;
        if (!sample_patch(x, y, kLeadTemplateHalf + 1, tmpl(free_slot, 0))) continue;
        float *p = pos(free_slot, 0);
        p[0] = x;
        p[1] = y;
        age_[static_cast<size_t>(free_slot)] = 0;
        alive_[static_cast<size_t>(free_slot)] = 1;
        ++alive;
    }
}
