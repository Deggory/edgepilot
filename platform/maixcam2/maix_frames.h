#pragma once

// IVPS·VENC·VDEC에 넘기는 NV12 프레임, 늘이기 크기 바꾸기 설정, 캐시 없는 블록 풀. 여러 래퍼가 같은 설정을
// 쓴다. AX 헤더를 끌어오므로 platform/maixcam2의 .cc에서만 쓴다.

#include "ax_ivps_api.h"
#include "ax_pool_type.h"
#include "ax_sys_api.h"

#include <cstddef>
#include <cstring>

// phys에서 시작하는 w x h NV12(UV는 Y 바로 뒤, 행 간격은 폭). 가상 주소는 비워 둔다.
inline AX_VIDEO_FRAME_T nv12_frame(AX_U64 phys, int w, int h) {
    AX_VIDEO_FRAME_T f = {};
    f.u32Width = static_cast<AX_U32>(w);
    f.u32Height = static_cast<AX_U32>(h);
    f.enImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
    f.u32PicStride[0] = f.u32PicStride[1] = static_cast<AX_U32>(w);
    f.u64PhyAddr[0] = phys;
    f.u64PhyAddr[1] = phys + static_cast<AX_U64>(w) * h;
    f.u32FrameSize = static_cast<AX_U32>(static_cast<size_t>(w) * h * 3 / 2);
    return f;
}

// 비율을 보지 않고 대상 크기로 늘린다(같은 크기면 복사).
inline AX_IVPS_CROP_RESIZE_ATTR_T stretch_resize() {
    AX_IVPS_CROP_RESIZE_ATTR_T attr = {};
    attr.eSclType = AX_IVPS_SCL_TYPE_AUTO;
    attr.eSclInput = AX_IVPS_SCL_INPUT_SHARE;
    attr.tAspectRatio.eMode = AX_IVPS_ASPECT_RATIO_STRETCH;
    return attr;
}

// count개 블록의 캐시 없는 풀. 실패하면 AX_INVALID_POOLID.
inline AX_POOL make_noncache_pool(AX_U64 block_size, AX_U32 count) {
    AX_POOL_CONFIG_T cfg = {};
    cfg.MetaSize = 512;
    cfg.BlkCnt = count;
    cfg.BlkSize = block_size;
    cfg.CacheMode = AX_POOL_CACHE_MODE_NONCACHE;
    std::strcpy(reinterpret_cast<char *>(cfg.PartitionName), "anonymous");
    return AX_POOL_CreatePool(&cfg);
}
