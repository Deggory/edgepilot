#ifndef MAIXCAM2_CMM_H
#define MAIXCAM2_CMM_H

/* 프로세스 사이에 공유하는 CMM(물리 연속) 블록. 카메라 프레임 링의 슬롯을 여기에
 * 두면 IVPS/GDC/VO가 복사 없이 직접 읽는다. 다른 프로세스는 물리 주소로 매핑한다.
 * 하드웨어가 쓰고 하드웨어가 읽으므로 캐시하지 않는 메모리를 쓴다(CPU가 읽는
 * 대체 경로도 캐시 일관성 문제가 없다). AX_SYS_Init 뒤에 쓴다. */

#include <cstddef>
#include <cstdint>

struct CmmBlock {
    unsigned long long phys = 0;  // AX_U64
    uint8_t *virt = nullptr;
    size_t size = 0;
};

bool cmm_alloc(CmmBlock *block, size_t size, const char *token);
void cmm_free(CmmBlock *block);
// 다른 프로세스가 할당한 블록을 매핑한다(캐시 없음). 실패하면 nullptr.
uint8_t *cmm_map(unsigned long long phys, size_t size);
void cmm_unmap(uint8_t *virt, size_t size);

#endif
