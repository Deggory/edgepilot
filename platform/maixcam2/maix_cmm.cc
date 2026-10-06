#include "maixcam2/maix_cmm.h"

#include "ax_sys_api.h"

bool cmm_alloc(CmmBlock *block, size_t size, const char *token)
{
    void *virt = nullptr;
    AX_U64 phys = 0;
    if (AX_SYS_MemAlloc(&phys, &virt, static_cast<AX_U32>(size), 128,
                        reinterpret_cast<const AX_S8 *>(token)) != 0)
        return false;
    block->phys = phys;
    block->virt = static_cast<uint8_t *>(virt);
    block->size = size;
    return true;
}

void cmm_free(CmmBlock *block)
{
    if (block->virt) AX_SYS_MemFree(block->phys, block->virt);
    *block = CmmBlock{};
}

uint8_t *cmm_map(unsigned long long phys, size_t size)
{
    return static_cast<uint8_t *>(AX_SYS_Mmap(phys, static_cast<AX_U32>(size)));
}

void cmm_unmap(uint8_t *virt, size_t size)
{
    if (virt) AX_SYS_Munmap(virt, static_cast<AX_U32>(size));
}
