#ifndef AX_ENGINE_API_H
#define AX_ENGINE_API_H

/* AX620E(AX630C) NPU 런타임 libax_engine 선언과 NPU 세션이 쓰는 libax_sys 함수. 엔진
 * 헤더는 MSP SDK에 없어서, 같은 라이브러리를 여는 pyaxengine(_axe_capi.py)의 cffi 정의를
 * aarch64 64비트 배치 그대로 옮겼다. 쓰는 함수만 둔다. (GDC는 SDK 헤더를 쓰고, 이
 * 파일은 SDK 헤더와 같은 번역 단위에 넣지 않는다.) */

#include <cstdint>

extern "C" {

typedef int AX_S32;
typedef unsigned int AX_U32;
typedef unsigned long long int AX_U64;
typedef unsigned char AX_U8;
typedef signed char AX_S8;
typedef char AX_CHAR;
typedef void AX_VOID;
typedef AX_U32 AX_ENGINE_NPU_SET_T;

typedef enum { AX_FALSE = 0, AX_TRUE = 1 } AX_BOOL;

typedef enum {
    AX_ENGINE_TENSOR_LAYOUT_UNKNOWN = 0,
    AX_ENGINE_TENSOR_LAYOUT_NHWC = 1,
    AX_ENGINE_TENSOR_LAYOUT_NCHW = 2,
} AX_ENGINE_TENSOR_LAYOUT_T;

typedef enum {
    AX_ENGINE_MT_PHYSICAL = 0,
    AX_ENGINE_MT_VIRTUAL = 1,
    AX_ENGINE_MT_OCM = 2,
} AX_ENGINE_MEMORY_TYPE_T;

typedef enum {
    AX_ENGINE_DT_UNKNOWN = 0,
    AX_ENGINE_DT_UINT8 = 1,
    AX_ENGINE_DT_UINT16 = 2,
    AX_ENGINE_DT_FLOAT32 = 3,
    AX_ENGINE_DT_SINT16 = 4,
    AX_ENGINE_DT_SINT8 = 5,
    AX_ENGINE_DT_SINT32 = 6,
    AX_ENGINE_DT_UINT32 = 7,
    AX_ENGINE_DT_FLOAT64 = 8,
    AX_ENGINE_DT_BFLOAT16 = 9,
} AX_ENGINE_DATA_TYPE_T;

typedef enum { AX_ENGINE_VIRTUAL_NPU_DISABLE = 0, AX_ENGINE_VIRTUAL_NPU_ENABLE = 1 } AX_ENGINE_NPU_MODE_T;
typedef enum { AX_ENGINE_MODEL_TYPE0 = 0 } AX_ENGINE_MODEL_TYPE_T;

typedef struct {
    AX_ENGINE_NPU_MODE_T eHardMode;
    AX_U32 reserve[8];
} AX_ENGINE_NPU_ATTR_T;

typedef struct {
    AX_S32 eColorSpace;
    AX_U64 u64Reserved[18];
} AX_ENGINE_IO_META_EX_T;

typedef struct {
    AX_ENGINE_NPU_SET_T nNpuSet;
    AX_S8 *pName;
    AX_U32 reserve[8];
} AX_ENGINE_HANDLE_EXTRA_T;

typedef struct {
    AX_U32 nWbtIndex;
    AX_U64 u64Reserved[7];
} AX_ENGINE_IO_SETTING_T;

typedef struct {
    AX_CHAR *pName;
    AX_S32 *pShape;
    AX_U8 nShapeSize;
    AX_ENGINE_TENSOR_LAYOUT_T eLayout;
    AX_ENGINE_MEMORY_TYPE_T eMemoryType;
    AX_ENGINE_DATA_TYPE_T eDataType;
    AX_ENGINE_IO_META_EX_T *pExtraMeta;
    AX_U32 nSize;
    AX_U32 nQuantizationValue;
    AX_S32 *pStride;
    AX_U64 u64Reserved[9];
} AX_ENGINE_IO_META_T;

typedef struct {
    AX_ENGINE_IO_META_T *pInputs;
    AX_U32 nInputSize;
    AX_ENGINE_IO_META_T *pOutputs;
    AX_U32 nOutputSize;
    AX_U32 nMaxBatchSize;
    AX_BOOL bDynamicBatchSize;
    AX_U64 u64Reserved[11];
} AX_ENGINE_IO_INFO_T;

typedef struct {
    AX_U64 phyAddr;
    AX_VOID *pVirAddr;
    AX_U32 nSize;
    AX_S32 *pStride;
    AX_U8 nStrideSize;
    AX_U64 u64Reserved[11];
} AX_ENGINE_IO_BUFFER_T;

typedef struct {
    AX_ENGINE_IO_BUFFER_T *pInputs;
    AX_U32 nInputSize;
    AX_ENGINE_IO_BUFFER_T *pOutputs;
    AX_U32 nOutputSize;
    AX_U32 nBatchSize;
    AX_ENGINE_IO_SETTING_T *pIoSetting;
    AX_U64 u64Reserved[10];
} AX_ENGINE_IO_T;

AX_S32 AX_SYS_Init(AX_VOID);
AX_S32 AX_SYS_MemAllocCached(AX_U64 *phyaddr, AX_VOID **pviraddr, AX_U32 size, AX_U32 align,
                             const AX_S8 *token);
AX_S32 AX_SYS_MemFree(AX_U64 phyaddr, AX_VOID *pviraddr);
AX_S32 AX_SYS_MflushCache(AX_U64 phyaddr, AX_VOID *pviraddr, AX_U32 size);
AX_S32 AX_SYS_MinvalidateCache(AX_U64 phyaddr, AX_VOID *pviraddr, AX_U32 size);

AX_S32 AX_ENGINE_Init(AX_ENGINE_NPU_ATTR_T *pNpuAttr);
AX_S32 AX_ENGINE_CreateHandleV2(uint64_t **pHandle, const AX_VOID *pData, AX_U32 nDataSize,
                                AX_ENGINE_HANDLE_EXTRA_T *pExtraParam);
AX_S32 AX_ENGINE_DestroyHandle(uint64_t *nHandle);
AX_S32 AX_ENGINE_GetIOInfo(uint64_t *nHandle, AX_ENGINE_IO_INFO_T **pIO);
AX_S32 AX_ENGINE_CreateContextV2(uint64_t *nHandle, uint64_t **pContext);
AX_S32 AX_ENGINE_RunSyncV2(uint64_t *handle, uint64_t *context, AX_ENGINE_IO_T *pIO);
const AX_CHAR *AX_ENGINE_GetModelToolsVersion(uint64_t *nHandle);

}  // extern "C"

#endif
