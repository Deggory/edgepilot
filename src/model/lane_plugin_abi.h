// modeld와 차선선 플러그인(공유 라이브러리) 사이의 C ABI.
//
// 플러그인은 계산만 한다. modeld가 카메라 프레임의 관심 영역을 VPP로 잘라 플러그인 모델의 NV12 입력에 바로 쓰고
// NPU를 돌린 뒤 출력 텐서(fp32)를 넘기면, 플러그인이 차선선·도로 경계·소실점으로 해석한다. 모델 파일과 해석 코드는
// 플러그인 쪽에만 있고, 이 저장소에는 이 머리말과 그것을 부르는 쪽만 있다.
//
// 계약(docs/lane-line-plugin.md):
// - 플러그인은 AX SDK(NPU, IVPS, CMM)를 부르지 않는다. 초기화와 실행은 modeld가 한다.
// - decode는 modeld 주 스레드에서만 불린다. 예외를 밖으로 던지거나 프로세스를 끝내지 않고, 실패는 0이 아닌 값으로 알린다.
// - 결과의 좌표는 원본 프레임 화소다. 점은 가까운 쪽(영상 아래)부터 놓는다.
// - 열거값은 이 저장소의 어휘다. 모델의 클래스는 플러그인이 여기로 옮긴다.
#ifndef LANE_PLUGIN_ABI_H
#define LANE_PLUGIN_ABI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EP_LANE_PLUGIN_ABI_VERSION 1u
// 플러그인이 내보내는 진입점 이름(EpLanePluginEntry 형).
#define EP_LANE_PLUGIN_ENTRY "edgepilot_lane_plugin_v1"

enum { EP_LANE_MAX_LINES = 6, EP_LANE_MAX_BOUNDARIES = 4, EP_LANE_MAX_POINTS = 72 };

// 차선선 자리: 자차 차선의 왼쪽·오른쪽 선, 옆 차선이 보는 같은 선, 그 바깥 선.
enum EpLaneSlot {
    EP_LANE_EGO_LEFT = 0,
    EP_LANE_EGO_RIGHT = 1,
    EP_LANE_LEFT_LANE_RIGHT = 2,
    EP_LANE_RIGHT_LANE_LEFT = 3,
    EP_LANE_LEFT_NEXT = 4,
    EP_LANE_RIGHT_NEXT = 5,
};
enum EpLaneShape { EP_LANE_SOLID = 0, EP_LANE_DASHED = 1, EP_LANE_BOTTS = 2 };
enum EpLaneDouble { EP_LANE_SINGLE = 0, EP_LANE_DOUBLE_SOLID = 1, EP_LANE_DOUBLE_DASHED = 2, EP_LANE_DOUBLE_BOTTS = 3 };
enum EpLaneColor { EP_LANE_WHITE = 0, EP_LANE_YELLOW = 1, EP_LANE_BLUE = 2, EP_LANE_COLOR_UNKNOWN = 3 };
enum EpBoundarySlot {
    EP_BOUNDARY_NEAR_LEFT = 0,
    EP_BOUNDARY_NEAR_RIGHT = 1,
    EP_BOUNDARY_FAR_LEFT = 2,
    EP_BOUNDARY_FAR_RIGHT = 3,
};
enum EpBoundaryType {
    EP_BOUNDARY_WALL = 0,
    EP_BOUNDARY_PARKED_CARS = 1,
    EP_BOUNDARY_GUARDRAIL = 2,
    EP_BOUNDARY_CURB = 3,
    EP_BOUNDARY_SEPARATOR = 4,
    EP_BOUNDARY_CONES = 5,
    EP_BOUNDARY_EDGE = 6,
    EP_BOUNDARY_UNKNOWN = 7,
    EP_BOUNDARY_FLAT = 8,
};

// NPU 출력 텐서 하나(fp32, 모델이 낸 배치 그대로).
typedef struct {
    const char *name;
    const float *data;
    uint64_t count;
} EpTensor;

// 원본 프레임과 플러그인 모델 입력의 관계(화소). crop은 원본에서 잘라 net 크기로 늘이거나 줄인 영역이다.
typedef struct {
    int32_t image_width, image_height;
    int32_t crop_x, crop_y, crop_width, crop_height;
    int32_t net_width, net_height;
} EpLaneGeometry;

typedef struct {
    float x, y;    // 원본 프레임 화소(차선선은 도색 가운데)
    float width;   // 차선선 도색 폭(가로 화소). 도로 경계는 0
    float edge_x;  // 차로 쪽 안쪽 경계의 x(같은 y). 도로 경계는 x와 같다
} EpLanePoint;

typedef struct {
    int32_t slot;         // EpLaneSlot
    int32_t shape;        // EpLaneShape
    int32_t double_line;  // EpLaneDouble
    int32_t color;        // EpLaneColor
    float confidence;     // 0..1
    uint32_t point_count;
    EpLanePoint points[EP_LANE_MAX_POINTS];
} EpLaneLine;

typedef struct {
    int32_t slot;  // EpBoundarySlot
    int32_t type;  // EpBoundaryType
    float confidence;
    uint32_t point_count;
    EpLanePoint points[EP_LANE_MAX_POINTS];
} EpLaneBoundary;

typedef struct {
    uint32_t line_count;
    uint32_t boundary_count;
    EpLaneLine lines[EP_LANE_MAX_LINES];
    EpLaneBoundary boundaries[EP_LANE_MAX_BOUNDARIES];
    float vanishing_x, vanishing_y;  // 원본 프레임 화소
    float vanishing_confidence;      // 0..1
} EpLaneResult;

typedef struct {
    uint32_t abi_version;  // EP_LANE_PLUGIN_ABI_VERSION
    uint32_t reserved;
    const char *name;        // 로그와 LaneState에 남길 이름(모델·버전)
    const char *model_file;  // 플러그인 파일과 같은 디렉터리의 axmodel 파일 이름(NV12 입력 1개, 출력 fp32)
    float roi[4];            // 원본 프레임에서 자를 영역 [왼쪽, 위, 오른쪽, 아래] 비율
    // 출력 텐서를 해석해 result를 채운다. 0이면 성공이고, 실패 코드는 플러그인이 정한다(로그에만 남는다).
    int32_t (*decode)(const EpTensor *outputs, uint32_t output_count, const EpLaneGeometry *geometry,
                      EpLaneResult *result);
} EpLanePlugin;

typedef const EpLanePlugin *(*EpLanePluginEntry)(void);

#ifdef __cplusplus
}

// 플러그인은 다른 저장소에서 따로 빌드된다. 아래 크기·자리가 바뀌면 EP_LANE_PLUGIN_ABI_VERSION도 함께 올린다(64비트).
#include <cstddef>
static_assert(sizeof(EpLanePoint) == 16 && sizeof(EpLaneLine) == 1176 && sizeof(EpLaneBoundary) == 1168 &&
                  sizeof(EpLaneResult) == 11748 && sizeof(EpLaneGeometry) == 32,
              "plugin ABI layout changed: bump EP_LANE_PLUGIN_ABI_VERSION");
static_assert(sizeof(void *) != 8 ||
                  (sizeof(EpTensor) == 24 && offsetof(EpTensor, count) == 16 && sizeof(EpLanePlugin) == 48 &&
                   offsetof(EpLanePlugin, roi) == 24 && offsetof(EpLanePlugin, decode) == 40),
              "plugin ABI layout changed: bump EP_LANE_PLUGIN_ABI_VERSION");
#endif

#endif
