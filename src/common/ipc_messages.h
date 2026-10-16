#ifndef IPC_MESSAGES_H
#define IPC_MESSAGES_H

// 프로세스 사이를 /dev/shm으로 건너가는 메시지 전부: 채널 매직·버전과 이름, 크기 상한, 채널
// 헤더, 상태 스냅샷. 채널 구현은 ipc_channels.h에, ModelState를 채우고 되돌리는 변환은
// model_state_fill.h와 hud_state.h에 있다. 메시지를 쓰기만 하는 코드는 이 헤더만 본다.

#include "common/app_config.h"
#include "common/model_output.h"
#include "common/utils_time.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

// ---- 채널 매직·버전 ----
constexpr uint32_t kIpcMagic = 0x4b323349;
constexpr uint32_t kIpcVersion = 1;
constexpr uint32_t kFrameRingMagic = 0x4b465249;
constexpr uint32_t kFrameRingVersion = 5;
constexpr uint32_t kCanQueueMagic = 0x4b435151;
constexpr uint32_t kCanQueueVersion = 2;  // 2: 머리에 futex 깨움(wake·waiters)

// ---- 채널 이름 ----
constexpr char kRoadAiFrameRing[] = "/edgepilot_road_ai";
constexpr char kRoadAiFrameTopic[] = "/edgepilot_road_ai_frame";
constexpr char kRecordFrameTopic[] = "/edgepilot_record_frame";
constexpr char kModelStateTopic[] = "/edgepilot_model_state";
constexpr char kManagerStateTopic[] = "/edgepilot_manager_state";
constexpr char kCanTopic[] = "/edgepilot_can";
constexpr char kSendCanTopic[] = "/edgepilot_sendcan";
constexpr char kCanLogTopic[] = "/edgepilot_can_log";
constexpr char kSendCanLogTopic[] = "/edgepilot_sendcan_log";
constexpr char kPandaStateTopic[] = "/edgepilot_panda_state";
constexpr char kControlStateTopic[] = "/edgepilot_control_state";
constexpr char kLearnerStateTopic[] = "/edgepilot_learner_state";
constexpr char kImuTopic[] = "/edgepilot_imu";
constexpr char kLocalizationStateTopic[] = "/edgepilot_localization";
constexpr char kRecordStateTopic[] = "/edgepilot_record_state";
// LaneState 레이아웃이 바뀌면 끝의 번호를 올린다: 다른 빌드의 modeld가 발행해도 다른 채널이라 섞여 읽히지 않는다.
constexpr char kLaneStateTopic[] = "/edgepilot_lane_state2";

// ---- 크기 ----
constexpr unsigned kFrameSlots = 8;
constexpr unsigned kMaxProcesses = 12;
constexpr unsigned kAiWidth = kDefaultAiWidth;
constexpr unsigned kAiHeight = kDefaultAiHeight;
constexpr unsigned kAiFrameBytes = kAiWidth * kAiHeight * 3 / 2;
constexpr unsigned kCanBatchMaxFrames = 256;
constexpr unsigned kCanQueueSlots = 64;

struct IpcHeader {
    uint32_t magic = kIpcMagic;
    uint32_t version = kIpcVersion;
    uint32_t payload_capacity = 0;
    // futex 단어: C++ 발행마다 하나 늘리고 늘 깨운다(LatestChannel::read_new가 기다린다). waiters는 기다리는 쪽이 세며
    // waiters를 보고서야 깨우던 예전 빌드 생산자와 맞추려고 남겨 둔다. Python 매니저의 ManagerState 발행은 둘 다 0으로 쓰고
    // 깨우지 않으므로, 그 채널을 기다리는 쪽은 WakeWaiter가 알아채고 폴링한다.
    std::atomic<uint32_t> wake{0};
    std::atomic<uint64_t> seq{0};
    std::atomic<uint64_t> timestamp_ns{0};
    std::atomic<uint32_t> payload_size{0};
    std::atomic<uint32_t> waiters{0};
};

static_assert(sizeof(IpcHeader) == 40, "IpcHeader layout is part of the Python manager ABI");
static_assert(sizeof(std::atomic<uint32_t>) == 4 && std::atomic<uint32_t>::is_always_lock_free,
              "IpcHeader::wake is a futex word");

struct FrameRingHeader {
    uint32_t magic = kFrameRingMagic;
    uint32_t version = kFrameRingVersion;
    uint32_t slot_count = kFrameSlots;
    uint32_t width = kAiWidth;
    uint32_t height = kAiHeight;
    uint32_t frame_bytes = kAiFrameBytes;
    uint32_t reserved0 = 0;
    uint32_t reserved1 = 0;
    std::atomic<uint64_t> slot_seq[kFrameSlots]{};
    std::atomic<uint64_t> slot_frame_id[kFrameSlots]{};
    // 슬롯 픽셀이 있는 CMM 블록의 물리 주소. 0이면 아직 붙지 않았다.
    uint64_t slot_phys[kFrameSlots]{};
};

struct RoadAiFrame {
    uint64_t frame_id = 0;
    uint64_t timestamp_ns = 0;
    uint32_t slot = 0;
    uint32_t width = kAiWidth;
    uint32_t height = kAiHeight;
    uint32_t format = 0;
    uint32_t crop_x = 0;
    uint32_t crop_y = 0;
    uint32_t crop_width = 0;
    uint32_t crop_height = 0;
};

struct IpcPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct LeadState {
    uint32_t valid = 0;
    float probability = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float velocity = 0.0f;
    float acceleration = 0.0f;
};

// 모델 앞차를 앞차로 쓸 수 있는가: 있고, 거리와 속도가 유한하고, 존재 확률이 kLeadProbabilityThreshold 이상이다.
// 비전 크루즈(controlsd)와 앞차 트랙(modeld)이 같은 규칙을 쓴다.
inline bool lead_usable(const LeadState &lead) {
    return lead.valid != 0 && lead.probability >= kLeadProbabilityThreshold && std::isfinite(lead.x) &&
           std::isfinite(lead.velocity);
}

struct PoseState {
    uint32_t valid = 0;
    float trans[3] = {};
    float rot[3] = {};
    float trans_std[3] = {};
    float rot_std[3] = {};
};

// 모델 입력 워프의 보정. 모든 필드가 워프에 쓴 보정과 그 출처(calibration_source.source)의 것이다.
// - 포즈 출처: status·valid_blocks·spread는 포즈 보정기 그대로(5블록이면 Calibrated, spread는 블록 범위).
// - 광류 출처: status는 Calibrated(쓸 만할 때만 쓴다), valid_blocks는 광류 추정에 든 블록 수(3블록부터 쓴다),
//   spread는 roll이 포즈 보정기 범위, pitch·yaw가 광류 블록 중앙값의 표준편차다.
// 출처마다 블록의 크기와 통계가 다르니 출처와 함께 읽는다.
struct CalibrationState {
    uint32_t status = 0;
    int32_t valid_blocks = 0;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float spread[3] = {};
};

// 광류 보정기(model/flow_calibration)의 상태. 카메라가 실제로 움직이는 방향을 영상에서 직접 잰
// 절대 측정이다. 웹 기기 설정이 광류이고 추정이 쓸 만하면 워프의 pitch·yaw가 된다(calibration_source).
// 각도는 calibration과 같은 규약(rad)이다.
struct FlowCalibrationState {
    uint64_t window_frame_id = 0;  // 마지막으로 받아들인 측정 창의 끝 프레임(0이면 아직 없음)
    uint32_t status = 0;           // 0 꺼짐, 1 모으는 중(블록 부족), 2 추정 있음
    uint32_t windows = 0;          // 받아들인 측정 창 수
    uint32_t blocks = 0;           // 끝난 블록 수(창 kFlowBlockWindows개마다 하나)
    float pitch = 0.0f;            // 블록 중앙값들의 평균
    float yaw = 0.0f;
    float sd_pitch = 0.0f;  // 블록 중앙값의 표준편차(v10 녹화는 이 자리가 spread_*, 범위였다)
    float sd_yaw = 0.0f;
    float window_pitch = 0.0f;  // 마지막 창 하나의 결과
    float window_yaw = 0.0f;
    float window_rotation = 0.0f;  // 그 창(0.4초) 동안 카메라가 돈 각도
    uint32_t window_tracks = 0;    // 끝까지 따라간 점
    uint32_t window_inliers = 0;   // 그중 풀이에 맞는 점
    // 녹화 v11부터
    uint32_t mount_changes = 0;     // 장착 변경으로 보고 예전 블록을 버린 횟수(저장 파일과 함께 이어진다)
    float window_parallax = 0.0f;   // 마지막으로 푼 창의 점 이동량 중앙값(px). 작으면 방향이 서지 않아 버린다
    uint32_t estimate_blocks = 0;   // 지금 추정에 든 블록 수(최대 kFlowMaxBlocks). blocks는 누적이다
    uint32_t rejected_windows = 0;  // 끝까지 갔지만 풀이가 맞지 않거나 이동량이 작아 버린 창
};
static_assert(sizeof(FlowCalibrationState) == 72, "FlowCalibrationState is recorded as-is");

// 워프에 쓴 보정의 출처와 모델 포즈 보정기 자신의 추정(녹화 v11부터). calibration은 언제나 워프에 쓴
// 값이다. 포즈 보정기는 자기 값이 워프를 맡을 때만 돈다(upstream 루프는 모델이 자기 값으로 워프된 영상을
// 봤다고 가정한다). 광류가 워프를 맡는 동안 pose_*는 멈춘 마지막 추정이다.
struct CalibrationSourceState {
    uint32_t source = 0;       // 0 모델 포즈, 1 광류
    uint32_t requested = 0;    // 웹 기기 설정(display.json calibration_use_flow). 결합 중에는 바뀌어도 기다린다
    uint32_t flags = 0;        // kCalibrationSource* 비트
    uint32_t pose_status = 0;  // 포즈 보정기의 CalibrationStatus
    int32_t pose_valid_blocks = 0;
    float pose_roll = 0.0f;  // rad
    float pose_pitch = 0.0f;
    float pose_yaw = 0.0f;
};
static_assert(sizeof(CalibrationSourceState) == 32, "CalibrationSourceState is recorded as-is");
constexpr uint32_t kCalibrationSourceFlowUsable = 1u << 0;  // 광류 추정을 워프에 쓸 수 있다
constexpr uint32_t kCalibrationSourcePending = 1u << 1;     // 설정이 바뀌었지만 결합 중이라 해제를 기다린다
constexpr uint32_t kCalibrationSourceRamping = 1u << 2;     // 워프가 새 목표로 천천히 옮겨 가는 중

// 앞차 확대율(녹화 v12부터, model/lead_flow). 모델 앞차 상자 안의 점을 광류로 따라가 창(약 0.5초) 동안의 크기 비를
// 잰다. 상대 속도(거리 변화율)는 -lead_x * expansion_rate(m/s, 음수면 가까워진다),
// 충돌까지 시간은 1 / expansion_rate다. 모델이 내는 앞차 속도는 25 m 밖에서 실제 접근 속도를 거의 따라가지 못해
// 따로 잰다.
struct LeadFlowState {
    uint64_t frame_id = 0;        // 창의 끝 프레임
    uint32_t valid = 0;           // 1이면 expansion_rate가 있다
    uint32_t points = 0;          // 창 내내 따라가 다듬은 점(맞춤에 넣은 수)
    uint32_t inliers = 0;         // 그중 같은 확대·이동에 맞는 점
    float expansion_rate = 0.0f;  // 1/s, ln(크기 비) / window_s. +면 가까워진다
    float window_s = 0.0f;        // 창 길이(s)
    float lead_x = 0.0f;          // 창 끝 프레임의 모델 앞차 거리(m, 카메라 기준)
};
static_assert(sizeof(LeadFlowState) == 32, "LeadFlowState is recorded as-is");

// 앞차 추적(녹화 v13부터, model/lead_tracker). 모델 앞차 거리와 광류 확대율을 칼만 필터로 섞은 거리와
// 상대 속도. 비전 크루즈가 쓴다(controls_tick observe_cruise_lead).
constexpr uint32_t kLeadTrackFlowUsed = 1u << 0;          // 이 프레임에 확대율을 넣었다
constexpr uint32_t kLeadTrackFlowRejected = 1u << 1;      // 확대율이 문 밖이라 버렸다
constexpr uint32_t kLeadTrackDistanceRejected = 1u << 2;  // 모델 거리가 문 밖이라 버렸다
constexpr uint32_t kLeadTrackStarted = 1u << 3;           // 이 프레임에 (다시) 시작했다
struct LeadTrackState {
    uint32_t valid = 0;               // 1이면 아래 값이 있다
    uint32_t flags = 0;               // kLeadTrack*
    float distance_m = 0.0f;          // 앞차 거리(m, 카메라 기준: lead.x와 같다)
    float relative_speed_mps = 0.0f;  // 앞차 - 자차(m/s, 음수면 가까워진다)
    float distance_std_m = 0.0f;      // 필터의 표준편차
    float relative_speed_std_mps = 0.0f;
};
static_assert(sizeof(LeadTrackState) == 24, "LeadTrackState is recorded as-is");

struct ModelState {
    uint64_t frame_id = 0;
    uint64_t capture_timestamp_ns = 0;
    uint64_t model_timestamp_ns = 0;
    float model_execution_ms = 0.0f;
    uint32_t valid = 0;
    int32_t best_plan = 0;
    float plan_probability = 0.0f;
    float model_t[kTrajectorySize] = {};
    float lane_t[kTrajectorySize] = {};
    IpcPoint plan[kTrajectorySize] = {};
    IpcPoint lanes[4][kTrajectorySize] = {};
    float lane_probabilities[4] = {};
    float lane_stds[4] = {};
    IpcPoint road_edges[2][kTrajectorySize] = {};
    float road_edge_stds[2] = {};
    float desire_state[kDesireLen] = {};
    LeadState lead;
    PoseState pose;
    CalibrationState calibration;
    // 녹화 v6부터. plan의 yaw·yaw rate(laneless 모드가 openpilot 메인처럼 곡률을 만든다).
    float plan_yaw[kTrajectorySize] = {};
    float plan_yaw_rate[kTrajectorySize] = {};
    // 녹화 v7부터. 이 프레임 워프에 쓴 카메라 장착(웹 기기 설정, modeld가 천천히 옮긴 값).
    // 모델 점은 이만큼 오른쪽 가상 카메라 기준이라 HUD가 같은 값으로 되돌려 그린다.
    float camera_offset_m = 0.0f;
    float camera_height_m = 0.0f;
    // 녹화 v8부터. 운전자가 0, 2, …, 10초 뒤 가속·브레이크 페달을 밟고 있을 확률(모델 meta). 정차
    // 출발 알림이 2초 가속 확률로 녹색 신호를 잡는다(경로보다 빨리, 경로가 안 열려도 오른다).
    float gas_press_probs[kMetaPressHorizons] = {};
    float brake_press_probs[kMetaPressHorizons] = {};
    // 녹화 v10부터. 광류 보정기의 상태(v11부터 끝에 네 필드가 붙었다).
    FlowCalibrationState flow_calibration;
    // 녹화 v11부터. 워프 보정의 출처와 포즈 보정기 자신의 추정.
    CalibrationSourceState calibration_source;
    // 녹화 v12부터. 광류로 잰 앞차 확대율.
    LeadFlowState lead_flow;
    // 녹화 v13부터. 모델 앞차와 확대율을 섞은 앞차 추적.
    LeadTrackState lead_track;
};

// 이 크기가 녹화 ModelState 레코드의 페이로드 크기다. 바뀌면 기존 녹화를
// 읽는 tools/model/recording_reader.py와 어긋나므로 recording_format.h의
// kRecordingVersion도 함께 올려야 한다.
static_assert(sizeof(ModelState) == 3736 && offsetof(ModelState, plan_yaw) == 3256 &&
                  offsetof(ModelState, camera_offset_m) == 3520 && offsetof(ModelState, gas_press_probs) == 3528 &&
                  offsetof(ModelState, flow_calibration) == 3576 && offsetof(ModelState, calibration_source) == 3648 &&
                  offsetof(ModelState, lead_flow) == 3680 && offsetof(ModelState, lead_track) == 3712,
              "ModelState layout is shared with the recording reader");
// 웹 콘솔(web_console/state_layout.py MODEL_CALIBRATION_AT, MODEL_FLOW_CALIBRATION_AT,
// MODEL_CALIBRATION_SOURCE_AT)이 이 위치에서 보정 상태를 읽는다.
static_assert(offsetof(ModelState, calibration) == 3224 && sizeof(CalibrationState) == 32,
              "ModelState.calibration is read by the web console");
// 웹 BEV 탭(scripts/web_console/static/bev_data.js)이 위치로 읽는 필드. 서버는 페이로드를 그대로 보내고
// 웹 콘솔(web_console/state_layout.py MODEL_STATE_AT)이 페이지에 위치를 알려 준다(check_web_console.py가
// 대조).
#define EDGEPILOT_MODEL_STATE_AT(field, expected) \
    static_assert(offsetof(ModelState, field) == (expected), "ModelState." #field " moved: web BEV")
EDGEPILOT_MODEL_STATE_AT(capture_timestamp_ns, 8);
EDGEPILOT_MODEL_STATE_AT(model_timestamp_ns, 16);
EDGEPILOT_MODEL_STATE_AT(valid, 28);
EDGEPILOT_MODEL_STATE_AT(plan, 304);
EDGEPILOT_MODEL_STATE_AT(lanes, 700);
EDGEPILOT_MODEL_STATE_AT(lane_probabilities, 2284);
EDGEPILOT_MODEL_STATE_AT(road_edges, 2316);
EDGEPILOT_MODEL_STATE_AT(road_edge_stds, 3108);
EDGEPILOT_MODEL_STATE_AT(lead, 3148);
EDGEPILOT_MODEL_STATE_AT(gas_press_probs, 3528);
EDGEPILOT_MODEL_STATE_AT(lead_flow, 3680);
EDGEPILOT_MODEL_STATE_AT(lead_track, 3712);
#undef EDGEPILOT_MODEL_STATE_AT
static_assert(sizeof(IpcPoint) == 12 && sizeof(LeadState) == 24 && offsetof(LeadState, probability) == 4 &&
                  offsetof(LeadState, x) == 8 && offsetof(LeadState, velocity) == 16 &&
                  offsetof(LeadState, acceleration) == 20,
              "web BEV reads IpcPoint and LeadState by position");
static_assert(offsetof(LeadFlowState, valid) == 8 && offsetof(LeadFlowState, expansion_rate) == 20 &&
                  offsetof(LeadFlowState, lead_x) == 28,
              "web BEV reads LeadFlowState by position");
static_assert(offsetof(LeadTrackState, valid) == 0 && offsetof(LeadTrackState, distance_m) == 8 &&
                  offsetof(LeadTrackState, relative_speed_mps) == 12,
              "web BEV reads LeadTrackState by position");

// 차선선 플러그인(modeld 안, 플러그인이 있을 때만; docs/lane-line-plugin.md). 프레임마다 supercombo 뒤에 돈 결과다.
// 녹화에는 LaneState 레코드(9)로 그대로 남는다. 열거값(slot·shape·double_line·color·type)은
// model/lane_plugin_abi.h의 Ep* 값이다.
constexpr unsigned kLaneMaxLines = 6;
constexpr unsigned kLaneMaxBoundaries = 4;
constexpr unsigned kLaneMaxPoints = 72;
constexpr unsigned kLaneMaxRoadLines = kLaneMaxLines + kLaneMaxBoundaries;
// 도로면으로 옮긴 선의 점: ModelState 차선과 같은 거리 격자 X_IDXS의 앞쪽 17개(0~48 m).
constexpr unsigned kLaneRoadPoints = 17;
constexpr int8_t kLaneRoadKindLine = 0;      // LaneRoadLine.kind: 차선선(lines[index])
constexpr int8_t kLaneRoadKindBoundary = 1;  // 도로 경계(boundaries[index])
constexpr int8_t kLaneSlotEgoLeft = 0;       // 차선선 slot의 자차 왼쪽·오른쪽 선(EP_LANE_EGO_LEFT·RIGHT)
constexpr int8_t kLaneSlotEgoRight = 1;

// 점 하나. 원본 프레임 화소의 4배를 정수로 둔다(1/4 화소 단위).
struct LanePointState {
    int16_t x4 = 0;  // 차선선은 도색 가운데
    int16_t y4 = 0;
    int16_t width4 = 0;   // 도색 폭(가로). 도로 경계는 0
    int16_t edge_x4 = 0;  // 차로 쪽 안쪽 경계. 도로 경계는 x4와 같다
};
static_assert(sizeof(LanePointState) == 8, "LanePointState is recorded as-is");

struct LaneLineState {
    int8_t slot = 0;  // EpLaneSlot
    int8_t shape = 0;
    int8_t double_line = 0;
    int8_t color = 0;
    float confidence = 0.0f;
    uint32_t point_count = 0;
    LanePointState points[kLaneMaxPoints] = {};  // 가까운 쪽(영상 아래)부터
};
static_assert(sizeof(LaneLineState) == 588, "LaneLineState is recorded as-is");

struct LaneBoundaryState {
    int8_t slot = 0;  // EpBoundarySlot
    int8_t type = 0;  // EpBoundaryType
    int8_t reserved[2] = {};
    float confidence = 0.0f;
    uint32_t point_count = 0;
    LanePointState points[kLaneMaxPoints] = {};
};
static_assert(sizeof(LaneBoundaryState) == 588, "LaneBoundaryState is recorded as-is");

// 차선선 하나 또는 도로 경계 하나를 도로면으로 옮긴 것. 같은 프레임 supercombo가 낸 노면 높이에 투영하고(못 쓰면
// 평평한 도로면), 거리 X_IDXS[0..kLaneRoadPoints)에서 읽은 가로 위치(m, 오른쪽 +, ModelState 차선과 같은 가상 카메라
// 기준)다. 차선선은 차로 쪽 안쪽 경계, 도로 경계는 그 점이다. 측정은 first..last 격자를 덮고, 가장 가까운 측정이 8 m
// 안이면 그보다 가까운 격자(x = 0까지)는 4~20 m 점에 맞춘 2차식으로 늘린 값이다. 덮지 않는 점은 NaN이다.
struct LaneRoadLine {
    int8_t kind = kLaneRoadKindLine;
    int8_t index = 0;        // lines 또는 boundaries 안의 번호
    int8_t slot = 0;         // EpLaneSlot 또는 EpBoundarySlot
    int8_t type = 0;         // 차선선은 EpLaneShape, 도로 경계는 EpBoundaryType
    int8_t double_line = 0;  // 차선선만(EpLaneDouble)
    int8_t color = 0;        // 차선선만(EpLaneColor)
    int8_t first = -1;       // 측정이 덮은 첫·마지막 격자 번호. 측정이 없으면 -1
    int8_t last = -1;
    float confidence = 0.0f;
    float y[kLaneRoadPoints];
    LaneRoadLine() {
        for (float &v : y) v = std::numeric_limits<float>::quiet_NaN();
    }
};
static_assert(sizeof(LaneRoadLine) == 80, "LaneRoadLine is recorded as-is");

struct LaneState {
    uint64_t frame_id = 0;
    uint64_t capture_timestamp_ns = 0;  // ModelState와 같은 센서 캡처 시각
    uint64_t lane_timestamp_ns = 0;     // 결과를 낸 시각
    float execution_ms = 0.0f;          // VPP + NPU + 해석
    uint32_t valid = 0;                 // 1이면 이 프레임을 해석했다(0이면 아래 값이 없다)
    uint16_t image_width = 0;
    uint16_t image_height = 0;
    uint32_t line_count = 0;
    uint32_t boundary_count = 0;
    // 도로면 블록(헤더 바로 뒤라 웹 BEV는 앞쪽 kLaneStateRoadPrefixBytes만 보낸다). 선이 앞, 경계가 뒤다.
    uint32_t road_count = 0;
    LaneRoadLine road[kLaneMaxRoadLines];
    LaneLineState lines[kLaneMaxLines] = {};
    LaneBoundaryState boundaries[kLaneMaxBoundaries] = {};
    float vanishing_x = 0.0f;  // 원본 프레임 화소
    float vanishing_y = 0.0f;
    float vanishing_confidence = 0.0f;
    char plugin_name[32] = {};  // 플러그인이 알린 이름(모델·버전)
    uint32_t reserved = 0;      // 끝 맞춤(바이트가 늘 같도록 0)
};
// 녹화 LaneState 레코드의 페이로드다. tools/model/recording_reader.py LANE_STATE가 같은 배치를 읽는다.
static_assert(sizeof(LaneState) == 6776 && offsetof(LaneState, road_count) == 44 && offsetof(LaneState, road) == 48 &&
                  offsetof(LaneState, lines) == 848 && offsetof(LaneState, boundaries) == 4376 &&
                  offsetof(LaneState, vanishing_x) == 6728 && offsetof(LaneState, plugin_name) == 6740,
              "LaneState layout is shared with the recording reader");
// 웹 BEV가 받는 앞부분: 헤더와 도로면 블록
constexpr unsigned kLaneStateRoadPrefixBytes = offsetof(LaneState, lines);
// 웹 BEV(scripts/web_console/state_layout.py LANE_STATE_AT·LANE_ROAD_AT, LANE_STATE_PREFIX_SIZE)가 위치로 읽는
// 필드(check_web_console.py가 대조)
#define EDGEPILOT_LANE_STATE_AT(field, expected) \
    static_assert(offsetof(LaneState, field) == (expected), "LaneState." #field " moved: web BEV")
EDGEPILOT_LANE_STATE_AT(frame_id, 0);
EDGEPILOT_LANE_STATE_AT(capture_timestamp_ns, 8);
EDGEPILOT_LANE_STATE_AT(valid, 28);
EDGEPILOT_LANE_STATE_AT(road_count, 44);
EDGEPILOT_LANE_STATE_AT(road, 48);
#undef EDGEPILOT_LANE_STATE_AT
#define EDGEPILOT_LANE_ROAD_AT(field, expected) \
    static_assert(offsetof(LaneRoadLine, field) == (expected), "LaneRoadLine." #field " moved: web BEV")
EDGEPILOT_LANE_ROAD_AT(kind, 0);
EDGEPILOT_LANE_ROAD_AT(slot, 2);
EDGEPILOT_LANE_ROAD_AT(type, 3);
EDGEPILOT_LANE_ROAD_AT(double_line, 4);
EDGEPILOT_LANE_ROAD_AT(color, 5);
EDGEPILOT_LANE_ROAD_AT(confidence, 8);
EDGEPILOT_LANE_ROAD_AT(y, 12);
#undef EDGEPILOT_LANE_ROAD_AT
static_assert(kLaneStateRoadPrefixBytes == 848, "web BEV LANE_STATE_PREFIX_SIZE");

// 도로면 블록에서 slot 자차 선(같은 slot이 여럿이면 신뢰도가 높은 것), 없으면 nullptr. 플래너, HUD, 재생 도구가 같은
// 선을 고르도록 모두 이것을 쓴다(웹 BEV의 bev.js도 같은 규칙).
inline const LaneRoadLine *lane_ego_road_line(const LaneState &lane, int8_t slot) {
    const LaneRoadLine *best = nullptr;
    for (uint32_t i = 0; i < std::min<uint32_t>(lane.road_count, kLaneMaxRoadLines); ++i) {
        const LaneRoadLine &road = lane.road[i];
        if (road.kind == kLaneRoadKindLine && road.slot == slot && (!best || road.confidence > best->confidence))
            best = &road;
    }
    return best;
}
static_assert(std::is_trivially_copyable<LaneState>::value, "LaneState goes through shared memory as bytes");

struct ProcessState {
    char name[16] = {};
    uint32_t running = 0;
};

static_assert(sizeof(ProcessState) == 20, "ProcessState layout is shared with the Python manager");

struct ManagerState {
    uint64_t timestamp_ns = 0;
    uint32_t process_count = 0;
    uint32_t reserved = 0;
    ProcessState processes[kMaxProcesses] = {};
};

static_assert(sizeof(ManagerState) == 256, "ManagerState layout is shared with the Python manager");

struct IpcCanFrame {
    uint32_t address = 0;
    uint32_t src = 0;
    uint32_t bus_time = 0;
    uint32_t data_len = 0;
    uint32_t flags = 0;
    uint8_t data[64] = {};
};

struct CanBatch {
    uint64_t timestamp_ns = 0;
    uint32_t valid = 0;
    uint32_t count = 0;
    uint32_t dropped = 0;
    uint32_t reserved = 0;
    IpcCanFrame frames[kCanBatchMaxFrames] = {};
};

inline bool can_batch_is_fresh(const CanBatch &batch, uint64_t now_ns, uint64_t max_age_ns) {
    return batch.valid && timestamp_fresh_ns(batch.timestamp_ns, now_ns, max_age_ns);
}

// 프레임 목록을 배치 하나로 담는다. 256개를 넘는 꼬리는 dropped로 센다.
// fill(dst, src)이 프레임별 필드를 옮긴다. controlsd(CanFrame)와 pandad
// (PandaCanFrame)가 같은 껍데기를 쓴다.
template <class Frame, class Fill>
CanBatch make_can_batch(const std::vector<Frame> &frames, Fill fill) {
    CanBatch batch;
    batch.timestamp_ns = monotonic_now_ns();
    batch.valid = 1;
    batch.count = static_cast<uint32_t>(std::min<size_t>(frames.size(), kCanBatchMaxFrames));
    batch.dropped = static_cast<uint32_t>(frames.size() - batch.count);
    for (uint32_t i = 0; i < batch.count; ++i) fill(&batch.frames[i], frames[i]);
    return batch;
}

// wake·waiters는 pop_wait의 futex 깨움이다(IpcHeader와 같은 방식: push가 늘 깨우고, waiters는 예전 빌드 호환용).
struct CanQueueHeader {
    uint32_t magic = kCanQueueMagic;
    uint32_t version = kCanQueueVersion;
    uint32_t slot_count = kCanQueueSlots;
    std::atomic<uint32_t> wake{0};
    std::atomic<uint64_t> write_seq{0};
    std::atomic<uint64_t> read_seq{0};
    std::atomic<uint32_t> waiters{0};
    uint32_t reserved1 = 0;
    uint64_t reserved2 = 0;
};
static_assert(sizeof(CanQueueHeader) == 48 && offsetof(CanQueueHeader, wake) == 12 &&
                  offsetof(CanQueueHeader, write_seq) == 16 && offsetof(CanQueueHeader, waiters) == 32,
              "CanQueueHeader layout");

struct PandaState {
    uint64_t timestamp_ns = 0;
    uint32_t connected = 0;
    uint32_t comms_healthy = 0;
    uint32_t tx_enabled = 0;
    uint32_t controls_allowed = 0;
    uint32_t ignition_line = 0;
    uint32_t ignition_can = 0;
    uint32_t safety_mode = 0;
    uint32_t safety_param = 0;
    uint32_t panda_type = 0;
    uint32_t can_rx_errs = 0;
    uint32_t can_send_errs = 0;
    uint32_t can_fwd_errs = 0;
    uint32_t blocked_msg_cnt = 0;
    uint32_t heartbeat_lost = 0;
    uint32_t usb_tx_timeouts = 0;
    uint32_t usb_tx_retries = 0;
    uint32_t malformed_rx_batches = 0;
    uint32_t faults = 0;
    uint32_t fault_status = 0;
    uint32_t voltage = 0;
    uint32_t current = 0;
};

// ---- ControlState::hud_flags 비트 ----
constexpr uint32_t kHudFlagLaneless = 1U << 0;
constexpr uint32_t kHudFlagBrakeHold = 1U << 1;
constexpr uint32_t kHudFlagSoftDisabling = 1U << 2;        // 3초 뒤 해제 예고(active_block이 사유)
constexpr uint32_t kHudFlagSteerSaturated = 1U << 3;       // 커브가 조향 한계를 넘음
constexpr uint32_t kHudFlagLaneChangePending = 1U << 4;    // 차선 변경 대기: 운전자가 그쪽으로 핸들을 밀어야 시작
constexpr uint32_t kHudFlagLaneChanging = 1U << 5;         // 차선 변경 중(시작·마무리)
constexpr uint32_t kHudFlagLaneChangeRight = 1U << 6;      // 차선 변경 방향(없으면 왼쪽)
constexpr uint32_t kHudFlagSteerPaused = 1U << 7;          // 85도 위에서 조향 요청을 끄고 쉰다
constexpr uint32_t kHudFlagSteerPausedByDriver = 1U << 8;  // 운전자가 넘겨받음: 15도 아래 손 뗌 또는 계획 커브 1초
constexpr uint32_t kHudFlagBrakeLights = 1U << 9;  // 자차 브레이크등(페달 스트로크 또는 AUTO HOLD, brake_lights_on)
constexpr uint32_t kHudFlagLanebow = 1U << 10;     // 경로 모드가 Lanebow(LD로 조향 중인지는 Laneless 비트가 꺼졌는지)
// 피드포워드에 학습한 토크 모델이 섞임(docs/lateral-nn.md). 비트 11은 2026-10-09 전 녹화에서 브레이크등이었다.
constexpr uint32_t kHudFlagNnFeedforward = 1U << 12;

struct ControlState {
    uint64_t timestamp_ns = 0;
    uint32_t enabled = 0;
    uint32_t engaged = 0;
    uint32_t active = 0;
    uint32_t should_send = 0;
    uint32_t path_usable = 0;
    uint32_t seeds_ready = 0;
    uint32_t vehicle_fresh = 0;
    uint32_t steering_fault = 0;
    uint32_t left_blinker = 0;
    uint32_t right_blinker = 0;
    uint32_t cruise_active = 0;
    int32_t gear = 0;
    float cluster_speed_kph = 0.0f;
    float cruise_max_speed_kph = 0.0f;
    float cruise_command_speed_kph = 0.0f;
    float steering_angle_deg = 0.0f;
    float desired_curvature = 0.0f;
    float actual_curvature = 0.0f;
    float normalized_output = 0.0f;
    int32_t desired_torque = 0;
    int32_t apply_torque = 0;
    int32_t driver_torque = 0;
    uint32_t desire = 0;
    char active_block[32] = {};
    uint32_t radar_lead_valid = 0;
    float radar_lead_distance_m = 0.0f;
    float radar_lead_relative_speed_mps = 0.0f;
    uint32_t departure_alert_type = 0;
    uint32_t departure_alert_event_id = 0;
    uint32_t green_light_alert_armed = 0;
    uint32_t tpms_valid = 0;
    uint32_t tpms_unit = 0;
    float tpms_pressure_fl = 0.0f;
    float tpms_pressure_fr = 0.0f;
    float tpms_pressure_rl = 0.0f;
    float tpms_pressure_rr = 0.0f;
    uint32_t tpms_warning = 0;
    uint32_t hud_flags = 0;
    uint32_t engage_event_id = 0;
    uint32_t disengage_event_id = 0;
    uint32_t engage_reject_event_id = 0;
    char engage_reject_block[32] = {};
    float ego_speed_kph = 0.0f;
    // 이번 횡 계획이 쓴 차선선 플러그인 결과(LaneState)의 frame_id 아래 32비트. Lanebow 모드가 아니거나 쓴 것이
    // 없으면 0. 녹화의 LaneState 레코드와 맞춰 보는 데 쓴다(2026-10-15 전 녹화는 정렬 꼬리라 0).
    uint32_t lane_state_frame = 0;
};

// paramsd·torqued 출력(상류 vehicleParameters·lateralTorqueParameters). controlsd가
// paramsd 출력마다(20 Hz) 발행하고 recordd가 LearnerState로 저장한다.
constexpr uint32_t kLearnerVehicleInputsOk = 1U << 0;
constexpr uint32_t kLearnerVehicleValid = 1U << 1;
constexpr uint32_t kLearnerSensorValid = 1U << 2;
constexpr uint32_t kLearnerSteerRatioValid = 1U << 3;
constexpr uint32_t kLearnerStiffnessValid = 1U << 4;
constexpr uint32_t kLearnerOffsetAverageValid = 1U << 5;
constexpr uint32_t kLearnerOffsetValid = 1U << 6;
constexpr uint32_t kLearnerTorqueInputsOk = 1U << 7;
constexpr uint32_t kLearnerTorqueValid = 1U << 8;
constexpr uint32_t kLearnerUseVehicle = 1U << 9;  // 스위치 적용 후 실제 사용
constexpr uint32_t kLearnerUseTorque = 1U << 10;
constexpr uint32_t kLearnerVehicleRestored = 1U << 11;
constexpr uint32_t kLearnerTorqueRestored = 1U << 12;
constexpr uint32_t kLearnerUseDelay = 1U << 13;         // 경로 지연에 lagd 추정 사용 중
constexpr uint32_t kLearnerLocalizerInputs = 1U << 14;  // paramsd·torqued 입력이 locationd
// 학습 입력 출처가 바뀐 torqued 캐시에서 배율·마찰만 이어 썼다(점·절편은 새로 모은다)
constexpr uint32_t kLearnerTorquePartlyRestored = 1U << 15;
constexpr uint32_t kLearnerLateralNnLoaded = 1U << 16;  // controlsd가 학습한 토크 모델 파일을 읽었다

struct LearnerState {
    uint64_t timestamp_ns = 0;
    uint32_t flags = 0;
    float steer_ratio = 0.0f;
    float stiffness_factor = 0.0f;
    float roll_rad = 0.0f;
    float angle_offset_average_deg = 0.0f;
    float angle_offset_deg = 0.0f;
    float steer_ratio_std = 0.0f;
    float stiffness_factor_std = 0.0f;
    float angle_offset_average_std = 0.0f;
    float angle_offset_fast_std = 0.0f;
    float yaw_bias_rad_s = 0.0f;
    float lat_accel_factor_raw = 0.0f;
    float lat_accel_offset_raw = 0.0f;
    float friction_raw = 0.0f;
    float lat_accel_factor = 0.0f;
    float lat_accel_offset = 0.0f;
    float friction = 0.0f;
    float decay = 0.0f;
    float max_resets = 0.0f;
    int32_t total_bucket_points = 0;
    int32_t cal_perc = 0;
    float road_bank_lat_accel = 0.0f;  // 기존 편경사 추정(비교용)
    // 학습기 사전값. controlsd 시작 때 파라미터에서 고정된다.
    float prior_steer_ratio = 0.0f;
    float prior_lat_accel_factor = 0.0f;
    float prior_friction = 0.0f;
    int16_t bucket_points[8] = {};
    float plan_delay_s = 0.0f;  // 실제 쓴 경로 지연(lagd 사용 중이면 추정값). 예전 기록은 0
};
// recordd가 그대로 저장하고 tools/model/recording_reader.py LEARNER_STATE와
// 웹 콘솔(web_console/learner_monitor.py LEARNER_FIELDS)이 위치로 읽는다(check_web_console.py가 대조).
#define EDGEPILOT_LEARNER_STATE_AT(field, expected) \
    static_assert(offsetof(LearnerState, field) == (expected), "LearnerState." #field " moved")
EDGEPILOT_LEARNER_STATE_AT(flags, 8);
EDGEPILOT_LEARNER_STATE_AT(steer_ratio, 12);
EDGEPILOT_LEARNER_STATE_AT(yaw_bias_rad_s, 48);
EDGEPILOT_LEARNER_STATE_AT(lat_accel_factor_raw, 52);
EDGEPILOT_LEARNER_STATE_AT(max_resets, 80);
EDGEPILOT_LEARNER_STATE_AT(total_bucket_points, 84);
EDGEPILOT_LEARNER_STATE_AT(road_bank_lat_accel, 92);
EDGEPILOT_LEARNER_STATE_AT(prior_steer_ratio, 96);
EDGEPILOT_LEARNER_STATE_AT(bucket_points, 108);
EDGEPILOT_LEARNER_STATE_AT(plan_delay_s, 124);
#undef EDGEPILOT_LEARNER_STATE_AT
static_assert(sizeof(LearnerState) == 128, "LearnerState size");

// 보드 IMU(LSM6DSOW, i2c-1 0x6B) 원시 샘플. imud가 104 Hz로 읽어 100 ms마다 묶어 발행하고
// recordd가 RecordType::Imu로 그대로 남긴다. 축은 칩 좌표(보정·회전 없음)이고 자이로
// 바이어스도 빼지 않는다: 차량/카메라 축과의 관계와 바이어스는 분석에서 구한다.
struct ImuSample {
    uint64_t timestamp_ns = 0;  // CLOCK_BOOTTIME, 데이터 준비를 본 시각
    float accel_mps2[3] = {};
    float gyro_rad_s[3] = {};
    float temperature_c = 0.0f;
    uint32_t reserved = 0;
};

static_assert(sizeof(ImuSample) == 40, "ImuSample layout is shared with the recording reader");

constexpr unsigned kImuBatchMaxSamples = 16;

struct ImuBatch {
    uint64_t timestamp_ns = 0;  // 발행 시각
    uint32_t count = 0;
    uint32_t dropped = 0;  // 누적: 읽기 전에 덮어쓰인 샘플(데이터 준비 두 번 이상 놓침)
    ImuSample samples[kImuBatchMaxSamples] = {};
};

static_assert(sizeof(ImuBatch) == 16 + 40 * kImuBatchMaxSamples, "ImuBatch layout is shared with the recording reader");

// locationd 출력: 자세 칼만 필터(상류 livePose)와 조향 지연 추정(상류 liveDelay).
// IMU 묶음마다(약 20 Hz) 발행하고 recordd가 RecordType::Localization으로 그대로 남긴다.
// 보정(차량) 좌표계는 x 앞, y 오른쪽, z 아래라 요레이트는 오른쪽 회전이 양수(곡률 관례와 같다).
constexpr uint32_t kLocalizationFilterValid = 1U << 0;
constexpr uint32_t kLocalizationInputsOk = 1U << 1;
constexpr uint32_t kLocalizationSensorsOk = 1U << 2;
constexpr uint32_t kLocalizationPosenetOk = 1U << 3;
constexpr uint32_t kLocalizationCalibValid = 1U << 4;
constexpr uint32_t kLocalizationLagRestored = 1U << 5;
// input_flags: 거부 누적이 한도를 넘은 입력(inputs_ok를 떨어뜨린 것)과 차속 가드
constexpr uint32_t kLocalizationInvalidAccel = 1U << 0;
constexpr uint32_t kLocalizationInvalidGyro = 1U << 1;
constexpr uint32_t kLocalizationInvalidCamera = 1U << 2;
constexpr uint32_t kLocalizationCameraGuarded = 1U << 3;

struct LocalizationState {
    uint64_t timestamp_ns = 0;  // 추정 시각 = 마지막 IMU 샘플 시각(CLOCK_BOOTTIME)
    uint32_t flags = 0;
    uint32_t lag_status = 0;               // LateralLagStatus: 0 미추정, 1 추정, 2 무효
    float orientation_calib[3] = {};       // roll(오른쪽 아래 +), pitch, yaw
    float orientation_std[3] = {};         // 기기 좌표계 표준편차
    float angular_velocity_calib[3] = {};  // rad/s
    float angular_velocity_calib_std[3] = {};
    float velocity_device[3] = {};  // m/s
    float velocity_device_std[3] = {};
    float acceleration_calib[3] = {};  // m/s², 중력 제외
    float lateral_delay_s = 0.0f;      // 쓸 지연: 추정되면 추정, 아니면 초기값
    float lag_estimate_s = 0.0f;       // 진행 중 블록 포함 평균
    float lag_estimate_std_s = 0.0f;
    int32_t lag_valid_blocks = 0;
    int32_t lag_cal_perc = 0;
    uint32_t lag_points = 0;   // 창 안의 조건 만족 점 수
    uint32_t input_flags = 0;  // kLocalizationInvalid*, kLocalizationCameraGuarded
};
static_assert(sizeof(LocalizationState) == 128, "LocalizationState layout is shared with the recording reader");

// RecordState::storage_blocked: 녹화를 거부했거나 멈춘 까닭(0이면 막히지 않음)
constexpr uint32_t kStorageBlockedSdReserve = 1;  // SD 여유가 예비분(5 GiB와 10%) 밑
constexpr uint32_t kStorageBlockedStaging = 2;    // SD가 기록을 따라오지 못해 tmpfs 스테이징이 참

// recordd가 0.5초마다 발행하고 overlayd가 녹화 표시(REC)에 쓴다.
struct RecordState {
    uint64_t timestamp_ns = 0;
    uint32_t active = 0;           // route를 쓰는 중
    uint32_t storage_blocked = 0;  // kStorageBlocked*: 저장 공간 때문에 녹화를 거부했거나 멈춤
};

// controlsd가 발행하고 overlayd/recordd가 읽는 공유 레이아웃이다. 기록 v5는 이
// 구조체를 그대로 저장하고 tools/model/recording_reader.py가 위치로 디코드하므로 필드
// 순서까지 전부 고정한다. 같은 크기 필드 둘을 맞바꿔도 여기서 걸린다.
#define EDGEPILOT_CONTROL_STATE_AT(field, expected) \
    static_assert(offsetof(ControlState, field) == (expected), "ControlState." #field " moved: recording v5 layout")
EDGEPILOT_CONTROL_STATE_AT(timestamp_ns, 0);
EDGEPILOT_CONTROL_STATE_AT(enabled, 8);
EDGEPILOT_CONTROL_STATE_AT(engaged, 12);
EDGEPILOT_CONTROL_STATE_AT(active, 16);
EDGEPILOT_CONTROL_STATE_AT(should_send, 20);
EDGEPILOT_CONTROL_STATE_AT(path_usable, 24);
EDGEPILOT_CONTROL_STATE_AT(seeds_ready, 28);
EDGEPILOT_CONTROL_STATE_AT(vehicle_fresh, 32);
EDGEPILOT_CONTROL_STATE_AT(steering_fault, 36);
EDGEPILOT_CONTROL_STATE_AT(left_blinker, 40);
EDGEPILOT_CONTROL_STATE_AT(right_blinker, 44);
EDGEPILOT_CONTROL_STATE_AT(cruise_active, 48);
EDGEPILOT_CONTROL_STATE_AT(gear, 52);
EDGEPILOT_CONTROL_STATE_AT(cluster_speed_kph, 56);
EDGEPILOT_CONTROL_STATE_AT(cruise_max_speed_kph, 60);
EDGEPILOT_CONTROL_STATE_AT(cruise_command_speed_kph, 64);
EDGEPILOT_CONTROL_STATE_AT(steering_angle_deg, 68);
EDGEPILOT_CONTROL_STATE_AT(desired_curvature, 72);
EDGEPILOT_CONTROL_STATE_AT(actual_curvature, 76);
EDGEPILOT_CONTROL_STATE_AT(normalized_output, 80);
EDGEPILOT_CONTROL_STATE_AT(desired_torque, 84);
EDGEPILOT_CONTROL_STATE_AT(apply_torque, 88);
EDGEPILOT_CONTROL_STATE_AT(driver_torque, 92);
EDGEPILOT_CONTROL_STATE_AT(desire, 96);
EDGEPILOT_CONTROL_STATE_AT(active_block, 100);
EDGEPILOT_CONTROL_STATE_AT(radar_lead_valid, 132);
EDGEPILOT_CONTROL_STATE_AT(radar_lead_distance_m, 136);
EDGEPILOT_CONTROL_STATE_AT(radar_lead_relative_speed_mps, 140);
EDGEPILOT_CONTROL_STATE_AT(departure_alert_type, 144);
EDGEPILOT_CONTROL_STATE_AT(departure_alert_event_id, 148);
EDGEPILOT_CONTROL_STATE_AT(green_light_alert_armed, 152);
EDGEPILOT_CONTROL_STATE_AT(tpms_valid, 156);
EDGEPILOT_CONTROL_STATE_AT(tpms_unit, 160);
EDGEPILOT_CONTROL_STATE_AT(tpms_pressure_fl, 164);
EDGEPILOT_CONTROL_STATE_AT(tpms_pressure_fr, 168);
EDGEPILOT_CONTROL_STATE_AT(tpms_pressure_rl, 172);
EDGEPILOT_CONTROL_STATE_AT(tpms_pressure_rr, 176);
EDGEPILOT_CONTROL_STATE_AT(tpms_warning, 180);
EDGEPILOT_CONTROL_STATE_AT(hud_flags, 184);
EDGEPILOT_CONTROL_STATE_AT(engage_event_id, 188);
EDGEPILOT_CONTROL_STATE_AT(disengage_event_id, 192);
EDGEPILOT_CONTROL_STATE_AT(engage_reject_event_id, 196);
EDGEPILOT_CONTROL_STATE_AT(engage_reject_block, 200);
EDGEPILOT_CONTROL_STATE_AT(ego_speed_kph, 232);
EDGEPILOT_CONTROL_STATE_AT(lane_state_frame, 236);
#undef EDGEPILOT_CONTROL_STATE_AT
static_assert(sizeof(ControlState) == 240, "ControlState layout is shared by controlsd, overlayd and the recording");
// recordd가 RecordType::PandaState로 그대로 저장한다.
static_assert(sizeof(PandaState) == 96, "PandaState is recorded as-is: bump kRecordingVersion");

#endif
