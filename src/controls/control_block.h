#pragma once

// 결합·조향 차단 사유. 컨트롤러가 정하고, ControlState에는 와이어 이름(문자열)으로
// 실려 녹화·Python 리더·HUD가 읽는다. 행 하나가 열거값·와이어 이름·HUD 라벨·분류라서
// 사유를 더할 때 다른 곳을 고칠 필요가 없다.

#include <cstddef>
#include <cstdint>
#include <cstring>

enum class BlockKind : uint8_t {
    None,    // 차단 없음: active
    Reject,  // 정적 gate: 결합 요청은 실패로 처리, 표시만 한다
    Hard,    // 차량 결함: 결합 중이면 즉시 해제한다
    // Panda의 controls_allowed는 비동기로 보고된다(브리지가 100 Hz 컨트롤러보다
    // 낮은 주기로 health를 폴링한다). 대응하는 허가보다 SET 해제가 한두 틱 먼저
    // 올 수 있어, handshake가 끝날 때까지 결합을 유지한다. 실패가 아닌 대기다.
    Transient,
    // 고장이 아니라 가용성 상태다. openpilot처럼 결합은 허용하고 조향만 쉰다.
    // 정차에서는 plan 도달거리가 짧아 path가 상시 무효라, 거부로 치면 정차 중
    // 결합할 수 없게 된다.
    Availability,
    // openpilot의 SOFT_DISABLE. 결합 요청은 거부하고, 결합 중에 생기면 경고를 띄운 채
    // 3초 더 조향한 뒤 해제한다(운전자가 넘겨받을 시간). 목록 맨 끝에서만 판정하므로
    // 앞선 차단 사유를 가리지 않는다. 3초는 앞선 사유가 끼어도 이 사유가 이어지는 동안 센다.
    SoftDisable,
    // 입력이 끊겼다: 차량 CAN, 모델, 계획. 결합 요청은 거부한다. 결합 중에 생기면 바로 조향을 쉬고(토크 0,
    // 요청 비트는 유지) 3초를 센다(0.5초가 넘으면 해제 예고를 띄운다). 그 안에 돌아오면 다시 조향하고, 넘으면
    // 해제한다.
    // 낡은 입력으로 3초를 조향하지 않는 것이 SoftDisable과 다르다. 상류는 같은 경우(commIssue,
    // canBusMissing) 해제한다. 결합 중 Panda 허가가 빠진 Transient(핸드셰이크가 아닐 때)도 이렇게 다룬다.
    Interrupt,
};

// ControlStale은 컨트롤러가 내지 않는다. 스냅샷이 오래됐을 때 overlayd가 대신 쓴다.
#define EDGEPILOT_BLOCK_REASONS(X)                                                                    \
    X(None, "", "", BlockKind::None)                                                                  \
    X(NotEngaged, "not_engaged", "STANDBY", BlockKind::Reject)                                        \
    X(ControllerDisabled, "controller_disabled", "CONTROL OFF", BlockKind::Hard)                      \
    X(SeedsMissing, "seeds_missing", "CAN SEEDS", BlockKind::Interrupt)                               \
    X(VehicleStateStale, "vehicle_state_stale", "CAR STALE", BlockKind::Interrupt)                    \
    X(DoorOpen, "door_open", "DOOR OPEN", BlockKind::Hard)                                            \
    X(SeatbeltUnlatched, "seatbelt_unlatched", "SEATBELT", BlockKind::Hard)                           \
    X(EspDisabled, "esp_disabled", "ESP OFF", BlockKind::Hard)                                        \
    X(ParkBrake, "park_brake", "PARK BRAKE", BlockKind::Hard)                                         \
    X(BrakeError, "brake_error", "BRAKE ERROR", BlockKind::Hard)                                      \
    X(GearNotDrive, "gear_not_drive", "GEAR NOT D", BlockKind::Hard)                                  \
    X(PandaNotReady, "panda_not_ready", "PANDA NOT READY", BlockKind::Transient)                      \
    X(PandaControlsOff, "panda_controls_off", "PANDA CTRL OFF", BlockKind::Transient)                 \
    X(Stopped, "stopped", "STOPPED", BlockKind::Availability)                                         \
    X(PathInvalid, "path_invalid", "PATH INVALID", BlockKind::Availability)                           \
    X(ModelLost, "model_lost", "MODEL LOST", BlockKind::Interrupt)                                    \
    X(LateralPlanInvalid, "lateral_plan_invalid", "PLAN INVALID", BlockKind::Interrupt)               \
    X(LateralPlanStale, "lateral_plan_stale", "PLAN STALE", BlockKind::Interrupt)                     \
    X(SteerTempUnavailable, "steer_temp_unavailable", "STEER UNAVAILABLE", BlockKind::SoftDisable)    \
    X(CalibrationIncomplete, "calibration_incomplete", "CALIBRATING", BlockKind::SoftDisable)         \
    X(CalibrationRecalibrating, "calibration_recalibrating", "RECALIBRATING", BlockKind::SoftDisable) \
    X(CalibrationInvalid, "calibration_invalid", "CALIB INVALID", BlockKind::SoftDisable)             \
    X(ParamsdInvalid, "paramsd_invalid", "PARAMS INVALID", BlockKind::SoftDisable)                    \
    X(ControlStale, "control_stale", "CONTROL STALE", BlockKind::Reject)

enum class BlockReason : uint8_t {
#define EDGEPILOT_BLOCK_ENUM(name, wire, label, kind) name,
    EDGEPILOT_BLOCK_REASONS(EDGEPILOT_BLOCK_ENUM)
#undef EDGEPILOT_BLOCK_ENUM
        Count
};

struct BlockReasonRow {
    BlockReason reason;
    const char *name;   // 와이어 이름. ControlState::active_block[32]에 들어간다.
    const char *label;  // HUD 라벨
    BlockKind kind;
};

constexpr BlockReasonRow kBlockReasons[] = {
#define EDGEPILOT_BLOCK_ROW(name, wire, label, kind) {BlockReason::name, wire, label, kind},
    EDGEPILOT_BLOCK_REASONS(EDGEPILOT_BLOCK_ROW)
#undef EDGEPILOT_BLOCK_ROW
};
static_assert(sizeof(kBlockReasons) / sizeof(kBlockReasons[0]) == static_cast<size_t>(BlockReason::Count),
              "one row per BlockReason");

constexpr const char *block_reason_name(BlockReason reason) { return kBlockReasons[static_cast<size_t>(reason)].name; }
constexpr const char *block_reason_label(BlockReason reason) {
    return kBlockReasons[static_cast<size_t>(reason)].label;
}
constexpr BlockKind block_kind(BlockReason reason) { return kBlockReasons[static_cast<size_t>(reason)].kind; }

// 와이어 이름 → 열거값. 모르는 이름(다른 빌드의 controlsd)이면 Count.
inline BlockReason block_reason_from_name(const char *name) {
    for (const BlockReasonRow &row : kBlockReasons) {
        if (std::strcmp(row.name, name) == 0) return row.reason;
    }
    return BlockReason::Count;
}
