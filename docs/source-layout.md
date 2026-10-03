# Source layout

[← Documentation index](../README.md)

## Configuration and input

- `src/app_config.*`
  - parses the small runtime option set once at startup, and holds the
    MaixCAM2 camera intrinsics, the capture size, and the 4:3 preview crop
    shared by `overlayd` and `projection`.
- `src/replay_source.*`
  - reads `SCNV12R1` replay files into `Nv12Frame` for `modeld` replay
    mode. POSIX only.

## MaixCAM2 platform

`platform/maixcam2/` keeps the AX and MaixCDK headers out of `src/`. It builds
only in the board build, against `deps/ax630` from
`scripts/fetch_maixcam2_sdk.sh`.

- `maix_camera.*`
  - opens the camera (VI) through `libmaixcam_lib`'s `ax_middleware` classes,
    AI-ISP always on, sensor at the requested fps with auto exposure and a capped
    shutter, and copies each frame into a CMM block by IVPS TDP.
- `maix_display.*`
  - the LCD as two VO layers: layer 0 takes a CMM frame, IVPS crops the centre
    4:3 and scales it to 640x480; layer 1 is a cached CMM BGRA block the HUD is
    drawn into and pushed without a copy. VO rotates for the 480x640 panel and
    applies the board's flip/mirror. Also turns the backlight on.
- `maix_touch.*`
  - the touchscreen (`hyn_ts`, multi-touch type B) read without blocking;
    reports short taps in screen coordinates, rotated clockwise 90° as
    MaixCDK's `maix_touchscreen_maixcam2.hpp` does.
- `maix_cmm.*`
  - physically contiguous CMM blocks shared across processes (allocate in one,
    map by physical address in another, uncached).
- `maix_gdc_warp.*`
  - the model input warp on the IVPS GDC (`AX_IVPS_Dewarp`, perspective): two
    512x256 views from a physical source address, unpacked to YUV6. Uses only
    MSP SDK headers, not the middleware, so the NPU process initialises AX SYS
    once.
- `maix_shim.cc`, `stub/`
  - the few MaixCDK runtime pieces (log, err, board config lookup) and Kconfig
    stubs that the inline code in `ax_middleware.hpp` references but
    `libmaixcam_lib` does not export.

## Perception

- `src/model_output.*`
  - owns the openpilot master supercombo raw-output layout
    (`model_output_layout`, every block offset with a `static_assert`) and
    exposes parsed plan, lanes, road edges, leads, and pose. Also owns the
    shared `T_IDXS`/`X_IDXS` trajectory grids.
- `src/model_temporal.h`
  - the history queues the NPU core does not carry: 100-tick desire pulses
    pooled to 25x8, 96 ticks of hidden state strided to 24x512, and the
    5-frame image history per tower. No engine dependency, so
    `gtest_model_output_parser` pins the convention on the host.
- `src/model_input_transform.*`
  - the CPU input warp: direct `NV12 -> calibrated warped YUV6`, fusing
    homography sampling and YUV6 packing through a compact fixed-point LUT.
    Also produces the projection matrices the GDC warp uses.
- `src/supercombo_model.*`
  - loads the axmodel, enforces the input/output contract, runs the GDC (or
    CPU) warp into the image histories, fills the temporal inputs, and runs one
    frame. `run_frame_phys` reads a ring slot by physical address and drops the
    frame if it was overwritten during the warp.
- `src/ax_engine_session.*`, `src/ax_engine_api.h`
  - a minimal `libax_engine` session with a cached CMM buffer per tensor. The
    board image ships no engine headers, so `ax_engine_api.h` declares the API.
    The only files under `src/` that touch the NPU.
- `src/calibration_service.*`, `src/calibration_online.*`
  - wrap pose-based online calibration, manual override, projection policy, and
    the model-input calibration feedback loop.
- `src/projection.*`
  - converts model road coordinates through the openpilot-style `view_from_calib`
    matrix onto the display, using the target's real width and the same 4:3
    preview crop as the video layer.

## Planning and control

- `src/lateral_planner.*`
  - applies openpilot lane probability/width logic, lane-change state, and the
    lateral MPC in `src/lateral_mpc.*` to produce curvature targets. This is the
    only producer of `LateralTarget`.
- `src/lateral_mpc.*`
  - the lateral MPC itself: one Gauss-Newton SQP iteration per call over the
    openpilot 0.8.16 OCP, solved by a backward Riccati recursion. No external
    solver. See [Verification](verification.md#lateral-mpc-solver).
- `src/lateral_target.h`
  - declares `LateralTarget`, the planner-to-controller interface.
- `src/lateral_controller.*`, `src/lateral_torque.*`,
  `src/control_params.*`, `src/hyundai_can.*`
  - apply the planner's lag-adjusted curvature through the validated K7
    torque/CAN path.
- `src/vehicle_params_learner.*`, `src/torque_estimator.*`,
  `src/lateral_learners.*`, `src/localizer_inputs.h`
  - the paramsd/torqued ports that estimate steer ratio and torque response
    while driving (opt-in). `vehicle_params_learner` is paramsd with its
    car_kf EKF and saved-value restore; `torque_estimator` is torqued with its
    cache; `lateral_learners` is the controlsd glue that turns vehicle CAN and
    locationd samples into learner inputs and collects the outputs as
    `LiveLateralParams`. `localizer_inputs.h` converts the IPC
    `LocalizationState` into a learner sample, so the learner library does not
    depend on the IPC layout.
- `src/lateral_path.*`
  - reduces `modelState` to the steering-usability gate (reach and point
    count). It computes no path geometry; curvature comes from the MPC.
- `src/adaptive_cruise.*`, `src/departure_alert.*`
  - vision cruise setpoint control and departure alerting.
- `src/can_frame.h`, `src/vehicle_can.*`, `src/hyundai_can.*`
  - `can_frame.h` holds the transport type and the K7 YG HEV address/bus table;
    `vehicle_can` decodes received frames into vehicle state, `hyundai_can`
    encodes LKAS11/CLU11/MDPS12 commands.
- `src/control_block.h`
  - the engage/steer block reasons as one table: enum, wire name, HUD label,
    and kind (reject / hard disengage / transient Panda handshake /
    availability). The controller decides in `BlockReason`, `ControlState`
    carries the wire name so recordings and the Python readers stay text, and
    `overlay_state` labels it from the same rows. `gtest_overlay_state` proves
    every reason has a label.

### Control safety holds

`src/control_holds.*` implements both holds as `PandaHealthGate` and
`PathHoldGate`; `gtest_control_holds` exercises their boundaries.
`controlsd` tolerates a single malformed plan frame by holding the last
usable path for at most 150 ms; the normal 250 ms model freshness timeout remains
a hard safety gate, so a stale or invalid model still removes control. A
transient Panda health-snapshot gap is similarly limited to 100 ms; a fresh,
transport-ready `controls_allowed=0` is never held. Other health faults are
released after that short hold if they persist.

## Processes and IPC

- `src/ipc_messages.*`
  - every message that crosses `/dev/shm`: topic names, magics, channel headers,
    the `K230*State` snapshots with their `static_assert`s, and the
    `ModelState` → `ParsedModelOutput`/`ProjectionState` unpacking. Recording
    v5 stores `ModelState`, `ControlState`, and `PandaState` as-is, so their
    offsets are pinned here and tied to `kRecordingVersion`. Code that only
    reads or fills a message includes this and nothing else.
- `src/model_state_fill.h`
  - `fill_model_state`, which modeld calls to pack a frame's outputs into
    `ModelState`. It needs the online calibrator's snapshot, so it is kept out
    of `ipc_messages.h` and message consumers do not pull in the calibrator.
- `src/ipc_channels.*`
  - the `/dev/shm` channel implementations: latest-message channel, CAN queue,
    and the camera frame ring, all on one `ShmRegion` (open, size, map, close).
    The frame ring (version 5) keeps only its header in shm; the slots are
    camerad's CMM blocks, listed by physical address, each with a seqlock that
    hardware readers check before and after reading.
- `src/camerad.cc`, `src/modeld.cc`, `src/overlayd.cc`
  - openpilot-style process split: capture into the ring, model, and the
    two-layer LCD HUD.
- `src/overlay_renderer.*`, `src/overlay_canvas.*`, `src/overlay_font.*`
  - draw the 640x480 HUD into the straight-alpha BGRA buffer of VO layer 1:
    state border, speed and set speed, steering mode, plan/lane/road-edge
    ribbons faded with distance, lead chevron, torque bar, alerts, TPMS and
    camera calibration cards, a recording/Wi-Fi status pill (and the network
    card a tap opens), and chips that appear only when something needs
    attention (temperature, panda, storage). The remaining numeric diagnostics
    sit in a card behind the `hud_debug` device setting. `overlay_canvas`
    fills polygons with 4-subrow anti-aliasing that touches only covered spans,
    fills the straight rows of integer rounded rectangles directly, blends
    without divisions, and records the 64 px tiles each row touched so the
    next use of the same buffer clears only those; `overlay_font` holds the
    glyphs that `tools/ui/make_hud_font.py` bakes from Pillow's Aileron (CC0).
    No OpenCV, so `gtest_overlay_canvas` and `hud_snapshot` run on the host.
    The renderer keeps only coverage scratch and the per-buffer tiles; the
    turn-signal phase and the network card toggle come from `overlayd`.
- `src/overlay_state.*`
  - `OverlayHudState`, the `K230*State` → `OverlayHudState` mapping shared by
    `overlayd` and `hud_snapshot`, the engage-block label table, and
    `OverlayAlertEvents`, which turns the controlsd event counters into the one
    toast/log alert a frame may raise (baseline on first sight, rebaseline on a
    controlsd restart, reject > engage > disengage > departure).
    `gtest_overlay_state` pins all of it on the host.
- `src/alert_tones.*`, `src/alert_sound.*`
  - the alert sounds: `alert_tones` synthesises them (overlapping bell-like
    notes with soft attacks and decaying overtones; portable, so
    `alert_sound_preview` writes them as WAV and `gtest_alert_tones` checks
    them on the host), and `alert_sound` streams them to one long-lived
    `aplay` on the board speaker.
- `src/system_monitor.*`
  - `/proc`, thermal-zone, and network sampling (the Wi-Fi SSID through the
    `SIOCGIWESSID` ioctl) into `OverlayHudState`, called at 1 Hz by `overlayd`.
- `src/recording_writer.*`, `src/recording_format.h`
  - the event-log writer and on-disk contract of the K230 recorder, kept for
    the recorder port and for the host tools that read K230 drives.
    `gtest_recording_writer` pins the layout; `recording_format.h`
    (`kRecordingVersion`, the `K230LOG1` / `K230IDX1` headers, record types)
    is mirrored by `tools/model/recording_reader.py`. No process uses it on the
    MaixCAM2 yet.
- `src/event_log_reader.h`
  - the one C++ reader of `events/NNN.bin`, shared by `replayd` and the replay
    and dataset tools. It checks the magic, skips `header_size`, and stops
    without resyncing at a truncated tail (record type out of range, a payload
    over 1 MiB, or a short header or payload), which `truncated()` reports.
- `src/panda_client.*`, `src/panda_can_codec.*`, `src/pandad.cc`
  - optional panda USB bridge. It handles USB, health, heartbeat, receive CAN,
    and the final TX gate, but does not generate vehicle control messages.
- `src/controlsd.cc`
  - standalone K7 YG HEV lateral controller using the validated Hyundai CAN bus
    split, torque limits, counters, checksums, 60 kph MDPS helper, and a 20 Hz
    planner worker separated from the 100 Hz control loop.
- `scripts/manager.py`
  - minimal supervisor and heartbeat publisher. It is intentionally not a full
    openpilot manager clone. It stops the stock launcher, switches USB-C to host
    for the Panda, and one table in start order decides which processes run
    (`EDGEPILOT_ENABLE_CONTROL`, `EDGEPILOT_ENABLE_PANDA`, `EDGEPILOT_ENABLE_PARAM_SERVER`) and
    with what nice value.
- `scripts/param_server.py`, `scripts/display_control.py`
  - the FastAPI parameter editor (`EDGEPILOT_ENABLE_PARAM_SERVER`) and the
    MaixCAM2 backlight helper it calls (PWM3).

## Scripts and tools

- `scripts/fetch_maixcam2_sdk.sh`, `tools/docker_ax630/`,
  `scripts/upload_to_board.sh`, `scripts/run_host_tests.sh`
  - pinned SDK and board-library fetch, the arm64 build container, deploy, and
    host tests. See `scripts/README.md` and
    [Build and deploy](build-and-deploy.md).
- `tools/model/axmodel/`
  - the openpilot master → axmodel pipeline (core extraction, calibration data,
    Pulsar2 config).
- `tools/model/`
  - the recording readers and the v0.9.4-era model helpers:
    `recording_reader.py` decodes `recordd` routes (frame index, event log,
    H.264 from MaixCAM2 or HEVC from K230) and is the one Python mirror of `recording_format.h` /
    `ipc_messages.h`; `lane_bias.py`, `route_frames.py`, `make_replay.py`, and
    `make_calibration.py` build on it. See `tools/model/README.md`.
- `tools/calib/`
  - camera calibration: board-side chessboard capture, the PC-side solver, the
    chessboard image, the resulting MaixCAM2 intrinsics, and the model-view
    preview.
- `tools/control/`
  - `fit_lateral_params.py` (torque regression and actuator-lag estimate from
    drives) and `export_can_fixture.py` (recorded CAN → `gtest_control_replay`
    fixture).
- `tools/ui/hud_tools.py`
  - extracts `hud_snapshot` inputs from a route and composes its frames.
- `gtest/`
  - host unit tests (`gtest_*.cc`, googletest + CTest), one self-contained file
    per target, each registered by one `add_host_test(<name> <libraries>)` line in
    `gtest/CMakeLists.txt`; `scripts/run_host_tests.sh` runs them. See
    `gtest/README.md`.
- `diagnostics/`
  - replay and HUD tools (built by `diagnostics/CMakeLists.txt`), and the
    Python `check_param_server.py`; see `diagnostics/README.md`.

## Shared helpers

- `src/utils_process.h`
  - what a process gets from the OS: environment variables (`env_flag` is the
    one boolean convention), the `params/` directory path, and the
    SIGINT/SIGTERM → stop-flag hookup used by every `*d` main.
- `src/utils_math.h`
  - clamping, openpilot `interp`, degree/radian conversion.
- `src/utils_time.h`
  - `monotonic_now_ns` (`CLOCK_BOOTTIME`), the clock behind every timestamp that
    crosses a process boundary, and the freshness predicates for ns and
    CAN-seconds timestamps. Per-process scheduling may still use
    `std::chrono::steady_clock`.
- `src/utils_json.*`
  - minimal JSON value readers, the clamped `parse_json_optional_*` helpers, and
    the `Json*Field` tables that `control_params` and `adaptive_cruise` fill
    their structs from: one `{key, min, max, member}` row per parameter.
