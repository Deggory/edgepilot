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
  - the LCD: VO video layer 0 takes a CMM frame, IVPS crops the centre 4:3 and
    scales it to 640x480, and VO rotates it for the 480x640 panel with the
    board's flip/mirror. The HUD goes to the graphic layer: it is drawn in the
    panel's portrait orientation into a cached 480x640 BGRA buffer, and only the
    64 px tiles drawn this frame or last frame are copied into the visible page
    of `/dev/fb0`. MaixCDK's layer-1 push rotates with IVPS TDP, which smears
    G/R/A into the next pixel for 32-bit BGRA (stair-stepped, fringed edges), so
    the HUD does not use it. Also turns the backlight on.
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
- `maix_venc.*`, `maix_vdec.*`
  - the hardware H.264 encoder and decoder. `recordd` copies a ring slot into
    the encoder's own pool block with IVPS before encoding; `replayd` decodes a
    recorded frame into a ring slot. Both use only MSP SDK headers.
- `maix_shim.cc`, `stub/`
  - the few MaixCDK runtime pieces (log, err, board config lookup) and Kconfig
    stubs that the inline code in `ax_middleware.hpp` references but
    `libmaixcam_lib` does not export.

## Perception

- `src/model_output.*`
  - owns the openpilot master supercombo raw-output layout
    (`model_output_layout`, every block offset with a `static_assert`) and
    exposes parsed plan, lanes, road edges, leads, pose, and the meta pedal
    predictions (the chance the driver presses gas or brake 0, 2, …, 10 s
    ahead; the departure alert uses gas at 2 s). Also owns the shared
    `T_IDXS`/`X_IDXS` trajectory grids.
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
  - applies openpilot lane probability/width logic and the lateral MPC in
    `src/lateral_mpc.*` (Lane mode), or the plan yaw (Laneless mode and Lane
    mode's model-path stretches), to produce curvature targets. This is the
    only producer of `LateralTarget`.
- `src/desire_helper.*`
  - the openpilot desire_helper port the planner runs each model frame: lane
    change states (`LaneChangeState`), the blinker/torque/blind-spot/road-edge
    gates, the lane-line fade and the experimental turn desire pulses, and the
    `Desire` the model gets.
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
    while driving (`use_live_vehicle_params`, `use_live_torque_params`, both on
    by default). `vehicle_params_learner` is paramsd with its
    car_kf EKF and saved-value restore; `torque_estimator` is torqued with its
    cache; `lateral_learners` is the controlsd glue that turns vehicle CAN and
    locationd samples into learner inputs and collects the outputs as
    `LiveLateralParams`. `localizer_inputs.h` converts the IPC
    `LocalizationState` into a learner sample, so the learner library does not
    depend on the IPC layout.
- `src/location_estimator.*`, `src/lateral_lag.*`, `src/localization_pipeline.*`
  - ports of openpilot locationd (the 18-state pose EKF over the board IMU and
    the model's camera odometry) and lagd (the steering delay from desired vs
    actual lateral acceleration), and the pipeline that feeds them in time
    order. `locationd` and `replay_localization` share it.
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

- `src/ipc_messages.h`
  - every message that crosses `/dev/shm`: topic names, magics, channel headers,
    the state snapshots (`ModelState`, `ControlState`, `PandaState`, …) with
    their `static_assert`s. Recording v8 stores `ModelState`, `ControlState`,
    and `PandaState` as-is, so their offsets are pinned here and tied to
    `kRecordingVersion`. Code that only reads or fills a message includes this
    and nothing else.
- `src/model_state_fill.*`
  - `fill_model_state`, which modeld calls to pack a frame's outputs into
    `ModelState`, and `compute_lane_t` (the openpilot plan→lane time mapping).
    It needs the online calibrator's snapshot, so it lives in the `perception`
    library and message consumers do not pull in the calibrator.
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
  - draw the 640x480 HUD into a straight-alpha BGRA buffer (landscape
    coordinates; `HudOrientation` maps them onto the portrait panel buffer):
    state border, speed with yellow turn-signal/hazard chevrons, set speed
    (with the vision cruise `SET` speed) and gear cards, steering mode and the
    manoeuvre in progress (turn desire, lane change), an `AUTO HOLD` badge
    under the speed, plan/lane/road-edge ribbons faded with distance, the
    car's position in its lane marked on the road, lead chevron, torque bar
    with the driver's torque, alerts (including the lane-change nudge and the
    large-angle pause), TPMS and camera calibration cards with the board
    state card above TPMS and the learned values card above calibration
    (white while control uses them), a
    recording/Wi-Fi status pill (and the network card a tap opens), and chips
    that appear only when something needs attention (panda, storage). The
    numbers no other card shows sit in a card behind the `hud_debug` device
    setting. `overlay_canvas`
    fills polygons with 4-subrow anti-aliasing that touches only covered spans,
    fills the straight rows of integer rounded rectangles directly, blends
    without divisions, and records the 64 px tiles each row touched so the
    next use of the same buffer clears only those; `overlay_font` holds the
    glyphs that `tools/ui/make_hud_font.py` bakes from Pillow's Aileron (CC0).
    No OpenCV, so `gtest_overlay_canvas` and `hud_snapshot` run on the host.
    The renderer keeps only coverage scratch and the per-buffer tiles; the
    turn-signal phase and the network card toggle come from `overlayd`.
- `src/overlay_state.*`
  - `OverlayHudState`, the IPC state (`ControlState`, `ModelState`, …) →
    `OverlayHudState` mapping shared by
    `overlayd` and `hud_snapshot`, the `ModelState` →
    `ParsedModelOutput`/`ProjectionState` unpacking, the engage-block label table, and
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
    Only a `wlan` link with an address and an associated SSID counts as
    connected; another link (the USB virtual Ethernet, which always has an
    address) is kept apart for the network card.
- `src/recording_writer.*`, `src/recording_format.h`
  - the event-log writer and on-disk contract that `recordd` writes. It is the
    K230 recorder's format, so the host tools read MaixCAM2 and K230 drives.
    `gtest_recording_writer` pins the layout; `recording_format.h`
    (`kRecordingVersion`, the `K230LOG1` / `K230IDX1` headers, record types)
    is mirrored by `tools/model/recording_reader.py`.
- `src/event_log_reader.h`
  - the one C++ reader of `events/NNN.bin`, shared by `replayd` and the replay
    and dataset tools. It checks the magic, skips `header_size`, and stops
    without resyncing at a truncated tail (record type out of range, a payload
    over 1 MiB, or a short header or payload), which `truncated()` reports.
- `src/panda_client.*`, `src/panda_can_codec.*`, `src/pandad.cc`
  - optional panda USB bridge. It handles USB, health, heartbeat, receive CAN,
    and the final TX gate, but does not generate vehicle control messages.
- `src/controls_tick.*`
  - one 100 Hz controlsd tick without shared memory or files: CAN, model, Panda
    and locationd inputs in; the lateral controller, departure alerts, vision
    cruise and learners in between; the frames to send, `ControlState` and
    `LearnerState` out. The 20 Hz planner sits behind `PlannerPort`: a worker
    thread on the board (`LateralPlannerWorker`), computed in place in tools
    and tests (`SyncPlanner`). It also holds the parameter-file watcher.
- `src/controlsd.cc`
  - the controlsd process: opens the channels, reads them each tick in the
    order `ControlsTick` documents, publishes, sends, writes the learner files
    on a background thread, and logs the one-second stats line.
- `src/recordd.cc`
  - the drive recorder: encodes the frames `modeld` used and writes the CAN
    and state channels through `recording_writer`.
- `src/imud.cc`, `src/locationd.cc`
  - the board IMU reader (LSM6DSOW over `i2c-dev`) and the process that runs
    `localization_pipeline` on its samples and publishes `LocalizationState`.
- `src/replayd.cc`
  - rehearsal: plays a recorded route in place of `camerad` and `pandad`
    ([Rehearsal](rehearsal.md)).
- `src/camcal.cc`
  - still capture through the runtime's camera path for the intrinsics
    measurement ([Camera calibration](camcal.md)).
- `scripts/manager.py`
  - minimal supervisor and heartbeat publisher. It is intentionally not a full
    openpilot manager clone. It stops the stock launcher, switches USB-C to host
    for the Panda, and one table in start order decides which processes run
    (`EDGEPILOT_ENABLE_CONTROL`, `EDGEPILOT_ENABLE_PANDA`, `EDGEPILOT_ENABLE_PARAM_SERVER`) and
    with what nice value.
- `scripts/param_server.py`, `scripts/display_control.py`
  - the FastAPI parameter editor (`EDGEPILOT_ENABLE_PARAM_SERVER`) and the
    MaixCAM2 backlight helper it calls (PWM3).
- `scripts/web/`
  - the editor's BEV tab, drawn by the browser: `bev.js` (three.js view, ported
    from sv_recorder_bev), `bev_k7.js` (the ego car, a black 2017 K7 made in code),
    `bev_car.js` (the lead's car model), `bev_data.js` (reads the ModelState and
    ControlState bytes the editor streams), and three.js 0.186.1 in `three/`. On
    the board they go next to `param_server.py` as `web/`;
    `scripts/upload_to_board.sh` does not upload them, so copy them by hand.

## Scripts and tools

- `scripts/fetch_maixcam2_sdk.sh`, `tools/docker_ax630/`,
  `scripts/upload_to_board.sh`, `scripts/run_host_tests.sh`
  - pinned SDK and board-library fetch, the arm64 build container, deploy, and
    host tests. See `scripts/README.md` and
    [Build and deploy](build-and-deploy.md).
- `tools/model/axmodel/`
  - the openpilot master → axmodel pipeline (core extraction, output split,
    calibration and evaluation data, the board evaluation runner, Pulsar2
    config).
- `tools/model/`
  - the recording readers: `recording_reader.py` decodes `recordd` routes
    (frame index, event log, H.264 from MaixCAM2 or HEVC from K230) and mirrors
    `recording_format.h` / `ipc_messages.h` for the analysis tools;
    `lane_bias.py` and `make_replay.py` build on it. See
    `tools/model/README.md`.
- `tools/camcal/`
  - intrinsics from `camcal` captures (`calibrate_intrinsics.py`), the TV
    checkerboard generator and the measured MaixCAM2 intrinsics. See
    [Camera calibration](camcal.md).
- `tools/calib/`
  - the IMU-to-camera extrinsic estimate, the model-view preview and the
    12x7 chessboard image.
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
- `src/utils_file.h`
  - `file_stamp` (the stat fingerprint the processes poll parameter files
    with), `read_text_file`, and `write_file_atomic` (temporary file + rename,
    so a reader never sees a half-written file).
