# Runtime options

[← Documentation index](../README.md)

Boolean environment variables follow one convention: unset or empty uses the
default, `0`/`false`/`no`/`off`/`n` (case-insensitive) is false, anything else is
true.

## Manager

- `EDGEPILOT_MODEL=/path/to/supercombo.axmodel`
  - overrides the model selected by `manager.py` (default
    `models/supercombo.axmodel` in the install directory). A command-line
    argument wins over it. This replaces the K230 `K230_KMODEL`.
- `EDGEPILOT_STOP_LAUNCHER=0`
  - leaves the stock `launcher.service` and `/maixapp/apps/` running. The
    default stops them, since they hold the camera and NPU.

## Model and calibration

- `modeld <supercombo.axmodel>`
  - the only argument. The runtime targets the openpilot master supercombo core:
    5 inputs (`input_imgs` and `big_input_imgs` uint8 `[1,12,128,256]`,
    `desire` `[1,25,8]`, `features_buffer` `[1,24,512]`, `traffic_convention`
    `[1,2]`) and 2576 output floats. `modeld` checks names, shapes, dtypes,
    and buffer sizes at startup and refuses any other axmodel rather than
    misreading it.
- `EDGEPILOT_PROFILE=1`
  - `modeld` prints warp/inputs/NPU/outputs averages every 30 frames;
    `overlayd` adds HUD draw and video push timing to its status line.
- `EDGEPILOT_CAMERA_INTRINSICS=fx,fy,cx,cy`
  - camera intrinsics at 1920x1080, scaled to the capture size. The default is
    the calibrated MaixCAM2 camera (`1131.24,1130.85,940.13,552.60`). Replaying a
    K230 recording needs the K230 camera:
    `1583.3981,1583.7622,954.9441,545.1774`.
- `EDGEPILOT_CALIB_ROLL_DEG`, `EDGEPILOT_CALIB_PITCH_DEG`,
  `EDGEPILOT_CALIB_YAW_DEG`
  - manual calibration in degrees for both the overlay projection and the model
    input warp. If any value is set, it wins and online calibration is applied to
    neither. Otherwise the saved calibration is restored and pose-based online
    calibration feeds the next frame's input warp, matching openpilot's
    `cameraOdometry -> liveCalibration -> modeld` loop.
- `EDGEPILOT_CALIB_AUTO=0`
  - disables pose-based online overlay calibration and keeps the restored or
    manually supplied projection.
- `EDGEPILOT_LOG_CALIB=1`
  - prints the online calibrator status, accepted/rejected sample counts, valid
    block count, rpy, and spread.

## Camera and input warp

- `EDGEPILOT_AI_ISP=0|1`
  - turns the AX630C AI-ISP (NPU denoise) off or on in `camerad`, and makes
    `manager.py` pick the matching model (`supercombo_npu1.axmodel` when on).
    The default follows `/boot/configs` `maix_npu_ai_isp`, which also decides
    whether the NPU boots split.
- `EDGEPILOT_MAX_SHUTTER_US=N`
  - caps the auto-exposure shutter in `camerad` (default `33333`, the
    30 fps frame time, so the 20 fps sensor does not add motion blur). `0`
    leaves the sensor default.
- `EDGEPILOT_WARP_CPU=1`
  - disables the GDC warp and returns the whole input warp to the CPU; the
    wide tower is warped on the second core. The GDC path also falls back to the
    CPU when it cannot be set up or the frame is NV21.

## Storage and replay

- `EDGEPILOT_PARAMS_DIR=/path/to/params`
  - overrides the shared runtime parameter directory, and is the only way to
    relocate parameter files; there are no per-file overrides. The default is
    `params/` relative to the runtime working directory. Stable online
    calibration is stored atomically in `params/calibration.json` and restored
    before the first model frame. Manual `EDGEPILOT_CALIB_*` values take
    precedence and seed this file.
- `EDGEPILOT_REPLAY_NV12=/path/to/replay.scnv12`
  - when launching `modeld` directly, runs headless from an `SCNV12R1` NV12
    replay file instead of the camera ring. Width, height, and frame count are
    read from the replay header; the warp uses the same GDC path as live. This is
    for validating inference and online calibration from collected logs. It
    still runs on the board, since it needs the NPU.
- `EDGEPILOT_MAX_FRAMES=N`
  - stops after `N` frames (`modeld` inferred frames, `camerad`
    captured frames). This is mainly useful with replay mode.
- `EDGEPILOT_RAW_DUMP=/path/to/dump.bin`
  - during replay, writes every raw model output to an `SCODMP1` file that
    `gtest_model_output_parser` reads; see
    [diagnostics](diagnostics.md#model-swap-verification).

`recordd` reads `EDGEPILOT_RECORD_ROOT` (default `recordings` under the install
directory) and `EDGEPILOT_RECORD_STAGING`; `K230_RECORD_CODEC` (the K230 V4L2 device)
is gone. `params/recording.json` `enabled` toggles recording.

## Panda

- `EDGEPILOT_ENABLE_PANDA=1`
  - manager also starts `pandad` and switches the USB-C port to host mode
    (restored on exit). Off by default on the MaixCAM2 until the Panda wiring
    exists. The binary must have been built with `-DEDGEPILOT_BUILD_PANDA=ON`.
- `EDGEPILOT_USB_ROLE=host|device`
  - fixes the USB-C role at manager start and leaves it on exit.
    `edgepilot.service` sets `host` (the Panda plugs into the USB-C port);
    set `device` to reach the board from a computer over USB. Unset, the role
    only changes with `EDGEPILOT_ENABLE_PANDA=1` as above.
- `EDGEPILOT_PANDA_SAFETY=nooutput|silent|elm327|hyundai|hyundaiCommunity|allOutput`
  - panda safety mode for `pandad`. Its standalone default is `nooutput`;
    the manager defaults to `hyundaiCommunity`. `hyundai` defaults its parameter
    to `2` (the Hyundai/Kia hybrid path), every other mode to `0`; an unknown
    name falls back to `nooutput`.
  - collected KIA K7 YG HEV logs from the current openpilot fork report
    `safety=hyundaiCommunity:0`, `sccBus=-1`, `mdpsBus=1`, and `sasBus=1`. Use
    `EDGEPILOT_PANDA_SAFETY=hyundaiCommunity` for shadow/TX experiments unless a newer
    fingerprint proves otherwise.
- `EDGEPILOT_PANDA_SAFETY_PARAM=N`
  - numeric safety parameter passed with the safety mode. Unset takes the mode's
    default; the manager sets `0`.
- `EDGEPILOT_PANDA_TX=1`
  - allows `pandad` to relay ordered `/dev/shm/edgepilot_sendcan` batches to
    panda. Its standalone default is `0`; the manager defaults to `1`.
- `EDGEPILOT_PANDA_LOG_CAN=1`
  - prints every received CAN frame from `pandad`. This is a bus-bringup
    aid only; at full bus load it is far too noisy to leave on.
- `EDGEPILOT_PANDA_ENGAGED=1`
  - sends panda heartbeat as engaged, only meaningful with `EDGEPILOT_PANDA_TX=1`. Its
    standalone default is disengaged; the manager defaults to engaged.
- `EDGEPILOT_PANDA_IDLE_US=5000`
  - sleep time used by `pandad` when panda returns no CAN frames and no
    pending `sendcan` batch exists. This keeps USB-only or parked shadow runs from
    stealing scheduler time from `modeld`.

## K7 control

- `EDGEPILOT_ENABLE_CONTROL=1`
  - manager starts `controlsd`. This is the manager default. It does not
    start `pandad` on its own; that needs `EDGEPILOT_ENABLE_PANDA=1`. No
    openpilot checkout or Python native extension is required.
- `EDGEPILOT_FORCE_ENGAGED=0|1`
  - bypasses the SET/CANCEL engage latch for offline replay only. Default is `0`
    and must remain `0` in a vehicle.

## Parameter server and display

- `EDGEPILOT_ENABLE_PARAM_SERVER=0|1`
  - starts the FastAPI parameter editor with the manager. It defaults to the
    value of `EDGEPILOT_ENABLE_CONTROL`.
- `EDGEPILOT_PARAM_HOST=address`, `EDGEPILOT_PARAM_PORT=port`
  - select the parameter editor listen address and port. Defaults are
    `0.0.0.0:8080`.
- `EDGEPILOT_PARAM_DEFAULTS_DIR=/path/to/params.defaults`
  - directory the editor reads factory defaults from. The default is
    `params.defaults/` under the runtime working directory; the upload script
    fills it from the repository's `params/`.
- `EDGEPILOT_LEARNER_STATE_PATH=/dev/shm/...`
  - shared-memory file the editor reads the live learner state from (default
    `/dev/shm/edgepilot_learner_state`, written by `controlsd`). Tests point it
    at a temporary file.

`overlayd` turns the backlight on at start (`pwmchip0/pwm3`, level from
`maix_backlight_value` in `/boot/configs` and `disp_max_backlight` in
`/boot/board`). The param server then applies `params/display.json` through
`scripts/display_control.py` on the same PWM: duty = 100 µs period x
brightness % x `disp_max_backlight` %, and `enabled: false` sets the duty to 0.

## Alerts

`overlayd` plays the K230 piezo alert melodies on the board speaker. One
`aplay` starts with overlayd and stays open. A sound thread feeds it silence,
or the alert's samples when one fires. The HUD loop never spawns a process at
alert time. The amplifier stays on, so the first note is not cut. Latency is
under 0.1 s. A new alert interrupts the one playing.

- `EDGEPILOT_ALERT_SOUND=0` turns the sounds off (default on)
- `EDGEPILOT_ALERT_VOLUME` 0–100, default 70
- `EDGEPILOT_ALERT_PCM` ALSA device, default `plughw:0,1`

To check the speaker at the desk, run `pkill -USR1 -x overlayd`. Each
signal plays the next sound in the order engage, disengage, unable,
signal_changed, unavailable.

Departure alerts and engage refusals are also shown on the HUD, and every alert
is written as a `overlayd: alert=...` log line. The K230
`K230_PIEZO_BUZZER` and `K230_PIEZO_PIN` are gone. Which events alert
is described in [Departure alerts](departure-alerts.md).

## Parameter files

The tracked JSON files in `params/` are the source of truth for K7 steering,
driving, vision-cruise, recording, and display configuration. Changes are written
atomically. Control changes are signaled to `controlsd` and also detected by
its 100 ms fallback poll.

`params/calibration.json` is also tracked as the initial calibration seed. The
runtime replaces it atomically when a stable calibration is learned, while
`scripts/upload_to_board.sh` preserves an existing runtime copy and installs the
repository copy under `params.defaults/` as a fallback. A seed learned on the
K230 does not describe the MaixCAM2 mounting; let online calibration relearn it.

## Live parameter editor

Open the editor at `http://<board-ip>:8080`. It can also be started directly:

```sh
python3 /root/edgepilot/param_server.py --host 0.0.0.0 --port 8080
```

> [!WARNING]
> The editor has no authentication and writes steering parameters that
> `controlsd` hot-reloads while driving. Expose it only on a trusted vehicle
> or development network.

## Production defaults

- Capture is `NV12 1280x720`, the full sensor field of view scaled (no crop),
  sensor at 20 fps.
- The frame ring has 8 CMM slots.
- `overlayd` starts 1 s after `camerad`, because opening VI resets
  the AX pools.
- Child process nice levels are fixed as `camerad=0`, `overlayd=10`,
  `modeld=-15`, optional `pandad=-10`, `controlsd=-8`, and `param_server=10`.
- The front-vehicle marker is always enabled with probability threshold `0.5`.
- Desired curvature is clamped to openpilot's `0.2 1/m`; it is intentionally not
  a runtime tuning option.
