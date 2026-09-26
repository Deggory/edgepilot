# Split runtime

[← Documentation index](../README.md)

## Managed startup

```sh
python3 /root/sc_run/k230_manager.py [supercombo.axmodel]
```

The manager changes to its own directory, so it can be started from anywhere.
The no-argument command selects `models/supercombo.axmodel` (or `K230_MODEL`),
K7 control enabled, Panda TX enabled, and the FastAPI parameter server enabled.
`k230_pandad` is off unless `K230_ENABLE_PANDA=1`. Every setting can be
overridden with its environment variable; see
[Runtime options](runtime-options.md).

Stop it with Ctrl-C or `SIGTERM`: children get `SIGTERM`, then `SIGKILL` after
3 s.

For boot autostart, `scripts/install_autostart.sh [root@board]` installs
`scripts/supercombo.service` (after `rc-local.service`, which loads the AX
drivers; `Conflicts=launcher.service`; `Restart=always`) and disables the stock
launcher at boot. The unit reads `/etc/environment` for `LD_LIBRARY_PATH` and
sets `K230_LOG_DIR=/run/supercombo`, so each child's output goes to
`/run/supercombo/<name>.log` (tmpfs, emptied past 1 MiB). `systemctl start
launcher.service` switches back to the stock UI; `install_autostart.sh
--remove` undoes the install.

It also installs `wifi-dhcp-renew.service`. The service waits for
wpa_supplicant's control socket, then renews the wlan0 DHCP lease on every
reconnect; otherwise udhcpc keeps the old network's lease. It also sets the
AIC8800 Wi-Fi driver's debug level to errors only. The stock level 1039 logs
more than 5 lines/s, which the journal writes to the SD card.

Real-time daemons must not write to the SD card from their loop. With the
recording mover busy, a small open/rename can block for seconds. modeld's
calibration save did this, stalled modeld and dropped steering. Use a
background writer, as calibration and controlsd's learners do.

The manager restarts a dead child after 1 s. When `camerad` restarts,
`overlayd` and `recordd` are restarted after it settles (1.5 s), because opening
the VI resets the AX common pools they hold. If `camerad` crashes on a signal or
fails twice within 5 s of starting, the manager restarts every process: a
crashed `camerad` leaves its VI/IVPS group and pools allocated while `modeld`
keeps the AX system open.

## Runtime processes

### `k230_manager.py`

- stops the stock UI (`systemctl stop launcher.service`) and any app under
  `/maixapp/apps/`, which cost about 30% CPU and hold the camera and NPU, unless
  `K230_STOP_LAUNCHER=0`
- with `K230_ENABLE_PANDA=1`, switches the USB-C port to host mode
  (`/sys/class/usb_role/8000000.dwc3-role-switch/role`) and restores the
  previous role on exit
- starts, in this order: `k230_camerad`, `k230_overlayd` (1 s later, after
  camerad has opened VI and the AX pools), `k230_modeld`, then `k230_pandad`,
  `k230_controlsd`, and the parameter server when enabled; binaries that are
  not installed are skipped
- restarts a process 1 s after it exits
- publishes `managerState` to `/dev/shm/k230_manager_state` every second
- nice levels: `camerad=0`, `overlayd=10`, `modeld=-15`, `pandad=-10`,
  `controlsd=-8`, `param_server=10`

### `k230_camerad`

- opens the `ov_os04d10` through `libmaixcam_lib` (VI) at `NV12 1280x720` with
  AI-ISP off, so the NPU stays free for `k230_modeld`
- runs the sensor itself at 20 fps (`AX_ISP_SetSnsAttr`) with auto exposure on
  and the maximum shutter capped at 33,333 us (`SUPERCOMBO_MAX_SHUTTER_US`), so
  frames arrive 50 ms apart as the model expects
- allocates the 8 frame-ring slots as CMM (physically contiguous) blocks and
  copies each camera frame into the next slot with IVPS TDP; the CPU never
  touches the pixels
- the ring header in `/dev/shm/k230_road_ai` (ring version 5) carries each
  slot's physical address and a per-slot seqlock; only frame metadata is
  published as `roadAiFrame`
- lowers the sensor library's log level, which otherwise writes a harmless
  gain-table error on every AE update

### `k230_overlayd`

- drives the LCD with two VO layers; the hardware composes them and rotates the
  result for the 480x640 panel
- layer 0 is the camera: IVPS reads the newest ring slot, crops the centre 4:3
  (960x720 of 1280x720), and scales it to 640x480, so the preview is not
  stretched; a frame overwritten during the read is dropped
- layer 1 is the HUD: straight-alpha BGRA drawn by the CPU renderer directly
  into a CMM block at native 640x480 (compact layout with 208 px panels) and
  pushed without a copy
- redraws the HUD when a new model, control, panda, or manager snapshot
  arrives and once a second, at most every 45 ms, which gives 20 Hz with the
  model; lanes and path are anti-aliased; the turn-signal animation has its own
  50 ms clock
- HUD content: center speed, left `OPENPILOT`/`CONTROL`/`DRIVE`/`TPMS` panels,
  right `SYSTEM`/`HEALTH`/`CALIBRATION`/`LEAD` panels, and a centered status
  alert, filled with camera/model/HUD FPS, inference time,
  CPU/temperature/memory/storage, process health, Panda state, vehicle speed,
  steering torque, and K7 control state
- loads the traffic-signal PNG sprites from `assets/ui` next to the executable
- turns the backlight on (`/sys/class/pwm/pwmchip0/pwm3`, level from
  `/boot/configs`)
- plays the K230 piezo alert melodies on the board speaker, shows departure
  alerts and engage refusals on screen, and writes every alert as a
  `k230_overlayd: alert=...` log line

### `k230_modeld`

- takes one argument, the axmodel path, and refuses any model that does not
  match the openpilot master core contract
- runs on every frame (20 Hz): the GDC (`AX_IVPS_Dewarp`, perspective) warps
  the ring slot directly from its physical address into the two 512x256 model
  views; `SUPERCOMBO_WARP_CPU=1` selects the CPU warp instead
- checks after the warp that the slot was not overwritten; a torn frame is
  dropped before it enters the image and feature queues
- runs the NPU, parses the output, updates online calibration, feeds the
  calibration back into the next warp, and publishes compact `modelState`
- see [Model pipeline](model-pipeline.md)

### `k230_pandad` (optional)

- started only with `K230_ENABLE_PANDA=1`, when built with
  `-DSUPERCOMBO_BUILD_PANDA=ON` (the default)
- connects to the Panda over `libusb`, publishes compact Panda health and
  ordered CAN receive batches, and can relay ordered `sendcan` batches
- standalone default is shadow mode (`K230_PANDA_TX=0`); the manager's default
  enables TX

### `k230_controlsd` (K7 controller)

- enabled by default (`K230_ENABLE_CONTROL=1`)
- runs the openpilot-compatible lane planner and lateral MPC in a worker,
  with the KIA K7 YG HEV torque controller and `LKAS11`/`CLU11`/`MDPS12` packer
  at 100 Hz
- consumes model path, lane, road-edge, and vehicle-state IPC
- uses the vision lead distance and relative speed to adjust the stock
  fixed-speed cruise setting with rate-limited `SET-`/`RES+` CLU11 pulses; the
  first driver SET speed remains the maximum. Closing distance is projected
  through the measured 1.5 km/h/s vehicle response, repeated `SET-` pulses wait
  for that response, and `RES+` cannot immediately reverse a recent slowdown
- publishes generated raw `sendcan` batches for `k230_pandad`
- publishes compact `controlState` diagnostics for the display HUD
- does not transmit by itself; actual TX still requires `k230_pandad` with
  `K230_PANDA_TX=1`

### `k230_recordd`

- follows the frame `k230_modeld` used (`/dev/shm/k230_record_frame`), copies
  the ring slot into its own pool block with IVPS, checks the slot was not
  overwritten meanwhile, and encodes it with the hardware H.264 encoder (VENC,
  CBR, 1 s GOP) — the CPU never touches pixels (~5% of a core while recording).
  H.264 because the board's decoder only decodes H.264, so a route can be
  replayed as recorded ([Rehearsal](rehearsal.md))
- writes the recording format with the codec in the manifest
  (`segments/NNN/road.h264` + `frames.bin`; K230 routes are `road.hevc`),
  event log with CAN, model, control, panda and learner state,
  params snapshot), staged in tmpfs and moved to `recordings/` on the SD card
- `params/recording.json` `enabled` starts/stops a route; `bitrate_bps` applies
  at the next start

## Measured load

CPU per process on the board with the full camera/model/HUD/control pipeline
(one core = 100%): `camerad` ~11% (mostly the ISP 3A thread), `overlayd` ~14%,
`modeld` ~8%, `controlsd` ~1%. Camera, model, and HUD all run at 20 Hz with no
missed model frames.

## Recording format

The recording format is kept as the K230 recorder wrote it, so the host tools
(`recording_reader.py`, the replay tools, `lane_bias.py`) read K230 drives, and
`src/recording_writer.*` stays covered by `gtest_recording_writer` for the
recorder port.

The event log is written as 60 s chunks in `events/NNN.bin`, each starting with
an 8-byte `K230LOG1` magic, a version word, and fixed 16-byte record headers.
The current version is `5`.

| Record type | Payload |
| --- | ---: |
| `CanRx` / `CanTx` | variable CAN batch |
| `ModelState` | 3256 B |
| `ControlState` | 240 B |
| `PandaState` | 96 B |

Older recordings are not `ModelState`-compatible: version 1 carried 4384 B
including unused lateral draft fields, versions 2–3 carried 4080 B including the
stop-line block that openpilot v0.9.4 does not emit, and version 4 carried
4048 B including plan position stds and orientations that nothing read.
Version 2 also kept a single route-level `events.bin`; CAN logging alone
(~0.5 MB/s) filled the 988 MB tmpfs staging in about 30 minutes on long drives
and silently killed the rest of the recording, which is why version 3 rotates
event chunks alongside video segments. `tools/model/recording_reader.py` reads
the v3, v4, and v5 layouts; `lane_bias.py`, `hud_tools.py`,
`fit_lateral_params.py lag`, and `export_can_fixture.py` all walk the event log
through its `iter_event_records`.

## IPC boundaries

Camera frames never go through the small-message IPC. The frame ring's pixels
live in camerad's CMM blocks and are read by physical address: the GDC in
`k230_modeld` and IVPS in `k230_overlayd` read the slot directly, and a CPU
reader (the CPU warp fallback) maps it uncached. Only the ring header and
per-frame metadata are in `/dev/shm`. This keeps the split runtime close to
openpilot's process boundaries without paying the cost of Cap'n Proto/cereal.

The lateral plan has a single producer: the lateral MPC inside `k230_controlsd`.
`modelState` carries perception output only.
