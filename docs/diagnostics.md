# Diagnostics

[← Documentation index](../README.md)

Procedures and measured results for the diagnostic tools. The tools themselves,
their options, and how to build them are listed in
[diagnostics/README.md](../diagnostics/README.md); the host unit tests are in
[gtest/README.md](../gtest/README.md).

## HUD snapshots

`hud_snapshot` renders the overlay renderer off-line for the idle / standby /
drive / busy / depart / fault scenarios, writes each frame as a `K230ARGB` file
and prints draw timings. `--maixcam2` draws the native 640x480 layout that
`k230_overlayd` puts on VO layer 1; without it the tool draws the K230 `480x800`
portrait frame (`--landscape` for `800x480`). It links OpenCV like
`k230_overlayd`, so build it where OpenCV is found: a host with OpenCV, or the
container build with `-DSUPERCOMBO_BUILD_DIAGNOSTICS=ON`.

```sh
./hud_snapshot --maixcam2 --assets assets/ui --out /tmp/hud
```

`hud_snapshot --model model.bin --control control.bin` replays a recorded
`K230ModelState` / `K230ControlState` pair instead of the synthetic scene;
`python3 tools/ui/hud_tools.py inputs <route_dir> <out_dir>` extracts such a
pair, plus the matching camera frame, from a K230 `recordd` route.
`python3 tools/ui/hud_tools.py compose /tmp/hud [camera.png]` turns K230-size
frames (`480x800` or `800x480`) into PNGs and composites a camera frame the way
the K230 panel showed it; it does not accept 640x480 frames yet.

## NV12 replay

`k230_modeld` can run headless from a recorded route: replay mode reads an
`SCNV12R1` file instead of the camera ring and feeds the same GDC warp as live
capture, so it validates model execution and online calibration from stored
segments. It needs the NPU, so it runs on the board. The recordings available
today come from the K230 camera, so pass its intrinsics.

```sh
# host: cut 120 frames of a route into an SCNV12R1 replay
python3 tools/model/make_replay.py --route recordings/<route> --out /tmp/replay_nv12 --frames 120
scp /tmp/replay_nv12/replay.scnv12 root@192.168.219.117:/root/sc_run/

# board (stop the manager first, or at least k230_modeld)
cd /root/sc_run
SUPERCOMBO_REPLAY_NV12=/root/sc_run/replay.scnv12 \
SUPERCOMBO_CAMERA_INTRINSICS=1583.3981,1583.7622,954.9441,545.1774 \
  ./k230_modeld models/supercombo.axmodel
```

## Model swap verification

A model swap changes three things at once: the warp input, the temporal
plumbing, and the network. The board side is verified by running a replay
with the raw outputs dumped:

```sh
# board: same frames through the runtime, dumping raw outputs
SUPERCOMBO_REPLAY_NV12=/root/verify/replay.scnv12 \
SUPERCOMBO_RAW_DUMP=/root/verify/board_raw.bin \
SUPERCOMBO_CALIB_AUTO=0 \
SUPERCOMBO_CAMERA_INTRINSICS=1583.3981,1583.7622,954.9441,545.1774 \
  ./k230_modeld models/<candidate>.axmodel
```

`board_raw.bin` is an `SCODMP1` file of 2576-float frames. Compare it against a
host reference on the slices that drive control (plan lateral offset, lane
positions) rather than on the raw vector, and check that the hidden-state slice
evolves smoothly — a dead temporal buffer still produces plausible single-frame
output. With `SUPERCOMBO_CALIB_AUTO=0`, the host reference must use the rpy the
board restored from `params/calibration.json`, because the calibration service
feeds the input warp on every frame.

The host reference in `tools/model/make_replay.py --model` runs the v0.9.4 ONNX
and does not fit the master contract. For the master core, the fp32 reference
with the runtime's queue semantics is built by
`tools/model/axmodel/make_core_data.py` (its `eval/` set); a host runner that
takes an `SCNV12R1` replay is not in the repository yet.

Measured when the master axmodel was brought up (200-frame K230 replay): the
board output against the host axengine runner gave a plan lateral difference of
0.0008 m at 2 s and a hidden-state cosine similarity of 0.9994. The K230 v0.9.4
numbers (int16 PTQ vs fp32) are in [models/README.md](../models/README.md).

## Lateral bias

The tools below run on the host over recorded drives. The recorder is not
ported to the MaixCAM2 yet, so today their input is K230 recordings.

If the car holds one side of the lane, `tools/model/lane_bias.py` says whether
the camera calibration is responsible:

```sh
python tools/model/lane_bias.py <route_dir> [<route_dir> ...]
```

It reads the recorded `modelState` and `controlState`, keeps straight engaged
stretches, and fits the perceived lane-centre offset against distance:

| term | meaning |
| --- | --- |
| translation (m) | camera off the vehicle centreline, or the car genuinely off-centre. **Camera intrinsics cannot produce this** -- a principal-point or focal-length error acts about the camera, so its lateral effect is exactly zero at `x=0`. |
| rotation (rad/m) | the calibration-shaped term. A wrong `cx` of `dcx` pixels appears here as roughly `dcx/fx`. |

It also prints the tuning that was active on that drive from the route's own
`params/` snapshot, the mean steering angle needed to hold a straight line, and
the mean curvature command, so a control bias can be told apart from a
perception bias.

Straightness is judged from the model's own 48 m path, never from the steering
angle: when the car needs a non-zero angle to go straight, an `|angle| < k`
filter keeps one side of the curve distribution and manufactures a rotation
term that is not there. On the 2026-08-19 route that mistake reported
-4.22 mrad where the honest figure is -0.74 mrad.

Measured on the two logged drives (no lane-line offset was applied on either):

| drive | `path_offset_m` | translation | rotation |
| --- | ---: | ---: | ---: |
| 2026-08-16 | 0.00 | +19.9 cm | +0.18 mrad |
| 2026-08-19 | 0.07 | +7.2 cm | -0.74 mrad |

The rotation term is under 1 mrad on both, so the left-hugging on those drives
was a lateral offset, not a camera-matrix error.

## Lateral dataset extraction

`extract_lateral_dataset` replays a recording and writes one CSV row per
`ControlState` record (~62 Hz), joining the CAN state decoded by the runtime's
own `vehicle_can` so signs and scaling match the board exactly:

```sh
cmake --build build-host --target extract_lateral_dataset -j2
./build-host/bin/extract_lateral_dataset out.csv <route>/events/*.bin
```

Version 2 route-level `events.bin` files work as well; a file truncated by the
tmpfs fill stops at the zero-filled gap with the byte offset on stderr.

Columns: time, wheel speed, `active`/`desire`/`block`, steering angle, driver
and applied torque (`tq_norm` is sign-corrected and divided by `steer_max`),
ESP12 lateral/longitudinal acceleration and yaw rate, the live bank estimate
and its roll equivalent, requested and measured curvature, and the NNFF-style
future values `la_p03..p15` / `roll_p03..p15` taken from the recorded model
plan.

Two deviations from the runtime are deliberate. The bank filter runs at row
rate with a `dt`-derived alpha rather than the controller's fixed 100 Hz step,
and future lateral acceleration is `v * dpsi/dt` off the plan's yaw angles
because the recorded `ModelState` has no plan acceleration. Both were checked
against known results: straight-line bank reproduces -0.178 on the 8-28 route
(logged: -0.176), and the total-least-squares torque fit over the 8-19 route
returns `latAccelFactor` 4.00, the value that route was fit to.

## Lateral planner replay

`replay_planner` re-runs `LateralPlanner` over a recording and writes
what the planner asked for, one row per `ModelState`. The MPC has no
board-specific dependency, so a recorded route can be re-planned without the
board:

```sh
cmake --build build-host --target replay_planner -j2
./build-host/bin/replay_planner out.csv <route>/events/*.bin
```

Columns include the recorded and re-planned desired curvature, the MPC's own
`target_curv`/`heading0`, the lane observations behind the plan, and the
`laneless`/`mpc_valid` flags. Comparing two builds' CSVs over the same route is
the check used for planner and solver changes. `--laneless` before the event
files forces Laneless mode, so the same route can be re-planned both ways
without touching `params/driving.json`.

## Related documents

- [Departure alerts](departure-alerts.md)
- [K7 Panda port](k7-panda-port.md)
