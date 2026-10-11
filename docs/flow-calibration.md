# Flow calibration

[← Documentation index](../README.md)

`modeld` warps the camera into the model's frame with a roll/pitch/yaw calibration. The online calibrator
(`model/calibration_online`, a port of openpilot's `calibrationd`) learns it from the model's own pose
output. The flow calibrator measures the same angles straight from the image instead. It always runs and is
recorded; the device setting `calibration_use_flow` (web console, default off) makes the warp take its pitch
and yaw ([Using the estimate](#using-the-estimate)).

## Why

`calibrationd` closes a loop through the model. Every sample is the current warp composed with the travel
direction the model's pose reports, so the loop settles where the pose says "straight ahead". That works
when the pose follows the warp one to one. On the MaixCAM2 it does not:

- In a controlled fp32 test, rotating the warp by 1° moved the pose's travel angle by only 0.185° and the
  lane direction by 0.31°.
- The quantized model behaves the same; quantized minus fp32 is 0.01°.
- With a restoring gain of about 0.18, a small pose bias moves the fixed point by bias / 0.18.
- On the 2026-10-10 drives the yaw crept back to about -0.5° after every reset. The measured direction of
  travel was about -0.05°.
- The offset is not a constant: it was -0.18° on the morning mount and about -0.5° in the evening.

The weak response belongs to our images, not to the model alone. On a public comma segment from an OS04C10
device (the same sensor family, night, city streets), the same fp32 model follows a warp change by about 0.5
(travel angle) and 0.85 (lane direction). On our footage it is 0.15 to 0.32 and 0.2 to 0.48, day and night.
- **Resolution:** resampling the comma footage to our pixel density does not change it. Neither does building
  both inputs from one camera, or degrading our own footage 2.5 times further. So a higher-resolution frame
  ring would not help.
- **Highlights:** clipping the brightest 10% of the comma footage halves the travel-angle response, while a
  linear 2x brightening does not change it. Our night exposure clips highlights.

The flow calibrator does not ask the model. It measures where the camera is actually going: the focus of
expansion (FOE). The result is an absolute measurement with no feedback loop, so a bias cannot build up and
averaging only removes noise.

## How

| Step | Where | Notes |
| --- | --- | --- |
| Gate | `flow_gate_open` | ≥ 40 km/h, yaw rate (curvature × speed) ≤ 0.25°/s, no blinker or desire, fresh CAN |
| Pyramid | VPP (`modeld`'s own sync queue), IVE as fallback | 640x360 and 320x180 in one call, read straight from the ring slot. The ring slot itself is the full-resolution level, with no copy |
| Corners | IVE Sobel + CornerDet response, then `pick_flow_corners` | at each window start, strongest response per 8x8 cell of the 640x360 road band, up to 300 points |
| Tracking | IVE pyramidal LK | one frame at a time for 8 frames (0.4 s), about 5 ms of IVE per frame |
| Solve | `solve_flow_direction` | windows whose median point displacement is under 1 px are dropped first; then travel direction (a, b, 1) plus a small rotation, Sampson epipolar residuals, Cauchy-weighted Gauss-Newton, then two rounds of trimming at three robust scales (1.4826·MAD) |
| Aggregate | `FlowCalibrator` | median of every 20 windows (a block), mean of the last 30 block medians, status 2 after 3 blocks; a mount change keeps only the newest blocks |

Yaw is `atan(a)` and pitch is `-atan(b)`, the same convention as the calibration's rpy. Roll is not
observed; it is not observed by `calibrationd` either. A window is accepted when at least 50 tracks survive
all 8 steps, their median displacement is at least 1 px, at least 40 fit the solution, and the camera turned
less than 0.5° in the window.

- **Parallax gate:** with no motion, every direction fits the epipolar constraint. A parked desk test with the
  gate forced open accepted 45% of its windows with random directions.
  - Real windows at 40 km/h and up have a median displacement of 0.85 px (night) and 0.32 px (day) at their
    1st percentile.
  - Below 1 px the yaw leans 0.1° to 0.2° (only distant points are left).
  - The 1 px gate drops 2.5% of night windows and 8% of day windows.
- **Mount change:** a block median is 20 windows, so its noise is about 0.2°. When the three newest blocks all
  sit more than 1° to the same side of the older blocks' mean, and within 0.5° of each other, the older blocks
  are dropped. The estimate then follows the new mount within three blocks instead of 30. A change under 1°
  is absorbed by the running mean.

Per gated frame, the board spends about 1.1 ms of CPU. Each window start adds about 2 ms of CPU for the
corner pick. That is about 2.6% of one core while driving straight, and nothing otherwise.

The IVE does the LK, about 5 ms per frame. It is otherwise unused. The pyramid goes to the VPP queue that
`modeld` holds anyway; see [Hardware engines](runtime.md#hardware-engines).

In the replay, a gated frame took 15.9 ms of wall time with the original copy-plus-IVE-pyramid path.
Reading the ring slot in place cut that to 12.7 ms with identical results. Moving the pyramid to the VPP cut
it to 9.2 ms, with 1.2 ms of CPU. The VPP refuses YUV400 output, so the levels come out as NV12. Paired by
window, the VPP and IVE pyramids agree to within 0.05° of median difference.

### What the IVE actually does

The AX630C IVE (MSP v3.0.0) differs from its header documentation in ways that shaped the design
(`platform/maixcam2/maix_ive_flow.h`):

- **LK start points:** they go in in top-level pixels, the level-0 position / 2^(levels - 1). The output
  points are level-0 pixels.
- **LK range:** the total displacement clips at ±8 px, and negative moves come back +16 px.
  - `bUseInitFlow` does not extend the range.
  - So the tracker steps one frame at a time, unwraps the result, and drops tracks that move more than
    7.5 px.
  - Within range the LK is precise: synthetic shifts come back to a 0.002 px median, and real frames agree
    with OpenCV to 0.08 px median.
- **Corners:** `AX_IVE_CornerDet` never fills its corner list in Shi-Tomasi mode.
  - Its response image equals the 5x5-window minimum eigenvalue exactly, so the points are picked on the
    CPU from that image.
  - Sobel accepts only a U16C1 output, but the values are signed.
- **Query:** `AX_IVE_Query` with block set returns `AX_ERR_IVE_QUERY_TIMEOUT` after a few ms while a task
  is still running, and has to be asked again.
  - That macro is the full `AX_DEF_ERR` code `0x80150054`, not the enum value `0x54`.
  - Not asking again leaves tasks running and makes the following ones fail.
- **Scalers:** the IVPS scalers (TDP/VPP) have four sync queues per board, one pair per process that calls
  `AX_IVPS_Init`, and the runtime takes all four. Inside `modeld` the flow calibrator uses `modeld`'s own
  queue. A separate tool, such as `flow_calibration_replay` while the runtime is up, gets `AX_ERR_IVPS_BUSY`
  and falls back to an IVE pyramid: a 5x5 Gaussian filter, then a DMA interval copy.

## Using the estimate

`model/calibration_source` picks what the warp uses, each model frame:

- **Setting off (default):** the pose loop's value, unchanged from before.
- **Setting on, flow estimate usable:** pitch and yaw from the flow estimate, roll from the pose loop (flow
  cannot see roll).
  - Usable means status 2 (3 blocks or more), a standard deviation of the block medians under 0.5°, and
    pitch and yaw inside upstream's calibration limits.
  - Until then the pose loop is used, so a fresh install calibrates the same way as before.
- **While engaged:** switching from the pose loop to the flow estimate waits for disengage. That covers a
  changed setting (the web console refuses it too) and an estimate that becomes usable mid-drive. Falling
  back to the pose loop happens at once, even while engaged. An applied estimate that moves (a new block, a
  mount change) is followed while engaged too.
- **Hysteresis:** the estimate starts being used below a 0.5° block-median standard deviation and stops above
  0.8°, so it does not toggle at the edge.
- **Ramp:** when the source changes or the flow estimate moves, the warp moves by 1/5 of the remaining
  difference per second, but at least 0.05°/s.
  - The model sees two frames and a 4.8 s feature history, so a step would jerk the path.
  - A 0.3° switch takes about 6 s and a 2° mount change about 10 s.
  - Back on the pose source, once caught up, the warp follows the pose loop directly again.

The pose loop pauses while the flow estimate drives the warp, and while the warp ramps between the two.
Like upstream, it composes each sample onto its own output, assuming the model saw an image warped with
that output. With another value in the warp that assumption breaks:
- there is no feedback through the warp, so the loop would settle at the flow value plus the model's own pose
  bias;
- the pitch-yaw cross term of the rotation composition would build up roll block after block (about
  0.05°/h at -1° pitch and a 0.1° bias), with nothing pulling it back.

Paused, the loop keeps its last estimate and its saved `calibration.json`. Back on the pose source it carries
on from there. `model/calibration_online` stays a plain port of upstream; `model/calibration_service` decides
when to feed it.

`ModelState.calibration` always describes the source the warp used. On the flow source it reports Calibrated
with the estimate's block count and standard deviations, so engagement is not held back by the pose loop.

The blocks are saved to `params/flow_calibration.json`:
- the first estimate and every mount change are saved at once, otherwise once a minute and at exit;
- they are restored at start, so the flow estimate is used from the first frame.

The web console's calibration reset clears both the pose calibration and the flow blocks, and both start
over. If the car engaged between the request and `modeld` picking it up, the request waits for disengage.

## Outputs

- `ModelState.flow_calibration` (`FlowCalibrationState`, recording v10; v11 appended four fields):
  - `status`: 0 off, 1 collecting, 2 estimate.
  - `windows` and `blocks`: counts so far (`blocks` keeps counting); `estimate_blocks`: blocks in the
    current estimate, up to 30.
  - `yaw` and `pitch`: the estimate, in rad.
  - `sd_yaw` and `sd_pitch`: the standard deviation of the block medians (v10 stored their range in the same
    place).
  - The last accepted window: `window_frame_id`, `window_yaw`, `window_pitch`, `window_rotation`,
    `window_tracks` and `window_inliers`. Per-window results can be pulled from a recording by watching
    `window_frame_id` change.
  - `window_parallax` (median displacement of the last solved window, px), `rejected_windows` and
    `mount_changes`.
- `ModelState.calibration_source` (`CalibrationSourceState`, recording v11):
  - `source`: 0 pose, 1 flow; `requested`: the setting as `modeld` last read it.
  - `flags`: flow usable, switch pending while engaged, warp ramping.
  - `pose_*`: the pose loop's own estimate and status, frozen while it is paused.
- `ModelState.calibration` (`CalibrationState`) is the value the warp used. Its status, block count and spread
  belong to the same source: on the pose source the pose loop's status, valid blocks and block range; on the
  flow source Calibrated, `estimate_blocks`, and the pitch and yaw standard deviations.
- The `modeld` log gets one `flowcal:` line per block. It holds the estimate, the last window, frame and
  window counts, and the wall and CPU time per frame. It also logs `calibration: warp source flow|pose` on
  every switch.
- The web console's camera calibration card (vehicle tab) has the source switch. It sends
  `POST /api/calibration/source`, which is refused while engaged. The generic parameter API does not take
  `calibration_use_flow`. The card shows the warp's value, the flow estimate, the pose loop's estimate and
  their difference. The HUD's CAL card reads FLOW instead of OK while the flow estimate is applied.
- `EDGEPILOT_FLOW_CALIB=0` turns the calibrator off ([Runtime options](runtime-options.md)). If the IVE
  cannot be set up, `modeld` logs `flow calibration off` and runs without it. A stored estimate is still
  recorded and can still be applied.

In `tools/model/recording_reader.py` the layouts are `FLOW_CALIBRATION_STATE` and `CALIBRATION_SOURCE_STATE`;
`flow_calibration(payload, version)` and `calibration_source(payload, version)` read them from any version.

## Validation

Offline, the own solver matched OpenCV's essential matrix within 0.03° on the same tracks. It also scattered
three times less between one-minute segments.

On the board, `diagnostics/flow_calibration_replay` decodes a recorded segment with the VDEC and feeds it
through the same `FlowWindowTracker` that `modeld` runs. Over six night segments and four day segments of
2026-10-10:

| Route | Flow calibrator on the board (median of accepted windows) | OpenCV reference (0.2 s pairs with a forward-backward check) |
| --- | --- | --- |
| 19:32, night, after the remount (646 windows) | yaw -0.02°, pitch -0.21° | -0.00°, -0.16° |
| 11:16, day, morning mount (265 windows) | yaw -0.58°, pitch -1.07° | -0.63°, -1.05° |

The online calibrator showed -0.16° to -0.51° on the night route and -0.79° on the day route. The day gap
is about one standard error of the median at 265 windows.

The corner threshold (0.2% of the strongest response) matters only at night. At 1%, two night segments kept
38 and 28 windows instead of 101 and 118, while the day results did not change.

Per gated frame the replay measured 15-17 ms of wall time and 1.6-1.9 ms of CPU. That includes the corner
pick and the solve, which are amortised over each window.

## Next

The setting stays off until a drive with recording shows a stable flow estimate. Then turn it on in the web
console, drive, and compare lane-relative position with the earlier drives. Because our images make the
model follow the warp only partly, fixing the yaw moves the car only a few centimetres, so `camera_offset_m`
gets rechecked afterwards. The pose loop's pitch matched the flow pitch within 0.07°, so mainly yaw changes.
