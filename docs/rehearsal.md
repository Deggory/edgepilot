# Rehearsal (on-board replay)

[← Documentation index](../README.md)

A rehearsal runs the whole MaixCAM2 runtime on a recorded drive at real time:
`replayd` takes the place of `camerad` and `pandad`, and
`modeld`, `controlsd`, `overlayd` and `recordd` run exactly
as in the car. The LCD shows the recorded road with the live HUD, alert sounds
play, and the rehearsal can itself be recorded and compared with the original.

## What `replayd` does

- **Video:** decodes `segments/NNN/road.h264` frame by frame on the AX630C
  hardware decoder (VDEC) at the recorded capture intervals and copies each
  frame into the CMM frame ring with IVPS, where the camera would put it. The
  CPU never touches pixels.
- **CAN and panda state:** publishes the recorded `CanRx` batches and
  `PandaState` on the same timeline, into the queues `pandad` feeds (and
  the recorder's CAN log queue).
- **Transmit:** `controlsd`'s send requests are **not transmitted**; they
  go to the send log only, so a rehearsal is always shadow.
- Timestamps are rewritten to the current time; recorded times only set the
  pacing. When the clip ends it idles so the manager does not restart it.

## Routes

- **MaixCAM2 routes** record H.264 (`road.h264`) and replay as they are.
- **K230 routes** are HEVC, which the AX630C decoder cannot decode
  (`AX_ERR_NOT_SUPPORT`). Convert the segments you need on the host first:

      python3 tools/rehearsal/transcode_route.py ROUTE OUT --segments 2-2 --crf 26

  It re-encodes to H.264 (no B-frames, 1 s GOP, AUD per frame) and rewrites
  `frames.bin` so every frame keeps its recorded frame id and capture time; the
  matching `events/NNN.bin`, manifest and params are copied.

Keep the clip in tmpfs (`/tmp`) on the board and short (about a minute): it is
read at 20 fps, and the SD card is the component that failed under sustained
load. Record the rehearsal on the SD card, since the recorder requires 5 GB and
10% free space at its root.

## Running

Stop the service and start the manager in rehearsal mode:

    systemctl stop edgepilot
    EDGEPILOT_LOG_DIR=/run/edgepilot \
    EDGEPILOT_REPLAY_ROUTE=/tmp/rehearsal/<route> EDGEPILOT_REPLAY_START=0 EDGEPILOT_REPLAY_DURATION=58 \
    EDGEPILOT_FORCE_ENGAGED=1 EDGEPILOT_RECORD_ROOT=/root/rehearsal_out \
    EDGEPILOT_CAMERA_INTRINSICS=1583.3981,1583.7622,954.9441,545.1774 \
    python3 /root/edgepilot/manager.py

- `EDGEPILOT_REPLAY_START`/`_DURATION` are seconds from the first frame of the
  copied segments (duration 0 = to the end).
- `EDGEPILOT_FORCE_ENGAGED=1` because a clip cut from the middle of a drive has no
  engage button edge; nothing is transmitted anyway.
- `EDGEPILOT_CAMERA_INTRINSICS` for K230 footage (both the model warp and the
  HUD projection follow it); omit it for MaixCAM2 routes.
- Turn on `params/recording.json` `enabled` to record the rehearsal, then
  `systemctl start edgepilot` when done.

## Comparing

    python3 tools/rehearsal/compare_rehearsal.py ORIGINAL_ROUTE REHEARSAL_ROUTE

It aligns the two routes on the CAN speed trace and reports engagement
agreement, desired curvature and torque differences over the ticks both were
active, and the rates. The steering loop is open in a rehearsal (the recorded
car answered the original controller, not this one), so the torque integrator
winds up and torque differences are not meaningful; desired curvature is the
number to watch. Use `diagnostics/replay_closed_loop.cc` on the host for
closed-loop questions.

First run (2026-09-25, K230 route 2026-09-07 120–180 s, 66 km/h, engaged):
speed alignment 0.05 kph; model 19.7 Hz, 0 decode errors; engaged/active agree
100% while the clip played; desired curvature corr 0.904, mean |d| 0.0005 1/m
(the K230 ran openpilot v0.9.4, the MaixCAM2 runs the master model).
