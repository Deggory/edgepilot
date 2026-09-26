# Model pipeline

[← Documentation index](../README.md)

`camerad` captures `NV12 1280x720` into the CMM frame ring. On every frame
(20 Hz), `modeld` warps the newest slot into independent `512x256` medmodel
and sbigmodel views, pushes them into the per-tower image histories, runs the
openpilot master supercombo core on the AX630C NPU, and publishes compact
`modelState`.

| Stage | Time on the board |
| --- | --- |
| GDC warp, both views | ~1 ms |
| NPU inference | ~8.5 ms |
| whole `modeld` frame | ~12.5 ms |

The K230 ran openpilot v0.9.4 as an nncase kmodel in 27.7 ms of KPU time; that
pipeline is not part of this branch.

## Model

The model is the openpilot master `driving_supercombo` with its history queues
cut off, compiled with Pulsar2 6.0 as a U16 axmodel whose image inputs are
uint8. How it is built is in
[tools/model/axmodel](../tools/model/axmodel/README.md); the resulting file is
not in the repository and is uploaded with `EDGEPILOT_AXMODEL=...
scripts/upload_to_board.sh`. `src/ax_engine_session.*` wraps `libax_engine`
(the board image ships no engine headers, so `src/ax_engine_api.h` declares the
API) with a cached CMM buffer per tensor.

```text
input_imgs          [1, 12, 128, 256]  uint8   road tower: frames t-4 and t, 6 YUV planes each
big_input_imgs      [1, 12, 128, 256]  uint8   wide tower, same layout
desire              [1, 25, 8]         float   100-tick 20 Hz pulse history, max-pooled by 4
features_buffer     [1, 24, 512]       float   hidden states from slots 0, 4, ..., 92 of 96 ticks
traffic_convention  [1, 2]             float   constant (right-hand traffic)
-> outputs          [1, 2576]          float
```

`modeld` checks every name, shape, dtype, and buffer size at load and
refuses any other model, so a mismatched axmodel fails loudly instead of being
misparsed.

## Temporal inputs

The released ONNX keeps the history queues inside the graph; the NPU core does
not, so `src/model_temporal.h` keeps them on the CPU with the same convention.
Every queue has the newest entry last and shifts one slot per frame:

- desire: rising-edge pulses at 20 Hz over 100 ticks, max-pooled 4:1 into the
  25x8 input
- features: the last 96 ticks of the 512-float hidden state; the model sees
  every 4th slot. This frame's output is pushed after the run, so the model
  sees its own output from the next frame on
- images: the last 5 frames per tower; the model sees the oldest (t-4) and the
  newest (t)

A frame the camera overwrote while the GDC was reading it is dropped before it
enters any queue, and a failed NPU run pushes an empty feature row, so the
three queues stay aligned.

## Input warp

The warp is a calibrated pinhole homography per view (medmodel focal 910,
sbigmodel focal 455, both from one camera, like openpilot's single-camera
path), followed by packing into the YUV6 plane order
(`Y00, Y10, Y01, Y11, U, V`) at 128x256.

- **GDC (default).** `platform/maixcam2/maix_gdc_warp.*` runs `AX_IVPS_Dewarp`
  in perspective mode, reading the ring slot straight from its physical address
  and producing two 512x256 NV12 views, then unpacks them into the newest
  history slot. Against the CPU warp, Y differs by at most 1 LSB and U/V by
  0.4 LSB on average; on a K230 replay the plan lateral offset at 2 s against
  the fp32 host reference differs by 0.0005 m.
- **CPU (`EDGEPILOT_WARP_CPU=1`, NV21 frames, or GDC unavailable).**
  `src/model_input_transform.*` fuses the homography sampling with YUV6
  packing through compact fixed-point lookup tables (12-bit weights). The wide
  tower is warped on the second core. The ring slot is mapped uncached for this
  path. It is much slower (~23 ms for both towers) and meant for diagnostics.

The calibration service updates the warp's rpy after every frame, matching
openpilot's `cameraOdometry -> liveCalibration -> modeld` loop.

## Intrinsics

The source intrinsics are scaled from the measured `1920x1080` MaixCAM2
(`ov_os04d10`) camera matrix in `src/app_config.h` `kCamera*`:
`fx=1131.24`, `fy=1130.85`, `cx=940.13`, `cy=552.60`.

- **At 1280x720** (the capture size) that is `fx=754.2`, `fy=753.9`,
  `cx=626.8`, `cy=368.4`.
- **Measurement:** 42 photos of an 11x6 inner-corner chessboard on a 65" TV, taken
  with `camcal` through the runtime's camera path (0.52 px RMS; see
  [camcal](camcal.md)).
- **Earlier values:** `tools/calib/maixcam2_os04d10_intrinsics.json` came from the
  stock camera app. It agrees on focal length, but `cx` differs by 7 px.

Captures at
1280x720, 1920x1080, and 2560x1440 register to each other by pure scaling, so
the values scale to any capture size. Distortion stays around 1% at the
corners, so the warp stays pinhole. The camera's horizontal field of view is
80.6°, so 720p gives 13.2 px/° against the 17.9 px/° of the camera supercombo
was trained on.

`tools/calib/warp_preview.py` shows what the model sees from a captured frame:
both model views and their footprints on the source image.
`EDGEPILOT_CAMERA_INTRINSICS` overrides the matrix, for example with the K230
camera (`1583.3981,1583.7622,954.9441,545.1774`) to replay K230 recordings.

## Model output

The master supercombo emits 2576 floats. `src/model_output.*` owns the layout,
taken from the ONNX metadata `output_slices`:

| Block | Offset | Floats | Contents |
| --- | ---: | ---: | --- |
| meta | 0 | 55 | unused by this runtime |
| desire prediction | 55 | 32 | unused by this runtime |
| pose | 87 | 12 | translation, rotation, log stds |
| wide_from_device_euler | 99 | 6 | unused by this runtime |
| road_transform | 105 | 12 | unused by this runtime |
| lane lines | 117 | 528 | 4 lines x 33 points x (y, z), mean then log std |
| lane probabilities | 645 | 8 | 2 logits per line, the second is existence |
| road edges | 653 | 264 | 2 edges x 33 points x (y, z), mean then log std |
| leads | 917 | 144 | 3 time offsets (0/2/4 s) x 6 steps x (x, y, v, a), mean then log std |
| lead probabilities | 1061 | 3 | one per time offset |
| hidden state | 1064 | 512 | fed back through `features_buffer` |
| plan | 1576 | 990 | 33 points x 15 values, mean then log std |
| desire state | 2566 | 8 | desire softmax |

Unlike v0.9.4, the plan is a single hypothesis and each lead time offset has
its own trajectory. Of the 15 values per plan point, the parser consumes the
position (0–2). The last two floats are not used.
