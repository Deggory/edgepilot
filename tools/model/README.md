# Model tools

[← Documentation](../../docs/README.md)

Host scripts that read recorded routes and reproduce the device's model input
pipeline, and in [`axmodel/`](axmodel/README.md) the build of the model itself.
Use them to make a replay for the board, to measure lateral bias, or to rebuild
the axmodel.

Which tool reads which part of a route:

```mermaid
flowchart LR
    video["segments/NNN/<br>frames.bin, road.h264"]
    events["events/*.bin<br>ModelState, ControlState"]
    video --> replay["make_replay.py"]
    video --> m2["axmodel/<br>make_m2_data.py"]
    events --> m2
    events --> bias["lane_bias.py"]
    replay --> scnv12["replay.scnv12<br>modeld replay mode"]
    m2 --> sets["calibration and<br>evaluation sets"]
    bias --> terms["translation and<br>rotation terms"]
    classDef proc fill:#f6f8fa,stroke:#8b949e,color:#1f2328
    class video,events,replay,m2,bias,scnv12,sets,terms proc
```

All of them decode the route through `recording_reader.py`, and
`make_m2_data.py` warps the frames with `model_warp.py`.

## axmodel pipeline

The MaixCAM2 model is built by [`axmodel/`](axmodel/README.md): it cuts the NPU
core out of the openpilot master `driving_supercombo.onnx`, builds calibration
and evaluation data, and compiles it with Pulsar2 6.0 into the axmodel that
`modeld` loads. See [Model package](../../models/README.md) for the contract and
the board numbers.

| File | What it does |
| --- | --- |
| `axmodel/extract_core.py` | cuts the history queues out of the released graph and rewrites the ops Pulsar2 does not accept (opset-20 Cast, GatherND, the `Where(-inf)` mask, 2D LpNorm) into equivalent ones; fp16 is promoted to fp32 |
| `axmodel/split_outputs.py` | exports every head as its own output so each gets its own U16 scale; the runtime puts them back together (`src/model/model_output_assembly.h`) |
| `axmodel/make_m2_data.py` | builds the calibration and evaluation sets from MaixCAM2 recordings with inputs built like the runtime's: the numpy device warp ([`model_warp.py`](#model_warppy)) with the recorded calibration and camera mount, t-4/t pairs, recorded desire, the fp32 core's own feature history; `m2_calib_20261004.json` is the spec used on 2026-10-04 |
| `axmodel/run_axmodel_assembled.py` | runs an axmodel on the board over a `make_m2_data.py eval` set and writes the outputs in the original 2576-float layout, for comparison with the fp32 reference |
| `axmodel/pulsar2_u16_u8in.json` | the Pulsar2 build config: AX620E / NPU1 (one core; the AI-ISP denoiser uses the other), U16 everywhere, SmoothQuant, uint8 image inputs |

## Host environment

```sh
python3 -m venv .model-venv
.model-venv/bin/pip install -r tools/model/requirements.txt
```

The requirements are `numpy`, `onnx`, `onnxruntime`, `av` (the H.264 decoder
behind `recording_reader.py`) and `pillow` (for `tools/ui/hud_tools.py`
compose). `axmodel/` needs `onnx` and `onnxruntime`, and the Pulsar2 6.0 Docker
image for the compile step; nothing here needs a GPU or a compiler.

## Recording-driven helpers

`recording_reader.py` decodes a route and `model_warp.py` is a numpy port of
the CPU warp in `src/model/model_input_transform.cc`.

| File | What it does |
| --- | --- |
| `recording_reader.py` | the Python mirror of `src/recording/recording_format.h` and `src/common/ipc_messages.h` for the analysis tools |
| `make_replay.py` | writes an `SCNV12R1` replay for `modeld` replay mode on the board |
| `lane_bias.py` | measures the lateral bias of a drive and splits it into a translation term and a rotation term |
| `model_warp.py` | the device warp in numpy, and the MaixCAM2 intrinsics in one place |

### `recording_reader.py`

It is checked against the C++ asserts by `tests/check_recording_reader.py`.
`scripts/web_console/` keeps its own standard-library copy of the layouts it
shows, checked by `tests/check_web_console.py`. ControlState's field at offset
56 is `cluster_speed_kph`, the cluster's display speed; `ego_speed_kph` is the
wheel speed the controller uses.

### `make_replay.py`

```sh
python make_replay.py --route ROUTE --out DIR --frames 100 --skip 400
```

It decodes the route's frames into `DIR/replay.scnv12` (with
`replay_meta.json`), which `modeld` reads instead of the camera ring
(`EDGEPILOT_REPLAY_NV12`). The board side is in
[NV12 replay](../../docs/diagnostics.md#nv12-replay).

### `lane_bias.py`

Splitting the bias is what tells you whether a lane-hugging complaint is a
camera-calibration problem or not. See
[Lateral bias](../../docs/diagnostics.md#lateral-bias).

### `model_warp.py`

It is also used by `tools/camera/warp_preview.py` to show the MaixCAM2 model
views. It holds the MaixCAM2 intrinsics once (`maixcam2_intrinsics`, the
`src/common/app_config.h` values) for the preview, the calibration report and
the axmodel calibration data; the warp uses them unless given others.

It copies the runtime's 12-bit fixed-point sampling and its projection:
openpilot's `get_warp_matrix` (a rotation about the camera centre) with the
camera-mount shift `modeld` applies (`camera_offset_m`, `camera_height_m`).
The runtime multiplies in single precision, so the matrices agree to a
relative 1e-5, well under a thousandth of a pixel.
`check_recording_reader.py` holds them to the golden values that
`gtest_calibration` pins on the C++ side.

## Related

- [axmodel pipeline](axmodel/README.md): building the NPU model step by step
- [Model package](../../models/README.md): the shipped axmodel, its contract and
  board numbers
- [Model pipeline](../../docs/model-pipeline.md): the runtime warp and model
  that these tools reproduce
- [Recording](../../docs/recording.md): the route layout the reader decodes
- [Diagnostics](../../docs/diagnostics.md): the replay and lane-bias procedures
