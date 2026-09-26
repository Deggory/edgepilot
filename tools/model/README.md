# Model tools

The MaixCAM2 model is built by [`axmodel/`](axmodel/README.md): it cuts the NPU
core out of the openpilot master `driving_supercombo.onnx`, builds calibration
and evaluation data, and compiles it with Pulsar2 6.0 into the axmodel that
`modeld` loads. See [`../../models/README.md`](../../models/README.md) for
the contract and the board numbers.

The other scripts here come from the K230 v0.9.4 pipeline. The nncase compile
and ONNX sanitizer are gone with it; the helpers below remain because they read
the K230 recordings, produce the PTQ samples `axmodel/make_core_data.py` still
uses, or (for the last two) are kept for reference only.

## axmodel pipeline

- `axmodel/extract_core.py`
  - cuts the history queues out of the released graph and rewrites the ops
    Pulsar2 does not accept (opset-20 Cast, GatherND, the `Where(-inf)` mask, 2D
    LpNorm) into equivalent ones; fp16 is promoted to fp32.
- `axmodel/make_core_data.py`
  - writes `calib/*.tar` from the 180 PTQ samples in `models/ptq` and an
    `eval/` set from the K230 v0.9.4 evaluation bundle (`QEXP094_DIR`, outside
    the repository), run through the fp32 core with the runtime's queue
    semantics.
- `axmodel/pulsar2_u16_u8in.json`
  - the Pulsar2 build config: AX620E / NPU1 (one core; the AI-ISP denoiser
    uses the other), U16 everywhere, uint8 image inputs.

## Host environment

```sh
python3 -m venv .model-venv
.model-venv/bin/pip install -r tools/model/requirements.txt
```

`axmodel/` additionally needs `onnx` and `onnxruntime` (both in the
requirements) and the Pulsar2 6.0 Docker image for the compile step.

## Recording-driven helpers

These read K230 `recordd` routes and reproduce the device's input pipeline
(`recording_reader.py` decodes the route, `model_warp.py` is a numpy port of
the CPU warp in `src/model_input_transform.cc`, `route_frames.py` joins them,
and `op094_runner.py` drives the v0.9.4 ONNX with the desire/feature history the
K230 runtime kept).

- `recording_reader.py`
  - the one Python mirror of `src/recording_format.h` and `src/ipc_messages.h`.
- `make_calibration.py`
  - captured the PTQ samples in `models/ptq` across routes; each sample carries
    the feature buffer the v0.9.4 model itself produced, so calibration saw the
    real activation ranges.
- `make_replay.py`
  - writes an `SCNV12R1` replay for `modeld` replay mode on the board.
    Its optional `--model` host reference runs the v0.9.4 ONNX and does not fit
    the master contract; see `../../docs/diagnostics.md`.
- `lane_bias.py`
  - measures the lateral bias of a drive and splits it into a translation term
    and a rotation term, which is what tells you whether a lane-hugging
    complaint is a camera-calibration problem or not. See
    `../../docs/diagnostics.md`.
- `model_warp.py`
  - also used by `tools/calib/warp_preview.py` to show the MaixCAM2 model views.

## K230 only (unused on this branch)

- `prequant_bias_correct.py`
  - rounded the v0.9.4 Conv/Gemm weights onto nncase's per-channel uint8 grid
    and cancelled the mean output shift in each bias.
- `retype_image_inputs_uint8.py`
  - retyped the v0.9.4 image inputs to uint8. The axmodel gets uint8 image
    inputs from Pulsar2's `input_processors` instead.
