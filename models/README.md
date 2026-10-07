# supercombo model package

The MaixCAM2 runtime runs the openpilot **master** `driving_supercombo` core as
a Pulsar2 6.0 axmodel (U16 activations, uint8 image inputs) on the AX630C NPU.
It is `supercombo.axmodel` here, compiled for one NPU core (NPU1) because the
AI-ISP denoiser always uses the other. It is built with
[`tools/model/axmodel`](../tools/model/axmodel/README.md), and
`scripts/upload_to_board.sh` puts it on the board as
`/root/edgepilot/models/supercombo.axmodel`.

The current build (2026-10-04, sha256 `4ef2b2ab…`) adds Pulsar2 SmoothQuant
(`enable_smooth_quant`), which moves activation outliers of 124 convs into their
weights before the U16/S8 quantization. Against the fp32 core on identical
MaixCAM2 inputs (12 driving windows and 8 green-light stops, run on the board),
it is equal to or better than the previous build on every head:
- laneless lateral-acceleration error: mean 0.018 vs 0.020 m/s², p95 0.057 vs 0.063 m/s²;
- hidden-state cosine: 0.952 vs 0.947;
- green lights: the gas-press probability passes 0.3 within 0.25 s at 6 of the 8 stops, vs 4.

Other options did not help:
- FP32 conv weights are ignored on AX620E, so weights stay S8;
- highest mix precision does not build;
- EasyQuant needs more than Docker's 9.7 GB.

The fp32 core opens its plan at two of the stops where no U16 build does.

The shipped build was calibrated on an earlier sample set that is no longer in
this repository. New builds calibrate on MaixCAM2 recordings with
`tools/model/axmodel/make_m2_data.py`; in the 2026-10-04 comparison that
calibration widened the MinMax ranges and scored worse than the shipped build,
so compare a new build against it on the same inputs before replacing it.

The calibration data must carry desire pulses: `make_m2_data.py` uses the ones
controlsd recorded and adds a synthetic pulse to every fourth sample. The first
build calibrated desire with zeros only, so its range was [0, 0] and the
NPU model ignored every lane-change desire; the rebuild of 2026-09-27 reacts
like openpilot (laneChangeLeft 0.99 after a pulse) and is otherwise identical.

## Contents

- `supercombo.axmodel`
  - the model `modeld` loads.
- `manifest.sha256`
  - SHA-256 manifest for the tracked files.

## Contract

```text
input_imgs          [1, 12, 128, 256]  uint8   road tower, frames t-4 and t
big_input_imgs      [1, 12, 128, 256]  uint8   wide tower, frames t-4 and t
desire              [1, 25, 8]         float   5 Hz max-pooled desire history
features_buffer     [1, 24, 512]       float   5 Hz hidden-state history
traffic_convention  [1, 2]             float
-> 15 outputs (out_meta ... out_desire_state), reassembled into the openpilot
   [2576] layout by src/model/model_output_assembly.h
```

The heads are separate outputs (tools/model/axmodel/split_outputs.py) so each
gets its own U16 range. With the original single [1, 2576] output every value
was quantized to one step of about 0.007, which reduced the plan yaw to a few
levels and made laneless steering jump by about 0.6 m/s^2 per step. The runtime
still accepts a single-output axmodel.

The release graph (sha256 `65a08adc…`, the same model openpilot master ships as of
2026-09-23) also takes `action_t`, the lateral/longitudinal delay openpilot's modeld
feeds in, but it only goes into a Cast nothing reads, so it cannot change the
outputs and the core drops it. The delay is applied after the model instead, in
the laneless curvature law and the torque controller, as openpilot does.
`extract_core.py` stops if a newer graph starts to use it.

Both image towers are active: the runtime warps one `1280x720` NV12 frame
through two calibrated virtual cameras (`medmodel` fl=910 and `sbigmodel`
fl=455) and keeps a 5-frame history per tower. The queues the released ONNX
keeps in its graph (images, desire, features) are kept on the CPU by
`src/model/model_temporal.h`; the 512-float hidden state at offset 1064 is fed back
through `features_buffer`.

`modeld` verifies this contract at load and refuses any other axmodel, so
a mismatched model fails loudly instead of being misparsed. The output layout
is in [Model pipeline](../docs/model-pipeline.md#model-output).

## Verification

On the board, NPU inference takes about 15.5 ms on one core and a whole
`modeld` frame about 19.5 ms. On the 552-frame evaluation set the NPU1 model's
output is bit-identical to the earlier two-core (NPU2) build. On a 200-frame
replay the board output matched the host axengine runner to 0.0008 m in plan
lateral offset at 2 s, with a hidden-state cosine similarity of 0.9994.

The master model replaced v0.9.4 because Pulsar2 6.0 and 7.0 both miscompile
v0.9.4's Elu at U16 (see [`tools/model/axmodel`](../tools/model/axmodel/README.md)).
