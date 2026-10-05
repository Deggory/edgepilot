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

On the older K230-camera evaluation set its laneless p95 is 0.29 vs 0.22 m/s²,
while hidden cosine and desire are better. Other options did not help:
- calibrating on MaixCAM2 recordings widened the MinMax ranges and was worse;
- FP32 conv weights are ignored on AX620E, so weights stay S8;
- highest mix precision does not build;
- EasyQuant needs more than Docker's 9.7 GB.

The fp32 core opens its plan at two of the stops where no U16 build does.

The desire input is calibrated with one-hot pulses (half the samples carry one).
The first build calibrated it with zeros only, so its range was [0, 0] and the
NPU model ignored every lane-change desire; the rebuild of 2026-09-27 reacts
like openpilot (laneChangeLeft 0.99 after a pulse) and is otherwise identical.

Also tracked are the calibration samples the axmodel build uses.

## Contents

- `ptq/supercombo_calib.npz`, `ptq/supercombo_calib_k230_120.npz`
  (+ `_metadata.json`)
  - 60 + 120 samples captured from recorded K7 drives on the K230 (city,
    highway, day/evening/night, standstill included) with the K230 v0.9.4
    runtime's inputs. `tools/model/axmodel/make_core_data.py` turns all 180 into
    the Pulsar2 calibration set; the samples only hold frames t-1/t, so t-1
    stands in for the master core's t-4.
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
   [2576] layout by src/model_output_assembly.h
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
`src/model_temporal.h`; the 512-float hidden state at offset 1064 is fed back
through `features_buffer`.

`modeld` verifies this contract at load and refuses any other axmodel, so
a mismatched model fails loudly instead of being misparsed. The output layout
is in [Model pipeline](../docs/model-pipeline.md#model-output).

## Verification

On the board, NPU inference takes about 15.5 ms on one core and a whole
`modeld` frame about 19.5 ms. On the 552-frame evaluation set the NPU1 model's
output is bit-identical to the earlier two-core (NPU2) build. On a 200-frame
K230 replay the board output matched the host axengine runner to 0.0008 m in plan lateral offset at 2 s, with a hidden-state
cosine similarity of 0.9994.

The master model replaced v0.9.4 because Pulsar2 6.0 and 7.0 both miscompile
v0.9.4's Elu at U16 (see [`tools/model/axmodel`](../tools/model/axmodel/README.md)).

### K230 v0.9.4 build (history)

Board measurements for the K230 `supercombo.kmodel` (openpilot v0.9.4, nncase;
removed from this branch, kept on the `k230` branch) on the K230 (pipeline stopped; the
previous build is the 60-sample PTQ without weight pre-quantization, measured on
the same frames):

| check | this build | previous build |
| --- | --- | --- |
| NPU time, 600 frames | 27.7 ms/frame mean, inside the 50 ms 20 Hz budget | 28.6 ms |
| uint8 image retype | bit-identical to the fp32 graph (max diff 0.0) | same |
| int16 PTQ vs host fp32, 600 held-out frames, 6 routes | plan hypothesis match 94.5 %, plan lateral 0.020 m mean at 2 s / 0.051 m mean over the horizon, plan longitudinal 0.91 m, lanes 0.159 m mean | 92.5 %, 0.020 / 0.061 m, 0.98 m, 0.155 m |
| runtime end to end (warp + temporal inputs + kmodel) vs host fp32, 100 city frames | plan lateral 0.029 m mean / 0.183 m max at 2 s, plan lateral 0.075 m mean over the horizon, lanes 0.102 m mean | 0.030 / 0.190 m, 0.086 m, 0.129 m |
| feature buffer liveness, 100 city frames | frame-to-frame correlation 0.60 (fp32 reference on the same frames 0.62) | 0.60 |

The weight pre-quantization with bias correction is what moved the plan
numbers; a larger calibration set alone changed nothing, and SQuant lowered the
raw-output MAE without improving the plan. About two thirds of the remaining
error was the KPU's ELU table, not quantization. The full candidate table is on
the `k230` branch.

Quality against the recorded drives (held-out routes, error against the path the
car actually drove, reconstructed from CAN speed and steering): plan lateral
error 0.51 m at 2 s. The metric is lineage-independent, so the master model can
be scored the same way.
