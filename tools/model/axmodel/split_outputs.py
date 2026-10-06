"""Give each output head of the extracted core its own quantization range.

The core ends in one Concat of 13 heads ([1, 2576]). Pulsar2 quantizes a tensor
with one U16 scale, so that Concat forced every value onto the step of the
widest head: about 0.007 (plan x reaches ~200 m). That is fine for positions
but not for the plan yaw and yaw rate (a few mrad on a straight road): they
came out as 0 or +-0.007 and the laneless curvature law 2*psi/(v*t) jumped by
~0.6 m/s^2 per step (2026-09-29 road test: wheel swinging left and right).

This removes the Concats and makes every head a graph output (the policy's
Slice of the hidden state out of the vision Concat is fed from the hidden head
directly). The plan head
(Gemm 990x256 then Mul by a per-row scale) is split by rows into three Gemms
so the small orientation columns get a range of their own:

  out_plan_motion  mean columns 0-8 (position, velocity, acceleration), 33x9
  out_plan_orient  mean columns 9-14 (euler, orientation rate), 33x6
  out_plan_std     the 495 log-std values

src/model/model_output_assembly.h puts the outputs back into the 2576 layout; keep
the names and the row order in sync with it.

usage: python3 split_outputs.py core_fp32.onnx core_split.onnx
"""
import sys
import numpy as np
import onnx
from onnx import helper, numpy_helper

src, dst = sys.argv[1], sys.argv[2]
m = onnx.load(src)
g = m.graph
prod = {o: n for n in g.node for o in n.output}
inits = {i.name: i for i in g.initializer}
consumers = {}
for n in g.node:
    for i in n.input:
        consumers.setdefault(i, []).append(n)

VISION = ["out_meta", "out_desire_pred", "out_pose", "out_wide_from_device", "out_road_transform",
          "out_lanes", "out_lane_prob", "out_road_edges", "out_lead", "out_lead_prob", "out_hidden"]
VISION_SIZES = [55, 32, 12, 6, 12, 528, 8, 264, 144, 3, 512]

top = prod[g.output[0].name]
assert top.op_type == "Concat" and len(top.input) == 2, top
vision_cat, policy_cat = prod[top.input[0]], prod[top.input[1]]
assert vision_cat.op_type == "Concat" and len(vision_cat.input) == len(VISION), vision_cat
assert policy_cat.op_type == "Concat" and len(policy_cat.input) == 3, policy_cat
plan_t, desire_state_t, pad_t = policy_cat.input
assert pad_t in inits and int(np.prod(inits[pad_t].dims)) == 2, "expected a 2-wide zero pad"


def rename(old, new):
    """Rename tensor `old` to `new` everywhere (producer output and all consumers)."""
    n = prod[old]
    n.output[list(n.output).index(old)] = new
    for c in consumers.get(old, []):
        for j, x in enumerate(c.input):
            if x == old:
                c.input[j] = new
    prod[new] = n


for t, name in zip(vision_cat.input, VISION):
    rename(t, name)

# The policy reads the hidden state back out of the vision Concat with a Slice
# [1064:1576]. Feed it from the hidden head directly: the Concat goes away, and
# the policy input no longer carries the Concat's coarse step.
drop_extra = []
for n in consumers.get(vision_cat.output[0], []):
    if n is top:
        continue
    const = {i.name: numpy_helper.to_array(i) for i in g.initializer}
    const.update({c.output[0]: numpy_helper.to_array(c.attribute[0].t) for c in g.node if c.op_type == "Constant"})
    assert n.op_type == "Slice", n
    start, end, axis = (int(const[x].reshape(-1)[0]) for x in n.input[1:4])
    assert (start, end, axis) == (1064, 1576, 1), (start, end, axis)
    for c in consumers.get(n.output[0], []):
        for j, x in enumerate(c.input):
            if x == n.output[0]:
                c.input[j] = "out_hidden"
    drop_extra.append(n)
rename(desire_state_t, "out_desire_state")

# plan: Mul(Gemm(x, W, b), scale) -> three row groups
mul = prod[plan_t]
assert mul.op_type == "Mul", mul
gemm_t, scale_t = (mul.input if mul.input[1] in inits else mul.input[::-1])
gemm = prod[gemm_t]
assert gemm.op_type == "Gemm" and scale_t in inits, (gemm, scale_t)
attrs = {a.name: helper.get_attribute_value(a) for a in gemm.attribute}
assert attrs.get("transB", 0) == 1 and attrs.get("transA", 0) == 0, attrs
W = numpy_helper.to_array(inits[gemm.input[1]])
b = numpy_helper.to_array(inits[gemm.input[2]])
s = numpy_helper.to_array(inits[scale_t])
assert W.shape == (990, 256) and b.shape == (990,) and s.shape == (990,), (W.shape, b.shape, s.shape)
knots, width = 33, 15
mean_rows = np.arange(knots * width).reshape(knots, width)
groups = {
    "out_plan_motion": mean_rows[:, 0:9].reshape(-1),
    "out_plan_orient": mean_rows[:, 9:15].reshape(-1),
    "out_plan_std": np.arange(knots * width, 2 * knots * width),
}
assert sorted(np.concatenate(list(groups.values())).tolist()) == list(range(990))
new_nodes = []
for name, rows in groups.items():
    for suffix, arr in (("W", W[rows]), ("b", b[rows]), ("s", s[rows])):
        g.initializer.append(numpy_helper.from_array(arr.astype(np.float32), f"{name}_{suffix}"))
    new_nodes.append(helper.make_node("Gemm", [gemm.input[0], f"{name}_W", f"{name}_b"], [f"{name}_raw"],
                                      name=f"{name}_gemm", transB=1))
    new_nodes.append(helper.make_node("Mul", [f"{name}_raw", f"{name}_s"], [name], name=f"{name}_scale"))

drop = {id(top), id(vision_cat), id(policy_cat), id(mul), id(gemm)} | {id(n) for n in drop_extra}
kept = [n for n in g.node if id(n) not in drop]
del g.node[:]
g.node.extend(kept + new_nodes)
for name in (gemm.input[1], gemm.input[2], scale_t, pad_t):
    if not any(name in n.input for n in g.node):
        g.initializer.remove(inits[name])

del g.output[:]
sizes = dict(zip(VISION, VISION_SIZES))
sizes.update({"out_plan_motion": 297, "out_plan_orient": 198, "out_plan_std": 495, "out_desire_state": 8})
order = VISION + ["out_plan_motion", "out_plan_orient", "out_plan_std", "out_desire_state"]
for name in order:
    g.output.append(helper.make_tensor_value_info(name, onnx.TensorProto.FLOAT, [1, sizes[name]]))
del g.value_info[:]
m = onnx.shape_inference.infer_shapes(m)
onnx.checker.check_model(m)
onnx.save(m, dst)
print("outputs:", [(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim]) for o in m.graph.output])
