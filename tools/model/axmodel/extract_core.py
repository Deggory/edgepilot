"""Cut the NPU core out of openpilot master driving_supercombo.onnx.

The released graph also carries the history queues (image/desire/feature
shift registers, desire 20->5 Hz max-pool). Those are cheap tensor shuffles the
runtime does on the CPU; the core keeps only the network:

  input_imgs      [1,12,128,256]  2 frames x 6 YUV planes (road)
  big_input_imgs  [1,12,128,256]  same for wide
  desire          [1,25,8]        5 Hz max-pooled desire history
  features_buffer [1,24,512]      5 Hz feature history
  traffic_convention [1,2]   (action_t only feeds an unused Cast in the release graph; dropped,
                             and the script stops if a newer graph starts to use it)
  -> outputs [1,2576]

fp16 weights/casts are promoted to fp32 (Pulsar2 quantizes from fp32).
"""
import sys
import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper, utils

src, dst = sys.argv[1], sys.argv[2]
tmp = dst + ".tmp.onnx"
cut = {"_unsafe_view": "input_imgs", "_unsafe_view_1": "big_input_imgs", "transpose": "desire",
       "transpose_1": "features_buffer", "traffic_convention": "traffic_convention"}
m = onnx.shape_inference.infer_shapes(onnx.load(src))


def reaches_outputs(graph, name):
    """Whether tensor `name` feeds any graph output."""
    outputs, frontier, seen = {o.name for o in graph.output}, [name], set()
    while frontier:
        t = frontier.pop()
        if t in outputs:
            return True
        seen.add(t)
        for n in graph.node:
            if t in n.input:
                frontier.extend(o for o in n.output if o not in seen)
    return False


# The release graph feeds action_t (the lateral/longitudinal delay openpilot's modeld
# passes in) into one Cast whose result nothing reads, so the core drops it. A model
# that starts to use it needs a new core input and runtime support: stop here.
if any(i.name == "action_t" for i in m.graph.input) and reaches_outputs(m.graph, "action_t"):
    sys.exit("action_t now feeds the outputs: add it as a core input and feed it at runtime")
onnx.save(m, tmp)
utils.extract_model(tmp, tmp, list(cut), ["outputs"])
m = onnx.load(tmp)
g = m.graph
# fp16 -> fp32
for init in g.initializer:
    if init.data_type == TensorProto.FLOAT16:
        init.CopyFrom(numpy_helper.from_array(numpy_helper.to_array(init).astype(np.float32), init.name))
for n in g.node:
    for a in n.attribute:
        if n.op_type == "Cast" and a.name == "to" and a.i == TensorProto.FLOAT16:
            a.i = TensorProto.FLOAT
        if a.type == onnx.AttributeProto.TENSOR and a.t.data_type == TensorProto.FLOAT16:
            a.t.CopyFrom(numpy_helper.from_array(numpy_helper.to_array(a.t).astype(np.float32)))
# uint8 image inputs -> fp32 graph inputs, rename to runtime names
for i in g.input:
    i.type.tensor_type.elem_type = TensorProto.FLOAT
ren = {k: v for k, v in cut.items() if k != v}
for n in g.node:
    for j, x in enumerate(n.input):
        if x in ren: n.input[j] = ren[x]
for i in g.input:
    if i.name in ren: i.name = ren[i.name]
# every Cast is now fp32->fp32 (Pulsar2 rejects opset-20 Cast): bypass them,
# and Identity with them
alias = {}
keep = []
for n in g.node:
    if n.op_type == "Cast":
        assert next(a.i for a in n.attribute if a.name == "to") == TensorProto.FLOAT
        alias[n.output[0]] = alias.get(n.input[0], n.input[0])
    elif n.op_type == "Identity":   # Pulsar2 7.0 rejects bare Identity links
        alias[n.output[0]] = alias.get(n.input[0], n.input[0])
    else:
        keep.append(n)
for n in keep:
    for j, x in enumerate(n.input):
        n.input[j] = alias.get(x, x)
outs = {o.name for o in g.output}
for o in outs & set(alias):   # graph output produced by a Cast: rename producer's output
    src_name = alias[o]
    for n in keep:
        for j, x in enumerate(n.output):
            if x == src_name: n.output[j] = o
        for j, x in enumerate(n.input):
            if x == src_name: n.input[j] = o
del g.node[:]
g.node.extend(keep)

# Pulsar2 cannot run the policy's GatherND (constant indices = last 9 feature
# tokens) nor Where(mask, -inf, x); rewrite both into equivalent simple ops.
consts = {i.name: numpy_helper.to_array(i) for i in g.initializer}
consts.update({n.output[0]: numpy_helper.to_array(n.attribute[0].t) for n in g.node if n.op_type == "Constant"})
byname = {n.name: n for n in g.node}
gnd = byname["model/p_node_GatherND_7"]
assert list(consts[gnd.input[1]].reshape(-1)) == list(range(-9, 0))
t_in, t_out = byname["model/p_node_Transpose_0"], byname["model/p_node_index"]
g.initializer.extend([numpy_helper.from_array(np.array([-9], np.int64), "last9_start"),
                      numpy_helper.from_array(np.array([np.iinfo(np.int64).max], np.int64), "last9_end"),
                      numpy_helper.from_array(np.array([1], np.int64), "last9_axis")])
sl = helper.make_node("Slice", [t_in.input[0], "last9_start", "last9_end", "last9_axis"], [t_out.output[0]], name="last9_slice")
wh = byname["model/p_node_masked_fill"]
mask = consts[wh.input[0]]
# scores measured in [-6.3, 4.7] on K230 drives: -30 leaves masked weights < e^-19
# and keeps the quantized range tight (an -inf/-1e4 fill would wreck it)
g.initializer.append(numpy_helper.from_array(np.where(mask, -30.0, 0.0).astype(np.float32), "attn_mask_add"))
add = helper.make_node("Add", [wh.input[2], "attn_mask_add"], [wh.output[0]], name="attn_mask_add")
new = []
for n in g.node:
    if n is gnd: new.append(sl)
    elif n is wh: new.append(add)
    elif n is t_in or n is t_out: continue
    else: new.append(n)
del g.node[:]
g.node.extend(new)

# Pulsar2 fuses x / expand(clip(ReduceL2(x))) into LpNormalization, which the
# AX620E backend only tiles for 3D/4D; run the two [1,512] L2-normalizations
# on a [1,1,512] view.
prod = {o: n for n in g.node for o in n.output}
g.initializer.extend([numpy_helper.from_array(np.array([1, 1, 512], np.int64), "l2n_3d"),
                      numpy_helper.from_array(np.array([1, 512], np.int64), "l2n_2d")])
new = []
for n in g.node:
    e = prod.get(n.input[1]) if n.op_type == "Div" and len(n.input) == 2 else None
    c = prod.get(e.input[0]) if e is not None and e.op_type == "Expand" else None
    r = prod.get(c.input[0]) if c is not None and c.op_type == "Clip" else None
    if r is not None and r.op_type == "ReduceL2" and r.input[0] == n.input[0]:
        x, y = n.input[0], n.output[0]
        new.append(helper.make_node("Reshape", [x, "l2n_3d"], [x + "_3d"], name=n.name + "_to3d"))
        r.input[0] = x + "_3d"; e.input[1] = "l2n_3d"
        n.input[0] = x + "_3d"; n.output[0] = y + "_3d"
        new.append(n)
        new.append(helper.make_node("Reshape", [y + "_3d", "l2n_2d"], [y], name=n.name + "_to2d"))
    else:
        new.append(n)
# ReduceL2 axes must point at the last dim of the 3D view
for n in new:
    if n.op_type == "ReduceL2":
        ax = n.input[1] if len(n.input) > 1 else None
        if ax:
            g.initializer.append(numpy_helper.from_array(np.array([-1], np.int64), n.name + "_ax"))
            n.input[1] = n.name + "_ax"
del g.node[:]
g.node.extend(new)
# topological order: the new Reshape must precede its ReduceL2
order, seen, avail = [], set(), {i.name for i in g.input} | {i.name for i in g.initializer} | {""}
pending = list(g.node)
while pending:
    rest = []
    for n in pending:
        if all(i in avail for i in n.input): order.append(n); avail.update(n.output)
        else: rest.append(n)
    assert len(rest) < len(pending), "cycle"
    pending = rest
del g.node[:]
g.node.extend(order)
del g.value_info[:]
m = onnx.shape_inference.infer_shapes(m)
onnx.checker.check_model(m)
onnx.save(m, dst)
print([(i.name, [d.dim_value for d in i.type.tensor_type.shape.dim]) for i in m.graph.input])
