"""잘라 낸 코어의 출력 헤드마다 고유한 양자화 범위를 준다.

코어는 헤드 13개를 잇는 Concat 하나([1, 2576])로 끝난다. Pulsar2는 텐서 하나를
U16 스케일 하나로 양자화하므로, 그 Concat 때문에 모든 값이 가장 넓은 헤드의
간격에 맞춰졌다: 약 0.007(plan x가 ~200 m까지 간다). 위치에는 괜찮지만
plan yaw와 yaw rate(직선 도로에서 몇 mrad)에는 맞지 않는다: 이 값들이 0이나
+-0.007로 나와서 laneless 곡률 법칙 2*psi/(v*t)가 한 간격마다 ~0.6 m/s^2씩
뛰었다(2026-09-29 도로 시험: 핸들이 좌우로 흔들렸다).

이 스크립트는 Concat을 없애고 모든 헤드를 그래프 출력으로 만든다(policy가 vision
Concat에서 hidden state를 꺼내던 Slice는 hidden 헤드에서 바로 받는다). plan 헤드
(Gemm 990x256 뒤에 행별 scale을 곱하는 Mul)는 행으로 나눠 Gemm 세 개로 쪼개서
작은 orientation 열들이 자기 범위를 갖게 한다:

  out_plan_motion  평균 열 0-8(위치, 속도, 가속도), 33x9
  out_plan_orient  평균 열 9-14(euler, orientation rate), 33x6
  out_plan_std     log-std 값 495개

src/model/model_output_assembly.h가 출력을 2576 배치로 다시 맞춘다. 이름과 행 순서를
그 파일과 맞춰 둔다.

사용: python3 split_outputs.py core_fp32.onnx core_split.onnx
"""
import argparse
import numpy as np
import onnx
from onnx import helper, numpy_helper

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("src", help="core_fp32.onnx")
parser.add_argument("dst", help="core_split.onnx")
args = parser.parse_args()
src, dst = args.src, args.dst
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
    """텐서 `old`의 이름을 모든 곳(생산 노드 출력과 모든 소비 노드)에서 `new`로 바꾼다."""
    n = prod[old]
    n.output[list(n.output).index(old)] = new
    for c in consumers.get(old, []):
        for j, x in enumerate(c.input):
            if x == old:
                c.input[j] = new
    prod[new] = n


for t, name in zip(vision_cat.input, VISION):
    rename(t, name)

# policy는 vision Concat에서 Slice [1064:1576]로 hidden state를 다시 꺼낸다.
# hidden 헤드에서 바로 넣는다: Concat이 없어지고, policy 입력도 더는
# Concat의 거친 간격을 물려받지 않는다.
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

# plan: Mul(Gemm(x, W, b), scale) -> 행 그룹 세 개
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
