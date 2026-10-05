"""Build calibration + open-loop eval data for the openpilot master core.

Images come from the K230 recordings already used for the 0.9.4 evaluation
(same medmodel/sbigmodel warps, 6-plane YUV per frame). The master core looks
at frames t-4 and t (5 Hz context), so:

  eval  : the 6 held-out 100-frame routes; the host fp32 core is run
          sequentially from zero state with the runtime's queue semantics, and
          frames 8..99 are kept with the fp32 features/desire actually fed.
          Reference outputs are the fp32 outputs for those same inputs.
  calib : the 180 K230 PTQ samples (disjoint routes/segments). They only hold
          t-1/t, so t-1 stands in for t-4; features come from a random eval
          frame (feature statistics only, not route content).
"""
import json, os, io, tarfile
import numpy as np, onnxruntime as ort

# Q: K230 0.9.4 평가 묶음(eval094_all.npz/json), P: PTQ 샘플(저장소 models/ptq),
# CORE: extract_core.py 출력. 결과는 현재 디렉터리의 calib/, eval/에 쓴다.
HERE = os.path.dirname(os.path.abspath(__file__))
Q = os.environ.get("QEXP094_DIR", os.path.join(HERE, "../../../models/work/qexp094"))
P = os.environ.get("PTQ_DIR", os.path.join(HERE, "../../../models/ptq"))
CORE = os.environ.get("CORE_ONNX", "core_fp32.onnx")
sess = ort.InferenceSession(CORE)
ev = np.load(f"{Q}/eval094_all.npz"); meta = json.load(open(f"{Q}/eval094_all.json"))

keep = {k: [] for k in ["input_imgs", "big_input_imgs", "desire", "features_buffer", "traffic_convention"]}
refs, tags = [], []
off = 0
for r in meta:
    n = r["n"]; road = ev["input_imgs"][off:off+n, 6:]; wide = ev["big_input_imgs"][off:off+n, 6:]
    tc = ev["traffic_convention"][off:off+1]
    feat_q = np.zeros((96, 512), np.float32); desire_q = np.zeros((100, 8), np.float32)
    for k in range(4, n):
        desire_q = np.roll(desire_q, -1, 0); desire_q[-1] = 0
        feed = {"input_imgs": np.concatenate([road[k-4], road[k]])[None].astype(np.float32),
                "big_input_imgs": np.concatenate([wide[k-4], wide[k]])[None].astype(np.float32),
                "desire": desire_q.reshape(25, 4, 8).max(1)[None],
                "features_buffer": feat_q[0::4][None].copy(), "traffic_convention": tc}
        out = sess.run(None, feed)[0]
        if k >= 8:
            for key in keep: keep[key].append(feed[key][0])
            refs.append(out[0]); tags.append(r["tag"])
        feat_q = np.roll(feat_q, -1, 0); feat_q[-1] = out[0, 1064:1576]
    off += n
    print(r["tag"], "done", flush=True)
os.makedirs("eval", exist_ok=True)
for key, v in keep.items(): np.save(f"eval/{key}.npy", np.stack(v))
np.save("eval_ref.npy", np.stack(refs)); json.dump(tags, open("eval_tags.json", "w"))

# calibration tars
c = [np.load(f"{P}/{f}") for f in ("supercombo_calib.npz", "supercombo_calib_k230_120.npz")]
road = np.concatenate([x["input_imgs"] for x in c]); wide = np.concatenate([x["big_input_imgs"] for x in c])
tcs = np.concatenate([x["traffic_convention"] for x in c])
feats = np.stack(keep["features_buffer"]); rng = np.random.default_rng(0)
# desire는 0/1 원-핫 펄스다. 전부 0으로 보정하면 MinMax 범위가 [0, 0]이 되어 NPU 모델이 펄스를
# 못 보고(2026-09-27 실차: 차선 변경 desire_state가 끝내 0), 차선 변경을 스스로 하지 않는다.
# 절반은 0, 절반은 25칸 중 한 칸에 펄스 하나(주로 차선 변경 3·4)를 넣어 범위를 [0, 1]로 잡는다.
# 전용 생성기를 써서 features 표본(rng)을 건드리지 않는다. 이 순서가 배포 axmodel의 보정 데이터다.
desire_rng = np.random.default_rng(0)
desire_cal = np.zeros((len(road), 25, 8), np.float32)
for i in range(1, len(road), 2):
    d = desire_rng.choice([1, 2, 3, 4, 5, 6], p=[0.1, 0.1, 0.3, 0.3, 0.1, 0.1])
    desire_cal[i, desire_rng.integers(0, 25), d] = 1.0
cal = {"input_imgs": road.astype(np.float32), "big_input_imgs": wide.astype(np.float32),
       "desire": desire_cal,
       "features_buffer": feats[rng.integers(0, len(feats), len(road))],
       "traffic_convention": tcs}
os.makedirs("calib", exist_ok=True)
for key, v in cal.items():
    with tarfile.open(f"calib/{key}.tar", "w") as t:
        for i, x in enumerate(v):
            b = io.BytesIO(); np.save(b, x[None].astype(np.float32)); raw = b.getvalue()
            ti = tarfile.TarInfo(f"{i:04d}.npy"); ti.size = len(raw); t.addfile(ti, io.BytesIO(raw))
print("eval frames", len(refs), "calib", len(road))
