"""lgpost: CPU post without N^2 log. sc_ij = log(A_ij*BT_ji) + ls0_i + ls1_j is monotone in A_ij*BT_ji*sig(z1_j) per row and in
A_ij*BT_ji*sig(z0_i) per column -> argmax on products (float32), log only once per row for the threshold.
Compares match sets vs current post (lgnpu.post, = nncrun lgpost 1/2) on PC fp32 L6r ONNX outputs, 13 eval pairs."""
import os, sys, json, time, hashlib, numpy as np
os.environ.setdefault("LGLN", "native")
sys.path.insert(0, "/mnt/c/Users/a3162/scan3d_handoff/npu_upload/lgnpu")
import torch, onnxruntime as ort
from lgnpu import pack, post, load_net
from lgpost import data, mset_md5
net = load_net(); s = ort.InferenceSession("/mnt/c/Users/a3162/scan3d_handoff/npu_upload/lgpost/up/L6r.onnx")
def prod_post(o, N):
    A, BT, z0, z1 = o[0, :, :N], o[0, :, N:2 * N], o[0, :, 2 * N], o[0, :, 2 * N + 1]
    s0, s1 = 1 / (1 + np.exp(-z0)), 1 / (1 + np.exp(-z1))
    P = A * BT.T                                  # float32, as phone C++
    Q0 = P * s1[None, :]; m0 = Q0.argmax(1)
    Q1 = P * s0[:, None]; m1 = Q1.argmax(0)
    rmax = np.log(np.maximum(Q0[np.arange(N), m0].astype(np.float64), 1e-30)) + np.log(s0.astype(np.float64))
    ok = (m1[m0] == np.arange(N)) & (rmax > np.log(0.1))
    return {(i, int(m0[i])) for i in np.nonzero(ok)[0]}
d = data("ev", 512); rows = []
for i in range(13):
    x = pack(net, *[d[k][i:i + 1] for k in ("kpts0", "kpts1", "desc0", "desc1")]).numpy()
    o = s.run(None, {"x": x})[0]
    a, b = post(o, 512), prod_post(o, 512)
    rows.append(dict(pair=i, n=len(a), same=mset_md5(a) == mset_md5(b), diff=len(a ^ b)))
print(json.dumps(rows)); json.dump(dict(made=time.strftime("%F %T"), rows=rows, all_same=all(r["same"] for r in rows)), open("cpu_prod_check.json", "w"), indent=1)
print("ALL_SAME", all(r["same"] for r in rows))
