"""lgpost: Lab eais golden (a16 sim) vs PC fp32 ORT on the same (synthetic random) golden input. python gold_post.py DIR ONNX"""
import sys, glob, json, numpy as np, onnxruntime as ort
d, onx = sys.argv[1], sys.argv[2]
s = ort.InferenceSession(onx); sh = s.get_inputs()[0].shape; N, C = sh[1], sh[2]
x = np.fromfile(glob.glob(d + "/*input_data_float32.bin")[0], np.float32).reshape(1, N, C)
r = s.run(None, {"x": x})[0]
g = np.fromfile(glob.glob(d + "/*golden_data_float*.bin")[0], np.float32).reshape(r.shape)
nv = int(x[0, :, 512].sum()) if C == 513 else N
LN = np.log(0.1)
def fin(o):
    o = o[0]; m0 = np.rint(o[:, 0]).astype(int); m1 = np.rint(o[:, 1]).astype(int)
    return {(i, int(m0[i])) for i in range(nv) if 0 <= m0[i] < nv and m1[m0[i]] == i and o[i, 2] > LN}
def rows(o):
    if o.shape[2] != 2 * N + 2: return np.rint(o[0, :nv, 0]).astype(int), np.rint(o[0, :nv, 1]).astype(int), o[0, :nv, 2]
    A, BT, z0, z1 = o[0, :, :N], o[0, :, N:2 * N], o[0, :, 2 * N], o[0, :, 2 * N + 1]
    ls = lambda z: -np.logaddexp(0, -z.astype(np.float64))
    sc = np.log(np.maximum((A * BT.T).astype(np.float64), 1e-30)) + ls(z0)[:, None] + ls(z1)[None, :]
    return sc.argmax(1), sc.argmax(0), sc.max(1)
def post(o):
    A, BT, z0, z1 = o[0, :, :N], o[0, :, N:2 * N], o[0, :, 2 * N], o[0, :, 2 * N + 1]
    ls = lambda z: -np.logaddexp(0, -z.astype(np.float64))
    sc = np.log(np.maximum((A * BT.T).astype(np.float64), 1e-30)) + ls(z0)[:, None] + ls(z1)[None, :]
    m0, m1 = sc.argmax(1), sc.argmax(0)
    ok = (m1[m0] == np.arange(N)) & (np.exp(sc.max(1)) > 0.1)
    return {(i, int(m0[i])) for i in np.nonzero(ok)[0]}
f = post if r.shape[2] == 2 * N + 2 else fin
a, b = f(r), f(g)
ra, rb = rows(r), rows(g)
res = dict(row_argmax_agree=float((ra[0] == rb[0]).mean()), col_argmax_agree=float((ra[1] == rb[1]).mean()),
           rowmax_absdiff_median=float(np.median(np.abs(ra[2] - rb[2]))), rowmax_fp32_median=float(np.median(ra[2])),dir=d, nvalid=nv, fp32=len(a), sim=len(b), agree=len(a & b) / max(1, len(a | b)),
           idx_exact_m0=float((np.rint(r[0, :nv, 0]) == np.rint(g[0, :nv, 0])).mean()) if f is fin else None,
           score_maxabs=float(np.abs(r[0, :nv, 2] - g[0, :nv, 2]).max()) if f is fin else None)
print(json.dumps(res))
