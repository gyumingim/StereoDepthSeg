"""lgpost synthetic random calibration (no user data). python gen_cal.py lg_wr.npy N masked(0/1) outdir [n=40]
kpts U[-1,1]^2, desc = unit gaussian 64-d, x = d0|d1|cos p0 x2|sin p0 x2|cos p1 x2|sin p1 x2 (p = k @ W, W [2,48]);
masked: + column v, real count U{N/2..N}, padding rows: desc 0, cos 1, sin 0, v 0."""
import sys, os, numpy as np
W = np.load(sys.argv[1]).astype(np.float32); N = int(sys.argv[2]); msk = sys.argv[3] == "1"; od = sys.argv[4]
n = int(sys.argv[5]) if len(sys.argv) > 5 else 40
os.makedirs(od, exist_ok=True)
rng = np.random.default_rng(1234)
def nd(): d = rng.standard_normal((N, 64)).astype(np.float32); return d / np.linalg.norm(d, axis=1, keepdims=True)
for i in range(n):
    k0, k1 = (rng.random((N, 2), dtype=np.float32) * 2 - 1), (rng.random((N, 2), dtype=np.float32) * 2 - 1)
    d0, d1 = nd(), nd()
    p0, p1 = k0 @ W, k1 @ W
    f = lambda t: np.concatenate([t, t], 1)
    x = np.concatenate([d0, d1, f(np.cos(p0)), f(np.sin(p0)), f(np.cos(p1)), f(np.sin(p1))], 1).astype(np.float32)
    if msk:
        nv = int(rng.integers(N // 2, N + 1))
        x[nv:] = 0; x[nv:, 128:224] = 1; x[nv:, 320:416] = 1
        v = np.zeros((N, 1), np.float32); v[:nv] = 1
        x = np.concatenate([x, v], 1)
    x[None].astype(np.float32).tofile(f"{od}/c{i:03d}.raw")
print("ok", od, n, x.shape)
