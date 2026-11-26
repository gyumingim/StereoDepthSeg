# Fable audit6 chk.py two-sided agreement metric (mv() unchanged) applied to named depth sets, judged with the NEW BA poses.
# usage: chk2.py OUT.json name=dir [name=dir ...]
import os
for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"): os.environ[k] = "1"
import json, sys, time
from pathlib import Path
import numpy as np, cv2
cv2.setNumThreads(1)
V2 = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"; O = V2 / "ba_partners"
cn = np.load(O / "run" / "dump_ba" / "cams.npz"); kf = cn["kf"]; n = len(kf)
Kr = cn["Kr"]; W, H = [int(v) for v in cn["size"]]; fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
mx, my = cv2.initUndistortRectifyMap(cn["K"], cn["dist"], None, Kr, (W, H), cv2.CV_32FC1)
vm = cv2.erode(((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
def load(d):
    Z = np.full((n, H, W), np.nan, np.float16)
    for ci, i in enumerate(kf):
        f = Path(d) / f"{int(i):05d}.npz"
        if f.exists(): Z[ci] = np.load(f)["Z"]
    return Z
sets = dict(s.split("=", 1) for s in sys.argv[2:])
T = cn["T"].astype(np.float64); Tw = np.linalg.inv(T); cen = T[:, :3, 3]
out = dict(made=time.strftime("%F %T"), judge=str(O / "run/dump_ba/cams.npz"), rule="audit6 chk.py mv(): query step 6 in vm; fail = measure3 rule; "
           "sym_agree = of view pairs >= 10 cm apart with |zj-zr| < 15 % zr, share within 2 cm + 1 %", sets={})
t0 = time.time()
for nm, d in sets.items():
    Z = load(d); has = np.isfinite(Z).reshape(n, -1).any(1); js = np.flatnonzero(has); res = np.zeros((n, 4), np.int64)
    for ci in range(n):
        z0 = Z[ci, ::6, ::6].astype(np.float32); val = np.isfinite(z0) & vm[::6, ::6]
        if not val.any(): continue
        v, u = np.nonzero(val); z = z0[v, u].astype(np.float64); ut = u * 6.0; vt = v * 6.0
        X = np.stack([(ut - cx) / fx * z, (vt - cy) / fy * z, z], 1) @ T[ci, :3, :3].T + T[ci, :3, 3]
        jj = js[js != ci]
        Xj = np.einsum("vij,mj->vmi", Tw[jj, :3, :3], X) + Tw[jj, None, :3, 3]; zj = Xj[..., 2]; zc = np.maximum(zj, 1e-3)
        pu = np.round(fx * Xj[..., 0] / zc + cx).astype(np.int64); pv = np.round(fy * Xj[..., 1] / zc + cy).astype(np.int64)
        ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
        zr = Z[jj[:, None], pv.clip(0, H - 1), pu.clip(0, W - 1)].astype(np.float64); zr[~ok] = np.nan
        fin = np.isfinite(zr); chk = fin.sum(0); viol = (fin & (zj < zr - np.maximum(0.03 * zr, 0.03))).sum(0)
        far = (np.linalg.norm(cen[jj] - cen[ci], axis=1) >= 0.10)[:, None]
        near = fin & far & (np.abs(zj - zr) < 0.15 * zr); agr = near & (np.abs(zj - zr) < 0.02 + 0.01 * zr)
        res[ci] = [int(((viol >= 2) & (viol >= 0.3 * chk)).sum()), len(z), int(agr.sum()), int(near.sum())]
    out["sets"][nm] = dict(dir=d, fail_pct=round(100 * res[:, 0].sum() / res[:, 1].sum(), 2), n_query=int(res[:, 1].sum()),
                           sym_agree_2cm1pct=round(res[:, 2].sum() / res[:, 3].sum(), 4), n_near=int(res[:, 3].sum()))
    print(nm, out["sets"][nm], f"{time.time() - t0:.0f}s", flush=True)
    Path(sys.argv[1]).write_text(json.dumps(out, indent=1))
