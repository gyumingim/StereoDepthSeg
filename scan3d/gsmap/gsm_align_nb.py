#!/usr/bin/env python3
"""gsmap 2026-11-25: MoGe-2 alignment for keyframes WITHOUT own FFS, to the FFS of neighbouring keyframes (not one global scale).

  GSMAP_RUN=<run> ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_align_nb.py [--nb 12] [--min-px 3000] [--max-rel 0.10]

For each keyframe whose fuse/ffs has no valid pixel: FFS points of keyframes within +-nb indices (stride 2 px) are projected
into it (z-buffer: nearest), robust scale+shift fit of the raw MoGe depth (depth_moge) to them (align_fuse.robust_fit).
Weak fit (< --min-px pixels or median |residual|/Z > --max-rel) -> the keyframe gets NO MoGe depth (Zai = NaN), so it drops
out of the depth loss and the init. Rewrites fuse/moge/<kf>.npz (Z, F, Zai) in place for those keyframes; writes
fuse/align_nb.json. Edge cut and limits as align_fuse (median5 3 %, 0.05..4 m).
"""
import argparse, json, os, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from align_fuse import robust_fit  # noqa: E402

R = Path(os.environ["GSMAP_RUN"])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--nb", type=int, default=12); ap.add_argument("--min-px", type=int, default=3000)
    ap.add_argument("--max-rel", type=float, default=0.10)
    a = ap.parse_args(); t0 = time.time()
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    FF = [np.load(R / "fuse" / "ffs" / f"{i:05d}.npz")["Z"] for i in kf]
    has = np.array([np.isfinite(z).any() for z in FF])
    vv, uu = np.mgrid[0:H:2, 0:W:2]
    out = []
    for ci in np.nonzero(~has)[0]:
        zb = np.full((H, W), np.inf, np.float32)
        for j in range(max(0, ci - a.nb), min(n, ci + a.nb + 1)):
            if not has[j]:
                continue
            z = FF[j][vv, uu]; m = np.isfinite(z)
            X = np.stack([(uu[m] - K[0, 2]) / K[0, 0] * z[m], (vv[m] - K[1, 2]) / K[1, 1] * z[m], z[m]], 1) @ T[j][:3, :3].T + T[j][:3, 3]
            Xc = (X - T[ci][:3, 3]) @ T[ci][:3, :3]; zc = Xc[:, 2]; g = zc > 0.1
            u = np.round(K[0, 0] * Xc[g, 0] / zc[g] + K[0, 2]).astype(int); v = np.round(K[1, 1] * Xc[g, 1] / zc[g] + K[1, 2]).astype(int); zg = zc[g]
            ok = (u >= 0) & (u < W) & (v >= 0) & (v < H)
            np.minimum.at(zb, (v[ok], u[ok]), zg[ok].astype(np.float32))
        Zm = np.load(R / "depth_moge" / f"{kf[ci]:05d}.npz")["Z"].astype(np.float32)
        m = np.isfinite(zb) & np.isfinite(Zm) & (Zm > 0)
        rec = dict(idx=int(ci), kf=int(kf[ci]), n_px=int(m.sum()))
        f = R / "fuse" / "moge" / f"{kf[ci]:05d}.npz"; d = dict(np.load(f))
        if m.sum() >= a.min_px:
            x, y = Zm[m].astype(np.float64), zb[m].astype(np.float64); s, b = robust_fit(x, y); r = np.abs(y - (s * x + b)) / y
            rec.update(s=s, b=b, rel_med=float(np.median(r)))
        if "rel_med" in rec and rec["rel_med"] <= a.max_rel:
            Za = s * Zm + b; med = cv2.medianBlur(np.nan_to_num(Za, nan=0).astype(np.float32), 5)
            ok = np.isfinite(Za) & (Za > 0.05) & (Za < 4.0) & (np.abs(Za - med) / np.maximum(Za, 1e-3) < 0.03)
            d["Zai"] = Za.astype(np.float16); d["Z"] = np.where(ok, Za, np.nan).astype(np.float32); d["F"] = np.where(ok, 2, 0).astype(np.uint8); rec["used"] = True
        else:
            d["Zai"] = np.full((H, W), np.nan, np.float16); d["Z"] = np.full((H, W), np.nan, np.float32); d["F"] = np.zeros((H, W), np.uint8); rec["used"] = False
        np.savez_compressed(f, **d); out.append(rec)
    res = dict(made=time.strftime("%F %T"), args=vars(a), n_no_ffs=len(out), n_aligned=sum(r["used"] for r in out),
               n_dropped=sum(not r["used"] for r in out), rel_med_median=float(np.median([r["rel_med"] for r in out if "rel_med" in r])) if out else None,
               runtime_s=round(time.time() - t0, 1), per_kf=out)
    (R / "fuse" / "align_nb.json").write_text(json.dumps(res, indent=1)); print(json.dumps({k: v for k, v in res.items() if k != "per_kf"}), flush=True)


if __name__ == "__main__":
    main()
