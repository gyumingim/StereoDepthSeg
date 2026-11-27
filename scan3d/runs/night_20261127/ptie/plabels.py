#!/usr/bin/env python3
"""Night 11-27 item 3: FFS-supported planar segment labels per keyframe for gsm_train_ptie.py --ptie (CPU).

  python plabels.py OUT     -> OUT/KF.npz (L int16, 0 = none), OUT/labels.json (+ OUT/viz_*.jpg for a few keyframes)

Same segments and plane gates as W-fill (../wfill/wfill.py P): Felzenszwalb segment, plane fitted to its FFS pixels (fused F == 1 of
ffsonly2), support >= 300 px and >= 5 % of the segment, inlier share >= 90 %, inlier RMS <= max(0.6 cm, 0.4 % Z), spread >= 10 px.
Label region = segment pixels inside the support's convex hull (dilated 15 px), inside the valid mask. No hole needed (unlike W-fill).
Extra guard against flattening real objects: segments whose FFS plane residual p95 > 2 x thr are skipped.
"""
import json, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "wfill"))
import wfill as WF  # noqa: E402


def labels(rgb, Z, valid, vmask, R, rng):
    from scipy import ndimage
    from skimage.segmentation import felzenszwalb
    P = WF.P; seg = felzenszwalb(rgb, scale=P["scale"], sigma=P["sigma"], min_size=P["min_size"])
    L = np.zeros(Z.shape, np.int16); k = 0; ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * P["hull_dil"] + 1,) * 2)
    for l, sl in enumerate(ndimage.find_objects(seg + 1)):
        if sl is None:
            continue
        m = seg[sl] == l; A = int(m.sum())
        if A < P["min_area"]:
            continue
        sup = m & valid[sl]; ns = int(sup.sum())
        if ns < P["min_sup"] or ns < P["min_sup_frac"] * A:
            continue
        yy, xx = np.nonzero(sup); X = R[sl][sup] * Z[sl][sup][:, None]; zmed = float(np.median(X[:, 2]))
        thr = max(P["rms_abs"], P["rms_rel"] * zmed); n, d = WF.ransac(X, thr, P["ransac_it"], rng); r = X @ n + d; inl = np.abs(r) < thr
        ev = np.linalg.eigvalsh(np.cov(np.c_[xx, yy].T.astype(np.float64)))
        if inl.mean() < P["inl_frac"] or np.sqrt(np.mean(r[inl] ** 2)) > thr or np.sqrt(max(ev[0], 0)) < P["min_spread_px"] or np.percentile(np.abs(r), 95) > 2 * thr:
            continue
        hm = np.zeros(m.shape, np.uint8); cv2.fillConvexPoly(hm, cv2.convexHull(np.c_[xx, yy].astype(np.int32)), 1)
        reg = m & cv2.dilate(hm, ker).astype(bool) & vmask[sl]
        k += 1; blk = L[sl]; blk[reg] = k
    return L, k


def main():
    out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True); assert not (out / "labels.json").exists(), "never overwrite"
    t0 = time.time(); cx = WF.Ctx(); rng = np.random.default_rng(0); sh, ks = [], []
    for j in range(cx.n):
        i = int(cx.kf[j]); q = out / f"{i:05d}.npz"
        if q.exists():
            L = np.load(q)["L"]; sh.append(float((L > 0).sum() / cx.vmask.sum())); ks.append(int(L.max())); continue
        rgb, Z, F, S = cx.load(i); valid = (F == 1) & np.isfinite(Z) & (Z > 0)
        L, k = labels(rgb, Z, valid, cx.vmask, cx.R, rng); np.savez_compressed(q, L=L)
        sh.append(float((L > 0).sum() / cx.vmask.sum())); ks.append(k)
        if j in (36, 72, 200, 320, 470):
            col = cv2.applyColorMap(((L.astype(np.int32) * 37) % 255).astype(np.uint8), cv2.COLORMAP_HSV); col[L == 0] = 0
            cv2.imwrite(str(out / f"viz_{i:05d}.jpg"), np.hstack([cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), cv2.addWeighted(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), 0.5, col, 0.5, 0)])[::2, ::2])
        if j % 50 == 0:
            print(f"{j}/{cx.n} kf {i} segs {k} share {sh[-1]:.3f} {time.time() - t0:.0f}s", flush=True)
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__).resolve()), fused=str(WF.FUSED), params=WF.P, n=cx.n,
               label_px_share_mean=float(np.mean(sh)), segs_per_kf_mean=float(np.mean(ks)), runtime_s=round(time.time() - t0, 1))
    (out / "labels.json").write_text(json.dumps(res, indent=1)); print(json.dumps(res))


if __name__ == "__main__":
    main()
