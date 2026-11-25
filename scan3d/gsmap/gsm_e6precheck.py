#!/usr/bin/env python3
"""gsmap 2026-11-25 E6 pre-check (plan section 6): far-half extrapolation error of AI depth vs FFS, CPU.

  GSMAP_RUN=<run> python scan3d/gsmap/gsm_e6precheck.py OUT.json [--n 60] [--da3 DIR]
For ~--n keyframes with own FFS (fuse/ffs of the run): split the FFS pixels at their median depth into a near half (fit)
and a far half (test). Error = median |Z_ffs - Z_pred| / Z_ffs on the far half, per keyframe; reported median / mean over keyframes
and the share of far pixels with error > 5 %.
  A  MoGe affine (current align_fuse): robust scale + shift fitted on the near half of the SAME keyframe.
  B  MoGe scale-only pooled: s = median(Z_ffs / Z_moge) over the near halves of keyframes within +-12 (incl. itself), no shift.
  C  DA3 pose-conditioned raw depth (--da3 dir, already in pose units): no fit at all.
  C2 DA3 scale-only pooled +-12 (same as B on DA3), for reference.
"""
import json, os, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from align_fuse import robust_fit  # noqa: E402

R = Path(os.environ["GSMAP_RUN"])


def main():
    outp = sys.argv[1]; arg = lambda k, d=None: sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d  # noqa: E731
    n_want = int(arg("--n", 60)); da3 = arg("--da3")
    t0 = time.time()
    c = np.load(R / "dump_ba" / "cams.npz"); kf = c["kf"]
    ffs = {}
    for ci, i in enumerate(kf):
        Z = np.load(R / "fuse" / "ffs" / f"{i:05d}.npz")["Z"]
        if np.isfinite(Z).sum() > 5000:
            ffs[ci] = Z
    have = sorted(ffs); pick = [have[k] for k in np.linspace(0, len(have) - 1, min(n_want, len(have))).astype(int)]
    def load(dirn, i):
        return np.load(Path(dirn) / f"{i:05d}.npz")["Z"].astype(np.float32)
    srcs = {"moge": R / "depth_moge"}
    if da3:
        srcs["da3"] = Path(da3)
    near_ratio = {k: {} for k in srcs}   # per kf: median Z_ffs / Z_ai over the near half
    halves = {}
    for ci in have:
        Z = ffs[ci]; m = np.isfinite(Z); med = np.median(Z[m]); halves[ci] = (m & (Z <= med), m & (Z > med))
    for k, d in srcs.items():
        for ci in have:
            A = load(d, kf[ci]); nr = halves[ci][0] & np.isfinite(A) & (A > 0)
            if nr.sum() > 500:
                near_ratio[k][ci] = float(np.median(ffs[ci][nr] / A[nr]))
    res = {nm: [] for nm in ("A_moge_affine", "B_moge_scale_pooled12", "C_da3_raw", "C2_da3_scale_pooled12")}
    gt5 = {nm: [0, 0] for nm in res}
    for ci in pick:
        Z = ffs[ci]; nr, fr = halves[ci]
        preds = {}
        Am = load(srcs["moge"], kf[ci]); ok = nr & np.isfinite(Am) & (Am > 0)
        if ok.sum() > 500:
            s, b = robust_fit(Am[ok].astype(np.float64), Z[ok].astype(np.float64)); preds["A_moge_affine"] = s * Am + b
        nb = [near_ratio["moge"][j] for j in range(ci - 12, ci + 13) if j in near_ratio["moge"]]
        if nb:
            preds["B_moge_scale_pooled12"] = float(np.median(nb)) * Am
        if da3:
            Ad = load(srcs["da3"], kf[ci]); preds["C_da3_raw"] = Ad
            nb = [near_ratio["da3"][j] for j in range(ci - 12, ci + 13) if j in near_ratio["da3"]]
            if nb:
                preds["C2_da3_scale_pooled12"] = float(np.median(nb)) * Ad
        for nm, P in preds.items():
            f = fr & np.isfinite(P) & (P > 0)
            if f.sum() < 200:
                continue
            e = np.abs(Z[f] - P[f]) / Z[f]; res[nm].append(float(np.median(e))); gt5[nm][0] += int((e > 0.05).sum()); gt5[nm][1] += int(f.sum())
    out = dict(made=time.strftime("%F %T"), run=str(R), n_kf=len(pick), da3=da3, split="FFS median depth per keyframe; fit on near half, test on far half",
               far_half_rel_err={nm: dict(median=round(float(np.median(v)), 4) if v else None, mean=round(float(np.mean(v)), 4) if v else None,
                                          share_gt5pct=round(gt5[nm][0] / max(1, gt5[nm][1]), 4), n=len(v)) for nm, v in res.items()},
               runtime_s=round(time.time() - t0, 1))
    a = out["far_half_rel_err"]["A_moge_affine"]["median"]
    out["best_vs_affine"] = {nm: round(1 - v["median"] / a, 3) for nm, v in out["far_half_rel_err"].items() if v["median"] is not None and a}
    Path(outp).write_text(json.dumps(out, indent=1)); print(json.dumps(out), flush=True)


if __name__ == "__main__":
    main()
