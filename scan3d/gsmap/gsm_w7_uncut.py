#!/usr/bin/env python3
"""gsmap W7 2026-11-27: recompute the wide/relax FFS pairs of the probe keyframes WITHOUT the per-pixel sigma cap and zmax
(identical gsm_ffs_pairs3.partial_depth otherwise: 25 deg epipole cone, FFS uncertainty mask, Z >= 0.3 m, same model, BA2 poses),
so the probe can split "no stored pair value" into sigma-cut / range-cut / truly no FFS output.
  python gsm_w7_uncut.py   -> runs/.../w7/uncut/<kf>.npz (Z float32 uncut, sin, B, f, partner) + uncut_info.json
Sanity: where the stored pair value exists, the uncut value must match it (median |dz|/z reported)."""
import json, sys, time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/gsmap"
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
import gsm_ba_partners as BP  # noqa: E402

V2 = BP.V2; BA2 = V2 / "ba_partners2"; SRC = V2 / "ffs_relax/src/wide"; OUT = V2 / "w7/uncut"


def main():
    import cv2, lib_ffs
    import gsm_ffs_pairs2 as P2, gsm_ffs_pairs3 as P3, motion_stereo as ms
    t0 = time.time(); OUT.mkdir(parents=True, exist_ok=True)
    pr = json.loads((V2 / "w7/probe.json").read_text()); frames = pr["kf_frames"]
    a = BP.ffs_args(run=V2 / "ba_partners" / "run", out=OUT); kf, _Tk, Kr, W0, H0, t_ns, K, dist, Tv = P2.load_common(a)
    pa = np.load(BA2 / "poses_all.npz"); TmD = {int(f): T for f, T in zip(pa["frames"], pa["T"])}
    A = SimpleNamespace(theta_min=25.0, cap=1e9, zmax=1e9, z_near=1.0, f_scale=1.0, max_px=1.0e6)
    plan = {}; psrc = {}
    for fi in frames:
        f = SRC / f"{fi:05d}.npz"
        if f.exists() and fi in TmD:
            d = np.load(f); js = [int(j) for j in d["partner"] if int(j) in TmD or (int(j) in Tv and fi in Tv)]
            if js:
                plan[fi] = js
            for j in js:   # partner without a BA2 pose: VIO relative motion anchored on the keyframe BA2 pose (as gsm_ffs_relax plan)
                if j not in TmD:
                    TmD[j] = TmD[fi] @ np.linalg.inv(Tv[fi]) @ Tv[j]; psrc[j] = "vio"
    need = set(plan) | {j for v in plan.values() for j in v}; last = max(need)
    model = lib_ffs.load(valid_iters=8); cap = cv2.VideoCapture(str(BP.R1 / "rec" / "cam5.mp4")); buf = {}; fidx = -1
    while fidx < last:
        r_, fr = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(fr, (W0, H0), interpolation=cv2.INTER_AREA) if fr.shape[1] != W0 else fr
    cap.release(); info = {}
    for fi, js in plan.items():
        Zs, Ss, Bs, rows = [], [], [], []
        old = np.load(SRC / f"{fi:05d}.npz"); op = [int(x) for x in old["partner"]]
        for j in js:
            R, t = ms.rel_pose(TmD[fi], TmD[j])
            Z, st, B, f0, inf = P3.partial_depth(model, lib_ffs.infer, buf[fi], buf[j], R, t, K, dist, W0, H0, A, 8)
            Zo = old["Z"][op.index(j)].astype(np.float32); both = np.isfinite(Zo) & np.isfinite(Z)
            rows.append(dict(j=j, uncut_valid=round(float(np.isfinite(Z).mean()), 4), stored_valid=round(float(np.isfinite(Zo).mean()), 4),
                             stored_vs_uncut_med_rel_pct=round(float(np.median(np.abs(Z[both] - Zo[both]) / Zo[both]) * 100), 3) if both.any() else None,
                             stored_not_in_uncut=int((np.isfinite(Zo) & ~np.isfinite(Z)).sum()), B_cm=round(B * 100, 2), pose=psrc.get(j, "ba2")))
            Zs.append(Z); Ss.append(st); Bs.append(B)
        np.savez_compressed(OUT / f"{fi:05d}.npz", Z=np.stack(Zs).astype(np.float32), sin=np.stack(Ss).astype(np.float16), B=np.array(Bs),
                            f=np.full(len(Bs), float(K[0, 0])), partner=np.array(js))
        info[fi] = rows; print(fi, rows, f"{time.time() - t0:.0f}s", flush=True)
    (OUT / "uncut_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), poses=str(BA2 / "poses_all.npz"), src=str(SRC), args=vars(A),
                                                        note=__doc__, per_kf={str(k): v for k, v in info.items()}, runtime_s=round(time.time() - t0, 1)), indent=1, default=float))


if __name__ == "__main__":
    import gpu_guard
    with gpu_guard.job("gsmap w7 uncut", torch_cap=True):
        main()
