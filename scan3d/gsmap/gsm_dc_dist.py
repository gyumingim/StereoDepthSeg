#!/usr/bin/env python3
"""gsmap 11-27 dc: probe follow-up. Error vs BA2 sparse targets on FFS-empty pixels, binned by the distance (px) to the nearest valid FFS
pixel, for every probe variant dir (Df = completion with the full FFS prior, Af = DA3 affine). Read-only on the probe dirs.
  python scan3d/gsmap/gsm_dc_dist.py OUT.json probe probe_g probe_gdg probe_mdc
"""
import json, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
V2 = HERE.parent / "runs/gsmap_20261125_121801_v2"; DC = V2 / "dc"
DUMP = V2 / "ba_partners2/run/dump_ba"; FUSED = V2 / "ffsonly_ba2/fused"; SPARSE = V2 / "w5/sparse"
BINS = [0, 24, 64, 128, 256, 2000]


def main():
    out = Path(sys.argv[1]); names = sys.argv[2:]
    c = np.load(DUMP / "cams.npz"); W, H = [int(v) for v in c["size"]]
    mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, c["Kr"], (W, H), cv2.CV_32FC1)
    vmask = cv2.erode(((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    res = dict(made=time.strftime("%F %T"), inputs=[str(DC / n) for n in names] + [str(SPARSE)], bins_px=BINS, variants={})
    acc = {}
    for n in names:
        for f in sorted((DC / n).glob("[0-9]*.npz")):
            i = int(f.stem); d = np.load(f); fu = np.load(FUSED / f"{i:05d}.npz")
            valid = (np.where(vmask, fu["F"], 0) == 1) & np.isfinite(fu["Z"]) & (fu["Z"] > 0)
            dist = cv2.distanceTransform((~valid).astype(np.uint8), cv2.DIST_L2, 5)
            Zb = np.load(SPARSE / f"{i:05d}.npz")["Z"]; bm = np.isfinite(Zb) & (Zb > 0) & ~valid & vmask & np.isfinite(d["Af"]) & np.isfinite(d["Df"])
            for k, P in (("dc", d["Df"]), ("da3", d["Af"])):
                e = np.abs(P[bm] - Zb[bm]) / Zb[bm]
                acc.setdefault((n, k), []).append(np.stack([e, dist[bm]], 1))
    for (n, k), L in acc.items():
        a = np.concatenate(L); r = {}
        for lo, hi in zip(BINS[:-1], BINS[1:]):
            m = (a[:, 1] >= lo) & (a[:, 1] < hi)
            if m.sum():
                r[f"{lo}-{hi}px"] = dict(n=int(m.sum()), med_pct=round(float(np.median(a[m, 0]) * 100), 2), gt5_pct=round(float((a[m, 0] > 0.05).mean() * 100), 1))
        res["variants"].setdefault(n, {})[k] = r
    out.write_text(json.dumps(res, indent=1)); print(json.dumps(res["variants"], indent=1))


if __name__ == "__main__":
    main()
