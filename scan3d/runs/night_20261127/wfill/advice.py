#!/usr/bin/env python3
"""W-fill closed (NO-GO): turn its segment statistics into recording advice. CPU.
For the ROI footprint keyframes (cabinet / wall / floor) and every 8th keyframe: per Felzenszwalb segment (same params as wfill.P)
with hole pixels: area, FFS support share, fail reason. Output: hole pixels by segment support share bins and by reason -> advice.json
"""
import json, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wfill as WF  # noqa: E402


def main():
    from scipy import ndimage
    from skimage.segmentation import felzenszwalb
    import plane_diag as PD
    out = Path(__file__).resolve().parent / "advice.json"; t0 = time.time()
    cx = WF.Ctx(); RS = PD.roi_setup(); kfi, T, Kr, (W, H) = PD.cams(); rcp = PD.rays(Kr, W, H)
    groups = {nm: sorted(ci for ci in range(cx.n) if PD.footprint(R, T[ci], rcp, W, H).sum() >= PD.MIN_FP) for nm, R in RS.items()}
    groups["every8"] = list(range(0, cx.n, 8))
    bins = [0, 0.001, 0.01, 0.05, 0.10, 0.25, 1.01]; res = {}
    for g, idxs in groups.items():
        hole_by_bin = np.zeros(len(bins) - 1); seg_by_bin = np.zeros(len(bins) - 1); n_seg = 0; hole_tot = 0; fill_tot = 0
        rng = np.random.default_rng(0); sup_px = []
        for j in idxs:
            i = int(cx.kf[j]); rgb, Z, F, S = cx.load(i); valid = (F == 1) & np.isfinite(Z) & (Z > 0)
            seg = felzenszwalb(rgb, scale=WF.P["scale"], sigma=WF.P["sigma"], min_size=WF.P["min_size"])
            Zf, _, st = WF.fill(rgb, Z, valid, cx.vmask, cx.R, rng, seg); fill_tot += int(np.isfinite(Zf).sum())
            for l, sl in enumerate(ndimage.find_objects(seg + 1)):
                if sl is None:
                    continue
                m = seg[sl] == l; A = int(m.sum()); hole = int((m & ~valid[sl] & cx.vmask[sl]).sum())
                if A < WF.P["min_area"] or hole == 0:
                    continue
                sf = (m & valid[sl]).sum() / A; b = np.searchsorted(bins, sf, side="right") - 1
                hole_by_bin[b] += hole; seg_by_bin[b] += 1; n_seg += 1; hole_tot += hole; sup_px.append(int((m & valid[sl]).sum()))
        res[g] = dict(n_kf=len(idxs), n_seg_with_holes=n_seg, hole_px=hole_tot, filled_px=fill_tot, filled_share_of_holes=fill_tot / max(hole_tot, 1),
                      bins=[f"{bins[k]:.3f}-{bins[k + 1]:.3f}" for k in range(len(bins) - 1)],
                      hole_share_by_support_bin=(hole_by_bin / max(hole_tot, 1)).round(4).tolist(), seg_count_by_support_bin=seg_by_bin.astype(int).tolist(),
                      hole_share_support_lt5pct=float(hole_by_bin[:3].sum() / max(hole_tot, 1)))
        print(g, json.dumps(res[g]), flush=True)
    doc = dict(made=time.strftime("%F %T"), script=str(Path(__file__).resolve()), params=WF.P,
               gate_support="a segment needs >= 300 FFS px AND >= 5 % of its area with FFS before its plane can be fitted",
               groups=res, runtime_s=round(time.time() - t0, 1))
    out.write_text(json.dumps(doc, indent=1)); print("done", doc["runtime_s"])


if __name__ == "__main__":
    main()
