"""Is the fx drift explained by board distance (defocus, focus 0.5 dpt = 2 m) rather than tilt? (2026-10-24, no new data)

1. k12_notan full fit (59 poses) → per cluster: tilt, mean corner depth (board units from json square size, estimate).
2. Cells: tilted clusters (>= 25 deg) split at their median depth x tilt 25-45 / >= 45; fit = all frontal (< 25 deg) +
   that cell (as split_test.py). Frontal near/far (split at frontal median): fit = that frontal half + all tilted.
3. Per tilted cluster: fx from frontal + that single cluster (leave-group-in), then OLS fx ~ a + b*depth + c*tilt.
4. Sharpness per recording: one picked frame, per corner a 31x31 patch: edge sigma = contrast / (max |grad| * sqrt(2 pi))
   (blurred-step width, px; median over corners). Compared with depth and a thin-lens defocus estimate.
→ results/dist_tilt.json
"""
import json
import pickle
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
S3D = HERE.parents[1]
sys.path.insert(0, str(HERE))
import calib  # noqa: E402

M = "k12_notan"


def depth_of(v, rv, tv):
    R, _ = cv2.Rodrigues(rv)
    X = (R @ v["obj"].reshape(-1, 3).T).T + np.ravel(tv)
    return float(X[:, 2].mean())


def sharpness(rec, frame_idx, pts):
    cap = cv2.VideoCapture(str(S3D / "recordings" / rec / "cam5.mp4"))
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ok, f = cap.read()
    if not ok:
        return None
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).astype(float)
    gy, gx = np.gradient(g)
    mag = np.hypot(gx, gy)
    s = []
    for x, y in pts:
        x, y = int(round(x)), int(round(y))
        if 15 <= x < g.shape[1] - 15 and 15 <= y < g.shape[0] - 15:
            p = g[y - 15:y + 16, x - 15:x + 16]
            m = mag[y - 15:y + 16, x - 15:x + 16]
            c = np.percentile(p, 95) - np.percentile(p, 5)
            s.append(c / (np.percentile(m, 99) * np.sqrt(2 * np.pi)))
    return float(np.median(s)) if s else None


def fit_fx(vs):
    r = calib.fit(vs, M)
    return float(r[1][0, 0]), float(r[1][0, 2]), float(r[1][1, 2]), float(r[0])


def main():
    views, clusters, members = calib.build()
    r = calib.fit(views, M)
    tilt, depth = {}, {}
    for v, rv, tv in zip(views, r[3], r[4]):
        tilt.setdefault(v["cluster"], []).append(calib.tilt_dist(rv, tv)[0])
        depth.setdefault(v["cluster"], []).append(depth_of(v, rv, tv))
    tilt = {c: float(np.mean(x)) for c, x in tilt.items()}
    depth = {c: float(np.mean(x)) for c, x in depth.items()}
    front = [c for c in clusters if tilt[c] < 25]
    tilted = [c for c in clusters if tilt[c] >= 25]
    md_t = float(np.median([depth[c] for c in tilted]))
    md_f = float(np.median([depth[c] for c in front]))
    print(f"clusters {len(clusters)}: frontal {len(front)} (median depth {md_f:.2f}), tilted {len(tilted)} (median depth {md_t:.2f})")
    print(f"corr(depth, tilt) over tilted: {np.corrcoef([depth[c] for c in tilted], [tilt[c] for c in tilted])[0, 1]:.2f}")
    out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), model=M, median_depth_tilted_m=md_t, median_depth_frontal_m=md_f,
               cluster=dict(tilt=tilt, depth_m=depth), cells={})
    sel = lambda cs: [v for v in views if v["cluster"] in cs]
    cells = {}
    for dn, dsel in (("near", lambda c: depth[c] < md_t), ("far", lambda c: depth[c] >= md_t)):
        for tn, tsel in (("25-45", lambda c: tilt[c] < 45), (">=45", lambda c: tilt[c] >= 45)):
            cells[f"tilted {tn} {dn}"] = (set(front) | {c for c in tilted if dsel(c) and tsel(c)},
                                          [c for c in tilted if dsel(c) and tsel(c)])
    cells["frontal near (+all tilted)"] = (set(tilted) | {c for c in front if depth[c] < md_f}, [c for c in front if depth[c] < md_f])
    cells["frontal far (+all tilted)"] = (set(tilted) | {c for c in front if depth[c] >= md_f}, [c for c in front if depth[c] >= md_f])
    cells["tilted near (all tilts)"] = (set(front) | {c for c in tilted if depth[c] < md_t}, [c for c in tilted if depth[c] < md_t])
    cells["tilted far (all tilts)"] = (set(front) | {c for c in tilted if depth[c] >= md_t}, [c for c in tilted if depth[c] >= md_t])
    for name, (cs, cell) in cells.items():
        fx, cx, cy, rms = fit_fx(sel(cs))
        out["cells"][name] = dict(n=len(cell), fx=fx, cx=cx, cy=cy, rms=rms,
                                  depth_mean=float(np.mean([depth[c] for c in cell])) if cell else None,
                                  tilt_mean=float(np.mean([tilt[c] for c in cell])) if cell else None)
        print(f"{name:28s} n={len(cell):2d} depth {out['cells'][name]['depth_mean']:.2f} tilt {out['cells'][name]['tilt_mean']:4.1f} "
              f"fx {fx:.1f} cx {cx:.1f} cy {cy:.1f}")
    # leave-group-in per tilted cluster
    rows = []
    for c in tilted:
        fx, cx, cy, rms = fit_fx(sel(set(front) | {c}))
        rows.append((c, depth[c], tilt[c], fx))
    X = np.c_[np.ones(len(rows)), [r_[1] for r_ in rows], [r_[2] for r_ in rows]]
    y = np.array([r_[3] for r_ in rows])
    reg = {}
    for nm, cols in (("depth+tilt", [0, 1, 2]), ("depth", [0, 1]), ("tilt", [0, 2])):
        A = X[:, cols]
        beta, *_ = np.linalg.lstsq(A, y, rcond=None)
        res = y - A @ beta
        dof = len(y) - len(cols)
        cov = np.linalg.inv(A.T @ A) * (res @ res) / dof
        r2 = 1 - (res @ res) / np.sum((y - y.mean()) ** 2)
        reg[nm] = dict(beta=beta.tolist(), se=np.sqrt(np.diag(cov)).tolist(), r2=float(r2), resid_sd=float(res.std(ddof=len(cols))))
        print(f"OLS fx ~ {nm}: beta {np.round(beta, 2)} se {np.round(np.sqrt(np.diag(cov)), 2)} R2 {r2:.2f}")
    out["leave_group_in"] = [dict(cluster=c, depth_m=d, tilt=t, fx=f) for c, d, t, f in rows]
    out["regression"] = reg
    # sharpness
    P = pickle.load(open(HERE / "views" / "poses.pkl", "rb"))
    sh = {}
    for p in P["poses"]:
        fi, ids, pts = p["views"][len(p["views"]) // 2]
        sh[p["rec"]] = sharpness(p["rec"], fi, pts)
    rec_cluster = {rec: c for c, ms in out_members(members).items() for rec in ms}
    pairs = [(depth[rec_cluster[k]], s, tilt[rec_cluster[k]]) for k, s in sh.items() if s is not None and k in rec_cluster]
    d_, s_, t_ = map(np.array, zip(*pairs))
    # thin lens defocus blur (diameter, px): f 6.3 mm, f/1.8, 1920-px pitch 4.25 um (S24 main, ASSUMED values)
    blur = 0.0035 * 0.0063 * np.abs(1 / d_ - 0.5) / 4.25e-6
    out["sharpness"] = dict(per_rec=sh, corr_depth=float(np.corrcoef(d_, s_)[0, 1]), corr_tilt=float(np.corrcoef(t_, s_)[0, 1]),
                            corr_thinlens_blur=float(np.corrcoef(blur, s_)[0, 1]),
                            note="edge sigma px = contrast / (p99 |grad| sqrt(2 pi)); thin-lens blur uses ASSUMED f 6.3 mm, f/1.8, 4.25 um px")
    for lo, hi in ((0, 0.6), (0.6, 0.8), (0.8, 1.0), (1.0, 1.4)):
        m = (d_ >= lo) & (d_ < hi)
        if m.any():
            print(f"depth {lo:.1f}-{hi:.1f} m: n {m.sum():2d} edge sigma median {np.median(s_[m]):.2f} px, thin-lens blur diam {np.median(blur[m]):.1f} px")
            out["sharpness"][f"depth_{lo}_{hi}"] = dict(n=int(m.sum()), sigma_px=float(np.median(s_[m])), thinlens_blur_px=float(np.median(blur[m])))
    print(f"corr(sharp sigma, depth) {out['sharpness']['corr_depth']:.2f}, (.., tilt) {out['sharpness']['corr_tilt']:.2f}, "
          f"(.., thin-lens blur) {out['sharpness']['corr_thinlens_blur']:.2f}")
    json.dump(out, open(HERE / "results" / "dist_tilt.json", "w"), indent=1)


def out_members(members):
    return {c: [p["rec"] for p in ps] for c, ps in members.items()}


if __name__ == "__main__":
    main()
