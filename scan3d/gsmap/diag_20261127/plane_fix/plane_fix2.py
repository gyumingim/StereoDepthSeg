#!/usr/bin/env python3
"""Follow-up variants of plane_fix.py (2026-11-27), same ruler. Motivation: plane_fix P1 (N >= 50, a+b+c) made a few maps much worse
(low N, no held-out BA points in the ROI -> extrapolation). Variants (stage a = corrected pairs, stage b = simple re-fusion b2, vs b2_before):
  p1_n200_abc  a+b+c fit, applied only if N_fit >= 200 (else uncorrected)
  p1_n200_a    constant-only fit (a), applied only if N_fit >= 200
  p1_n50_a     constant-only, N_fit >= 50
  rs_k033      RS row-slope correction scaled by the observed regression slope 0.33 (sign +), no P1
Plus: BA2 3D points inside each ROI hull (map frame) vs the global plane = how flat the anchor itself is.
"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE))
import plane_diag as PD
import plane_fix as PF


def main():
    import cv2
    import gsm_ba_partners as BP
    t0 = time.time(); OUT = PF.OUT
    prev = json.loads((OUT / "plane_fix.json").read_text())
    rows_by = {}
    for r in prev["pairs"]:
        rows_by.setdefault(r["idx"], []).append(r)
    kf, T, Kr, (W, H) = PD.cams(); rc = PD.rays(Kr, W, H); RS_ = PD.roi_setup()
    g = np.load(Path(BP.R1) / "rec" / "lensonly" / "gt.npz"); Kfull = g["K"].copy(); Kfull[:2] *= W / 1920.0
    base_c = json.loads((PD.SRC / "plan_4m.json").read_text())["base_c"]
    BA, ba_info, P2 = PF.load_ba(kf, T)
    fps_all = {}
    for name, R in RS_.items():
        fp = {ci: PD.footprint(R, T[ci], rc, W, H) for ci in range(len(kf))}
        fps_all[name] = {ci: f for ci, f in fp.items() if f.sum() >= PD.MIN_FP}
    targets = {ci for ci, f in enumerate(kf) if not (PD.V2 / "ffsonly_ba" / "wide" / f"{int(f):05d}.npz").exists()}
    mix_labels = {ci: ("target" if ci in targets else "nontarget") for ci in range(len(kf))}
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * PF.DIL + 1, 2 * PF.DIL + 1))

    # ---- BA points flatness per ROI (map frame): X_map = s G X_ba + t, from camera centres
    s = np.load(PF.BA2 / "ba" / "ba_state.npz"); pa = np.load(PF.BA2 / "poses_all.npz")
    R1, t1, X1 = s["R1"], s["t1"], s["X1"]; Gm = pa["T"][1:, :3, :3] @ R1[1:]; Gr = Gm[0]
    sc = ba_info["scale_map_over_ba"]; Cb = -np.einsum("nji,nj->ni", R1, t1); tv = np.median(pa["T"][1:, :3, 3] - sc * Cb[1:] @ Gr.T, 0)
    Xm = sc * X1 @ Gr.T + tv
    chk = np.linalg.norm(pa["T"][1:, :3, 3] - (sc * Cb[1:] @ Gr.T + tv), axis=1)
    flat = dict(centre_fit_err_cm_med=float(np.median(chk) * 100))
    # global planes as in measure(before): refit from stage-b points
    def global_plane(R, mapsb):
        R = dict(R); rng = np.random.default_rng(0)
        for it in range(2):
            Pb = []
            for ci, fp in fps_all[R["name"]].items():
                Z = mapsb(ci); mm = fp & np.isfinite(Z) & (Z > 0.05)
                X = rc[mm.reshape(-1)] * Z[mm][:, None] @ T[ci][:3, :3].T + T[ci][:3, 3]; X = X[np.abs(X @ R["n"] + R["d"]) <= PD.GATE]
                if len(X) >= PD.MIN_PTS:
                    Pb.append(X[rng.choice(len(X), min(len(X), 5000), replace=False)])
            n, d = PD.fit_plane(np.concatenate(Pb))
            if n @ R["n"] < 0:
                n, d = -n, -d
            R["n"], R["d"] = n, d
        return R
    def fusedZ(ci):
        dd = np.load(PD.FUSED / f"{int(kf[ci]):05d}.npz"); return np.where(dd["F"] == 1, dd["Z"], np.nan).astype(np.float64)
    for name, R in RS_.items():
        R["name"] = name; Rg = global_plane(R, fusedZ)
        e1, e2 = PD.basis(R["fn"]); Q = np.c_[Xm @ R["fe1"], Xm @ R["fe2"]]
        inh = PD.in_hull(Q, R["hull"]); dist = Xm @ Rg["n"] + Rg["d"]; m = inh & (np.abs(dist) <= PD.GATE)
        dm = dist[m]
        flat[name] = dict(n_ba_pts=int(m.sum()), rms_cm=float(np.sqrt(np.mean(dm ** 2)) * 100) if m.sum() else None,
                          mad_cm=float(1.4826 * np.median(np.abs(dm - np.median(dm))) * 100) if m.sum() else None,
                          median_cm=float(np.median(dm) * 100) if m.sum() else None, within_2cm_frac=float(np.mean(np.abs(dm) < 0.02)) if m.sum() else None)
        if m.sum() >= 10:
            n2, d2 = PD.fit_plane(Xm[m]); r2 = Xm[m] @ n2 + d2
            flat[name].update(own_plane_rms_cm=float(np.sqrt(np.mean(r2 ** 2)) * 100), own_plane_mad_cm=float(1.4826 * np.median(np.abs(r2)) * 100),
                              tilt_vs_global_deg=float(np.degrees(np.arccos(min(1, abs(n2 @ Rg["n"]))))))
    print("BA flatness", json.dumps(flat), flush=True)

    def hold_of(ci):
        h = np.zeros((H, W), bool)
        for v in fps_all.values():
            if ci in v:
                h |= v[ci]
        return cv2.dilate(h.astype(np.uint8), ker) > 0 if h.any() else h

    def variant(mode):
        cache = {}

        def f(ci):
            if ci in cache:
                return cache[ci]
            P = PF.pairs(kf, ci, Kfull, base_c, W, H); dd = np.load(PD.FUSED / f"{int(kf[ci]):05d}.npz"); F = dd["F"] == 1
            hold = hold_of(ci); out = []
            for p, q in enumerate(P):
                r = rows_by[ci][p]; assert r["partner"] == q["partner"] and r["tag"] == q["tag"]
                sm = q["f"] * q["B"] * q["sn"]; Z = q["Z"]
                if mode.startswith("p1"):
                    nmin = 200 if "n200" in mode else 50
                    PF.NMIN = nmin
                    x, info = PF.fit_anchor(Z, sm, BA[ci][0], BA[ci][1], hold, Kr, W, H)
                    if x is not None and mode.endswith("_a"):
                        # constant only: Huber on the gated residual with the design column 0 only
                        uv, zba = BA[ci]; iu = np.clip(np.round(uv[:, 0]).astype(int), 0, W - 1); iv = np.clip(np.round(uv[:, 1]).astype(int), 0, H - 1)
                        with np.errstate(invalid="ignore", divide="ignore"):
                            rr = sm[iv, iu] * (1.0 / Z[iv, iu] - 1.0 / zba)
                        ok = np.isfinite(rr) & ~hold[iv, iu]; med = np.median(rr[ok]); gg = ok & (np.abs(rr - med) < PF.GATE_PX)
                        x = PF.huber_fit(PF.design(uv[:, 0], uv[:, 1], Kr)[gg], rr[gg], [0])
                    if x is not None:
                        Z = PF.apply_corr(Z, sm, x, Kr, W, H)
                elif mode == "rs_k033" and "rs_pred_b" in r:
                    Z = PF.apply_corr(Z, sm, np.zeros(3), Kr, W, H, extra_row=0.33 * r["rs_pred_b"])
                out.append(dict(q, Z=Z))
            res = dict(a=out, b=PF.refuse(out, F))
            if len(cache) > 3:
                cache.pop(next(iter(cache)))
            cache[ci] = res
            return res
        return f
    results = {}
    for mode in ("p1_n200_abc", "p1_n200_a", "p1_n50_a", "rs_k033"):
        results[mode] = PF.measure(mode, RS_, fps_all, variant(mode), kf, T, rc, W, H, mix_labels)
    PF.NMIN = 50
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__)), note=__doc__, inputs=dict(prev=str(OUT / "plane_fix.json"), **prev["inputs"]),
               ba_flatness=flat, conditions=results, runtime_s=round(time.time() - t0, 1))
    (OUT / "plane_fix2.json").write_text(json.dumps(res, indent=1, default=float))
    print("done", res["runtime_s"], flush=True)


if __name__ == "__main__":
    main()
