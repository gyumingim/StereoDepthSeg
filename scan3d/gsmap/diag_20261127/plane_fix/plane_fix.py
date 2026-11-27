#!/usr/bin/env python3
"""Plane-offset fix preview (2026-11-27), no retraining. Same ruler as diag_plane_20261127/plane_diag.py (imported unchanged copy).

P1  per-pair disparity anchor: BA2 3D points observed in the keyframe (ba_partners2/ba/ba_state.npz, R1 t1 X1 obs) -> Z_ba in the
    keyframe camera (scaled to the map frame). residual r = f B sin (1/Z_ffs - 1/Z_ba) [px]; robust fit r ~ a + b*(v-cy)/100 + c*(u-cx)/100
    (median gate 2 px, Huber 0.3 px IRLS), min N points; correction 1/Z' = 1/Z - (a + b.. + c..)/(f B sin).
    HOLD-OUT: BA points inside any ROI footprint of that keyframe (dilated 20 px) are NOT used in the fit; they are used only to
    evaluate the fit (held-out residual before/after).
    stage a = corrected pair maps; stage b1 = same anchor fitted directly on the fused map (inverse-depth units, f B = 64 px m);
    stage b2 = simple re-fusion (median gate + inverse-variance) of the corrected pairs on the fused pixels (F == 1), with the
    same re-fusion of the uncorrected pairs as its own baseline.
RS  gyro (imu.csv type 4, R_CtoI from config/phone_ov_main/base60, t_imu = t_cam + 2.172 ms) at both frames' t_mid;
    predicted disparity row-slope = f * (t_r / 540) * (dw_y e_x - dw_x e_y), dw = w_partner - w_kf (camera frame),
    e = baseline direction in the kf image. t_r = rolling_shutter_skew_ns of frames_cam5.csv. Sign unknown -> both tried.
MIX pairs whose depth was made with ba_partners (BAP) poses vs BA2-posed relax pairs; predicted disparity shift of the BAP->BA2
    relative-rotation change vs fitted a.
usage: python plane_fix.py
"""
import json, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import plane_diag as PD   # unchanged copy of diag_plane_20261127/scripts/plane_diag.py

V2 = PD.V2; BA2 = V2 / "ba_partners2"; BAP = V2 / "ba_partners"; OUT = V2 / "diag_plane_fix_20261127"
REC = Path.home() / "scanner/StereoDepthSeg/scan3d/recordings/hs_20261112_121801_std60"
OVC = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/config/phone_ov_main/base60/kalibr_imucam_chain.yaml"
NMIN, GATE_PX, HUB, DIL = 50, 2.0, 0.3, 20
rng0 = np.random.default_rng(0)


def rotvec(R):
    from scipy.spatial.transform import Rotation as Ro
    return Ro.from_matrix(R).as_rotvec()


# ------------------------------------------------------------------ BA points per keyframe
def load_ba(kf, T):
    s = np.load(BA2 / "ba" / "ba_state.npz"); pa = np.load(BA2 / "poses_all.npz"); fr = pa["frames"]; Tm = pa["T"]
    R1, t1 = s["R1"], s["t1"]
    G = np.einsum("nij,njk->nik", Tm[:, :3, :3], R1)                       # map R_c2w = G R1^T  -> G = Rm R1
    gang = np.degrees([np.linalg.norm(rotvec(G[0].T @ g)) for g in G])
    Cb = -np.einsum("nji,nj->ni", R1, t1); Cm = Tm[:, :3, 3]
    i, j = rng0.integers(0, len(fr), (2, 20000)); ok = i != j
    db = np.linalg.norm(Cb[i[ok]] - Cb[j[ok]], axis=1); dm = np.linalg.norm(Cm[i[ok]] - Cm[j[ok]], axis=1); m = db > 0.05
    sc = float(np.median(dm[m] / db[m]))
    fi = {int(f): q for q, f in enumerate(fr)}
    cam, pt, uv, X = s["cam"], s["pt"], s["uv"], s["X1"]
    o = np.argsort(cam, kind="stable"); cs = cam[o]; out = {}
    for ci, f in enumerate(kf):
        b = fi[int(f)]; lo, hi = np.searchsorted(cs, [b, b + 1]); ii = o[lo:hi]
        Xc = (X[pt[ii]] @ R1[b].T + t1[b]) * sc
        keep = Xc[:, 2] > 0.1
        out[ci] = (uv[ii][keep].astype(np.float64), Xc[keep, 2])
        assert np.allclose(T[ci], Tm[b], atol=1e-6)
    info = dict(G_spread_deg_max=float(gang.max()), scale_map_over_ba=sc, scale_ratio_p5_95=[float(x) for x in np.percentile(dm[m] / db[m], [5, 95])])
    return out, info, {int(f): Tm[q] for q, f in enumerate(fr)}


# ------------------------------------------------------------------ pairs (same stack as PD.pair_stack, plus f, sin, source)
def pairs(kf, ci, Kfull, base_c, W, H):
    import gsm_ffs_4m as M
    fi = int(kf[ci]); L = []
    fb = PD.P2D / "pairs" / f"{fi:05d}.npz"
    if fb.exists():
        d = np.load(fb); Z = d["Z"].astype(np.float32); cmap = {int(r[0]): r[1:] for r in base_c.get(str(fi), [])}
        for p in range(len(d["B"])):
            L.append((Z[p], float(d["f"][p]), float(d["B"][p]), M.sin_map(Kfull, cmap[int(d["partner"][p])], W, H), "base", int(d["partner"][p]), "bap"))
    for tag, fe in (("low", PD.P3D / "extra" / f"{fi:05d}.npz"), ("wide", PD.SRC / "wide" / f"{fi:05d}.npz")):
        if fe.exists():
            src = "ba2" if "ffs_relax/relax" in str(fe.resolve()) else "bap"
            d = np.load(fe); Z = d["Z"].astype(np.float32); sn = d["sin"].astype(np.float32)
            for p in range(len(d["B"])):
                if tag == "low" and str(d["tag"][p]) != "low":
                    continue
                L.append((Z[p], float(d["f"][p]), float(d["B"][p]), sn[p], tag, int(d["partner"][p]), src))
    out = []
    for Z, f, B, sn, tag, j, src in L:
        sn = np.maximum(sn, 1e-3)
        with np.errstate(invalid="ignore"):
            bad = ~((Z / (f * B * sn)) <= PD.CAP) | (Z > PD.ZMAX) | (Z < 0.05)
        out.append(dict(Z=np.where(bad, np.nan, Z).astype(np.float64), f=f, B=B, sn=sn.astype(np.float64), tag=tag, partner=j, src=src))
    return out


# ------------------------------------------------------------------ anchor fit
def design(u, v, Kr):
    return np.c_[np.ones(len(u)), (v - Kr[1, 2]) / 100.0, (u - Kr[0, 2]) / 100.0]


def huber_fit(A, r, cols):
    w = np.ones(len(r)); A = A[:, cols]
    for _ in range(8):
        sw = np.sqrt(w); x = np.linalg.lstsq(A * sw[:, None], r * sw, rcond=None)[0]
        e = np.abs(r - A @ x); w = np.where(e <= HUB, 1.0, HUB / np.maximum(e, 1e-9))
    full = np.zeros(3); full[cols] = x
    return full


def fit_anchor(Z, scale_map, uv, zba, hold, Kr, W, H):
    """scale_map: f B sin map (px m) -> residual in px. returns dict or None."""
    u, v = uv[:, 0], uv[:, 1]; iu = np.clip(np.round(u).astype(int), 0, W - 1); iv = np.clip(np.round(v).astype(int), 0, H - 1)
    z = Z[iv, iu]; s = scale_map[iv, iu]
    with np.errstate(invalid="ignore", divide="ignore"):
        r = s * (1.0 / z - 1.0 / zba)
    fin = np.isfinite(r); ho = hold[iv, iu]
    tr = fin & ~ho
    res = dict(n_obs=int(len(u)), n_valid=int(tr.sum()), n_hold=int((fin & ho).sum()))
    if tr.sum() < NMIN:
        return None, res
    med = np.median(r[tr]); g = tr & (np.abs(r - med) < GATE_PX)
    res["n_fit"] = int(g.sum())
    if g.sum() < NMIN:
        return None, res
    A = design(u, v, Kr)
    cols = [0]
    vs, us = np.percentile(v[g], [5, 95]), np.percentile(u[g], [5, 95])
    if vs[1] - vs[0] >= 150:
        cols.append(1)
    if us[1] - us[0] >= 150:
        cols.append(2)
    x = huber_fit(A[g], r[g], cols)
    rr = r - A @ x
    mad = lambda q: float(1.4826 * np.median(np.abs(q - np.median(q)))) if len(q) else None
    res.update(a=float(x[0]), b=float(x[1]), c=float(x[2]), cols=cols, row_span=float(vs[1] - vs[0]), col_span=float(us[1] - us[0]),
               fit_med_before=float(np.median(r[g])), fit_mad_before=mad(r[g]), fit_mad_after=mad(rr[g]), fit_absmed_after=float(np.median(np.abs(rr[g]))))
    hg = fin & ho
    if hg.sum():
        hm = np.abs(r[hg] - np.median(r[tr])) < 3 * GATE_PX     # held-out inliers (same kind of gate, centred on the outside median)
        if hm.sum() >= 10:
            res.update(hold_n=int(hm.sum()), hold_med_before=float(np.median(r[hg][hm])), hold_med_after=float(np.median(rr[hg][hm])),
                       hold_absmed_before=float(np.median(np.abs(r[hg][hm]))), hold_absmed_after=float(np.median(np.abs(rr[hg][hm]))))
    return x, res


def apply_corr(Z, scale_map, x, Kr, W, H, extra_row=0.0):
    """1/Z' = 1/Z - (a + b*(v-cy)/100 + c*(u-cx)/100 + extra_row*(v-cy)/100)/scale."""
    vv, uu = np.mgrid[0:H, 0:W]
    delta = x[0] + (x[1] + extra_row) * (vv - Kr[1, 2]) / 100.0 + x[2] * (uu - Kr[0, 2]) / 100.0
    with np.errstate(invalid="ignore", divide="ignore"):
        inv = 1.0 / Z - delta / scale_map
        return np.where(inv > 1.0 / 20, 1.0 / inv, np.nan)


def refuse(maps, F):
    """simple re-fusion on F pixels: per-pixel median gate (2 sigma, >= 1 cm) then inverse-variance mean. needs >= 2 agreeing pairs
    or the single pair."""
    if not maps:
        return np.full(F.shape, np.nan)
    Zs = np.stack([m["Z"] for m in maps]); S = np.stack([m["Z"] ** 2 / (m["f"] * m["B"] * m["sn"]) for m in maps])
    with np.errstate(invalid="ignore", all="ignore"):
        med = np.nanmedian(Zs, 0)
        ok = np.isfinite(Zs) & (np.abs(Zs - med) <= np.maximum(2 * S, 0.01))
        w = np.where(ok, 1.0 / np.maximum(S, 1e-4) ** 2, 0); z = np.where(ok, Zs, 0)
        out = (w * z).sum(0) / w.sum(0)
    return np.where(F & (w.sum(0) > 0), out, np.nan)


# ------------------------------------------------------------------ gyro
def gyro_setup():
    import re
    txt = OVC.read_text(); rows = re.findall(r"- \[([^\]]+)\]", txt)[:3]
    Tic = np.array([[float(x) for x in r.split(",")] for r in rows]); R_ci = Tic[:, :3]
    tshift = float(re.search(r"timeshift_cam_imu:\s*([-0-9.eE]+)", txt).group(1))
    d = np.genfromtxt(REC / "imu.csv", delimiter=",", skip_header=1, usecols=(0, 1, 2, 3, 4), invalid_raise=False)
    g = d[d[:, 0] == 4]; tg = g[:, 1]; wg = g[:, 2:5]; o = np.argsort(tg); tg, wg = tg[o], wg[o]
    fc = np.genfromtxt(REC / "frames_cam5.csv", delimiter=",", skip_header=1)
    tmid = {int(r[0]): float(r[1]) for r in fc}; skew = float(np.median(fc[:, 5]))

    def w_cam(f):
        t = tmid[f] + tshift * 1e9
        wi = np.array([np.interp(t, tg, wg[:, k]) for k in range(3)])
        return R_ci.T @ wi
    return dict(R_ci=R_ci, tshift=tshift, skew_ns=skew, n_gyro=int(len(tg)), gyro_rate_hz=float(1e9 / np.median(np.diff(tg))), tmid=tmid, tg=tg, wg=wg), w_cam


def gyro_check(G, posesBA2):
    """integrated gyro rotation vs BA2 relative rotation between close BA2 frames (dt 30-200 ms)."""
    fr = sorted(posesBA2); er = []; ang = []; cs = []
    for a, b in zip(fr[:-1], fr[1:]):
        ta, tb = G["tmid"][a], G["tmid"][b]
        if not (0.03e9 <= tb - ta <= 0.2e9):
            continue
        Ra, Rb = posesBA2[a][:3, :3], posesBA2[b][:3, :3]; phi_p = rotvec(Ra.T @ Rb)
        sel = (G["tg"] >= ta + G["tshift"] * 1e9) & (G["tg"] <= tb + G["tshift"] * 1e9)
        if sel.sum() < 3:
            continue
        dt = np.diff(G["tg"][sel]) * 1e-9; wi = G["wg"][sel][:-1]
        phi_g = G["R_ci"].T @ (wi * dt[:, None]).sum(0)
        if np.linalg.norm(phi_p) > np.radians(0.5):
            er.append(np.degrees(np.linalg.norm(phi_p - phi_g))); ang.append(np.degrees(np.linalg.norm(phi_p)))
            cs.append(float(phi_p @ phi_g / np.linalg.norm(phi_p) / max(np.linalg.norm(phi_g), 1e-9)))
    return dict(n=len(er), rot_deg_med=float(np.median(ang)), err_deg_med=float(np.median(er)), err_over_rot_med=float(np.median(np.array(er) / np.array(ang))),
                cos_med=float(np.median(cs)))


# ------------------------------------------------------------------ measure (= PD.cmd_measure stages a/b, CPU, no d/e)
def measure(name_cond, RS_, fps_all, maps_of, kf, T, rc, W, H, mix_labels):
    res = {}
    for name, R0 in RS_.items():
        R = dict(R0); fps = fps_all[name]; rng = np.random.default_rng(0)

        def world(ci, Z, fp):
            mm = fp & np.isfinite(Z) & (Z > 0.05)
            return rc[mm.reshape(-1)] * Z[mm][:, None] @ T[ci][:3, :3].T + T[ci][:3, 3]
        for it in range(2):
            Pb = []
            for ci, fp in fps.items():
                X = world(ci, maps_of(ci)["b"], fp); X = X[np.abs(X @ R["n"] + R["d"]) <= PD.GATE]
                if len(X) >= PD.MIN_PTS:
                    Pb.append(X[rng.choice(len(X), min(len(X), 5000), replace=False)])
            n, d = PD.fit_plane(np.concatenate(Pb))
            if n @ R["n"] < 0:
                n, d = -n, -d
            R["n"], R["d"] = n, d; R["e1"], R["e2"] = PD.basis(n)
        stages = {"a": [], "b": []}; pooled = {"a": [], "b": []}
        for ci, fp in fps.items():
            mo = maps_of(ci)
            L = [("a", q["Z"], dict(partner=q["partner"], tag=q["tag"], src=q["src"])) for q in mo["a"]] + [("b", mo["b"], {})]
            for st, Z, info in L:
                X = world(ci, Z, fp); X = X[np.abs(X @ R["n"] + R["d"]) <= PD.GATE]
                s = PD.map_stats(X, R)
                if s is None:
                    continue
                s.pop("r_own"); s.update(idx=int(ci), kf=int(kf[ci]), mix=mix_labels.get(int(ci)), **info)
                # row tilt: slope of signed distance to the global plane vs image row (cm per 100 rows)
                stages[st].append(s)
                k = min(len(X), 20000); sel = rng.choice(len(X), k, replace=False)
                pooled[st].append(X[sel] @ R["n"] + R["d"])
        rr = {}
        for st in "ab":
            Ls = stages[st]
            off = np.array([s["offset_cm"] for s in Ls]); rg = np.concatenate(pooled[st])
            sm = dict(n_maps=len(Ls), within_rms_own_cm_median=float(np.median([s["rms_own_cm"] for s in Ls])),
                      offset_sd_cm=float(off.std()), offset_robust_sd_cm=float(1.4826 * np.median(np.abs(off - np.median(off)))),
                      tilt_median_deg=float(np.median([s["tilt_deg"] for s in Ls])),
                      combined_rms_global_cm=float(np.sqrt(np.mean(rg ** 2)) * 100), combined_mad_global_cm=float(1.4826 * np.median(np.abs(rg - np.median(rg))) * 100))
            if st == "a":
                by = {}
                for s in Ls:
                    by.setdefault(s["idx"], []).append(s["offset_cm"])
                w = [np.std(v) for v in by.values() if len(v) >= 2]
                sm.update(within_kf_partner_offset_sd_cm_mean=float(np.mean(w)) if w else None, across_kf_mean_offset_sd_cm=float(np.std([np.mean(v) for v in by.values()])))
            for lab in ("bap", "ba2", "target", "nontarget"):
                key = "src" if lab in ("bap", "ba2") else "mix"
                o2 = [s["offset_cm"] for s in Ls if s.get(key) == lab]
                if len(o2) >= 2:
                    sm[f"offset_sd_cm_{lab}"] = float(np.std(o2)); sm[f"n_{lab}"] = len(o2); sm[f"offset_mean_cm_{lab}"] = float(np.mean(o2))
            rr[st] = dict(summary=sm, maps=[{k: v for k, v in s.items() if k not in ("bands_own",)} for s in Ls])
        rr["c"] = dict(summary=dict(note="pooled stage b", combined_rms_global_cm=rr["b"]["summary"]["combined_rms_global_cm"],
                                    combined_mad_global_cm=rr["b"]["summary"]["combined_mad_global_cm"]))
        res[name] = rr
        print(name_cond, name, {st: {k: (round(v, 3) if isinstance(v, float) else v) for k, v in rr[st]["summary"].items() if k in
                                     ("n_maps", "within_rms_own_cm_median", "offset_sd_cm", "offset_robust_sd_cm", "combined_rms_global_cm")} for st in "ab"}, flush=True)
    return res


def main():
    import cv2
    import gsm_ba_partners as BP
    t0 = time.time(); OUT.mkdir(parents=True, exist_ok=True)
    kf, T, Kr, (W, H) = PD.cams(); rc = PD.rays(Kr, W, H); RS_ = PD.roi_setup()
    g = np.load(Path(BP.R1) / "rec" / "lensonly" / "gt.npz"); Kfull = g["K"].copy(); Kfull[:2] *= W / 1920.0
    base_c = json.loads((PD.SRC / "plan_4m.json").read_text())["base_c"]
    BA, ba_info, P2 = load_ba(kf, T); print("BA", ba_info, flush=True)
    PB = np.load(BAP / "poses_all.npz"); PBAP = {int(f): t for f, t in zip(PB["frames"], PB["T"])}
    wp = np.load(V2 / "ffsonly_ba" / "wide_poses.npz")
    for f, t, s in zip(wp["frames"], wp["T"], wp["src"]):
        PBAP.setdefault(int(f), t)
    G, w_cam = gyro_setup(); gchk = gyro_check(G, P2); print("gyro", {k: v for k, v in G.items() if k in ("tshift", "skew_ns", "n_gyro", "gyro_rate_hz")}, gchk, flush=True)
    t_r = G["skew_ns"] * 1e-9
    fps_all = {}
    for name, R in RS_.items():
        fp = {ci: PD.footprint(R, T[ci], rc, W, H) for ci in range(len(kf))}
        fps_all[name] = {ci: f for ci, f in fp.items() if f.sum() >= PD.MIN_FP}
    roi_kf = sorted({ci for v in fps_all.values() for ci in v})
    targets = {ci for ci, f in enumerate(kf) if not (V2 / "ffsonly_ba" / "wide" / f"{int(f):05d}.npz").exists()}
    mix_labels = {ci: ("target" if ci in targets else "nontarget") for ci in range(len(kf))}
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * DIL + 1, 2 * DIL + 1))
    # ---------------- pass over ALL keyframes: fits + RS / MIX predictors
    rows = []; keep = {}
    for ci in range(len(kf)):
        fi = int(kf[ci]); hold = np.zeros((H, W), bool)
        for v in fps_all.values():
            if ci in v:
                hold |= v[ci]
        if hold.any():
            hold = cv2.dilate(hold.astype(np.uint8), ker) > 0
        uv, zba = BA[ci]; P = pairs(kf, ci, Kfull, base_c, W, H)
        wk = w_cam(fi); Tk = P2[fi]
        for q in P:
            sm = q["f"] * q["B"] * q["sn"]
            x, info = fit_anchor(q["Z"], sm, uv, zba, hold, Kr, W, H)
            j = q["partner"]; row = dict(idx=ci, kf=fi, partner=j, tag=q["tag"], src=q["src"], B_cm=q["B"] * 100, roi=ci in roi_kf, target=ci in targets, **info)
            if j in P2 and j in G["tmid"]:
                Tj = P2[j]; c = Tk[:3, :3].T @ (Tj[:3, 3] - Tk[:3, 3]); e = c[:2] / max(np.linalg.norm(c[:2]), 1e-9)
                dw = w_cam(j) - wk
                row.update(dw_norm=float(np.linalg.norm(dw)), w_kf=float(np.linalg.norm(wk)), e=[float(e[0]), float(e[1])],
                           rs_pred_b=float(q["f"] * t_r / 540.0 * 100.0 * (dw[1] * e[0] - dw[0] * e[1])),
                           rs_pred_b_roll=float(q["f"] * t_r / 540.0 * 100.0 * dw[2]))
                if q["src"] == "bap" and fi in PBAP and j in PBAP:
                    Rb = PBAP[fi][:3, :3].T @ PBAP[j][:3, :3]; Rn = Tk[:3, :3].T @ Tj[:3, :3]
                    dphi = rotvec(Rn @ Rb.T)       # rotation change BAP -> BA2 (approx, kf camera frame)
                    Bb = np.linalg.norm(PBAP[j][:3, 3] - PBAP[fi][:3, 3]); Bn = np.linalg.norm(Tj[:3, 3] - Tk[:3, 3])
                    row.update(mix_pred_a=float(q["f"] * (dphi[1] * e[0] - dphi[0] * e[1])), mix_rot_deg=float(np.degrees(np.linalg.norm(dphi))), mix_B_ratio=float(Bn / Bb))
            rows.append(row)
            if ci in roi_kf:
                keep[(ci, len([r for r in rows if r["idx"] == ci]) - 1)] = x
        if ci % 50 == 0:
            print("fit", ci, len(rows), f"{time.time() - t0:.0f}s", flush=True)
    fitted = [r for r in rows if "a" in r]
    # ---------------- correlations
    def corr(xk, yk, sel):
        L = [r for r in fitted if sel(r) and xk in r and yk in r and np.isfinite(r[xk])]
        if len(L) < 10:
            return dict(n=len(L))
        x = np.array([r[xk] for r in L]); y = np.array([r[yk] for r in L])
        from scipy.stats import spearmanr
        sl = float(np.polyfit(x, y, 1)[0])
        return dict(n=len(L), pearson=float(np.corrcoef(x, y)[0, 1]), spearman=float(spearmanr(x, y)[0]), slope=sl,
                    x_sd=float(x.std()), y_sd=float(y.std()), x_absmed=float(np.median(np.abs(x))))
    slope_ok = lambda r: 1 in r["cols"]
    rs = dict(all=corr("rs_pred_b", "b", slope_ok), fast=corr("rs_pred_b", "b", lambda r: slope_ok(r) and r.get("dw_norm", 0) > 0.3),
              roll_term=corr("rs_pred_b_roll", "b", slope_ok), roi=corr("rs_pred_b", "b", lambda r: slope_ok(r) and r["roi"]),
              a_vs_rs=corr("rs_pred_b", "a", lambda r: True))
    mix = dict(a_vs_pred=corr("mix_pred_a", "a", lambda r: r["src"] == "bap"),
               a_sd_bap=float(np.std([r["a"] for r in fitted if r["src"] == "bap"])), a_sd_ba2=float(np.std([r["a"] for r in fitted if r["src"] == "ba2"])),
               a_mad_bap=float(1.4826 * np.median(np.abs([r["a"] for r in fitted if r["src"] == "bap"]))),
               a_mad_ba2=float(1.4826 * np.median(np.abs([r["a"] for r in fitted if r["src"] == "ba2"]))),
               n_bap=sum(r["src"] == "bap" for r in fitted), n_ba2=sum(r["src"] == "ba2" for r in fitted),
               mix_rot_deg_p50_95=[float(x) for x in np.percentile([r["mix_rot_deg"] for r in fitted if "mix_rot_deg" in r], [50, 95])],
               mix_pred_a_absmed=float(np.median(np.abs([r["mix_pred_a"] for r in fitted if "mix_pred_a" in r]))))
    print("RS", json.dumps(rs), "\nMIX", json.dumps(mix), flush=True)
    sign_rs = 1.0  # predictor already in px/100 rows; both signs tried below
    # ---------------- per-condition maps for the ROI keyframes
    cache = {}

    def base_maps(ci):
        if ci not in cache:
            P = pairs(kf, ci, Kfull, base_c, W, H)
            d = np.load(PD.FUSED / f"{int(kf[ci]):05d}.npz"); F = d["F"] == 1
            cache.clear(); cache[ci] = (P, F, np.where(F, d["Z"], np.nan).astype(np.float64))
        return cache[ci]
    rows_by = {}
    for r in rows:
        rows_by.setdefault(r["idx"], []).append(r)
    fusedfit = {}

    def cond_maps(mode):
        def f(ci):
            P, F, Zb = base_maps(ci); a_out = []
            for p, q in enumerate(P):
                r = rows_by[ci][p]; sm = q["f"] * q["B"] * q["sn"]; Z = q["Z"]
                rsb = 0.0
                if mode.startswith("rs") and "rs_pred_b" in r:
                    rsb = (1 if "+" in mode else -1) * r["rs_pred_b"]
                if mode in ("p1", "rs+p1", "rs-p1") or mode.startswith("rs"):
                    x = np.zeros(3)
                    if mode in ("p1",):
                        x = keep.get((ci, p)); x = np.zeros(3) if x is None else x
                    if mode in ("rs+p1", "rs-p1"):
                        Zr = apply_corr(Z, sm, np.zeros(3), Kr, W, H, extra_row=rsb)
                        hold = np.zeros((H, W), bool)
                        for v in fps_all.values():
                            if ci in v:
                                hold |= v[ci]
                        hold = cv2.dilate(hold.astype(np.uint8), ker) > 0
                        x2, _ = fit_anchor(Zr, sm, BA[ci][0], BA[ci][1], hold, Kr, W, H)
                        Z = apply_corr(Zr, sm, np.zeros(3) if x2 is None else x2, Kr, W, H)
                    elif mode in ("rs+", "rs-"):
                        Z = apply_corr(Z, sm, x, Kr, W, H, extra_row=rsb)
                    else:
                        Z = apply_corr(Z, sm, x, Kr, W, H)
                a_out.append(dict(q, Z=Z))
            if mode == "before":
                b = Zb
            elif mode == "b2_before":
                b = refuse(P, F)
            elif mode == "p1_direct":      # stage a = p1 pairs, stage b = anchor fitted on the fused map itself
                hold = np.zeros((H, W), bool)
                for v in fps_all.values():
                    if ci in v:
                        hold |= v[ci]
                hold = cv2.dilate(hold.astype(np.uint8), ker) > 0
                sm = np.full((H, W), 64.3)   # f B = 643 px x 0.10 m reference scale (inverse-depth units)
                x, info = fit_anchor(Zb, sm, BA[ci][0], BA[ci][1], hold, Kr, W, H); fusedfit[int(ci)] = info
                b = Zb if x is None else np.where(F, apply_corr(Zb, sm, x, Kr, W, H), np.nan)
            else:
                b = refuse(a_out, F)
            return dict(a=a_out if mode not in ("p1_direct",) else cond_maps("p1")(ci)["a"], b=b)
        return f
    results = {}
    for mode in ("before", "b2_before", "p1", "p1_direct", "rs+", "rs-", "rs+p1", "rs-p1"):
        cache.clear(); results[mode] = measure(mode, RS_, fps_all, cond_maps(mode), kf, T, rc, W, H, mix_labels)
    # ---------------- fit coverage
    nf = lambda k: sum(1 for r in rows if r.get("n_fit", 0) >= k)
    cov = dict(n_pairs=len(rows), n_kf=len(kf), fitted=len(fitted), n_fit_ge50=nf(50), n_fit_ge100=nf(100), n_fit_ge200=nf(200),
               with_row_slope=sum(1 in r["cols"] for r in fitted), with_col_slope=sum(2 in r["cols"] for r in fitted),
               roi_pairs=sum(r["roi"] for r in rows), roi_fitted=sum(r["roi"] and "a" in r for r in rows),
               a_px_p5_50_95=[float(x) for x in np.percentile([r["a"] for r in fitted], [5, 50, 95])], a_px_mad=float(1.4826 * np.median(np.abs(np.array([r["a"] for r in fitted]) - np.median([r["a"] for r in fitted])))),
               b_px100_p5_50_95=[float(x) for x in np.percentile([r["b"] for r in fitted if 1 in r["cols"]], [5, 50, 95])],
               fit_mad_before_med=float(np.median([r["fit_mad_before"] for r in fitted])), fit_mad_after_med=float(np.median([r["fit_mad_after"] for r in fitted])),
               hold=dict(n_pairs=sum("hold_n" in r for r in fitted),
                         absmed_before_med=float(np.median([r["hold_absmed_before"] for r in fitted if "hold_n" in r])),
                         absmed_after_med=float(np.median([r["hold_absmed_after"] for r in fitted if "hold_n" in r])),
                         signed_med_sd_before=float(np.std([r["hold_med_before"] for r in fitted if "hold_n" in r])),
                         signed_med_sd_after=float(np.std([r["hold_med_after"] for r in fitted if "hold_n" in r]))),
               n_obs_med=float(np.median([r["n_obs"] for r in rows])), n_fit_med=float(np.median([r.get("n_fit", 0) for r in rows])))
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__)), note=__doc__,
               inputs=dict(ba_state=str(BA2 / "ba/ba_state.npz"), poses_ba2=str(BA2 / "poses_all.npz"), poses_bap=str(BAP / "poses_all.npz"),
                           cams=str(PD.DUMP / "cams.npz"), fused=str(PD.FUSED), pairs=[str(PD.P2D / "pairs"), str(PD.P3D / "extra (tag low)"), str(PD.SRC / "wide")],
                           rois=str(PD.ROIS), imu=str(REC / "imu.csv"), frames=str(REC / "frames_cam5.csv"), imucam=str(OVC), ruler="diag_plane_20261127/scripts/plane_diag.py (copy)"),
               params=dict(nmin=NMIN, gate_px=GATE_PX, huber_px=HUB, holdout_dilate_px=DIL, t_r_ms=t_r * 1e3, plane_gate_m=PD.GATE, min_pts=PD.MIN_PTS),
               ba_info=ba_info, gyro=dict(tshift=G["tshift"], skew_ns=G["skew_ns"], n_gyro=G["n_gyro"], rate_hz=G["gyro_rate_hz"], check_vs_ba2=gchk),
               coverage=cov, rs_corr=rs, mix=mix, n_target_kf=len(targets), fused_direct_fit=fusedfit, conditions=results, pairs=rows, runtime_s=round(time.time() - t0, 1))
    (OUT / "plane_fix.json").write_text(json.dumps(res, indent=1, default=float))
    print("coverage", json.dumps(cov), "\ndone", res["runtime_s"], flush=True)


if __name__ == "__main__":
    main()
