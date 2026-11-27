#!/usr/bin/env python3
"""Analysis of OUT/pairs_table.json (pair_diag.py table) -> OUT/analysis.json. CPU, seconds.
off = offset_cm * cam_sign  (+ = map plane nearer to the camera than the ROI consensus plane, i.e. depth too short)
dev = off - mean(off of the other pairs of the same keyframe in the same ROI)  (within-keyframe partner disagreement; kf with >= 2 pairs)
"""
import json, sys, time
from pathlib import Path
import numpy as np
from scipy.stats import spearmanr

V2 = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"; OUT = V2 / "diag_pairs_20261127"
T = json.loads((OUT / "pairs_table.json").read_text())
ROIS = ["cabinet_front_kf774", "wall_kf538", "floor_kf3205"]
rows = []
for r in ROIS:
    bmap = {s["idx"]: s["offset_cm"] * s["cam_sign"] for s in T["before"][r]["b"]["maps"]}
    A = T["before"][r]["a"]["maps"]
    for s in A:
        s = dict(s); s["roi"] = r; s["off"] = s["offset_cm"] * s["cam_sign"]; s["off_vs_b"] = s["off"] - bmap.get(s["idx"], np.nan)
        s["abs_dt"] = abs(s["dt_frames"]); s["ffs_px"] = s.get("ffs_W", np.nan) * s.get("ffs_H", np.nan) if "ffs_W" in s else np.nan
        s["B_m"] = s["B_geom_cm"] / 100; s["zz_fb"] = s["Zref_med_m"] ** 2 / (643.127 * s["B_m"] * s["sin_med"])   # m per px
        s["off_px"] = s["off"] / 100 / s["zz_fb"]       # plane offset converted to disparity px (+ = disparity too large)
        s["target_kf"] = None
        rows.append(s)
# within-kf deviation
for r in ROIS:
    by = {}
    for s in rows:
        if s["roi"] == r:
            by.setdefault(s["idx"], []).append(s)
    for idx, L in by.items():
        for s in L:
            o = [x["off"] for x in L if x is not s]
            s["dev"] = s["off"] - np.mean(o) if o else np.nan
            s["n_kf_pairs"] = len(L)
            s["kf_has_relax"] = any(x["cls"] == "relax" for x in L)
pf = json.loads((V2 / "diag_plane_fix_20261127" / "plane_fix.json").read_text())
CLS = ["base", "base_fb", "low", "wide_ba", "wide_vio", "relax"]


def mad(x):
    x = np.asarray(x, float); x = x[np.isfinite(x)]
    return float(1.4826 * np.median(np.abs(x - np.median(x)))) if len(x) else None


def grp(L, key="off"):
    x = np.array([s[key] for s in L], float); x = x[np.isfinite(x)]
    if not len(x):
        return dict(n=0)
    return dict(n=int(len(x)), mean=round(float(x.mean()), 3), median=round(float(np.median(x)), 3), sd=round(float(x.std()), 3),
                robust_sd=round(mad(x), 3), rms=round(float(np.sqrt(np.mean(x ** 2))), 3), absmed=round(float(np.median(np.abs(x))), 3))


res = dict(made=time.strftime("%F %T"), script=str(Path(__file__)), input=str(OUT / "pairs_table.json"), input_made=T["made"], note=__doc__,
           repro_ok=T["repro_check"]["all_equal_3dec"], n_maps={r: sum(s["roi"] == r for s in rows) for r in ROIS})
# ---- groups
G = {}
for r in ROIS + ["pooled"]:
    L = [s for s in rows if r == "pooled" or s["roi"] == r]
    G[r] = {c: dict(off=grp([s for s in L if s["cls"] == c]), dev=grp([s for s in L if s["cls"] == c and np.isfinite(s["dev"])], "dev"),
                    off_px=grp([s for s in L if s["cls"] == c], "off_px"), rho_pct=grp([dict(x=100 * s["rho_med"]) for s in L if s["cls"] == c], "x"),
                    tilt=grp([s for s in L if s["cls"] == c], "tilt_deg"), rms_own=grp([s for s in L if s["cls"] == c], "rms_own_cm"),
                    B_cm=grp([s for s in L if s["cls"] == c], "B_geom_cm"), perp=grp([s for s in L if s["cls"] == c], "perp_deg"),
                    disp_px=grp([s for s in L if s["cls"] == c], "disp_px_med"), Z=grp([s for s in L if s["cls"] == c], "Z_med_m"))
            for c in CLS if any(s["cls"] == c for s in L)}
    G[r]["_all"] = dict(off=grp(L), dev=grp([s for s in L if np.isfinite(s["dev"])], "dev"), off_px=grp(L, "off_px"))
res["groups"] = G
# ---- correlations (per ROI and stratified)
VARS = ["B_geom_cm", "perp_deg", "rot_deg", "abs_dt", "disp_px_med", "theta_med_deg", "sin_p5", "radius_med", "scale", "epi_out_kf_px",
        "Z_med_m", "zz_fb", "ba2_pred_dpx", "ba2_rot_change_deg", "ba2_B_ratio", "ba2_dir_change_deg", "p1_a_px", "rs_pred_b", "near_sector_boundary_frac",
        "rms_own_cm", "fit_alpha_px", "fit_beta", "extent_minor_cm", "n_px"]


def sp(L, xk, yk):
    x = np.array([s.get(xk, np.nan) if s.get(xk) is not None else np.nan for s in L], float); y = np.array([s[yk] for s in L], float)
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 8 or np.std(x[m]) == 0:
        return None
    return dict(n=int(m.sum()), rho=round(float(spearmanr(x[m], y[m])[0]), 3))


C = {}
for r in ROIS:
    L = [s for s in rows if s["roi"] == r]
    for s in L:
        s["absoff_dm"] = abs(s["off"] - np.median([x["off"] for x in L])); s["absdev"] = abs(s["dev"])
    C[r] = {v: dict(absoff=sp(L, v, "absoff_dm"), off=sp(L, v, "off"), absdev=sp(L, v, "absdev"), dev=sp(L, v, "dev")) for v in VARS}
strat = {}
for v in VARS:
    out = {}
    for yk in ("absoff_dm", "off", "absdev", "dev"):
        ws, rs = [], []
        for r in ROIS:
            q = sp([s for s in rows if s["roi"] == r], v, yk)
            if q:
                ws.append(q["n"]); rs.append(q["rho"])
        out[yk] = dict(n=int(sum(ws)), rho_wmean=round(float(np.average(rs, weights=ws)), 3)) if ws else None
    strat[v] = out
res["spearman_per_roi"] = C; res["spearman_stratified"] = strat


# ---- robust regression: off ~ ROI fixed effects + class dummies (ref = base)
def huber(A, y, c=None, it=30):
    w = np.ones(len(y)); c = c or 1.345 * mad(y - np.median(y))
    for _ in range(it):
        sw = np.sqrt(w); b = np.linalg.lstsq(A * sw[:, None], y * sw, rcond=None)[0]
        e = np.abs(y - A @ b); w = np.where(e <= c, 1, c / np.maximum(e, 1e-12))
    # sandwich-free approx se from weighted residuals
    r = y - A @ b; s2 = np.sum(w * r ** 2) / max(len(y) - A.shape[1], 1)
    cov = s2 * np.linalg.pinv((A * w[:, None]).T @ A)
    return b, np.sqrt(np.diag(cov))


def design(L, terms):
    cols, names = [], []
    for r in ROIS:
        cols.append([1.0 if s["roi"] == r else 0.0 for s in L]); names.append(f"roi:{r}")
    for t in terms:
        if t.startswith("cls:"):
            c = t[4:]; cols.append([1.0 if s["cls"] == c else 0.0 for s in L]); names.append(t)
        else:
            cols.append([float(s[t]) for s in L]); names.append(t)
    return np.array(cols).T, names


REG = {}
L = [s for s in rows]
for yk in ("off", "dev"):
    LL = [s for s in L if np.isfinite(s[yk])]
    terms = [f"cls:{c}" for c in CLS if c != "base" and any(s["cls"] == c for s in LL)]
    A, nm = design(LL, terms); y = np.array([s[yk] for s in LL])
    b, se = huber(A, y)
    REG[f"{yk}~roi+class"] = dict(n=len(y), coef={n_: [round(float(x), 3), round(float(e), 3)] for n_, x, e in zip(nm, b, se)})
# |off| scaling: log|off - roi median| ~ roi + a log B + b log Z + c log sin  (disparity-error model: a=-1, b=+2, c=-1; scale model: a=0, b=+1)
for yk in ("absoff_dm", "absdev"):
    LL = [s for s in L if np.isfinite(s[yk]) and s[yk] > 0.02]
    for s in LL:
        s["lB"] = np.log(s["B_m"]); s["lZ"] = np.log(s["Zref_med_m"]); s["lS"] = np.log(s["sin_med"]); s["ly"] = np.log(s[yk])
    A, nm = design(LL, ["lB", "lZ", "lS"]); y = np.array([s["ly"] for s in LL])
    b, se = huber(A, y, c=1.0)
    REG[f"log_{yk}~roi+logB+logZ+logsin"] = dict(n=len(y), coef={n_: [round(float(x), 3), round(float(e), 3)] for n_, x, e in zip(nm, b, se)},
                                                 note="disparity(px)-error model predicts logB -1, logZ +2 (within ROI Z varies little), logsin -1; scale(%) model logB 0, logZ +1")
res["regress"] = REG
# ---- disparity-px vs scale consistency across baseline: dispersion of off_px (px) and rho (%) per B tercile, per ROI-demeaned
D = {}
for yk, lab in (("off_px", "disp_px"), ("rho_pct", "depth_pct")):
    for s in rows:
        s["rho_pct"] = 100 * s["rho_med"]
    out = {}
    allB = np.array([s["B_geom_cm"] for s in rows]); qs = np.percentile(allB, [33.3, 66.7])
    for i, (lo, hi) in enumerate(((0, qs[0]), (qs[0], qs[1]), (qs[1], 1e9))):
        vals = []
        for r in ROIS:
            L = [s for s in rows if s["roi"] == r]; med = np.median([s[yk] for s in L])
            vals += [s[yk] - med for s in L if lo <= s["B_geom_cm"] < hi]
        out[f"B_{lo:.0f}-{min(hi, 99):.0f}cm"] = dict(n=len(vals), robust_sd=round(mad(vals), 3), sd=round(float(np.std(vals)), 3))
    D[lab] = out
res["dispersion_by_B"] = D
# ---- relax / target keyframe view on the wall and all ROIs
TK = {}
for r in ROIS:
    L = [s for s in rows if s["roi"] == r]
    TK[r] = dict(relax_pairs=grp([s for s in L if s["cls"] == "relax"]), other_pairs_in_relax_kf=grp([s for s in L if s["kf_has_relax"] and s["cls"] != "relax"]),
                 pairs_in_nonrelax_kf=grp([s for s in L if not s["kf_has_relax"]]))
res["relax_kf_view"] = TK
# ---- compact per-pair table for the page
keys = ["roi", "kf", "idx", "partner", "cls", "rect", "partner_pose_src", "B_geom_cm", "perp_deg", "rot_deg", "dt_frames", "epi_in_kf", "epi_out_kf_px",
        "epi_in_partner", "disp_px_med", "theta_med_deg", "sin_p5", "scale", "ffs_W", "ffs_H", "rect_f", "sector_major", "n_sectors_roi", "near_sector_boundary_frac",
        "radius_med", "u_c", "v_c", "Z_med_m", "off", "dev", "off_vs_b", "off_px", "rho_med", "tilt_deg", "rms_own_cm", "fit_alpha_px", "fit_beta",
        "ba2_pred_dpx", "ba2_B_ratio", "ba2_rot_change_deg", "p1_a_px", "rs_pred_b", "n_px"]
res["pairs"] = [{k: s.get(k) for k in keys} for s in rows]
(OUT / "analysis.json").write_text(json.dumps(res, indent=1, default=lambda o: float(o) if isinstance(o, (np.floating, np.integer)) else (bool(o) if isinstance(o, np.bool_) else str(o))))
# ---- print
print("repro", res["repro_ok"], res["n_maps"])
for r in ROIS + ["pooled"]:
    print("==", r)
    for c, g in G[r].items():
        print(f"  {c:9s} off {g['off']}  dev {g.get('dev')}" + (f"  B {g['B_cm'].get('median')} perp {g['perp'].get('median')} disp {g['disp_px'].get('median')} rms_own {g['rms_own'].get('median')} px {g['off_px']}" if c != "_all" else ""))
print("strat spearman (|off|, off, |dev|, dev):")
for v, o in strat.items():
    print(f"  {v:26s}", {k: (x["rho_wmean"], x["n"]) if x else None for k, x in o.items()})
for k, v in REG.items():
    print(k, v)
print("dispersion", json.dumps(D))
print("relax view", json.dumps(TK))
