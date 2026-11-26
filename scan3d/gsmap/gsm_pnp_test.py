#!/usr/bin/env python3
"""Test only (CPU): does PnP-refining the partner-frame pose against BA-pose 3D points reduce rectification dy vs VIO / E-matrix?

BA 3D points were not saved by the BA run (save_tri false), so the "BA points" here are re-triangulated from the keyframe's
SIFT features with its BA neighbour keyframes (BA poses fixed, reprojection < 1 px in both views, angle >= 1.5 deg, kf +-4).
For each sampled pair (kf, partner j): kf<->j SIFT matches split 70 % fit / 30 % held-out (random, seed per pair).
PnP (RANSAC from VIO guess + LM refine) and E use only fit matches; dy is measured on held-out matches for VIO / PnP / E.
Also reports baseline length VIO vs PnP and the audit4 an3s BA/VIO baseline ratio (nearest kf on the partner side).
"""
import gzip, json, sys, time, os
from pathlib import Path
import cv2, numpy as np

cv2.setNumThreads(4)
G = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d"
sys.path.insert(0, str(G))
import motion_stereo as ms

R1 = G / "runs/gsmap_20261125_121801"; R2 = G / "runs/gsmap_20261125_121801_v2"
OUT = R2 / "pnp_test"; OUT.mkdir(exist_ok=True)
N_STRICT, N_FB, N_LOW = int(os.environ.get("N_STRICT", 60)), int(os.environ.get("N_FB", 30)), int(os.environ.get("N_LOW", 30))
NFEAT = 3000
t0 = time.time()
c = np.load(R1 / "dump_ba/cams.npz"); kf, Tk = c["kf"].astype(int), c["T"].astype(np.float64); W0, H0 = [int(v) for v in c["size"]]
K = c["K"].astype(np.float64); dist = c["dist"].astype(np.float64); f = float(K[0, 0]); n = len(kf)
g = np.load(R1 / "rec/lensonly/gt.npz"); t_ns = g["t_ns"]
pl2 = json.load(open(R2 / "ffs_pairs2/plan.json")); pl3 = json.load(open(R2 / "ffs_pairs3/plan3.json"))
vt, vT = [], []
for line in gzip.open(pl2["args"]["vio"], "rt"):
    d = json.loads(line)
    if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
        vt.append(d["t_ns"]); vT.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
vt = np.array(vt); vT = np.array(vT)
idx = np.searchsorted(t_ns, vt).clip(1, len(t_ns) - 1); idx = np.where(np.abs(t_ns[idx - 1] - vt) < np.abs(t_ns[idx] - vt), idx - 1, idx)
Tv = {int(i): T_ for i, T_, ok in zip(idx, vT, np.abs(t_ns[idx] - vt) < 8e6) if ok}
kfi = {int(x): i for i, x in enumerate(kf)}

# ---- sample pairs evenly over the recording per tier
allp = []
for fi, L in pl2["chosen"]["plan"].items():
    for e in L:
        allp.append((int(fi), int(e[0]), "strict" if e[1] == "local" else e[1]))
for fi, L in pl3["plan"].items():
    for e in L:
        allp.append((int(fi), int(e[0]), e[1]))
allp = [p for p in allp if p[0] in Tv and p[1] in Tv and p[0] in kfi]
sel = []
for tier, k in (("strict", N_STRICT), ("fallback", N_FB), ("low", N_LOW)):
    L = sorted([p for p in allp if p[2] == tier])
    if L:
        ii = np.unique(np.linspace(0, len(L) - 1, min(k, len(L))).round().astype(int)); sel += [L[i] for i in ii]
print("pool", {t: sum(p[2] == t for p in allp) for t in ("strict", "fallback", "low")}, "sampled", len(sel), flush=True)

# ---- frames needed: kf, partners, BA neighbour keyframes (+-4)
NB = {}
for fi, j, _ in sel:
    ci = kfi[fi]; NB[fi] = [cj for cj in range(ci - 4, ci + 5) if cj != ci and 0 <= cj < n]
need = set()
for fi, j, _ in sel:
    need |= {fi, j} | {int(kf[cj]) for cj in NB[fi]}
cap = cv2.VideoCapture(str(R1 / "rec/cam5.mp4")); FT = {}; i = -1; mx = max(need)
while i < mx:
    if (i + 1) in need:
        ok, im = cap.read()
    else:
        ok = cap.grab(); im = None
    i += 1
    if not ok:
        break
    if im is not None:
        im = cv2.resize(im, (W0, H0), interpolation=cv2.INTER_AREA) if im.shape[1] != W0 else im
        FT[i] = ms.sift_feats(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), NFEAT)
print("features", len(FT), f"{time.time() - t0:.0f}s", flush=True)
BF = cv2.BFMatcher(cv2.NORM_L2)


def match_idx(a, b):
    (pa, da), (pb, db) = FT[a], FT[b]
    m = [p[0] for p in BF.knnMatch(da, db, k=2) if len(p) == 2 and p[0].distance < 0.8 * p[1].distance]
    return np.array([x.queryIdx for x in m], int), np.array([x.trainIdx for x in m], int)


NORM = {}
def norm(fr):
    if fr not in NORM:
        NORM[fr] = cv2.undistortPoints(FT[fr][0].reshape(-1, 1, 2).astype(np.float64), K, dist).reshape(-1, 2)
    return NORM[fr]


def rotdeg(Ra):
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra) - 1) / 2, -1, 1))))


PTS = {}
def kf_points(fi):
    """kf keypoint index -> world point, triangulated with BA neighbour keyframes (best angle)."""
    if fi in PTS:
        return PTS[fi]
    ci = kfi[fi]; Tw0 = np.linalg.inv(Tk[ci]); P0 = Tw0[:3]; xa = norm(fi); best = {}
    C0 = Tk[ci][:3, 3]
    for cj in NB[fi]:
        fj = int(kf[cj]); ia, ib = match_idx(fi, fj)
        if len(ia) < 20:
            continue
        Tw1 = np.linalg.inv(Tk[cj]); P1 = Tw1[:3]
        X = cv2.triangulatePoints(P0, P1, xa[ia].T, norm(fj)[ib].T); X = (X[:3] / X[3]).T
        x0 = X @ Tw0[:3, :3].T + Tw0[:3, 3]; x1 = X @ Tw1[:3, :3].T + Tw1[:3, 3]
        e0 = np.linalg.norm(x0[:, :2] / x0[:, 2:] - xa[ia], axis=1) * f; e1 = np.linalg.norm(x1[:, :2] / x1[:, 2:] - norm(fj)[ib], axis=1) * f
        r0 = X - C0; r1 = X - Tk[cj][:3, 3]
        ang = np.degrees(np.arccos(np.clip(np.sum(r0 * r1, 1) / (np.linalg.norm(r0, axis=1) * np.linalg.norm(r1, axis=1) + 1e-12), -1, 1)))
        ok = (x0[:, 2] > 0.1) & (x1[:, 2] > 0.1) & (x0[:, 2] < 8) & (e0 < 1.0) & (e1 < 1.0) & (ang >= 1.5)
        for k_, X_, a_ in zip(ia[ok], X[ok], ang[ok]):
            if k_ not in best or a_ > best[k_][1]:
                best[k_] = (X_, a_)
    PTS[fi] = {k_: v[0] for k_, v in best.items()}
    return PTS[fi]


def dyq(a, b, R, t):
    if R is None or len(a) < 5:
        return None
    return ms.rect_match_stats(a, b, R, t, f)["dy_med"]


rows = []; tp = []
for fi, j, tier in sel:
    ts = time.time(); ci = kfi[fi]
    Tj_v = Tk[ci] @ np.linalg.inv(Tv[fi]) @ Tv[j]
    row = dict(kf=fi, j=j, tier=tier, dt_s=round(abs(t_ns[j] - t_ns[fi]) * 1e-9, 3))
    ia, ib = match_idx(fi, j); row["n_match"] = int(len(ia))
    if len(ia) < 30:
        row["why"] = "few matches"; rows.append(row); continue
    rng = np.random.default_rng(fi * 100003 + j); perm = rng.permutation(len(ia)); nf_ = int(round(0.7 * len(ia)))
    fit, ho = perm[:nf_], perm[nf_:]
    a_all, b_all = norm(fi)[ia], norm(j)[ib]
    Rv, tv = ms.rel_pose(Tk[ci], Tj_v); Bv = float(np.linalg.norm(tv)); row["B_vio_cm"] = round(100 * Bv, 2)
    row["perp_vio"] = round(ms.pair_geometry(Rv, tv)["perp_deg"], 1)
    # an3s ratio: nearest kf on the partner side (within 30 frames), BA baseline / VIO baseline
    cand = [cj for cj in range(n) if cj != ci and int(kf[cj]) in Tv and (int(kf[cj]) - fi) * (j - fi) > 0]
    if cand:
        cj = min(cand, key=lambda q: abs(int(kf[q]) - j)); bb = np.linalg.norm(Tk[cj][:3, 3] - Tk[ci][:3, 3]); bv = np.linalg.norm(Tv[int(kf[cj])][:3, 3] - Tv[fi][:3, 3])
        if bb > 0.02 and bv > 0.02 and abs(int(kf[cj]) - j) <= 30:
            row["an3s_ratio_raw"] = round(float(bb / bv), 4); row["an3s_ratio"] = round(float(np.clip(bb / bv, 0.7, 1.4)), 4); row["an3s_kf"] = int(kf[cj])
    # --- PnP on fit matches whose kf keypoint has a 3D point
    P = kf_points(fi); row["kf_pts"] = len(P)
    fm = [k for k in fit if ia[k] in P]; row["pnp_corr_fit"] = len(fm)
    Rp = tp_ = None
    if len(fm) >= 15:
        Xw = np.array([P[ia[k]] for k in fm]); xb = norm(j)[ib[fm]]
        Tcw = np.linalg.inv(Tj_v); rv0, _ = cv2.Rodrigues(Tcw[:3, :3]); tv0 = Tcw[:3, 3].reshape(3, 1).copy()
        ok, rvec, tvec, inl = cv2.solvePnPRansac(Xw, xb, np.eye(3), None, rv0.copy(), tv0.copy(), useExtrinsicGuess=True,
                                                 iterationsCount=500, reprojectionError=1.5 / f, confidence=0.999, flags=cv2.SOLVEPNP_ITERATIVE)
        ninl = 0 if inl is None else len(inl); row["pnp_inl"] = int(ninl); row["pnp_inl_frac"] = round(ninl / len(fm), 3)
        if ok and ninl >= 30 and ninl >= 0.3 * len(fm):
            ii = inl.reshape(-1)
            rvec, tvec = cv2.solvePnPRefineLM(Xw[ii], xb[ii], np.eye(3), None, rvec, tvec)
            Rc, _ = cv2.Rodrigues(rvec); Tcw_p = np.eye(4); Tcw_p[:3, :3] = Rc; Tcw_p[:3, 3] = tvec.reshape(3)
            pr = (Xw[ii] @ Rc.T + tvec.reshape(3)); row["pnp_rmse_px"] = round(float(np.sqrt(np.mean(np.sum((pr[:, :2] / pr[:, 2:] - xb[ii]) ** 2, 1))) * f), 3)
            Tj_p = np.linalg.inv(Tcw_p); Rp, tp_ = ms.rel_pose(Tk[ci], Tj_p)
            row["pnp_ok"] = True; row["dR_vio_pnp_deg"] = round(rotdeg(Tj_v[:3, :3].T @ Tj_p[:3, :3]), 3)
            row["dC_vio_pnp_cm"] = round(100 * float(np.linalg.norm(Tj_v[:3, 3] - Tj_p[:3, 3])), 2)
            row["B_pnp_cm"] = round(100 * float(np.linalg.norm(tp_)), 2); row["pnp_over_vio_B"] = round(float(np.linalg.norm(tp_)) / Bv, 4)
            row["dir_vio_pnp_deg"] = round(float(np.degrees(np.arccos(np.clip((tv / Bv) @ (tp_ / np.linalg.norm(tp_)), -1, 1)))), 2)
    row.setdefault("pnp_ok", False)
    # --- E on fit matches
    Re, te, info = ms.pose_from_E(a_all[fit], b_all[fit], f, Rv, tv, ms.MSConfig())
    row["E_ok"] = Re is not None; row["E_why"] = info.get("why")
    # --- dy on held-out (and in-sample for reference)
    a, b = a_all[ho], b_all[ho]; row["n_holdout"] = int(len(ho))
    row["dy_vio"] = dyq(a, b, Rv, tv); row["dy_pnp"] = dyq(a, b, Rp, tp_); row["dy_E"] = dyq(a, b, Re, te)
    row["dy_E_insample"] = dyq(a_all[fit], b_all[fit], Re, te); row["dy_vio_fitset"] = dyq(a_all[fit], b_all[fit], Rv, tv)
    row["t_pair_s"] = round(time.time() - ts, 3); tp.append(row["t_pair_s"]); rows.append(row)
    if len(rows) % 20 == 0:
        print(len(rows), f"{time.time() - t0:.0f}s", flush=True)


def q(x):
    x = np.array([v for v in x if v is not None], float)
    return dict(n=int(len(x)), med=round(float(np.median(x)), 3), p90=round(float(np.percentile(x, 90)), 3)) if len(x) else None


summ = {}
for tier in ("all", "strict", "fallback", "low"):
    R_ = [r for r in rows if tier == "all" or r["tier"] == tier]
    both = [r for r in R_ if r.get("dy_pnp") is not None]
    allE = [r for r in R_ if r.get("dy_pnp") is not None and r.get("dy_E") is not None]
    summ[tier] = dict(n=len(R_), pnp_success=round(sum(r["pnp_ok"] for r in R_) / max(len(R_), 1), 3), E_success=round(sum(r.get("E_ok", False) for r in R_) / max(len(R_), 1), 3),
                      dy_vio_all=q([r.get("dy_vio") for r in R_]),
                      on_pnp_ok=dict(vio=q([r["dy_vio"] for r in both]), pnp=q([r["dy_pnp"] for r in both])),
                      on_all3=dict(vio=q([r["dy_vio"] for r in allE]), pnp=q([r["dy_pnp"] for r in allE]), E=q([r["dy_E"] for r in allE]), E_insample=q([r["dy_E_insample"] for r in allE])),
                      dE_all=q([r.get("dy_E") for r in R_]),
                      dR_deg=q([r.get("dR_vio_pnp_deg") for r in both]), dC_cm=q([r.get("dC_vio_pnp_cm") for r in both]),
                      dir_deg=q([r.get("dir_vio_pnp_deg") for r in both]),
                      pnp_over_vio_B=q([r.get("pnp_over_vio_B") for r in both]))
sc = [(r["pnp_over_vio_B"], r["an3s_ratio_raw"]) for r in rows if r.get("pnp_ok") and "an3s_ratio_raw" in r]
if sc:
    s = np.array(sc); summ["scale"] = dict(n=len(s), pnp_ratio=q(s[:, 0]), an3s_ratio_raw=q(s[:, 1]), corr=round(float(np.corrcoef(s[:, 0], s[:, 1])[0, 1]), 3),
                                           abs_diff=q(np.abs(s[:, 0] - s[:, 1])), pnp_dev_from_1=q(np.abs(s[:, 0] - 1)), an3s_dev_from_1=q(np.abs(s[:, 1] - 1)))
summ["time_per_pair_s"] = q(tp); summ["n_kf_tri"] = len(PTS)
summ["kf_pts"] = q([len(v) for v in PTS.values()])
res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), inputs=dict(cams=str(R1 / "dump_ba/cams.npz"), vio=pl2["args"]["vio"], plan2=str(R2 / "ffs_pairs2/plan.json"), plan3=str(R2 / "ffs_pairs3/plan3.json"),
           video=str(R1 / "rec/cam5.mp4"), note="BA 3D points re-triangulated from BA keyframe poses (BA run had save_tri false)"),
           runtime_s=round(time.time() - t0, 1), summary=summ, rows=rows)
(OUT / "pnp_test.json").write_text(json.dumps(res, indent=1, default=float))
print(json.dumps(summ, indent=1, default=float))
