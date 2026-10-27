"""mrcal part of the printed-board lens pipeline + acceptance criteria C1-C8 (lenskit 2026-10-27).

Runs with SYSTEM python3 + the local mrcal 2.1 (source runs/lenscal_20261027/mrcal_env.sh); called by lenskit_run.sh.
  python3 lenskit_mrcal.py WORK_DIR [--boot 30] [--sub-boot 20] [--with-g]
Inputs in WORK_DIR: corners_{A,B[,G]}.vnl + meta_*.json (lenskit_dump.py), opencv_{A,B,AB}.json (calcap_calib results).
Output: WORK_DIR/criteria.json (every number, thresholds, PASS / FAIL / 판정 불가 per criterion) + criteria_ko.txt.

Criteria = runs/lenscal_20261027/README.md section 3 (numbers copied, "추정" thresholds marked there):
  C1 per set: RMS <= 0.25 px (OpenCV style = sqrt(2) x mrcal per-coordinate RMS), outliers <= 3 %, per-corner mean
     residual <= 0.05 px (no pattern).
  C2 per set: mrcal projection uncertainty (at infinity, worst direction, ONE still per pose) centre <= 0.2 px,
     r <= 0.8 (4 diagonals) <= 0.5 px.
  C3 A vs B (one still per pose, mrcal rotation fit): diff <= uA + uB at centre / r0.5 / r0.8, and diff(r0.8) <= 1.0 px.
  C4 |fx_A - fx_B| <= 0.15 %, pose-bootstrap sd(fx) per set <= 0.15 %, OpenCV k12_notan principal point A vs B <= 2 px.
  C5 measured flatness (board json flatness_mm) <= 0.1 mm, mrcal warp (both axes, sag inside the corner span) within
     +-0.1 mm of it, |fx(warp) - fx(no warp)| <= 0.1 %.
  C6 per set, 6 random pose halves (one still per pose): diff <= 2 sqrt(u1^2 + u2^2) on >= 70 % of grid points r <= 0.8.
  C7 A+B near (< DSPLIT m) vs far subsets: |dfx| <= 2 sd (pose bootstrap of each subset); with --with-g (set G, phone
     mounted differently): |dfx| <= 0.1 %, |dcx|, |dcy| <= 2 px vs A+B.
  C8 A+B lean+ vs lean- subsets (>= 10 poses each): |dfx|, |dcy| <= 2 sd (pose bootstrap) - board self-sag check.
Model for all checks: LENSMODEL_OPENCV4 (k1 k2 p1 p2) + calobject_warp (2 axes; observable with 8 corner rows).
Lens values come only from these board data; nothing here looks at VIO.
"""
import argparse
import copy
import json
import os
import subprocess
import time

import numpy as np
import mrcal

W, H = 1920, 1080
LM = "LENSMODEL_OPENCV4"
DSPLIT = 1.15
FLOOR_FX_PCT, FLOOR_PP_PX = 0.075, 1.0      # --accept-small only: half of the C4 tolerances (0.15 %, 2 px)


def log(s, L):
    print(s, flush=True)
    L.append(s)


def test_points():
    c = np.array([(W - 1) / 2, (H - 1) / 2]); hd = c.copy()
    P = {"centre": [c]}
    for r in (0.5, 0.8):
        P[f"r{r}"] = [c + r * hd * np.array(s) for s in ((1, 1), (-1, 1), (1, -1), (-1, -1))]
    return P


def cli_solve(work, tags, grid, out):
    os.makedirs(out, exist_ok=True)
    cache = os.path.join(out, "corners.vnl")
    with open(cache, "w") as f:
        f.write("# filename x y level\n")
        for t in tags:
            for ln in open(os.path.join(work, f"corners_{t}.vnl")):
                if not ln.startswith("#"):
                    f.write(ln)
    cmd = ["mrcal-calibrate-cameras", "--corners-cache", cache, "--lensmodel", LM, "--focal", "1286",
           "--imagersize", str(W), str(H), "--object-spacing", f"{grid['spacing_m']:.6f}",
           "--object-width-n", str(grid["nx"]), "--object-height-n", str(grid["ny"]), "--outdir", out, "[" + "".join(tags) + "]/*"]   # ONE glob = one camera (several globs = several cameras)
    r = subprocess.run(cmd, cwd=out, capture_output=True, text=True)
    open(os.path.join(out, "mrcal_stdout.txt"), "w").write(r.stdout + "\n" + r.stderr)
    if r.returncode != 0:
        raise RuntimeError("mrcal-calibrate-cameras failed: " + r.stderr[-2000:])
    m = mrcal.cameramodel(os.path.join(out, "camera-0.cameramodel"))
    return m, m.optimization_inputs()


def subset(oi, idx, warp=True, one=False):
    o = copy.deepcopy(oi)
    idx = np.asarray(idx)
    ob = o["observations_board"][idx].copy()
    o["observations_board"] = ob
    o["frames_rt_toref"] = o["frames_rt_toref"][idx].copy()
    n = len(idx)
    o["indices_frame_camintrinsics_camextrinsics"] = np.c_[np.arange(n), np.zeros(n), -np.ones(n)].astype(np.int32)
    o["imagepaths"] = np.asarray(o["imagepaths"])[idx]
    if warp:
        if o.get("calobject_warp") is None:
            o["calobject_warp"] = np.zeros(2)
        o["do_optimize_calobject_warp"] = True
    else:
        o["calobject_warp"] = None
        o["do_optimize_calobject_warp"] = False
    o["do_apply_outlier_rejection"] = True
    o["verbose"] = False
    return o


def solve(o):
    s = mrcal.optimize(**o)
    return s


def resid_stats(o, s):
    ob = o["observations_board"]
    N, ny, nx, _ = ob.shape
    x = s["x"][:N * ny * nx * 2].reshape(N, ny, nx, 2)
    good = ob[..., 2] > 0
    r2 = np.sum(x ** 2, -1)
    rms = float(np.sqrt(np.mean(r2[good])))                 # OpenCV style (per point, both coordinates)
    mean_vec = np.zeros((ny, nx, 2))
    for i in range(ny):
        for j in range(nx):
            g = good[:, i, j]
            if g.sum():
                mean_vec[i, j] = x[g, i, j].mean(0)
    return rms, int(good.sum()), mean_vec


def model_of(o):
    return mrcal.cameramodel(optimization_inputs=o, icam_intrinsics=0)


def unc(m, pts):
    v = mrcal.unproject(np.asarray(pts, float), *m.intrinsics())
    return np.atleast_1d(mrcal.projection_uncertainty(v, model=m, atinfinity=True, what="worstdirection-stdev"))


def rotdiff(m1, m2, n=61):
    difflen, diff, q0, Rt = mrcal.projection_diff([m1, m2], gridn_width=n, use_uncertainties=False)
    return difflen, q0


def at_points(difflen, q0, pts):
    gy, gx = difflen.shape
    return np.array([difflen[int(round(p[1] / (H - 1) * (gy - 1))), int(round(p[0] / (W - 1) * (gx - 1)))] for p in pts])


def intr(o):
    i = np.asarray(o["intrinsics"]).ravel()
    return dict(fx=float(i[0]), fy=float(i[1]), cx=float(i[2]), cy=float(i[3]), dist=[float(x) for x in i[4:]])


def first_still_idx(names, meta):
    seen, idx = set(), []
    for k, n in enumerate(names):
        c = meta[n]["cap"]
        if c not in seen:
            seen.add(c); idx.append(k)
    return np.array(idx)


def boot(oi, names, meta, sel, B, rng):
    """pose bootstrap over images `sel` (indices) -> array of (fx, cx, cy)."""
    caps = np.array([meta[names[k]]["cap"] for k in sel])
    uc = np.array(sorted(set(caps)))
    out = []
    for _ in range(B):
        pick = rng.choice(uc, len(uc), replace=True)
        idx = np.concatenate([np.asarray(sel)[caps == c] for c in pick])
        o = subset(oi, idx)
        try:
            solve(o)
        except Exception:
            continue
        i = intr(o); out.append((i["fx"], i["cx"], i["cy"]))
    return np.array(out)


def verdict(ok):
    return "PASS" if ok is True else ("FAIL" if ok is False else "판정 불가")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("work")
    ap.add_argument("--boot", type=int, default=30)
    ap.add_argument("--sub-boot", type=int, default=20)
    ap.add_argument("--halves", type=int, default=6)
    ap.add_argument("--with-g", action="store_true")
    ap.add_argument("--flatness-mm", type=float, default=None, help="override the board json flatness_mm (synthetic tests only)")
    ap.add_argument("--dsplit", type=float, default=DSPLIT)
    ap.add_argument("--accept-small", action="store_true",
                    help="C7 / C8: a statistically significant difference still passes when it is below HALF the C4 set-to-set "
                         "tolerance (fx 0.075 %%, cy / pp 1 px). NOT in the README rule - a human choice, recorded in the output.")
    a = ap.parse_args()
    W_ = a.work
    rng = np.random.default_rng(0)
    L = []
    t0 = time.time()
    tags = ["A", "B"] + (["G"] if a.with_g else [])
    metas = {t: json.load(open(os.path.join(W_, f"meta_{t}.json"))) for t in tags}
    grid = metas["A"]["grid"]
    meta = {m["name"]: m for t in tags for m in metas[t]["images"]}
    res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), work=os.path.abspath(W_), lensmodel=LM, grid=grid,
               sets={t: metas[t]["sets"] for t in tags}, per_set={}, criteria={})
    if abs(grid["scale_xy"][0] - grid["scale_xy"][1]) > 5e-4:
        log(f"주의: 자로 잰 가로/세로 배율이 다름 {grid['scale_xy']} - mrcal 은 칸 간격 하나만 받음 (OpenCV 결과는 반영)", L)
    M, OI, F = {}, {}, {}
    for t in tags + ["AB"]:
        tt = ["A", "B"] if t == "AB" else [t]
        m, oi = cli_solve(W_, tt, grid, os.path.join(W_, f"mrcal_{t}"))
        M[t], OI[t] = m, oi
        # full re-solve in python (same inputs) to get residuals
        o = subset(oi, np.arange(len(oi["imagepaths"])))
        s = solve(o)
        rms, ngood, mv = resid_stats(o, s)
        names = ["/".join(str(n).split("/")[-2:]) for n in oi["imagepaths"]]   # mrcal prefixes the cache dir
        nobs = sum(meta[n]["n"] for n in names)
        w = np.asarray(o["calobject_warp"]) * 1000.0
        # no-warp fit for C5
        o_nw = subset(oi, np.arange(len(names)), warp=False); solve(o_nw)
        # one still per pose -> uncertainty
        i1 = first_still_idx(names, meta)
        o1 = subset(oi, i1); solve(o1)
        m1 = model_of(o1)
        P = test_points()
        u = {k: unc(m1, p) for k, p in P.items()}
        F[t] = dict(o=o, o1=o1, m1=m1, names=names, u=u, m=model_of(o))
        ps = dict(n_images=len(names), n_poses=len(i1), rms_px=round(rms, 4), corners_used=ngood, corners_seen=nobs,
                  outlier_frac=round(1 - ngood / max(nobs, 1), 4),
                  per_corner_mean_resid_max_px=round(float(np.linalg.norm(mv, axis=-1).max()), 4),
                  intrinsics=intr(o), warp_mm_x_y=[round(float(x), 4) for x in w],
                  fx_nowarp=round(intr(o_nw)["fx"], 3), intrinsics_one_still=intr(o1),
                  unc_one_still={k: [round(float(x), 3) for x in v] for k, v in u.items()})
        res["per_set"][t] = ps
        log(f"[{t}] 사진 {ps['n_images']} 자세 {ps['n_poses']} · RMS {ps['rms_px']} px · 이상치 {ps['outlier_frac'] * 100:.2f} % · "
            f"fx {ps['intrinsics']['fx']:.2f} cx {ps['intrinsics']['cx']:.2f} cy {ps['intrinsics']['cy']:.2f} · 휨 x/y {ps['warp_mm_x_y']} mm · "
            f"불확실성(자세당 1장) 가운데 {u['centre'].max():.3f} r0.8 {u['r0.8'].max():.3f} px", L)
    C = res["criteria"]
    # C1
    c1 = {t: dict(rms=res["per_set"][t]["rms_px"], outlier=res["per_set"][t]["outlier_frac"],
                  corner_mean=res["per_set"][t]["per_corner_mean_resid_max_px"]) for t in ("A", "B")}
    ok1 = all(v["rms"] <= 0.25 and v["outlier"] <= 0.03 and v["corner_mean"] <= 0.05 for v in c1.values())
    C["C1"] = dict(values=c1, rule="RMS <= 0.25 px, outliers <= 3 %, per-corner mean residual <= 0.05 px", result=verdict(ok1))
    # C2
    c2 = {t: dict(centre=round(float(F[t]["u"]["centre"].max()), 3), r08=round(float(F[t]["u"]["r0.8"].max()), 3)) for t in ("A", "B")}
    ok2 = all(v["centre"] <= 0.2 and v["r08"] <= 0.5 for v in c2.values())
    C["C2"] = dict(values=c2, rule="uncertainty (one still per pose) centre <= 0.2 px, r0.8 <= 0.5 px", result=verdict(ok2))
    # C3
    dl, q0 = rotdiff(F["A"]["m1"], F["B"]["m1"])
    c3 = {}
    ok3 = True
    for k, pts in test_points().items():
        d = at_points(dl, q0, pts); us = F["A"]["u"][k] + F["B"]["u"][k]
        c3[k] = dict(diff_max=round(float(d.max()), 3), unc_sum=round(float(us.max()), 3), ratio_max=round(float((d / us).max()), 2))
        ok3 = ok3 and bool(np.all(d <= us))
    ok3 = ok3 and c3["r0.8"]["diff_max"] <= 1.0
    C["C3"] = dict(values=c3, rule="A vs B (rotation fit) <= uA + uB at centre / r0.5 / r0.8 and r0.8 <= 1.0 px; ratio > 2 = look further",
                   result=verdict(ok3))
    # C4
    fA, fB = res["per_set"]["A"]["intrinsics"]["fx"], res["per_set"]["B"]["intrinsics"]["fx"]
    dfx = abs(fA - fB) / ((fA + fB) / 2) * 100
    sd = {}
    for t in ("A", "B"):
        bt = boot(OI[t], F[t]["names"], meta, np.arange(len(F[t]["names"])), a.boot, rng)
        sd[t] = dict(n=len(bt), sd_fx_pct=round(float(bt[:, 0].std(ddof=1) / bt[:, 0].mean() * 100), 4),
                     sd_cx=round(float(bt[:, 1].std(ddof=1)), 3), sd_cy=round(float(bt[:, 2].std(ddof=1)), 3))
        res["per_set"][t]["boot"] = sd[t]
    pp = None
    try:
        ocA = json.load(open(os.path.join(W_, "opencv_A.json")))["models"]["k12_notan"]["lens"]
        ocB = json.load(open(os.path.join(W_, "opencv_B.json")))["models"]["k12_notan"]["lens"]
        pp = dict(dcx=round(ocA["cx"] - ocB["cx"], 3), dcy=round(ocA["cy"] - ocB["cy"], 3))
    except (OSError, KeyError) as e:
        log(f"OpenCV k12_notan 결과 없음: {e}", L)
    ok4 = dfx <= 0.15 and all(v["sd_fx_pct"] <= 0.15 for v in sd.values()) and (pp is not None and abs(pp["dcx"]) <= 2 and abs(pp["dcy"]) <= 2)
    C["C4"] = dict(values=dict(dfx_pct=round(dfx, 4), boot=sd, k12_notan_pp_A_minus_B=pp),
                   rule="|fx_A - fx_B| <= 0.15 %, bootstrap sd(fx) <= 0.15 % per set, k12_notan principal point A-B <= 2 px",
                   result=verdict(bool(ok4)))
    # C5
    flat = grid.get("flatness_mm") if a.flatness_mm is None else a.flatness_mm
    res["flatness_source"] = "board json" if a.flatness_mm is None else "--flatness-mm override (test)"
    c5 = {}
    ok5 = None if flat is None else True
    for t in ("A", "B"):
        ps = res["per_set"][t]
        wmax = max(abs(x) for x in ps["warp_mm_x_y"])
        dfxw = abs(ps["intrinsics"]["fx"] - ps["fx_nowarp"]) / ps["intrinsics"]["fx"] * 100
        c5[t] = dict(warp_mm_x_y=ps["warp_mm_x_y"], fx_warp_vs_nowarp_pct=round(dfxw, 4))
        if flat is not None:
            ok5 = ok5 and flat <= 0.1 and abs(wmax - flat) <= 0.1 and dfxw <= 0.1
    C["C5"] = dict(values=dict(measured_flatness_mm=flat, sets=c5),
                   rule="flatness (straightedge) <= 0.1 mm, |mrcal warp| within +-0.1 mm of it, fx warp vs no warp <= 0.1 %",
                   result=verdict(ok5), note=None if flat is not None else "board json flatness_mm 이 비어 있음 - 재고 적을 것")
    # C6
    c6 = {}
    ok6 = True
    for t in ("A", "B"):
        names = F[t]["names"]; i1 = first_still_idx(names, meta)
        caps = np.array([meta[names[k]]["cap"] for k in i1])
        fr = []
        for _ in range(a.halves):
            perm = rng.permutation(len(i1)); h = perm[: len(i1) // 2]
            ia, ib = i1[np.sort(h)], i1[np.sort(perm[len(i1) // 2:])]
            oa, ob = subset(OI[t], ia), subset(OI[t], ib)
            solve(oa); solve(ob)
            ma, mb = model_of(oa), model_of(ob)
            dl, q0 = rotdiff(ma, mb, 31)
            q = q0.reshape(-1, 2); d = dl.ravel()
            rr = np.hypot(q[:, 0] - (W - 1) / 2, q[:, 1] - (H - 1) / 2) / np.hypot((W - 1) / 2, (H - 1) / 2)
            sel = rr <= 0.8
            ua, ub = unc(ma, q[sel]), unc(mb, q[sel])
            fr.append(float(np.mean(d[sel] <= 2 * np.sqrt(ua ** 2 + ub ** 2))))
        c6[t] = dict(frac_min=round(min(fr), 3), frac_med=round(float(np.median(fr)), 3), n=len(fr))
        ok6 = ok6 and min(fr) >= 0.7
    C["C6"] = dict(values=c6, rule="halves (one still per pose): diff <= 2 sqrt(u1^2 + u2^2) on >= 70 % of grid points r <= 0.8 (every split)",
                   result=verdict(ok6))
    # C7 / C8 on A+B
    namesAB = F["AB"]["names"]
    def sub_fit(sel):
        o = subset(OI["AB"], sel); solve(o); i = intr(o)
        bt = boot(OI["AB"], namesAB, meta, sel, a.sub_boot, rng)
        return dict(fx=round(i["fx"], 3), cx=round(i["cx"], 3), cy=round(i["cy"], 3), n_poses=len({meta[namesAB[k]]["cap"] for k in sel}),
                    sd_fx=round(float(bt[:, 0].std(ddof=1)), 3), sd_cy=round(float(bt[:, 2].std(ddof=1)), 3))
    near = np.array([k for k, n in enumerate(namesAB) if meta[n].get("dist", 9) < a.dsplit])
    far = np.array([k for k, n in enumerate(namesAB) if meta[n].get("dist", 0) >= a.dsplit])
    sn, sf = sub_fit(near), sub_fit(far)
    lim7 = 2 * np.hypot(sn["sd_fx"], sf["sd_fx"])
    small7 = abs(sn["fx"] - sf["fx"]) / sn["fx"] * 100 <= FLOOR_FX_PCT
    ok7 = abs(sn["fx"] - sf["fx"]) <= lim7 or (a.accept_small and small7)
    c7 = dict(near=sn, far=sf, dfx=round(sn["fx"] - sf["fx"], 3), limit_2sd=round(float(lim7), 3), dsplit_m=a.dsplit,
              below_half_C4=bool(small7), accept_small=a.accept_small)
    if a.with_g:
        iA = res["per_set"]["AB"]["intrinsics"]; iG = res["per_set"]["G"]["intrinsics"]
        g = dict(dfx_pct=round((iG["fx"] - iA["fx"]) / iA["fx"] * 100, 4), dcx=round(iG["cx"] - iA["cx"], 3), dcy=round(iG["cy"] - iA["cy"], 3))
        c7["G"] = g
        ok7 = ok7 and abs(g["dfx_pct"]) <= 0.1 and abs(g["dcx"]) <= 2 and abs(g["dcy"]) <= 2
    C["C7"] = dict(values=c7, rule="near vs far |dfx| <= 2 sd (pose bootstrap); set G (optional) |dfx| <= 0.1 %, |dpp| <= 2 px",
                   result=verdict(bool(ok7)), note=None if a.with_g else "세트 G 없음 (선택) - 거리 부분만 판정")
    lp = np.array([k for k, n in enumerate(namesAB) if meta[n].get("lean", 0) > 5])
    ln = np.array([k for k, n in enumerate(namesAB) if meta[n].get("lean", 0) < -5])
    np_p, np_n = len({meta[namesAB[k]]["cap"] for k in lp}), len({meta[namesAB[k]]["cap"] for k in ln})
    if min(np_p, np_n) < 10:
        C["C8"] = dict(values=dict(n_poses_plus=np_p, n_poses_minus=np_n), rule="lean+ vs lean- (>= 10 poses each)", result=verdict(None))
    else:
        sp_, sm_ = sub_fit(lp), sub_fit(ln)
        lf, lc = 2 * np.hypot(sp_["sd_fx"], sm_["sd_fx"]), 2 * np.hypot(sp_["sd_cy"], sm_["sd_cy"])
        small8 = (abs(sp_["fx"] - sm_["fx"]) / sp_["fx"] * 100 <= FLOOR_FX_PCT and abs(sp_["cy"] - sm_["cy"]) <= FLOOR_PP_PX)
        ok8 = (abs(sp_["fx"] - sm_["fx"]) <= lf and abs(sp_["cy"] - sm_["cy"]) <= lc) or (a.accept_small and small8)
        C["C8"] = dict(values=dict(plus=sp_, minus=sm_, dfx=round(sp_["fx"] - sm_["fx"], 3), dcy=round(sp_["cy"] - sm_["cy"], 3),
                                   limit_fx=round(float(lf), 3), limit_cy=round(float(lc), 3), below_half_C4=bool(small8),
                                   accept_small=a.accept_small),
                       rule="lean+ vs lean- |dfx|, |dcy| <= 2 sd (pose bootstrap), >= 10 poses each", result=verdict(bool(ok8)))
    # info: OpenCV k12_notan (A+B, the app lens candidate) vs mrcal OPENCV4+warp (A+B), rays without rotation fit
    try:
        oc = json.load(open(os.path.join(W_, "opencv_AB.json")))["models"]["k12_notan"]
        K = np.array(oc["K"]); d = np.array(oc["dist"])
        intr_oc = np.array([K[0, 0], K[1, 1], K[0, 2], K[1, 2], d[0], d[1], 0.0, 0.0])
        mo = mrcal.cameramodel(intrinsics=(LM, intr_oc), imagersize=(W, H))
        dl, q0 = rotdiff(F["AB"]["m"], mo)
        res["info_k12_vs_mrcal_AB"] = {k: round(float(at_points(dl, q0, p).max()), 3) for k, p in test_points().items()}
    except Exception as e:  # noqa: BLE001
        res["info_k12_vs_mrcal_AB"] = f"not computed: {e}"
    allp = all(C[k]["result"] == "PASS" for k in ("C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8"))
    res["all_pass"] = allp
    res["accept_small"] = a.accept_small
    res["sec"] = round(time.time() - t0, 1)
    for k in sorted(C):
        log(f"{k} {C[k]['result']:6s} {json.dumps(C[k]['values'], ensure_ascii=False)}", L)
        log(f"     기준: {C[k]['rule']}" + (f" · {C[k]['note']}" if C[k].get("note") else ""), L)
    log(f"참고: OpenCV k12_notan(A+B) 대 mrcal OPENCV4+휨(A+B) 광선 차이 {res['info_k12_vs_mrcal_AB']}", L)
    log("판정: " + ("C1–C8 모두 통과" if allp else "통과 못 함 → 앱 렌즈 파일 만들지 않음 (보드·촬영을 의심, VIO 로 고르지 않음)"), L)
    json.dump(res, open(os.path.join(W_, "criteria.json"), "w"), indent=1, ensure_ascii=False,
              default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    open(os.path.join(W_, "criteria_ko.txt"), "w", encoding="utf-8").write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
