"""lenscal an1: base solves, projection uncertainty, model holdout CV, A vs B and split-half cross-validation, fixed lenses."""
import json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from lenscal_lib import *
t0 = time.time()
M = meta(); rng = np.random.default_rng(0)
LENS = os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d/config")
V = json.load(open(os.path.join(LENS, "phone_ov_main", "variants.json")))
L24d = json.load(open(os.path.join(LENS, "lens", "std_1920x1080_f0.50_cal20261024d.json")))
C26 = json.load(open(os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d/runs/calcap_calib_20261026/lens_candidate_20261026.json")))
fixed = {"c4f60_app (hs f2.22 cal20261022 x0.98624)": opencv_to_mrcal(V["K_raw_used"], V["dist"]),
         "cal20261024d": opencv_to_mrcal(L24d["K"], L24d["dist"])}
try:
    fixed["calcap_20261026 k12 (OpenCV)"] = opencv_to_mrcal(C26["K"], C26["dist"])
except Exception as e:
    print("candidate json keys", list(C26.keys()), e)
import mrcal
fixedm = {k: mrcal.cameramodel(intrinsics=v, imagersize=(W, H)) for k, v in fixed.items()}
R = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), inputs=dict(
    A="calcap_20261025_214000 (132 stills, 33 slots, 2 sessions 10-25/10-26, YUV stills 10 fps, ISO 1317/625, 2 ms)",
    B="lens_calib_f050_20261023 views/poses.pkl, board_laptop_5x3 only: 55 recs (47 near-pose clusters), 4 frames/rec evenly = 217 frames (std60 recordings 10-23/10-24)",
    board="board_laptop_5x3 ChArUco on laptop screen, 4x2 inner corners, 60.4 mm", tool="mrcal 2.1 (Ubuntu jammy debs, local)",
    note="calobject_warp y is unobservable with 2 corner rows -> warp solved only by profile (an2.py)"), base={}, cv={}, xval={}, fixed={})
SETS = ["A", "B", "AB"]
B = {}
for s in SETS:
    for mk in MODELS:
        m, oi, names = base(s, mk)
        idx = np.arange(len(names))
        o = subset(oi, idx); st, rms, cost, n = solve(o); mm = model_of(o)
        B[(s, mk)] = (mm, o, names)
        us, grid = unc_summary(mm)
        it = mm.intrinsics()[1]
        R["base"][f"{s}_{mk}"] = dict(n_frames=len(names), n_caps=len(set(caps_of(names, M))), rms=round(rms, 4), n_pts=n,
                                       noutliers=int(st["Noutliers"]), intr=[round(float(x), 5) for x in it[:8]] if mk != "SPLINED" else [round(float(x), 3) for x in it[:4]],
                                       unc_px_inf=us)
        np.save(os.path.join(T, f"unc_{s}_{mk}.npy"), grid[2])
        print(s, mk, "rms %.4f" % rms, "fxfycxcy", np.round(it[:4], 2), "unc", us["centre"], us["r0.5"], us["r0.8"], us["grid33x19"], flush=True)
# model holdout CV by capture (5 folds x 2 repeats) on A, B
for s in ["A", "B"]:
    for mk in MODELS:
        mm, oi, names = B[(s, mk)]
        caps = caps_of(names, M); uc = np.array(sorted(set(caps)))
        errs = []
        for rep in range(2):
            perm = rng.permutation(len(uc)); folds = np.array_split(uc[perm], 5)
            for f in folds:
                te = np.where(np.isin(caps, f))[0]; tr = np.where(~np.isin(caps, f))[0]
                o = subset(oi, tr); solve(o)
                ot = subset(oi, te, intr=o["intrinsics"][0], fix_intr=True, outlier=False)
                _, r, _, _ = solve(ot); errs.append(r)
        R["cv"][f"{s}_{mk}"] = dict(holdout_rms_mean=round(float(np.mean(errs)), 4), holdout_rms_med=round(float(np.median(errs)), 4))
        print("cv", s, mk, R["cv"][f"{s}_{mk}"], flush=True)
# cross-validation: A vs B, and split halves within A and B (random capture halves, 20 each)
def comb_unc(m1, m2):
    P = test_points(); out = {}
    for k, pts in P.items():
        u = np.sqrt(np.atleast_1d(unc_at(m1, pts)) ** 2 + np.atleast_1d(unc_at(m2, pts)) ** 2)
        out[k] = dict(max=round(float(u.max()), 3), med=round(float(np.median(u)), 3))
    return out
for mk in MODELS:
    mA, mB = B[("A", mk)][0], B[("B", mk)][0]
    R["xval"][f"AvsB_{mk}"] = dict(diff_norot=diff_summary(mA, mB, True), diff_rotfit=diff_summary(mA, mB, False), unc_comb=comb_unc(mA, mB))
    print("AvsB", mk, R["xval"][f"AvsB_{mk}"], flush=True)
    for s in ["A", "B"]:
        mm, oi, names = B[(s, mk)]
        caps = caps_of(names, M); uc = np.array(sorted(set(caps)))
        rows = []
        for k in range(20):
            perm = rng.permutation(len(uc)); h1 = uc[perm[: len(uc) // 2]]
            i1 = np.where(np.isin(caps, h1))[0]; i2 = np.where(~np.isin(caps, h1))[0]
            o1 = subset(oi, i1); solve(o1); o2 = subset(oi, i2); solve(o2)
            m1, m2 = model_of(o1), model_of(o2)
            d = diff_summary(m1, m2, True); u = comb_unc(m1, m2)
            rows.append(dict(d=d, u=u, dfx=float(o1["intrinsics"][0][0] - o2["intrinsics"][0][0]), dcx=float(o1["intrinsics"][0][2] - o2["intrinsics"][0][2]),
                             dcy=float(o1["intrinsics"][0][3] - o2["intrinsics"][0][3])))
        summ = {}
        for key in ["centre", "r0.5", "r0.8", "r1.0"]:
            dd = np.array([r["d"][key]["max"] for r in rows]); uu = np.array([r["u"][key]["max"] for r in rows])
            summ[key] = dict(diff_med=round(float(np.median(dd)), 3), unc_med=round(float(np.median(uu)), 3), ratio_med=round(float(np.median(dd / uu)), 2),
                             frac_diff_le_2unc=round(float(np.mean(dd <= 2 * uu)), 2))
        summ["abs_dfx_med_pct"] = round(float(np.median(np.abs([r["dfx"] for r in rows])) / 1286 * 100), 3)
        summ["abs_dcx_med"] = round(float(np.median(np.abs([r["dcx"] for r in rows]))), 2)
        summ["abs_dcy_med"] = round(float(np.median(np.abs([r["dcy"] for r in rows]))), 2)
        R["xval"][f"halves_{s}_{mk}"] = summ
        print("halves", s, mk, summ, flush=True)
# fixed lenses: score on A and B (frames only), and ray diff vs AB models
for name, mf in fixedm.items():
    lm, it = mf.intrinsics()
    r = {}
    for s in ["A", "B"]:
        mm, oi, names = B[(s, "OPENCV4")]
        o = subset(oi, np.arange(len(names)), lensmodel=lm, intr=it, fix_intr=True, outlier=False)
        _, rms, _, _ = solve(o); r[f"rms_{s}_fixed"] = round(rms, 4)
    for mk in MODELS:
        mAB = B[("AB", mk)][0]
        r[f"vs_AB_{mk}_norot"] = diff_summary(mf, mAB, True)
        r[f"vs_AB_{mk}_rotfit"] = diff_summary(mf, mAB, False)
    R["fixed"][name] = r
    print("fixed", name, r["rms_A_fixed"], r["rms_B_fixed"], r["vs_AB_OPENCV8_norot"], flush=True)
# same-set reference: rms of each set with its own model, outliers off
for s in ["A", "B"]:
    for mk in MODELS:
        mm, oi, names = B[(s, mk)]
        o = subset(oi, np.arange(len(names)), intr=oi["intrinsics"][0], fix_intr=True, outlier=False)
        _, rms, _, _ = solve(o); R["fixed"].setdefault("_own_model_rms_no_outlier_rej", {})[f"{s}_{mk}"] = round(rms, 4)
R["sec"] = round(time.time() - t0, 1)
json.dump(R, open(os.path.join(T, "an1.json"), "w"), indent=1, default=float)
print("done", R["sec"])
