"""Shared helpers for lenscal_20261027 mrcal analysis (system python3 + local mrcal 2.1)."""
import json, os, copy
import numpy as np
import mrcal

T = os.path.expanduser("~/scanner/tmp_lenscal")
W, H = 1920, 1080
SPL = "LENSMODEL_SPLINED_STEREOGRAPHIC_order=3_Nx=12_Ny=7_fov_x_deg=80"
MODELS = {"OPENCV4": "LENSMODEL_OPENCV4", "OPENCV8": "LENSMODEL_OPENCV8", "SPLINED": SPL}
DIRN = {"OPENCV4": "OPENCV4", "OPENCV8": "OPENCV8", "SPLINED": "SPLINED_STEREOGRAPHIC"}

def meta():
    d = {}
    for tag in "AB":
        for m in json.load(open(os.path.join(T, f"meta_{tag}.json"))):
            d[m["name"]] = m
    return d

def base(setname, model):
    m = mrcal.cameramodel(os.path.join(T, "m", f"{setname}_{DIRN[model]}_nw", "camera-0.cameramodel"))
    oi = m.optimization_inputs()
    names = [os.path.relpath(p, "../..") if p.startswith("../..") else p for p in oi["imagepaths"]]
    names = [n.replace("../../", "") for n in oi["imagepaths"]]
    return m, oi, names

def caps_of(names, M):
    return np.array([M[n]["cap"] for n in names])

# named test points: centre, r = 0.5 / 0.8 / 1.0 of the half diagonal along the 4 diagonals (same as rectdiff)
def test_points():
    c = np.array([(W - 1) / 2, (H - 1) / 2]); hd = np.array([(W - 1) / 2, (H - 1) / 2])
    P = {"centre": [c]}
    for r in (0.5, 0.8, 1.0):
        P[f"r{r}"] = [c + r * hd * np.array(s) for s in ((1, 1), (-1, 1), (1, -1), (-1, -1))]
    return P

def subset(oi, idx, warp=None, lensmodel=None, intr=None, fix_intr=False, outlier=True, reg=True):
    o = copy.deepcopy(oi)
    idx = np.asarray(idx)
    ob = o["observations_board"][idx].copy()
    ob[..., 2] = np.where(ob[..., 2] < 0, 1.0, ob[..., 2])  # undo previous outlier marks
    o["observations_board"] = ob
    o["frames_rt_toref"] = o["frames_rt_toref"][idx].copy()
    n = len(idx)
    o["indices_frame_camintrinsics_camextrinsics"] = np.c_[np.arange(n), np.zeros(n), -np.ones(n)].astype(np.int32)
    o["imagepaths"] = o["imagepaths"][idx]
    if lensmodel is not None:
        o["lensmodel"] = lensmodel
    if intr is not None:
        o["intrinsics"] = np.asarray(intr, float).reshape(1, -1).copy()
    if warp is None:
        o["calobject_warp"] = None; o["do_optimize_calobject_warp"] = False
    else:
        o["calobject_warp"] = np.array([warp, 0.0]); o["do_optimize_calobject_warp"] = False
    if fix_intr:
        o["do_optimize_intrinsics_core"] = False; o["do_optimize_intrinsics_distortions"] = False
    o["do_apply_outlier_rejection"] = outlier
    o["do_apply_regularization"] = reg
    o["verbose"] = False
    return o

def solve(o):
    s = mrcal.optimize(**o)
    x = s["x"]
    nb = o["observations_board"].shape[0] * 8 * 2
    xb = x[:nb]
    w = o["observations_board"][..., 2].ravel()
    good = np.repeat(w > 0, 2)
    rms = float(np.sqrt(np.mean(xb[good] ** 2)))
    return s, rms, float(np.sum(xb[good] ** 2)), int(good.sum() // 2)

def model_of(o):
    return mrcal.cameramodel(optimization_inputs=o, icam_intrinsics=0)

def unc_at(m, pts, what="worstdirection-stdev"):
    v = mrcal.unproject(np.asarray(pts, float), *m.intrinsics())
    return mrcal.projection_uncertainty(v, model=m, atinfinity=True, what=what)

def unc_summary(m):
    P = test_points(); out = {}
    for k, pts in P.items():
        u = np.atleast_1d(unc_at(m, pts)); out[k] = dict(max=round(float(u.max()), 3), med=round(float(np.median(u)), 3))
    # full grid
    gx, gy = np.meshgrid(np.linspace(0, W - 1, 33), np.linspace(0, H - 1, 19))
    q = np.c_[gx.ravel(), gy.ravel()]
    u = unc_at(m, q)
    out["grid33x19"] = dict(max=round(float(u.max()), 3), med=round(float(np.median(u)), 3), p90=round(float(np.percentile(u, 90)), 3))
    return out, (gx, gy, u.reshape(gx.shape))

def ray_diff(m1, m2, pts):
    """pixel distance when the ray m1 sees at q is projected through m2 (no rotation fit), in 1920 px."""
    q = np.asarray(pts, float)
    v = mrcal.unproject(q, *m1.intrinsics())
    q2 = mrcal.project(v, *m2.intrinsics())
    return np.linalg.norm(q2 - q, axis=-1)

def diff_summary(m1, m2, intrinsics_only):
    P = test_points(); out = {}
    if intrinsics_only:
        for k, pts in P.items():
            d = np.atleast_1d(ray_diff(m1, m2, pts)); out[k] = dict(max=round(float(d.max()), 3), med=round(float(np.median(d)), 3))
        return out
    # mrcal projection_diff with fitted rotation (default focus region), evaluate at test points by grid lookup
    difflen, q0, _, _ = mrcal.projection_diff([m1, m2], gridn_width=61, use_uncertainties=False)
    gy, gx = difflen.shape
    for k, pts in P.items():
        vals = []
        for p in pts:
            i = int(round(p[1] / (H - 1) * (gy - 1))); j = int(round(p[0] / (W - 1) * (gx - 1)))
            vals.append(difflen[i, j])
        out[k] = dict(max=round(float(np.max(vals)), 3), med=round(float(np.median(vals)), 3))
    return out

def opencv_to_mrcal(K, dist):
    d = list(dist) + [0.0] * (5 - len(dist))
    return ("LENSMODEL_OPENCV5", np.array([K[0][0], K[1][1], K[0][2], K[1][2]] + d[:5], float))
