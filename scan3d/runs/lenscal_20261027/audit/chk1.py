import sys, os, json, numpy as np
sys.path.insert(0, os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d/runs/lenscal_20261027/scripts"))
from lenscal_lib import *
import mrcal
M = meta()
# 1. measurement layout
m, oi, names = base("A", "OPENCV4")
o = subset(oi, np.arange(len(names)))
print("Nmeas boards", mrcal.num_measurements_boards(**{k:o[k] for k in o if k in ("indices_frame_camintrinsics_camextrinsics","observations_board","calibration_object_width_n","calibration_object_height_n")}) if False else "", flush=True)
s = mrcal.optimize(**o)
x = s["x"]
print("len x", len(x), "N*16", o["observations_board"].shape, o["observations_board"].shape[0]*16)
try:
    print("idx boards", mrcal.measurement_index_boards(0, **o), "nboards", mrcal.num_measurements_boards(**o), "idx reg", mrcal.measurement_index_regularization(**o), "nreg", mrcal.num_measurements_regularization(**o))
except Exception as e: print("idx err", e)
# compare with direct reprojection
q = mrcal.project(mrcal.transform_point_rt(o["frames_rt_toref"][:,None,None,:], mrcal.ref_calibration_object(4,2,o["calibration_object_spacing"])[None]), *m.intrinsics() if False else (o["lensmodel"], o["intrinsics"][0]))
ob = o["observations_board"]
d = (q - ob[...,:2]); w = ob[...,2]
print("direct rms per-coord (w>0)", np.sqrt(np.mean(d[w>0]**2)), "lib", solve(subset(oi,np.arange(len(names))))[1], "noutl", (w<=0).sum())
print("x board part vs direct max abs diff", np.max(np.abs(x[:ob.shape[0]*16].reshape(d.shape)[w>0]-d[w>0])))
# 2. coverage radius
for S in ["A","B"]:
    m, oi, names = base(S, "OPENCV4")
    q = oi["observations_board"][...,:2].reshape(-1,2)
    c = np.array([(W-1)/2,(H-1)/2]); hd = np.linalg.norm(c)
    r = np.linalg.norm(q-c,axis=1)/hd
    print(S, "coverage r max %.3f p99 %.3f p95 %.3f; frac in corner r>0.8: %.3f" % (r.max(), np.percentile(r,99), np.percentile(r,95), np.mean(r>0.8)))
    # per quadrant max r
    for sx in (-1,1):
        for sy in (-1,1):
            sel = (np.sign(q[:,0]-c[0])==sx)&(np.sign(q[:,1]-c[1])==sy)
            print("  quad", sx, sy, "max r %.3f" % r[sel].max() if sel.any() else "none")
# 3. within-capture residual correlation (ICC) with base solve
for S in ["A","B"]:
    m, oi, names = base(S, "OPENCV4")
    o = subset(oi, np.arange(len(names)), outlier=False); st, rms, _, _ = solve(o)
    x = st["x"][:len(names)*16].reshape(len(names), 16)
    caps = caps_of(names, M); uc = sorted(set(caps))
    # ICC: share of variance explained by capture mean, per measurement coordinate
    within = []; total = np.var(x)
    capmean = np.zeros_like(x)
    for cc in uc:
        i = np.where(caps==cc)[0]; capmean[i] = x[i].mean(0)
    vb = np.var(capmean); vw = np.var(x-capmean)
    k = np.mean([np.sum(caps==cc) for cc in uc])
    icc = (vb - vw/(k-1)*1) / (vb + vw) if k>1 else np.nan
    print(S, "rms_coord %.4f var between-cap-mean %.5f within %.5f  frames/cap %.2f  approx ICC %.2f" % (rms, vb, vw, k, (vb - vw/(k-1))/(vb - vw/(k-1) + vw*k/(k-1))))
