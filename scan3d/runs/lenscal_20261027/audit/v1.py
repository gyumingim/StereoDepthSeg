import sys, os, numpy as np
sys.path.insert(0, os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d/runs/lenscal_20261027/scripts"))
from lenscal_lib import *
import mrcal
from mrcal import model_analysis as MA
M = meta()
def fx_sd(o):
    b, x, J, f = mrcal.optimizer_callback(**o)[:4]
    D = np.ones_like(b); mrcal.unpack_state(D, **o)
    i0 = mrcal.state_index_intrinsics(0, **o)
    dF = np.zeros((2, len(b))); dF[0, i0] = D[i0]; dF[1, i0+2] = D[i0+2]
    nobs = mrcal.num_measurements_boards(**o)
    sig = np.std(mrcal.residuals_chessboard(o, residuals=x).ravel())
    C = MA._propagate_calibration_uncertainty(dF, f, J, nobs, sig, "covariance")
    return np.sqrt(np.diag(C)), sig
mods = {}
for S in "AB":
    m, oi, names = base(S, "OPENCV4")
    caps = caps_of(names, M); uc = sorted(set(caps))
    full = np.arange(len(names))
    one = np.array([np.where(caps == c)[0][0] for c in uc])
    for tag, idx in (("all", full), ("1percap", one)):
        o = subset(oi, idx); st, rms, _, _ = solve(o)
        o["frames_rt_toref"] = st.get("frames_rt_toref", o["frames_rt_toref"]) if isinstance(st, dict) and "frames_rt_toref" in st else o["frames_rt_toref"]
        mm = model_of(o)
        oo = mm.optimization_inputs()
        sd, sig = fx_sd(oo)
        us, _ = unc_summary(mm)
        mods[(S, tag)] = mm
        print(S, tag, "n", len(idx), "rms %.4f sig %.4f" % (rms, sig), "analytic sd fx %.2f cx %.2f" % tuple(sd), "unc c/r0.5/r0.8/r1", us["centre"]["med"], us["r0.5"]["med"], us["r0.8"]["med"], us["r1.0"]["med"], "fx %.2f" % mm.intrinsics()[1][0], flush=True)
def comb(m1, m2):
    P = test_points(); out = {}
    for k, pts in P.items():
        out[k] = np.sqrt(np.atleast_1d(unc_at(m1, pts))**2 + np.atleast_1d(unc_at(m2, pts))**2).max()
    return out
for tag in ("all", "1percap"):
    d = diff_summary(mods[("A", tag)], mods[("B", tag)], False); u = comb(mods[("A", tag)], mods[("B", tag)])
    print("AvsB", tag, {k: (d[k]["max"], round(float(u[k]), 3), round(d[k]["max"] / u[k], 2)) for k in d})
# ratio of all-frames AvsB diff to 1percap uncertainty (diff is from full data)
d = diff_summary(mods[("A", "all")], mods[("B", "all")], False); u = comb(mods[("A", "1percap")], mods[("B", "1percap")])
print("AvsB diff(all)/unc(1percap)", {k: round(d[k]["max"] / u[k], 2) for k in d})
