"""lenscal an3: fx/cx/cy at the best board warp per set, fixed lenses scored with warp, figures (uncertainty map, warp profile)."""
import json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lenscal_lib import *
import mrcal
t0 = time.time()
R2 = json.load(open(os.path.join(T, "an2.json")))
OUT = os.path.expanduser(sys.argv[1]) if len(sys.argv) > 1 else T
R = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), at_best_warp={}, fixed_with_warp={})
LENS = os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d/config")
V = json.load(open(os.path.join(LENS, "phone_ov_main", "variants.json")))
L24d = json.load(open(os.path.join(LENS, "lens", "std_1920x1080_f0.50_cal20261024d.json")))
fixed = {"c4f60_app": opencv_to_mrcal(V["K_raw_used"], V["dist"]), "cal20261024d": opencv_to_mrcal(L24d["K"], L24d["dist"])}
for s in ["A", "B", "AB"]:
    for mk in ["OPENCV4", "OPENCV8"]:
        wb = R2["warp"][f"{s}_{mk}"]["w_best_mm"]
        m0, oi, names = base(s, mk)
        o = subset(oi, np.arange(len(names)), warp=wb / 1000.0); st, rms, cost, n = solve(o)
        mm = model_of(o); us, _ = unc_summary(mm)
        it = o["intrinsics"][0]
        R["at_best_warp"][f"{s}_{mk}"] = dict(w_mm=wb, rms_coord=round(rms, 4), fx=round(float(it[0]), 2), fy=round(float(it[1]), 2),
                                             cx=round(float(it[2]), 2), cy=round(float(it[3]), 2), k=[round(float(x), 5) for x in it[4:8]], unc=us)
        print("best warp", s, mk, R["at_best_warp"][f"{s}_{mk}"], flush=True)
for s in ["A", "B"]:
    m0, oi, names = base(s, "OPENCV4")
    for w in [0.0, 0.09, 0.13, 0.19]:
        row = {}
        o = subset(oi, np.arange(len(names)), warp=w / 1000.0, outlier=False); solve(o)
        of = subset(oi, np.arange(len(names)), warp=w / 1000.0, intr=o["intrinsics"][0], fix_intr=True, outlier=False)
        row["own_OPENCV4"] = round(solve(of)[1], 4)
        for name, (lm, itf) in fixed.items():
            of = subset(oi, np.arange(len(names)), warp=w / 1000.0, lensmodel=lm, intr=itf, fix_intr=True, outlier=False)
            row[name] = round(solve(of)[1], 4)
        R["fixed_with_warp"][f"{s}_w{w}"] = row
        print("fixed with warp", s, w, row, flush=True)
# figures
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 3, figsize=(16, 4.2))
for a, s in zip(ax, ["A", "B", "AB"]):
    u = np.load(os.path.join(T, f"unc_{s}_OPENCV4.npy"))
    im = a.imshow(u, extent=[0, W, H, 0], vmin=0, vmax=1.5, cmap="viridis")
    cs = a.contour(np.linspace(0, W - 1, u.shape[1]), np.linspace(0, H - 1, u.shape[0]), u, levels=[0.25, 0.5, 1.0], colors="w", linewidths=0.8)
    a.clabel(cs, fmt="%.2f", fontsize=7)
    m0, oi, names = base(s, "OPENCV4")
    q = oi["observations_board"][..., :2].reshape(-1, 2)
    a.plot(q[:, 0], q[:, 1], ",", color="r", alpha=0.5)
    a.set_title(f"set {s} (red = observed corners)", fontsize=9)
fig.colorbar(im, ax=ax, shrink=0.8)
fig.suptitle("mrcal OPENCV4, no warp: projection uncertainty at infinity, worst direction (px, 1920x1080)", fontsize=10)
fig.savefig(os.path.join(OUT, "uncertainty_maps.png"), dpi=110, bbox_inches="tight")
fig, ax = plt.subplots(1, 2, figsize=(11, 4))
for s, c in zip(["A", "B", "AB"], ["C0", "C1", "C2"]):
    w = R2["warp"][f"{s}_OPENCV4"]
    r = np.array(w["rms"]); ax[0].plot(w["w_mm"], r / r.min(), "-o", ms=3, color=c, label=f"{s} (best {w['w_best_mm']} mm)")
    ax[1].plot(w["w_mm"], w["fx"], "-o", ms=3, color=c, label=s)
ax[0].set_xlabel("board x-bow calobject_warp[0] (mm, mrcal sign)"); ax[0].set_ylabel("RMS / min RMS"); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
ax[1].set_xlabel("board x-bow (mm)"); ax[1].set_ylabel("fx (px)"); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
fig.suptitle("Laptop-screen board: fit vs assumed bow (OPENCV4, mrcal)", fontsize=10)
fig.savefig(os.path.join(OUT, "warp_profile.png"), dpi=110, bbox_inches="tight")
R["sec"] = round(time.time() - t0, 1)
json.dump(R, open(os.path.join(T, "an3.json"), "w"), indent=1, default=float)
print("done", R["sec"])
