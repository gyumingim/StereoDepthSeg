"""FAR two-marker set: per-pose residual, tilt, distance, marker size; radial (corner-diagonal) residual bias
(is the outer-corner position biased inward/outward?); IPPE alternative pose check. (2026-10-24) → results/far_diag.json"""
import json, sys, time
from pathlib import Path
import cv2
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib
ex = open(HERE / "logs" / "ex_A_nonfar.txt").read().split()
views, clusters, members = calib.build(ex)
rms, K, d, rv, tv = calib.fit(views, "k12_notan")
out = {"fit": dict(fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2], dist=d.tolist(), rms=rms), "poses": {}}
rad_all = []
for v, r_, t_ in zip(views, rv, tv):
    e, res, _, _ = calib.view_err(v, K, d, r_, t_)
    img = v["img"].reshape(-1, 2)
    rad = []
    for m in range(2):
        sel = (v["ids"] // 4) == m
        if sel.sum() == 4:
            c = img[sel].mean(0)
            u = img[sel] - c
            u /= np.linalg.norm(u, axis=1, keepdims=True)
            rad += list(np.sum(res[sel] * u, 1))   # + = projected corner further out than detected (detected inward)
    rad_all += rad
    tl, dist = calib.tilt_dist(r_, t_)
    # IPPE alternatives with the fitted K
    n, rvs, tvs, errs = cv2.solvePnPGeneric(v["obj"].reshape(-1, 3).astype(np.float64), img.astype(np.float64), K, d,
                                            flags=cv2.SOLVEPNP_IPPE)
    alts = []
    for a, b, er in zip(rvs, tvs, np.ravel(errs)):
        alts.append(dict(err=float(er), tilt=calib.tilt_dist(a, b)[0]))
    side = float(np.mean([np.linalg.norm(img[0] - img[1]), np.linalg.norm(img[1] - img[2])]))
    p = out["poses"].setdefault(v["rec"], dict(err=[], tilt=[], dist=[], rad=[], side_px=side, ippe=alts))
    p["err"].append(e); p["tilt"].append(tl); p["dist"].append(dist); p["rad"].append(float(np.mean(rad)) if rad else None)
for k, p in sorted(out["poses"].items()):
    for q in ("err", "tilt", "dist"):
        p[q] = float(np.mean(p[q]))
    p["rad"] = float(np.mean([x for x in p["rad"] if x is not None]))
    print(f"{k[12:18]} err {p['err']:.3f} tilt {p['tilt']:4.1f} dist {p['dist']:.2f} side {p['side_px']:5.1f} radial bias {p['rad']:+.3f} "
          f"ippe " + " | ".join(f"{a['err']:.2f}/{a['tilt']:.0f}" for a in p['ippe']))
out["radial_bias_mean_px"] = float(np.mean(rad_all))
out["radial_bias_sd_px"] = float(np.std(rad_all))
out["fit"] = {k: (float(x) if not isinstance(x, list) else x) for k, x in out["fit"].items()}
print("radial residual (projected - detected along corner diagonal) mean %.3f sd %.3f px" % (np.mean(rad_all), np.std(rad_all)))
out["made"] = time.strftime("%Y-%m-%d %H:%M:%S")
json.dump(out, open(HERE / "results" / "far_diag.json", "w"), indent=1)
