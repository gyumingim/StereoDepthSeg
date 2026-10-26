"""calcap_calib_20261026: distortion difference in the c4f60 rectified 960x540 image (coordinator request 2026-10-26).
Fixed rectified K = current c4f60 yaml K (652.10 ...). For a rectified pixel q: ray = K_rect^-1 q -> raw pixel with lens A
(distort + K_A) -> rectify that raw pixel with lens B (undistort with K_B, d_B, then K_rect) -> |q' - q| = where the two
models put the same ray, in rectified px. A = new k12_notan fit, B = c4f60 raw lens / cal20261024d; bootstrap lenses vs
new = spread of the new fit computed the same way. Also: getOptimalNewCameraMatrix(alpha 0) per lens (why 659.35 vs 652.10).
Results only; no app/config file changed.   python rectdiff.py <run dir>"""
import json, math, os, sys, time
import numpy as np, cv2
O = sys.argv[1]
S3D = os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d")
cal = json.load(open(os.path.join(O, "calib_20261026.json")))["models"]["k12_notan"]
Kn = np.array(cal["K"]); dn = np.array((cal["dist"] + [0] * 5)[:5], float)
vj = json.load(open(os.path.join(S3D, "config", "phone_ov_main", "variants.json")))
Kc = np.array(vj["K_raw_used"]); dc = np.array(vj["dist"], float)
j24 = json.load(open(os.path.join(S3D, "config", "lens", "std_1920x1080_f0.50_cal20261024d.json")))
K24 = np.array(j24["K"]); d24 = np.array(j24["dist"], float)
Kr = np.array(vj["variant_sizes"]["c4"]["Knew"])          # current c4f60 rectified K, 960x540
SZ = (960, 540)
# grid: centre, r = 0.5, 0.8 (of the half-diagonal) at 8 angles, 4 corners (1 px inside)
hd = math.hypot(480, 270); pts = {"centre": [(479.5, 269.5)]}
for r in (0.5, 0.8):
    pts[f"r{r}"] = [(479.5 + r * hd * math.cos(a) * 1.0, 269.5 + r * hd * math.sin(a)) for a in np.radians(np.arange(0, 360, 45))]
    pts[f"r{r}"] = [(min(max(x, 0), 959), min(max(y, 0), 539)) for x, y in pts[f"r{r}"]]
pts["corners"] = [(0, 0), (959, 0), (0, 539), (959, 539)]
def diff(KA, dA, KB, dB, q):
    q = np.array(q, float).reshape(-1, 1, 2)
    ray = cv2.undistortPoints(q, Kr, None).reshape(-1, 2)                 # normalised pinhole
    obj = np.concatenate([ray, np.ones((len(ray), 1))], 1)
    raw, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), KA, dA)       # raw pixel under lens A
    q2 = cv2.undistortPoints(raw, KB, dB, P=Kr).reshape(-1, 2)             # rectified under lens B
    return np.linalg.norm(q2 - q.reshape(-1, 2), axis=1), raw.reshape(-1, 2)
res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), K_rect_fixed=Kr.tolist(), note=__doc__, groups={})
boot = np.loadtxt(os.path.join(O, "bootstrap_fx_fy_cx_cy_k1_k2_rms.csv"), delimiter=",")
for g, q in pts.items():
    e_c, raw = diff(Kn, dn, Kc, dc, q); e_24, _ = diff(Kn, dn, K24, d24, q)
    eb = []
    for b in boot:
        Kb = np.array([[b[0], 0, b[2]], [0, b[1], b[3]], [0, 0, 1]]); db = np.array([b[4], b[5], 0, 0, 0])
        eb.append(diff(Kn, dn, Kb, db, q)[0])
    eb = np.array(eb)
    rr = np.hypot(raw[:, 0] - 960, raw[:, 1] - 540) / math.hypot(960, 540)
    res["groups"][g] = dict(n=len(q), raw_r_of_points=[round(float(x), 3) for x in rr],
                            vs_c4f60=dict(max=float(e_c.max()), median=float(np.median(e_c))),
                            vs_24d=dict(max=float(e_24.max()), median=float(np.median(e_24))),
                            boot_p95_per_point_max=float(np.quantile(eb, .95, axis=0).max()),
                            boot_p95_per_point_median=float(np.median(np.quantile(eb, .95, axis=0))),
                            boot_sd_per_point_median=float(np.median(np.sqrt(np.mean(eb ** 2, 0)))),
                            c4f60_inside_boot_p95=[bool(x) for x in e_c <= np.quantile(eb, .95, axis=0)],
                            d24_inside_boot_p95=[bool(x) for x in e_24 <= np.quantile(eb, .95, axis=0)])
res["optimal_new_K_alpha0_960x540"] = {nm: cv2.getOptimalNewCameraMatrix(K, d, (1920, 1080), 0, SZ)[0].tolist()
                                       for nm, K, d in (("new", Kn, dn), ("c4f60_raw", Kc, dc), ("cal20261024d", K24, d24))}
res["optimal_new_K_note"] = "alpha 0 = largest rectified view with no invalid pixels; its focal depends on the distortion at the frame edge, so equal raw fx can give different rectified fx"
json.dump(res, open(os.path.join(O, "rectdiff_960x540.json"), "w"), indent=1, default=float)
for g, x in res["groups"].items():
    print(g, "raw r", x["raw_r_of_points"][:4], "c4f60 max/med %.2f/%.2f" % (x["vs_c4f60"]["max"], x["vs_c4f60"]["median"]),
          "24d %.2f/%.2f" % (x["vs_24d"]["max"], x["vs_24d"]["median"]), "boot p95 max/med %.2f/%.2f" % (x["boot_p95_per_point_max"], x["boot_p95_per_point_median"]),
          "c4 in", sum(x["c4f60_inside_boot_p95"]), "24d in", sum(x["d24_inside_boot_p95"]), "/", x["n"])
for k, v in res["optimal_new_K_alpha0_960x540"].items(): print(k, [round(v[0][0], 2), round(v[1][1], 2), round(v[0][2], 2), round(v[1][2], 2)])
