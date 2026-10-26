"""calcap_calib_20261026: extra analysis on top of tools/calcap/calcap_calib.py (same views, same model k12_notan).
Per-view / per-slot reprojection error, outlier views, bootstrap over captures (resample slots with replacement),
per-session fit (session 1 = ISO 1317, session 2 = ISO 625), comparison with the lens values currently in the repo.
Lens values come ONLY from the board fit; nothing here is written to config/ (results only).

  ~/scanner/StereoDepthSeg/venv_ffs/bin/python analyze.py <set dir> <out dir> [--boot 200]
"""
import argparse, json, os, sys, time
import numpy as np, cv2

S3D = os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d")
sys.path.insert(0, os.path.join(S3D, "tools", "calcap"))
import calcap_calib as CC  # noqa: E402
import slots as SL  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("set"); ap.add_argument("out")
ap.add_argument("--boot", type=int, default=200)
ap.add_argument("--model", default="k12_notan")
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
t0 = time.time()
boards = SL.load_boards(os.path.join(S3D, "config", "charuco"))
gK, gd = SL.load_lens(CC.DEFAULT)
views, caps = CC.load_views([a.set], boards, gK, gd, print)
M = a.model
rms, K, d, rv, tv = CC.fit(views, M)
L = CC.summ(K, d)
print("full fit", L, "rms", rms)

# per view / per slot
per = []
for v, r_, t_ in zip(views, rv, tv):
    p, _ = cv2.projectPoints(v["obj"], r_, t_, K, d)
    e = np.linalg.norm((p - v["img"]).reshape(-1, 2), axis=1)
    per.append(dict(file=v["file"], cap=v["cap"], slot=v["slot"], n=len(v["ids"]), rms=float(np.sqrt(np.mean(e ** 2))),
                    max=float(e.max()), tilt=v["tilt"], dist=v["dist"]))
pe = np.array([x["rms"] for x in per])
med = float(np.median(pe)); mad = float(np.median(np.abs(pe - med))) * 1.4826
for x in per:
    x["outlier_2p5med"] = bool(x["rms"] > 2.5 * med)
    x["z_mad"] = (x["rms"] - med) / mad if mad > 0 else 0.0
slots = {}
for x in per:
    s = slots.setdefault(x["cap"], dict(slot=x["slot"], n_views=0, errs=[], tilt=[], dist=[]))
    s["n_views"] += 1; s["errs"].append(x["rms"]); s["tilt"].append(x["tilt"] or 0); s["dist"].append(x["dist"] or 0)
slot_tab = []
for c, s in slots.items():
    slot_tab.append(dict(cap=c, slot=s["slot"], n_views=s["n_views"], rms_mean=float(np.mean(s["errs"])),
                         rms_max=float(np.max(s["errs"])), tilt=float(np.mean(s["tilt"])), dist=float(np.mean(s["dist"]))))
slot_tab.sort(key=lambda x: -x["rms_mean"])
dist_stats = dict(view_rms_min=float(pe.min()), p10=float(np.quantile(pe, .1)), median=med, p90=float(np.quantile(pe, .9)),
                  max=float(pe.max()), mad_sigma=mad, n_outlier_2p5med=int(sum(x["outlier_2p5med"] for x in per)),
                  n_z_mad_gt3=int(sum(x["z_mad"] > 3 for x in per)))
hist_edges = [0, .05, .1, .15, .2, .25, .3, .4, .5, 1, 5]
dist_stats["hist_edges_px"] = hist_edges
dist_stats["hist_counts"] = np.histogram(pe, bins=hist_edges)[0].tolist()

# outlier test: refit without z_mad>3 views
out_files = [x["file"] for x in per if x["z_mad"] > 3]
refit_wo = None
if out_files:
    vs2 = [v for v in views if v["file"] not in out_files]
    r2 = CC.fit(vs2, M); refit_wo = dict(CC.summ(r2[1], r2[2]), rms=r2[0], n_views=len(vs2), removed=out_files)

# bootstrap over captures (slots), resample with replacement
ids = sorted({v["cap"] for v in views})
byc = {c: [v for v in views if v["cap"] == c] for c in ids}
rng = np.random.default_rng(20261026)
B = []
for b in range(a.boot):
    pick = rng.choice(ids, len(ids), replace=True)
    vb = [v for c in pick for v in byc[c]]
    r = CC.fit(vb, M)
    B.append([r[1][0, 0], r[1][1, 1], r[1][0, 2], r[1][1, 2], r[2][0], r[2][1], r[0]])
B = np.array(B)
boot = dict(n=a.boot, unit="capture (slot, 4 stills) resampled with replacement",
            sd=dict(zip(("fx", "fy", "cx", "cy", "k1", "k2"), [float(x) for x in B[:, :6].std(0, ddof=1)])),
            p2_5=dict(zip(("fx", "fy", "cx", "cy"), [float(x) for x in np.quantile(B[:, :4], .025, 0)])),
            p97_5=dict(zip(("fx", "fy", "cx", "cy"), [float(x) for x in np.quantile(B[:, :4], .975, 0)])),
            mean=dict(zip(("fx", "fy", "cx", "cy"), [float(x) for x in B[:, :4].mean(0)])))
np.savetxt(os.path.join(a.out, "bootstrap_fx_fy_cx_cy_k1_k2_rms.csv"), B, delimiter=",", fmt="%.6f",
           header="fx,fy,cx,cy,k1,k2,rms (one row per bootstrap fit)")

# per session (session 1: ISO 1317, 9 caps on 10-25; session 2: ISO 625, 26 caps on 10-26) - mixed: day + ISO + slots
sess = {}
for sname in sorted({v["cap"].split("#")[2] for v in views}):
    vs = [v for v in views if v["cap"].split("#")[2] == sname]
    r = CC.fit(vs, M)
    sess[sname] = dict(CC.summ(r[1], r[2]), rms=r[0], n_views=len(vs), n_caps=len({v["cap"] for v in vs}))

# comparison with lens values found in the repo
def lensj(p):
    j = json.load(open(p)); K_ = np.array(j["K"]); return K_, np.array(j["dist"], float), j
cmp_rows = []
for nm, p in [("cal20261024d (calcap_calib 기본값, 최신 std f0.50)", "config/lens/std_1920x1080_f0.50_cal20261024d.json"),
              ("cal20261024b", "config/lens/std_1920x1080_f0.50_cal20261024b.json"),
              ("cal20261024", "config/lens/std_1920x1080_f0.50_cal20261024.json"),
              ("cal20261023", "config/lens/std_1920x1080_f0.50_cal20261023.json")]:
    K_, d_, j = lensj(os.path.join(S3D, p))
    cmp_rows.append(dict(name=nm, path="scan3d/" + p, fx=K_[0, 0], fy=K_[1, 1], cx=K_[0, 2], cy=K_[1, 2], dist=d_.tolist(), status=j.get("status")))
vj = json.load(open(os.path.join(S3D, "config", "phone_ov_main", "variants.json")))
Kc = np.array(vj["K_raw_used"]); dc = np.array(vj["dist"])
cmp_rows.insert(0, dict(name="phone_ov_main c4/c4f60 원본 렌즈 (variants.json K_raw_used = hs f2.22 cal20261022 x focus 0.98624)",
                        path="scan3d/config/phone_ov_main/variants.json", fx=Kc[0, 0], fy=Kc[1, 1], cx=Kc[0, 2], cy=Kc[1, 2], dist=dc.tolist(),
                        status="in use (c4f60 / rec60), made " + vj["made"]))
for r in cmp_rows:
    r.update(dfx_pct=(L["fx"] - r["fx"]) / r["fx"] * 100, dfy_pct=(L["fy"] - r["fy"]) / r["fy"] * 100, dcx=L["cx"] - r["cx"], dcy=L["cy"] - r["cy"],
             dfx_in_boot_sd=(L["fx"] - r["fx"]) / boot["sd"]["fx"])
# rectified 960x540 pinhole (what the c4f60 yaml holds) for the new lens vs the current yaml
Knew_new, _ = cv2.getOptimalNewCameraMatrix(K, d, (1920, 1080), 0, (960, 540))
Knew_cur = np.array(vj["variant_sizes"]["c4"]["Knew"])
# image-space effect: max pixel difference over the 1920x1080 frame between new and c4f60 undistortion (normalized rays * fx_new)
xs, ys = np.meshgrid(np.linspace(0, 1919, 49), np.linspace(0, 1079, 28))
pts = np.stack([xs.ravel(), ys.ravel()], 1).reshape(-1, 1, 2).astype(np.float64)
un_new = cv2.undistortPoints(pts, K, d).reshape(-1, 2); un_cur = cv2.undistortPoints(pts, Kc, dc).reshape(-1, 2)
ray_diff_px = np.linalg.norm(un_new - un_cur, axis=1) * K[0, 0]
res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), input_set_real=os.path.realpath(os.path.join(a.set, "captures.json")),
           set_dir=a.set, model=M, n_views=len(views), n_caps=len(ids), n_corners=int(sum(len(v["ids"]) for v in views)),
           full_fit=dict(L, rms=rms), view_err_stats=dist_stats, per_slot=slot_tab, per_view=per, refit_without_zmad3=refit_wo,
           bootstrap=boot, per_session=sess, compare=cmp_rows,
           rectified_960x540=dict(new=Knew_new.tolist(), current_c4f60_yaml=Knew_cur.tolist()),
           ray_diff_vs_c4f60_px_at_1920=dict(median=float(np.median(ray_diff_px)), max=float(ray_diff_px.max()),
                                             center=float(ray_diff_px[np.argmin(np.hypot(pts[:, 0, 0] - 960, pts[:, 0, 1] - 540))]),
                                             within_r_le_0p7=float(ray_diff_px[np.hypot(pts[:, 0, 0] - 960, pts[:, 0, 1] - 540) / np.hypot(960, 540) <= 0.7].max()),
                                             note="|undistorted ray(new) - ray(c4f60 raw lens)| x fx_new over a 49x28 grid of the raw 1920x1080 frame; max is at the corners (outside board coverage r_max 0.876)"),
           sec=round(time.time() - t0, 1))
json.dump(res, open(os.path.join(a.out, "analysis.json"), "w"), indent=1, default=float)
print(json.dumps({k: res[k] for k in ("full_fit", "view_err_stats", "refit_without_zmad3", "bootstrap", "per_session",
                                      "rectified_960x540", "ray_diff_vs_c4f60_px_at_1920", "sec")}, indent=1, default=float))
print("slots worst 6:"); [print(" ", s) for s in slot_tab[:6]]
print("compare:"); [print(" ", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items() if k != "dist"}) for r in cmp_rows]
