"""calcap_calib_20261026 part 2 (coordinator request 2026-10-26): coverage map, tilt/distance distribution, what drives
the fx/cx/cy spread (leave-one-slot-out + group removal + stills-per-slot), next-capture suggestions (Fisher information
on the current fit, slot-definition format, suggestion only), board square-size / aspect / flatness sensitivity,
held-out (cross-validated by slot) reprojection error incl. the lenses already in the repo.
Board fit only - no VIO numbers used anywhere.

  ~/scanner/StereoDepthSeg/venv_ffs/bin/python analyze2.py <set dir> <out dir>
"""
import argparse, json, math, os, sys, time
import numpy as np, cv2

S3D = os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d")
sys.path.insert(0, os.path.join(S3D, "tools", "calcap"))
import calcap_calib as CC  # noqa: E402
import slots as SL  # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument("set"); ap.add_argument("out"); a = ap.parse_args()
t0 = time.time()
W, H = 1920, 1080
M = "k12_notan"
boards = SL.load_boards(os.path.join(S3D, "config", "charuco"))
B5 = [b for b in boards if b.name == "board_laptop_5x3"][0]
gK, gd = SL.load_lens(CC.DEFAULT)
views, caps = CC.load_views([a.set], boards, gK, gd, lambda s: None)
rms, K, d, rv, tv = CC.fit(views, M)
F0 = np.array([K[0, 0], K[1, 1], K[0, 2], K[1, 2]])
ids = sorted({v["cap"] for v in views})
byc = {c: [v for v in views if v["cap"] == c] for c in ids}
slot_of = {c: byc[c][0]["slot"] for c in ids}
R = {}

def fitK(vs):
    r = CC.fit(vs, M); return np.array([r[1][0, 0], r[1][1, 1], r[1][0, 2], r[1][1, 2]]), r

# ---------- 1. coverage map, tilt / distance distribution
pts = np.concatenate([v["img"].reshape(-1, 2) for v in views])
gx, gy = 8, 6
cnt = np.zeros((gy, gx), int)
for x, y in pts:
    cnt[min(int(y / H * gy), gy - 1), min(int(x / W * gx), gx - 1)] += 1
cnt3 = np.zeros((3, 3), int)
for x, y in pts:
    cnt3[min(int(y / H * 3), 2), min(int(x / W * 3), 2)] += 1
rr = np.hypot(pts[:, 0] - W / 2, pts[:, 1] - H / 2) / np.hypot(W / 2, H / 2)
redges = [0, .2, .4, .6, .7, .8, .9, 1.0]
# per-view tilt using the fitted lens (not the guide lens)
tilts, dists, turns, leans = [], [], [], []
for v, r_, t_ in zip(views, rv, tv):
    Rm, _ = cv2.Rodrigues(r_); n = Rm[:, 2]
    tilts.append(math.degrees(math.acos(min(1, abs(n[2])))))
    turns.append(math.degrees(math.atan2(n[0], abs(n[2])))); leans.append(math.degrees(math.atan2(n[1], abs(n[2]))))
    dists.append(float(np.linalg.norm(Rm @ B5.centre + t_.ravel())))
tilts, dists = np.array(tilts), np.array(dists)
R["coverage"] = dict(
    grid_8x6_corner_counts=cnt.tolist(), grid_note="rows top->bottom, cols left->right, cell = 240 x 180 px; counts = detected corners (132 views x 8)",
    empty_cells_8x6=int((cnt == 0).sum()), grid_3x3_counts=cnt3.tolist(),
    radius_edges=redges, radius_counts=np.histogram(rr, bins=redges)[0].tolist(),
    radius_note="r = distance from image centre / half-diagonal (1101 px); r>0.9 = the outer image corners",
    tilt_edges_deg=[0, 10, 20, 30, 40, 50, 60], tilt_counts_views=np.histogram(tilts, bins=[0, 10, 20, 30, 40, 50, 60])[0].tolist(),
    tilt_max=float(tilts.max()), turn_range=[float(min(turns)), float(max(turns))], lean_range=[float(min(leans)), float(max(leans))],
    dist_edges_m=[.4, .5, .6, .7, .8, .9, 1.0, 1.2], dist_counts_views=np.histogram(dists, bins=[.4, .5, .6, .7, .8, .9, 1.0, 1.2])[0].tolist(),
    dist_range=[float(dists.min()), float(dists.max())],
    per_slot=[dict(slot=slot_of[c], tilt=float(np.mean([tilts[i] for i, v in enumerate(views) if v["cap"] == c])),
                   turn=float(np.mean([turns[i] for i, v in enumerate(views) if v["cap"] == c])),
                   lean=float(np.mean([leans[i] for i, v in enumerate(views) if v["cap"] == c])),
                   dist=float(np.mean([dists[i] for i, v in enumerate(views) if v["cap"] == c])),
                   bbox=[float(x) for x in np.concatenate([np.concatenate([v["img"].reshape(-1, 2) for v in byc[c]]).min(0),
                                                            np.concatenate([v["img"].reshape(-1, 2) for v in byc[c]]).max(0)])]) for c in ids])

# ---------- 2. what drives the spread
lopo = []
for c in ids:
    k_, _ = fitK([v for v in views if v["cap"] != c])
    dd = k_ - F0
    lopo.append(dict(slot=slot_of[c], dfx=float(dd[0]), dfy=float(dd[1]), dcx=float(dd[2]), dcy=float(dd[3])))
def top(key, n=5):
    return sorted(lopo, key=lambda x: -abs(x[key]))[:n]
groups = {
    "no_frontal_F": lambda s: not s.startswith("F-"),
    "no_T20": lambda s: not s.startswith("T20"),
    "no_40deg_centre_T40": lambda s: not s.startswith("T40"),
    "no_edge_E40": lambda s: not s.startswith("E40"),
    "no_corner_K40": lambda s: not s.startswith("K40"),
    "no_edge_and_corner(E40+K40)": lambda s: not (s.startswith("E40") or s.startswith("K40")),
    "only_tilted(T20+T40+E40+K40)": lambda s: not s.startswith("F-"),
    "no_session1(ISO1317)": None,
}
grp = {}
for g, f in groups.items():
    if f is None:
        vs = [v for v in views if v["cap"].split("#")[2] != "20261025_214259"]
    else:
        vs = [v for v in views if f(v["slot"])]
    k_, r_ = fitK(vs)
    grp[g] = dict(n_slots=len({v["cap"] for v in vs}), fx=float(k_[0]), fy=float(k_[1]), cx=float(k_[2]), cy=float(k_[3]),
                  dfx_pct=float((k_[0] - F0[0]) / F0[0] * 100), dcx=float(k_[2] - F0[2]), dcy=float(k_[3] - F0[3]),
                  k1=float(r_[2][0]), k2=float(r_[2][1]), rms=float(r_[0]))
# stills per slot: 1 still per slot vs 4 (are the 4 stills independent information?)
rng = np.random.default_rng(7)
one = []
for b in range(100):
    vs = [byc[c][rng.integers(len(byc[c]))] for c in rng.choice(ids, len(ids), replace=True)]
    one.append(fitK(vs)[0])
one = np.array(one)
four = np.loadtxt(os.path.join(a.out, "bootstrap_fx_fy_cx_cy_k1_k2_rms.csv"), delimiter=",")[:, :4]
# formal (corner-noise-only) sd, for the ratio
ext = cv2.calibrateCameraExtended([v["obj"] for v in views], [v["img"] for v in views], (W, H), None, None,
                                  flags=CC.FLAGS[M], criteria=CC.TERM)
sd_formal = ext[5].ravel()[:4]
R["spread_sources"] = dict(
    lopo=lopo, lopo_top_fx=top("dfx"), lopo_top_cx=top("dcx"), lopo_top_cy=top("dcy"),
    lopo_jackknife_sd=dict(zip(("fx", "fy", "cx", "cy"), [float(math.sqrt((len(lopo) - 1) / len(lopo) * sum((x[k] - np.mean([y[k] for y in lopo])) ** 2 for x in lopo)))
                                                         for k in ("dfx", "dfy", "dcx", "dcy")])),
    group_removal=grp,
    bootstrap_sd_4stills=dict(zip(("fx", "fy", "cx", "cy"), four.std(0, ddof=1).tolist())),
    bootstrap_sd_1still_per_slot=dict(zip(("fx", "fy", "cx", "cy"), one.std(0, ddof=1).tolist())),
    formal_sd=dict(zip(("fx", "fy", "cx", "cy"), sd_formal.tolist())),
    ratio_boot_over_formal=dict(zip(("fx", "fy", "cx", "cy"), (four.std(0, ddof=1) / sd_formal).tolist())))

# ---------- 3. Fisher information: current set and candidate new slots
SIG = float(rms)
def view_info(obj, r_, t_, Kc, dc):
    _, J = cv2.projectPoints(obj, r_, t_, Kc, dc)
    # cols: rvec 3, tvec 3, fx fy, cx cy, k1 k2 p1 p2 k3 ; keep pose 6 + fx fy cx cy k1 k2
    Ji = J[:, [6, 7, 8, 9, 10, 11]]; Jp = J[:, :6]
    Iii = Ji.T @ Ji; Iip = Ji.T @ Jp; Ipp = Jp.T @ Jp
    return (Iii - Iip @ np.linalg.solve(Ipp, Iip.T)) / SIG ** 2
dc5 = np.zeros(5); dc5[:2] = d[:2]
I0 = sum(view_info(v["obj"], r_, t_, K, dc5) for v, r_, t_ in zip(views, rv, tv))
sd_fisher = np.sqrt(np.diag(np.linalg.inv(I0)))[:4]
infl = four.std(0, ddof=1) / sd_fisher   # empirical inflation (model error / correlated stills), applied to predictions

def board_pose(u, v_, dist, turn, lean, roll=0.0):
    """board centre on the ray through pixel (u, v) at distance dist; normal tilted by turn (about y) / lean (about x)."""
    ray = cv2.undistortPoints(np.array([[[u, v_]]], float), K, dc5).ravel(); ray = np.array([ray[0], ray[1], 1.0]); ray /= np.linalg.norm(ray)
    Ry = cv2.Rodrigues(np.array([0, math.radians(turn), 0.0]))[0]; Rx = cv2.Rodrigues(np.array([math.radians(-lean), 0, 0.0]))[0]
    Rz = cv2.Rodrigues(np.array([0, 0, math.radians(roll)]))[0]
    Rm = Ry @ Rx @ Rz            # board z axis = camera z (facing camera, board normal along +z) then tilted
    t = ray * dist - Rm @ B5.centre
    return cv2.Rodrigues(Rm)[0], t.reshape(3, 1)

all_ids = np.arange(B5.n)
def candidate_ok(r_, t_):
    # full board (squares incl. border) must be in the image with 20 px margin, all 8 corners detected
    sq = B5.sq; outline = np.array([[0, 0, 0], [B5.sx * sq, 0, 0], [B5.sx * sq, B5.sy * sq, 0], [0, B5.sy * sq, 0]], float)
    p, _ = cv2.projectPoints(outline, r_, t_, K, dc5); p = p.reshape(-1, 2)
    return bool((p[:, 0] > 20).all() and (p[:, 0] < W - 20).all() and (p[:, 1] > 20).all() and (p[:, 1] < H - 20).all())
POS = {"C": (960, 540), "T": (960, 200), "B": (960, 880), "L": (330, 540), "R": (1590, 540),
       "TL": (330, 200), "TR": (1590, 200), "BL": (330, 880), "BR": (1590, 880),
       "B2": (960, 960), "BL2": (260, 940), "BR2": (1660, 940), "TL2": (260, 130), "TR2": (1660, 130), "L2": (220, 540), "R2": (1700, 540)}
cands = []
for pn, (u, v_) in POS.items():
    for dist in (0.45, 0.6, 0.8):
        for tilt in (0, 20, 35, 45, 55):
            dirs = [(0, 0)] if tilt == 0 else [(tilt, 0), (-tilt, 0), (0, tilt), (0, -tilt),
                                                (tilt / 1.414, tilt / 1.414), (-tilt / 1.414, tilt / 1.414),
                                                (tilt / 1.414, -tilt / 1.414), (-tilt / 1.414, -tilt / 1.414)]
            for tu, le in dirs:
                r_, t_ = board_pose(u, v_, dist, tu, le)
                if not candidate_ok(r_, t_):
                    continue
                Iv = view_info(B5.obj, r_, t_, K, dc5) * 4      # 4 stills as in the set
                cands.append(dict(pos=pn, u=u, v=v_, dist=dist, tilt=tilt, turn=round(tu, 1), lean=round(le, 1), I=Iv))
def score(I):
    s = np.sqrt(np.diag(np.linalg.inv(I)))[:4]
    return s, s[0] / F0[0] * 100 / 0.3 + s[2] / 3 + s[3] / 3          # weights = the repeatability rule (fx 0.3 %, pp 3 px)
s_base, sc_base = score(I0)
picked, I = [], I0.copy()
for step in range(8):
    best = min(cands, key=lambda c: score(I + c["I"])[1])
    I = I + best["I"]
    s_new, _ = score(I)
    picked.append(dict({k: best[k] for k in ("pos", "u", "v", "dist", "tilt", "turn", "lean")},
                       pred_sd_fisher=s_new.tolist(), pred_sd_inflated=(s_new * infl).tolist()))
    cands = [c for c in cands if c is not best]
# singles ranking (each candidate alone added to current set)
single = sorted(({**{k: c[k] for k in ("pos", "dist", "tilt", "turn", "lean")}, "sc": score(I0 + c["I"])[1]} for c in cands), key=lambda x: x["sc"])[:10]
def slot_fmt(p, i):
    tilt = p["tilt"]; ax = "turn" if abs(p["turn"]) >= abs(p["lean"]) else "lean"; sg = 1 if (p["turn"] if ax == "turn" else p["lean"]) >= 0 else -1
    diag = abs(p["turn"]) > 1 and abs(p["lean"]) > 1
    kind = "F" if tilt == 0 else f"N{tilt}"
    sid = f"{kind}-{p['pos']}" + ("" if tilt == 0 else (f"-diag{'+' if p['turn'] > 0 else '-'}{'+' if p['lean'] > 0 else '-'}" if diag else f"-{ax}{'+' if sg > 0 else '-'}")) + f"-{p['dist']:.2f}m"
    return dict(id=sid, pos=p["pos"], kind=kind, ax=None if tilt == 0 or diag else ax, sign=None if tilt == 0 or diag else sg,
                turn_deg=p["turn"], lean_deg=p["lean"], dist_m=p["dist"], target_px=[p["u"], p["v"]], stills=4,
                label=f"제안 {i + 1}: 위치 {p['pos']} ({p['u']},{p['v']}) · 기울기 {tilt}° (돌림 {p['turn']}, 젖힘 {p['lean']}) · {p['dist']} m")
R["next_capture"] = dict(
    method="Fisher information of the current k12_notan fit (corner sigma = fit RMS %.3f px, per-view pose eliminated), "
           "greedy pick of candidate poses of the 5x3 laptop board (whole board inside the frame, 4 stills each); "
           "score = sd_fx%%/0.3 + sd_cx/3 + sd_cy/3. Predicted sd x empirical inflation (bootstrap / Fisher) = 'inflated'. Estimate only." % SIG,
    fisher_sd_now=s_base.tolist(), bootstrap_sd_now=four.std(0, ddof=1).tolist(), inflation=infl.tolist(),
    greedy_8=picked, slot_format=[slot_fmt(p, i) for i, p in enumerate(picked)], best_single=single,
    note="POS with suffix 2 = beyond the current 3x3 targets (closer to the frame edge). Suggestion only; slots.py not changed.")

# ---------- 4. board size / aspect / flatness
def fit_obj_scaled(sx, sy):
    vs = [dict(v, obj=(v["obj"] * np.array([sx, sy, 1], np.float32)).astype(np.float32)) for v in views]
    return fitK(vs)[0]
k_s = fit_obj_scaled(1.0017, 1.0017); k_a = fit_obj_scaled(1.002, 1.0); k_a2 = fit_obj_scaled(1.0, 1.002)
# mean residual per board corner id (systematic pattern = non-flat / wrong geometry), in board-aligned pixels
res_by_id = {i: [] for i in range(B5.n)}
for v, r_, t_ in zip(views, rv, tv):
    if v["board"] != B5.name: continue
    p, _ = cv2.projectPoints(v["obj"], r_, t_, K, d)
    e = (v["img"] - p).reshape(-1, 2)
    for i, ee in zip(v["ids"], e): res_by_id[int(i)].append(ee)
rid = {i: dict(n=len(x), mean_dx=float(np.mean(x, 0)[0]), mean_dy=float(np.mean(x, 0)[1]), mean_abs=float(np.mean(np.linalg.norm(x, axis=1))),
               sem=float(np.std(np.linalg.norm(x, axis=1)) / math.sqrt(len(x)))) for i, x in res_by_id.items() if x}
# bow test: give the board a parabolic bow z = b * (1 - (x-xc)^2/a^2) and see fx change for b = 0.5 mm
def bow(bmm):
    vs = []
    for v in views:
        o = v["obj"].astype(np.float64).copy(); xc, yc = B5.centre[0], B5.centre[1]
        o[:, 2] = bmm / 1000 * (1 - ((o[:, 0] - xc) / xc) ** 2)
        vs.append(dict(v, obj=o.astype(np.float32)))
    r = cv2.calibrateCamera([v["obj"] for v in vs], [v["img"] for v in vs], (W, H), K.copy(), None,
                            flags=CC.FLAGS[M] | cv2.CALIB_USE_INTRINSIC_GUESS, criteria=CC.TERM)
    return np.array([r[1][0, 0], r[1][1, 1], r[1][0, 2], r[1][1, 2]]), r[0]
kb, rb = bow(0.5)
R["board"] = dict(
    square_mm=B5.spec["square_mm"], source=B5.spec["square_mm_source"], square_estimated=B5.spec["square_estimated"],
    uniform_scale_plus_0p17pct=dict(dfx=float(k_s[0] - F0[0]), dfy=float(k_s[1] - F0[1]), dcx=float(k_s[2] - F0[2]), dcy=float(k_s[3] - F0[3]),
                                    note="uniform square-size error only scales the board distance (planar target); intrinsics unchanged"),
    aspect_x_plus_0p2pct=dict(dfx=float(k_a[0] - F0[0]), dfy=float(k_a[1] - F0[1]), dcx=float(k_a[2] - F0[2]), dcy=float(k_a[3] - F0[3])),
    aspect_y_plus_0p2pct=dict(dfx=float(k_a2[0] - F0[0]), dfy=float(k_a2[1] - F0[1]), dcx=float(k_a2[2] - F0[2]), dcy=float(k_a2[3] - F0[3])),
    bow_0p5mm=dict(dfx=float(kb[0] - F0[0]), dfy=float(kb[1] - F0[1]), dcx=float(kb[2] - F0[2]), dcy=float(kb[3] - F0[3]), rms=float(rb), rms_flat=float(rms),
                   note="screen modelled as a 0.5 mm cylindrical bow along x (unmeasured; sensitivity only)"),
    residual_by_corner_id=rid)

# ---------- 5. held-out reprojection error (5-fold by slot, 10 repeats); lens fixed, pose by PnP on each held-out still
def pnp_err(v, Kc, dc_):
    ok, r_, t_ = cv2.solvePnP(v["obj"], v["img"], Kc, dc_, flags=cv2.SOLVEPNP_ITERATIVE)
    p, _ = cv2.projectPoints(v["obj"], r_, t_, Kc, dc_)
    return float(np.sqrt(np.mean(np.sum((p - v["img"]).reshape(-1, 2) ** 2, -1))))
# pose-shared variant: one pose per held-out slot from its 4 stills is not possible with PnP per still; also report
# "pose from the other 3 stills -> error on the 4th" (pose not fitted to the tested still)
def loo_still_err(vs, Kc, dc_):
    out = []
    for i, v in enumerate(vs):
        o = np.concatenate([w["obj"] for j, w in enumerate(vs) if j != i]); im = np.concatenate([w["img"] for j, w in enumerate(vs) if j != i])
        ok, r_, t_ = cv2.solvePnP(o, im, Kc, dc_, flags=cv2.SOLVEPNP_ITERATIVE)
        p, _ = cv2.projectPoints(v["obj"], r_, t_, Kc, dc_)
        out.append(float(np.sqrt(np.mean(np.sum((p - v["img"]).reshape(-1, 2) ** 2, -1)))))
    return out
rng = np.random.default_rng(11)
cv_pnp, cv_loo = [], []
for rep in range(10):
    perm = rng.permutation(ids); folds = np.array_split(perm, 5)
    for f in folds:
        tr = [v for v in views if v["cap"] not in set(f)]; _, r_ = fitK(tr)
        for c in f:
            cv_pnp += [pnp_err(v, r_[1], r_[2]) for v in byc[c]]
            cv_loo += loo_still_err(byc[c], r_[1], r_[2])
others = {}
vj = json.load(open(os.path.join(S3D, "config", "phone_ov_main", "variants.json")))
lens_list = {"c4f60_raw(K_raw_used)": (np.array(vj["K_raw_used"]), np.array(vj["dist"]))}
for t in ("24d", "24b", "24", "23"):
    j = json.load(open(os.path.join(S3D, "config", "lens", f"std_1920x1080_f0.50_cal202610{t}.json")))
    lens_list[f"cal202610{t}"] = (np.array(j["K"]), np.array(j["dist"], float))
lens_list["new_full_fit(in-sample)"] = (K, d)
for nm, (Kc, dc_) in lens_list.items():
    e1 = [pnp_err(v, Kc, dc_) for v in views]; e2 = [x for c in ids for x in loo_still_err(byc[c], Kc, dc_)]
    others[nm] = dict(pnp_median=float(np.median(e1)), pnp_mean=float(np.mean(e1)), loo_still_median=float(np.median(e2)), loo_still_mean=float(np.mean(e2)))
R["holdout"] = dict(
    method="5-fold by slot x 10 repeats: fit on 4/5 of the slots, held-out stills scored with the fitted lens fixed. "
           "pnp = pose fitted to the tested still itself (8 corners, 6 dof -> optimistic); loo_still = pose from the other 3 stills of the same slot, error on the 4th "
           "(pose not fitted to the tested points). Repo lenses are scored on all stills (they were not fitted to this set).",
    new_cv=dict(pnp_median=float(np.median(cv_pnp)), pnp_mean=float(np.mean(cv_pnp)), loo_still_median=float(np.median(cv_loo)), loo_still_mean=float(np.mean(cv_loo)), n=len(cv_pnp)),
    repo_lenses=others)
R["made"] = time.strftime("%Y-%m-%d %H:%M:%S"); R["set"] = a.set; R["input_real"] = os.path.realpath(os.path.join(a.set, "20261026_204528"))
R["sec"] = round(time.time() - t0, 1)
json.dump(R, open(os.path.join(a.out, "analysis2.json"), "w"), indent=1, default=float)
pr = {k: R[k] for k in ("coverage",)}; pr["coverage"] = {k: v for k, v in R["coverage"].items() if k != "per_slot"}
print(json.dumps(pr, indent=0, default=float))
ss = R["spread_sources"]
print(json.dumps({k: ss[k] for k in ss if k != "lopo"}, indent=0, default=float))
nc = R["next_capture"]; print(json.dumps({k: nc[k] for k in nc if k not in ("slot_format",)}, indent=0, default=float))
for s in nc["slot_format"]: print(s)
bd = R["board"]; print(json.dumps({k: bd[k] for k in bd}, indent=0, default=float))
print(json.dumps(R["holdout"], indent=0, default=float)); print("sec", R["sec"])
