"""calcap_calib_20261026 part 3 (user request 2026-10-26): board pose as roll / pitch / yaw (signed) instead of one tilt,
leave-out contribution by pitch+/- and yaw+/-, next-capture suggestions on the 3 axes, capture motion vs reprojection
error, strict (motion < 0.1 px) subset vs full fit. Board fit only - no VIO numbers.

Axes (camera frame, OpenCV: x right, y down, z forward along the optical axis). Board rotation board->camera
R = Ry(yaw) * Rx(pitch) * Rz(roll)  (intrinsic Y-X'-Z'': yaw first about camera y, then pitch about the new x, then roll
about the board normal). Reference (all 0) = board facing the camera, board x = image right, board y = image down.
  yaw   + : the board's RIGHT edge comes toward the camera      (yaw = atan2(n_x, n_z), n = board normal R[:,2], n_z > 0)
  pitch + : the board's TOP edge comes toward the camera        (pitch = asin(-n_y))
  roll  + : board turned clockwise in the image (x right -> y down), about the optical axis
Slot names in slots.py: turn = atan2(n_x,|n_z|) ~ yaw (same sign); lean = atan2(n_y,|n_z|) ~ -pitch (opposite sign).

  ~/scanner/StereoDepthSeg/venv_ffs/bin/python analyze3.py <set dir> <out dir>
"""
import argparse, json, math, os, sys, time
import numpy as np, cv2
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from scipy import stats

S3D = os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d")
sys.path.insert(0, os.path.join(S3D, "tools", "calcap"))
import calcap_calib as CC  # noqa: E402
import slots as SL  # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument("set"); ap.add_argument("out"); a = ap.parse_args()
t0 = time.time(); W, H = 1920, 1080; M = "k12_notan"
boards = SL.load_boards(os.path.join(S3D, "config", "charuco")); B5 = boards[0]
gK, gd = SL.load_lens(CC.DEFAULT)
views, caps = CC.load_views([a.set], boards, gK, gd, lambda s: None)
rms, K, d, rv, tv = CC.fit(views, M)
F0 = np.array([K[0, 0], K[1, 1], K[0, 2], K[1, 2]])
ids = sorted({v["cap"] for v in views}); byc = {c: [v for v in views if v["cap"] == c] for c in ids}
slot_of = {c: byc[c][0]["slot"] for c in ids}
motion = {c["id"]: c.get("motion_px") for c in caps}
def fitK(vs):
    r = CC.fit(vs, M); return np.array([r[1][0, 0], r[1][1, 1], r[1][0, 2], r[1][1, 2]]), r
def Rx(t): return cv2.Rodrigues(np.array([t, 0, 0.0]))[0]
def Ry(t): return cv2.Rodrigues(np.array([0, t, 0.0]))[0]
def Rz(t): return cv2.Rodrigues(np.array([0, 0, t]))[0]
def rpy(R):
    n = R[:, 2].copy()
    if n[2] < 0:
        n = -n
    yaw = math.atan2(n[0], n[2]); pitch = math.asin(max(-1, min(1, -n[1])))
    Rr = (Ry(yaw) @ Rx(pitch)).T @ R
    roll = math.atan2(Rr[1, 0], Rr[0, 0])
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
R = {"axes": __doc__.split("Axes")[1].split("  ~/")[0].strip()}

# ---------- pose per view (fitted lens), per slot
pv = []
for v, r_, t_ in zip(views, rv, tv):
    Rm, _ = cv2.Rodrigues(r_); ro, pi, ya = rpy(Rm)
    c = (Rm @ B5.centre + t_.ravel())
    n = Rm[:, 2] if Rm[2, 2] > 0 else -Rm[:, 2]
    pv.append(dict(file=v["file"], cap=v["cap"], slot=v["slot"], roll=ro, pitch=pi, yaw=ya, dist=float(np.linalg.norm(c)),
                   tilt=math.degrees(math.acos(abs(n[2]))), u=float(v["img"].reshape(-1, 2)[:, 0].mean()), v=float(v["img"].reshape(-1, 2)[:, 1].mean())))
# per-view error
err = {}
for v, r_, t_ in zip(views, rv, tv):
    p, _ = cv2.projectPoints(v["obj"], r_, t_, K, d)
    err[v["file"]] = float(np.sqrt(np.mean(np.sum((p - v["img"]).reshape(-1, 2) ** 2, -1))))
ps = {}
for c in ids:
    q = [x for x in pv if x["cap"] == c]
    ps[c] = dict(slot=slot_of[c], roll=float(np.mean([x["roll"] for x in q])), pitch=float(np.mean([x["pitch"] for x in q])),
                 yaw=float(np.mean([x["yaw"] for x in q])), dist=float(np.mean([x["dist"] for x in q])),
                 u=float(np.mean([x["u"] for x in q])), v=float(np.mean([x["v"] for x in q])),
                 motion_px=motion.get(c), rms=float(np.mean([err[x["file"]] for x in q])), session=c.split("#")[2])
edges = list(range(-60, 61, 10))
def hist(key):
    return np.histogram([x[key] for x in pv], bins=edges)[0].tolist()
rolls = np.array([x["roll"] for x in pv])
R["pose_distribution"] = dict(
    edges_deg=edges, pitch_views=hist("pitch"), yaw_views=hist("yaw"),
    roll_range=[float(rolls.min()), float(rolls.max())], roll_views_edges=[-180, -20, -10, -5, 0, 5, 10, 20, 180],
    roll_views=np.histogram(rolls, bins=[-180, -20, -10, -5, 0, 5, 10, 20, 180])[0].tolist(),
    sign_counts_slots={k: int(sum(1 for s in ps.values() if f(s))) for k, f in {
        "pitch>+15": lambda s: s["pitch"] > 15, "pitch<-15": lambda s: s["pitch"] < -15,
        "yaw>+15": lambda s: s["yaw"] > 15, "yaw<-15": lambda s: s["yaw"] < -15,
        "|pitch|,|yaw|<=15 (frontal)": lambda s: abs(s["pitch"]) <= 15 and abs(s["yaw"]) <= 15,
        "both |pitch|,|yaw|>15 (diagonal)": lambda s: abs(s["pitch"]) > 15 and abs(s["yaw"]) > 15,
        "pitch>+35": lambda s: s["pitch"] > 35, "pitch<-35": lambda s: s["pitch"] < -35,
        "yaw>+35": lambda s: s["yaw"] > 35, "yaw<-35": lambda s: s["yaw"] < -35}.items()},
    max_abs=dict(pitch_plus=float(max(x["pitch"] for x in pv)), pitch_minus=float(min(x["pitch"] for x in pv)),
                 yaw_plus=float(max(x["yaw"] for x in pv)), yaw_minus=float(min(x["yaw"] for x in pv))),
    per_slot=[dict(id=c, **ps[c]) for c in ids],
    appendix_combined_tilt_views=np.histogram([x["tilt"] for x in pv], bins=[0, 10, 20, 30, 40, 50, 60])[0].tolist())
# 2D scatter pitch-yaw, image positions
fig, ax = plt.subplots(1, 2, figsize=(11, 4.6))
for s in ps.values():
    ax[0].scatter(s["yaw"], s["pitch"], s=60 * s["dist"] ** -1, c="C0" if s["session"].startswith("20261026") else "C1")
    ax[0].annotate(s["slot"], (s["yaw"], s["pitch"]), fontsize=6)
    ax[1].scatter(s["u"], s["v"], c="C0" if s["session"].startswith("20261026") else "C1")
    ax[1].annotate(s["slot"], (s["u"], s["v"]), fontsize=6)
ax[0].set_xlabel("yaw deg (+ = right edge toward camera)"); ax[0].set_ylabel("pitch deg (+ = top edge toward camera)")
ax[0].axhline(0, c="k", lw=.5); ax[0].axvline(0, c="k", lw=.5); ax[0].set_xlim(-60, 60); ax[0].set_ylim(-60, 60); ax[0].set_aspect("equal")
ax[0].set_title("slot pose (blue 10-26 ISO 625, orange 10-25 ISO 1317)")
ax[1].set_xlim(0, W); ax[1].set_ylim(H, 0); ax[1].set_aspect("equal"); ax[1].set_title("board centre in image (px)")
fig.tight_layout(); fig.savefig(os.path.join(a.out, "pose_pitch_yaw.png"), dpi=110); plt.close(fig)

# ---------- leave-out by axis sign group
lopo = {}
for c in ids:
    k_, _ = fitK([v for v in views if v["cap"] != c]); lopo[c] = k_ - F0
n = len(ids)
jk_var_total = (n - 1) / n * np.sum([(lopo[c] - np.mean(list(lopo.values()), 0)) ** 2 for c in ids], 0)
groups = {"pitch+ (>15)": lambda s: s["pitch"] > 15, "pitch- (<-15)": lambda s: s["pitch"] < -15,
          "yaw+ (>15)": lambda s: s["yaw"] > 15, "yaw- (<-15)": lambda s: s["yaw"] < -15,
          "frontal (|p|,|y|<=15)": lambda s: abs(s["pitch"]) <= 15 and abs(s["yaw"]) <= 15}
gtab = {}
for g, f in groups.items():
    mem = [c for c in ids if f(ps[c])]
    k_, r_ = fitK([v for v in views if v["cap"] not in mem])
    mu = np.mean(list(lopo.values()), 0)
    share = ((n - 1) / n * np.sum([(lopo[c] - mu) ** 2 for c in mem], 0) / jk_var_total) if mem else np.zeros(4)
    gtab[g] = dict(n_slots=len(mem), slots=[slot_of[c] for c in mem],
                   remove_group=dict(zip(("dfx", "dfy", "dcx", "dcy"), (k_ - F0).tolist())),
                   jackknife_variance_share=dict(zip(("fx", "fy", "cx", "cy"), share.tolist())))
R["leave_out_by_axis"] = dict(note="remove_group = refit without those slots (new - full, px). jackknife_variance_share = part of the "
                                   "leave-one-slot-out variance coming from slots of that group (groups overlap for diagonal slots).",
                              jackknife_sd_total=dict(zip(("fx", "fy", "cx", "cy"), np.sqrt(jk_var_total).tolist())), groups=gtab,
                              per_slot_lopo=[dict(slot=slot_of[c], pitch=ps[c]["pitch"], yaw=ps[c]["yaw"], **dict(zip(("dfx", "dfy", "dcx", "dcy"), lopo[c].tolist()))) for c in ids])

# ---------- Fisher next-capture on roll / pitch / yaw
SIG = float(rms); dc5 = np.zeros(5); dc5[:2] = d[:2]
def view_info(obj, r_, t_):
    _, J = cv2.projectPoints(obj, r_, t_, K, dc5)
    Ji = J[:, 6:12]; Jp = J[:, :6]
    return (Ji.T @ Ji - Ji.T @ Jp @ np.linalg.solve(Jp.T @ Jp, Jp.T @ Ji)) / SIG ** 2
I0 = sum(view_info(v["obj"], r_, t_) for v, r_, t_ in zip(views, rv, tv))
boot = np.loadtxt(os.path.join(a.out, "bootstrap_fx_fy_cx_cy_k1_k2_rms.csv"), delimiter=",")[:, :4]
sd_f0 = np.sqrt(np.diag(np.linalg.inv(I0)))[:4]; infl = boot.std(0, ddof=1) / sd_f0
def score(I):
    s = np.sqrt(np.diag(np.linalg.inv(I)))[:4]; return s, s[0] / F0[0] * 100 / 0.3 + s[1] / F0[1] * 100 / 0.3 * 0 + s[2] / 3 + s[3] / 3
POS = {"C": (960, 540), "T": (960, 200), "B": (960, 880), "L": (330, 540), "R": (1590, 540), "TL": (330, 200), "TR": (1590, 200),
       "BL": (330, 880), "BR": (1590, 880), "B2": (960, 960), "BR2": (1660, 940), "TR2": (1660, 130)}
def ok(r_, t_):
    sq = B5.sq; o = np.array([[0, 0, 0], [B5.sx * sq, 0, 0], [B5.sx * sq, B5.sy * sq, 0], [0, B5.sy * sq, 0]], float)
    p, _ = cv2.projectPoints(o, r_, t_, K, dc5); p = p.reshape(-1, 2)
    return bool((p[:, 0] > 20).all() and (p[:, 0] < W - 20).all() and (p[:, 1] > 20).all() and (p[:, 1] < H - 20).all())
cands = []
for pn, (u, vv) in POS.items():
    ray = cv2.undistortPoints(np.array([[[u, vv]]], float), K, dc5).ravel(); ray = np.array([ray[0], ray[1], 1.0]); ray /= np.linalg.norm(ray)
    for dist in (0.45, 0.6, 0.8):
        for pi in (-55, -45, -30, -15, 0, 15, 30, 45, 55):
            for ya in (-55, -45, -30, -15, 0, 15, 30, 45, 55):
                for ro in (0, 30, -30):
                    Rm = Ry(math.radians(ya)) @ Rx(math.radians(pi)) @ Rz(math.radians(ro))
                    if math.degrees(math.acos(min(1, abs((Rm[:, 2])[2])))) > 60:
                        continue
                    t_ = (ray * dist - Rm @ B5.centre).reshape(3, 1); r_ = cv2.Rodrigues(Rm)[0]
                    if ok(r_, t_):
                        cands.append(dict(pos=pn, u=u, v=vv, dist=dist, roll=ro, pitch=pi, yaw=ya, I=view_info(B5.obj, r_, t_) * 4))
s_now, _ = score(I0); I = I0.copy(); picked = []
pool = list(cands)
for k in range(10):
    best = min(pool, key=lambda c: score(I + c["I"])[1]); I = I + best["I"]; s_, _ = score(I)
    picked.append(dict({q: best[q] for q in ("pos", "u", "v", "dist", "roll", "pitch", "yaw")}, stills=4,
                       pred_sd_fisher=s_.tolist(), pred_sd_inflated=(s_ * infl).tolist()))
    pool = [c for c in pool if c is not best]
def sid(p):
    def sg(x): return ("+" if x > 0 else "-") + str(abs(int(x)))
    return f"P{sg(p['pitch'])}_Y{sg(p['yaw'])}_R{sg(p['roll']) if p['roll'] else '0'}-{p['pos']}-{p['dist']:.2f}m"
R["next_capture_rpy"] = dict(
    method="Fisher information of the current fit (corner sigma %.3f px, pose eliminated), greedy over a roll x pitch x yaw x position x distance grid "
           "(roll 0/+-30, pitch/yaw -55..55, whole 5x3 board inside the frame, combined angle <= 60 deg), 4 stills each; score = sd_fx%%/0.3 + sd_cx/3 + sd_cy/3. "
           "inflated = Fisher x (bootstrap/Fisher now). Estimate; board flatness error is not reduced by more slots." % SIG,
    sd_now_bootstrap=boot.std(0, ddof=1).tolist(), inflation=infl.tolist(), n_candidates=len(cands),
    greedy=[dict(id=sid(p), **p) for p in picked],
    slot_format=[dict(id=sid(p), pos=p["pos"], kind="RPY", roll_deg=p["roll"], pitch_deg=p["pitch"], yaw_deg=p["yaw"],
                      turn_equiv_deg=p["yaw"], lean_equiv_deg=-p["pitch"], dist_m=p["dist"], target_px=[p["u"], p["v"]], stills=4,
                      label=f"제안 {i + 1}: {p['pos']} · pitch {p['pitch']:+d}° yaw {p['yaw']:+d}° roll {p['roll']:+d}° · {p['dist']} m") for i, p in enumerate(picked)])

# ---------- motion vs error, strict subset
mv = np.array([ps[c]["motion_px"] for c in ids], float); er = np.array([ps[c]["rms"] for c in ids])
# per-slot error with the pose shared by the 4 stills (one pose, 4x8 points) = direct measure of motion inside the slot
shared = []
for c in ids:
    o = np.concatenate([v["obj"] for v in byc[c]]); im = np.concatenate([v["img"] for v in byc[c]])
    okk, r_, t_ = cv2.solvePnP(o, im, K, d, flags=cv2.SOLVEPNP_ITERATIVE)
    p, _ = cv2.projectPoints(o, r_, t_, K, d); shared.append(float(np.sqrt(np.mean(np.sum((p - im).reshape(-1, 2) ** 2, -1)))))
shared = np.array(shared)
rho1 = stats.spearmanr(mv, er); rho2 = stats.spearmanr(mv, shared)
fig, ax = plt.subplots(1, 2, figsize=(10, 4))
ax[0].scatter(mv, er); ax[0].set_xscale("log"); ax[0].set_xlabel("capture motion px (captures.json)"); ax[0].set_ylabel("slot mean reproj RMS px (own pose)")
ax[0].set_title(f"Spearman {rho1.correlation:.2f} (p {rho1.pvalue:.2f})")
ax[1].scatter(mv, shared, c="C1"); ax[1].set_xscale("log"); ax[1].set_yscale("log"); ax[1].set_xlabel("capture motion px")
ax[1].set_ylabel("RMS px, one pose for the 4 stills"); ax[1].set_title(f"Spearman {rho2.correlation:.2f} (p {rho2.pvalue:.3f})")
for x_, y_, c in zip(mv, er, ids): ax[0].annotate(slot_of[c], (x_, y_), fontsize=5)
fig.tight_layout(); fig.savefig(os.path.join(a.out, "motion_vs_error.png"), dpi=110); plt.close(fig)
strict = [c for c in ids if ps[c]["motion_px"] is not None and ps[c]["motion_px"] < 0.1]
loose = [c for c in ids if c not in strict]
def boot_sd(sub, nb=200, seed=5):
    rg = np.random.default_rng(seed); out = []
    for _ in range(nb):
        out.append(fitK([v for c in rg.choice(sub, len(sub), replace=True) for v in byc[c]])[0])
    return np.array(out).std(0, ddof=1)
def heldout(Kc, dc_, test):
    e = []
    for c in test:
        for i, v in enumerate(byc[c]):
            okk, r_, t_ = cv2.solvePnP(v["obj"], v["img"], Kc, dc_, flags=cv2.SOLVEPNP_ITERATIVE)
            p, _ = cv2.projectPoints(v["obj"], r_, t_, Kc, dc_); e.append(float(np.sqrt(np.mean(np.sum((p - v["img"]).reshape(-1, 2) ** 2, -1)))))
    return float(np.median(e))
kS, rS = fitK([v for c in strict for v in byc[c]])
# same-size random subsets (any motion) -> is the strict set better than a random set of the same number of slots?
rg = np.random.default_rng(3); rnd = []
for _ in range(200):
    sub = list(rg.choice(ids, len(strict), replace=False)); k_, r_ = fitK([v for c in sub for v in byc[c]])
    test = [c for c in ids if c not in sub]
    rnd.append(dict(k=(k_ - F0).tolist(), held=heldout(r_[1], r_[2], test), n_strict_in=len(set(sub) & set(strict))))
rnd_dev = np.array([x["k"] for x in rnd]); rnd_held = np.array([x["held"] for x in rnd])
strict_held = heldout(rS[1], rS[2], loose)
# also: full set minus the slots with motion >= 1 px (moderate rule)
mod = [c for c in ids if ps[c]["motion_px"] is not None and ps[c]["motion_px"] < 1.0]
kM, rM = fitK([v for c in mod for v in byc[c]])
R["motion"] = dict(
    per_slot=[dict(slot=slot_of[c], motion_px=ps[c]["motion_px"], rms_own_pose=ps[c]["rms"], rms_shared_pose=float(s)) for c, s in zip(ids, shared)],
    spearman_motion_vs_rms_own_pose=[float(rho1.correlation), float(rho1.pvalue)],
    spearman_motion_vs_rms_shared_pose=[float(rho2.correlation), float(rho2.pvalue)],
    strict_lt_0p1=dict(n_slots=len(strict), slots=[slot_of[c] for c in strict],
                       fit=dict(zip(("fx", "fy", "cx", "cy"), kS.tolist())), minus_full=dict(zip(("dfx", "dfy", "dcx", "dcy"), (kS - F0).tolist())),
                       rms=float(rS[0]), bootstrap_sd=dict(zip(("fx", "fy", "cx", "cy"), boot_sd(strict).tolist())),
                       heldout_median_on_other_slots=strict_held,
                       pose_of_strict_slots=[dict(slot=slot_of[c], pitch=ps[c]["pitch"], yaw=ps[c]["yaw"], u=ps[c]["u"], v=ps[c]["v"]) for c in strict]),
    random_same_size=dict(n=200, dev_sd=dict(zip(("fx", "fy", "cx", "cy"), rnd_dev.std(0, ddof=1).tolist())),
                          dev_abs_median=dict(zip(("fx", "fy", "cx", "cy"), np.median(np.abs(rnd_dev), 0).tolist())),
                          heldout_median_p50=float(np.median(rnd_held)), heldout_p10_p90=[float(np.quantile(rnd_held, .1)), float(np.quantile(rnd_held, .9))],
                          strict_heldout_percentile=float((rnd_held < strict_held).mean() * 100)),
    moderate_lt_1px=dict(n_slots=len(mod), fit=dict(zip(("fx", "fy", "cx", "cy"), kM.tolist())), minus_full=dict(zip(("dfx", "dfy", "dcx", "dcy"), (kM - F0).tolist())),
                         rms=float(rM[0]), bootstrap_sd=dict(zip(("fx", "fy", "cx", "cy"), boot_sd(mod).tolist()))),
    full=dict(fit=dict(zip(("fx", "fy", "cx", "cy"), F0.tolist())), rms=float(rms), bootstrap_sd=dict(zip(("fx", "fy", "cx", "cy"), boot.std(0, ddof=1).tolist()))),
    note="motion_px = calcap server's spread of the board over the 4 stills (captures.json). heldout = median per-still RMS on the slots NOT used in that fit "
         "(pose by PnP on the still, lens fixed); strict set held out on the 24 looser slots, random sets on their own complements (different test sets - indicative).")
R["made"] = time.strftime("%Y-%m-%d %H:%M:%S"); R["set"] = a.set; R["sec"] = round(time.time() - t0, 1)
json.dump(R, open(os.path.join(a.out, "analysis3.json"), "w"), indent=1, default=float)
P = R["pose_distribution"]; print(json.dumps({k: P[k] for k in P if k != "per_slot"}, default=float))
for s in P["per_slot"]: print("  %-16s roll %7.1f pitch %6.1f yaw %6.1f d %.2f u %5.0f v %5.0f mot %s rms %.3f" % (s["slot"], s["roll"], s["pitch"], s["yaw"], s["dist"], s["u"], s["v"], s["motion_px"], s["rms"]))
L = R["leave_out_by_axis"]; print(json.dumps(L["jackknife_sd_total"]))
for g, x in L["groups"].items(): print(g, x["n_slots"], {k: round(v, 2) for k, v in x["remove_group"].items()}, {k: round(v, 2) for k, v in x["jackknife_variance_share"].items()})
N = R["next_capture_rpy"]; print("now", N["sd_now_bootstrap"], "infl", N["inflation"], "ncand", N["n_candidates"])
for p in N["greedy"]: print("  ", p["id"], [round(x, 2) for x in p["pred_sd_inflated"]])
Mo = R["motion"]; print(json.dumps({k: Mo[k] for k in Mo if k != "per_slot"}, default=float, indent=0))
print("sec", R["sec"])
