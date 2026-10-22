"""Tables for docs/lens_calib_round2_20261022.md + repeatability verdict + config/lens JSON.
usage: summarize.py [--write-config]"""
import json, sys, time, itertools
import numpy as np
from common import D, S3, FOCUS, ROUND, REC2F

R = lambda n: json.load(open(D / "results" / n))
FX_TOL, PP_TOL = 0.3, 2.0
FD = {"0.37": 0.37, "1.00": 1.00, "2.22": 2.22, "3.33": 3.33}
POSE_STARVED = {"173113": "33 frontal views (tilt <5 deg), 8 one-second blocks: fx sd 10 px - not a calibration"}
out_lines = []
P = lambda s="": out_lines.append(s)

# ---------- 1. counts / coverage per recording and per focus
P("### counts")
P("| focus | rec | round | frames | still <0.05 | detected >=12 (every 2nd still) | mirror dup markers dropped | stand | blurry | usable (0.1 s thin) | selected | 32x18 cover usable / selected | tilt p50 / p90 / max (fit) | PnP dist p50 m |")
P("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for f in ("0.37", "1.00", "2.22", "3.33"):
    for r in FOCUS[f]:
        j = R(f"cal_{r}.json"); c = j["counts"][r]; fr = j["fits"]["rational"]
        P(f"| {f} | {r} | {ROUND[r]} | {c['n_frames']} | {c['n_still']} | {c['n_detected_ge12']} | {c['mirror_dup_markers_dropped']} | "
          f"{c['n_stand']} | {c['n_blurry']} | {c['n_after_thin']} | {fr['n_views']} | {j['cov32x18_usable']:.2f} / {j['cov32x18_selected']:.2f} | "
          f"{fr['tilt_deg_pct'][1]:.0f} / {fr['tilt_deg_pct'][2]:.0f} / {fr['tilt_deg_pct'][3]:.0f} | {fr['dist_m_pct'][1]:.2f} |")
    j = R(f"cal_{f}.json"); fr = j["fits"]["rational"]
    P(f"| **{f} pooled** | {'+'.join(FOCUS[f])} | | | | | | | | {j['n_usable']} | {fr['n_views']} | {j['cov32x18_usable']:.2f} / {j['cov32x18_selected']:.2f} | "
      f"{fr['tilt_deg_pct'][1]:.0f} / {fr['tilt_deg_pct'][2]:.0f} / {fr['tilt_deg_pct'][3]:.0f} | {fr['dist_m_pct'][1]:.2f} |")
P()
P("### tilt bins of selected views (rough tilt from nominal-K PnP; direction = far edge in image)")
for f in ("0.37", "1.00", "2.22", "3.33"):
    for r in FOCUS[f] + [f]:
        j = R(f"cal_{r}.json"); t = j["tilt_selected"]
        agg = {}
        for k, v in t.items():
            b = k.split(":")[1]; agg[b] = agg.get(b, 0) + v
        dirs = sorted({k.split(":")[0] for k in t if not k.startswith("front")})
        P(f"- {r}: " + ", ".join(f"{b} {agg.get(b, 0)}" for b in ("0-15", "15-30", "30-45", ">=45")) + f"; directions {len(dirs)}/4 ({' '.join(dirs)})")
P()

# ---------- 2. fits
P("### per recording / pooled fits (block bootstrap sd, B=100)")
P("| focus | unit | model | fx | fy | cx | cy | rms px | holdout px | sd fx / cx / cy |")
P("|---|---|---|---|---|---|---|---|---|---|")
fits = {}
for f in ("0.37", "1.00", "2.22", "3.33"):
    for r in FOCUS[f] + [f]:
        j = R(f"cal_{r}.json")
        for m in ("rational", "k1k2p_k3"):
            fr = j["fits"][m]; sd = j["boot"][m]["sd"]
            fits[(r, m)] = (np.array([fr["fx"], fr["fy"], fr["cx"], fr["cy"]]), np.array(sd))
            P(f"| {f} | {r}{' (pose-starved)' if r in POSE_STARVED else ''} | {m} | {fr['fx']:.1f} | {fr['fy']:.1f} | {fr['cx']:.1f} | {fr['cy']:.1f} | "
              f"{fr['rms_px']:.3f} | {j['holdout_px'][m]:.3f} | {sd[0]:.1f} / {sd[2]:.1f} / {sd[3]:.1f} |")
P()


# ---------- 3. repeatability
def pairs_table(get, label):
    P(f"### same-focus repeatability: {label}")
    P("| focus | A vs B | dfx % (sigma) | dfy % | dcx px (sigma) | dcy px (sigma) | fx <0.3 % | pp <2 px |")
    P("|---|---|---|---|---|---|---|---|")
    verdict = {}
    for f in ("0.37", "1.00", "2.22", "3.33"):
        rs = [r for r in FOCUS[f] if r not in POSE_STARVED]
        ok_f = True
        for ra, rb in itertools.combinations(rs, 2):
            (va, sa), (vb, sb) = get(ra), get(rb)
            d = vb - va; s = np.hypot(sa, sb)
            dfx = 100 * d[0] / va[0]; dfy = 100 * d[1] / va[1]
            okfx = abs(dfx) < FX_TOL; okpp = abs(d[2]) < PP_TOL and abs(d[3]) < PP_TOL
            ok_f &= okfx and okpp
            P(f"| {f} | {ra} vs {rb} | {dfx:+.2f} ({100 * s[0] / va[0]:.2f}) | {dfy:+.2f} | {d[2]:+.1f} ({s[2]:.1f}) | {d[3]:+.1f} ({s[3]:.1f}) | "
              f"{'yes' if okfx else 'NO'} | {'yes' if okpp else 'NO'} |")
        verdict[f] = bool(ok_f)
    P()
    return verdict


V = {}
for m in ("rational", "k1k2p_k3"):
    V[f"own_{m}"] = pairs_table(lambda r, m=m: fits[(r, m)], f"per-recording fits, own distortion ({m})")
for name in ("joint_rec_k5", "joint_rec_k5_k1g"):
    j = R(f"{name}.json")
    get = lambda r, j=j: (np.array(j["K4"][r][:4]), np.array(j["boot"]["sd"][r][:4]))
    V[name] = pairs_table(get, f"{name} (shared distortion{', k1 per recording' if 'k1g' in name else ''}, B={len(j['boot']['K4'])})")
    if "k1g" in name:
        P("k1 per recording: " + ", ".join(f"{r} {j['K4'][r][4]:.4f}" for r in j["group_names"]))
        P()

# ---------- sensitivity
P("### selection sensitivity (rational fx / cx / cy): base 160 spread | tilt<45 | w<0.02 | all usable | corner outliers >4x median dropped")
for f in ("0.37", "1.00", "2.22", "3.33"):
    for r in FOCUS[f] + [f]:
        cells = []
        for tag in ("", "_t45", "_w02", "_all", "_cr"):
            try:
                fr = R(f"cal_{r}{tag}.json")["fits"]["rational"]
                cells.append(f"{fr['fx']:.1f}/{fr['cx']:.1f}/{fr['cy']:.1f} (n{fr['n_views']})")
            except FileNotFoundError:
                cells.append("-")
        P(f"- {r}: " + " | ".join(cells))
P()

# ---------- 4. joint across focus vs scale law
P("### joint across focus (shared distortion) vs sweep scale law (+0.80 %/dpt; sweep nodes 0.37 -1.370 %, 1.00 -0.969 %, 3.33 +0.900 % vs 2.22)")
SWEEP = {"0.37": -1.370, "1.00": -0.969, "2.22": 0.0, "3.33": 0.900}
law = {}
for name in ("joint_focus_k5", "joint_focus_k5_k1g", "joint_focus_rational"):
    j = R(f"{name}.json")
    K = {g: np.array(j["K4"][g]) for g in j["group_names"]}
    Ds = np.array([FD[g] for g in ("0.37", "1.00", "2.22", "3.33")])
    fxs = np.array([K[g][0] for g in ("0.37", "1.00", "2.22", "3.33")])
    slope = np.polyfit(Ds, 100 * np.log(fxs), 1)[0]
    row = {}
    if "boot" in j:
        Kb = np.array(j["boot"]["K4"])   # (B, G, NK)
        gi = {g: k for k, g in enumerate(j["group_names"])}
        rel_b = {g: 100 * (Kb[:, gi[g], 0] / Kb[:, gi["2.22"], 0] - 1) for g in FD}
        sl_b = np.array([np.polyfit(Ds, 100 * np.log([Kb[b, gi[g], 0] for g in ("0.37", "1.00", "2.22", "3.33")]), 1)[0]
                         for b in range(len(Kb))])
        sci = np.percentile(sl_b, [2.5, 97.5])
    P(f"- **{name}** (rms {j['rms_px']:.3f} px, views {j['n_views']}): slope {slope:.2f} %/dpt" +
      (f" [95% {sci[0]:.2f}, {sci[1]:.2f}]" if "boot" in j else " (no bootstrap)"))
    P("  | focus | fx | fy | cx | cy | fx vs 2.22 % (95% CI) | sweep % | law 0.8 %/dpt % |")
    P("  |---|---|---|---|---|---|---|---|")
    for g in ("0.37", "1.00", "2.22", "3.33"):
        rel = 100 * (K[g][0] / K["2.22"][0] - 1)
        ci = f" [{np.percentile(rel_b[g], 2.5):+.2f}, {np.percentile(rel_b[g], 97.5):+.2f}]" if "boot" in j and g != "2.22" else ""
        P(f"  | {g} | {K[g][0]:.1f} | {K[g][1]:.1f} | {K[g][2]:.1f} | {K[g][3]:.1f} | {rel:+.2f}{ci} | {SWEEP[g]:+.2f} | {0.8 * (FD[g] - 2.22):+.2f} |")
    law[name] = dict(slope_pct_per_dpt=float(slope), slope_ci95=[float(x) for x in sci] if "boot" in j else None)
    if "k1g" in name:
        P("  k1 per focus: " + ", ".join(f"{g} {K[g][4]:.4f}" for g in j["group_names"]))
P()

# ---------- verdict
P("### verdict per focus (all same-focus pairs fx <0.3 % and |dcx|,|dcy| <2 px)")
for k, v in V.items():
    P(f"- {k}: " + ", ".join(f"{f} {'PASS' if ok else 'fail'}" for f, ok in v.items()))
allpass = {k: all(v.values()) for k, v in V.items()}
P(f"- all focus pass: {allpass}")
txt = "\n".join(out_lines)
open(D / "results" / "summary_tables.md", "w").write(txt + "\n")
print(txt)

# ---------- config JSON
if "--write-config" in sys.argv:
    jf = R("joint_focus_k5_k1g.json")
    gi = {g: k for k, g in enumerate(jf["group_names"])}
    Kb = np.array(jf["boot"]["K4"])
    main = V["joint_rec_k5_k1g"]
    status = "ok" if all(main.values()) else "not_repeatable"
    entries = []
    for g in ("0.37", "1.00", "2.22", "3.33"):
        k4 = jf["K4"][g]; ci = np.percentile(Kb[:, gi[g]], [2.5, 97.5], axis=0)
        dist = list(jf["dist"]); dist[0] = k4[4]
        pooled = R(f"cal_{g}.json")
        entries.append(dict(
            focus_dpt=FD[g], repeatable=bool(main[g]),
            K=[[k4[0], 0, k4[2]], [0, k4[1], k4[3]], [0, 0, 1]], dist_k1k2p1p2k3=dist,
            ci95_fx_fy_cx_cy=[[float(a), float(b)] for a, b in zip(ci[0][:4], ci[1][:4])],
            ci95_k1=[float(ci[0][4]), float(ci[1][4])],
            ci_note="1 s block bootstrap within recordings (B=%d); same-focus recording differences can exceed it (see repeatability)" % len(Kb),
            recs=[r for r in FOCUS[g] if r not in jf["excluded"]],
            pooled_own_distortion={m: dict(K=pooled["fits"][m]["K"], dist=pooled["fits"][m]["dist"], rms_px=pooled["fits"][m]["rms_px"],
                                           ci95_fx_fy_cx_cy=np.array(pooled["boot"][m]["ci95"]).T.tolist())
                                   for m in ("rational", "k1k2p_k3")}))
    why = []
    for f, ok in main.items():
        if not ok:
            why.append(f)
    cfg = dict(
        status=status,
        status_reason=("targets fx <0.3 %, principal point <2 px between same-focus recordings: met at " +
                       ", ".join(f for f, ok in main.items() if ok) + "; NOT met at " + ", ".join(why) +
                       " (see docs/lens_calib_round2_20261022.md). Do not use the not-repeatable nodes as absolute K."),
        size=[1920, 1080], camera="S24 main (cam5), 240 fps, app hs-20261022-afreg",
        model="OpenCV k1 k2 p1 p2 k3; k2 p1 p2 k3 shared across focus, k1 and fx fy cx cy per focus (joint fit)",
        board="config/charuco/board_monitor_6x6.json (square 63.294 mm)",
        rms_px=jf["rms_px"], n_views=jf["n_views"], excluded_recs=jf["excluded"],
        scale_law_check=law, focus=entries,
        made=time.strftime("%Y-%m-%d %H:%M:%S"), source="runs/lens_calib_round2_20261022 (summarize.py --write-config)",
        doc="docs/lens_calib_round2_20261022.md")
    p = S3 / "config/lens/hs_1920x1080_focus_cal_20261022.json"
    json.dump(cfg, open(p, "w"), indent=1)
    print("->", p, status)
