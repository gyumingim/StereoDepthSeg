#!/usr/bin/env python3
"""Scoring for docs/zupt_onlybegin_plan.md (3 criteria). Same scoring for OB (stock zbase + zupt_only_at_beginning: true)
and, report-only, A2. Base = runs/zupt_fixA/*_base.

Per run (GT camera centre c = p_bc, ok & |p_bc| < 1.2):
  window     = first tracked pose .. min(last valid GT sample, last pose)
  divergent  = some 5 s window relative error > 30 cm or window ATE > 50 cm (SE3 align in window; zupt_aa.metrics rules)
  still      = GT-still segments: 0.1 s grid points in the window with GT speed (+-0.25 s) < 0.05 cm/s, merged, >= 1 s
  drift      = sum over still segments of |p_vio(end) - p_vio(start)| / total still seconds  (cm/s, VIO frame)
Recording value = median over its replicates (o0-o2; T cells: d0/dm025/dp025 of one segment).

  ../venv_ffs/bin/python imu_learn/zupt_ob_score.py aa      # A/A on base only -> runs/zupt_onlybegin/aa_drift.json
  ../venv_ffs/bin/python imu_learn/zupt_ob_score.py score   # OB and A2 vs base -> runs/zupt_onlybegin/score.json
"""
import gzip, itertools, json, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import zupt_fixA as zf             # noqa: E402  (cell list; chdir to scan3d)
import zupt_fixA_analyse as an     # noqa: E402
import zupt_aa as aa               # noqa: E402

R = an.SCAN3D / "runs"
OUT = R / "zupt_onlybegin"
CONDS = {"base": R / "zupt_fixA" / "{}_base", "OB": R / "zupt_onlybegin" / "{}_OB", "A2": R / "zupt_fixA2" / "{}_A2"}
TARGETS = ["BR_133820_o0", "BR_133820_o1", "BR_133820_o2", "BR_134550_o2", "Z_F_222747_o0", "Z_F_222747_o1", "Z_F_222747_o2"]
EXCLUDE_REC = "hs_20261021_213955"   # phone drop (input-event detector)
MARGIN_FILE = OUT / "aa_drift.json"


def run_metrics(pre, rec, pos="cam"):
    """pos='cam' (default since 2026-10-23, docs/imucal_result.md): VIO position = camera centre (T_world_cam0 translation),
    the same point the GT p_bc describes. pos='imu' = the old ruler ('p' = p_IinG, the IMU position): it is offset from the
    camera by the online camera-IMU lever (several cm, rotating with the phone) and inflated window ATE (base o0 median
    2.16 vs 0.90 cm). Score files made before 2026-10-23 used 'imu'."""
    if pos not in ("cam", "imu"):
        raise ValueError(pos)
    j = json.load(open(str(pre) + ".json"))
    if j.get("ate_cm") is None:
        return dict(result=False, divergent=True, why="no result")
    P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
    if pos == "cam":
        T = [p for p in P if p.get("status") == "tracking" and p.get("T_world_cam0")]
        p = np.array([np.asarray(m["T_world_cam0"], float).reshape(4, 4)[:3, 3] for m in T]).reshape(-1, 3)
    else:
        T = [p for p in P if p.get("status") == "tracking" and p.get("p")]
        p = np.array([p["p"] for p in T])
    t = np.array([m["t"] for m in T])
    g = np.load(an.SCAN3D / "recordings" / rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    off = P[0]["t_ns"] - P[0]["t"] * 1e9 + j["align_dt_ms"] * 1e6
    tg = (g["t_ns"][ok] - off) / 1e9
    cg = g["p_bc"][ok].reshape(-1, 3)
    o = np.argsort(tg); tg, cg = tg[o], cg[o]
    w0, w1 = t[0], min(tg[-1], t[-1])
    # --- divergence (same as zupt_aa.metrics, window end = last GT sample)
    m = (t >= w0) & (t <= w1)
    tw, pw = t[m], p[m]
    k = np.clip(np.searchsorted(tg, tw), 1, len(tg) - 1)
    k = np.where(np.abs(tg[k - 1] - tw) < np.abs(tg[k] - tw), k - 1, k)
    good = np.abs(tg[k] - tw) <= 0.002
    if good.sum() < 50:
        return dict(result=False, divergent=None, why="few GT matches")
    Rm, tr = aa.umeyama_se3(pw[good], cg[k[good]])
    pa = (Rm @ pw.T).T + tr
    ate = float(np.sqrt(np.mean(np.sum((pa[good] - cg[k[good]]) ** 2, 1)))) * 100

    def gtok(x):
        i = np.clip(np.searchsorted(tg, x), 1, len(tg) - 1)
        return np.minimum(np.abs(tg[i] - x), np.abs(tg[i - 1] - x)) <= 0.02

    def ia(x, T_, X):
        return np.array([np.interp(x, T_, X[:, q]) for q in range(3)]).T
    grid = np.arange(w0, w1 - 5, 0.1)
    rel = np.array([])
    if len(grid):
        gg = grid[gtok(grid) & gtok(grid + 5)]
        if len(gg):
            rel = np.linalg.norm((ia(gg + 5, tw, pa) - ia(gg, tw, pa)) - (ia(gg + 5, tg, cg) - ia(gg, tg, cg)), axis=1) * 100
    rel_max = float(rel.max()) if len(rel) else None
    div = bool(ate > 50 or (rel_max is not None and rel_max > 30))
    # --- true-still drift
    sg = np.arange(w0, w1 - 0.25, 0.1)
    st = gtok(sg - 0.25) & gtok(sg + 0.25)
    sp = np.full(len(sg), np.inf)
    if st.any():
        sp[st] = np.linalg.norm(ia(sg[st] + 0.25, tg, cg) - ia(sg[st] - 0.25, tg, cg), axis=1) / 0.5 * 100
    still = sp < 0.05
    segs, a = [], None
    for x, s in zip(sg, still):
        if s and a is None:
            a = x
        if not s and a is not None:
            segs.append((a, x - 0.1)); a = None
    if a is not None:
        segs.append((a, sg[-1]))
    segs = [(a, b) for a, b in segs if b - a >= 1.0]
    dur = float(sum(b - a for a, b in segs))
    dr = float(sum(np.linalg.norm(ia(np.array([b]), t, p)[0] - ia(np.array([a]), t, p)[0]) for a, b in segs)) * 100
    return dict(result=True, window=[float(w0), float(w1)], ate_w=ate, rel5_max=rel_max, divergent=div,
                still_s=dur, still_drift_cm=dr, drift_rate=(dr / dur) if dur > 0 else None, n_still_segs=len(segs))


def unit_of(c):
    k = c["key"]
    if c["sub"] == "T":
        return k.rsplit("_", 1)[0], ["d0", "dm025", "dp025"].index(k.rsplit("_", 1)[1])
    return k.rsplit("_", 1)[0], int(k[-1])


def collect(cond):
    out = {}
    for c in zf.cells():
        pre = Path(str(CONDS[cond]).format(c["key"]))
        if not Path(str(pre) + ".json").exists():
            out[c["key"]] = None
            continue
        out[c["key"]] = run_metrics(pre, c["rec"])
        out[c["key"]].update(rec=c["rec"], sub=c["sub"])
    return out


def aa_drift():
    OUT.mkdir(exist_ok=True)
    b = collect("base")
    units = {}
    for c in zf.cells():
        u, i = unit_of(c)
        units.setdefault(u, {})[i] = b[c["key"]]
    diffs, ratios = [], []
    for u, reps in units.items():
        for i, j in itertools.permutations(sorted(reps), 2):
            x, y = reps[i], reps[j]
            if not (x and y and x.get("drift_rate") is not None and y.get("drift_rate") is not None):
                continue
            if x["still_s"] < 3 or y["still_s"] < 3:
                continue
            diffs.append(dict(unit=u, cand=i, base=j, cand_rate=x["drift_rate"], base_rate=y["drift_rate"],
                              diff=x["drift_rate"] - y["drift_rate"],
                              ratio=x["drift_rate"] / y["drift_rate"] if y["drift_rate"] > 0 else None))
    d = np.array([x["diff"] for x in diffs])
    res = dict(made=time.strftime("%F %T"), note="A/A of true-still drift rate on base runs (o0-o2), for the margin",
               n_pairs=len(diffs), diff_p50=float(np.median(d)), diff_p90=float(np.percentile(d, 90)),
               diff_p95=float(np.percentile(d, 95)), diff_max=float(d.max()),
               rate_median=float(np.median([x["base_rate"] for x in diffs])), pairs=diffs,
               per_run={k: v for k, v in b.items()})
    json.dump(res, open(MARGIN_FILE, "w"), indent=1, default=float)
    print({k: v for k, v in res.items() if k not in ("pairs", "per_run")})
    for x in sorted(diffs, key=lambda z: -z["diff"])[:10]:
        print(x)
    return res


def score():
    OUT.mkdir(exist_ok=True)
    plan_margin = PLAN_MARGIN
    data = {k: collect(k) for k in CONDS}
    res = dict(made=time.strftime("%F %T"), plan="docs/zupt_onlybegin_plan.md", drift_rule="cand <= k x base + m", k_m=plan_margin, conds={})
    for cond in ("OB", "A2"):
        D, B = data[cond], data["base"]
        c1 = {c: (None if D.get(c) is None else D[c]["divergent"]) for c in TARGETS}
        c1_pass = all(v is False for v in c1.values())
        groups = {}
        for c in zf.cells():
            if c["rec"] == EXCLUDE_REC:
                continue
            x, y = D.get(c["key"]), B.get(c["key"])
            if x is None or y is None:
                continue
            gr = groups.setdefault(c["sub"], dict(cand=0, base=0, cand_runs=[], base_runs=[]))
            if x["divergent"]:
                gr["cand"] += 1; gr["cand_runs"].append(c["key"])
            if y["divergent"]:
                gr["base"] += 1; gr["base_runs"].append(c["key"])
        c2_pass = all(g["cand"] <= g["base"] for g in groups.values())
        units = {}
        for c in zf.cells():
            u, _ = unit_of(c)
            x, y = D.get(c["key"]), B.get(c["key"])
            units.setdefault(u, dict(cand=[], base=[]))
            if x and x.get("drift_rate") is not None and x["still_s"] >= 3:
                units[u]["cand"].append(x["drift_rate"])
            if y and y.get("drift_rate") is not None and y["still_s"] >= 3:
                units[u]["base"].append(y["drift_rate"])
        c3 = []
        for u, v in units.items():
            if not v["cand"] or not v["base"]:
                continue
            mc, mb = float(np.median(v["cand"])), float(np.median(v["base"]))
            c3.append(dict(unit=u, cand=mc, base=mb, limit=mb * plan_margin[0] + plan_margin[1], pass_=mc <= mb * plan_margin[0] + plan_margin[1]))
        c3_pass = all(x["pass_"] for x in c3)
        res["conds"][cond] = dict(c1=dict(items=c1, pass_=c1_pass), c2=dict(groups=groups, pass_=c2_pass),
                                  c3=dict(items=c3, pass_=c3_pass), all_pass=c1_pass and c2_pass and c3_pass,
                                  n_divergent_total=sum(g["cand"] for g in groups.values()),
                                  n_divergent_base=sum(g["base"] for g in groups.values()),
                                  drift_median_of_units=float(np.median([x["cand"] for x in c3])) if c3 else None,
                                  drift_median_of_units_base=float(np.median([x["base"] for x in c3])) if c3 else None)
    res["runs"] = data
    json.dump(res, open(OUT / "score.json", "w"), indent=1, default=float)
    for cond, r in res["conds"].items():
        print(cond, "C1", r["c1"]["pass_"], r["c1"]["items"])
        print(cond, "C2", r["c2"]["pass_"], {g: (v["cand"], v["base"]) for g, v in r["c2"]["groups"].items()})
        print(cond, "C3", r["c3"]["pass_"], [(x["unit"], round(x["cand"], 4), round(x["base"], 4)) for x in r["c3"]["items"] if not x["pass_"]])
        print(cond, "ALL", r["all_pass"], "div", r["n_divergent_total"], "vs base", r["n_divergent_base"],
              "drift med", r["drift_median_of_units"], "base", r["drift_median_of_units_base"])
    return res


PLAN_MARGIN = (2.5, 0.02)   # cand <= 2.5 x base + 0.02 cm/s; from the A/A (aa_drift.json), docs/zupt_onlybegin_plan.md

if __name__ == "__main__":
    if sys.argv[1:] == ["aa"]:
        aa_drift()
    elif sys.argv[1:] == ["score"]:
        if PLAN_MARGIN is None:
            sys.exit("PLAN_MARGIN not set")
        score()
