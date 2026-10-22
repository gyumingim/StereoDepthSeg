#!/usr/bin/env python3
"""A/A false-alarm rates (runs/zupt_fix_review/judge.md 5 step 4), PRELIMINARY: existing base runs, 3 replicates only.

Revised criteria 2-3 (judge.md 4 items 2, 5, 6, 8, 9), scored base-vs-base:
  unit        = recording (board / i3 / F / O cells: offsets o0-o2) or recording+segment (T cells: d0/dm025/dp025)
  window      = first tracked pose .. min(last valid GT sample, last pose - 3 s)              (item 2)
  GT          = camera centre c = p_bc (fixed), ok & |p_bc| < 1.2, nearest within 2 ms
  align       = SE3 Umeyama of tracked positions to GT centre inside the window
  ate_w       = RMS position error after that alignment (cm)
  rel5        = |(p(t+5)-p(t)) - (c(t+5)-c(t))| on a 0.1 s grid inside the window (aligned, cm); p95 and max
  divergent   = max rel5 > 30 cm or ate_w > 50 cm                                                  (item 6)
  bundle      = "impact" if imu_learn/input_events.detect() flags the recording (thresholds unchanged), else sub group
Pseudo-tests: for every ordered pair (i, j), i != j, replicate i plays "candidate" and j plays "base" in every unit
(6 pseudo-tests). With one replicate per side the offset median is that run and "majority" is that run.
  C2  per bundle: n divergent(cand) <= n divergent(base), and no unit with base not divergent but cand divergent
  C3  per unit: rel5 p95 <= base x 1.1 + 0.5 cm and ate_w <= base x 1.2 + 0.5 cm; true-still ZUPT >= 0.9 x base
      (true-still counts from runs/zupt_gtfix_report/report.json, base >= 30 only)
False-alarm rate = share of pseudo-tests that fail (whole test) and share of unit comparisons that fail.

  ../venv_ffs/bin/python imu_learn/zupt_aa.py   # -> runs/zupt_next_prep/aa.json, aa.out
"""
import gzip, itertools, json, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import zupt_fixA as zf             # noqa: E402  (cell list; chdir to scan3d)
import zupt_fixA_analyse as an     # noqa: E402

R = an.SCAN3D / "runs"
OUT = R / "zupt_next_prep"
BASE = R / "zupt_fixA"


def umeyama_se3(X, Y):
    mx, my = X.mean(0), Y.mean(0)
    U, _, Vt = np.linalg.svd((Y - my).T @ (X - mx))
    S = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        S[2, 2] = -1
    Rm = U @ S @ Vt
    return Rm, my - Rm @ mx


def metrics(name, rec, pre=None):
    pre = pre or BASE / f"{name}_base"
    j = json.load(open(str(pre) + ".json"))
    if j.get("ate_cm") is None:
        return dict(result=False)
    P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
    T = [p for p in P if p.get("status") == "tracking" and p.get("p")]
    t = np.array([p["t"] for p in T]); p = np.array([p["p"] for p in T])
    g = np.load(an.SCAN3D / "recordings" / rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    off = P[0]["t_ns"] - P[0]["t"] * 1e9 + j["align_dt_ms"] * 1e6
    tg = (g["t_ns"][ok] - off) / 1e9
    cg = g["p_bc"][ok].reshape(-1, 3)
    o = np.argsort(tg); tg, cg = tg[o], cg[o]
    w0, w1 = t[0], min(tg[-1], P[-1]["t"] - 3.0)
    m = (t >= w0) & (t <= w1)
    tw, pw = t[m], p[m]
    k = np.clip(np.searchsorted(tg, tw), 1, len(tg) - 1)
    k = np.where(np.abs(tg[k - 1] - tw) < np.abs(tg[k] - tw), k - 1, k)
    good = np.abs(tg[k] - tw) <= 0.002
    if good.sum() < 50:
        return dict(result=False, why="few GT matches")
    Rm, tr = umeyama_se3(pw[good], cg[k[good]])
    pa = (Rm @ pw.T).T + tr
    ate = float(np.sqrt(np.mean(np.sum((pa[good] - cg[k[good]]) ** 2, 1)))) * 100
    grid = np.arange(w0, w1 - 5, 0.1)
    rel = []
    ia = lambda x, T_, X: np.array([np.interp(x, T_, X[:, q]) for q in range(3)]).T
    if len(grid):
        # GT only where valid samples exist within 20 ms of both ends
        def gtok(x):
            i = np.clip(np.searchsorted(tg, x), 1, len(tg) - 1)
            return np.minimum(np.abs(tg[i] - x), np.abs(tg[i - 1] - x)) <= 0.02
        sel = gtok(grid) & gtok(grid + 5)
        gg = grid[sel]
        if len(gg):
            dv = ia(gg + 5, tw, pa) - ia(gg, tw, pa)
            dc = ia(gg + 5, tg, cg) - ia(gg, tg, cg)
            rel = np.linalg.norm(dv - dc, axis=1) * 100
    rel = np.asarray(rel)
    return dict(result=True, window=[float(w0), float(w1)], ate_w=ate, ate_bench=j["ate_cm"],
                rel5_p95=float(np.percentile(rel, 95)) if len(rel) else None,
                rel5_max=float(rel.max()) if len(rel) else None, n_rel=int(len(rel)),
                divergent=bool(ate > 50 or (len(rel) and rel.max() > 30)))


def unit_of(c):
    k = c["key"]
    if c["sub"] == "T":
        return k.rsplit("_", 1)[0], ["d0", "dm025", "dp025"].index(k.rsplit("_", 1)[1])
    return k.rsplit("_", 1)[0], int(k[-1])


def main():
    import input_events as ie  # noqa: E402
    OUT.mkdir(exist_ok=True)
    cells = zf.cells()
    recs = sorted({c["rec"] for c in cells})
    ev = {}
    for r in recs:
        try:
            d = ie.detect(r)
            ev[r] = dict(event=d["event"], reasons=d["reasons"], clusters_meta=d["event_clusters_meta"],
                         window_meta=d["window_s_meta"], source=d["interval_source"])
        except Exception as e:  # noqa: BLE001
            ev[r] = dict(event=None, error=repr(e)[:200])
        print("input_events", r, ev[r], flush=True)
    still = json.load(open(R / "zupt_gtfix_report" / "report.json"))["cells"]
    units = {}
    for c in cells:
        u, i = unit_of(c)
        m = metrics(c["key"], c["rec"])
        m["still"] = still[c["key"]]["base"]["still_zupt"]
        units.setdefault(u, dict(rec=c["rec"], sub=c["sub"], reps={}))["reps"][i] = m
        print(c["key"], {a: (round(b, 2) if isinstance(b, float) else b) for a, b in m.items() if a != "window"}, flush=True)
    for u, v in units.items():
        v["bundle"] = "impact" if ev.get(v["rec"], {}).get("event") else v["sub"]

    tests = []
    for i, j in itertools.permutations(range(3), 2):
        c2, c3 = {}, []
        for u, v in units.items():
            A, B = v["reps"].get(i), v["reps"].get(j)
            if not (A and B and A.get("result") and B.get("result")):
                continue
            b = c2.setdefault(v["bundle"], dict(n_cand=0, n_base=0, bad_units=[]))
            b["n_cand"] += A["divergent"]; b["n_base"] += B["divergent"]
            if A["divergent"] and not B["divergent"]:
                b["bad_units"].append(u)
            if v["bundle"] == "impact":
                continue
            why = []
            if A["rel5_p95"] is not None and B["rel5_p95"] is not None and A["rel5_p95"] > B["rel5_p95"] * 1.1 + 0.5:
                why.append(f"rel5 p95 {A['rel5_p95']:.2f} > {B['rel5_p95'] * 1.1 + 0.5:.2f}")
            if A["ate_w"] > B["ate_w"] * 1.2 + 0.5:
                why.append(f"ATE {A['ate_w']:.2f} > {B['ate_w'] * 1.2 + 0.5:.2f}")
            if B["still"] is not None and B["still"] >= 30 and (A["still"] or 0) < 0.9 * B["still"]:
                why.append(f"still {A['still']} < 0.9 x {B['still']}")
            c3.append(dict(unit=u, fail=bool(why), why=why))
        for b in c2.values():
            b["fail"] = b["n_cand"] > b["n_base"] or bool(b["bad_units"])
        tests.append(dict(cand=i, base=j, c2=c2, c2_fail=any(b["fail"] for b in c2.values()),
                          c3_fail=any(x["fail"] for x in c3), c3_units=len(c3),
                          c3_fail_units=[x for x in c3 if x["fail"]]))
    n = len(tests)
    nu = sum(t["c3_units"] for t in tests)
    nb = sum(len(t["c2"]) for t in tests)
    summ = dict(n_pseudo_tests=n,
                c2_test_fail_rate=sum(t["c2_fail"] for t in tests) / n,
                c3_test_fail_rate=sum(t["c3_fail"] for t in tests) / n,
                any_fail_rate=sum(t["c2_fail"] or t["c3_fail"] for t in tests) / n,
                c2_bundle_fail_rate=sum(b["fail"] for t in tests for b in t["c2"].values()) / nb,
                c3_unit_fail_rate=sum(len(t["c3_fail_units"]) for t in tests) / nu,
                c3_unit_fail_rate_rel5=sum(any("rel5" in w for w in x["why"]) for t in tests for x in t["c3_fail_units"]) / nu,
                c3_unit_fail_rate_ate=sum(any("ATE" in w for w in x["why"]) for t in tests for x in t["c3_fail_units"]) / nu,
                c3_unit_fail_rate_still=sum(any("still" in w for w in x["why"]) for t in tests for x in t["c3_fail_units"]) / nu)
    res = dict(made=time.strftime("%F %T"), note="PRELIMINARY A/A (3 replicates, base runs only). Not a verdict.",
               inputs=dict(base="runs/zupt_fixA/*_base", gt="recordings/<rec>/imu_gt_spl/gt.npz (c = p_bc)",
                           still="runs/zupt_gtfix_report/report.json"),
               input_events=ev, summary=summ, tests=tests,
               units={u: dict(rec=v["rec"], bundle=v["bundle"], reps=v["reps"]) for u, v in units.items()})
    json.dump(res, open(OUT / "aa.json", "w"), indent=1, default=float)
    print(json.dumps(summ, indent=1))
    for t in tests:
        print(f"cand o{t['cand']} vs base o{t['base']}: C2 {'FAIL' if t['c2_fail'] else 'ok'} "
              f"{ {k: (b['n_cand'], b['n_base'], b['bad_units']) for k, b in t['c2'].items() if b['fail']} } "
              f"C3 {'FAIL' if t['c3_fail'] else 'ok'} {[(x['unit'], x['why']) for x in t['c3_fail_units']]}")


if __name__ == "__main__":
    main()
