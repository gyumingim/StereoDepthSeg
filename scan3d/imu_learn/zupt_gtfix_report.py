#!/usr/bin/env python3
"""REPORT-ONLY re-count after the GT camera-centre fix (runs/zupt_fix_review/judge.md, section 4 item 0 / section 5 step 1).

Bug: the zupt_fix* analysis / check / extra / tcum scripts used c = -R_bc^T p_bc as the GT camera centre.
p_bc is already the camera centre (bench ATE uses p_bc; 134204 ATE 1.32 cm with p_bc vs 27.3 cm with -R^T p).
Fixed in the scripts on 2026-10-23. This script re-counts, with the fixed scripts, for base / A / C / A' / A2:
  still_zupt = ZUPT-accepted frames with GT camera speed < 0.05 cm/s over +-0.25 s (zupt_fixA2_check.still_zupt_count)
  rel1s_cm   = zupt_fixA2_extra.one() 1 s relative error
  T_cum      = zupt_fixA2_tcum recipe (p95 of cumulative disparity at true-still frames in runs/zupt_fixA/*_A), rounded up
and lists them next to the old (buggy-centre) values from the committed check.json / extra.json / tcum.json.
NOTHING here changes any verdict (A / C / A' / A2 stay "not adopted"). Old result files are not overwritten.

  ../venv_ffs/bin/python imu_learn/zupt_gtfix_report.py   # -> runs/zupt_gtfix_report/report.json, report.out
"""
import gzip, json, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import zupt_fixA_analyse as an      # noqa: E402  (fixed gt_speed_fn)
import zupt_fixA2_check as ck       # noqa: E402  (fixed gt_centres / still_zupt_count)
import zupt_fixA2_extra as ex       # noqa: E402  (rel1s via fixed zupt_fixC_diagA.gt_center)
import zupt_fixA2_tcum as tc        # noqa: E402  (disparity parser)
import zupt_fixA as zf              # noqa: E402  (cell list)

R = an.SCAN3D / "runs"
OUT = R / "zupt_gtfix_report"
CONDS = {"base": ("zupt_fixA", "base"), "A": ("zupt_fixA", "A"), "C": ("zupt_fixC", "C"),
         "Ap": ("zupt_fixAp", "Ap"), "A2": ("zupt_fixA2", "A2")}
OLD_CHECK = {"A": "zupt_fixA", "C": "zupt_fixC", "Ap": "zupt_fixAp", "A2": "zupt_fixA2"}


def still(cell, cond):
    d, suf = CONDS[cond]
    r = ck.analyse_run(R / d / f"{cell}_{suf}")
    return r.get("still_zupt"), r.get("still_zupt_evaluable")


def old_still(cond):
    j = json.load(open(R / OLD_CHECK[cond] / "check.json"))
    return {c: ((v.get("base") or {}).get("still_zupt"), (v.get(cond) or {}).get("still_zupt"))
            for c, v in j["cells"].items()}


def tcum_fixed():
    allv = []
    for c in zf.cells():
        pre = R / "zupt_fixA" / f"{c['key']}_A"
        if not Path(str(pre) + ".stderr.log.gz").exists():
            continue
        j = json.load(open(str(pre) + ".json"))
        if j.get("ate_cm") is None:
            continue
        P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
        tr = [p["t"] for p in P if p.get("status") == "tracking" and p.get("p")]
        t0 = tr[0] if tr else 1e9
        sp = an.gt_speed_fn(Path(j["rec"]).name, j, P[0])
        if sp is None:
            continue
        allv += [d for t, d, n in tc.disp(str(pre) + ".stderr.log.gz")
                 if t >= t0 and n > 20 and (s := sp(t)) is not None and s < 0.05]
    p95 = float(np.percentile(allv, 95))
    return dict(n=len(allv), p50=float(np.percentile(allv, 50)), p95=p95, p99=float(np.percentile(allv, 99)),
                T_cum=float(np.ceil(p95 * 10 - 1e-9) / 10))


def main():
    OUT.mkdir(exist_ok=True)
    cells = [c["key"] for c in zf.cells()]
    old_ex = json.load(open(R / "zupt_fixA2" / "extra.json"))["cells"]
    olds = {k: old_still(k) for k in OLD_CHECK}
    rows = {}
    for c in cells:
        row = {}
        for k in CONDS:
            s, e = still(c, k)
            x = ex.one(c, next(z["rec"] for z in zf.cells() if z["key"] == c), k)
            row[k] = dict(still_zupt=s, still_eval=e, rel1s_cm=(x or {}).get("rel1s_cm"),
                          rel1s_cm_old=((old_ex.get(c) or {}).get(k) or {}).get("rel1s_cm"),
                          still_zupt_old=(olds[k].get(c) or (None, None))[1] if k in olds else
                          (olds["A"].get(c) or (None, None))[0])
        rows[c] = row
        print(c, {k: (v["still_zupt"], v["still_zupt_old"], v["rel1s_cm"], v["rel1s_cm_old"]) for k, v in row.items()}, flush=True)

    summ = {}
    for k in ("A", "C", "Ap", "A2"):
        judged, listed = [], []
        for c, row in rows.items():
            b, a = row["base"]["still_zupt"], row[k]["still_zupt"]
            if b is None or b < 30 or a is None:
                listed.append(c)
                continue
            judged.append((c, b, a, a / b))
        rat = [x[3] for x in judged]
        rel = [(row["base"]["rel1s_cm"], row[k]["rel1s_cm"]) for row in rows.values()
               if row["base"]["rel1s_cm"] is not None and row[k]["rel1s_cm"] is not None]
        relo = [(row["base"]["rel1s_cm_old"], row[k]["rel1s_cm_old"]) for row in rows.values()
                if row["base"]["rel1s_cm_old"] is not None and row[k]["rel1s_cm_old"] is not None]
        summ[k] = dict(still_cells_judged=len(judged), still_listed_only=len(listed),
                       still_ratio_min=min(rat) if rat else None, still_ratio_median=float(np.median(rat)) if rat else None,
                       still_cells_below_0_9=[dict(cell=c, base=b, cand=a, ratio=round(r, 3)) for c, b, a, r in judged if r < 0.9],
                       rel1s_mean_base=float(np.mean([a for a, _ in rel])) if rel else None,
                       rel1s_mean_cand=float(np.mean([b for _, b in rel])) if rel else None, rel1s_n=len(rel),
                       rel1s_mean_base_old=float(np.mean([a for a, _ in relo])) if relo else None,
                       rel1s_mean_cand_old=float(np.mean([b for _, b in relo])) if relo else None)
    tcum = tcum_fixed()
    tcum_old = {k: v for k, v in json.load(open(R / "zupt_fixA2" / "tcum.json")).items()
                if k in ("n_samples", "p50", "p95", "p99", "T_cum")}
    res = dict(made=time.strftime("%F %T"), note="REPORT ONLY after GT camera-centre fix (c = p_bc). No verdict changes.",
               inputs=dict(base="runs/zupt_fixA/*_base", A="runs/zupt_fixA/*_A", C="runs/zupt_fixC/*_C",
                           Ap="runs/zupt_fixAp/*_Ap", A2="runs/zupt_fixA2/*_A2", gt="recordings/<rec>/imu_gt_spl/gt.npz"),
               tcum_fixed=tcum, tcum_old=tcum_old, summary=summ, cells=rows)
    json.dump(res, open(OUT / "report.json", "w"), indent=1, default=float)
    print("TCUM fixed", tcum, "old", tcum_old)
    for k, v in summ.items():
        print(k, {a: b for a, b in v.items()})


if __name__ == "__main__":
    main()
