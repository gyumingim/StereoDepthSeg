#!/usr/bin/env python3
"""Exemption-leak check (runs/zupt_fix_review/judge.md 5 step 3) on 214550 o0-o2 A2.

A2 exempts the start still from the cumulative test only while has_moved_since_zupt is false. In VioManager.cpp
has_moved_since_zupt = true on every non-ZUPT frame that reaches propagate (>= 5 clones), even with 0 features.
Per run this lists: first tracked pose, first normal update (= exemption ends) and its SLAM/MSCKF features,
number of normal updates with SLAM+MSCKF == 0 / < 10 before the first update with >= 10 features (t10),
'[ZUPTA2]: cumulative fail' times before t10 (and total), GT speed (fixed centre c = p_bc) at those times,
and first GT motion (speed > 2 cm/s for >= 0.5 s).

  ../venv_ffs/bin/python imu_learn/zupt_a2_leak.py   # -> runs/zupt_next_prep/a2_leak.json
"""
import gzip, json, re, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import zupt_fixA_analyse as an  # noqa: E402

R = an.SCAN3D / "runs"
OUT = R / "zupt_next_prep"
RE_A2 = re.compile(r"\[ZUPTA2\]: cumulative (ok|fail) \(([\d.]+) vs [\d.]+, (\d+) features")


def a2lines(path):
    out, t = [], None
    for line in gzip.open(path, "rt", errors="replace"):
        line = an.ANSI.sub("", line)
        m = an.RE_T.search(line)
        if m:
            t = float(m.group(1)); continue
        m = RE_A2.search(line)
        if m and t is not None:
            out.append((t, m.group(1), float(m.group(2)), int(m.group(3))))
    return out


def one(cell, cond="A2", d="zupt_fixA2"):
    pre = R / d / f"{cell}_{cond}"
    j = json.load(open(str(pre) + ".json"))
    fr = an.parse(str(pre) + ".stderr.log.gz")
    P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
    tr = [p["t"] for p in P if p.get("status") == "tracking" and p.get("p")]
    sp = an.gt_speed_fn(Path(j["rec"]).name, j, P[0])
    up = [f for f in fr if not f["zacc"] and (f["slam"] is not None or f["msckf"] is not None)]
    nf = lambda f: (f["slam"] or 0) + (f["msckf"] or 0)
    t10 = next((f["t"] for f in up if nf(f) >= 10), None)
    first = up[0] if up else None
    # first GT motion: speed > 2 cm/s on every 0.1 s step for 0.5 s
    tmove = None
    for x in np.arange(tr[0] if tr else 0, 30, 0.05):
        v = [sp(x + k * 0.1) for k in range(6)] if sp else []
        if v and all(s is not None and s > 2 for s in v):
            tmove = round(float(x), 2)
            break
    a2 = a2lines(str(pre) + ".stderr.log.gz")
    fails = [(t, dd, n) for t, s, dd, n in a2 if s == "fail"]
    pre10 = [(round(t, 3), dd, n, None if sp is None or sp(t) is None else round(sp(t), 3)) for t, dd, n in fails
             if t10 is None or t < t10]
    return dict(cell=cell, cond=cond, first_track=tr[0] if tr else None,
                first_update=None if first is None else dict(t=first["t"], slam=first["slam"], msckf=first["msckf"]),
                n_upd_0feat_before_t10=sum(1 for f in up if t10 is not None and f["t"] < t10 and nf(f) == 0),
                n_upd_lt10_before_t10=sum(1 for f in up if t10 is not None and f["t"] < t10),
                t_first_update_ge10feat=t10, t_gt_motion=tmove,
                n_cum_checks_before_t10=sum(1 for t, *_ in a2 if t10 is None or t < t10),
                n_cum_fail_before_t10=len(pre10), cum_fail_before_t10=pre10,
                n_cum_fail_total=len(fails))


def main():
    OUT.mkdir(exist_ok=True)
    res = dict(made=time.strftime("%F %T"), note="A2 start-still exemption leak check (fixed GT centre)", items=[])
    for k in range(3):
        res["items"].append(one(f"I3_214550_o{k}"))
    json.dump(res, open(OUT / "a2_leak.json", "w"), indent=1, default=float)
    for it in res["items"]:
        print({a: b for a, b in it.items() if a != "cum_fail_before_t10"})
        print("   fails<t10", it["cum_fail_before_t10"])


if __name__ == "__main__":
    main()
