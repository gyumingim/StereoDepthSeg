#!/usr/bin/env python3
"""REPORTED-ONLY items for the fix A2 test (docs/zupt_fixA2_plan.md; same as zupt_fixAp_extra.py plus A2, "보고만 하는 항목"). Not used for the verdict.

Per cell and condition (base / A / C / Ap / A2):
  wipe1s   = wipe-outs with a 1 s window: normal update j with N_j <= 3 and some normal update i, t_j-1.0 <= t_i < t_j,
             N_i >= 20; re-armed only after N >= 20 again (same as the plan's rule, window 0.2 -> 1.0 s)
  sim3_cm  = bench ate_sim3_cm
  rel1s_cm = mean over a 0.1 s grid of | |p(t+1)-p(t)| - |c_gt(t+1)-c_gt(t)| | (tracked poses vs GT camera centre), cm
Pre-registered 154320 o0 diagnostic (window = [first tracked pose, 7.5 s]): ZUPT-accepted count, longest ZUPT streak
(frames), "split" = longest streak <= 3 frames AND count <= 0.8 x base count; prediction: split -> run collapse.

  ../venv_ffs/bin/python imu_learn/zupt_fixA2_extra.py     # -> runs/zupt_fixA2/extra.json
"""
import gzip
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import zupt_fixA_analyse as an  # noqa: E402  (log parser only)
import zupt_fixC_diagA as dg    # noqa: E402  (GT camera centre)
import zupt_fixA as zf          # noqa: E402  (cell list)

R = an.SCAN3D / "runs"
SRC = {"base": (R / "zupt_fixA", "base"), "A": (R / "zupt_fixA", "A"), "C": (R / "zupt_fixC", "C"),
       "Ap": (R / "zupt_fixAp", "Ap"), "A2": (R / "zupt_fixA2", "A2")}


def wipe1s(up):
    out, armed = [], True
    for j, f in enumerate(up):
        if f["slam"] >= 20:
            armed = True
        if not armed or f["slam"] > 3:
            continue
        k = j - 1
        while k >= 0 and up[k]["t"] >= f["t"] - 1.0:
            if up[k]["slam"] >= 20:
                out.append(round(f["t"], 3))
                armed = False
                break
            k -= 1
    return out


def one(cell, rec, cond):
    d, suf = SRC[cond]
    pre = d / f"{cell}_{suf}"
    if not Path(str(pre) + ".json").exists():
        return None
    j = json.load(open(str(pre) + ".json"))
    if j.get("ate_cm") is None or not Path(str(pre) + ".stderr.log.gz").exists():
        return dict(result=False)
    fr = an.parse(str(pre) + ".stderr.log.gz")
    up = an.updates(fr)
    P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
    T = [p for p in P if p.get("status") == "tracking" and p.get("p")]
    t = np.array([p["t"] for p in T])
    p = np.array([p["p"] for p in T])
    r = dict(result=True, ate_cm=j["ate_cm"], sim3_cm=j.get("ate_sim3_cm"), wipe1s=wipe1s(up),
             first_track=float(t[0]) if len(t) else None)
    try:
        gt, gc = dg.gt_center(rec, j, P[0])
        grid = np.arange(max(t[0], gt[0]), min(t[-1], gt[-1]) - 1, 0.1)
        dv = np.linalg.norm(dg.interp(grid + 1, t, p) - dg.interp(grid, t, p), axis=1)
        dgt = np.linalg.norm(dg.interp(grid + 1, gt, gc) - dg.interp(grid, gt, gc), axis=1)
        r["rel1s_cm"] = round(float(np.mean(np.abs(dv - dgt))) * 100, 3) if len(grid) else None
    except FileNotFoundError:
        r["rel1s_cm"] = None
    if cell == "Z_O_154320_o0":
        t0 = r["first_track"]
        st, s = [], 0
        for f in fr:
            if not (t0 <= f["t"] <= 7.5):
                continue
            if f["zacc"]:
                s += 1
            elif f["slam"] is not None:
                if s:
                    st.append(s)
                s = 0
        if s:
            st.append(s)
        r["diag154320"] = dict(window=[t0, 7.5], n_zupt=int(sum(st)), longest=int(max(st, default=0)),
                               median=float(np.median(st)) if st else 0.0, n_streaks=len(st))
    return r


def main():
    res = dict(made=time.strftime("%F %T"), plan="docs/zupt_fixA2_plan.md", note="reported only, not used for the verdict",
               cells={})
    for c in zf.cells():
        rec = c["rec"]
        res["cells"][c["key"]] = {k: one(c["key"], rec, k) for k in SRC}
    d = {k: (v or {}).get("diag154320") for k, v in res["cells"]["Z_O_154320_o0"].items()}
    if d.get("base") and d.get("A2"):
        split = d["A2"]["longest"] <= 3 and d["A2"]["n_zupt"] <= 0.8 * d["base"]["n_zupt"]
        res["diag154320"] = dict(per_cond=d, A2_split=split, prediction="collapse" if split else "no collapse")
    json.dump(res, open(R / "zupt_fixA2" / "extra.json", "w"), indent=1)
    print(json.dumps(res.get("diag154320"), indent=1))
    for k, v in res["cells"].items():
        print(k, {c: None if not x or not x.get("result") else (x["ate_cm"], x["sim3_cm"], x.get("rel1s_cm"), len(x["wipe1s"]))
                  for c, x in v.items()})


if __name__ == "__main__":
    main()
