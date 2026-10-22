#!/usr/bin/env python3
"""Mechanism re-check (runs/zupt_fix_review/judge.md 5 step 2): 133820 all-reject updates at 50.9-52.7 s (clip time).

For every all-reject normal update (in >= 10, rej/in >= 0.9) in 133820 o0-o2, all conditions, report:
  t (clip), t_abs (clip + offset), ZUPT acceptances in the preceding 10 s, last ZUPT before it, first ZUPT after it,
  SLAM count / in / rej of the update and the 5 normal updates before it,
  GT samples (ok & |p_bc| < 1.2) in [t-0.5, t+0.5] vs all GT samples (raw) in that window, GT speed (p_bc, fixed centre).
Is it inside "ZUPT streak end + 1 s" (the mechanism the plan targeted)?

  ../venv_ffs/bin/python imu_learn/zupt_mech133820.py   # -> runs/zupt_next_prep/mech133820.json
"""
import gzip, json, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import zupt_fixA_analyse as an  # noqa: E402  (log parser, fixed gt_speed_fn)

R = an.SCAN3D / "runs"
OUT = R / "zupt_next_prep"
CONDS = {"base": ("zupt_fixA", "base"), "A": ("zupt_fixA", "A"), "C": ("zupt_fixC", "C"),
         "Ap": ("zupt_fixAp", "Ap"), "A2": ("zupt_fixA2", "A2")}
OFFS = {0: 0.0, 1: 1.0, 2: 2.0}
REC = "hs_20261022_133820"


def main():
    OUT.mkdir(exist_ok=True)
    g = np.load(an.SCAN3D / "recordings" / REC / "imu_gt_spl" / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    res = dict(made=time.strftime("%F %T"), rec=REC, gt_fix="c = p_bc", items=[])
    for k, off in OFFS.items():
        for cond, (d, suf) in CONDS.items():
            pre = R / d / f"BR_133820_o{k}_{suf}"
            j = json.load(open(str(pre) + ".json"))
            fr = an.parse(str(pre) + ".stderr.log.gz")
            up = an.updates(fr)
            P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
            o = P[0]["t_ns"] - P[0]["t"] * 1e9 + j["align_dt_ms"] * 1e6
            tt_all = (g["t_ns"] - o) / 1e9
            sp = an.gt_speed_fn(REC, j, P[0])
            zt = np.array([f["t"] for f in fr if f["zacc"]])
            streak_ends = [b for a, b, n in an.pauses(fr)]
            ar = an.all_reject(up)
            for t in ar:
                if not (45 <= t <= 60):
                    continue
                i = next(n for n, f in enumerate(up) if round(f["t"], 3) == t)
                win = (tt_all >= t - 0.5) & (tt_all <= t + 0.5)
                prev_end = max([e for e in streak_ends if e < t], default=None)
                res["items"].append(dict(
                    off=k, cond=cond, t=t, t_abs=round(t + off, 3), t_gt_rel=round(float((t * 1e9 + o - g["t_ns"][0]) / 1e9), 3),
                    zupt_acc_prev10s=int(((zt >= t - 10) & (zt < t)).sum()),
                    last_zupt_before=float(zt[zt < t].max()) if (zt < t).any() else None,
                    first_zupt_after=float(zt[zt > t].min()) if (zt > t).any() else None,
                    last_streak_end_before=prev_end,
                    in_streak_end_plus_1s=bool(prev_end is not None and t - prev_end <= 1.0),
                    slam=up[i]["slam"], s_in=up[i]["sin"], s_rej=up[i]["srej"],
                    prev5=[(round(f["t"], 3), f["slam"], f["sin"], f["srej"]) for f in up[max(0, i - 5):i]],
                    gt_valid_1s=int((win & ok).sum()), gt_raw_1s=int(win.sum()),
                    gt_speed_cms=sp(t) if sp else None,
                    gt_speed_cms_pm05=[None if sp is None or sp(t + dt) is None else round(sp(t + dt), 1)
                                       for dt in (-1.0, -0.5, 0.5, 1.0)]))
    json.dump(res, open(OUT / "mech133820.json", "w"), indent=1, default=float)
    for it in res["items"]:
        print({a: b for a, b in it.items() if a != "prev5"})
        print("   prev5", it["prev5"])


if __name__ == "__main__":
    main()
