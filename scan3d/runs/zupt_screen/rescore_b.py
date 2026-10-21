#!/usr/bin/env python3
"""POST-HOC re-score of i3_takeoff B / B' (docs/i3_takeoff_plan.md unchanged; registered verdicts stand).
Paired cell differences (i3 - base) on continuous 10 s post-departure ATE (SE3 and Sim3), log ratio, sign counts,
sign-flip permutation p (exploratory). Recomputes each cell's window ATE from raw poses via i3_takeoff_analyse.window
to check summary.json."""
import sys, json, time, itertools, os
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "imu_learn")); os.chdir(ROOT)
S = json.load(open(ROOT / "runs/i3_takeoff/summary.json"))


def recompute(name, rec, t_dep_s, f0):
    import i3_takeoff_analyse as ta
    pre = ROOT / "runs/i3_takeoff" / name
    if not Path(str(pre) + ".json").exists():
        pre = ROOT / "runs/i3_confirm" / name
    r = ta.load_run(pre)
    return ta.window(r, rec, f0 + int(round(t_dep_s * 1e9)))


def perm_p(d, n=200000, seed=0):
    d = np.asarray(d, float)
    if len(d) <= 16:
        sig = np.array(list(itertools.product([-1, 1], repeat=len(d))))
    else:
        sig = np.random.default_rng(seed).choice([-1, 1], size=(n, len(d)))
    st = np.abs((sig * d).mean(1))
    return float((st >= abs(d.mean()) - 1e-12).mean())


def block(rows, label, active_key=None):
    import i3_confirm as ic
    cells = []
    for r in rows:
        if r.get("skip") or "base" not in r or "i3" not in r:
            continue
        b, i = r["base"], r["i3"]
        if b.get("ate_cm") is None or i.get("ate_cm") is None:
            cells.append(dict(key=r["key"], tested=False))
            continue
        x = ic.inputs(r["rec"]) if r["rec"] not in _INP else _INP[r["rec"]]
        _INP[r["rec"]] = x
        rb = recompute(b["name"], r["rec"], b["t_dep_s"], x["first_frame_imu_ns"])
        ri = recompute(i["name"], r["rec"], i["t_dep_s"], x["first_frame_imu_ns"])
        cells.append(dict(key=r["key"], rec=r["rec"][-6:], tested=bool(r.get("window_inside", True)),
                          gate=r.get("gate"), base=b["ate_cm"], i3=i["ate_cm"], base_s=b["ate_sim3_cm"], i3_s=i["ate_sim3_cm"],
                          base_scale=b["scale"], i3_scale=i["scale"], base_st=b["status"], i3_st=i["status"],
                          i3_active=bool(i.get("held_before_dep") or 0) or bool(i.get("frames_held")),
                          held_before_dep=i.get("held_before_dep"),
                          recheck=[rb.get("ate_cm"), ri.get("ate_cm")]))
    return cells


_INP = {}


def stats(cells, tag):
    c = [x for x in cells if x.get("tested")]
    if not c:
        return dict(tag=tag, n=0)
    d = np.array([x["i3"] - x["base"] for x in c]); ds = np.array([x["i3_s"] - x["base_s"] for x in c])
    lr = np.log(np.array([x["i3"] for x in c]) / np.array([x["base"] for x in c]))
    tie = 0.05
    return dict(tag=tag, n=len(c), recs=sorted({x["rec"] for x in c}),
                d_med=round(float(np.median(d)), 3), d_mean=round(float(d.mean()), 3),
                ds_med=round(float(np.median(ds)), 3), logratio_med=round(float(np.median(lr)), 3),
                i3_better=int((d < -tie).sum()), tie=int((np.abs(d) <= tie).sum()), i3_worse=int((d > tie).sum()),
                perm_p_mean_d=round(perm_p(d), 4), perm_p_logratio=round(perm_p(lr), 4),
                d_sorted=[round(float(v), 2) for v in sorted(d)])


out = dict(made=time.strftime("%F %T"), label="POST-HOC exploratory re-score; registered verdicts unchanged",
           input="runs/i3_takeoff/summary.json made " + S["made"] + " git " + S["git"])
B = block(S["B"]["rows"], "B")
for x in [y for y in B if "base" in y]:
    x["tested"] = x["key"] in S["B"]["criteria"]["tested"]
B2 = block(S["B2"]["rows"], "B2")
ta_keys = set(S["B2"]["criteria_a"]["tested"]); tb_keys = set(S["B2"]["criteria_b"]["tested"])
for x in [y for y in B2 if "base" in y]:
    x["part"] = "a" if x["key"] in ta_keys else ("b" if x["key"] in tb_keys else "b_out")
out["cells_B"] = B; out["cells_B2"] = B2
Bt = [x for x in B if x["tested"]]
out["stats"] = [
    stats(Bt, "B all 12 tested"),
    stats([x for x in Bt if x["rec"] != "214550"], "B without 214550 (9)"),
    stats([x for x in Bt if x["i3_active"]], "B i3 active (held before dep)"),
    stats([x for x in Bt if x["i3_active"] and x["rec"] != "214550"], "B i3 active, without 214550"),
    stats([x for x in B2 if x["part"] == "a"], "B'a (12, already-seen runs)"),
    stats([dict(x, tested=True) for x in B2 if x["part"] == "a" and x["rec"] == "214550"], "B'a 214550 only"),
    stats([dict(x, tested=True) for x in B2 if x["part"] != "a"], "B'b all 8 incl. out-of-gate (reference)"),
]
for s in out["stats"]:
    print(json.dumps(s, ensure_ascii=False))
print("cells B:")
for x in [y for y in B if "base" in y]:
    print(x["key"], x["tested"], x["base"], x["i3"], round(x["i3"] - x["base"], 3), x["base_scale"], x["i3_scale"], x["i3_active"], "recheck", x["recheck"])
print("cells B2:")
for x in [y for y in B2 if "base" in y]:
    print(x["key"], x["part"], x["base"], x["i3"], round(x["i3"] - x["base"], 3), x["base_scale"], x["i3_scale"], x["i3_active"], "recheck", x["recheck"])
mism = [x["key"] for x in B + B2 if "recheck" in x and x["recheck"][0] is not None and (abs(x["recheck"][0] - x["base"]) > 0.002 or abs(x["recheck"][1] - x["i3"]) > 0.002)]
out["recheck_mismatch"] = mism
print("recheck mismatches", mism)
json.dump(out, open(ROOT / "runs/zupt_screen/rescore_b.json", "w"), indent=1, ensure_ascii=False)
