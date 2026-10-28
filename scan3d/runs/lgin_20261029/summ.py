#!/usr/bin/env python3
"""lgin 2026-10-29: apply the pre-registered rule (README) to the main run and tabulate main + exploratory results.
  summ.py OUT_DIR -> OUT_DIR/summary.json + printed tables"""
import glob
import json
import os
import re
import sys
import time

O = sys.argv[1]
MAIN = ["165506", "213341", "214550", "215348", "221925", "222747", "223815", "224323"]


def load(cond, rec, w, s, k):
    pre = os.path.join(O, cond, f"{rec}_w{w}_s{s}_r{k}")
    r = dict(pre=pre, exists=os.path.exists(pre + ".json"))
    if not r["exists"]:
        return r
    job = json.load(open(pre + ".job.json")) if os.path.exists(pre + ".job.json") else {}
    sc = json.load(open(pre + ".score.json")) if os.path.exists(pre + ".score.json") else {}
    cs = json.load(open(pre + ".cand.json")) if os.path.exists(pre + ".cand.json") else {}
    be = open(pre + ".bridge.err", errors="replace").read() if os.path.exists(pre + ".bridge.err") else ""
    m = re.search(r"keyframes pushed (\d+) dropped (\d+)", be)
    xf_down = sum(1 for s_ in open(pre + ".loop.jsonl", errors="replace") if "xf_down" in s_) if os.path.exists(pre + ".loop.jsonl") else -1
    r.update(exit=job.get("exit"), map_ate=(sc.get("map") or {}).get("ate_cm"), vio_ate=(sc.get("vio") or {}).get("ate_cm"),
             map_end=(sc.get("map") or {}).get("end_cm"), loops=(sc.get("loop") or {}).get("n_loop"),
             correct=(sc.get("loop") or {}).get("n_loop_correct"), loop_gt=(sc.get("loop") or {}).get("n_loop_gt"),
             kf_pushed_dropped=[int(m.group(1)), int(m.group(2))] if m else None, xf_down=xf_down, server_ok="[xf_server] F:" in be,
             **{k_: cs.get(k_) for k_ in ("n_cand", "n_cand_gt", "n_cand_gt_rev", "n_pass", "n_pass_gt", "n_pass_gt_rev", "pass_precision",
                                          "pass_recall", "rpe_rot_med_deg", "rpe_rot_max_deg", "rpe_trans_med_cm", "rpe_trans_max_cm",
                                          "convention_suspect", "lg_nv_med", "lg_ms_med", "loop_path_between_max_m", "status_counts")})
    r["wrong"] = (r["loop_gt"] - r["correct"]) if r.get("loop_gt") is not None else None
    r["valid"] = r["exit"] == 0 and r["map_ate"] is not None and r["xf_down"] == 0 and r["server_ok"]
    return r


W = {"165506": 0, "213341": 40, "214550": 40}
res = dict(made=time.strftime("%F %T"), out=O, main={}, verdict={}, expl={})
side = {"A": 0, "B": 0, "tie": 0, "none": 0}
wrong = {"A": 0, "B": 0}
for rec in MAIN:
    w = W.get(rec, 30)
    R = {f"{c}{k}": load(c, rec, w, 0, k) for c in ("A", "B") for k in (1, 2)}
    R.update({f"{c}s1": load(c, rec, w, 1, 1) for c in ("A", "B")})
    res["main"][rec] = R
    for key, r in R.items():
        if r.get("wrong") is not None:
            wrong[key[0]] += r["wrong"]
    four = [R["A1"], R["A2"], R["B1"], R["B2"]]
    if all(x.get("valid") for x in four):
        a = [R["A1"]["map_ate"], R["A2"]["map_ate"]]
        b = [R["B1"]["map_ate"], R["B2"]["map_ate"]]
        v = "A" if max(a) < min(b) else ("B" if max(b) < min(a) else "tie")
    else:
        v = "none"
    side[v] += 1
    res["verdict"][rec] = v
over = "A" if side["A"] >= 2 and side["B"] == 0 else ("B" if side["B"] >= 2 and side["A"] == 0 else "no difference seen")
if over == "B" and wrong["B"] <= wrong["A"]:
    choice = "B"
elif over == "A" and wrong["A"] > wrong["B"]:
    choice = "A (but more wrong loops)"
else:
    choice = "A"
res.update(sides=side, overall=over, wrong_loops=wrong, choice=choice)
# exploratory
for c in ("Bm", "A640", "B640"):
    res["expl"][c] = {rec: load(c, rec, W.get(rec, 30), 1, 1) for rec in MAIN}
for c in ("A", "B"):
    res["expl"][c + "_full"] = {rec: load(c, rec, 0, 1, 1) for rec in MAIN[1:]}
json.dump(res, open(os.path.join(O, "summary.json"), "w"), indent=1)


def f(x, n=3):
    return "-" if x is None else (f"{x:.{n}f}" if isinstance(x, float) else str(x))


print("rec | A1 A2 | B1 B2 (map ATE cm, sync0) | verdict | A s1 / B s1 | loops corr/acc A1 A2 B1 B2 | dropped A1 B1")
for rec in MAIN:
    R = res["main"][rec]
    print(rec, "|", f(R["A1"].get("map_ate")), f(R["A2"].get("map_ate")), "|", f(R["B1"].get("map_ate")), f(R["B2"].get("map_ate")), "|",
          res["verdict"][rec], "|", f(R["As1"].get("map_ate")), "/", f(R["Bs1"].get("map_ate")), "|",
          " ".join(f"{R[k].get('correct')}/{R[k].get('loops')}" for k in ("A1", "A2", "B1", "B2")), "|",
          R["A1"].get("kf_pushed_dropped"), R["B1"].get("kf_pushed_dropped"), "vio", f(R["A1"].get("vio_ate")), f(R["B1"].get("vio_ate")))
print("sides", side, "overall", over, "wrong", wrong, "choice", choice)
print("\ncand (sync1): rec | cond n_cand gt_rev pass pass_prec pass_rec loops corr rpe_rot_med rpe_tr_med nv")
for rec in MAIN:
    for c in ("A", "B"):
        r = res["main"][rec][c + "s1"]
        print(rec, c, r.get("n_cand"), r.get("n_cand_gt_rev"), r.get("n_pass"), r.get("pass_precision"), r.get("pass_recall"),
              r.get("loops"), r.get("correct"), r.get("rpe_rot_med_deg"), r.get("rpe_trans_med_cm"), r.get("lg_nv_med"), r.get("convention_suspect"))
print("\nexpl: cond rec map_ate loops correct n_pass prec rpe_rot rpe_tr")
for c, D in res["expl"].items():
    for rec, r in D.items():
        if r.get("exists"):
            print(c, rec, f(r.get("map_ate")), r.get("loops"), r.get("correct"), r.get("n_pass"), r.get("pass_precision"),
                  r.get("rpe_rot_med_deg"), r.get("rpe_trans_med_cm"), r.get("lg_nv_med"), "valid" if r.get("valid") else "INVALID")
