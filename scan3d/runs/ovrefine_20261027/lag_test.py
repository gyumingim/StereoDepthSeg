#!/usr/bin/env python3
"""ovrefine PLAN 7 (2nd hypothesis, committed before the test run): score B60 vs R10w lag 9 (delayed, smoothed) and lag 0
(= real-time ref) on the test HS27 + std60 + hand loops, then rules (1)(2) on HS27.
  python3 lag_test.py   -> lag_test.json (runs dir)
"""
import gzip
import importlib.util
import json
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def _load(name, f):
    spec = importlib.util.spec_from_file_location(name, HERE / f)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


ovr = _load("ovr", "ovrefine_run.py")
lag = _load("lagd", "lag_diag.py")
LAGS = (0, 9)


def variant(pre, L, which):
    P = ovr.load_P(pre)
    wst = [json.loads(x) for x in open(str(pre) + ".refine.jsonl") if '"wst"' in x]
    Q = lag.lag_poses(P, wst, L)
    dd = ovr.OUT / f"R10w_lag{L}"
    dd.mkdir(parents=True, exist_ok=True)
    dst = dd / pre.name
    r = json.load(open(str(pre) + ".json"))
    if which == "loop":
        tr = [m for m in Q if m.get("status") == "tracking" and m.get("T_world_cam0")]
        r["traj"] = [[int(m["t_ns"])] + [float(v) for v in np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3]]
                     + [int(m.get("n_slam") or 0), int(m.get("n_msckf") or 0)] for m in tr]
    json.dump(r, open(str(dst) + ".json", "w"))
    with gzip.open(str(dst) + ".poses.jsonl.gz", "wt") as f:
        for m in Q:
            f.write(json.dumps(m) + "\n")
    return dst, P


def main():
    import lens_switch_gtscore as gs
    from fi_limits_run import loop_eval  # noqa: E402
    gs.OUT = ovr.OUT / "score_tmp_lagtest"
    gs.OUT.mkdir(parents=True, exist_ok=True)
    res = dict(made=time.strftime("%F %T"), doc=__doc__,
               plan_commit=ovr.git("log", "-1", "--format=%H", "--", "runs/ovrefine_20261027/PLAN.md"),
               bridge=json.load(open(ovr.OUT / "bin" / "bridge.json")), runner_sha256=None)
    for which in ("test", "std"):
        gt = ovr.GT_HS if which == "test" else ovr.GT_STD
        rows = []
        for r, o in ovr.cells(which):
            k = ovr.key_of(r, o, which)
            bpre, pre = ovr.OUT / "B60" / f"{k}_B60", ovr.OUT / "R10w" / f"{k}_R10w"
            if not (Path(str(bpre) + ".json").exists() and Path(str(pre) + ".json").exists()):
                continue
            PB = ovr.load_P(bpre)
            mb = gs.cam_metrics(bpre, r, gt)
            trb = [m for m in PB if m.get("T_world_cam0") is not None]
            row = dict(key=k, rec=r, B60=dict(div=bool(mb.get("divergent")), ate_w=mb.get("ate_w"), rpe1=ovr.rpe1(bpre, r, gt),
                                              hash=ovr.vio_hash(PB), init_t_ns=trb[0]["t_ns"] if trb else None,
                                              pose0_sha=ovr.hashlib.sha256(json.dumps(trb[0]["T_world_cam0"]).encode()).hexdigest()[:12] if trb else None,
                                              plan_commit=json.load(open(str(bpre) + ".json")).get("plan_commit")))
            for L in LAGS:
                dst, P = variant(pre, L, which)
                mk = gs.cam_metrics(dst, r, gt)
                row[f"lag{L}"] = dict(div=bool(mk.get("divergent")), ate_w=mk.get("ate_w"), rpe1=ovr.rpe1(dst, r, gt))
            row["vio_same"] = ovr.vio_hash(P) == row["B60"]["hash"]
            row["rs"] = ovr.ref_stats(pre, P)
            rows.append(row)
            print(which, k, json.dumps({x: row[x] for x in ("B60", "lag0", "lag9", "vio_same")}), flush=True)
        S = {}
        for name in ("B60", "lag0", "lag9"):
            S[name] = ovr.summarize(rows, lambda x, n=name: x.get(n))
        Pp = {}
        for name in ("lag0", "lag9"):
            for met in ("ate_w", "rpe1"):
                Pp[f"B60->{name}.{met}"] = ovr.paired(rows, lambda x: x.get("B60"), lambda x, n=name: x.get(n), met)
        res[which] = dict(summary=S, paired=Pp, identity=dict(same=sum(x["vio_same"] for x in rows), n=len(rows)), cells=rows)
    loops = {}
    for r, o in ovr.cells("loop"):
        k = ovr.key_of(r, o, "loop")
        bpre, pre = ovr.OUT / "B60" / f"{k}_B60", ovr.OUT / "R10w" / f"{k}_R10w"
        if not (Path(str(bpre) + ".json").exists() and Path(str(pre) + ".json").exists()):
            continue
        e = {"B60": loop_eval(Path(str(bpre) + ".json"))}
        for L in LAGS:
            dst, P = variant(pre, L, "loop")
            e[f"lag{L}"] = loop_eval(Path(str(dst) + ".json"))
        loops[k] = {n: {x: v.get(x) for x in ("loop_err_cm", "path_m", "loop_pct", "tracked")} for n, v in e.items()}
        loops[k]["vio_same"] = ovr.vio_hash(P) == ovr.vio_hash(ovr.load_P(bpre))
        print("loop", k, json.dumps(loops[k]), flush=True)
    res["loops"] = loops
    t = res["test"]
    s9, b, p9 = t["summary"]["lag9"], t["summary"]["B60"], t["paired"]["B60->lag9.ate_w"]
    r1 = s9["div"] <= b["div"]
    r2 = p9.get("d_med") is not None and p9["d_med"] < 0
    res["verdict"] = dict(rule1=r1, rule2=r2, adoptable_pc=bool(r1 and r2))
    print(f"(1) div lag9 {s9['div']} {s9['div_cells']} vs B60 {b['div']} {b['div_cells']} -> {'PASS' if r1 else 'FAIL'}")
    print(f"(2) HS27 paired ATE d_med {p9.get('d_med')} (better {p9.get('b_better')}/{p9.get('n')}, rec {p9.get('rec_b_better')}) -> {'PASS' if r2 else 'FAIL'}")
    for k, v in t["paired"].items():
        print(" ", k, {x: v.get(x) for x in ("n", "d_med", "b_better", "rec_b_better")})
    for k, v in res["std"]["paired"].items():
        print("  std", k, {x: v.get(x) for x in ("n", "d_med", "b_better")})
    print("summary test", t["summary"], "std", res["std"]["summary"])
    print("VERDICT (PLAN 7):", "adoptable on PC (offline reconstruction)" if r1 and r2 else "NOT adopted")
    json.dump(res, open(ovr.RD / "lag_test.json", "w"), indent=1)
    print("saved", ovr.RD / "lag_test.json")


if __name__ == "__main__":
    main()
