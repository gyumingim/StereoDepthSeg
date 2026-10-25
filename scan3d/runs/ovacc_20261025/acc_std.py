#!/usr/bin/env python3
"""ovacc: supplementary live-like set = std60 board recordings (165506, 173432) x offsets 0/1/2 (acc_run.STD).
Same jobs as acc_run.py (imports it, conditions from acc_run.CONDS + the std-only ones below), GT imu_gt_spl.

  python3 acc_std.py run L0,LC,FXp [--par 6]
  python3 acc_std.py score L0,LC,FXp --ref L0
"""
import argparse
import gzip
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import acc_run as A  # noqa: E402

# std60 chordal-mean R_ci (runs/std60_board_20261024/summary.json, 0.96 deg from the HS mean), dt = live -2.17 ms
EXTRA = {
    "LS": A.L(live_cal=True, std_R=True),
}


def install():
    A.CONDS.update(EXTRA)
    A.cells = lambda: (A.cells_std(), None, set())
    orig_wc_job = A.job

    def job(name):
        cond = name.split("_")[2]
        d = A.CONDS[cond]
        if d.get("std_R"):
            # swap the live R_ci for the std60 mean: patch variants.json reading via a temp file
            import tempfile
            vj = json.load(open(A.VARJSON))
            vj["R_ci"] = json.load(open(A.STD_SUMMARY))["std60_R_ci_chordal_mean"]
            t = Path(tempfile.mkdtemp(dir=str(A.OUT / "tmp"))) / "variants_stdR.json"
            json.dump(vj, open(t, "w"))
            A.VARJSON = t
        return orig_wc_job(name)
    A.job = job


def score(conds, ref):
    import lens_switch_gtscore as gs
    gs.OUT = A.OUT / "score_tmp"
    gs.OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for r, o in A.cells_std():
        row = dict(rec=r, off=o, key=f"{r.split('_')[2]}_o{o}")
        for c in conds:
            pre = A.OUT / c / A.name_of(r, o, c)
            if not Path(str(pre) + ".json").exists():
                continue
            st = json.load(open(str(pre) + ".json"))
            mc = gs.cam_metrics(pre, r, A.STD_GT)
            P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
            trk = [m for m in P if m.get("status") == "tracking"]
            last = trk[-1] if trk else {}
            row[c] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), why=mc.get("why"), tracked=st.get("tracked"),
                          scale=st.get("scale"), dt_final_ms=st.get("dt_cam_imu_final_ms"),
                          R_ItoC_final=last.get("R_ItoC0"), applied=st.get("ovacc_applied", {}).get("timeshift"))
        rows.append(row)
    print(f"| cell | " + " | ".join(conds) + " |")
    print("|---" * (len(conds) + 1) + "|")
    for x in rows:
        print(f"| {x['key']} | " + " | ".join(
            (f"{x[c]['ate_w']:.2f}{' DIV' if x[c]['div'] else ''} (trk {x[c]['tracked']})" if c in x and x[c]['ate_w'] is not None
             else (f"none (trk {x[c]['tracked']})" if c in x else "-")) for c in conds) + " |")
    json.dump(dict(made=time.strftime("%F %T"), conds=conds, ref=ref, gt=A.STD_GT, cells=rows),
              open(A.OUT / f"score_std_{'_'.join(conds)}.json", "w"), indent=1)
    print("saved", A.OUT / f"score_std_{'_'.join(conds)}.json")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "job", "score"])
    ap.add_argument("arg")
    ap.add_argument("--par", type=int, default=6)
    ap.add_argument("--ref", default="L0")
    a = ap.parse_args()
    install()
    if a.cmd == "run":
        # run_sched spawns "acc_run.py job NAME" -> make it spawn this file instead (std cells + extra conds)
        A.__file__ = __file__
        A.run_sched(a.arg.split(","), a.par)
    elif a.cmd == "job":
        A.job(a.arg)
    else:
        score(a.arg.split(","), a.ref)
