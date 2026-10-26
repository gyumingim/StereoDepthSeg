#!/usr/bin/env python3
"""dpvofast 2026-10-26: score every knob run with the dpvo_20261026 ruler (dpvo_run.score_one / loop_eval), plus
OpenVINS GL60 on the SAME cells re-scored with the SAME GT-Sim3 alignment as DPVO (fairness, coordinator request):
  ov_sim3      = vio_bench_hs.evaluate Sim3 ATE on the OpenVINS trajectory (same code DPVO's sim3 uses)
  ov_ate_w_gts = window ATE (lens_switch_gtscore.cam_metrics) after scaling OpenVINS by its full GT Sim3 scale
  ov_ate_w_own = window ATE with OpenVINS's own (IMU) scale = the published 1.50/1.55/1.70
venv_ffs:  ~/scanner/StereoDepthSeg/venv_ffs/bin/python dpvofast_score.py  -> score.json, table.txt next to this file
"""
import gzip
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

RD = Path(__file__).resolve().parent
os.environ.setdefault("DPVO_OUT", "/home/system/scanner/tmp_dpvofast/out")
sys.path.insert(0, str(RD.parent / "dpvo_20261026"))
import dpvo_run as R  # noqa: E402

OUT = Path(os.environ["DPVO_OUT"])
OVD = Path("/home/system/scanner/tmp_dpvo/ov/GL60")
STD = ["s165506_o0_f60", "s165506_o1_f60", "s165506_o2_f60"]
LOOPS = ["l142148_o0_f60", "l142610_o0_f60"]


def loop_one(P, rec, tinit, tag):
    from fi_limits_run import loop_eval
    tr = [m for m in P if m.get("status") == "tracking" and m.get("T_world_cam0")]
    tmpj = OUT / "score_tmp" / "loops" / f"{tag}.json"
    tmpj.parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(rec=rec, tracked=len(tr), t_init_clip_s=tinit,
                   traj=[[m["t_ns"]] + np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3].tolist() + [0, 0] for m in tr]), open(tmpj, "w"))
    e = loop_eval(tmpj)
    return {k: e.get(k) for k in ("loop_err_cm", "path_m", "loop_pct", "tracked")}


def ov_rows(gs):
    rows = {}
    for name in STD + LOOPS:
        key = name.rsplit("_f", 1)[0]
        src = OVD / f"{key}_GL60.poses.jsonl.gz"
        meta = json.load(open(OVD / f"{key}_GL60.json"))
        rec = meta["rec"]
        P = R.load_lines(src)
        if name in LOOPS:
            rows[name] = dict(loop=loop_one(P, rec, None, f"ov_{key}"))
            continue
        d = OUT / "ovgl60"
        d.mkdir(parents=True, exist_ok=True)
        for kind in ("final", "causal"):
            ln = d / f"{name}_ovgl60.{kind}.poses.jsonl.gz"
            if not ln.exists():
                ln.symlink_to(src)
        res = dict(name=name, cfg="ovgl60", rec=rec, which="std", fps=60.0, frames_fed=meta["frames_fed"])
        s = R.score_one(res, "final", gs, None)
        # own-scale window ATE (no rescale), same cam_metrics
        dt_ns = int(json.load(open(R.MAIN / "recordings" / rec / "imu_gt" / "align.json"))["dt_ns"])
        dst = OUT / "score_tmp" / "ovgl60" / f"{name}_own"
        R.write_pre(dst, P, rec, dt_ns, 60.0)
        mo = gs.cam_metrics(dst, rec, R.STD_GT)
        rows[name] = dict(ov_sim3=s.get("sim3"), ov_scale=s.get("scale"), ov_ate_w_gts=s.get("ate_w"), ov_div_gts=s.get("div"),
                          ov_ate_w_own=mo.get("ate_w"), ov_div_own=bool(mo.get("divergent")), matched=s.get("matched"))
    return rows


def main():
    sys.path.insert(0, str(R.MAIN / "imu_learn"))
    sys.path.insert(0, str(R.MAIN))
    os.chdir(R.MAIN)
    import lens_switch_gtscore as gs
    gs.OUT = OUT / "score_tmp"
    gs.OUT.mkdir(parents=True, exist_ok=True)
    ov = ov_rows(gs)
    rows = []
    for cd in sorted(p for p in OUT.iterdir() if p.is_dir() and p.name not in ("cells", "split", "score_tmp", "ovgl60")):
        for jf in sorted(cd.glob("*.json")):
            res = json.load(open(jf))
            row = dict(tag=res["cfg"], name=res["name"], knobs=res.get("knobs"), size=res["size"], fps=res["fps"], frames=res["frames_fed"],
                       ms_med=res["ms_med"], ms_p95=res["ms_p95"], ms_mean=res["ms_mean"], mem_mb=res["gpu_mem_alloc_max_mb"],
                       t_init=res["t_init_clip_s"], error=res.get("error"), load1=res["load"][0], made=res["made"])
            for kind in ("final", "causal"):
                if res["which"] == "loop":
                    P = R.load_lines(cd / f"{res['name']}_{res['cfg']}.{kind}.poses.jsonl.gz")
                    row[kind] = loop_one(P, res["rec"], res["t_init_clip_s"], f"{res['name']}_{res['cfg']}_{kind}")
                else:
                    key = res["name"].rsplit("_f", 1)[0]
                    ref = OVD / f"{key}_GL60"
                    row[kind] = R.score_one(res, kind, gs, ref if Path(str(ref) + ".poses.jsonl.gz").exists() else None)
            rows.append(row)
            print(row["tag"], row["name"], row["ms_med"], {k: (row[k].get("ate_w"), row[k].get("loop_pct")) for k in ("final", "causal")}, flush=True)
    out = dict(made=time.strftime("%F %T"), doc=__doc__, ov=ov, rows=rows)
    json.dump(out, open(RD / "score.json", "w"), indent=1)
    table(out)


def table(out):
    rows = out["rows"]
    tags = []
    for r in rows:
        if r["tag"] not in tags:
            tags.append(r["tag"])
    L = [f"made {out['made']} (dpvofast_score.py)", "",
         "OpenVINS GL60 same cells: ate_w own-scale | ate_w GT-Sim3-scale | sim3 ATE (cm)"]
    for n in STD:
        o = out["ov"][n]
        L.append(f"  {n}: own {o['ov_ate_w_own']} | gts {o['ov_ate_w_gts']} | sim3 {o['ov_sim3']} (scale {o['ov_scale']})")
    for n in LOOPS:
        L.append(f"  {n}: loop {out['ov'][n]['loop']['loop_pct']} %")
    L += ["", "tag | size fps | ms med (mean of 5 cells' medians) p95 | std ate_w final o0/o1/o2 (med) | causal med | loops final 142148/142610 | causal | t_init"]
    for t in tags:
        rr = {r["name"]: r for r in rows if r["tag"] == t}
        if len(rr) < 5:
            L.append(f"{t}: incomplete {sorted(rr)}")
            continue
        ms = [rr[n]["ms_med"] for n in STD + LOOPS]
        p95 = [rr[n]["ms_p95"] for n in STD + LOOPS]

        def f(x):
            return "x" if x is None else f"{x:.2f}"
        sf = [rr[n]["final"].get("ate_w") for n in STD]
        sc = [rr[n]["causal"].get("ate_w") for n in STD]
        dv = sum(bool(rr[n]["final"].get("div")) for n in STD) + sum(bool(rr[n]["causal"].get("div")) for n in STD)
        lf = [rr[n]["final"].get("loop_pct") for n in LOOPS]
        lc = [rr[n]["causal"].get("loop_pct") for n in LOOPS]
        r0 = rr[STD[0]]
        L.append(f"{t} | {r0['size']} {r0['fps']:.0f} | {np.mean(ms):.1f} ({min(ms):.1f}-{max(ms):.1f}) p95 {np.mean(p95):.1f} | "
                 f"{'/'.join(f(x) for x in sf)} ({f(np.median([x for x in sf if x is not None]) if any(x is not None for x in sf) else None)}) | "
                 f"{'/'.join(f(x) for x in sc)} | {'/'.join(f(x) for x in lf)} % | {'/'.join(f(x) for x in lc)} % | div {dv} | "
                 f"init {'/'.join(str(rr[n]['t_init']) for n in STD + LOOPS)}")
    open(RD / "table.txt", "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "table":
        table(json.load(open(RD / "score.json")))
    else:
        main()
