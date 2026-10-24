#!/usr/bin/env python3
"""30 fps vs 60 fps test, 2026-10-24 (plan docs/fps30_plan.md, result docs/fps30_result.md).

Final baseline (imu_learn/fi_limits_run.py F1 = config/mono + try_zupt=false + F1, lever, new lens, strict init window
1.0 / disparity 15, pts 200, frame cache, no gyro rebias, num_opencv_threads=1) with only fps / clones / size changed:
  C1 = 60 fps, clones 11, 640x360 (HS + std60 loops reused from runs/fi_limits/*_F1; one cell re-run here as a check)
  C2 = 30 fps, clones 11, 640x360
  C3 = 30 fps, clones 20, 640x360
  C4 = 30 fps, clones 15, 960x540, init disparity 15 x 1.5 = 22.5 px (px threshold scaled with the image width)
  <cond>a = audit init (window 1.5 s, disparity = strict default from K: 2.30 px at 640, ~3.45 px at 960) - only for
            cells that do not initialise with the baseline init; reported separately.
Timing: ms per frame = ov_bridge 'ms' of each pose; a separate sequential pass (PAR 1, '__T' tag, runs/fps30/timing/)
on 3 HS cells + std142610 per condition.

  (setsid nohup ../venv_ffs/bin/python imu_learn/fps30_run.py run hs > runs/fps30/sched_hs.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/fps30_run.py run std|std165|check|timing|NAME,NAME
  ../venv_ffs/bin/python imu_learn/fps30_run.py job 133820_o0_C2[__T]
  ../venv_ffs/bin/python imu_learn/fps30_run.py score      # -> runs/fps30/score.json
"""
import argparse
import gzip
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
from phone60_run import RAW, OLD, DROP  # noqa: E402
from fi_limits_run import loop_eval, STD  # noqa: E402

PY = "../venv_ffs/bin/python"
OUT = Path("runs/fps30")
REUSE = Path("runs/fi_limits")            # C1 = <key>_F1 there
PAR = int(os.environ.get("PAR", 8))
LEVER = "config/imu_cam_lever.json"
OFFS = (0, 1, 2)
STD165 = "hs_20261024_165506_std60"       # only if docs/std60_board_check_20261024.md exists (GT imu_gt_spl)
CFG = "num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,fi_max_baseline=200"   # = fi_limits F1 cfg_over
# name: (fps, clones, size, init window s, init disparity px; None = strict default from K)
CONDS = {"C1": (60.0, 11, "640x360", 1.0, 15.0), "C2": (30.0, 11, "640x360", 1.0, 15.0),
         "C3": (30.0, 20, "640x360", 1.0, 15.0), "C4": (30.0, 15, "960x540", 1.0, 22.5)}
AUD = {c + "a": (v[0], v[1], v[2], 1.5, None) for c, v in CONDS.items()}
ALL = {**CONDS, **AUD}
GT = "imu_gt_spl_cal1022"
TIMING_CELLS = ["133820_o0", "214550_o0", "222747_o0", "std142610"]


def all_jobs():
    js = []
    for c in ALL:
        js += [dict(rec=r, off=o, cond=c, gt="auto", name=f"{r[-6:]}_o{o}_{c}") for r in RAW + OLD for o in OFFS]
        js += [dict(rec=r, off=0, cond=c, gt="none", name=f"std{r[-12:-6]}_{c}") for r in STD]
        js += [dict(rec=STD165, off=o, cond=c, gt="auto", name=f"b165506_o{o}_{c}") for o in OFFS]
    return js


def job(name):
    import vio_bench_hs as vb
    base, _, tag = name.partition("__")
    j = next(x for x in all_jobs() if x["name"] == base)
    fps, clones, size, iw, idp = ALL[j["cond"]]
    vb.TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
    a = argparse.Namespace(rec=f"recordings/{j['rec']}", fps=fps, size=size, npts=200, clones=clones, noise="cfg",
                           noise_scale=1.0, imu_calib=None, lever=LEVER, lens_file=None, dropout="", max_secs=0.0,
                           imu_raw=j["rec"] not in OLD, gt=j["gt"], out=None, init_cfg="strict", start_off=float(j["off"]),
                           decode_threads=2, imu_lead=2.0, init_window=iw, init_disp=idp, frame_cache=True,
                           gyro_rebias=False, cfg_over=CFG, still_hold=None)
    saved = {}
    orig = vb.evaluate

    def ev(rec_, a_, poses, *rest):
        saved["poses"] = poses
        return orig(rec_, a_, poses, *rest)
    vb.evaluate = ev
    t0 = time.time()
    try:
        r = vb.run(a)
    finally:
        vb.evaluate = orig
    r.update(f30_made=time.strftime("%F %T"), f30_job_s=round(time.time() - t0, 1), f30_runner="imu_learn/fps30_run.py",
             f30_cond=j["cond"], f30_tag=tag, f30_init=[iw, idp], f30_load=list(os.getloadavg()))
    od = OUT / "timing" if tag == "T" else OUT
    od.mkdir(parents=True, exist_ok=True)
    with gzip.open(str(od / name) + ".poses.jsonl.gz", "wt") as f:
        for m in saved.get("poses", []):
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(od / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def names(which):
    if which == "hs":
        return [f"{r[-6:]}_o{o}_{c}" for c in ("C2", "C3", "C4") for r in RAW + OLD for o in OFFS]
    if which == "std":
        return [f"std{r[-12:-6]}_{c}" for c in ("C2", "C3", "C4") for r in STD]
    if which == "std165":
        return [f"b165506_o{o}_{c}" for c in CONDS for o in OFFS]
    if which == "check":
        return ["133820_o0_C1"]
    if which == "timing":
        return [f"{k}_{c}__T" for c in CONDS for k in TIMING_CELLS]
    return which.split(",")             # explicit list (e.g. audit-init variants)


def out_json(n):
    return (OUT / "timing" if n.endswith("__T") else OUT) / f"{n}.json"


def run_sched(which):
    q = [n for n in names(which) if not out_json(n).exists()]
    par = 1 if which == "timing" else PAR
    q.sort(key=lambda n: "224323" not in n)
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {which} {len(q)} jobs PAR {par}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if out_json(n).exists() else 'FAILED'}",
                      flush=True)
        while q and len(running) < par:
            n = q.pop(0)
            out_json(n).parent.mkdir(parents=True, exist_ok=True)
            running[n] = (subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/fps30_run.py", "job", n], stdout=subprocess.DEVNULL,
                                           stderr=open(out_json(n).with_suffix(".err"), "w")), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end {which} wall {time.time() - t_all:.0f} s", flush=True)


def pre_of(key, c):
    """C1 HS/std cells come from runs/fi_limits (<key>_F1) unless re-run here."""
    p = OUT / f"{key}_{c}"
    if Path(str(p) + ".json").exists() or c != "C1" or key.startswith("b165506"):
        return p
    return REUSE / f"{key}_F1"


def ms_stats(pre):
    P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
    a = np.array([m["ms"] for m in P if "ms" in m], float)
    t = np.array([m["ms"] for m in P if "ms" in m and m.get("status") == "tracking"], float)

    def f(v, q):
        return round(float(np.percentile(v, q)), 2) if len(v) else None
    return dict(n=len(a), med=f(a, 50), p95=f(a, 95), trk_med=f(t, 50), trk_p95=f(t, 95))


def score():
    import lens_switch_gtscore as gs
    os.chdir(SCAN3D)
    gs.OUT = OUT
    rows = {}
    for r in RAW + OLD:
        for o in OFFS:
            key = f"{r[-6:]}_o{o}"
            row = dict(rec=r, key=key, off=o)
            for c in ALL:
                pre = pre_of(key, c)
                if not Path(str(pre) + ".json").exists():
                    continue
                st = json.load(open(str(pre) + ".json"))
                mc = gs.cam_metrics(pre, r, GT)
                be = gs.bench_eval(pre, r, GT)
                row[c] = dict(src=str(pre), div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), rel5_max=mc.get("rel5_max"),
                              why=mc.get("why"), **be, tracked=st.get("tracked"), tracked_frac=st.get("tracked_frac"),
                              t_init_clip_s=st.get("t_init_clip_s"), cfg_over=st.get("cfg_over"),
                              config_used=st.get("config_used"), fps=st.get("fps"), size=st.get("size"),
                              clones=st.get("clones"), ms=ms_stats(pre))
            rows[key] = row
            print(key, {c: (('*' if v['div'] else '') + str(None if v['ate_w'] is None else round(v['ate_w'], 2)))
                        for c, v in row.items() if c in ALL}, flush=True)
    use = [x for x in rows.values() if x["rec"] != DROP and all(c in x for c in CONDS)]

    def summ(c, cells):
        ok = [x for x in cells if not x[c]["div"]]

        def v(f):
            vals = [f(x[c]) for x in ok if f(x[c]) is not None]
            return round(float(np.median(vals)), 4) if vals else None
        return dict(div=sum(x[c]["div"] for x in cells), div_cells=[x["key"] for x in cells if x[c]["div"]],
                    no_init=[x["key"] for x in cells if not x[c]["tracked"]], n=len(cells), n_normal=len(ok),
                    ate_w_med=v(lambda y: y["ate_w"]), sim3_med=v(lambda y: y.get("ate_sim3_cm")),
                    scale_med=v(lambda y: y.get("scale")),
                    ms_med_of_cells=round(float(np.median([x[c]["ms"]["med"] for x in cells])), 2),
                    ms_p95_of_cells=round(float(np.median([x[c]["ms"]["p95"] for x in cells])), 2))

    def paired(a, b, cells):
        both = [x for x in cells if not x[a]["div"] and not x[b]["div"]]
        d = [x[b]["ate_w"] - x[a]["ate_w"] for x in both]
        return dict(n_both_normal=len(both), d_med=round(float(np.median(d)), 4) if d else None,
                    d_mean=round(float(np.mean(d)), 4) if d else None, n_b_better=int(sum(v < 0 for v in d)),
                    med_a=round(float(np.median([x[a]["ate_w"] for x in both])), 4) if both else None,
                    med_b=round(float(np.median([x[b]["ate_w"] for x in both])), 4) if both else None,
                    worse_03=[(x["key"], round(x[a]["ate_w"], 2), round(x[b]["ate_w"], 2)) for x in both
                              if x[b]["ate_w"] - x[a]["ate_w"] > 0.3],
                    better_03=[(x["key"], round(x[a]["ate_w"], 2), round(x[b]["ate_w"], 2)) for x in both
                               if x[a]["ate_w"] - x[b]["ate_w"] > 0.3])
    S = dict(per_config={c: summ(c, use) for c in CONDS},
             per_offset={o: {c: summ(c, [x for x in use if x["off"] == o]) for c in CONDS} for o in OFFS},
             paired_vs_C1={c: paired("C1", c, use) for c in CONDS if c != "C1"})
    aud = {k: {c: dict(div=v[c]["div"], ate_w=v[c]["ate_w"], tracked=v[c]["tracked"]) for c in AUD if c in v}
           for k, v in rows.items() if any(c in v for c in AUD)}
    loops = {}
    for r in STD:
        for c in ALL:
            pre = pre_of(f"std{r[-12:-6]}", c)
            if Path(str(pre) + ".json").exists():
                loops[f"{r[-12:-6]}_{c}"] = dict(loop_eval(Path(str(pre) + ".json")), ms=ms_stats(pre), src=str(pre))
    b165 = {}
    if Path("docs/std60_board_check_20261024.md").exists():
        for c in ALL:
            for o in OFFS:
                pre = OUT / f"b165506_o{o}_{c}"
                if Path(str(pre) + ".json").exists():
                    st = json.load(open(str(pre) + ".json"))
                    mc = gs.cam_metrics(pre, STD165, "imu_gt_spl")
                    be = gs.bench_eval(pre, STD165, "imu_gt_spl")
                    b165[f"o{o}_{c}"] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), rel5_max=mc.get("rel5_max"),
                                             **be, tracked=st.get("tracked"), t_init_clip_s=st.get("t_init_clip_s"),
                                             ms=ms_stats(pre))
    timing = {}
    for n in names("timing"):
        p = OUT / "timing" / n
        if Path(str(p) + ".json").exists():
            st = json.load(open(str(p) + ".json"))
            timing[n] = dict(ms_stats(p), load=st.get("f30_load"), frames_fed=st.get("frames_fed"), wall_s=st.get("wall_s"))
    tsum = {}
    for c in CONDS:
        ms = [m for n, m in timing.items() if n.endswith(f"_{c}__T")]
        if ms:
            tsum[c] = dict(med_of_med=round(float(np.median([m["med"] for m in ms])), 2),
                           med_of_p95=round(float(np.median([m["p95"] for m in ms])), 2),
                           max_p95=max(m["p95"] for m in ms),
                           trk_med_of_med=round(float(np.median([m["trk_med"] for m in ms if m["trk_med"]])), 2),
                           trk_med_of_p95=round(float(np.median([m["trk_p95"] for m in ms if m["trk_p95"]])), 2))
    res = dict(made=time.strftime("%F %T"), plan="docs/fps30_plan.md", gt=GT, excluded=DROP, conds=ALL, cfg_over=CFG,
               summary=S, audit_init=aud, loops=loops, std165506=b165 or "skipped (no docs/std60_board_check_20261024.md)",
               timing=timing, timing_summary=tsum, rows=rows)
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(dict(S, audit_init=aud, timing_summary=tsum), ensure_ascii=False, indent=1))
    print(json.dumps({k: {x: v.get(x) for x in ("loop_err_cm", "path_m", "tracked", "n_slam_motion_med", "ms")}
                      for k, v in loops.items()}, ensure_ascii=False, indent=1))
    print(json.dumps(b165, ensure_ascii=False, indent=1))


def main():
    cmd = sys.argv[1]
    OUT.mkdir(parents=True, exist_ok=True)
    if cmd == "job":
        return job(sys.argv[2])
    if cmd == "run":
        return run_sched(sys.argv[2])
    score()


if __name__ == "__main__":
    main()
