#!/usr/bin/env python3
"""std60 board check, 2026-10-24 (docs/std60_board_check_20261024.md).

Recording hs_20261024_165506_std60 (std 60 fps, 0.5 dpt, laptop ChArUco 5x3, still 5 s / move 20 s / still 5 s).
GT: runs/std60_board_20261024/gt.sh (imu_gt -> batch_sync -> imu_gt --rs -> spline_gt, std60 0.5 dpt lens).
VIO: final baseline = config/mono (F1 in template) + try_zupt=false, lever, bench default lens (std 0.5 dpt file),
     strict init window 1.0 / disparity 15, clones 11, frame cache, threads 2 (same as imu_learn/final_baseline_run.py).
     'aud' = same but init window 1.5 / disparity 2.30 (the setting that initialised in the audit).
Score: camera-position ruler on imu_gt_spl (lens_switch_gtscore.cam_metrics / bench_eval), plus n_slam in motion.

  STD60_REC=hs_..._std60 ../venv_ffs/bin/python imu_learn/std60_board_run.py run      # o0/o1/o2 x base/aud, parallel
  ../venv_ffs/bin/python imu_learn/std60_board_run.py score    # -> runs/std60_board_20261024/score.json
"""
import argparse
import csv
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

PY = "../venv_ffs/bin/python"
REC = os.environ.get("STD60_REC", "hs_20261024_165506_std60")
# v1 (square 60.3 mm, 165506 only) = files directly in runs/std60_board_20261024; v2 (60.40 mm) = per-recording folder
OUT = Path("runs/std60_board_20261024") / (REC.split("_")[2] if os.environ.get("STD60_V", "2") == "2" else "")
LEVER = "config/imu_cam_lever.json"
OFFS = (0, 1, 2)
INIT = {"base": (1.0, 15.0), "aud": (1.5, 2.30)}
# *A conditions: same init, but R_ci / dt = the ASSUMED std60 values of the no-board loop tests
# (config/phone_ov_main/variants.json: HS chordal mean R_ci, median dt -2.17 ms) instead of this recording's align.json.
# Their scores use the run-time evaluation (stored json, same GT, consistent dt), not bench_eval (which reads align.json).
CONDS = {"base": "base", "aud": "aud", "baseA": "base", "audA": "aud"}
CONDS_RUN = [c for c in CONDS if c in os.environ.get("STD60_CONDS", "base,aud").split(",")]
GT = "imu_gt_spl"


def jobs():
    return [dict(off=o, cond=c, name=f"o{o}_{c}") for c in CONDS_RUN for o in OFFS]


def assumed_align():
    v = json.load(open("config/phone_ov_main/variants.json"))
    return dict(dt_ns=int(np.median(v["dt_ns"])), R_ci=v["R_ci"], source="config/phone_ov_main/variants.json (assumed)")


def job(name):
    import vio_bench_hs as vb
    j = next(x for x in jobs() if x["name"] == name)
    vb.TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
    iw, idp = INIT[CONDS[j["cond"]]]
    if j["cond"].endswith("A"):                     # swap the align.json contents read by vb.run (this process only)
        _load, sub = vb.json.load, assumed_align()

        def load(f, *k, **kw):
            d = _load(f, *k, **kw)
            if str(getattr(f, "name", "")).endswith("imu_gt/align.json"):
                d = dict(d, dt_ns=sub["dt_ns"], R_ci=sub["R_ci"])
            return d
        vb.json.load = load
    a = argparse.Namespace(rec=f"recordings/{REC}", fps=60.0, size="640x360", npts=200, clones=11, noise="cfg",
                           noise_scale=1.0, imu_calib=None, lever=LEVER, lens_file=None, dropout="", max_secs=0.0,
                           imu_raw=True, gt="auto", out=None, init_cfg="strict", start_off=float(j["off"]),
                           decode_threads=2, imu_lead=2.0, init_window=iw, init_disp=idp, frame_cache=True,
                           gyro_rebias=False, cfg_over="num_opencv_threads=2,try_zupt=false", still_hold=None)
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
    r.update(sb_made=time.strftime("%F %T"), sb_job_s=round(time.time() - t0, 1), sb_runner="imu_learn/std60_board_run.py",
             sb_cond=j["cond"], sb_init=[iw, idp],
             sb_align=assumed_align() if j["cond"].endswith("A") else f"recordings/{REC}/imu_gt/align.json")
    with gzip.open(str(OUT / name) + ".poses.jsonl.gz", "wt") as f:
        for m in saved.get("poses", []):
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched():
    ps = {}
    for j in jobs():
        if (OUT / f"{j['name']}.json").exists():
            continue
        ps[j["name"]] = subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/std60_board_run.py", "job", j["name"]],
                                         stdout=subprocess.DEVNULL, stderr=open(OUT / f"{j['name']}.err", "w"))
    for n, p in ps.items():
        p.wait()
        print(time.strftime("%F %T"), n, "ok" if (OUT / f"{n}.json").exists() else "FAILED", flush=True)


def motion_window():
    g = [x for x in csv.DictReader(open(f"recordings/{REC}/imu.csv")) if x["type"] in ("16", "4")]
    typ = "16" if any(x["type"] == "16" for x in g) else "4"
    g = [x for x in g if x["type"] == typ]
    t = np.array([int(x["t_ns"]) for x in g], float)
    w = np.linalg.norm(np.array([[float(x["v0"]), float(x["v1"]), float(x["v2"])] for x in g]), axis=1)
    w -= np.median(w[: max(50, len(w) // 20)])
    mv = np.convolve(np.abs(w) > 0.08, np.ones(100), "same") > 0
    i = np.where(mv)[0]
    return (t[i[0]], t[i[-1]]) if len(i) else (t[0], t[-1])


def score():
    import lens_switch_gtscore as gs
    os.chdir(SCAN3D)
    gs.OUT = OUT
    t0m, t1m = motion_window()
    rows = {}
    for j in jobs():
        pre = OUT / j["name"]
        if not Path(str(pre) + ".json").exists():
            continue
        st = json.load(open(str(pre) + ".json"))
        mc = gs.cam_metrics(pre, REC, GT)
        be = (gs.bench_eval(pre, REC, GT) if not j["cond"].endswith("A") else
              ({k: st.get(k) for k in ("ate_cm", "ate_sim3_cm", "scale", "matched", "gt")} if st.get("ate_cm") is not None else {}))
        P = gs.load_poses(pre)
        tr = [m for m in P if m.get("status") == "tracking"]
        ns = np.array([m.get("n_slam", np.nan) for m in tr if t0m <= m["t_ns"] <= t1m], float) if tr and "t_ns" in tr[0] else np.array([])
        if not len(ns):   # fall back on the traj array (t_ns, x, y, z, n_slam)
            trj = np.array([x[:5] for x in st.get("traj", [])], float)
            ns = trj[(trj[:, 0] >= t0m) & (trj[:, 0] <= t1m), 4] if len(trj) else np.array([])
        rows[j["name"]] = dict(off=j["off"], cond=j["cond"], div=mc.get("divergent"), ate_w=mc.get("ate_w"),
                               rel5_max=mc.get("rel5_max"), why=mc.get("why"), **be, tracked_frac=st.get("tracked_frac"),
                               t_init_clip_s=st.get("t_init_clip_s"), status_counts=st.get("status_counts"),
                               n_slam_motion_med=float(np.median(ns)) if len(ns) else None,
                               n_slam_motion_frac0=round(float(np.mean(ns == 0)), 3) if len(ns) else None,
                               lens_file=st.get("lens_file"), cfg_over=st.get("cfg_over"))
        print(j["name"], {k: rows[j["name"]].get(k) for k in ("div", "ate_w", "ate_cm", "ate_sim3_cm", "scale", "tracked_frac",
                                                        "t_init_clip_s", "n_slam_motion_med", "n_slam_motion_frac0")}, flush=True)
    res = dict(made=time.strftime("%F %T"), rec=REC, gt=GT, motion_ns=[t0m, t1m], init=INIT, rows=rows)
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)


def main():
    cmd = sys.argv[1]
    OUT.mkdir(parents=True, exist_ok=True)
    if cmd == "job":
        return job(sys.argv[2])
    {"run": run_sched, "score": score}[cmd]()


if __name__ == "__main__":
    main()
