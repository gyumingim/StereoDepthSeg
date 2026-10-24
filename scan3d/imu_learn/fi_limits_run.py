#!/usr/bin/env python3
"""Feature-initializer limit test, 2026-10-24 (plan docs/fi_limits_plan.md, result docs/fi_limits_result.md).

Audit finding (std60 room-scale loops): OpenVINS fi_max_baseline (40) and fi_max_cond_number (10000) rejected all new
features during hand motion when the scene is > ~1 m away; relaxing both fixed the loop error. Here:
  base = final baseline (imu_learn/final_baseline_run.py: config/mono + try_zupt=false, lever, new lens, strict init
         window 1.0 / disparity 15, clones 11, frame cache, no gyro rebias) but num_opencv_threads=THREADS (see plan)
  F1   = base + fi_max_cond_number=1e6, fi_max_baseline=200
  F2   = base + fi_max_cond_number=1e8, fi_max_baseline=1000
on the 15 HS recordings x o0/o1/o2 (scored on the new-K GT with the camera-position ruler = final_baseline score) and on
the std60 loop recordings (no GT, loop error = first tracked pose vs pose 0.5 s after the motion ends).

  (setsid nohup ../venv_ffs/bin/python imu_learn/fi_limits_run.py run hs > runs/fi_limits/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/fi_limits_run.py run std
  ../venv_ffs/bin/python imu_learn/fi_limits_run.py job 133820_o0_base [rep tag via NAME__tag]
  ../venv_ffs/bin/python imu_learn/fi_limits_run.py score      # -> runs/fi_limits/score.json
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
from phone60_run import RAW, OLD, DROP  # noqa: E402

PY = "../venv_ffs/bin/python"
OUT = Path("runs/fi_limits")
PAR = int(os.environ.get("PAR", 12))
THREADS = os.environ.get("FI_THREADS", "1")
LEVER = "config/imu_cam_lever.json"
OFFS = (0, 1, 2)
STD = ["hs_20261024_142148_std60", "hs_20261024_142610_std60", "hs_20261024_151646_std60"]
FI = {"base": "", "F1": "fi_max_cond_number=1e6,fi_max_baseline=200", "F2": "fi_max_cond_number=1e8,fi_max_baseline=1000"}
GT = "imu_gt_spl_cal1022"


def cfg_over(cond, threads):
    s = f"num_opencv_threads={threads},try_zupt=false"
    return s + ("," + FI[cond] if FI[cond] else "")


def jobs(which="hs"):
    js = []
    if which in ("hs", "all"):
        js += [dict(rec=r, off=o, cond=c, gt="auto", name=f"{r[-6:]}_o{o}_{c}") for c in FI for r in RAW + OLD for o in OFFS]
    if which in ("std", "all"):
        js += [dict(rec=r, off=0, cond=c, gt="none", name=f"std{r[-12:-6]}_{c}") for c in FI for r in STD]
    return js


def job(name):
    """NAME = <job name>[__<tag>]; tag 'T2' = num_opencv_threads 2 (bit-identity check vs runs/final_baseline)."""
    import vio_bench_hs as vb
    base, _, tag = name.partition("__")
    j = next(x for x in jobs("all") if x["name"] == base)
    threads = "2" if tag.startswith("T2") else THREADS
    vb.TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
    a = argparse.Namespace(rec=f"recordings/{j['rec']}", fps=60.0, size="640x360", npts=200, clones=11, noise="cfg",
                           noise_scale=1.0, imu_calib=None, lever=LEVER, lens_file=None, dropout="", max_secs=0.0,
                           imu_raw=j["rec"] not in OLD, gt=j["gt"], out=None, init_cfg="strict", start_off=float(j["off"]),
                           decode_threads=2, imu_lead=2.0, init_window=1.0, init_disp=15.0, frame_cache=True,
                           gyro_rebias=False, cfg_over=cfg_over(j["cond"], threads), still_hold=None)
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
    r.update(fi_made=time.strftime("%F %T"), fi_job_s=round(time.time() - t0, 1), fi_runner="imu_learn/fi_limits_run.py",
             fi_cond=j["cond"], fi_threads=threads)
    with gzip.open(str(OUT / name) + ".poses.jsonl.gz", "wt") as f:
        for m in saved.get("poses", []):
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched(which):
    q = [j for j in jobs(which) if not (OUT / f"{j['name']}.json").exists()]
    q.sort(key=lambda j: j["rec"] not in ("hs_20261020_224323",))   # slowest (divergent) first
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {len(q)} jobs PAR {PAR} threads {THREADS}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < PAR:
            j = q.pop(0)
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/fi_limits_run.py", "job", j["name"]],
                                                   stdout=subprocess.DEVNULL,
                                                   stderr=open(OUT / f"{j['name']}.err", "w")), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end wall {time.time() - t_all:.0f} s", flush=True)


def loop_eval(res_json):
    """Scratchpad loop_eval.py logic: start = first tracked pose; end = mean of +-10 poses around (motion end + 0.5 s),
    motion end = last |gyro| - bias > 0.08 rad/s sample (0.2 s windows); path = up to that pose (every 6th pose)."""
    r = json.load(open(res_json))
    tr = np.array([x[:4] for x in r.get("traj", [])], float)
    out = dict(rec=r["rec"], tracked=r.get("tracked"), status_counts=r.get("status_counts"), t_init_clip_s=r.get("t_init_clip_s"))
    if len(tr) < 30:
        return dict(out, loop_err_cm=None, note="no tracking")
    g = [x for x in csv.DictReader(open(f"recordings/{r['rec']}/imu.csv")) if x["type"] == "16"]
    gt = np.array([int(x["t_ns"]) for x in g], float)
    gw = np.linalg.norm(np.array([[float(x["v0"]), float(x["v1"]), float(x["v2"])] for x in g]), axis=1)
    gw -= np.median(gw[: max(50, len(gw) // 20)])
    moving = np.convolve(np.abs(gw) > 0.08, np.ones(100), "same") > 0
    t_end = gt[np.where(moving)[0][-1]] if moving.any() else gt[-1]
    tv, p = tr[:, 0], tr[:, 1:]
    i = min(int(np.searchsorted(tv, t_end + 0.5e9)), len(p) - 1)
    put = p[max(0, i - 10): i + 10].mean(0)
    L = float(np.linalg.norm(np.diff(p[: i + 1: 6], axis=0), axis=1).sum())
    e = float(np.linalg.norm(put - p[0]))
    n_slam = np.array([x[4] for x in r["traj"]], float)
    mid = (tv > tv[0] + 3e9) & (tv < t_end)
    return dict(out, loop_err_cm=round(e * 100, 2), loop_vec_cm=np.round((put - p[0]) * 100, 2).tolist(), path_m=round(L, 3),
                loop_pct=round(e / max(L, 1e-6) * 100, 3), drift_final_still_cm=round(float(np.linalg.norm(p[-30:].mean(0) - put)) * 100, 2),
                n_slam_motion_med=float(np.median(n_slam[mid])) if mid.any() else None,
                n_slam_motion_frac0=round(float(np.mean(n_slam[mid] == 0)), 3) if mid.any() else None)


def score():
    import lens_switch_gtscore as gs
    os.chdir(SCAN3D)
    gs.OUT = OUT
    rows = {}
    for r in RAW + OLD:
        for o in OFFS:
            key = f"{r[-6:]}_o{o}"
            row = dict(rec=r, key=key, off=o)
            for c in FI:
                pre = OUT / f"{key}_{c}"
                if not Path(str(pre) + ".json").exists():
                    row[c] = None
                    continue
                mc = gs.cam_metrics(pre, r, GT)
                be = gs.bench_eval(pre, r, GT)
                st = json.load(open(str(pre) + ".json"))
                row[c] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), rel5_max=mc.get("rel5_max"), **be,
                              cfg_over=st.get("cfg_over"), tracked_frac=st.get("tracked_frac"), t_init_clip_s=st.get("t_init_clip_s"))
            rows[key] = row
            print(key, {c: (('*' if v['div'] else '') + str(round(v['ate_w'], 2)) if v and v['ate_w'] is not None else v and 'None')
                        for c, v in row.items() if c in FI}, flush=True)
    use = [x for x in rows.values() if x["rec"] != DROP and all(x[c] for c in FI)]

    def summ(c, cells):
        ok = [x for x in cells if not x[c]["div"]]
        v = lambda f: round(float(np.median([f(x[c]) for x in ok])), 4) if ok else None  # noqa: E731
        return dict(div=sum(x[c]["div"] for x in cells), div_cells=[x["key"] for x in cells if x[c]["div"]], n=len(cells),
                    n_normal=len(ok), ate_w_med=v(lambda y: y["ate_w"]), sim3_med=v(lambda y: y["ate_sim3_cm"]),
                    scale_med=v(lambda y: y["scale"]))

    def paired(a, b, cells):
        both = [x for x in cells if not x[a]["div"] and not x[b]["div"]]
        d = [x[b]["ate_w"] - x[a]["ate_w"] for x in both]
        return dict(n_both_normal=len(both), d_med=round(float(np.median(d)), 4) if d else None,
                    d_mean=round(float(np.mean(d)), 4) if d else None, n_b_better=int(sum(v < 0 for v in d)),
                    n_equal=int(sum(v == 0 for v in d)),
                    med_a=round(float(np.median([x[a]["ate_w"] for x in both])), 4) if both else None,
                    med_b=round(float(np.median([x[b]["ate_w"] for x in both])), 4) if both else None)
    S = dict(per_config={c: summ(c, use) for c in FI},
             per_offset={o: {c: summ(c, [x for x in use if x["off"] == o]) for c in FI} for o in OFFS},
             paired_base_to_F1=paired("base", "F1", use), paired_base_to_F2=paired("base", "F2", use))
    loops = {}
    for r in STD:
        for c in FI:
            p = OUT / f"std{r[-12:-6]}_{c}.json"
            if p.exists():
                loops[f"{r}_{c}"] = loop_eval(p)
    res = dict(made=time.strftime("%F %T"), plan="docs/fi_limits_plan.md", gt=GT, excluded=DROP, threads=THREADS, conds=FI,
               summary=S, loops=loops, rows=rows)
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(S, ensure_ascii=False, indent=1))
    print(json.dumps(loops, ensure_ascii=False, indent=1))


def main():
    cmd = sys.argv[1]
    OUT.mkdir(parents=True, exist_ok=True)
    if cmd == "job":
        return job(sys.argv[2])
    if cmd == "run":
        return run_sched(sys.argv[2] if len(sys.argv) > 2 else "hs")
    score()


if __name__ == "__main__":
    main()
