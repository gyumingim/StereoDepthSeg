#!/usr/bin/env python3
"""Lens switch A/B, 2026-10-23 (docs/lens_switch_20261023.md). Reporting only: the user already decided to switch the
default 2.22-dpt lens to config/lens/hs_1920x1080_f2.22_cal20261022.json.

Setup = runs/imucal C4 (config/mono + try_zupt=false = config/base_nozupt, stock bridge ov_bridge/build, clones 11, strict
init, init window 1.0, disparity 15, frame cache, opencv threads 2, gyro_rebias False, lever arm via bench
--lever config/imu_cam_lever.json = C4 inputs), o0 only, 15 recordings, one run per cell. Only the lens file differs:
  old = config/lens/hs_1920x1080_f2.22_cal20261019_old.json (fx 1267.1, rational; = the former hs_1920x1080_f2.22.json)
  new = config/lens/hs_1920x1080_f2.22_cal20261022.json      (fx 1303.9, k1k2p_k3)
Both passed explicitly (vio_bench_hs --lens-file). Identity check: every old cell vs runs/imucal/<key>_C4 poses.
Score = window ATE / divergence on the CAMERA position (same as imucal_run.cam_metrics), Sim3 scale = bench 'scale'.
GT = imu_gt_spl as it is (built with the OLD K, see doc).

  (setsid nohup ../venv_ffs/bin/python imu_learn/lens_switch_ab.py run > runs/lens_switch/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/lens_switch_ab.py score     # -> runs/lens_switch/score.json
"""
import argparse
import gzip
import hashlib
import json
import os
import shutil
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
OUT = Path("runs/lens_switch")
LEVER = "config/imu_cam_lever.json"     # bench --lever (adopted 2026-10-23); = imucal C4 JSON path (levercheck identical)
REF = Path("runs/imucal")
PAR = int(os.environ.get("PAR", 12))
LENS = {"old": Path("config/lens/hs_1920x1080_f2.22_cal20261019_old.json"),
        "new": Path("config/lens/hs_1920x1080_f2.22_cal20261022.json")}


def jobs():
    return [dict(rec=r, cond=c, name=f"{r[-6:]}_o0_{c}") for r in RAW + OLD for c in ("old", "new")]


def replay(rec, cond):
    import vio_bench_hs as vb
    raw = Path(rec).name not in OLD
    vb.TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
    a = argparse.Namespace(rec=str(rec), fps=60.0, size="640x360", npts=200, clones=11, noise="cfg",
                           noise_scale=1.0, imu_calib=None, lever=LEVER,
                           dropout="", max_secs=0.0, imu_raw=raw,
                           gt="auto", out=None, init_cfg="strict", start_off=0.0, decode_threads=2,
                           imu_lead=2.0, init_window=1.0, init_disp=15.0, frame_cache=True, gyro_rebias=False,
                           cfg_over="num_opencv_threads=2,try_zupt=false", still_hold=None,
                           lens_file=str(LENS[cond]))
    saved = {}
    orig_ev = vb.evaluate

    def ev(rec_, a_, poses, *rest):
        saved["poses"] = poses
        return orig_ev(rec_, a_, poses, *rest)
    vb.evaluate = ev
    try:
        r = vb.run(a)
    finally:
        vb.evaluate = orig_ev
    r.update(ls_cond=cond, ls_lever=LEVER)
    return r, saved.get("poses", [])


def job(name):
    j = next(x for x in jobs() if x["name"] == name)
    t0 = time.time()
    r, P = replay(f"recordings/{j['rec']}", j["cond"])
    r.update(ls_made=time.strftime("%F %T"), ls_job_s=round(time.time() - t0, 1))
    with gzip.open(str(OUT / name) + ".poses.jsonl.gz", "wt") as f:
        for m in P:
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched():
    q = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists()]
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {len(q)} jobs PAR {PAR}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < PAR:
            j = q.pop(0)
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/lens_switch_ab.py", "job", j["name"]],
                                                   stdout=subprocess.DEVNULL,
                                                   stderr=open(OUT / f"{j['name']}.err", "w")), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end wall {time.time() - t_all:.0f} s", flush=True)


def load_poses(pre):
    return [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]


def strip(P):
    return [{k: v for k, v in m.items() if k not in ("ms", "wall_ms")} for m in P]


def cam_metrics(pre, rec, zs):
    """= imucal_run.cam_metrics (camera position T_world_cam0 instead of 'p'), temp files under runs/lens_switch/campos."""
    tmp = OUT / "campos"
    tmp.mkdir(exist_ok=True)
    dst = tmp / Path(str(pre)).name
    shutil.copy(str(pre) + ".json", str(dst) + ".json")
    with gzip.open(str(dst) + ".poses.jsonl.gz", "wt") as f:
        for m in load_poses(pre):
            if m.get("T_world_cam0"):
                m["p"] = list(np.asarray(m["T_world_cam0"], float).reshape(4, 4)[:3, 3])
            f.write(json.dumps(m) + "\n")
    return zs.run_metrics(dst, rec)


def score():
    import zupt_ob_score as zs
    os.chdir(SCAN3D)
    rows = {}
    for r in RAW + OLD:
        key = f"{r[-6:]}_o0"
        row = dict(rec=r, key=key, old_imu=r in OLD)
        for c in ("old", "new"):
            pre = OUT / f"{key}_{c}"
            d = json.load(open(str(pre) + ".json")) if Path(str(pre) + ".json").exists() else {}
            if d.get("ate_cm") is None:
                row[c] = dict(missing=not d, div=True)
                continue
            mc = cam_metrics(pre, r, zs)
            row[c] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), rel5_max=mc.get("rel5_max"),
                          ate_cm=d.get("ate_cm"), ate_sim3_cm=d.get("ate_sim3_cm"), scale=d.get("scale"),
                          t_init=d.get("t_init_clip_s"), lens_file=d.get("lens_file"), lens_sha256=d.get("lens_sha256"),
                          cache_hit=(d.get("frame_cache") or {}).get("hit"))
        ref = REF / f"{key}_C4"
        if Path(str(ref) + ".poses.jsonl.gz").exists() and Path(str(OUT / f"{key}_old") + ".poses.jsonl.gz").exists():
            row["old_identical_to_imucal_C4"] = strip(load_poses(OUT / f"{key}_old")) == strip(load_poses(ref))
        rows[key] = row
    use = [x for x in rows.values() if x["rec"] != DROP]
    both = [x for x in use if not x["old"]["div"] and not x["new"]["div"]]

    def med(c, f):
        v = [f(x[c]) for x in both if x[c].get("scale") is not None]
        return round(float(np.median(v)), 4) if v else None
    summ = dict(n_cells=len(use), div_old=sum(x["old"]["div"] for x in use), div_new=sum(x["new"]["div"] for x in use),
                n_both_normal=len(both),
                ate_w_med=dict(old=med("old", lambda y: y["ate_w"]), new=med("new", lambda y: y["ate_w"])),
                ate_w_d_med=round(float(np.median([x["new"]["ate_w"] - x["old"]["ate_w"] for x in both])), 3) if both else None,
                n_new_better=sum(x["new"]["ate_w"] < x["old"]["ate_w"] for x in both),
                sim3_ate_med=dict(old=med("old", lambda y: y["ate_sim3_cm"]), new=med("new", lambda y: y["ate_sim3_cm"])),
                scale_med=dict(old=med("old", lambda y: y["scale"]), new=med("new", lambda y: y["scale"])),
                abs_scale_err_med=dict(old=med("old", lambda y: abs(y["scale"] - 1)),
                                       new=med("new", lambda y: abs(y["scale"] - 1))),
                identical_old_vs_imucal_C4=sum(bool(x.get("old_identical_to_imucal_C4")) for x in rows.values()))
    lens = {c: dict(path=str(p), sha256=hashlib.sha256(open(p, "rb").read()).hexdigest()) for c, p in LENS.items()}
    res = dict(made=time.strftime("%F %T"), doc="docs/lens_switch_20261023.md", lens=lens, setup="imucal C4, o0",
               excluded_from_summary=DROP, metric="window ATE / divergence on camera position; GT imu_gt_spl (old K)",
               summary=summ, rows=rows)
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(summ, ensure_ascii=False, indent=1))
    for k, x in sorted(rows.items()):
        print(k, " | ".join(f"{c} {'*' if x[c]['div'] else ''}{x[c].get('ate_w') and round(x[c]['ate_w'], 2)} "
                            f"sim3 {x[c].get('ate_sim3_cm')} s{x[c].get('scale')}" for c in ("old", "new")),
              "ident" if x.get("old_identical_to_imucal_C4") else "")


def main():
    cmd = sys.argv[1]
    OUT.mkdir(parents=True, exist_ok=True)
    if cmd == "job":
        return job(sys.argv[2])
    {"run": run_sched, "score": score}[cmd]()


if __name__ == "__main__":
    main()
