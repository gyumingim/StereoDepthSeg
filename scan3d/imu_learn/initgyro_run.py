#!/usr/bin/env python3
"""Gyro stillness gate for static init: bit-check and pre-registered comparison (docs/initgyro_plan.md, 2026-10-23).

Bridge = stock ov_bridge/main.cpp built out of tree at ~/scanner/src/ovbuild_initgyro_bridge against
~/scanner/src/open_vins_initgyro (= ~/scanner/src/open_vins copy + tools/ov_initgyro_patch.py, diff tools/ov_patches/initgyro.diff),
library ~/scanner/src/ovbuild_initgyro. One new optional estimator key: init_max_gyro (rad/s, 0 = off).
Everything else = phone60_run V4 (config/mono + try_zupt=false, clones 11, strict init, --init-window 1.0, --init-disp 15,
frame cache, gyro_rebias off, num_opencv_threads 2). The key is appended to the written estimator_config.yaml.
User decision 2026-10-23: start offset o0 only (15 cells, one run each). Base = runs/phone60/<rec>_o0_V4 (reused).

  ../venv_ffs/bin/python imu_learn/initgyro_run.py bitcheck [KEY]      # init_max_gyro 0 vs stored V4, full length
  (setsid nohup ../venv_ffs/bin/python imu_learn/initgyro_run.py run > runs/initgyro/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/initgyro_run.py score              # -> runs/initgyro/score.json
"""
import argparse
import gzip
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
from phone60_run import RAW, OLD, DROP  # noqa: E402

PY = "../venv_ffs/bin/python"
# IG_TH / IG_OUT (env, inherited by job subprocesses): 2026-10-23 POST-HOC loose gate 0.25 -> runs/initgyro_025
# (docs/initgyro025_plan.md). Defaults = the first pre-registered run (0.05, runs/initgyro).
OUT = Path(os.environ.get("IG_OUT", "runs/initgyro"))
REF_DIR = Path("runs/phone60")
PAR = int(os.environ.get("PAR", 12))
BRIDGE = Path.home() / "scanner" / "src" / "ovbuild_initgyro_bridge" / "ov_bridge"
BITCHECK = Path("runs/initgyro/bitcheck.json")   # same bridge for every threshold
TH = float(os.environ.get("IG_TH", 0.05))  # rad/s; 0.05 fixed from runs/initgyro/thresh.json BEFORE the first comparison
CONDS = {"ig0": 0.0, "ig": TH}


def replay(rec, off, th):
    import vio_bench_hs as vb
    vb.TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
    vb.BRIDGE = BRIDGE
    orig_wc = vb.write_config

    def wc(d, *args, **kw):
        used = orig_wc(d, *args, **kw)
        with open(d / "estimator_config.yaml", "a") as f:
            f.write(f"init_max_gyro: {th:.6f}\n")
        used["init_max_gyro"] = f"{th:.6f}"
        return used
    vb.write_config = wc
    a = argparse.Namespace(rec=str(rec), fps=60.0, size="640x360", npts=200, clones=11, noise="cfg",
                           noise_scale=1.0, imu_calib=None, dropout="", max_secs=0.0, imu_raw=Path(rec).name not in OLD,
                           gt="auto", out=None, init_cfg="strict", start_off=float(off), decode_threads=2,
                           imu_lead=2.0, init_window=1.0, init_disp=15.0, frame_cache=True, gyro_rebias=False,
                           cfg_over="num_opencv_threads=2,try_zupt=false", still_hold=None)
    saved = {}
    orig_ev = vb.evaluate

    def ev(rec_, a_, poses, *rest):
        saved["poses"] = poses
        return orig_ev(rec_, a_, poses, *rest)
    vb.evaluate = ev
    try:
        r = vb.run(a)
    finally:
        vb.write_config, vb.evaluate = orig_wc, orig_ev
    r.update(ig_init_max_gyro=th, ig_bridge=str(BRIDGE))
    return r, saved.get("poses", [])


def strip(P):
    return [{k: v for k, v in m.items() if k not in ("ms", "wall_ms")} for m in P]


def load_poses(pre):
    return [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]


def bitcheck(key="131326_o0"):
    OUT.mkdir(parents=True, exist_ok=True)
    rec = next(r for r in RAW + OLD if r.endswith(key.split("_")[0]))
    r, P = replay(f"recordings/{rec}", int(key.split("_o")[1]), 0.0)
    V = load_poses(REF_DIR / f"{key}_V4")
    same = strip(P) == strip(V)
    res = dict(made=time.strftime("%F %T"), key=key, ref=f"{REF_DIR}/{key}_V4", n_poses=[len(P), len(V)], identical=same,
               ate=[r.get("ate_cm"), json.load(open(REF_DIR / f"{key}_V4.json")).get("ate_cm")],
               config_used=r.get("config_used"), keys_ignored=["ms", "wall_ms"], bridge=str(BRIDGE))
    if not same:
        res["first_diff"] = next((i for i, (x, y) in enumerate(zip(strip(P), strip(V))) if x != y), None)
    json.dump(res, open(OUT / "bitcheck.json", "w"), indent=1)
    print(json.dumps(res, indent=1))


def jobs():
    return [dict(rec=r, off=0, cond="ig", name=f"{r[-6:]}_o0_ig") for r in RAW + OLD]


def job(name):
    j = next(x for x in jobs() if x["name"] == name)
    t0 = time.time()
    r, P = replay(f"recordings/{j['rec']}", j["off"], CONDS[j["cond"]])
    r.update(ig_cond=j["cond"], ig_made=time.strftime("%F %T"), ig_job_s=round(time.time() - t0, 1))
    with gzip.open(str(OUT / name) + ".poses.jsonl.gz", "wt") as f:
        for m in P:
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched():
    dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no", "imu_learn/vio_bench_hs.py",
                            "imu_learn/initgyro_run.py", "config/mono", "ov_bridge/main.cpp", "tools/ov_patches/initgyro.diff",
                            "docs/initgyro_plan.md", "docs/initgyro025_plan.md"], capture_output=True, text=True).stdout
    if dirty.strip():
        sys.exit(f"REFUSED: uncommitted {dirty}")
    if not BITCHECK.exists() or not json.load(open(BITCHECK)).get("identical"):
        sys.exit("REFUSED: bitcheck missing or not identical")
    q = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists()]
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {len(q)} jobs PAR {PAR} TH {TH}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < PAR:
            j = q.pop(0)
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/initgyro_run.py", "job", j["name"]],
                                                   stdout=subprocess.DEVNULL,
                                                   stderr=open(OUT / f"{j['name']}.err", "w")), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end wall {time.time() - t_all:.0f} s", flush=True)


def first_tracking(P):
    return next((m for m in P if m.get("status") == "tracking"), None)


def score():
    import numpy as np
    import zupt_ob_score as zs
    os.chdir(SCAN3D)
    runs = {}
    for j in jobs():
        key = j["name"].rsplit("_", 1)[0]
        row = dict(rec=j["rec"], key=key)
        for c, pre in (("base", REF_DIR / f"{key}_V4"), ("ig", OUT / j["name"])):
            d = json.load(open(str(pre) + ".json")) if Path(str(pre) + ".json").exists() else {}
            m = zs.run_metrics(pre, j["rec"]) if d else dict(divergent=True, why="missing")
            ft = first_tracking(load_poses(pre)) if d else None
            row[c] = dict(div=bool(m.get("divergent")), ate_w=m.get("ate_w"), rel5_max=m.get("rel5_max"),
                          t_init=d.get("t_init_clip_s"), covered=d.get("covered_frac"),
                          bg0=[round(x, 4) for x in ft["bg"]] if ft and ft.get("bg") else None,
                          bg0_norm=round(float(np.linalg.norm(ft["bg"])), 4) if ft and ft.get("bg") else None,
                          missing=not d)
        tb, ti = row["base"]["t_init"], row["ig"]["t_init"]
        row["init_delay_s"] = round(ti - tb, 3) if tb is not None and ti is not None else None
        runs[key] = row
    use = [r for r in runs.values() if r["rec"] != DROP]
    nb = sum(r["base"]["div"] for r in use)
    ni = sum(r["ig"]["div"] for r in use)
    normal = [r for r in use if not r["base"]["div"] and not r["ig"]["div"]]
    mb = float(np.median([r["base"]["ate_w"] for r in normal]))
    mi = float(np.median([r["ig"]["ate_w"] for r in normal]))
    c131 = runs["131326_o0"]["ig"]["div"]
    crit = dict(a=dict(div_base=nb, div_ig=ni, d131326_o0_ig=c131, pass_=ni <= nb and not c131),
                b=dict(n_normal=len(normal), med_base=round(mb, 3), med_ig=round(mi, 3), limit=round(1.2 * mb + 0.3, 3),
                       pass_=mi <= 1.2 * mb + 0.3))
    delays = [r["init_delay_s"] for r in use if r["init_delay_s"] is not None]
    res = dict(made=time.strftime("%F %T"), th=TH, base=f"{REF_DIR}/<key>_V4", runs=runs, criteria=crit,
               verdict="통과" if crit["a"]["pass_"] and crit["b"]["pass_"] else "불통과",
               init_delay=dict(n=len(delays), median=float(np.median(delays)) if delays else None,
                               max=max(delays) if delays else None, n_later=sum(d > 0.05 for d in delays)),
               missing=[k for k, r in runs.items() if r["ig"]["missing"]])
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(dict(criteria=crit, verdict=res["verdict"], init_delay=res["init_delay"], missing=res["missing"]),
                     ensure_ascii=False, indent=1))
    for k, r in sorted(runs.items()):
        print(k, "base", r["base"]["div"], r["base"]["ate_w"] and round(r["base"]["ate_w"], 2), r["base"]["t_init"],
              r["base"]["bg0_norm"], "| ig", r["ig"]["div"], r["ig"]["ate_w"] and round(r["ig"]["ate_w"], 2),
              r["ig"]["t_init"], r["ig"]["bg0_norm"], "delay", r["init_delay_s"])


def main():
    cmd = sys.argv[1]
    OUT.mkdir(parents=True, exist_ok=True)
    if cmd == "job":
        return job(sys.argv[2])
    if cmd == "bitcheck":
        return bitcheck(*sys.argv[2:3])
    {"run": run_sched, "score": score}[cmd]()


if __name__ == "__main__":
    main()
