#!/usr/bin/env python3
"""phone60 vs base (docs/phone60_plan.md, 2026-10-23). 15 recordings x offsets 0/1/2, stock bridge (ov_bridge/build),
--frame-cache, gyro_rebias False explicit, num_opencv_threads 2, still-hold off. Only the template differs:
  base    = config/mono    + --clones 11
  phone60 = config/phone60 + --clones 20
Init values are the same for both (strict, --init-window 1.0, --init-disp 15; the bench writes them over the template).

  ../venv_ffs/bin/python imu_learn/phone60_run.py cache       # build frame caches (once)
  (setsid nohup ../venv_ffs/bin/python imu_learn/phone60_run.py run > runs/phone60/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/phone60_run.py score       # -> runs/phone60/score.json
  CONDS=V1,V2,V3 ... run / CONDS=base,phone60,V1,V2,V3 ... score  # ablations -> score_ablation.json
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

PY = "../venv_ffs/bin/python"
OUT = Path("runs/phone60")
PAR = int(os.environ.get("PAR", 12))
RAW = ["hs_20261022_133820", "hs_20261022_134550", "hs_20261022_134204",
       "hs_20261021_214550", "hs_20261021_213341", "hs_20261021_213955", "hs_20261021_215348",
       "hs_20261020_214138", "hs_20261020_221052", "hs_20261020_221925", "hs_20261020_222747", "hs_20261020_223815",
       "hs_20261020_224323"]
OLD = ["hs_20261019_131326", "hs_20261019_154320"]          # no raw IMU (types 16/35) -> calibrated types 1/4
DROP = "hs_20261021_213955"                                  # phone drop: reported separately
CONDS = {"base": ("config/mono", 11, ""), "phone60": ("config/phone60", 20, ""),
         # ablations (docs/phone60_ablation_plan.md): one change each
         "V1": ("config/phone60", 20, "calib_cam_intrinsics=false"),
         "V2": ("config/mono", 11, "zupt_only_at_beginning=true"),
         "V3": ("config/mono", 11, "calib_cam_intrinsics=true"),
         "V4": ("config/mono", 11, "try_zupt=false"),
         "zob": ("config/base_zob", 11, "")}   # = V2 via the template file (identity check)
ACTIVE = os.environ.get("CONDS", "base,phone60").split(",")


def cells():
    return [dict(rec=r, off=o, key=f"{r[-6:]}_o{o}") for r in RAW + OLD for o in (0, 1, 2)]


def jobs():
    return [dict(c, cond=k, name=f"{c['key']}_{k}") for c in cells() for k in CONDS if k in ACTIVE]


def job(name):
    import vio_bench_hs as vb
    j = next(x for x in jobs() if x["name"] == name)
    tdir, clones, over = CONDS[j["cond"]]
    vb.TEMPLATE = SCAN3D / tdir / "estimator_config.yaml"
    a = argparse.Namespace(rec=f"recordings/{j['rec']}", fps=60.0, size="640x360", npts=200, clones=clones, noise="cfg",
                           noise_scale=1.0, imu_calib=None, dropout="", max_secs=0.0, imu_raw=j["rec"] not in OLD,
                           gt="auto", out=None, init_cfg="strict", start_off=float(j["off"]), decode_threads=2,
                           imu_lead=2.0, init_window=1.0, init_disp=15.0, frame_cache=True, gyro_rebias=False,
                           cfg_over="num_opencv_threads=2" + ("," + over if over else ""), still_hold=None)
    saved = {}
    orig = vb.evaluate

    def ev(rec, a_, poses, *rest):
        saved["poses"] = poses
        return orig(rec, a_, poses, *rest)
    vb.evaluate = ev
    t0 = time.time()
    r = vb.run(a)
    P = saved.get("poses", [])
    intr = [m["intr0"] for m in P if m.get("intr0")]
    r.update(p60_cond=j["cond"], p60_template=tdir, p60_over=over, p60_made=time.strftime("%F %T"), p60_job_s=round(time.time() - t0, 1),
             p60_template_blobs={f: subprocess.run(["git", "hash-object", f"{tdir}/{f}"], capture_output=True,
                                                   text=True).stdout.strip()
                                 for f in ("estimator_config.yaml", "kalibr_imu_chain.yaml")},
             intr0_first=intr[0] if intr else None, intr0_last=intr[-1] if intr else None)
    with gzip.open(str(OUT / name) + ".poses.jsonl.gz", "wt") as f:
        for m in P:
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def build_cache():
    ps = []
    for r in RAW + OLD:
        ps.append(subprocess.Popen([PY, "imu_learn/frame_cache.py", "build", f"recordings/{r}", "--offsets", "0,1,2",
                                    "--max-secs", "0", "--threads", "2"],
                                   stdout=open(OUT / f"cache_{r[-6:]}.log", "w"), stderr=subprocess.STDOUT))
    t0 = time.time()
    for p in ps:
        p.wait()
    print(f"cache built {time.time() - t0:.0f} s, rc {[p.returncode for p in ps]}")


def run_sched():
    dirty = subprocess.run(["git", "status", "--porcelain", "imu_learn/vio_bench_hs.py", "imu_learn/phone60_run.py",
                            "config/mono", "config/phone60", "ov_bridge/main.cpp"], capture_output=True, text=True).stdout
    if dirty.strip():
        sys.exit(f"REFUSED: uncommitted {dirty}")
    q = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists()]
    t_all, running = time.time(), {}
    print(f"{time.strftime('%F %T')} start {len(q)} jobs PAR {PAR}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < PAR:
            j = q.pop(0)
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/phone60_run.py", "job", j["name"]],
                                                   stdout=subprocess.DEVNULL,
                                                   stderr=open(OUT / f"{j['name']}.err", "w")), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end wall {time.time() - t_all:.0f} s", flush=True)


def score():
    import numpy as np
    import zupt_ob_score as zs
    os.chdir(SCAN3D)
    runs = {}
    for j in jobs():
        pre = OUT / j["name"]
        d = json.load(open(str(pre) + ".json")) if Path(str(pre) + ".json").exists() else {}
        m = zs.run_metrics(pre, j["rec"]) if d else dict(result=False, divergent=True, why="missing")
        dts = [d.get("dt_cam_imu_final_ms")]
        runs[j["name"]] = dict(rec=j["rec"], key=j["key"], cond=j["cond"], div=m.get("divergent"),
                               ate_w=m.get("ate_w"), rel5_max=m.get("rel5_max"), ate_cm=d.get("ate_cm"),
                               scale=d.get("scale"), t_init=d.get("t_init_clip_s"), job_s=d.get("p60_job_s"),
                               wall_s=d.get("wall_s"), ms_med=d.get("ms_med"), dt_final_ms=dts[0],
                               intr_first=d.get("intr0_first"), intr_last=d.get("intr0_last"),
                               cache_hit=(d.get("frame_cache") or {}).get("hit"))
    res = dict(made=time.strftime("%F %T"), runs=runs, conds={})
    for c in ACTIVE:
        R = [r for r in runs.values() if r["cond"] == c]
        res["conds"][c] = dict(n_div_excl_drop=sum(bool(r["div"]) for r in R if r["rec"] != DROP),
                               n_div_drop=sum(bool(r["div"]) for r in R if r["rec"] == DROP),
                               job_s_med=float(np.median([r["job_s"] for r in R if r["job_s"]])))
    res["criteria"], res["verdict"] = {}, {}
    keys = sorted({r["key"] for r in runs.values()})
    for c in ACTIVE:                                   # each condition vs base, same two criteria (plan)
        if c == "base":
            continue
        normal = [k for k in keys if not runs[f"{k}_base"]["div"] and not runs[f"{k}_{c}"]["div"] and DROP[-6:] not in k]
        mb = float(np.median([runs[f"{k}_base"]["ate_w"] for k in normal]))
        mc = float(np.median([runs[f"{k}_{c}"]["ate_w"] for k in normal]))
        nb, nc = res["conds"]["base"]["n_div_excl_drop"], res["conds"][c]["n_div_excl_drop"]
        res["criteria"][c] = dict(a_div=dict(base=nb, cond=nc, pass_=nc <= nb),
                                  b_ate=dict(n_normal=len(normal), med_base=round(mb, 2), med_cond=round(mc, 2),
                                             limit=round(1.2 * mb + 0.3, 2), pass_=mc <= 1.2 * mb + 0.3))
        ok = res["criteria"][c]["a_div"]["pass_"] and res["criteria"][c]["b_ate"]["pass_"]
        res["verdict"][c] = "통과" if ok else "불통과"
    name = "score.json" if ACTIVE == ["base", "phone60"] else "score_ablation.json"
    json.dump(res, open(OUT / name, "w"), indent=1, ensure_ascii=False)
    print(json.dumps(dict(conds=res["conds"], criteria=res["criteria"], verdict=res["verdict"]), ensure_ascii=False, indent=1))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if sys.argv[1] == "job":
        return job(sys.argv[2])
    {"cache": build_cache, "run": run_sched, "score": score}[sys.argv[1]]()


if __name__ == "__main__":
    main()
