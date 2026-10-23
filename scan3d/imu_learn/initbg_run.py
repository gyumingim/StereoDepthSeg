#!/usr/bin/env python3
"""Static-init gyro bias from a prior (P1 zero, P2 leave-one-recording-out still prior): bit-check and pre-registered
comparison (docs/initbg_plan.md, 2026-10-23). Init timing is NOT changed (no gate).

Bridge = stock ov_bridge/main.cpp built out of tree at ~/scanner/src/ovbuild_initbg_bridge against
~/scanner/src/open_vins_initbg (= ~/scanner/src/open_vins copy + tools/ov_initbg_patch.py, diff tools/ov_patches/initbg.diff),
library ~/scanner/src/ovbuild_initbg. Keys init_bg_mode (0 off / 1 zero / 2 prior) and init_bg_prior [x, y, z] appended to
the written estimator_config.yaml. Everything else = phone60_run V4 (as imu_learn/initgyro_run.py). o0 only (user decision).
P2 prior per recording = runs/initbg/prior.json (imu_learn/initbg_prior.py, made before any run).

  ../venv_ffs/bin/python imu_learn/initbg_run.py bitcheck [KEY]
  (setsid nohup ../venv_ffs/bin/python imu_learn/initbg_run.py run > runs/initbg/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/initbg_run.py score
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
OUT = Path("runs/initbg")
REF_DIR = Path("runs/phone60")
PAR = int(os.environ.get("PAR", 12))
BRIDGE = Path.home() / "scanner" / "src" / "ovbuild_initbg_bridge" / "ov_bridge"
CONDS = {"P0": 0, "P1": 1, "P2": 2}
RUN_CONDS = ["P1", "P2"]


def prior(rec):
    return json.load(open(OUT / "prior.json"))["prior"][rec[-6:]]["prior"]


def replay(rec, off, mode):
    import vio_bench_hs as vb
    vb.TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
    vb.BRIDGE = BRIDGE
    orig_wc = vb.write_config
    pr = prior(rec) if mode == 2 else [0.0, 0.0, 0.0]
    lines = f"init_bg_mode: {mode}\ninit_bg_prior: [ {pr[0]:.6f}, {pr[1]:.6f}, {pr[2]:.6f} ]\n"

    def wc(d, *args, **kw):
        used = orig_wc(d, *args, **kw)
        with open(d / "estimator_config.yaml", "a") as f:
            f.write(lines)
        used["init_bg_mode"] = str(mode)
        used["init_bg_prior"] = pr
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
    r.update(ib_mode=mode, ib_prior=pr, ib_bridge=str(BRIDGE))
    return r, saved.get("poses", [])


def strip(P):
    return [{k: v for k, v in m.items() if k not in ("ms", "wall_ms")} for m in P]


def load_poses(pre):
    return [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]


def bitcheck(key="131326_o0"):
    OUT.mkdir(parents=True, exist_ok=True)
    rec = next(r for r in RAW + OLD if r.endswith(key.split("_")[0]))
    r, P = replay(f"recordings/{rec}", int(key.split("_o")[1]), 0)
    V = load_poses(REF_DIR / f"{key}_V4")
    same = strip(P) == strip(V)
    res = dict(made=time.strftime("%F %T"), key=key, ref=f"{REF_DIR}/{key}_V4", n_poses=[len(P), len(V)], identical=same,
               ate=[r.get("ate_cm"), json.load(open(REF_DIR / f"{key}_V4.json")).get("ate_cm")],
               config_tail=dict(init_bg_mode=0, init_bg_prior=[0, 0, 0]), keys_ignored=["ms", "wall_ms"], bridge=str(BRIDGE))
    if not same:
        res["first_diff"] = next((i for i, (x, y) in enumerate(zip(strip(P), strip(V))) if x != y), None)
    json.dump(res, open(OUT / "bitcheck.json", "w"), indent=1)
    print(json.dumps(res, indent=1))


def jobs():
    return [dict(rec=r, off=0, cond=c, name=f"{r[-6:]}_o0_{c}") for r in RAW + OLD for c in RUN_CONDS]


def job(name):
    j = next(x for x in jobs() if x["name"] == name)
    t0 = time.time()
    r, P = replay(f"recordings/{j['rec']}", j["off"], CONDS[j["cond"]])
    r.update(ib_cond=j["cond"], ib_made=time.strftime("%F %T"), ib_job_s=round(time.time() - t0, 1))
    with gzip.open(str(OUT / name) + ".poses.jsonl.gz", "wt") as f:
        for m in P:
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched():
    dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no", "imu_learn/vio_bench_hs.py",
                            "imu_learn/initbg_run.py", "config/mono", "ov_bridge/main.cpp", "tools/ov_patches/initbg.diff",
                            "docs/initbg_plan.md", "runs/initbg/prior.json"], capture_output=True, text=True).stdout
    if dirty.strip():
        sys.exit(f"REFUSED: uncommitted {dirty}")
    if not (OUT / "bitcheck.json").exists() or not json.load(open(OUT / "bitcheck.json")).get("identical"):
        sys.exit("REFUSED: bitcheck missing or not identical")
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
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/initbg_run.py", "job", j["name"]],
                                                   stdout=subprocess.DEVNULL,
                                                   stderr=open(OUT / f"{j['name']}.err", "w")), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end wall {time.time() - t_all:.0f} s", flush=True)


def score():
    import numpy as np
    import zupt_ob_score as zs
    os.chdir(SCAN3D)
    runs = {}
    for r_ in RAW + OLD:
        key = f"{r_[-6:]}_o0"
        row = dict(rec=r_, key=key)
        for c, pre in [("base", REF_DIR / f"{key}_V4")] + [(c, OUT / f"{key}_{c}") for c in RUN_CONDS]:
            d = json.load(open(str(pre) + ".json")) if Path(str(pre) + ".json").exists() else {}
            m = zs.run_metrics(pre, r_) if d else dict(divergent=True, why="missing")
            P = load_poses(pre) if d else []
            ft = next((x for x in P if x.get("status") == "tracking"), None)
            row[c] = dict(div=bool(m.get("divergent")), ate_w=m.get("ate_w"), rel5_max=m.get("rel5_max"),
                          t_init=d.get("t_init_clip_s"), init_t_ns=ft["t_ns"] if ft else None,
                          bg0=[round(x, 4) for x in ft["bg"]] if ft and ft.get("bg") else None,
                          bg_end=[round(x, 4) for x in P[-1]["bg"]] if P and P[-1].get("bg") else None,
                          missing=not d)
        runs[key] = row
    use = [r for r in runs.values() if r["rec"] != DROP]
    nb = sum(r["base"]["div"] for r in use)
    res = dict(made=time.strftime("%F %T"), base=f"{REF_DIR}/<key>_V4", runs=runs, criteria={}, verdict={})
    for c in RUN_CONDS:
        nc = sum(r[c]["div"] for r in use)
        normal = [r for r in use if not r["base"]["div"] and not r[c]["div"]]
        mb = float(np.median([r["base"]["ate_w"] for r in normal]))
        mc = float(np.median([r[c]["ate_w"] for r in normal]))
        c131 = runs["131326_o0"][c]["div"]
        same_init = [r["key"] for r in runs.values() if r[c]["init_t_ns"] != r["base"]["init_t_ns"]]
        crit = dict(a=dict(div_base=nb, div_cond=nc, d131326_o0=c131, pass_=nc <= nb and not c131),
                    b=dict(n_normal=len(normal), med_base=round(mb, 3), med_cond=round(mc, 3),
                           limit=round(1.2 * mb + 0.3, 3), pass_=mc <= 1.2 * mb + 0.3),
                    init_same=dict(cells_differ=same_init, pass_=not same_init))
        res["criteria"][c] = crit
        res["verdict"][c] = "통과" if all(v["pass_"] for v in crit.values()) else "불통과"
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(dict(criteria=res["criteria"], verdict=res["verdict"]), ensure_ascii=False, indent=1))
    f = lambda x: "-" if x is None else ("%.1f" % x if x < 100 else "%.0f" % x)
    for k, r in sorted(runs.items()):
        print("|", k[:6], "|", " | ".join(f(r[c]["ate_w"]) + ("*" if r[c]["div"] else "") for c in ["base"] + RUN_CONDS), "|",
              r["base"]["t_init"], "|", " / ".join(str(r[c]["bg0"]) for c in ["base"] + RUN_CONDS), "|")


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
