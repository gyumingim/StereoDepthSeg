#!/usr/bin/env python3
"""Part 2 of docs/gyro_rebias_apply_plan.md: OpenVINS bench, U (--imu-raw) vs G (--imu-raw --gyro-rebias), same 9 pairs
and settings as imu_learn/calib_gain_p2.py (i3 confirm base). Both fresh.
  (setsid nohup ../venv_ffs/bin/python imu_learn/gyro_rebias_p2.py run > runs/gyro_rebias/p2/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/gyro_rebias_p2.py analyse      # -> runs/gyro_rebias/part2.json
"""
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
import calib_gain_p2 as p2  # noqa: E402
import z2_regress as zr  # noqa: E402

PY = "../venv_ffs/bin/python"
BASE = Path("runs/gyro_rebias")
OUT = BASE / os.environ.get("GRB_OUT", "p2")          # follow-up 2: GRB_OUT=p2b GRB_ONLY=214550_G_o0,...
LOG = BASE / f"{OUT.name}.log"
ONLY = [x for x in os.environ.get("GRB_ONLY", "").split(",") if x]
PAR = int(os.environ.get("PAR", 5))


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} grb-p2: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def jobs():
    js = []
    for rec in p2.RECS:
        offs = json.load(open(f"runs/i3_confirm/inputs/{rec}.json"))["offsets_s"]
        for k, off in enumerate(offs):
            for c in ("U", "G"):
                js.append(dict(rec=rec, cond=c, k=k, off=off, name=f"{rec[-6:]}_{c}_o{k}",
                               args=zr.common(rec, off, 2.0, 0, True)))
    return js


def cmd(j):
    return [PY, "imu_learn/gyro_rebias_p2.py", "job", j["cond"], "--cfg-over", "num_opencv_threads=2", "--still-hold", "",
            "--", *j["args"], "--out", str(OUT / j["name"])]


def job(argv):
    """argv = COND + z2_isolate job args. G sets a.gyro_rebias = True in the bench namespace (c5_run does not know it)."""
    import vio_bench_hs as vb
    import z2_isolate
    cond = argv[0]
    orig = vb.run

    def run(a):
        if cond == "G":
            a.gyro_rebias = True
        return orig(a)
    vb.run = run
    z2_isolate.job(argv[1:])
    out = argv[argv.index("--out") + 1]
    d = json.load(open(out + ".json"))
    d["grb_cond"] = cond
    json.dump(d, open(out + ".json", "w"), indent=1, ensure_ascii=False)


def run():
    OUT.mkdir(parents=True, exist_ok=True)
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain", "imu_learn/gyro_rebias.py", "imu_learn/gyro_rebias_p2.py",
                            "imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/z2_isolate.py",
                            "imu_learn/imu_gt.py"], capture_output=True, text=True).stdout.strip()
    if dirty:
        log(f"REFUSED: code not committed: {dirty}")
        sys.exit(1)
    queue = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists() and (not ONLY or j["name"] in ONLY)]
    log(f"start git {head} PAR {PAR} jobs {len(queue)}")
    running = {}
    while queue or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                tail = f"ate {d.get('ate_cm')} scale {d.get('scale')} init {d.get('t_init_clip_s')} rebias {d.get('gyro_rebias')}"
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s) {tail}")
        while queue and len(running) < PAR:
            j = queue.pop(0)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
            log(f"start {j['name']} off {j['off']} ({len(queue)} queued)")
            time.sleep(0.5)
        time.sleep(10)
    log("end")


def analyse():
    pairs = []
    for j in jobs():
        if j["cond"] != "U":
            continue
        u, g = OUT / j["name"], OUT / j["name"].replace("_U_", "_G_")
        if not (Path(str(u) + ".json").exists() and Path(str(g) + ".json").exists()):
            continue
        pairs.append(dict(rec=j["rec"], k=j["k"], off=j["off"], U=p2.metrics(u, j["rec"]), G=p2.metrics(g, j["rec"])))
    ok = [p for p in pairs if p["U"]["ate_cm"] is not None and p["G"]["ate_cm"] is not None]
    diff = [p["G"]["ate_cm"] - p["U"]["ate_cm"] for p in ok]
    new_collapse = [f"{p['rec'][-6:]}_o{p['k']}" for p in pairs if p["U"]["status"] == "ok" and p["G"]["status"] != "ok"]
    med_diff = float(np.median(diff)) if diff else None
    ym = lambda c: float(np.median([p[c]["yaw_dpm"] for p in ok if "yaw_dpm" in p[c]]))  # noqa: E731
    out = dict(made=time.strftime("%F %T"), plan="docs/gyro_rebias_apply_plan.md (bd63072)", n_pairs=len(pairs),
               n_scored=len(ok), pairs=pairs,
               ate_median_U=float(np.median([p["U"]["ate_cm"] for p in ok])),
               ate_median_G=float(np.median([p["G"]["ate_cm"] for p in ok])),
               ate_diff_median_cm=med_diff, G_lower=sum(d < 0 for d in diff), G_higher=sum(d > 0 for d in diff),
               new_collapse=new_collapse, collapses_U=sum(p["U"]["status"] != "ok" for p in pairs),
               collapses_G=sum(p["G"]["status"] != "ok" for p in pairs),
               vio_yaw_dpm_median_U=ym("U"), vio_yaw_dpm_median_G=ym("G"))
    out["criteria"] = dict(c1_rule="median paired ATE diff (G-U) <= +0.1 cm", c1_pass=len(pairs) == 9 and med_diff is not None
                           and med_diff <= 0.1, c2_rule="no pair with U ok and G not ok", c2_pass=len(pairs) == 9 and not new_collapse)
    out["criteria"]["passed"] = out["criteria"]["c1_pass"] and out["criteria"]["c2_pass"]
    json.dump(out, open(BASE / "part2.json", "w"), indent=1)
    for p in pairs:
        print(p["rec"][-6:], p["k"], "U", p["U"]["ate_cm"], p["U"]["status"], p["U"].get("yaw_dpm"), "| G",
              p["G"]["ate_cm"], p["G"]["status"], p["G"].get("yaw_dpm"))
    print({k: v for k, v in out.items() if k != "pairs"})


if __name__ == "__main__":
    if sys.argv[1] == "job":
        job(sys.argv[2:])
    else:
        {"run": run, "analyse": analyse}[sys.argv[1]]()
