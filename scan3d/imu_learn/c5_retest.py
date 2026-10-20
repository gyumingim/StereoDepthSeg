#!/usr/bin/env python3
"""C5 retest 2026-10-21 (docs/c5_retest_plan.md): short still before motion. Per recording with a still start, the clip
starts at motion_start_s - S + d (overnight still detection, runs/overnight_vio/inputs/<rec>_still.json), S in 1/2/3 s,
d in -0.25/0/+0.25 s, init config loose/strict/strict-dyn, IMU lead 0 (main) via imu_learn/c5_run.py. Secondary:
IMU lead 2 s (bench default), S = 1 only (names prefixed L2_), queued after the main matrix.
At most PAR (3) OpenVINS runs, video decode 2 threads, nice 5. Progress -> runs/c5_retest.log.

  (setsid nohup ../venv_ffs/bin/python imu_learn/c5_retest.py > runs/c5_retest/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/c5_retest.py --only NAME     # one run in the foreground (timing)
"""
import argparse
import json
import os
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
PY = "../venv_ffs/bin/python"
OUT = Path("runs/c5_retest")
LOG = Path("runs/c5_retest.log")
STILL = Path("runs/overnight_vio/inputs")
RECS = ["hs_20261020_214138", "hs_20261020_221052", "hs_20261020_222747", "hs_20261020_223815", "hs_20261020_224323"]
CFGS = {"loose": "loose", "strict": "strict", "strictdyn": "strict-dyn"}
SS = (1, 2, 3)
DS = {"dm025": -0.25, "d0": 0.0, "dp025": 0.25}
PAR = int(os.environ.get("PAR", 3))


def log(msg):
    line = f"{time.strftime('%F %T')} c5r: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def jobs():
    out = []
    for lead, pre, ss in ((0.0, "", SS), (2.0, "L2_", (1,))):
        for S in ss:
            for rec in RECS:
                ms = json.load(open(STILL / f"{rec}_still.json"))["motion_start_s"]
                for c in CFGS:
                    for dn, d in DS.items():
                        name = f"{pre}{rec[-6:]}_S{S}_{c}_{dn}"
                        off = round(ms - S + d, 3)
                        args = [PY, "imu_learn/c5_run.py", f"recordings/{rec}", "--init-cfg", CFGS[c], "--clones", "11",
                                "--noise", "cfg", "--start-off", str(off), "--imu-lead", str(lead), "--max-secs", "60",
                                "--decode-threads", "2", "--out", str(OUT / name)]
                        out.append(dict(name=name, rec=rec, args=args))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=None)
    o = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    js = jobs()
    if o.only:
        j = next(x for x in js if x["name"] == o.only)
        t0 = time.time()
        subprocess.run(["nice", "-n", "5"] + j["args"])
        log(f"timing {j['name']} {time.time() - t0:.0f} s")
        return
    files = ("imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/imu_gt.py", "imu_learn/c5_retest.py",
             "make_ov_config.py")
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    blobs = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in files}
    dirty = subprocess.run(["git", "status", "--porcelain", *files], capture_output=True, text=True).stdout.strip()
    log(f"scheduler start git {head} dirty={bool(dirty)} PAR {PAR} jobs {len(js)} blobs {blobs}")
    queue = [j for j in js if not (OUT / f"{j['name']}.json").exists()]
    running = {}
    while queue or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                tail = (f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} scale {d.get('scale')} "
                        f"tracked {d.get('tracked_frac')} err {d.get('error')}")
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s, load {open('/proc/loadavg').read().split()[0]}) {tail}")
        while queue and len(running) < PAR:
            j = queue.pop(0)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + j["args"], stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
            log(f"start {j['name']} ({len(queue)} left)")
        time.sleep(10)
    log("scheduler end (all done)")


if __name__ == "__main__":
    main()
