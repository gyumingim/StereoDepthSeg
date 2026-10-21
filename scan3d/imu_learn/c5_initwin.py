#!/usr/bin/env python3
"""Init window test 2026-10-21 (docs/init_window_plan.md): static-only OpenVINS init (init_dyn_use false, imu thresh 0.3
= strict) with init_window_time W in 0.5/1.0/1.5 s and init_max_disparity D in 2.24/5/15 px on the 5 still-start
recordings trimmed to a still of S = 1/2 s (clip starts at motion_start_s - S + d, d = -0.25/0/+0.25 s), IMU lead
0 (main) and 2 s, RAW IMU (types 16/35), clip 20 s, via imu_learn/c5_run.py. At most PAR (3) OpenVINS runs, decode
4 threads (load only, frames identical), nice 5. Progress -> runs/next_vio.log. Results runs/init_window/<name>.{json,poses.jsonl.gz,stderr.log.gz,err}.

  (setsid nohup ../venv_ffs/bin/python imu_learn/c5_initwin.py > runs/init_window/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/c5_initwin.py --only NAME     # one run in the foreground (timing)
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
OUT = Path("runs/init_window")
LOG = Path("runs/next_vio.log")
STILL = Path("runs/overnight_vio/inputs")
RECS = ["hs_20261020_214138", "hs_20261020_221052", "hs_20261020_222747", "hs_20261020_223815", "hs_20261020_224323"]
WS = {"W05": 0.5, "W10": 1.0, "W15": 1.5}
DISP = {"D224": 2.24, "D5": 5.0, "D15": 15.0}
SS = (2, 1)
LEADS = {"L0": 0.0, "L2": 2.0}
DS = {"dm025": -0.25, "d0": 0.0, "dp025": 0.25}
MAX_SECS = 20
PAR = int(os.environ.get("PAR", 3))


def log(msg):
    line = f"{time.strftime('%F %T')} iw: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def jobs():
    out = []
    for ln, lead in LEADS.items():            # main (L0) first, then L2
        for S in SS:
            for wn, w in WS.items():
                for dn_, disp in DISP.items():
                    for rec in RECS:
                        ms = json.load(open(STILL / f"{rec}_still.json"))["motion_start_s"]
                        for dn, d in DS.items():
                            name = f"{ln}_S{S}_{wn}_{dn_}_{rec[-6:]}_{dn}"
                            off = round(ms - S + d, 3)
                            args = [PY, "imu_learn/c5_run.py", f"recordings/{rec}", "--init-cfg", "strict",
                                    "--init-window", str(w), "--init-disp", str(disp), "--imu-raw",
                                    "--clones", "11", "--noise", "cfg", "--start-off", str(off), "--imu-lead", str(lead),
                                    "--max-secs", str(MAX_SECS), "--decode-threads", "4", "--out", str(OUT / name)]
                            out.append(dict(name=name, rec=rec, args=args))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=None)
    ap.add_argument("--phase", default=None, help="coarse | repeat:W05_D5,W10_D5 | lead2:W05_D5 (plan amendment 10:40)")
    o = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    js = jobs()
    if o.phase:                                   # amendment 2026-10-21 10:40: staged matrix instead of the full 540
        ph, _, arg = o.phase.partition(":")
        sel = set(arg.split(",")) if arg else set()
        if ph == "coarse":
            js = [j for j in js if j["name"].startswith("L0_") and j["name"].endswith("_d0")]
        elif ph == "repeat":
            js = [j for j in js if j["name"].startswith("L0_") and not j["name"].endswith("_d0")
                  and "_".join(j["name"].split("_")[2:4]) in sel]
        elif ph == "lead2":
            js = [j for j in js if j["name"].startswith("L2_") and "_".join(j["name"].split("_")[2:4]) in sel]
        else:
            raise SystemExit(f"unknown phase {o.phase}")
    if o.only:
        j = next(x for x in js if x["name"] == o.only)
        t0 = time.time()
        subprocess.run(["nice", "-n", "5"] + j["args"])
        log(f"timing {j['name']} {time.time() - t0:.0f} s")
        return
    files = ("imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/imu_gt.py", "imu_learn/c5_initwin.py",
             "make_ov_config.py")
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    blobs = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in files}
    dirty = subprocess.run(["git", "status", "--porcelain", *files], capture_output=True, text=True).stdout.strip()
    log(f"scheduler start phase {o.phase} git {head} dirty={bool(dirty)} PAR {PAR} jobs {len(js)} blobs {blobs}")
    queue = [j for j in js if not (OUT / f"{j['name']}.json").exists()]
    running = {}
    nd = 0
    while queue or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            nd += 1
            try:
                d = json.load(open(OUT / f"{n}.json"))
                tail = (f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} scale {d.get('scale')} "
                        f"m {d.get('init_info', {}).get('method')} err {d.get('error')}")
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            if nd % 10 == 0 or "FAILED" in tail:
                log(f"done {n} ({time.time() - t0:.0f} s, {len(queue)} queued, load "
                    f"{open('/proc/loadavg').read().split()[0]}) {tail}")
        while queue and len(running) < PAR:
            j = queue.pop(0)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + j["args"], stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
        time.sleep(5)
    log("scheduler end (all done)")


if __name__ == "__main__":
    main()
