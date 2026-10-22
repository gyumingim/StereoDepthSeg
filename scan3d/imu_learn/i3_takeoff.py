#!/usr/bin/env python3
"""i3 takeoff test, 2026-10-22 night (docs/i3_takeoff_plan.md, pre-registered 4718321 / c0b3c0b before any run).

A: 215348 base_o1 / i3_o1 repeated twice (run-to-run noise of the criterion-4 window values and full ATE).
B: long still then departure: 4 recordings x 2 set-down stills x clip start (still start +1/+5/+10 s) x base/i3,
   --max-secs 40; collapse = 10 s window starting at the IMU still end (departure): Sim3 scale outside 0.5..2 or SE3 ATE > 10 cm.
C: 214550 departure mechanism from the existing runs/i3_confirm runs (no new VIO).
Same bench settings as i3_confirm (z2_isolate job, raw IMU, strict static init 1.0 s / 15 px / dyn off, OpenVINS threads 2,
<= PAR (5) at once, nice 5).

  (setsid nohup ../venv_ffs/bin/python imu_learn/i3_takeoff.py run > runs/i3_takeoff/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/i3_takeoff.py list
  ../venv_ffs/bin/python imu_learn/i3_takeoff.py analyse        # -> runs/i3_takeoff/summary.json
  ../venv_ffs/bin/python imu_learn/i3_takeoff.py html           # -> docs/i3_takeoff_result.html
"""
import argparse
import base64
import gzip
import html as H
import io
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
import i3_confirm as ic  # noqa: E402
import z2_regress as zr  # noqa: E402

PY = "../venv_ffs/bin/python"
OUT = Path("runs/i3_takeoff")
LOG = Path("runs/i3_takeoff.log")
SRC = Path("runs/i3_confirm")
RECS = ic.RECS
PAR = int(os.environ.get("PAR", 5))
THREADS = "2"
CONDS = ic.CONDS
FILES = ic.FILES + ("imu_learn/i3_takeoff.py",)
DS = (1, 5, 10)
MAX_SECS = 40
MIN_LEFT = 1.5
WIN_S = 10.0
MIN_MATCH = 30
A_REC = "hs_20261021_215348"


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} i3t: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def blobs():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    b = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in FILES}
    dirty = subprocess.run(["git", "status", "--porcelain", *FILES], capture_output=True, text=True).stdout.strip()
    return head, b, bool(dirty)


def inp(rec):
    return json.load(open(SRC / "inputs" / f"{rec}.json"))


# ---------------------------------------------------------------- jobs
def cells():
    """B cells (rec, still index, d) with start offset; skipped ones carry 'skip'."""
    out = []
    for rec in RECS:
        x = inp(rec)
        f0 = x["first_frame_imu_ns"]
        for si, w in enumerate(x["setdown"]):
            st = w["still_imu_ns"]
            for d in DS:
                c = dict(rec=rec, si=si, d=d, still_imu_ns=st, still_s=w["still_s"], key=f"{rec[-6:]}_s{si}_d{d}")
                if st is None:
                    c["skip"] = "no IMU still"
                else:
                    c["off"] = round((st[0] - f0) / 1e9 + d, 3)
                    left = (st[1] - (st[0] + d * 1e9)) / 1e9
                    c["left_s"] = round(left, 3)
                    if left < MIN_LEFT - 1e-9:
                        c["skip"] = f"still left after start {left:.2f} s < {MIN_LEFT} s"
                out.append(c)
    return out


def jobs():
    js = []
    xa = inp(A_REC)
    for r in (1, 2):
        for c, hold in CONDS.items():
            js.append(dict(kind="A", rec=A_REC, cond=c, hold=hold, off=xa["offsets_s"][1], name=f"A_{A_REC[-6:]}_{c}_o1_r{r}",
                           args=zr.common(A_REC, xa["offsets_s"][1], 2.0, 0, True, rebias=False)))
    for cl in cells():
        if cl.get("skip"):
            continue
        for c, hold in CONDS.items():
            js.append(dict(kind="B", rec=cl["rec"], cond=c, hold=hold, off=cl["off"], name=f"B_{cl['key']}_{c}",
                           args=zr.common(cl["rec"], cl["off"], 2.0, MAX_SECS, True, rebias=False)))
    return js


B2_LEADS = (6, 3)


def b2_cells():
    """B' b cells: clip start = setdown[0] still start - L (inside the set-down motion)."""
    out = []
    for rec in RECS:
        x = inp(rec)
        st = x["setdown"][0]["still_imu_ns"]
        for L in B2_LEADS:
            out.append(dict(rec=rec, L=L, still_imu_ns=st, still_s=x["setdown"][0]["still_s"], key=f"{rec[-6:]}_L{L}",
                            off=round((st[0] - x["first_frame_imu_ns"]) / 1e9 - L, 3)))
    return out


def jobs2():
    return [dict(kind="B2", rec=cl["rec"], cond=c, hold=hold, off=cl["off"], name=f"B2_{cl['key']}_{c}",
                 args=zr.common(cl["rec"], cl["off"], 2.0, MAX_SECS, True, rebias=False)) for cl in b2_cells() for c, hold in CONDS.items()]


def cmd(j):
    return [PY, "imu_learn/z2_isolate.py", "job", "--cfg-over", f"num_opencv_threads={THREADS}", "--still-hold",
            j["hold"], "--", *j["args"], "--out", str(OUT / j["name"])]


def n_running():
    r = subprocess.run(["pgrep", "-fc", f"z2_isolate.py job .*--out {OUT}/"], capture_output=True, text=True)
    return int(r.stdout.strip() or 0)


def run_sched(second=False):
    OUT.mkdir(parents=True, exist_ok=True)
    head, b, dirty = blobs()
    if dirty:
        log(f"REFUSED: code files not committed (git {head}) — commit first (one code version)")
        sys.exit(1)
    for cl in cells():
        if cl.get("skip"):
            log(f"B cell {cl['key']} SKIPPED: {cl['skip']}")
    queue = [j for j in (jobs2() if second else jobs()) if not (OUT / f"{j['name']}.json").exists()]
    log(f"start git {head} PAR {PAR} threads {THREADS} conds {CONDS} jobs {len(queue)} blobs {b}")
    running = {}
    while queue or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                d.update(i3t_head=head, i3t_blobs=b, i3t_dirty=dirty)
                json.dump(d, open(OUT / f"{n}.json", "w"), indent=1, ensure_ascii=False)
                tail = (f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} scale {d.get('scale')} held "
                        f"{d.get('frames_held')} err {d.get('error')}")
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s, load {open('/proc/loadavg').read().split()[0]}) {tail}")
        while queue and n_running() < PAR:
            j = queue.pop(0)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
            log(f"start {j['name']} off {j['off']} ({len(queue)} queued) {j['hold'] or 'base'}")
            time.sleep(0.5)
        time.sleep(10)
    log("end (queue done)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "run2", "list", "analyse", "html"])
    o = ap.parse_args()
    if o.cmd in ("run", "run2"):
        return run_sched(second=o.cmd == "run2")
    if o.cmd == "list":
        for cl in cells():
            print(cl["key"], cl.get("off"), cl.get("left_s"), cl.get("skip", ""))
        for cl in b2_cells():
            print(cl["key"], cl["off"])
        for j in jobs() + jobs2():
            print(j["name"], " ".join(cmd(j)))
        return
    import i3_takeoff_analyse as ta
    if o.cmd == "analyse":
        res = ta.analyse()
        json.dump(res, open(OUT / "summary.json", "w"), indent=1, ensure_ascii=False, default=float)
        print(json.dumps(res["B"]["criteria"], ensure_ascii=False, default=float, indent=1))
        return
    res = json.load(open(OUT / "summary.json"))
    lead = OUT / "lead.html"
    if lead.exists():
        ta.LEAD = lead.read_text(encoding="utf-8")
    chk = OUT / "check.json"
    ta.write_html(res, "docs/i3_takeoff_result.html", json.load(open(chk)) if chk.exists() else None)
    print("wrote docs/i3_takeoff_result.html")


if __name__ == "__main__":
    main()
