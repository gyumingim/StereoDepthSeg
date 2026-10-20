#!/usr/bin/env python3
"""Overnight VIO matrix 2026-10-20 (docs/overnight_vio_prereg.md). Waits for each new recording's ground truth
("<rec> done:" in runs/overnight_gt.log), writes its still/bias inputs (c5_still.py) and queues runs of
imu_learn/c5_run.py; at most PAR (3) OpenVINS runs at a time, video decode 2 threads, nice 5.

  A  init config  loose | strict | strict-dyn, Android-calibrated IMU (1/4), app noise, start offset 0/1/2 s
  B  IMU source   raw 16/35 under the best A config (chosen once, prereg rule; the cal counterpart = the A run)
  C  calibration  old physical (old_phys_mean.json) | still bias (<rec>_bias_cal.json), calibrated IMU, best config
  N  noise        Allan, calibrated IMU, best config (secondary)
  C2 still bias on the raw IMU (secondary)
  D  determinism  best-config cal o0 again, alone, on the first two recordings
Progress -> runs/overnight_vio.log, results -> runs/overnight_vio/<name>.{json,poses.jsonl.gz,stderr.log.gz,err}.
Start detached: (setsid nohup ../venv_ffs/bin/python imu_learn/c5_overnight.py > runs/overnight_vio/sched.out 2>&1 &)
STOP_EPOCH (env, unix s): no new runs after this time (default: next 06:05 local).
"""
import datetime as dtm
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
PY = "../venv_ffs/bin/python"
OUT = Path("runs/overnight_vio")
INP = OUT / "inputs"
LOG = Path("runs/overnight_vio.log")
GTLOG = Path("runs/overnight_gt.log")
RECS = ["hs_20261020_214138", "hs_20261020_221052", "hs_20261020_221925", "hs_20261020_222747", "hs_20261020_223815"]
BIAS_FROM = {"hs_20261020_221925": "hs_20261020_221903"}      # same session, no still of its own
CFGS = {"loose": "loose", "strict": "strict", "strictdyn": "strict-dyn"}
PHASE_PRI = {"A": 0, "B": 1, "C": 2, "N": 3, "C2": 4, "D": 5}
PAR = int(os.environ.get("PAR", 3))
OFFS = (0, 1, 2)


def default_stop():
    now = dtm.datetime.now()
    t = now.replace(hour=6, minute=5, second=0, microsecond=0)
    if t <= now:
        t += dtm.timedelta(days=1)
    return t.timestamp()


STOP_EPOCH = float(os.environ.get("STOP_EPOCH", default_stop()))


def log(msg):
    line = f"{time.strftime('%F %T')} vio: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def gt_state(rec):
    """ready only if the LAST GT-log line that names this recording is its "done:" line (a later "start", redo note or
    step line = the GT is being rebuilt -> not ready; 2026-10-21 00:28 221052 was redone after its done line)."""
    if not GTLOG.exists():
        return "wait"
    lines = [ln for ln in GTLOG.read_text(errors="replace").splitlines() if rec[-6:] in ln.split("disagree")[0]]
    if not lines:
        return "wait"
    last = lines[-1]
    if re.search(rf"{rec} (imu_gt|batch_sync|imu_gt_rs) FAILED", last):
        return "failed"
    if f"{rec} done:" in last and Path(f"recordings/{rec}/imu_gt/align.json").exists():
        return "ready"
    return "wait"


def job(name, phase, rec, cfg, imu="cal", calib=None, calib_label="none", noise="cfg", off=0):
    args = [PY, "imu_learn/c5_run.py", f"recordings/{rec}", "--init-cfg", CFGS[cfg], "--clones", "11",
            "--noise", noise, "--start-off", str(off), "--max-secs", "60", "--decode-threads", "2",
            "--calib-label", calib_label, "--out", str(OUT / name)]
    if imu == "raw":
        args.append("--imu-raw")
    if calib:
        args += ["--imu-calib", str(calib)]
    return dict(name=name, phase=phase, rec=rec, args=args)


def a_jobs(rec):
    return [job(f"{rec[-6:]}_A_{c}_cal_o{o}", "A", rec, c, off=o) for c in CFGS for o in OFFS]


def later_jobs(rec, best):
    j = [job(f"{rec[-6:]}_B_{best}_raw_o{o}", "B", rec, best, imu="raw", off=o) for o in OFFS]
    j += [job(f"{rec[-6:]}_C_{best}_oldphys_o{o}", "C", rec, best, calib=INP / "old_phys_mean.json",
              calib_label="old_phys", off=o) for o in OFFS]
    if (INP / f"{rec}_bias_cal.json").exists():
        j += [job(f"{rec[-6:]}_C_{best}_stillbias_o{o}", "C", rec, best, calib=INP / f"{rec}_bias_cal.json",
                  calib_label="still_bias", off=o) for o in OFFS]
    j += [job(f"{rec[-6:]}_N_{best}_allan_o{o}", "N", rec, best, noise="allan", off=o) for o in OFFS]
    if (INP / f"{rec}_bias_raw.json").exists():
        j += [job(f"{rec[-6:]}_C2_{best}_rawstillbias_o{o}", "C2", rec, best, imu="raw",
                  calib=INP / f"{rec}_bias_raw.json", calib_label="still_bias_raw", off=o) for o in OFFS]
    return j


def choose_best(still_recs):
    """Prereg rule: over the A runs of the first two still-start recordings, the config with the most successful runs
    (plan definition, GT scale 1); tie -> lower median post-init ATE of the successful runs; tie -> strict (the app)."""
    import numpy as np
    import c5_overnight_analyse as an
    score = {}
    for c in CFGS:
        ok, ates = 0, []
        for rec in still_recs:
            for o in OFFS:
                p = OUT / f"{rec[-6:]}_A_{c}_cal_o{o}"
                if not Path(str(p) + ".json").exists():
                    continue
                r = an.analyse_run(str(p), k=1.0)
                if r.get("success"):
                    ok += 1
                    ates.append(r["ate_full_cm"])
        score[c] = (ok, float(np.median(ates)) if ates else 1e9)
    best = sorted(CFGS, key=lambda c: (-score[c][0], score[c][1], c != "strict"))[0]
    json.dump(dict(best=best, score=score, recs=still_recs, rule="most successes, then median ATE, then strict",
                   time=time.strftime("%F %T")), open(OUT / "best_config.json", "w"), indent=1)
    log(f"best config = {best} from {still_recs}: {score}")
    return best


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    files = ("imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/c5_still.py", "imu_learn/c5_overnight.py",
             "imu_learn/c5_overnight_analyse.py", "make_ov_config.py")
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    blobs = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in files}
    dirty = subprocess.run(["git", "status", "--porcelain", *files], capture_output=True, text=True).stdout.strip()
    log(f"scheduler start git {head} dirty={bool(dirty)} PAR {PAR} stop {time.ctime(STOP_EPOCH)} blobs {blobs}")
    queue, running, seen = [], {}, set()
    best = json.load(open(OUT / "best_config.json"))["best"] if (OUT / "best_config.json").exists() else None
    state = {r: None for r in RECS}
    det_queued = False
    while True:
        late = time.time() >= STOP_EPOCH
        for rec in RECS:
            if state[rec] == "queued" and gt_state(rec) != "ready":     # GT being rebuilt -> discard and wait again
                disc = OUT / f"discarded_{rec[-6:]}_{time.strftime('%H%M')}"
                disc.mkdir(exist_ok=True)
                for n, (p, j, t0) in list(running.items()):
                    if j["rec"] == rec:
                        p.kill()
                        p.wait()
                        del running[n]
                for f in list(OUT.glob(f"{rec[-6:]}_*")) + list(INP.glob(f"{rec}_*")):
                    f.rename(disc / f.name)
                queue = [j for j in queue if j["rec"] != rec]
                seen = {n for n in seen if not n.startswith(rec[-6:])}
                state[rec] = None
                log(f"{rec} GT changed after its done line -> runs killed, outputs moved to {disc}")
                continue
            if state[rec] in ("queued", "failed"):
                continue
            st = gt_state(rec)
            if st == "failed":
                state[rec] = "failed"
                log(f"{rec} GT failed -> skipped")
            elif st == "ready":
                cmd = [PY, "imu_learn/c5_still.py", f"recordings/{rec}"]
                if rec in BIAS_FROM:
                    cmd += ["--bias-from", f"recordings/{BIAS_FROM[rec]}"]
                r = subprocess.run(cmd, capture_output=True, text=True)
                log(f"{rec} GT ready; still: {(r.stdout.strip().splitlines() or [r.stderr[-300:]])[-1]}")
                queue += a_jobs(rec)
                if best:
                    queue += later_jobs(rec, best)
                state[rec] = "queued"
        if best is None:
            cands = []
            for rec in RECS:
                sj = INP / f"{rec}_still.json"
                if state[rec] != "queued" or not sj.exists() or not json.load(open(sj)).get("has_still"):
                    continue
                names = [j["name"] for j in a_jobs(rec)]
                if all((OUT / f"{n}.json").exists() or (n in seen and n not in running) for n in names):
                    cands.append(rec)
            if len(cands) >= 2:
                best = choose_best(cands[:2])
                for rec in RECS:
                    if state[rec] == "queued":
                        queue += later_jobs(rec, best)
        for n, (p, j, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            if (OUT / f"{n}.json").exists():
                try:
                    d = json.load(open(OUT / f"{n}.json"))
                    tail = (f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} scale {d.get('scale')} "
                            f"tracked {d.get('tracked_frac')} wall {d.get('wall_s')} err {d.get('error')}")
                except Exception as e:  # noqa: BLE001
                    tail = f"json unreadable {e!r}"
            else:
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s, load {open('/proc/loadavg').read().split()[0]}) {tail}")
        queue = [j for j in queue if j["name"] not in seen]
        queue.sort(key=lambda j: PHASE_PRI[j["phase"]])
        while queue and len(running) < PAR and not late:
            j = queue[0]
            if j["phase"] == "D" and running:
                break                                            # determinism repeats run alone
            queue.pop(0)
            seen.add(j["name"])
            if (OUT / f"{j['name']}.json").exists():
                continue
            err = open(OUT / f"{j['name']}.err", "w")
            p = subprocess.Popen(["nice", "-n", "5"] + j["args"], stdout=subprocess.DEVNULL, stderr=err)
            running[j["name"]] = (p, j, time.time())
            log(f"start {j['name']} load {open('/proc/loadavg').read().split()[0]}")
            if j["phase"] == "D":
                break
        all_known = all(state[r] in ("queued", "failed") for r in RECS)
        if best and all_known and not queue and not running and not det_queued:
            first = [r for r in RECS if state[r] == "queued"][:2]
            queue += [job(f"rep_{r[-6:]}_A_{best}_cal_o0", "D", r, best, off=0) for r in first]
            det_queued = True
            continue
        if not running and ((late) or (all_known and det_queued and not queue)):
            log(f"scheduler end ({'stop time' if late else 'all done'}); pending {[j['name'] for j in queue]}; "
                f"states {state}")
            break
        time.sleep(20)


if __name__ == "__main__":
    main()
