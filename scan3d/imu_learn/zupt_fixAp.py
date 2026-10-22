#!/usr/bin/env python3
"""ZUPT fix A' test, 2026-10-22 (docs/zupt_fixAp_plan.md; same criteria/run set as docs/zupt_fixA_plan.md).

Fix A' = fix A (ZUPT disparity reference = latest clone) + start-still exemption: before the first normal update
(has_moved_since_zupt false) the base reference is kept (tools/ov_patches/zuptAp.diff, ~/scanner/src/open_vins_zuptAp).
  base -> reused from runs/zupt_fixA/<cell>_base (same zbase bridge, same day); 2 base runs are re-run here and must be
          bit-identical to those (identity check) before any C run starts.
  Ap   -> tools/ov_bridge_zuptAp
Bench path = i3_confirm base: c5_run (in process) + zr.common(..., rebias=False), num_opencv_threads 2, still-hold off,
a.gyro_rebias = False explicitly; only c5_run.DIAG_BRIDGE is swapped per condition (like g10.py).

  (setsid nohup ../venv_ffs/bin/python imu_learn/zupt_fixAp.py run > runs/zupt_fixAp/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/zupt_fixAp.py list
  ../venv_ffs/bin/python imu_learn/zupt_fixAp.py ident     # identity check: re-run base vs runs/zupt_fixA base
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
import z2_regress as zr  # noqa: E402

PY = "../venv_ffs/bin/python"
OUT = Path("runs/zupt_fixAp")
LOG = Path("runs/zupt_fixAp.log")
BASE_DIR = Path("runs/zupt_fixA")
PAR = int(os.environ.get("PAR", 5))
THREADS = "2"
BRIDGES = {"base": SCAN3D / "tools" / "ov_bridge_zbase" / "build" / "ov_bridge",
           "Ap": SCAN3D / "tools" / "ov_bridge_zuptAp" / "build" / "ov_bridge"}
FILES = ("imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/imu_gt.py", "imu_learn/z2_regress.py",
         "imu_learn/zupt_fixA.py", "imu_learn/zupt_fixAp.py", "make_ov_config.py", "config/mono/estimator_config.yaml",
         "tools/ov_bridge_zbase/main.cpp", "tools/ov_bridge_zbase/CMakeLists.txt", "tools/ov_bridge_zuptAp/main.cpp",
         "tools/ov_bridge_zuptAp/CMakeLists.txt", "tools/ov_patches/zuptAp.diff")
BOARD = {"hs_20261022_133820": "target", "hs_20261022_134550": "target", "hs_20261022_134204": "control"}
I3 = ["hs_20261021_214550", "hs_20261021_213341", "hs_20261021_213955", "hs_20261021_215348"]
# identity check (plan): base re-run now vs the reused runs/zupt_fixA base, same args
IDENT = {"I3_215348_o1": "runs/zupt_fixA/I3_215348_o1_base", "BR_134204_o0": "runs/zupt_fixA/BR_134204_o0_base"}
POSE_KEYS = ("t", "t_ns", "status", "init", "p", "q", "v", "T_world_cam0", "n_slam", "n_msckf")


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} zfap: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def blobs():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    b = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in FILES}
    dirty = subprocess.run(["git", "status", "--porcelain", *FILES], capture_output=True, text=True).stdout.strip()
    return head, b, bool(dirty)


def cells():
    """key, rec, group (target | control | regress), sub (board | i3 | F | T | O), args."""
    out = []
    for rec, grp in BOARD.items():                     # board_rotation.py: offsets 0/1/2, full length, raw, lead 2
        for k, off in enumerate((0.0, 1.0, 2.0)):
            out.append(dict(key=f"BR_{rec[-6:]}_o{k}", rec=rec, group=grp, sub="board",
                            args=zr.common(rec, off, 2.0, 0, True, rebias=False)))
    for rec in I3:                                     # i3_confirm.jobs_for: offsets from inputs, full length, raw, lead 2
        offs = json.load(open(f"runs/i3_confirm/inputs/{rec}.json"))["offsets_s"]
        for k, off in enumerate(offs):
            out.append(dict(key=f"I3_{rec[-6:]}_o{k}", rec=rec, group="regress", sub="i3",
                            args=zr.common(rec, off, 2.0, 0, True, rebias=False)))
    for j in zr.jobs():                                # z2_regress base cells (F, T, O) with their exact args
        if j["cond"] != "base":
            continue
        out.append(dict(key=f"Z_{j['pair']}", rec=j["rec"], group="regress", sub=j["pair"][0], args=j["args"]))
    return out


def jobs(cs=None):
    cs = cs or cells()
    first = [c for c in cs if c["key"] in IDENT]
    pri = [c for c in cs if c["sub"] == "board" or c["rec"] == "hs_20261021_214550"]
    rest = [c for c in cs if c not in pri]
    js = [dict(name=f"{c['key']}_base", cond="base", cell=c["key"], args=c["args"], ident=True) for c in first]
    for c in pri + rest:                               # base is reused from BASE_DIR; only C runs here
        js.append(dict(name=f"{c['key']}_Ap", cond="Ap", cell=c["key"], args=c["args"], ident=False))
    return js


def cmd(j):
    return [PY, "imu_learn/zupt_fixAp.py", "job", "--cond", j["cond"], "--", *j["args"], "--out", str(OUT / j["name"])]


def job(argv):
    i = argv.index("--")
    ap = argparse.ArgumentParser()
    ap.add_argument("--cond", choices=list(BRIDGES), required=True)
    o = ap.parse_args(argv[:i])
    import c5_run
    import vio_bench_hs as vb
    c5_run.DIAG_BRIDGE = BRIDGES[o.cond]
    orig = vb.run

    def run(a):
        a.cfg_over = f"num_opencv_threads={THREADS}"
        a.still_hold = None
        a.gyro_rebias = False                          # explicit (i3_confirm / board_rotation base)
        return orig(a)
    vb.run = run
    sys.argv = ["c5_run.py"] + argv[i + 1:]
    c5_run.main()
    out = sys.argv[sys.argv.index("--out") + 1]
    d = json.load(open(out + ".json"))
    head, b, dirty = blobs()
    d.update(zfa_runner="imu_learn/zupt_fixAp.py", zfa_cond=o.cond, zfa_bridge=str(BRIDGES[o.cond]), zfa_head=head,
             zfa_blobs=b, zfa_dirty=dirty, zfa_made=time.strftime("%F %T"), zfa_gyro_rebias_set=False,
             zfa_cfg_over=f"num_opencv_threads={THREADS}")
    json.dump(d, open(out + ".json", "w"), indent=1, ensure_ascii=False)


def load_poses(prefix):
    return [json.loads(x) for x in gzip.open(str(prefix) + ".poses.jsonl.gz", "rt") if x.strip()]


def ident_one(new, old):
    a, b = load_poses(new), load_poses(old)
    ja, jb = json.load(open(str(new) + ".json")), json.load(open(str(old) + ".json"))
    diff = []
    if len(a) != len(b):
        diff.append(f"n poses {len(a)} vs {len(b)}")
    for k, (x, y) in enumerate(zip(a, b)):
        for key in POSE_KEYS:
            if x.get(key) != y.get(key):
                diff.append(f"pose {k} {key}: {x.get(key)} vs {y.get(key)}")
                break
        if len(diff) > 5:
            break
    for key in ("ate_cm", "ate_sim3_cm", "scale", "t_init_clip_s", "frames_fed", "align_dt_ms"):
        if ja.get(key) != jb.get(key):
            diff.append(f"{key}: {ja.get(key)} vs {jb.get(key)}")
    return dict(new=str(new), old=str(old), n_poses=[len(a), len(b)], identical=not diff, diffs=diff[:8],
                ate=[ja.get("ate_cm"), jb.get("ate_cm")])


def ident():
    res = dict(made=time.strftime("%F %T"), checks=[])
    for key, old in IDENT.items():
        res["checks"].append(ident_one(OUT / f"{key}_base", old))
    res["all_identical"] = all(c["identical"] for c in res["checks"])
    json.dump(res, open(OUT / "ident.json", "w"), indent=1)
    return res


def n_running():
    r = subprocess.run(["pgrep", "-fc", f"zupt_fixAp.py job .*--out {OUT}/"], capture_output=True, text=True)
    return int(r.stdout.strip() or 0)


def run_sched():
    OUT.mkdir(parents=True, exist_ok=True)
    head, b, dirty = blobs()
    if dirty:
        log(f"REFUSED: code files not committed (git {head})")
        sys.exit(1)
    for c, p in BRIDGES.items():
        if not p.exists():
            log(f"REFUSED: bridge {c} missing {p}")
            sys.exit(1)
    queue = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists()]
    log(f"start git {head} PAR {PAR} threads {THREADS} jobs {len(queue)} blobs {b}")
    running = {}
    ident_done = (OUT / "ident.json").exists() and json.load(open(OUT / "ident.json"))["all_identical"]
    while queue or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                tail = f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} scale {d.get('scale')} err {d.get('error')}"
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s, load {open('/proc/loadavg').read().split()[0]}) {tail}")
        if not ident_done and not any(j["ident"] for j in queue) and not any(n.startswith(tuple(IDENT)) for n in running):
            r = ident()
            log(f"IDENTITY {'OK' if r['all_identical'] else 'FAILED'}: " + json.dumps(r["checks"]))
            if not r["all_identical"]:
                log(f"STOP: re-run base is not identical to runs/zupt_fixA base ({len(queue)} jobs not started)")
                return
            ident_done = True
        while queue and n_running() < PAR:
            j = queue[0]
            if not j["ident"] and not ident_done:
                break                                  # main matrix waits for the identity check
            queue.pop(0)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
            log(f"start {j['name']} ({len(queue)} queued)")
            time.sleep(0.5)
        time.sleep(10)
    log("end (queue done)")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "job":
        return job(sys.argv[2:])
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "list", "ident"])
    o = ap.parse_args()
    if o.cmd == "run":
        return run_sched()
    if o.cmd == "ident":
        print(json.dumps(ident(), indent=1))
        return
    for j in jobs():
        print(j["name"], " ".join(cmd(j)))


if __name__ == "__main__":
    main()
