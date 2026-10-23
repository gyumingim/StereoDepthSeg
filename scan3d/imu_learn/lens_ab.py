#!/usr/bin/env python3
"""VIO lens A/B, 2026-10-22 (docs/lens_ab_plan.md, pre-registered before any run).

A = config/lens/hs_1920x1080_f2.22.json (the file the bench picks today for focus 2.22 recordings, rational, fx 1267.1)
B = config/lens/hs_1920x1080_f2.22_cal20261022.json (2.22 node of hs_1920x1080_focus_cal_20261022.json, joint fit, fx 1303.9)
Only vio_bench_hs.lens_path is swapped (both conditions). Bridge = tools/ov_bridge_zbase (unpatched copy of open_vins_diag,
identity-checked in zupt_fixA). Bench path = i3_confirm base: c5_run + zr.common(rec, off, 2.0, 0, raw, rebias=False),
num_opencv_threads 2, still-hold off, a.gyro_rebias = False explicit.
GT sensitivity: --gt-dir imu_gt_spl_lensB (GT of 134204 rebuilt with lens B by imu_learn/lens_ab_gt.sh) swaps vb.pick_gt.

  (setsid nohup ../venv_ffs/bin/python imu_learn/lens_ab.py run > runs/lens_ab/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/lens_ab.py list
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
OUT = Path("runs/lens_ab")
LOG = Path("runs/lens_ab.log")
PAR = int(os.environ.get("PAR", 3))                 # plan: at most 3 processes (another agent runs ZUPT fix C)
THREADS = "2"
BRIDGE = SCAN3D / "tools" / "ov_bridge_zbase" / "build" / "ov_bridge"
LENS = {"A": SCAN3D / "config" / "lens" / "hs_1920x1080_f2.22_cal20261019_old.json",   # renamed 2026-10-23, same bytes
        "B": SCAN3D / "config" / "lens" / "hs_1920x1080_f2.22_cal20261022.json"}
RECS = ["hs_20261022_111836", "hs_20261022_133820", "hs_20261022_134204", "hs_20261022_134550"]
GT_REC = "hs_20261022_134204"
GT_B = "imu_gt_spl_lensB"
FILES = ("imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/z2_regress.py", "imu_learn/lens_ab.py",
         "imu_learn/lens_ab_gt.sh", "make_ov_config.py", "config/mono/estimator_config.yaml",
         "tools/ov_bridge_zbase/main.cpp", "tools/ov_bridge_zbase/CMakeLists.txt",
         "config/lens/hs_1920x1080_f2.22_cal20261019_old.json", "config/lens/hs_1920x1080_f2.22_cal20261022.json")
# identity gate: our A run of this cell must equal the zupt_fixA base run (same bridge, same args, same lens file)
IDENT = {"L_134204_o0_A": "runs/zupt_fixA/BR_134204_o0_base"}
POSE_KEYS = ("t", "t_ns", "status", "init", "p", "q", "v", "T_world_cam0", "n_slam", "n_msckf")


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} lab: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def blobs():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    b = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in FILES}
    dirty = subprocess.run(["git", "status", "--porcelain", *FILES], capture_output=True, text=True).stdout.strip()
    return head, b, bool(dirty)


def cells():
    out = []
    for rec in RECS:
        for k, off in enumerate((0.0, 1.0, 2.0)):
            out.append(dict(key=f"L_{rec[-6:]}_o{k}", rec=rec, args=zr.common(rec, off, 2.0, 0, True, rebias=False)))
    return out


def jobs():
    cs = cells()
    js = [dict(name="GT_lensB", kind="gt", cmd=["bash", "imu_learn/lens_ab_gt.sh"], ident=False, needs_gt=False)]
    first = [c for c in cs if f"{c['key']}_A" in IDENT]
    for c in first:
        js.append(dict(name=f"{c['key']}_A", kind="vio", cond="A", args=c["args"], ident=True, needs_gt=False))
    for c in cs:
        for cond in ("A", "B"):
            n = f"{c['key']}_{cond}"
            if n in IDENT:
                continue
            js.append(dict(name=n, kind="vio", cond=cond, args=c["args"], ident=False, needs_gt=False))
    for c in cs:                                       # GT sensitivity (134204 only), after GT_lensB is built
        if c["rec"] != GT_REC:
            continue
        for cond in ("A", "B"):
            js.append(dict(name=f"{c['key']}_{cond}_gtB", kind="vio", cond=cond, args=c["args"], ident=False,
                           needs_gt=True, gt_dir=GT_B))
    return js


def cmd(j):
    if j["kind"] == "gt":
        return j["cmd"]
    extra = ["--gt-dir", j["gt_dir"]] if j.get("gt_dir") else []
    return [PY, "imu_learn/lens_ab.py", "job", "--cond", j["cond"], *extra, "--", *j["args"], "--out", str(OUT / j["name"])]


def job(argv):
    i = argv.index("--")
    ap = argparse.ArgumentParser()
    ap.add_argument("--cond", choices=list(LENS), required=True)
    ap.add_argument("--gt-dir", default=None)
    o = ap.parse_args(argv[:i])
    import c5_run
    import vio_bench_hs as vb
    c5_run.DIAG_BRIDGE = BRIDGE
    lens = LENS[o.cond]
    vb.lens_path = lambda rec, meta: lens             # the only change between A and B
    if o.gt_dir:
        orig_pick = vb.pick_gt

        def pick(rec, mode):
            d = Path(rec) / o.gt_dir
            if not (d / "gt.npz").exists():
                sys.exit(f"GT not found: {d}/gt.npz")
            return d
        vb.pick_gt = pick
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
    d.update(lab_runner="imu_learn/lens_ab.py", lab_cond=o.cond, lab_lens=str(lens), lab_gt_dir=o.gt_dir or "auto",
             lab_bridge=str(BRIDGE), lab_head=head, lab_blobs=b, lab_dirty=dirty, lab_made=time.strftime("%F %T"),
             lab_gyro_rebias_set=False, lab_cfg_over=f"num_opencv_threads={THREADS}")
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
    res = dict(made=time.strftime("%F %T"), checks=[ident_one(OUT / k, v) for k, v in IDENT.items()])
    res["all_identical"] = all(c["identical"] for c in res["checks"])
    json.dump(res, open(OUT / "ident.json", "w"), indent=1)
    return res


def n_running():
    a = subprocess.run(["pgrep", "-fc", f"lens_ab.py job .*--out {OUT}/"], capture_output=True, text=True)
    b = subprocess.run(["pgrep", "-fc", "lens_ab_gt.sh"], capture_output=True, text=True)
    return int(a.stdout.strip() or 0) + int(b.stdout.strip() or 0)


def done(j):
    if j["kind"] == "gt":
        return (SCAN3D / "recordings" / GT_REC / GT_B / "gt.npz").exists() and (OUT / "GT_lensB.done").exists()
    return (OUT / f"{j['name']}.json").exists()


def run_sched():
    OUT.mkdir(parents=True, exist_ok=True)
    head, b, dirty = blobs()
    if dirty:
        log(f"REFUSED: code files not committed (git {head})")
        sys.exit(1)
    if not BRIDGE.exists():
        log(f"REFUSED: bridge missing {BRIDGE}")
        sys.exit(1)
    queue = [j for j in jobs() if not done(j)]
    log(f"start git {head} PAR {PAR} threads {THREADS} jobs {len(queue)} blobs {b}")
    running = {}
    ident_done = (OUT / "ident.json").exists() and json.load(open(OUT / "ident.json"))["all_identical"]
    gt_ok = done(jobs()[0])
    while queue or running:
        for n, (p, t0, j) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            if j["kind"] == "gt":
                gt_ok = p.returncode == 0 and (SCAN3D / "recordings" / GT_REC / GT_B / "gt.npz").exists()
                if gt_ok:
                    (OUT / "GT_lensB.done").write_text(time.strftime("%F %T") + "\n")
                log(f"done {n} ({time.time() - t0:.0f} s) rc {p.returncode} gt_ok {gt_ok}")
                if not gt_ok:
                    queue = [q for q in queue if not q["needs_gt"]]
                    log("GT_lensB failed: GT sensitivity runs dropped")
                continue
            try:
                d = json.load(open(OUT / f"{n}.json"))
                tail = f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} sim3 {d.get('ate_sim3_cm')} scale {d.get('scale')} err {d.get('error')}"
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s, load {open('/proc/loadavg').read().split()[0]}) {tail}")
        if not ident_done and not any(j["ident"] for j in queue) and not any(n in IDENT for n in running):
            r = ident()
            log(f"IDENTITY {'OK' if r['all_identical'] else 'FAILED'}: " + json.dumps(r["checks"]))
            if not r["all_identical"]:
                log(f"STOP: A run is not identical to the zupt_fixA base run ({len(queue)} jobs not started)")
                for n, (p, _, _) in running.items():
                    log(f"(still running: {n})")
                return
            ident_done = True
        k = 0
        while k < len(queue) and n_running() < PAR:
            j = queue[k]
            if j["kind"] == "vio" and not j["ident"] and not ident_done:
                k += 1
                continue
            if j["needs_gt"] and not gt_ok:
                k += 1
                continue
            queue.pop(k)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL, stderr=err),
                                  time.time(), j)
            log(f"start {j['name']} ({len(queue)} queued)")
            time.sleep(1.0)
        if not running and queue and all(q["needs_gt"] for q in queue) and not gt_ok:
            log("only GT runs left and GT not available: stop")
            break
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
