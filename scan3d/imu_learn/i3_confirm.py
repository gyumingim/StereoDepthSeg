#!/usr/bin/env python3
"""i3 confirmation on new unseen recordings, 2026-10-21 night (docs/i3_confirm_plan.md: prereg ad2c02c, amendment 1
032a4d6, amendment 2 9d58cac). base vs i3 (--still-hold 1.5:0.05:0.05:i3), raw IMU, static init 1.0 s / 15 px / dyn off,
full length, 3 start offsets inside the IMU-detected start still; same bench path as z2_isolate (c5_run + diag bridge),
OpenVINS num_opencv_threads 2, <= PAR (5) runs at once. Recordings are picked up as their GT finishes
("<rec> done" in runs/overnight2_gt.log, written by imu_learn/overnight_gt.sh).

  (setsid nohup ../venv_ffs/bin/python imu_learn/i3_confirm.py run > runs/i3_confirm/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/i3_confirm.py inputs hs_20261021_213341     # still / offsets / GT quality only
  ../venv_ffs/bin/python imu_learn/i3_confirm.py analyse                        # -> runs/i3_confirm/summary.json
  ../venv_ffs/bin/python imu_learn/i3_confirm.py html                           # -> docs/i3_confirm_result.html
"""
import argparse
import csv
import fcntl
import gzip
import html as H
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
import z2_regress as zr  # noqa: E402

PY = "../venv_ffs/bin/python"
OUT = Path("runs/i3_confirm")
LOG = Path("runs/i3_confirm.log")
GTLOG = Path("runs/overnight2_gt.log")
RECS = ["hs_20261021_213341", "hs_20261021_213955", "hs_20261021_214550", "hs_20261021_215348"]
PAR = int(os.environ.get("PAR", 5))
THREADS = "2"
CONDS = {"base": "", "i3": "1.5:0.05:0.05:i3"}
FILES = ("imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/imu_gt.py", "imu_learn/z2_regress.py",
         "imu_learn/z2_isolate.py", "imu_learn/i3_confirm.py", "imu_learn/c5_overnight_analyse.py",
         "make_ov_config.py", "config/mono/estimator_config.yaml")
WIN, STEP, STD_T, MEAN_T, MIN_STILL = 0.5, 0.1, 0.01, 0.03, 1.5      # amendment 1 still rule
START_SEARCH_S = 10.0
SETDOWN_GUIDE = [(50.0, 75.0), (115.0, 1e9)]                          # amendment 2 reference windows (s after 1st frame)
GT_OK_MIN = 0.5
GATE_NC = 12
SHARE_MAX, VMAX_CMS = 0.005, 3.0
SD_TRIM, SD_GT_MIN, SD_TIE_CM = 0.5, 5, 0.05


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} i3c: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def blobs():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    b = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in FILES}
    dirty = subprocess.run(["git", "status", "--porcelain", *FILES], capture_output=True, text=True).stdout.strip()
    return head, b, bool(dirty)


# ---------------------------------------------------------------- inputs (no VIO)
def still_runs(t, g):
    """Still intervals [a_ns, b_ns] (IMU clock): 0.5 s windows every 0.1 s, per-axis std < 0.01 rad/s and |mean| < 0.03
    rad/s; consecutive still windows -> [first start, last start + 0.5 s]; >= 1.5 s only. Windows with < 20 samples
    count as not still."""
    ts = (t - t[0]) / 1e9
    starts = np.arange(0, ts[-1] - WIN, STEP)
    lo = np.searchsorted(ts, starts)
    hi = np.searchsorted(ts, starts + WIN)
    ok = np.zeros(len(starts), bool)
    for k, (i, j) in enumerate(zip(lo, hi)):
        if j - i < 20:
            continue
        w = g[i:j]
        ok[k] = bool(np.all(w.std(0) < STD_T) and np.linalg.norm(w.mean(0)) < MEAN_T)
    runs, k = [], 0
    while k < len(ok):
        if not ok[k]:
            k += 1
            continue
        k2 = k
        while k2 + 1 < len(ok) and ok[k2 + 1]:
            k2 += 1
        a, b = starts[k], starts[k2] + WIN
        if b - a >= MIN_STILL - 1e-9:
            runs.append([int(t[0] + a * 1e9), int(t[0] + b * 1e9)])
        k = k2 + 1
    return runs


def inputs(rec):
    from imu_gt import read_imu
    R = Path("recordings") / rec
    t_frames = np.array([int(r["t_mid_ns"]) for r in csv.DictReader(open(R / "frames_cam5.csv"))])
    al = json.load(open(R / "imu_gt" / "align.json"))
    dt = int(al["dt_ns"])
    f0 = int(t_frames[0] - dt)                                  # first frame on the IMU clock
    _, gyr = read_imu(R / "imu.csv", raw=True)
    runs = still_runs(gyr[:, 0].astype(np.int64), gyr[:, 1:4])
    out = dict(made=time.strftime("%F %T"), rec=rec, dt_ns=dt, first_frame_imu_ns=f0,
               last_frame_imu_ns=int(t_frames[-1] - dt), stills_s=[[round((a - f0) / 1e9, 2), round((b - f0) / 1e9, 2)]
                                                                   for a, b in runs], stills_imu_ns=runs)
    bp = R / "board_pick.json"
    out["board_pick"] = json.load(open(bp)) if bp.exists() else None
    g = np.load(R / "imu_gt_rs" / "gt.npz", allow_pickle=True)
    okr = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    out["gt_rs_ok_share"] = round(float(okr.mean()), 4)
    out["gt_square_mm"] = float(g["square_mm"]) if "square_mm" in g.files else None
    st = next(([a, b] for a, b in runs if a <= f0 + START_SEARCH_S * 1e9 and b > f0), None)
    out["start_still_imu_ns"] = st
    excl = []
    if out["gt_rs_ok_share"] < GT_OK_MIN:
        excl.append(f"GT pose ok share {out['gt_rs_ok_share']} < {GT_OK_MIN}")
    if st is None:
        excl.append("no start still (>= 1.5 s, starting within 10 s of the first frame)")
        out["start_still_len_s"] = None
    else:
        L = (st[1] - max(st[0], f0)) / 1e9
        out["start_still_len_s"] = round(L, 3)
        if L < MIN_STILL - 1e-9:
            excl.append(f"start still after the first frame {L:.2f} s < 1.5 s")
        else:
            base = max(0.0, (st[0] - f0) / 1e9)
            d = min(1.0, (L - 1.0) / 2)
            out["offsets_s"] = [round(base + k * d, 3) for k in range(3)]
            out["offset_step_s"] = round(d, 3)
    sd = []
    for g0, g1 in SETDOWN_GUIDE:
        cand = [[a, b] for a, b in runs if (b - f0) / 1e9 > g0 and (a - f0) / 1e9 < g1 and [a, b] != st]
        best = max(cand, key=lambda x: x[1] - x[0]) if cand else None
        sd.append(dict(guide_s=[g0, min(g1, round((out["last_frame_imu_ns"] - f0) / 1e9, 2))], still_imu_ns=best,
                       still_s=None if best is None else [round((best[0] - f0) / 1e9, 2), round((best[1] - f0) / 1e9, 2)]))
    out["setdown"] = sd
    out["excluded"] = excl
    (OUT / "inputs").mkdir(parents=True, exist_ok=True)
    json.dump(out, open(OUT / "inputs" / f"{rec}.json", "w"), indent=1)
    return out


# ---------------------------------------------------------------- runs
def jobs_for(rec, inp):
    js = []
    for k, off in enumerate(inp.get("offsets_s") or []):
        for c, hold in CONDS.items():
            js.append(dict(rec=rec, cond=c, k=k, off=off, hold=hold, name=f"{rec[-6:]}_{c}_o{k}",
                           args=zr.common(rec, off, 2.0, 0, True)))
    return js


def cmd(j):
    return [PY, "imu_learn/z2_isolate.py", "job", "--cfg-over", f"num_opencv_threads={THREADS}", "--still-hold",
            j["hold"], "--", *j["args"], "--out", str(OUT / j["name"])]


def gt_state(rec):
    if not GTLOG.exists():
        return None
    lines = [ln for ln in GTLOG.read_text(errors="replace").splitlines() if f" {rec} " in ln + " "]
    R = Path("recordings") / rec
    if any(f"{rec} done" in ln for ln in lines) and (R / "imu_gt_spl" / "gt.npz").exists() and \
            (R / "imu_gt_rs" / "gt.npz").exists() and (R / "imu_gt" / "align.json").exists():
        return "done"
    if any("FAILED" in ln and "spline" not in ln for ln in lines):
        return "failed: " + next(ln for ln in lines if "FAILED" in ln)[-200:]
    return None


def n_running():
    r = subprocess.run(["pgrep", "-fc", f"z2_isolate.py job .*--out {OUT}/"], capture_output=True, text=True)
    return int(r.stdout.strip() or 0)


def run_sched():
    OUT.mkdir(parents=True, exist_ok=True)
    head, b, dirty = blobs()
    if dirty:
        log(f"REFUSED: code files not committed (git {head}) — commit first (one code version)")
        sys.exit(1)
    log(f"start git {head} PAR {PAR} threads {THREADS} conds {CONDS} blobs {b}")
    todo, queue, running, state = list(RECS), [], {}, {}
    while todo or queue or running:
        for rec in list(todo):
            s = gt_state(rec)
            if s is None:
                continue
            todo.remove(rec)
            if s != "done":
                state[rec] = s
                log(f"{rec} EXCLUDED (GT {s})")
                continue
            try:
                inp = inputs(rec)
            except Exception as e:  # noqa: BLE001
                log(f"{rec} inputs FAILED {e!r}")
                continue
            log(f"{rec} inputs: start still {inp.get('start_still_len_s')} s offsets {inp.get('offsets_s')} "
                f"setdown {[x['still_s'] for x in inp['setdown']]} gt_ok {inp['gt_rs_ok_share']} "
                f"board {(inp.get('board_pick') or {}).get('board')} excluded {inp['excluded']}")
            if not inp["excluded"]:
                queue += [j for j in jobs_for(rec, inp) if not (OUT / f"{j['name']}.json").exists()]
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                d.update(i3c_head=head, i3c_blobs=b, i3c_dirty=dirty)
                json.dump(d, open(OUT / f"{n}.json", "w"), indent=1, ensure_ascii=False)
                tail = (f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} scale {d.get('scale')} held "
                        f"{d.get('frames_held')} waitTO {d.get('hold_wait_timeouts')} err {d.get('error')}")
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s, load {open('/proc/loadavg').read().split()[0]}) {tail}")
        with open(OUT / ".sched.lock", "w") as lk:
            fcntl.flock(lk, fcntl.LOCK_EX)
            while queue and n_running() < PAR:
                j = queue.pop(0)
                if (OUT / f"{j['name']}.json").exists():
                    continue
                err = open(OUT / f"{j['name']}.err", "w")
                running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL,
                                                       stderr=err), time.time())
                log(f"start {j['name']} off {j['off']} ({len(queue)} queued) {j['hold'] or 'base'}")
                time.sleep(0.5)
        time.sleep(10)
    log(f"end (queue done) excluded {state}")


# ---------------------------------------------------------------- analysis
def gt_gated_speed(rec):
    """GT speed exactly as zr.gt_speed (+-0.1 s central difference on imu_gt_spl, ok & |p| < 1.2, neighbours within
    0.02 s) plus the amendment gate: the sample and both neighbours have n_corners >= 12."""
    g = np.load(Path("recordings") / rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    t, p, nc = g["t_ns"][ok], g["p_bc"][ok], g["n_corners"][ok]
    ts = t / 1e9
    ja = np.clip(np.searchsorted(ts, ts + zr.V_HALF), 1, len(ts) - 1)
    jb = np.clip(np.searchsorted(ts, ts - zr.V_HALF), 1, len(ts) - 1)
    ja = np.where(np.abs(ts[ja - 1] - (ts + zr.V_HALF)) < np.abs(ts[ja] - (ts + zr.V_HALF)), ja - 1, ja)
    jb = np.where(np.abs(ts[jb - 1] - (ts - zr.V_HALF)) < np.abs(ts[jb] - (ts - zr.V_HALF)), jb - 1, jb)
    good = (np.abs(ts[ja] - ts - zr.V_HALF) < zr.GT_TOL) & (np.abs(ts[jb] - ts + zr.V_HALF) < zr.GT_TOL)
    v = np.full(len(t), np.nan)
    v[good] = np.linalg.norm(p[ja[good]] - p[jb[good]], axis=1) / (ts[ja[good]] - ts[jb[good]])
    gate = good & (nc >= GATE_NC) & (nc[ja] >= GATE_NC) & (nc[jb] >= GATE_NC)
    return t, v, gate, p


def load_poses(prefix):
    P = [json.loads(x) for x in gzip.open(str(prefix) + ".poses.jsonl.gz", "rt")]
    tr = [m for m in P if m.get("status") == "tracking" and m.get("T_world_cam0")]
    t = np.array([m["t_ns"] for m in tr], np.int64)
    X = np.array([np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3] for m in tr]) if tr else np.zeros((0, 3))
    t_last = max((m["t_ns"] for m in P if "t_ns" in m), default=None)
    return t, X, t_last


def run_metrics(prefix, inp, gts):
    import c5_overnight_analyse as an
    d = json.load(open(str(prefix) + ".json"))
    r = dict(name=Path(prefix).name, start_off=d.get("start_off"), ate_cm=d.get("ate_cm"), scale=d.get("scale"),
             t_init_clip_s=d.get("t_init_clip_s"), frames_held=d.get("frames_held") or 0,
             hold_wait_timeouts=d.get("hold_wait_timeouts"), hold_init_ns=d.get("hold_init_ns"),
             still_hold=d.get("still_hold"), cfg_over=d.get("cfg_over"), error=d.get("error"),
             threads=d.get("cfg_over"), tracked_frac=d.get("tracked_frac"),
             version=dict(bench=d.get("bench_blob"), runner=d.get("runner_blob"),
                          bin=sorted((d.get("binary_sha256") or {}).values()), i3c=d.get("i3c_blobs"),
                          ov_cfg_sha=None), wall_s=d.get("wall_s"))
    a = an.analyse_run(str(prefix))
    r.update(initialized=a.get("initialized"), scale10=a.get("scale10"), ate10_cm=a.get("ate10_cm"),
             win_collapsed=a.get("collapsed"), label_window_s=a.get("label_window_s"))
    if not a.get("initialized"):
        r["status"] = "noinit"
    elif r["ate_cm"] is None or r["scale"] is None:
        r["status"] = "unscored"
    elif a.get("collapsed") or not (0.5 <= r["scale"] <= 2) or r["ate_cm"] > 10:
        r["status"] = "collapse"
    else:
        r["status"] = "ok"
    dt = int(round(d["align_dt_ms"] * 1e6))
    t_p, X, t_last = load_poses(prefix)
    # criterion 3: held intervals
    gt_t, gv, gate, gt_p = gts
    segs = []
    for s, e in d.get("hold_segments_imu_ns") or []:
        e2 = t_last if e is None else e
        m = (gt_t >= s + dt) & (gt_t < e2 + dt)
        mv = m & np.isfinite(gv)
        mg = mv & gate
        segs.append(dict(s_ns=s, e_ns=e, dur_s=round((e2 - s) / 1e9, 3), gt_n=int(m.sum()), speed_n=int(mv.sum()),
                         gated_n=int(mg.sum()), gated_fast=int((gv[mg] > zr.V_MOVE).sum()),
                         gated_vmax_cms=round(float(gv[mg].max()) * 100, 3) if mg.any() else None,
                         ungated_vmax_cms=round(float(gv[mv].max()) * 100, 3) if mv.any() else None,
                         s_rec_s=round((s - inp["first_frame_imu_ns"]) / 1e9, 2)))
    r["held_segs"] = segs
    # criterion 4: set-down stills
    sdw = []
    for w in inp["setdown"]:
        st = w["still_imu_ns"]
        if st is None:
            sdw.append(dict(still_s=None, tested=False, why="no IMU still"))
            continue
        w0, w1 = st[0] + SD_TRIM * 1e9, st[1] - SD_TRIM * 1e9
        ng = int(((gt_t >= w0 + dt) & (gt_t <= w1 + dt)).sum())
        m = (t_p >= w0) & (t_p <= w1)
        x = dict(still_s=w["still_s"], gt_n=ng, poses_n=int(m.sum()))
        if ng < SD_GT_MIN or m.sum() < 2:
            x.update(tested=False, why=f"gt {ng} poses {int(m.sum())}")
        else:
            P = X[m]
            gm = (gt_t >= w0 + dt) & (gt_t <= w1 + dt)
            x.update(tested=True, drift_cm=round(float(np.linalg.norm(P[-1] - P[0])) * 100, 3),
                     path_cm=round(float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum()) * 100, 3),
                     gt_drift_cm=round(float(np.linalg.norm(gt_p[gm][-1] - gt_p[gm][0])) * 100, 3))
        sdw.append(x)
    r["setdown"] = sdw
    return r


def med(xs):
    xs = [x for x in xs if x is not None]
    return float(np.median(xs)) if xs else None


def analyse():
    res = dict(made=time.strftime("%F %T"), git=blobs()[0], blobs=blobs()[1], recs={}, criteria={})
    allruns = []
    for rec in RECS:
        f = OUT / "inputs" / f"{rec}.json"
        if not f.exists():
            res["recs"][rec] = dict(state="no inputs (GT not done or failed)", excluded=["GT not available"])
            continue
        inp = json.load(open(f))
        R = dict(inputs={k: inp.get(k) for k in ("start_still_len_s", "offsets_s", "offset_step_s", "stills_s",
                                                  "gt_rs_ok_share", "gt_square_mm", "excluded")},
                 setdown_stills=[w["still_s"] for w in inp["setdown"]],
                 board=(inp.get("board_pick") or {}).get("board"), excluded=inp["excluded"])
        if inp["excluded"]:
            res["recs"][rec] = R
            continue
        import input_events  # amendment 3 (post-hoc): raw-accel drop / impact detection
        R["input_event"] = input_events.detect(rec)
        gts = gt_gated_speed(rec)
        runs = {}
        for j in jobs_for(rec, inp):
            p = OUT / j["name"]
            if Path(str(p) + ".json").exists():
                runs[(j["cond"], j["k"])] = run_metrics(p, inp, gts)
        R["runs"] = {f"{c}_o{k}": x for (c, k), x in runs.items()}
        allruns += list(runs.values())
        complete = all((c, k) in runs for c in CONDS for k in range(3))
        R["complete"] = complete
        b = [runs.get(("base", k)) for k in range(3)]
        z = [runs.get(("i3", k)) for k in range(3)]
        if not complete:
            res["recs"][rec] = R
            continue
        # criterion 1
        bm = med([x["ate_cm"] for x in b if x["status"] in ("ok", "collapse")])
        zm = med([x["ate_cm"] for x in z if x["status"] in ("ok", "collapse")])
        lim = None if bm is None or zm is None else max(bm * 1.10, bm + 0.2)
        dd = [(f"o{k}", round(zz["ate_cm"] - bb["ate_cm"], 2)) for k, (bb, zz) in enumerate(zip(b, z))
              if bb["ate_cm"] is not None and zz["ate_cm"] is not None]
        c1 = dict(base_ates=[x["ate_cm"] for x in b], i3_ates=[x["ate_cm"] for x in z], base_med=bm, i3_med=zm, lim=lim,
                  ok=None if lim is None else bool(zm <= lim + 1e-9), worst_pair=max(dd, key=lambda x: x[1]) if dd else None,
                  pairs_over=[f"o{k}" for k, (bb, zz) in enumerate(zip(b, z)) if bb["ate_cm"] is not None and
                              zz["ate_cm"] is not None and zz["ate_cm"] > max(bb["ate_cm"] * 1.1, bb["ate_cm"] + 0.2) + 1e-9])
        # criterion 2
        new_bad = [f"o{k}: base {bb['status']} -> i3 {zz['status']}" for k, (bb, zz) in enumerate(zip(b, z))
                   if bb["status"] == "ok" and zz["status"] != "ok"]
        c2 = dict(base_status=[x["status"] for x in b], i3_status=[x["status"] for x in z], new_collapses=new_bad,
                  ok=not new_bad)
        # criterion 3
        segs = [dict(s, run=x["name"]) for x in z for s in x["held_segs"]]
        held = sum(x["frames_held"] for x in z)
        n = sum(s["gated_n"] for s in segs)
        fast = sum(s["gated_fast"] for s in segs)
        vmax = max([s["gated_vmax_cms"] for s in segs if s["gated_vmax_cms"] is not None], default=None)
        untested = [f"{s['run']} @{s['s_rec_s']} s ({s['dur_s']} s)" for s in segs if s["gated_n"] == 0]
        if held == 0:
            v3 = "n/a (0 held)"
        elif n and (fast / n > SHARE_MAX + 1e-12 or vmax > VMAX_CMS + 1e-9):
            v3 = "fail"
        elif untested:
            v3 = "partly not tested"
        elif n == 0:
            v3 = "not tested"
        else:
            v3 = "pass"
        c3 = dict(frames_held=[x["frames_held"] for x in z], n_segs=len(segs), gated_n=n, gated_fast=fast,
                  share=None if not n else round(fast / n, 5), vmax_cms=vmax, untested=untested, verdict=v3,
                  ungated_vmax_cms=max([s["ungated_vmax_cms"] for s in segs if s["ungated_vmax_cms"] is not None],
                                       default=None), base_held=[x["frames_held"] for x in b])
        # criterion 4
        wins = []
        for wi in range(len(inp["setdown"])):
            bw = [x["setdown"][wi] for x in b]
            zw = [x["setdown"][wi] for x in z]
            bd = [w["drift_cm"] for w in bw if w.get("tested")]
            zd = [w["drift_cm"] for w in zw if w.get("tested")]
            wins.append(dict(still_s=inp["setdown"][wi]["still_s"], base_drifts=[w.get("drift_cm") for w in bw],
                             i3_drifts=[w.get("drift_cm") for w in zw], base_med=med(bd), i3_med=med(zd),
                             gt_drift=[w.get("gt_drift_cm") for w in bw],
                             tested=bool(bd and zd), why=[w.get("why") for w in bw + zw if not w.get("tested")]))
        tw = [w for w in wins if w["tested"]]
        if tw:
            sb, sz = sum(w["base_med"] for w in tw), sum(w["i3_med"] for w in tw)
            sign = "+" if sz < sb - SD_TIE_CM else ("-" if sz > sb + SD_TIE_CM else "=")
        else:
            sb = sz = None
            sign = "not tested"
        c4 = dict(windows=wins, base_sum=sb, i3_sum=sz, sign=sign)
        R.update(c1=c1, c2=c2, c3=c3, c4=c4)
        res["recs"][rec] = R
    judged = [r for r, x in res["recs"].items() if "c1" in x]
    # amendment 3 (post-hoc): input-event recordings leave criteria 1/3/4, criterion 2 still counts all judged ones
    events = [r for r in judged if res["recs"][r]["input_event"]["event"]]
    agg = [r for r in judged if r not in events]
    c1_fail = [r for r in agg if res["recs"][r]["c1"]["ok"] is False]
    c1_none = [r for r in agg if res["recs"][r]["c1"]["ok"] is None]
    c2_new = [f"{r} {s}" for r in judged for s in res["recs"][r]["c2"]["new_collapses"]]
    c3v = {r: res["recs"][r]["c3"]["verdict"] for r in agg}
    signs = {r: res["recs"][r]["c4"]["sign"] for r in agg}
    npl, nmi = sum(s == "+" for s in signs.values()), sum(s == "-" for s in signs.values())
    nt4 = sum(s in "+-=" for s in signs.values())
    c4v = "not tested" if nt4 == 0 else ("improve" if npl > nmi else "not improve")
    ok1 = bool(agg) and not c1_fail and not c1_none
    ok2 = bool(judged) and not c2_new
    ok3 = bool(agg) and all(v in ("pass", "n/a (0 held)") for v in c3v.values())
    reasons = []
    if not judged:
        reasons.append("판정할 녹화 없음")
    elif not agg:
        reasons.append("입력 사건을 빼면 기준 1·3·4 를 판정할 녹화 없음")
    if c1_fail:
        reasons.append(f"기준 1 실패 {c1_fail}")
    if c1_none:
        reasons.append(f"기준 1 계산 불가 {c1_none}")
    if c2_new:
        reasons.append(f"기준 2 새 무너짐 {c2_new}")
    for r, v in c3v.items():
        if v not in ("pass", "n/a (0 held)"):
            reasons.append(f"기준 3 {r[-6:]}: {v}")
    if c4v != "improve":
        reasons.append(f"기준 4: {c4v} (+{npl} / -{nmi})")
    if judged and len(agg) < 3:
        reasons.append(f"근거 약함: 입력 사건을 빼고 남은 녹화 {len(agg)} 개 (< 3)")
    res["criteria"] = dict(judged=judged, n=len(judged), events=events, agg=agg, n_agg=len(agg), weak=len(agg) < 3,
                           c1_ok=ok1, c1_fail=c1_fail, c1_none=c1_none, c2_ok=ok2,
                           c2_new=c2_new, c3_ok=ok3, c3=c3v, c4=c4v, c4_signs=signs, c4_plus=npl, c4_minus=nmi,
                           c4_tested=nt4, c4_weak=nt4 < 3, reasons=reasons,
                           adopt=bool(ok1 and ok2 and ok3 and c4v == "improve"))
    vers = {}
    for x in allruns:
        vers.setdefault(json.dumps({k: v for k, v in x["version"].items()}, sort_keys=True), []).append(x["name"])
    res["code_versions"] = len(vers)
    res["threads"] = sorted({str(x["threads"]) for x in allruns})
    res["wait_timeouts"] = {x["name"]: x["hold_wait_timeouts"] for x in allruns if x.get("hold_wait_timeouts")}
    res["n_runs"] = len(allruns)
    # i3 arming: held frames before init + 3 s must be 0 (bench rule) -> earliest held time after init
    res["arm_check"] = {x["name"]: round((min(s["s_ns"] for s in x["held_segs"]) - x["hold_init_ns"]) / 1e9, 3)
                        for x in allruns if x["held_segs"] and x.get("hold_init_ns")}
    return res


# ---------------------------------------------------------------- HTML
def f_(x, nd=2):
    if x is None:
        return "–"
    if isinstance(x, float):
        return f"{x:.{nd}f}"
    return H.escape(str(x))


def write_html(res, path, check=None):
    c = res["criteria"]
    N = c.get("n", 0)
    NA = c.get("n_agg", N)
    concl = (f"앱 설정에 넣어도 됨 (추정, 녹화 {NA} 개)" if c.get("adopt") else
             f"앱 설정에 넣지 않음 (녹화 {N} 개로 판정, 기준 1·3·4 는 입력 사건을 뺀 {NA} 개)")
    if c.get("weak") and N:
        concl += f" — 근거 약함 (입력 사건을 빼고 남은 녹화 {NA} 개 < 3)"
    v3 = {"pass": "통과", "fail": "실패", "partly not tested": "일부 시험 안 됨 (통과 아님)", "not tested": "시험 안 됨",
          "n/a (0 held)": "붙잡은 프레임 없음"}
    o = ["<!doctype html><html lang='ko'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>",
         "<title>i3 확인 시험</title><style>body{font-family:system-ui,'Malgun Gothic',sans-serif;max-width:980px;margin:0 auto;"
         "padding:16px;line-height:1.55;color:#1d232b;background:#fff}h1{font-size:1.5em}h2{margin-top:1.6em;font-size:1.15em}"
         "table{border-collapse:collapse;width:100%;font-size:.88em;margin:.5em 0}td,th{border:1px solid #ccd;padding:4px 6px;"
         "text-align:left;vertical-align:top}th{background:#f1f3f7}.big{font-size:1.25em;font-weight:700;padding:12px;"
         "border-radius:8px}.yes{background:#e3f6e8}.no{background:#fde8e6}.mut{color:#5b6573;font-size:.9em}"
         ".ok{color:#17803d;font-weight:600}.bad{color:#b42318;font-weight:600}.wrap{overflow-x:auto}</style></head><body>"]
    o.append("<h1>i3 (가만히 있을 때 화면 고정, 초기화 3 초 뒤부터) 확인 시험 — 새 녹화</h1>")
    o.append(f"<div class='big {'yes' if c.get('adopt') else 'no'}'>결론: {H.escape(concl)}</div>")
    if not c.get("adopt"):
        o.append("<p>이유: " + "; ".join(H.escape(x) for x in c.get("reasons", [])) + "</p>")
    o.append("<ul>")
    o.append(f"<li>기준 1 (정확도가 10 % 또는 0.2 cm 넘게 나빠지지 않음): <b class='{'ok' if c.get('c1_ok') else 'bad'}'>"
             f"{'통과' if c.get('c1_ok') else '실패'}</b></li>")
    o.append(f"<li>기준 2 (새로 무너진 실행 0): <b class='{'ok' if c.get('c2_ok') else 'bad'}'>"
             f"{'통과' if c.get('c2_ok') else '실패 ' + H.escape(str(c.get('c2_new')))}</b></li>")
    o.append(f"<li>기준 3 (화면을 고정한 동안 실제로 안 움직였나, 정답 품질 문 통과 표본만): <b class='{'ok' if c.get('c3_ok') else 'bad'}'>"
             f"{'통과' if c.get('c3_ok') else '통과 아님'}</b> — " +
             ", ".join(f"{r[-6:]} {v3.get(v, v)}" for r, v in (c.get('c3') or {}).items()) + "</li>")
    c4t = {"improve": "개선 쪽", "not improve": "개선 아님", "not tested": "시험 안 됨"}.get(c.get("c4"), c.get("c4"))
    o.append(f"<li>기준 4 (내려놓은 뒤 VIO 가 덜 미끄러지나): <b class='{'ok' if c.get('c4') == 'improve' else 'bad'}'>{H.escape(str(c4t))}</b>"
             f" (녹화 개선 {c.get('c4_plus')} / 나빠짐 {c.get('c4_minus')} / 시험된 녹화 {c.get('c4_tested')})"
             f"{' — 녹화 수가 적어 근거 약함' if c.get('c4_weak') else ''}</li></ul>")
    o.append("<p class='mut'>추정: 녹화 몇 개로 본 결과라 다른 장면·다른 사람 손에서는 달라질 수 있다. 판정 기준 1–4 는 결과를 보기 전에 정해 둔 "
             "docs/i3_confirm_plan.md (사전 등록 ad2c02c, 추가 1 032a4d6, 추가 2 9d58cac) 그대로.</p>")
    ev = c.get("events") or []
    o.append("<p><b>주의 — 추가 3 은 사후 규칙이다.</b> 이 규칙(녹화 중 낙하 같은 입력 사건이 있는 녹화는 기준 1·3·4 집계에서 빼고 "
             "참고 표에만 적음, 기준 2 는 그대로 셈)은 <b>결과 일부(213341·213955)를 본 뒤</b>, 214550·215348 결과를 보기 전에 정했다 "
             "(docs/i3_confirm_plan.md 추가 3, f27d457; 근거 docs/collapse_213955.md). 검출: 원본 가속도(type 35), 시작 정지 끝 → 첫 내려놓기 정지 시작, "
             "|a| &gt; 39 m/s² 표본 하나 이상 또는 |a| &lt; 2 m/s² 가 0.1 s 넘게 이어짐 (imu_learn/input_events.py). "
             f"입력 사건 녹화: <b>{H.escape(', '.join(r[-6:] for r in ev) or '없음')}</b>.</p>")
    if check is not None:
        o.append(f"<p class='mut'>검산 (i3_confirm_check.py, 분석 코드를 쓰지 않고 다시 계산): "
                 f"<b>{'일치' if not check.get('mismatch') else '불일치 ' + H.escape(str(check.get('mismatch'))[:400])}</b>"
                 f" — 비교 {check.get('n_checked')} 항목.</p>")
    head = ("<div class='wrap'><table><tr><th>녹화</th><th>보드</th><th>시작 정지 (s)</th><th>시작 위치 (s)</th>"
            "<th>base ATE (cm) 3 개 → 중앙값</th><th>i3 ATE (cm) → 중앙값 / 한계</th><th>기준 1</th><th>상태 base → i3 (기준 2)</th>"
            "<th>기준 3: 문 통과 표본 / 빠름 / 최대 (cm/s)</th><th>기준 4: 내려놓은 뒤 이동 합 base → i3 (cm)</th></tr>")

    def row(rec, R):
        inp = R.get("inputs") or {}
        bd = Path(R.get("board") or "").name or "–"
        if "c1" not in R:
            return (f"<tr><td>{rec[-6:]}</td><td>{H.escape(bd)}</td><td>{f_(inp.get('start_still_len_s'))}</td>"
                    f"<td colspan='7'>판정 안 함: {H.escape('; '.join(R.get('excluded') or []) or ('실행 미완료' if R.get('runs') is not None else R.get('state', '')))}</td></tr>")
        c1, c2, c3, c4 = R["c1"], R["c2"], R["c3"], R["c4"]
        return (f"<tr><td>{rec[-6:]}</td><td>{H.escape(bd)}</td><td>{f_(inp.get('start_still_len_s'))}</td>"
                f"<td>{f_(inp.get('offsets_s'))}</td><td>{f_(c1['base_ates'])} → {f_(c1['base_med'])}</td>"
                f"<td>{f_(c1['i3_ates'])} → {f_(c1['i3_med'])} / {f_(c1['lim'])}</td>"
                f"<td class='{'ok' if c1['ok'] else 'bad'}'>{'통과' if c1['ok'] else '실패'} (짝 최악 {f_(c1['worst_pair'])})</td>"
                f"<td>{f_(c2['base_status'])} → {f_(c2['i3_status'])}</td>"
                f"<td>{c3['gated_n']} / {c3['gated_fast']} / {f_(c3['vmax_cms'])} — {H.escape(v3.get(c3['verdict'], c3['verdict']))}"
                f"{'<br><span class=mut>시험 안 된 구간: ' + H.escape(', '.join(c3['untested'][:6])) + '</span>' if c3['untested'] else ''}</td>"
                f"<td>{f_(c4['base_sum'])} → {f_(c4['i3_sum'])} ({H.escape(c4['sign'])})</td></tr>")

    o.append("<h2>녹화별 (판정에 씀)</h2>" + head)
    for rec, R in res["recs"].items():
        if rec not in ev:
            o.append(row(rec, R))
    o.append("</table></div>")
    if ev:
        o.append("<h2>입력 사건 녹화 (참고용 — 기준 1·3·4 판정에 안 씀, 기준 2 는 셈)</h2>" + head)
        for rec in ev:
            o.append(row(rec, res["recs"][rec]))
        o.append("</table></div>")
    o.append("<h2>입력 사건 검출 (추가 3, 사후 규칙)</h2><div class='wrap'><table><tr><th>녹화</th><th>검사 창 (s, 녹화 시작 기준)</th>"
             "<th>입력 사건?</th><th>사건 시각 (s)</th><th>최대 |a| (m/s²) @ s</th><th>가장 긴 자유낙하 (s)</th></tr>")
    for rec, R in res["recs"].items():
        e = R.get("input_event")
        if not e:
            continue
        o.append(f"<tr><td>{rec[-6:]}</td><td>{f_(e['window_s_meta'])}</td>"
                 f"<td class='{'bad' if e['event'] else 'ok'}'>{'예' if e['event'] else '아니오'} {H.escape('; '.join(e['reasons']))}</td>"
                 f"<td>{H.escape(', '.join(f'{a:.2f}–{b:.2f}' for a, b in e['event_clusters_meta']) or '–')}</td>"
                 f"<td>{f_(e['max_abs_a'])} @ {f_(e['max_abs_a_t_meta'])}</td><td>{f_(e['longest_freefall_s'], 3)}</td></tr>")
    o.append("</table></div><p class='mut'>시각은 meta.start_elapsed_ns 기준 (docs/collapse_213955.md 와 같은 시계).</p>")
    o.append("<h2>내려놓은 뒤 정지 구간 (IMU 로 찾음)</h2><div class='wrap'><table><tr><th>녹화</th><th>정지 (s)</th><th>base 이동 3 개 (cm)</th>"
             "<th>i3 이동 3 개 (cm)</th><th>정답 이동 (cm)</th><th>시험?</th></tr>")
    for rec, R in res["recs"].items():
        for w in (R.get("c4") or {}).get("windows", []):
            o.append(f"<tr><td>{rec[-6:]}{' (참고, 입력 사건)' if rec in ev else ''}</td><td>{f_(w['still_s'])}</td><td>{f_(w['base_drifts'])}</td><td>{f_(w['i3_drifts'])}</td>"
                     f"<td>{f_(w['gt_drift'])}</td><td>{'예' if w['tested'] else '아니오 ' + H.escape(str(w['why'][:2]))}</td></tr>")
    o.append("</table></div>")
    o.append("<h2>실행 점검</h2><ul>")
    o.append(f"<li>실행 수 {res.get('n_runs')}, 코드 버전 {res.get('code_versions')} 개 (1 이어야 함), OpenVINS 스레드 {f_(res.get('threads'))}, "
             f"i3 대기 시간 초과 {f_(res.get('wait_timeouts') or '없음')}</li>")
    am = res.get("arm_check") or {}
    o.append(f"<li>i3 가 처음 고정한 시각 − 초기화 시각 (s, 3 이상이어야 함): {H.escape(', '.join(f'{k}: {v}' for k, v in am.items()) or '–')}</li>")
    o.append(f"<li>git {H.escape(str(res.get('git')))}, 분석 {H.escape(res.get('made', ''))}</li></ul>")
    o.append("<p class='mut'>용어: ATE = 정답 궤적과 맞춘 뒤 남는 위치 오차 (작을수록 좋음). 무너짐 = 초기화 뒤 10 초 축척이 0.5–2 밖이거나 오차 10 cm 넘음 "
             "(또는 전체가 그러함). 붙잡은 구간 = i3 가 '가만히 있다' 고 보고 같은 화면을 다시 넣은 시간. 정답 품질 문 = 그 시각과 ±0.1 초에 "
             "보드 모서리가 12 개 이상 보인 정답만 씀. 코드 imu_learn/i3_confirm.py, 검산 imu_learn/i3_confirm_check.py, 결과 runs/i3_confirm/.</p>")
    o.append("</body></html>")
    Path(path).write_text("\n".join(o), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "inputs", "analyse", "html", "list"])
    ap.add_argument("recs", nargs="*")
    o = ap.parse_args()
    if o.cmd == "run":
        return run_sched()
    if o.cmd == "inputs":
        for r in o.recs or RECS:
            print(json.dumps({k: v for k, v in inputs(r).items() if k not in ("stills_imu_ns",)}, default=str))
        return
    if o.cmd == "list":
        for r in o.recs or RECS:
            inp = json.load(open(OUT / "inputs" / f"{r}.json"))
            for j in jobs_for(r, inp):
                print(j["name"], " ".join(cmd(j)))
        return
    if o.cmd == "analyse":
        res = analyse()
        json.dump(res, open(OUT / "summary.json", "w"), indent=1, ensure_ascii=False, default=float)
        print(json.dumps(res["criteria"], ensure_ascii=False, default=float, indent=1))
        print("code_versions", res["code_versions"], "threads", res["threads"], "timeouts", res["wait_timeouts"],
              "arm", res["arm_check"])
        return
    res = json.load(open(OUT / "summary.json"))
    chk = OUT / "check.json"
    write_html(res, "docs/i3_confirm_result.html", json.load(open(chk)) if chk.exists() else None)
    print("wrote docs/i3_confirm_result.html")


if __name__ == "__main__":
    main()
