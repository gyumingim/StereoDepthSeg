#!/usr/bin/env python3
"""Set-down failure mitigations (docs/setdown_mitigation_plan.md, 2026-10-21): full-length VIO on 224323 / 223815, raw IMU,
static-only init (strict, window 1.0 s, disparity 15 px), start offsets 0/1/2, conditions
  base | Z1 zupt_chi2_multipler=1 | Z2 + zupt_noise_multiplier=1 | Z3 zupt_chi2_multipler=5 | H --still-hold 1.0:0.05:0.05
Runs go through imu_learn/c5_run.py (diag bridge, DEBUG log); this script only adds the two bench options in-process.

  (setsid nohup ../venv_ffs/bin/python imu_learn/setdown_mitig.py run > runs/setdown_mitig/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/setdown_mitig.py analyse [--html docs/setdown_mitigation.html]
"""
import argparse
import gzip
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
PY = "../venv_ffs/bin/python"
OUT = Path("runs/setdown_mitig")
LOG = OUT / "sched.log"
RECS = ["hs_20261020_224323", "hs_20261020_223815"]
OFFS = (0, 1, 2)
PAR = int(os.environ.get("PAR", 3))
HOLD = "1.0:0.05:0.05"
CONDS = {"base": ("", None), "Z1": ("zupt_chi2_multipler=1", None),
         "Z2": ("zupt_chi2_multipler=1,zupt_noise_multiplier=1", None),
         "Z3": ("zupt_chi2_multipler=5", None), "H": ("", HOLD)}
ENDSTILL = Path("runs/endstill_tmp/endstill_check.json")
ALIGN_MAX_S = 100.0
LAST_WIN_S = 3.0
ANSI = re.compile(r"\x1b\[[0-9;]*m")
FILES = ("imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/imu_gt.py", "imu_learn/setdown_mitig.py",
         "make_ov_config.py", "config/mono/estimator_config.yaml")


def log(msg):
    line = f"{time.strftime('%F %T')} sdm: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def blobs():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    b = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in FILES}
    dirty = subprocess.run(["git", "status", "--porcelain", *FILES], capture_output=True, text=True).stdout.strip()
    return head, b, bool(dirty)


def jobs():
    out = []
    for rec in RECS:
        for cond, (over, hold) in CONDS.items():
            for o in OFFS:
                name = f"{rec[-6:]}_{cond}_o{o}"
                args = [PY, "imu_learn/setdown_mitig.py", "job", "--cfg-over", over, "--still-hold", hold or "", "--",
                        f"recordings/{rec}", "--init-cfg", "strict", "--init-window", "1.0", "--init-disp", "15",
                        "--clones", "11", "--noise", "cfg", "--start-off", str(o), "--max-secs", "0",
                        "--decode-threads", "2", "--imu-raw", "--no-gyro-rebias", "--out", str(OUT / name)]
                out.append(dict(name=name, args=args))
    return out


def job(argv):
    """In-process c5_run with the two extra bench options (c5_run builds its own Namespace)."""
    i = argv.index("--")
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg-over", default="")
    ap.add_argument("--still-hold", default="")
    o = ap.parse_args(argv[:i])
    import c5_run
    import vio_bench_hs as vb
    orig = vb.run

    def run(a):
        a.cfg_over = o.cfg_over
        a.still_hold = o.still_hold or None
        return orig(a)
    vb.run = run
    sys.argv = ["c5_run.py"] + argv[i + 1:]
    c5_run.main()
    out = sys.argv[sys.argv.index("--out") + 1]
    d = json.load(open(out + ".json"))
    head, b, dirty = blobs()
    d.update(mitig_runner="imu_learn/setdown_mitig.py", mitig_blobs=b, mitig_dirty=dirty, mitig_head=head,
             mitig_cfg_over=o.cfg_over, mitig_still_hold=o.still_hold or None)
    json.dump(d, open(out + ".json", "w"), indent=1, ensure_ascii=False)


def run_all():
    OUT.mkdir(parents=True, exist_ok=True)
    head, b, dirty = blobs()
    js = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists()]
    log(f"start git {head} dirty={dirty} PAR {PAR} jobs {len(js)} blobs {b}")
    running = {}
    while js or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                tail = (f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} scale {d.get('scale')} "
                        f"held {d.get('frames_held')} err {d.get('error')}")
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s) {tail}")
        while js and len(running) < PAR:
            j = js.pop(0)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + j["args"], stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
            log(f"start {j['name']} ({len(js)} left)")
        time.sleep(10)
    log("end (all done)")


# ---------------------------------------------------------------- analysis
def frames_from_log(prefix, t0):
    """Per processed frame (recording s): ZUPT accepted?, MSCKF+SLAM update feats, initialized yet. Bridge -> recording
    time via the pose lines (same as endstill_setdown_check.py)."""
    P = [json.loads(x) for x in gzip.open(prefix + ".poses.jsonl.gz", "rt")]
    off = float(np.median([m["t"] - (m["t_ns"] - t0) / 1e9 for m in P]))
    fr, cur, init = [], None, False
    for line in gzip.open(prefix + ".stderr.log.gz", "rt", errors="replace"):
        line = ANSI.sub("", line)
        if "[DIAGT]" in line:
            cur = dict(t=float(line.split("[DIAGT]")[1]) - off, z=False, f=0, init=init, dfail=False)
            fr.append(cur)
        elif "successful initialization" in line:
            init = True
            if cur is not None:
                cur["init"] = True
                cur["t_init"] = True
        elif cur is not None and "[ZUPT]: failed disparity" in line:
            cur["dfail"] = True
        elif cur is not None and "[ZUPT]: accepted" in line:
            cur["z"] = True
        elif cur is not None and ("for MSCKF update (" in line or "for SLAM update (" in line):
            cur["f"] += int(re.search(r"\((\d+) feats\)", line)[1])
    t_init = next((f["t"] for f in fr if f.get("t_init")), None)
    return fr, t_init


def gate_alarms(fr, N):
    """Times where (no ZUPT and 0 update feats) holds for N consecutive frames after init (start of each streak)."""
    out, k = [], 0
    for i, f in enumerate(fr):
        bad = f["init"] and not f["z"] and f["f"] == 0
        k = k + 1 if bad else 0
        if k == N:
            out.append(fr[i - N + 1]["t"])
    return out


def impact_flags(rec, t0, hold=HOLD, thr=15.0, within=3.0):
    from imu_gt import read_imu
    acc, gyr = read_imu(Path("recordings") / rec / "imu.csv", raw=True)
    t = gyr[:, 0]
    a_i = np.stack([np.interp(t, acc[:, 0], acc[:, 1 + j]) for j in range(3)], 1)
    hw, hg, ha = (float(v) for v in hold.split(":"))
    n = max(2, int(round(hw * 1e9 / float(np.median(np.diff(t))))))
    sw = np.lib.stride_tricks.sliding_window_view
    gmax, astd = np.full(len(t), np.inf), np.full(len(t), np.inf)
    gmax[n - 1:] = sw(np.linalg.norm(gyr[:, 1:4], axis=1), n).max(1)
    astd[n - 1:] = sw(np.linalg.norm(a_i, axis=1), n).std(1)
    still = (gmax < hg) & (astd < ha)
    ts = (t - t0) / 1e9
    dev = np.linalg.norm(a_i, axis=1) - 9.80665
    out, last = [], -1e9
    for i in np.nonzero(dev > thr)[0]:
        if ts[i] - last < within:
            continue
        last = ts[i]
        m = (ts > ts[i]) & (ts <= ts[i] + within) & still
        if m.any():
            out.append(dict(t_impact=round(float(ts[i]), 2), t_still=round(float(ts[m][0]), 2),
                            peak=round(float(dev[(ts >= ts[i]) & (ts <= ts[i] + 0.2)].max()), 1)))
    still_segs = []
    d = np.diff(still.astype(int))
    st, en = list(ts[1:][d == 1]), list(ts[1:][d == -1])
    if still[0]:
        st = [ts[0]] + st
    if still[-1]:
        en = en + [ts[-1]]
    still_segs = [[round(float(a), 2), round(float(b), 2)] for a, b in zip(st, en)]
    return out, still_segs


def analyse_one(prefix, es):
    import c5_overnight_analyse as an
    import endstill_vio as ev
    import vio_bench_hs as vb
    d = json.load(open(prefix + ".json"))
    rec = d["rec"]
    t0 = int(es["recs"][rec]["t0_ns"])
    r = ev.analyse_one(prefix, es)
    keep = ("name", "rec", "initialized", "ate_full_cm", "scale_full", "collapsed_full", "tracked_frac",
            "endstill_window_s", "v_mean_cms", "v_max_cms", "drift_cm", "path_cm", "last_pose_s", "note")
    res = {k: r.get(k) for k in keep}
    res.update(cond=Path(prefix).name.split("_")[1], off=d.get("start_off"), bench_ate_cm=d.get("ate_cm"),
               bench_scale=d.get("scale"), frames_held=d.get("frames_held"), cfg_over=d.get("cfg_over"),
               still_hold=d.get("still_hold"), config_zupt={k: v for k, v in (d.get("config_used") or {}).items()
                                                            if k.startswith("zupt") or k == "try_zupt"},
               version=dict(bench=d.get("bench_blob"), runner=d.get("runner_blob"), mitig=(d.get("mitig_blobs") or {}),
                            bin=sorted((d.get("binary_sha256") or {}).values()), dirty=d.get("mitig_dirty")))
    if d.get("hold_segments_imu_ns"):
        res["hold_segments_s"] = [[round((a - t0) / 1e9, 2), None if b is None else round((b - t0) / 1e9, 2)]
                                  for a, b in d["hold_segments_imu_ns"]]
    fr, t_init = frames_from_log(prefix, t0)
    res["t_init_s"] = None if t_init is None else round(t_init, 2)
    for N in (5, 30):
        al = gate_alarms(fr, N)
        res[f"gate{N}"] = dict(first_after_100=next((round(x, 2) for x in al if x > ALIGN_MAX_S), None),
                               n_le_100=sum(x <= ALIGN_MAX_S for x in al), n_total=len(al))
    # ZUPT accepted by the IMU chi2 test although the disparity check failed (Z conditions), t <= 100 s = hand-held motion
    # part (+ the start still): candidates for a wrong 'still' during motion
    m100 = [f for f in fr if f["init"] and f["t"] <= ALIGN_MAX_S]
    res["imu_only_zupt_le_100"] = sum(f["z"] and f["dfail"] for f in m100)
    res["zupt_le_100"] = sum(f["z"] for f in m100)
    res["frames_le_100"] = len(m100)
    zs = [f for f in fr if f["t"] > ALIGN_MAX_S and f["init"]]
    res["zupt_frac_after_100"] = round(float(np.mean([f["z"] for f in zs])), 3) if zs else None
    # trajectory vs GT, aligned on t <= 100 s only
    poses = [m for m in gzip.open(prefix + ".poses.jsonl.gz", "rt")]
    poses = [json.loads(x) for x in poses]
    tr = [m for m in poses if m.get("status") == "tracking" and m.get("T_world_cam0")]
    if len(tr) < 50:
        res["traj_err"] = "too few tracked poses"
        return res
    dt_ns = int(round(d["align_dt_ms"] * 1e6))
    gt_t, gt_p, _ = an.gt(rec, d.get("gt") or "imu_gt_spl")
    vt = np.array([m["t_ns"] for m in tr], np.int64) + dt_ns
    P = np.array([np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3] for m in tr])
    j, ok = an.match(vt, gt_t)
    ts = (vt[ok] - dt_ns - t0) / 1e9                          # recording s (IMU clock)
    Pm, Pg = P[ok], gt_p[j[ok]]
    a = ts <= ALIGN_MAX_S
    if a.sum() < 50:
        res["traj_err"] = "too few matches before 100 s"
        return res
    R, _, t = vb.umeyama(Pm[a], Pg[a])
    Rs, c, tsim = vb.umeyama(Pm[a], Pg[a], scale=True)
    e = np.linalg.norm(Pm @ R.T + t - Pg, axis=1) * 100
    res.update(ate100_cm=float(np.sqrt(np.mean(e[a] ** 2))), scale100=float(c),
               ate100_sim3_cm=float(np.sqrt(np.mean((np.linalg.norm(c * Pm[a] @ Rs.T + tsim - Pg[a], axis=1) * 100) ** 2))))
    tl = ts.max()
    w = ts >= tl - LAST_WIN_S
    res.update(lastwin_s=[round(float(ts[w].min()), 2), round(float(tl), 2)], lastwin_n=int(w.sum()),
               lastwin_med_cm=float(np.median(e[w])), lastwin_max_cm=float(e[w].max()),
               after100_max_cm=float(e[~a].max()) if (~a).any() else None)
    for N in (5, 30):                                          # error when the gate first fired after 100 s
        tf = res[f"gate{N}"]["first_after_100"]
        if tf is not None:
            k = np.argmin(np.abs(ts - tf))
            res[f"gate{N}"]["err_at_cm"] = round(float(e[k]), 2) if abs(ts[k] - tf) < 1.0 else None
    return res


def med(xs):
    xs = [x for x in xs if x is not None]
    return float(np.median(xs)) if xs else None


def summarize():
    es = json.load(open(ENDSTILL))
    runs, failed = [], []
    for j in jobs():
        p = OUT / j["name"]
        if not Path(str(p) + ".json").exists():
            failed.append(j["name"])
            continue
        try:
            runs.append(analyse_one(str(p), es))
        except Exception as e:  # noqa: BLE001
            failed.append(f"{j['name']}: {e!r}")
    cells = {}
    for rec in RECS:
        for cond in CONDS:
            rs = [r for r in runs if r["rec"] == rec and r["cond"] == cond]
            cells[f"{rec[-6:]}|{cond}"] = dict(
                n=len(rs), n_init=sum(bool(r.get("initialized")) for r in rs),
                **{k: med([r.get(k) for r in rs]) for k in ("lastwin_med_cm", "lastwin_max_cm", "drift_cm", "v_mean_cms",
                                                            "v_max_cms", "bench_ate_cm", "ate100_cm", "scale100",
                                                            "after100_max_cm", "zupt_frac_after_100",
                                                            "imu_only_zupt_le_100", "zupt_le_100")},
                ok100=all(r.get("initialized") and r.get("scale100") is not None and 0.5 <= r["scale100"] <= 2
                          and r.get("ate100_cm", 99) <= 10 for r in rs) and len(rs) == 3)
    imp = {}
    for rec in RECS:
        fl, segs = impact_flags(rec, int(es["recs"][rec]["t0_ns"]))
        imp[rec[-6:]] = dict(flags=fl, still_segments=segs)
    verdict = {}
    b4, b8 = cells["224323|base"], cells["223815|base"]
    reproduced = b4["lastwin_med_cm"] is not None and b4["lastwin_med_cm"] >= 10
    for cond in CONDS:
        if cond == "base":
            continue
        c4, c8 = cells[f"224323|{cond}"], cells[f"223815|{cond}"]
        helps_err = (c4["lastwin_med_cm"] is not None and b4["lastwin_med_cm"] is not None
                     and c4["lastwin_med_cm"] <= 0.5 * b4["lastwin_med_cm"])
        helps_drift = (c4["drift_cm"] is not None and b4["drift_cm"] is not None and c4["drift_cm"] <= 0.5 * b4["drift_cm"])
        helps = (helps_err and helps_drift) if reproduced else helps_drift
        noreg = all(cells[f"{r}|{cond}"]["ate100_cm"] is not None and cells[f"{r}|base"]["ate100_cm"] is not None
                    and cells[f"{r}|{cond}"]["ate100_cm"] <= cells[f"{r}|base"]["ate100_cm"] * 1.10 + 0.1
                    and cells[f"{r}|{cond}"]["ok100"] for r in ("224323", "223815"))
        noreg = noreg and c8["lastwin_med_cm"] is not None and c8["lastwin_med_cm"] <= b8["lastwin_med_cm"] * 1.5 + 0.5
        verdict[cond] = dict(helps=helps, helps_err=helps_err, helps_drift=helps_drift, no_regression=noreg)
    cand = [c for c, v in verdict.items() if v["helps"] and v["no_regression"]]
    rec_c = min(cand, key=lambda c: cells[f"224323|{c}"]["lastwin_med_cm"]) if cand else None
    head, b, dirty = blobs()
    vers = {json.dumps(r["version"], sort_keys=True) for r in runs}
    return dict(made=time.strftime("%F %T"), git=head, blobs=b, dirty=dirty, n_runs=len(runs), failed=failed,
                code_versions=len(vers), reproduced=reproduced, cells=cells, verdict=verdict, recommend=rec_c,
                impact=imp, runs=runs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "job", "analyse"])
    ap.add_argument("rest", nargs=argparse.REMAINDER)
    o, _ = ap.parse_known_args(sys.argv[1:2])
    if o.cmd == "job":
        return job(sys.argv[2:])
    if o.cmd == "run":
        return run_all()
    res = summarize()
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(res, open(OUT / "summary.json", "w"), indent=1, ensure_ascii=False, default=float)
    print(json.dumps({k: res[k] for k in ("n_runs", "failed", "code_versions", "reproduced", "verdict", "recommend")},
                     indent=1, ensure_ascii=False))
    for k, v in res["cells"].items():
        print(k, {kk: (round(vv, 2) if isinstance(vv, float) else vv) for kk, vv in v.items()})


if __name__ == "__main__":
    main()
