#!/usr/bin/env python3
"""Z2 regression check 2026-10-21 (docs/z2_regression_plan.md): set-down mitigation Z2 (zupt_chi2_multipler 1 +
zupt_noise_multiplier 1) vs base on more recordings before it goes into the app config. Same bench path as
imu_learn/setdown_mitig.py (c5_run.py, diag bridge, DEBUG log), static-only init (strict, window 1.0 s, disparity 15 px).

Groups (name prefix):
  F_  new recordings, full length, raw IMU, start offset 0/1/2 s, IMU lead 2 s (bench default)
  T_  C5 short-still trims: clip starts at motion_start_s - S + d (S 1/2 s, d -0.25/0/+0.25 s), raw IMU, IMU lead 0,
      MAX_T s long (plan: 60 s if the timed estimate of the whole matrix is over ~2 h, else full length)
  O_  old monitor recordings (hs_20261019, no raw IMU -> Android types 1/4), full length, offsets 0/1/2, lead 2 s
F_ runs of 224323/223815 are taken from runs/setdown_mitig (same args, same code blobs) if the timing rerun
reproduces that folder's result (plan: reuse rule) -> files are COPIED (never moved) into runs/z2_regress.

  ../venv_ffs/bin/python imu_learn/z2_regress.py time F_224323_base_o0 ...  # up to 3 runs together, foreground
  (setsid nohup ../venv_ffs/bin/python imu_learn/z2_regress.py run > runs/z2_regress/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/z2_regress.py analyse
"""
import argparse
import gzip
import json
import os
import re
import shutil
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
OUT = Path("runs/z2_regress")
LOG = OUT / "sched.log"
NEW = ["hs_20261020_214138", "hs_20261020_221052", "hs_20261020_221925", "hs_20261020_222747", "hs_20261020_223815",
       "hs_20261020_224323"]
STILL_RECS = ["hs_20261020_214138", "hs_20261020_221052", "hs_20261020_222747", "hs_20261020_223815",
              "hs_20261020_224323"]
OLD = ["hs_20261019_131326", "hs_20261019_154320"]
STILL = Path("runs/overnight_vio/inputs")
OFFS = (0, 1, 2)
SS = (1, 2)
DS = {"dm025": -0.25, "d0": 0.0, "dp025": 0.25}
MAX_T = float(os.environ.get("MAX_T", 60))
PAR = int(os.environ.get("PAR", 3))
CONDS = {"base": "", "Z2": "zupt_chi2_multipler=1,zupt_noise_multiplier=1"}
REUSE_DIR = Path("runs/setdown_mitig")
REUSE_RECS = ("224323", "223815")
V_MOVE = 0.02            # m/s: GT speed above this = moving (false-accept test)
V_HALF = 0.1             # s: GT speed = |p(t+h) - p(t-h)| / 2h
GT_TOL = 0.02            # s: GT sample must be this close to each needed time
GYR_MOVE = 0.05          # rad/s: secondary motion flag for accepts without GT (same gyro threshold as the H still-hold)
ANSI = re.compile(r"\x1b\[[0-9;]*m")
FILES = ("imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/imu_gt.py", "imu_learn/z2_regress.py",
         "make_ov_config.py", "config/mono/estimator_config.yaml")


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} z2r: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def blobs():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    b = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in FILES}
    dirty = subprocess.run(["git", "status", "--porcelain", *FILES], capture_output=True, text=True).stdout.strip()
    return head, b, bool(dirty)


def common(rec, off, lead, max_secs, raw):
    a = [f"recordings/{rec}", "--init-cfg", "strict", "--init-window", "1.0", "--init-disp", "15", "--clones", "11",
         "--noise", "cfg", "--start-off", str(off), "--imu-lead", str(lead), "--max-secs", str(max_secs),
         "--decode-threads", "2"]
    return a + (["--imu-raw"] if raw else [])


def jobs():
    out = []
    for cond, over in CONDS.items():
        for rec in NEW:
            for o in OFFS:
                out.append(dict(name=f"F_{rec[-6:]}_{cond}_o{o}", group=f"F_{rec[-6:]}", rec=rec, cond=cond, over=over,
                                args=common(rec, o, 2.0, 0, True), pair=f"F_{rec[-6:]}_o{o}"))
        for S in SS:
            for rec in STILL_RECS:
                ms = json.load(open(STILL / f"{rec}_still.json"))["motion_start_s"]
                for dn, d in DS.items():
                    out.append(dict(name=f"T_{rec[-6:]}_S{S}_{cond}_{dn}", group=f"T_{rec[-6:]}_S{S}", rec=rec, cond=cond,
                                    over=over, args=common(rec, round(ms - S + d, 3), 0.0, MAX_T, True),
                                    pair=f"T_{rec[-6:]}_S{S}_{dn}"))
        for rec in OLD:
            for o in OFFS:
                out.append(dict(name=f"O_{rec[-6:]}_{cond}_o{o}", group=f"O_{rec[-6:]}", rec=rec, cond=cond, over=over,
                                args=common(rec, o, 2.0, 0, False), pair=f"O_{rec[-6:]}_o{o}"))
    # interleave base/Z2 of the same pair so a stopped matrix still has pairs
    out.sort(key=lambda j: (j["pair"][0] != "F", j["pair"], j["cond"]))
    return out


def cmd(j):
    return [PY, "imu_learn/z2_regress.py", "job", "--cfg-over", j["over"], "--", *j["args"], "--out", str(OUT / j["name"])]


def job(argv):
    """In-process c5_run with --cfg-over (same mechanism as setdown_mitig.job)."""
    i = argv.index("--")
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg-over", default="")
    o = ap.parse_args(argv[:i])
    import c5_run
    import vio_bench_hs as vb
    orig = vb.run

    def run(a):
        a.cfg_over = o.cfg_over
        a.still_hold = None
        return orig(a)
    vb.run = run
    sys.argv = ["c5_run.py"] + argv[i + 1:]
    c5_run.main()
    out = sys.argv[sys.argv.index("--out") + 1]
    d = json.load(open(out + ".json"))
    head, b, dirty = blobs()
    d.update(z2_runner="imu_learn/z2_regress.py", z2_blobs=b, z2_dirty=dirty, z2_head=head, z2_cfg_over=o.cfg_over)
    json.dump(d, open(out + ".json", "w"), indent=1, ensure_ascii=False)


def time_runs(names):
    """Run the named jobs concurrently (<= 3, the matrix load) and log each wall time."""
    OUT.mkdir(parents=True, exist_ok=True)
    head, b, dirty = blobs()
    log(f"timing {names} git {head} dirty={dirty} MAX_T {MAX_T}")
    ps = {}
    for name in names[:3]:
        j = next(x for x in jobs() if x["name"] == name)
        ps[name] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL,
                                     stderr=open(OUT / f"{name}.err", "w")), time.time())
    for name, (p, t0) in ps.items():
        p.wait()
        try:
            d = json.load(open(OUT / f"{name}.json"))
            log(f"timing {name} {time.time() - t0:.0f} s frames {d.get('frames_fed')} ate {d.get('ate_cm')} "
                f"scale {d.get('scale')} init {d.get('t_init_clip_s')}")
        except Exception as e:  # noqa: BLE001
            log(f"timing {name} FAILED {e!r}")


def reuse(js):
    """Copy runs/setdown_mitig/<rec6>_<cond>_o<o>.* for F_ jobs of REUSE_RECS (only called when the plan's reuse rule held)."""
    n = 0
    for j in js:
        rec6 = j["rec"][-6:]
        if not (j["name"].startswith("F_") and rec6 in REUSE_RECS):
            continue
        src = REUSE_DIR / f"{rec6}_{j['cond']}_o{j['name'][-1]}"
        dst = OUT / j["name"]
        if Path(str(dst) + ".json").exists():
            continue
        for ext in (".json", ".poses.jsonl.gz", ".stderr.log.gz"):
            shutil.copy2(str(src) + ext, str(dst) + ext)
        n += 1
        log(f"reused {src} -> {dst}")
    return n


def run_all(do_reuse):
    OUT.mkdir(parents=True, exist_ok=True)
    head, b, dirty = blobs()
    js = jobs()
    if do_reuse:
        reuse(js)
    js = [j for j in js if not (OUT / f"{j['name']}.json").exists()]
    log(f"start git {head} dirty={dirty} PAR {PAR} MAX_T {MAX_T} jobs {len(js)} blobs {b}")
    running = {}
    while js or running:
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
        while js and len(running) < PAR:
            j = js.pop(0)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
            log(f"start {j['name']} ({len(js)} left)")
        time.sleep(10)
    log("end (all done)")


# ---------------------------------------------------------------- analysis
def frames_from_log(prefix):
    """Per processed frame: bridge time -> fed time (IMU clock ns) via the pose lines; ZUPT accepted?, disparity failed?,
    initialized yet? (same parsing as setdown_mitig.frames_from_log)."""
    P = [json.loads(x) for x in gzip.open(prefix + ".poses.jsonl.gz", "rt")]
    if not P:
        return []
    off = float(np.median([m["t"] - m["t_ns"] / 1e9 for m in P if "t" in m and "t_ns" in m]))
    fr, cur, init = [], None, False
    for line in gzip.open(prefix + ".stderr.log.gz", "rt", errors="replace"):
        line = ANSI.sub("", line)
        if "[DIAGT]" in line:
            cur = dict(t_ns=(float(line.split("[DIAGT]")[1]) - off) * 1e9, z=False, init=init, dfail=False)
            fr.append(cur)
        elif "successful initialization" in line:
            init = True
            if cur is not None:
                cur["init"] = True
        elif cur is not None and "[ZUPT]: failed disparity" in line:
            cur["dfail"] = True
        elif cur is not None and "[ZUPT]: accepted" in line:
            cur["z"] = True
    return fr


_GTV = {}


def gt_speed(rec):
    """GT camera speed (m/s) at each GT sample by a +-V_HALF central difference; NaN where GT has no sample within GT_TOL."""
    if rec not in _GTV:
        import c5_overnight_analyse as an
        t, p, _ = an.gt(rec, "imu_gt_spl")
        ts = t / 1e9
        v = np.full(len(t), np.nan)
        for sgn in (1,):
            ja = np.clip(np.searchsorted(ts, ts + V_HALF), 1, len(ts) - 1)
            jb = np.clip(np.searchsorted(ts, ts - V_HALF), 1, len(ts) - 1)
            ja = np.where(np.abs(ts[ja - 1] - (ts + V_HALF)) < np.abs(ts[ja] - (ts + V_HALF)), ja - 1, ja)
            jb = np.where(np.abs(ts[jb - 1] - (ts - V_HALF)) < np.abs(ts[jb] - (ts - V_HALF)), jb - 1, jb)
            ok = (np.abs(ts[ja] - ts - V_HALF) < GT_TOL) & (np.abs(ts[jb] - ts + V_HALF) < GT_TOL)
            v[ok] = np.linalg.norm(p[ja[ok]] - p[jb[ok]], axis=1) / (ts[ja[ok]] - ts[jb[ok]])
        _GTV[rec] = (t, v)
    return _GTV[rec]


_GYR = {}


def gyro_norm(rec, raw):
    if (rec, raw) not in _GYR:
        from imu_gt import read_imu
        _, gyr = read_imu(Path("recordings") / rec / "imu.csv", raw=raw)
        _GYR[(rec, raw)] = (gyr[:, 0], np.linalg.norm(gyr[:, 1:4], axis=1))
    return _GYR[(rec, raw)]


def analyse_one(prefix):
    d = json.load(open(prefix + ".json"))
    rec = d["rec"]
    P = [json.loads(x) for x in gzip.open(prefix + ".poses.jsonl.gz", "rt")]
    tr = [m for m in P if m.get("status") == "tracking"]
    res = dict(name=Path(prefix).name, rec=rec, start_off=d.get("start_off"), imu_lead=d.get("imu_lead_s"),
               imu=d.get("imu_source"), initialized=bool(tr), t_init_clip_s=d.get("t_init_clip_s"),
               ate_cm=d.get("ate_cm"), scale=d.get("scale"), tracked_frac=d.get("tracked_frac"), error=d.get("error"),
               zupt_cfg={k: v for k, v in (d.get("config_used") or {}).items() if k.startswith("zupt")},
               version=dict(bench=d.get("bench_blob"), runner=d.get("runner_blob"),
                            bin=sorted((d.get("binary_sha256") or {}).values()),
                            imu_gt=(d.get("z2_blobs") or d.get("mitig_blobs") or {}).get("imu_learn/imu_gt.py"),
                            mkcfg=(d.get("z2_blobs") or d.get("mitig_blobs") or {}).get("make_ov_config.py"),
                            tmpl=(d.get("z2_blobs") or d.get("mitig_blobs") or {}).get("config/mono/estimator_config.yaml"),
                            dirty=d.get("z2_dirty", d.get("mitig_dirty"))),
               reused=d.get("z2_runner") is None)
    if not tr:
        res["status"] = "noinit"
    elif d.get("ate_cm") is None or d.get("scale") is None:
        res["status"] = "unscored"
    elif 0.5 <= d["scale"] <= 2 and d["ate_cm"] <= 10:
        res["status"] = "ok"
    else:
        res["status"] = "collapse"
    fr = frames_from_log(prefix)
    gt_t, gv = gt_speed(rec)
    dt_ns = d["align_dt_ms"] * 1e6
    imu_only = [f for f in fr if f["init"] and f["z"] and f["dfail"]]
    all_z = [f for f in fr if f["init"] and f["z"]]

    def speed(f):
        tc = f["t_ns"] + dt_ns                                  # camera clock (GT)
        j = int(np.clip(np.searchsorted(gt_t, tc), 1, len(gt_t) - 1))
        j = j - 1 if abs(gt_t[j - 1] - tc) < abs(gt_t[j] - tc) else j
        return float(gv[j]) if abs(gt_t[j] - tc) < GT_TOL * 1e9 else float("nan")
    sp = np.array([speed(f) for f in imu_only]) if imu_only else np.zeros(0)
    spz = np.array([speed(f) for f in all_z]) if all_z else np.zeros(0)
    # secondary (not in the verdict): IMU-only accepts without GT speed -> max |gyro| over +-V_HALF > GYR_MOVE = 'moving (IMU)'
    unk = [f for f, v in zip(imu_only, sp) if np.isnan(v)]
    if unk:
        ti, gn = gyro_norm(rec, d.get("imu_source") == "raw_16_35")
        gm = [float(gn[np.searchsorted(ti, f["t_ns"] - V_HALF * 1e9):np.searchsorted(ti, f["t_ns"] + V_HALF * 1e9)].max(initial=0))
              for f in unk]
        res["unknown_gyro_moving"] = int(np.sum(np.array(gm) > GYR_MOVE))
    else:
        res["unknown_gyro_moving"] = 0
    res.update(frames=len(fr), frames_init=sum(f["init"] for f in fr), zupt_acc=len(all_z), zupt_imu_only=len(imu_only),
               false_acc=int(np.sum(sp > V_MOVE)), imu_only_gt_unknown=int(np.sum(np.isnan(sp))),
               false_acc_v_cms=[round(float(v) * 100, 2) for v in sp if v > V_MOVE][:20],
               false_acc_max_v_cms=round(float(np.nanmax(sp)) * 100, 2) if np.any(sp > V_MOVE) else None,
               false_acc_t_s=[round((f["t_ns"] - fr[0]["t_ns"]) / 1e9, 2) for f, v in zip(imu_only, sp) if v > V_MOVE][:20],
               zupt_moving_any=int(np.sum(spz > V_MOVE)))
    return res


def med(xs):
    xs = [x for x in xs if x is not None]
    return float(np.median(xs)) if xs else None


def summarize():
    runs, missing = [], []
    for j in jobs():
        p = OUT / j["name"]
        if not Path(str(p) + ".json").exists():
            missing.append(j["name"])
            continue
        try:
            r = analyse_one(str(p))
            r.update(group=j["group"], cond=j["cond"], pair=j["pair"])
            runs.append(r)
        except Exception as e:  # noqa: BLE001
            missing.append(f"{j['name']}: {e!r}")
    groups = {}
    for g in dict.fromkeys(j["group"] for j in jobs()):
        rs = {c: [r for r in runs if r["group"] == g and r["cond"] == c] for c in CONDS}
        cell = {}
        for c, xs in rs.items():
            sc = [r["ate_cm"] for r in xs if r["status"] in ("ok", "collapse")]
            cell[c] = dict(n=len(xs), n_init=sum(r["initialized"] for r in xs), n_ok=sum(r["status"] == "ok" for r in xs),
                           status=[r["status"] for r in sorted(xs, key=lambda r: r["pair"])],
                           ate_med=med(sc), ates=[r["ate_cm"] for r in sorted(xs, key=lambda r: r["pair"])],
                           scale_med=med([r["scale"] for r in xs if r["status"] in ("ok", "collapse")]),
                           false_acc=sum(r["false_acc"] for r in xs), imu_only=sum(r["zupt_imu_only"] for r in xs),
                           imu_only_unknown=sum(r["imu_only_gt_unknown"] for r in xs),
                           unknown_gyro_moving=sum(r["unknown_gyro_moving"] for r in xs),
                           zupt_moving_any=sum(r["zupt_moving_any"] for r in xs),
                           false_acc_runs=[r["false_acc"] for r in sorted(xs, key=lambda r: r["pair"])])
        b, z = cell["base"], cell["Z2"]
        new_bad = []
        for pr in dict.fromkeys(r["pair"] for r in rs["base"] + rs["Z2"]):
            sb = next((r["status"] for r in rs["base"] if r["pair"] == pr), None)
            sz = next((r["status"] for r in rs["Z2"] if r["pair"] == pr), None)
            if sb == "ok" and sz != "ok":
                new_bad.append(f"{pr}: base ok -> Z2 {sz}")
            elif sb == "noinit" and sz in ("collapse", "unscored"):
                new_bad.append(f"{pr}: base noinit -> Z2 {sz}")
        if b["ate_med"] is None or z["ate_med"] is None:
            ate_ok, lim = None, None
        else:
            lim = max(b["ate_med"] * 1.10, b["ate_med"] + 0.2)
            ate_ok = z["ate_med"] <= lim + 1e-9
        groups[g] = dict(cells=cell, ate_limit=lim, ate_ok=ate_ok, new_collapses=new_bad, false_acc_ok=z["false_acc"] == 0,
                         pass_=(ate_ok is not False) and not new_bad and z["false_acc"] == 0)
    head, b, dirty = blobs()
    vers = {}
    for r in runs:
        vers.setdefault(json.dumps(r["version"], sort_keys=True), []).append(r["name"])
    zcfg = {json.dumps(r["zupt_cfg"], sort_keys=True) + "|" + r["cond"] for r in runs}
    return dict(made=time.strftime("%F %T"), git=head, blobs=b, dirty=dirty, n_runs=len(runs), missing=missing,
                code_versions=len(vers), version_names={k: len(v) for k, v in vers.items()}, zupt_cfgs=sorted(zcfg),
                all_pass=all(g["pass_"] for g in groups.values()), groups=groups, runs=runs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "job", "analyse", "time", "list"])
    o, rest = ap.parse_known_args(sys.argv[1:2])
    if o.cmd == "job":
        return job(sys.argv[2:])
    if o.cmd == "time":
        return time_runs(sys.argv[2:])
    if o.cmd == "list":
        for j in jobs():
            print(j["name"], " ".join(cmd(j)))
        return
    if o.cmd == "run":
        return run_all("--reuse" in sys.argv[2:])
    res = summarize()
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(res, open(OUT / "summary.json", "w"), indent=1, ensure_ascii=False, default=float)
    print(json.dumps({k: res[k] for k in ("n_runs", "missing", "code_versions", "zupt_cfgs", "all_pass")}, indent=1))
    for g, v in res["groups"].items():
        c = v["cells"]
        print(f"{g:16s} base {c['base']['status']} ate_med {c['base']['ate_med']} | Z2 {c['Z2']['status']} "
              f"ate_med {c['Z2']['ate_med']} lim {v['ate_limit']} ok {v['ate_ok']} new {v['new_collapses']} "
              f"FA {c['Z2']['false_acc']} (imu-only {c['Z2']['imu_only']}, unk {c['Z2']['imu_only_unknown']}) "
              f"PASS {v['pass_']}")


if __name__ == "__main__":
    main()
