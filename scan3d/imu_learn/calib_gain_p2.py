#!/usr/bin/env python3
"""Part 2 of the gyro calibration gain test (docs/imu_calib_gain_p2_plan.md): OpenVINS bench with the raw gyro stream
bias-corrected by D (per-still causal re-estimate, same definition as imu_learn/calib_gain.py) vs uncorrected (U).
Same settings as the i3 confirm base runs. OpenVINS unchanged; only imu.csv is preprocessed.
  ../venv_ffs/bin/python imu_learn/calib_gain_p2.py prep        # runs/imu_calib_gain/recs/<rec>/ (symlinks + new imu.csv)
  (setsid nohup ../venv_ffs/bin/python imu_learn/calib_gain_p2.py run > runs/imu_calib_gain/p2/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/calib_gain_p2.py analyse     # -> runs/imu_calib_gain/p2_summary.json
"""
import gzip
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
from calib_gain import still_mean  # noqa: E402
from i3_confirm import still_runs  # noqa: E402
from imu_gt import read_imu  # noqa: E402

PY = "../venv_ffs/bin/python"
BASE = Path("runs/imu_calib_gain")
RECDIR = BASE / "recs"
OUT = BASE / "p2"
LOG = BASE / "p2.log"
RECS = ["hs_20261021_213341", "hs_20261021_214550", "hs_20261021_215348"]
PAR = int(os.environ.get("PAR", 5))


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} p2: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def bias_D(rec):
    """(still runs, still means, function t_ns -> bias rad/s) exactly as calib_gain.run_rec condition D."""
    _, g16 = read_imu(Path("recordings") / rec / "imu.csv", raw=True)
    g = g16[np.argsort(g16[:, 0], kind="stable")]
    g = g[np.r_[True, np.diff(g[:, 0]) > 0]]
    t16, w16 = g[:, 0].astype(np.int64), g[:, 1:4]
    runs = still_runs(t16, w16)
    sb = np.array([still_mean(t16, w16, a, b) for a, b in runs])
    ends = np.array([b for _, b in runs])
    return runs, sb, lambda t: sb[np.clip(np.searchsorted(ends, t, side="right") - 1, 0, None)]


def prep():
    for rec in RECS:
        src, dst = Path("recordings") / rec, RECDIR / rec
        dst.mkdir(parents=True, exist_ok=True)
        for p in src.iterdir():
            if p.name != "imu.csv" and not (dst / p.name).exists():
                (dst / p.name).symlink_to(p.resolve())
        runs, sb, fb = bias_D(rec)
        n = 0
        with open(src / "imu.csv") as fi, open(dst / "imu.csv.tmp", "w") as fo:
            for line in fi:
                if line.startswith("16,"):
                    p = line.rstrip("\n").split(",")
                    try:
                        t = int(p[1])
                        v = [float(x) for x in p[2:5]]
                    except (ValueError, IndexError):
                        fo.write(line)
                        continue
                    b = fb(np.array([t]))[0]
                    p[2:5] = [repr(v[k] - float(b[k])) for k in range(3)]
                    fo.write(",".join(p) + "\n")
                    n += 1
                else:
                    fo.write(line)
        (dst / "imu.csv.tmp").rename(dst / "imu.csv")
        info = dict(made=time.strftime("%F %T"), rec=rec, src=str(src / "imu.csv"), rows16_changed=n,
                    stills_imu_ns=runs, still_bias_mdps=np.round(np.degrees(sb) * 1e3, 2).tolist(),
                    rule="calib_gain D: latest ended still mean (trim 2 s if >= 4 s); first still mean before it ends")
        json.dump(info, open(dst.parent / f"{rec}_bias.json", "w"), indent=1)
        log(f"prep {rec}: {n} type-16 rows corrected, stills {len(runs)}, bias mdps {info['still_bias_mdps']}")


def jobs():
    js = []
    for rec in RECS:
        offs = json.load(open(f"runs/i3_confirm/inputs/{rec}.json"))["offsets_s"]
        for k, off in enumerate(offs):
            for c in ("U", "D"):
                args = zr.common(rec, off, 2.0, 0, True)
                if c == "D":
                    args[0] = str(RECDIR / rec)
                js.append(dict(rec=rec, cond=c, k=k, off=off, name=f"{rec[-6:]}_{c}_o{k}", args=args))
    return js


def cmd(j):
    return [PY, "imu_learn/z2_isolate.py", "job", "--cfg-over", "num_opencv_threads=2", "--still-hold", "", "--",
            *j["args"], "--out", str(OUT / j["name"])]


def run():
    OUT.mkdir(parents=True, exist_ok=True)
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain", "imu_learn/calib_gain_p2.py", "imu_learn/calib_gain.py",
                            "imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/z2_isolate.py",
                            "imu_learn/imu_gt.py"], capture_output=True, text=True).stdout.strip()
    if dirty:
        log(f"REFUSED: code not committed: {dirty}")
        sys.exit(1)
    queue = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists()]
    log(f"start git {head} PAR {PAR} jobs {len(queue)}")
    running = {}
    while queue or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                d["p2_head"] = head
                json.dump(d, open(OUT / f"{n}.json", "w"), indent=1, ensure_ascii=False)
                tail = f"ate {d.get('ate_cm')} scale {d.get('scale')} init {d.get('t_init_clip_s')} err {d.get('error')}"
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


# ---------------------------------------------------------------- analysis
def yaw_drift(prefix, rec):
    """|least-squares slope| (deg/min) of the world-z component of M(t) M(t0)^T, M = R_world_cam · R_bc^T."""
    d = json.load(open(str(prefix) + ".json"))
    dt = int(round(d["align_dt_ms"] * 1e6))
    P = [json.loads(x) for x in gzip.open(str(prefix) + ".poses.jsonl.gz", "rt")]
    tr = [m for m in P if m.get("status") == "tracking" and m.get("T_world_cam0")]
    if len(tr) < 50:
        return None
    g = np.load(Path("recordings") / rec / "imu_gt_spl" / "gt.npz")
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gR = g["t_ns"][ok], g["R_bc"][ok]
    vt = np.array([m["t_ns"] for m in tr], np.int64) + dt
    Rv = np.array([np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, :3] for m in tr])
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    m = np.abs(gt_t[j] - vt) < 2_000_000
    if m.sum() < 50:
        return None
    M = Rv[m] @ np.transpose(gR[j[m]], (0, 2, 1))
    from scipy.spatial.transform import Rotation as Rot
    rel = Rot.from_matrix(M @ M[0].T).as_rotvec()
    yaw = np.degrees(rel[:, 2])
    ts = (vt[m] - vt[m][0]) / 60e9
    slope = np.polyfit(ts, yaw, 1)[0]
    return dict(yaw_dpm=round(float(abs(slope)), 3), yaw_end_deg=round(float(yaw[-1]), 3),
                yaw_maxabs_deg=round(float(np.abs(yaw).max()), 3), dur_min=round(float(ts[-1]), 3))


def metrics(prefix, rec):
    import c5_overnight_analyse as an
    d = json.load(open(str(prefix) + ".json"))
    a = an.analyse_run(str(prefix))
    r = dict(name=Path(prefix).name, ate_cm=d.get("ate_cm"), scale=d.get("scale"), rot_med_deg=d.get("rot_med_deg"),
             t_init_clip_s=d.get("t_init_clip_s"), tracked_frac=d.get("tracked_frac"), error=d.get("error"),
             initialized=a.get("initialized"), win_collapsed=a.get("collapsed"))
    if not a.get("initialized"):
        r["status"] = "noinit"
    elif r["ate_cm"] is None or r["scale"] is None:
        r["status"] = "unscored"
    elif a.get("collapsed") or not (0.5 <= r["scale"] <= 2) or r["ate_cm"] > 10:
        r["status"] = "collapse"
    else:
        r["status"] = "ok"
    r.update(yaw_drift(prefix, rec) or {})
    return r


def analyse():
    rows, pairs = [], []
    for j in jobs():
        if j["cond"] != "U":
            continue
        u, dd = OUT / j["name"], OUT / j["name"].replace("_U_", "_D_")
        ref = Path("runs/i3_confirm") / f"{j['rec'][-6:]}_base_o{j['k']}"
        if not (Path(str(u) + ".json").exists() and Path(str(dd) + ".json").exists()):
            continue
        mu, md = metrics(u, j["rec"]), metrics(dd, j["rec"])
        mr = metrics(ref, j["rec"]) if Path(str(ref) + ".json").exists() else None
        pairs.append(dict(rec=j["rec"], k=j["k"], off=j["off"], U=mu, D=md, i3_base=mr))
    ok = [p for p in pairs if p["U"]["ate_cm"] is not None and p["D"]["ate_cm"] is not None]
    wins = sum(p["D"]["ate_cm"] < p["U"]["ate_cm"] for p in ok)
    losses = sum(p["D"]["ate_cm"] > p["U"]["ate_cm"] for p in ok)
    mu = float(np.median([p["U"]["ate_cm"] for p in ok])) if ok else None
    md = float(np.median([p["D"]["ate_cm"] for p in ok])) if ok else None
    better = len(ok) == 9 and wins >= 6 and md <= 0.8 * mu
    worse = len(ok) == 9 and losses >= 6 and mu <= 0.8 * md
    ym = lambda c: float(np.median([p[c]["yaw_dpm"] for p in ok if "yaw_dpm" in p[c]]))  # noqa: E731
    out = dict(made=time.strftime("%F %T"), plan="docs/imu_calib_gain_p2_plan.md", n_pairs=len(ok), pairs=pairs,
               ate_median_U=mu, ate_median_D=md, D_wins=wins, D_losses=losses,
               verdict="D better" if better else ("D worse" if worse else "no clear difference"),
               yaw_dpm_median_U=ym("U"), yaw_dpm_median_D=ym("D"),
               collapses_U=sum(p["U"]["status"] != "ok" for p in ok), collapses_D=sum(p["D"]["status"] != "ok" for p in ok),
               noise_ref_U_vs_i3base=[dict(rec=p["rec"], k=p["k"], U=p["U"]["ate_cm"],
                                           i3=(p["i3_base"] or {}).get("ate_cm")) for p in ok])
    json.dump(out, open(BASE / "p2_summary.json", "w"), indent=1)
    for p in ok:
        print(p["rec"][-6:], p["k"], "U", p["U"]["ate_cm"], p["U"]["status"], p["U"].get("yaw_dpm"), "| D",
              p["D"]["ate_cm"], p["D"]["status"], p["D"].get("yaw_dpm"), "| i3base",
              (p["i3_base"] or {}).get("ate_cm"), (p["i3_base"] or {}).get("yaw_dpm"))
    print({k: v for k, v in out.items() if k not in ("pairs", "noise_ref_U_vs_i3base")})


if __name__ == "__main__":
    {"prep": prep, "run": run, "analyse": analyse}[sys.argv[1]]()
