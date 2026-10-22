#!/usr/bin/env python3
"""Part 1 of docs/gyro_rebias_apply_plan.md: gyro-only 10 s rotation error of the streaming module (G,
imu_learn/gyro_rebias.py) vs B (type 4) / C (start still) / D / D2, same windows and scoring as imu_learn/calib_gain.py.
  ../venv_ffs/bin/python imu_learn/gyro_rebias_eval.py      # -> runs/gyro_rebias/part1.json
"""
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(HERE))
from calib_gain import (END_TOL_S, GATE, MIN_WIN, RECS, TRIM_MIN_LEN, TRIM_S, WIN_S, integrate,  # noqa: E402
                        orient_at, read_type4, still_mean)
from gyro_rebias import correct_array  # noqa: E402
from i3_confirm import still_runs  # noqa: E402
from imu_gt import read_imu  # noqa: E402

OUT = SCAN3D / "runs" / "gyro_rebias"


def clean(g):
    g = g[np.argsort(g[:, 0], kind="stable")]
    g = g[np.r_[True, np.diff(g[:, 0]) > 0]]
    return g[:, 0].astype(np.int64), g[:, 1:4]


def run_rec(rec):
    R = SCAN3D / "recordings" / rec
    acc, g16 = read_imu(R / "imu.csv", raw=True)
    acc1, _ = read_imu(R / "imu.csv", raw=False)
    al = json.load(open(R / "imu_gt" / "align.json"))
    R_ci, dt_ns = np.asarray(al["R_ci"]), int(al["dt_ns"])
    gt = np.load(R / "imu_gt_spl" / "gt.npz")
    ok = gt["ok"]
    tc, Rbc = gt["t_ns"][ok].astype(np.int64), gt["R_bc"][ok]
    ti = tc - dt_ns
    t16, w16 = clean(g16)
    t4, w4 = clean(read_type4(R / "imu.csv"))
    runs = still_runs(t16, w16)
    info = dict(rec=rec)
    if not runs:
        info["excluded"] = "no IMU still"
        return info
    sb = [still_mean(t16, w16, a, b) for a, b in runs]
    ends = np.array([b for _, b in runs])
    idx = np.clip(np.searchsorted(ends, t16, side="right") - 1, 0, None)
    bC = np.tile(sb[0], (len(t16), 1))
    bD = np.array(sb)[idx]
    acc_b = [sb[0]]
    for v in sb[1:]:
        acc_b.append(acc_b[-1] if np.any(np.abs(v - acc_b[-1]) > GATE) else v)
    bD2 = np.array(acc_b)[idx]
    _, bG, glog = correct_array(t16, w16)
    info["G_events"] = [[round((t - t16[0]) / 1e9, 2), ev, round(L, 1), np.round(np.degrees(v) * 1e3, 1).tolist()]
                        for t, ev, L, v in glog]
    conds = {"B": (t4, w4, np.zeros_like(w4)), "C": (t16, w16, bC), "D": (t16, w16, bD), "D2": (t16, w16, bD2),
             "G": (t16, w16, bG)}
    cal_end = runs[0][1]
    i0 = np.nonzero(ti > cal_end)[0]
    tgt = ti[i0] + int(WIN_S * 1e9)
    j = np.clip(np.searchsorted(ti, tgt), 1, len(ti) - 1)
    j = np.where(np.abs(ti[j - 1] - tgt) < np.abs(ti[j] - tgt), j - 1, j)
    good = (np.abs(ti[j] - tgt) <= END_TOL_S * 1e9) & (tgt <= min(t16[-1], t4[-1]))
    i0, j = i0[good], j[good]
    info["n_windows"] = int(len(i0))
    if len(i0) < MIN_WIN:
        info["excluded"] = f"windows {len(i0)} < {MIN_WIN}"
        return info
    Rr = Rot.from_matrix(Rbc)
    Rgt = Rr[i0].inv() * Rr[j]
    span = (ti[j] - ti[i0]) / 1e9
    a0, a1 = runs[0]
    if (a1 - a0) / 1e9 >= TRIM_MIN_LEN:
        a0 += int(TRIM_S * 1e9)
    am = acc1[(acc1[:, 0] >= a0) & (acc1[:, 0] <= a1), 1:4].mean(0)
    fs = (ti >= a0) & (ti <= a1)
    gb = (Rbc[fs] @ (R_ci @ am)).mean(0) if fs.any() else Rbc[0] @ (R_ci @ am)
    gb /= np.linalg.norm(gb)
    Rci = Rot.from_matrix(R_ci)
    res = {}
    for c, (t, w, b) in conds.items():
        q = integrate(t, w, b)
        Rest = Rci * (orient_at(t, q, w, b, ti[i0]).inv() * orient_at(t, q, w, b, ti[j])) * Rci.inv()
        err = np.degrees((Rgt.inv() * Rest).magnitude())
        yaw = np.degrees(np.abs((Rr[i0] * Rest * Rr[j].inv()).as_rotvec() @ gb)) / span * 60
        res[c] = dict(median=float(np.median(err)), p90=float(np.percentile(err, 90)),
                      yaw_dpm_median=float(np.median(yaw)))
    info["cond"] = res
    return info


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    head = subprocess.run(["git", "-C", str(SCAN3D), "rev-parse", "--short", "HEAD"], capture_output=True,
                          text=True).stdout.strip()
    out = dict(made=time.strftime("%F %T"), git_head=head, plan="docs/gyro_rebias_apply_plan.md (bd63072)",
               inputs="recordings/<rec>/{imu.csv, imu_gt/align.json, imu_gt_spl/gt.npz}", recs=[])
    for rec in RECS:
        r = run_rec(rec)
        out["recs"].append(r)
        print(rec, r.get("excluded", ""), {c: round(v["median"], 3) for c, v in r.get("cond", {}).items()}, flush=True)
    used = [r for r in out["recs"] if "cond" in r]
    mm = {c: float(np.median([r["cond"][c]["median"] for r in used])) for c in ["B", "C", "D", "D2", "G"]}
    ym = {c: float(np.median([r["cond"][c]["yaw_dpm_median"] for r in used])) for c in ["B", "C", "D", "D2", "G"]}
    worse = [r["rec"] for r in used if r["cond"]["G"]["median"] > 1.10 * r["cond"]["C"]["median"]]
    out["summary"] = dict(median_of_medians=mm, yaw_dpm_median_of_medians=ym, n_recs=len(used))
    out["criteria"] = dict(c1_rule="G median <= 1.10 x C median on every recording", c1_recs_failing=worse,
                           c1_pass=not worse, c2_rule="median-of-medians G < B", c2_pass=mm["G"] < mm["B"],
                           passed=(not worse) and mm["G"] < mm["B"],
                           reduction_B_to_G_pct=100 * (1 - mm["G"] / mm["B"]))
    json.dump(out, open(OUT / "part1.json", "w"), indent=1)
    print(json.dumps(out["summary"], indent=1), json.dumps(out["criteria"], indent=1))


if __name__ == "__main__":
    main()
