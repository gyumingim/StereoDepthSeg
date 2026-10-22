#!/usr/bin/env python3
"""Independent check of imu_learn/board_rotation.py (does NOT import it nor ate_floor.py).
Recomputes from gt.npz, run JSON + poses, imu.csv: gravity direction in the board frame, Sim3 error, 30 s / 5 s
low-pass (scipy uniform_filter1d on a 240 Hz grid, mode nearest), fractions f_v, f_y and r = f_y / f_v per run,
median per recording, decision with margin 1.5 -> runs/board_rotation/check.json
  ../venv_ffs/bin/python imu_learn/board_rotation_check.py
"""
import csv
import gzip
import json
import time
from pathlib import Path

import numpy as np
from scipy.ndimage import uniform_filter1d

RUNS = Path("runs/board_rotation")
RECS = {"hs_20261022_133820": "rot90", "hs_20261022_134204": "rot90", "hs_20261022_134550": "control"}


def raw_acc(path):
    t, a = [], []
    with open(path) as f:
        for row in csv.reader(f):
            if row and row[0] == "35" and len(row) >= 5:
                try:
                    v = [int(row[1]), float(row[2]), float(row[3]), float(row[4])]
                except ValueError:
                    continue
                t.append(v[0])
                a.append(v[1:])
    return np.array(t, np.int64), np.array(a)


def sim3(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    A, B = src - ms, dst - md
    H = A.T @ B / len(src)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    s = np.trace(np.diag(S) @ D) / (A ** 2).sum(1).mean()
    return s, R, md - s * R @ ms


def smooth(t, x, win_s):
    tg = np.arange(t[0], t[-1], 1 / 240)
    xg = np.column_stack([np.interp(tg, t, x[:, k]) for k in range(3)])
    yg = uniform_filter1d(xg, size=int(round(240 * win_s)), axis=0, mode="nearest")
    return np.column_stack([np.interp(t, tg, yg[:, k]) for k in range(3)])


def main():
    out = dict(made=time.strftime("%F %T"), recs={})
    for rec, cond in RECS.items():
        R = Path("recordings") / rec
        al = json.load(open(R / "imu_gt" / "align.json"))
        g = np.load(R / "imu_gt_spl" / "gt.npz", allow_pickle=True)
        ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
        gt_t, gt_p, gt_R = g["t_ns"][ok], g["p_bc"][ok], g["R_bc"][ok]
        # IMU accel header line format check: first field is type
        ta, a = raw_acc(R / "imu.csv")
        ta = ta + int(al["dt_ns"])
        ai = np.column_stack([np.interp(gt_t, ta, a[:, k]) for k in range(3)])
        f = np.array([Rb @ np.asarray(al["R_ci"]) @ v for Rb, v in zip(gt_R, ai)])
        ug = f.mean(0)
        ug /= np.linalg.norm(ug)
        rr = dict(cond=cond, u_g=ug.round(4).tolist(), angle_y_deg=round(float(np.degrees(np.arccos(abs(ug[1])))), 2),
                  runs={})
        for k in range(3):
            jf = RUNS / f"{rec[-6:]}_base_o{k}.json"
            if not jf.exists():
                continue
            j = json.load(open(jf))
            if j.get("error") or not j.get("ate_cm"):
                rr["runs"][k] = dict(status="no result")
                continue
            if j["ate_cm"] > 10 or not 0.5 <= j["scale"] <= 2:
                rr["runs"][k] = dict(status="collapsed")
                continue
            P, T = [], []
            for line in gzip.open(RUNS / f"{rec[-6:]}_base_o{k}.poses.jsonl.gz", "rt"):
                if not line.strip():
                    continue
                m = json.loads(line)
                if m.get("status") == "tracking" and m.get("T_world_cam0"):
                    T.append(m["t_ns"] + int(round(j["align_dt_ms"] * 1e6)))
                    P.append(np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3])
            T, P = np.array(T), np.array(P)
            idx = np.searchsorted(gt_t, T)
            idx = np.clip(idx, 1, len(gt_t) - 1)
            left = np.abs(gt_t[idx - 1] - T) <= np.abs(gt_t[idx] - T)
            idx[left] -= 1
            sel = np.abs(gt_t[idx] - T) < 2e6
            P, Pg, tt = P[sel], gt_p[idx[sel]], (T[sel] - T[sel][0]) / 1e9
            s, Rr, tr = sim3(P, Pg)
            e = s * P @ Rr.T + tr - Pg
            d = dict(status="ok", n=int(sel.sum()), ate_sim3_cm=round(float(np.sqrt((e ** 2).sum(1).mean()) * 100), 3),
                     bench_sim3=j["ate_sim3_cm"], gyro_rebias=j.get("gyro_rebias"))
            for lab, w in (("L30", 30.0), ("L5", 5.0)):
                L = smooth(tt, e, w)
                tot = (L ** 2).sum()
                fv = float(((L @ ug) ** 2).sum() / tot)
                fy = float((L[:, 1] ** 2).sum() / tot)
                d[lab] = dict(rms_cm=round(float(np.sqrt((L ** 2).sum(1).mean()) * 100), 3), f_v=round(fv, 4),
                              f_y=round(fy, 4), f_x=round(float((L[:, 0] ** 2).sum() / tot), 4), r=round(fy / fv, 3))
            rr["runs"][k] = d
        good = [d for d in rr["runs"].values() if d["status"] == "ok"]
        for lab in ("L30", "L5"):
            if good:
                med = {q: round(float(np.median([d[lab][q] for d in good])), 4) for q in ("rms_cm", "f_v", "f_y", "f_x")}
                med["r"] = round(med["f_y"] / med["f_v"], 3)
                rr[lab] = med
        out["recs"][rec] = rr
        print(rec, cond, "angle", rr["angle_y_deg"], "L30", rr.get("L30"), "L5", rr.get("L5"), flush=True)
    rot = [v["L30"]["r"] for v in out["recs"].values() if v["cond"] == "rot90" and "L30" in v]
    out["rot_r"] = rot
    out["verdict_ignoring_validity"] = ("GT 쪽" if len(rot) == 2 and min(rot) > 1.5 else
                                        "VIO 쪽" if len(rot) == 2 and max(rot) < 1 / 1.5 else "미판정")
    json.dump(out, open(RUNS / "check.json", "w"), indent=1, ensure_ascii=False)
    print(out["rot_r"], out["verdict_ignoring_validity"])


if __name__ == "__main__":
    main()
