#!/usr/bin/env python3
"""Rolling-shutter shift size per recording (docs/rs_correction_plan.md, 2026-10-23): rotational vs translational.

Per GT sample (imu_gt_spl, ok, |p_bc| < 1.2 m), top-to-bottom row shift over one readout t_r, fed image f (640x360):
  rotational    f * |w_xy| * t_r            w = raw gyro (IMU clock = t_cam - dt) rotated to the camera (align.json R_ci)
  translational f * |v_xy| * t_r / Z        v = d p_bc / dt (GT spline, central difference +-25 ms) in the camera frame,
                                            Z = depth of the board origin in the camera (rough scene depth; ASSUMPTION:
                                            tracked features lie at about board distance - room features farther away
                                            give smaller shifts)
for t_r = 6.54 ms (regular 60 fps session) and 2.94 ms (HS 240). Reports p50/p95 px.
  ../venv_ffs/bin/python imu_learn/rs_trans_est.py  -> runs/rs_corr/trans_est.json
"""
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
import rs_run  # noqa: E402
import vio_bench_hs as vb  # noqa: E402
from imu_gt import read_imu  # noqa: E402

TRS = {"t6.54": 6.54e-3, "t2.94": 2.94e-3}


def one(name):
    rec = Path("recordings") / name
    meta = json.load(open(rec / "meta.json"))
    _, _, Knew, _ = vb.undistort_maps(rec, meta, "640x360")
    f = float(Knew[1, 1])
    g = np.load(rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
    al = json.load(open(rec / "imu_gt" / "align.json"))
    t = g["t_ns"].astype(np.int64)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    R_bc, p_bc = g["R_bc"], g["p_bc"]
    # depth of the board origin in the camera; the convention of R_bc is picked by the sign (Z must be > 0)
    zA = np.einsum("nji,nj->ni", R_bc, -p_bc)[:, 2]          # R_bc = board<-cam  -> R_cb = R_bc^T
    zB = np.einsum("nij,nj->ni", R_bc, -p_bc)[:, 2]          # R_bc = cam<-board
    useA = np.median(zA[ok]) > 0
    Z = zA if useA else zB
    # GT velocity (board frame) by central difference over +-25 ms among ok samples
    idx = np.where(ok)[0]
    ts = t[idx] / 1e9
    P = p_bc[idx]
    h = 0.025
    jp = np.clip(np.searchsorted(ts, ts + h), 0, len(ts) - 1)
    jm = np.clip(np.searchsorted(ts, ts - h), 0, len(ts) - 1)
    dt = ts[jp] - ts[jm]
    good = (dt > 0.03) & (dt < 0.07)
    vb_ = (P[jp] - P[jm]) / np.maximum(dt, 1e-9)[:, None]
    Rk = R_bc[idx]
    v_c = np.einsum("nji,nj->ni", Rk, vb_) if useA else np.einsum("nij,nj->ni", Rk, vb_)
    vxy = np.linalg.norm(v_c[:, :2], axis=1)
    Zk = Z[idx]
    good &= Zk > 0.05
    # gyro in the camera frame at the GT times
    try:
        _, gyr = read_imu(rec / "imu.csv", raw=True)
    except ValueError:
        _, gyr = read_imu(rec / "imu.csv", raw=False)
    R_ci = np.asarray(al["R_ci"])
    ti = (t[idx] - int(al["dt_ns"])).astype(np.float64)
    w_i = np.stack([np.interp(ti, gyr[:, 0].astype(np.float64), gyr[:, 1 + j]) for j in range(3)], 1)
    w_c = w_i @ R_ci.T
    wxy = np.linalg.norm(w_c[:, :2], axis=1)
    out = dict(f_px=round(f, 1), n=int(good.sum()), R_bc_board_from_cam=bool(useA),
               depth_m_p50=round(float(np.median(Zk[good])), 3), v_xy_p95_m_s=round(float(np.percentile(vxy[good], 95)), 3),
               w_xy_p95_rad_s=round(float(np.percentile(wxy[good], 95)), 3))
    for k, tr in TRS.items():
        rot = f * wxy[good] * tr
        tra = f * vxy[good] * tr / Zk[good]
        out[k] = dict(rot_p50=round(float(np.median(rot)), 3), rot_p95=round(float(np.percentile(rot, 95)), 3),
                      trans_p50=round(float(np.median(tra)), 3), trans_p95=round(float(np.percentile(tra, 95)), 3))
    return out


def main():
    res = dict(made=time.strftime("%F %T"), inputs="recordings/<rec>/imu_gt_spl/gt.npz, imu.csv, imu_gt/align.json",
               recs={})
    for r in rs_run.RAW + rs_run.OLD:
        try:
            res["recs"][r] = one(r)
        except Exception as e:  # noqa: BLE001
            res["recs"][r] = dict(error=str(e))
        print(r, json.dumps(res["recs"][r]), flush=True)
    Path("runs/rs_corr").mkdir(parents=True, exist_ok=True)
    json.dump(res, open("runs/rs_corr/trans_est.json", "w"), indent=1)


if __name__ == "__main__":
    main()
