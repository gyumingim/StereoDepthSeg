#!/usr/bin/env python3
"""Synthetic check of spline_gt --imu: a real recording's camera-only spline is taken as the TRUE trajectory, corners
(+ noise) and IMU samples (+ noise, known bias / gravity / lever arm) are simulated from it, then the IMU fit must
recover the lever arm and biases. Also runs the real-data IMU fit on the same slice for comparison.
  ../venv_ffs/bin/python imu_learn/spline_synth.py recordings/hs_20261019_131326 --start 10 --secs 30
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import spline_gt as S  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--start", type=float, default=10)
    ap.add_argument("--secs", type=float, default=30)
    ap.add_argument("--knot", type=float, default=0.025)
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--real", action="store_true", help="also fit the real IMU on this slice")
    a = ap.parse_args()
    dev = "cuda"
    rng = np.random.default_rng(0)
    log = lambda s: print(s, flush=True)
    D = S.load(a.rec, "imu_gt_rs", a.start, a.secs)
    T = S.fit(D, a.knot, False, a.iters, dev, log=lambda s: None)          # "true" trajectory
    I = S.imu_data(D, dev)
    r_true = np.array([0.043, 0.004, -0.012])
    bg_true, ba_true = np.array([0.002, -0.001, 0.0015]), np.array([0.05, -0.03, 0.02])
    f32 = lambda x: torch.tensor(np.asarray(x), dtype=torch.float32, device=dev)
    ex = dict(bg=f32(np.tile(bg_true, (I["nb"], 1))), ba=f32(np.tile(ba_true, (I["nb"], 1))), gdir=f32(I["g0"] / S.G0),
              r=f32(r_true), dci=torch.zeros(3, device=dev))
    with torch.no_grad():
        wp, fp = S.imu_pred(T["sp"], T["cp"], T["C"], ex, I)
        Rt, pt = S.eval_pose(T, D["t"], dev)
    I["w"] = wp + 0.01 * torch.randn_like(wp)
    I["a"] = fp + 0.05 * torch.randn_like(fp)
    xc = np.einsum("nji,nj->ni", Rt, D["X"] - pt)
    Ds = dict(D, obs=xc[:, :2] / xc[:, 2:3] + rng.normal(0, 0.3 / D["f"], (len(xc), 2)))
    print(f"synthetic: {len(Ds['t'])} corners, {len(I['t'])} IMU samples; truth r {r_true * 100} cm, bg {bg_true}, ba {ba_true}")
    F0 = S.fit(Ds, a.knot, False, a.iters, dev, log=lambda s: None)
    F = S.fit(Ds, a.knot, True, a.iters, dev, log=log, I=I, init=F0)
    e = F["extra"]
    print(f"recovered r {np.round(np.array(e['r']) * 100, 2)} cm, bg {np.round(np.mean(e['bg'], 0), 4)}, "
          f"ba {np.round(np.mean(e['ba'], 0), 3)}, dci {np.round(e['dci'], 4)}, imu res {F['imu_res']}", flush=True)
    if a.real:
        print("real IMU, same slice:")
        F = S.fit(D, a.knot, True, a.iters, dev, log=log, init=T)
        e = F["extra"]
        print(f"real r {np.round(np.array(e['r']) * 100, 2)} cm, bg {np.round(np.mean(e['bg'], 0), 4)}, "
              f"ba {np.round(np.mean(e['ba'], 0), 3)}, dci {np.round(e['dci'], 4)}, imu res {F['imu_res']}", flush=True)


if __name__ == "__main__":
    main()
