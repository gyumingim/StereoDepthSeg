"""Is the board ground truth metric? Find the GT scale s that the raw IMU likes best.

Why: a flat board lets PnP trade focal length against distance, so reprojection cannot pick between lenses that differ
in f (1267 vs 1291 px, 2026-10-19) — but every camera position scales with that choice. The accelerometer measures
metric acceleration (gravity-referenced), independent of the camera.

Method (integral form — differentiating 240 fps positions twice was useless, 2026-10-19): train_imu windows of 1 s,
IMU integrated from the GT orientation, start position/velocity fitted per window (train_imu.window_loss), gravity
direction fitted per scale. GT positions are multiplied by s on a grid; the s with the smallest median position
residual is the metric correction (s > 1 → GT too small, i.e. focal length too small).
Caveat: an accelerometer scale error looks the same as 1/s; phone accelerometers are typically within ~1 % (estimate).
  ../venv_ffs/bin/python imu_learn/scale_check.py recordings/hs_... [--gt-dir imu_gt]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import train_imu as T  # noqa: E402


def fit_g(Wt, gb0, dev, iters=80):
    g = torch.tensor(gb0, dtype=torch.float32, device=dev, requires_grad=True)
    opt = torch.optim.Adam([g], 0.02)
    for _ in range(iters):
        j = torch.randperm(len(Wt["X"]), device=dev)[:256]
        W = {k: Wt[k][j] for k in ("idx", "R", "p", "m")}
        gn = g / g.norm() * 9.80665
        _, res = T.window_loss(Wt["X"][j], W, gn)
        loss = ((res / 0.01) ** 2 * W["m"]).sum() / W["m"].sum()
        opt.zero_grad(); loss.backward(); opt.step()
    return (g / g.norm() * 9.80665).detach()


def residual(Wt, gb):
    P = []
    with torch.no_grad():
        for b in range(0, len(Wt["X"]), 128):
            W = {k: Wt[k][b:b + 128] for k in ("idx", "R", "p", "m")}
            _, res = T.window_loss(Wt["X"][b:b + 128], W, gb)
            P.append(torch.sqrt((res ** 2 * W["m"]).sum(1) / W["m"].sum(1)).cpu().numpy())
    return float(np.median(np.concatenate(P)))


def check(rec, gt_dir, grid, dev):
    print(f"{Path(rec).name}: loading", flush=True)
    T.WIN = T.HZ                                   # 1 s windows
    T.MAXF = int(240 * 1.2)
    r = T.load_rec(rec, smooth=True, gt_dir=gt_dir)
    out, gb = [], None
    for s in grid:
        r2 = dict(r, p=r["p"] * s)
        W = T.windows(r2, stride=T.HZ)                        # 1 s apart
        if len(W["X"]) > 300:                                  # cap for speed (evenly spaced)
            sel = np.linspace(0, len(W["X"]) - 1, 300).astype(int)
            W = {k: (v[sel] if isinstance(v, np.ndarray) and v.ndim and len(v) == len(W["X"]) else v) for k, v in W.items()}
        Wt = {k: (torch.tensor(v, device=dev) if isinstance(v, np.ndarray) else v) for k, v in W.items()}
        if gb is None:                                          # gravity direction does not depend on the scale
            gb = fit_g(Wt, r["gb"].astype(np.float32), dev)
        out.append(residual(Wt, gb))
        print(f"  scale {s:.3f}: median residual {out[-1] * 100:.3f} cm", flush=True)
    out = np.array(out)
    i = int(np.argmin(out))
    # parabola through the minimum and its neighbours for a finer estimate
    if 0 < i < len(grid) - 1:
        x, y = np.array(grid[i - 1:i + 2]), out[i - 1:i + 2]
        c = np.polyfit(x, y, 2)
        s_best = float(-c[1] / (2 * c[0])) if c[0] > 0 else float(grid[i])
    else:
        s_best = float(grid[i])
    return dict(rec=Path(rec).name, gt=gt_dir, windows=len(W["X"]), s_best=round(s_best, 4),
                curve={f"{s:.3f}": round(v * 100, 3) for s, v in zip(grid, out)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    ap.add_argument("--gt-dir", default="imu_gt")
    a = ap.parse_args()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    grid = [0.96, 0.98, 1.0, 1.02, 1.04, 1.06]
    res = []
    for rec in a.recs:
        r = check(rec, a.gt_dir, grid, dev)
        res.append(r)
        print(f"{r['rec']} ({r['gt']}, {r['windows']} windows of 1 s): best GT scale {r['s_best']}  "
              f"(median residual cm by scale: {r['curve']})", flush=True)
    json.dump(res, open(HERE.parent / "runs" / f"scale_check_{a.gt_dir}.json", "w"), indent=1)


if __name__ == "__main__":
    main()
