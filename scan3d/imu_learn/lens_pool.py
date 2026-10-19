"""Lens (K, distortion) for the fixed-focus 240 fps recordings from ALL of them, not just one.

The shared lens (config/lens/hs_1920x1080_f2.22.json) came from rec 131326 alone: 119 slow frames covering 5 of the
27 image-position x tilt bins. Pooling the slow frames of every fixed-focus recording covers more of the image.
Fair comparison: views are split by recording parity into fit / test; the new lens is fitted on the fit half and both
lenses are scored on the SAME test half (per-view PnP with the lens fixed, reprojection RMS) — the old lens never saw
most of the test views either.
  ../venv_ffs/bin/python imu_learn/lens_pool.py --collect REC     (one process per recording, caches corners)
  ../venv_ffs/bin/python imu_learn/lens_pool.py REC [REC ...] [--n 200] [--write]
"""
import argparse
import json
import pickle
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import aruco_scale  # noqa: E402
import lens_calib  # noqa: E402

SHARED = HERE.parent / "config" / "lens" / "hs_1920x1080_f2.22.json"
BOARD = HERE.parent / "config" / "charuco" / "board_monitor_6x6.json"


def views_of(rec):
    cache = Path(rec) / "imu_gt" / "lens_views.pkl"
    if not cache.exists():
        board = aruco_scale.Board(aruco_scale.load_board_spec(str(BOARD)))
        v, size, ns = lens_calib.collect(rec, board)
        cache.parent.mkdir(exist_ok=True)
        pickle.dump((v, size, ns), open(cache, "wb"))
    return pickle.load(open(cache, "rb"))


def test_err(views, K, d):
    e = []
    for v in views:
        ok, rv, tv = cv2.solvePnP(v[0], v[1], K, d)
        p, _ = cv2.projectPoints(v[0], rv, tv, K, d)
        e.append(np.sqrt(np.mean(np.sum((p - v[1]) ** 2, -1))))
    return float(np.median(e)), float(np.mean(e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="*")
    ap.add_argument("--collect", default=None)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--write", action="store_true", help="replace the shared lens if the pooled one scores better")
    a = ap.parse_args()
    if a.collect:
        v, size, ns = views_of(a.collect)
        print(f"{Path(a.collect).name}: slow {ns}, board views {len(v)}")
        return
    fit, test, size = [], [], None
    for k, rec in enumerate(a.recs):
        v, size, ns = views_of(rec)
        (fit if k % 2 == 0 else test).extend(v)
        print(f"  {Path(rec).name}: {len(v)} views → {'fit' if k % 2 == 0 else 'test'}")
    pick, nb = lens_calib.spread(fit, size, a.n)
    sel = [fit[i] for i in pick]
    scores = {m: lens_calib.holdout_err(sel, size, f) for m, f in lens_calib.MODELS.items()}
    best = min(scores, key=scores.get)
    rms, K, d, _, _ = lens_calib.fit(sel, size, lens_calib.MODELS[best])
    K0, d0, info0 = lens_calib.load(SHARED)
    e_new, e_old = test_err(test, K, d), test_err(test, K0, d0)
    print(f"pooled: {len(sel)} views in {nb} bins (old lens: {info0.get('n_bins')} bins), model {best}, fit rms {rms:.3f} px, "
          f"f {K[0, 0]:.1f}/{K[1, 1]:.1f} (old {K0[0, 0]:.1f})")
    print(f"TEST views ({len(test)}): new median {e_new[0]:.3f} px (mean {e_new[1]:.3f})  vs  old median {e_old[0]:.3f} px "
          f"(mean {e_old[1]:.3f})")
    res = dict(n_fit=len(sel), bins=nb, model=best, test_new=e_new, test_old=e_old, K=K.tolist(), dist=d.tolist())
    json.dump(res, open(HERE.parent / "runs" / "lens_pool.json", "w"), indent=1)
    if a.write and e_new[1] < e_old[1]:
        shutil.copy(SHARED, SHARED.with_name(SHARED.stem + "_v1_rec131326.json"))
        info = dict(model=best, rms_px=round(float(rms), 4), holdout_px={m: round(e, 4) for m, e in scores.items()},
                    n_views=len(sel), n_bins=nb, recs=[Path(r).name for r in a.recs[0::2]], size=list(size),
                    board=BOARD.name, test_px=dict(new=e_new, old=e_old))
        lens_calib.save(SHARED, K, d, info, focus_dpt=2.22)
        print(f"→ replaced {SHARED.name} (old kept as {SHARED.stem}_v1_rec131326.json)")


if __name__ == "__main__":
    main()
