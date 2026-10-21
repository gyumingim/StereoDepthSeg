#!/usr/bin/env python3
"""Re-scale finished ground truth to a new board square size (one command, seconds — no re-run of imu_gt/spline).

Why it is exact: the board corners are square_mm × (integer grid), so a different square multiplies every camera
position in board coordinates by new/old and leaves every rotation, time offset dt, R_ci and gyro bias unchanged.
PnP (per-frame GT) and the camera-only spline fit (corner reprojection) are scale-equivariant; only the IMU-aided spline
(imu_gt_spl_imu: the accelerometer fixes metric scale) is NOT — it is marked stale and must be re-fitted.
Tiny caveat: distance gates (< 1.2 m in batch_sync / spline init / train_imu) pick frames by distance, so a few frames near
the gate may change membership after re-scaling (not re-evaluated here).

  # the user measured the lit laptop screen width W mm (square = W / 7.07):
  ../venv_ffs/bin/python imu_learn/rescale_gt.py --screen-width-mm W --update-board recordings/hs_20261020_*
  # or directly a square size:
  ../venv_ffs/bin/python imu_learn/rescale_gt.py --square-mm 41.3 --update-board recordings/hs_20261020_*
--update-board writes the new square into the board json recorded in the GT (config/charuco/board_laptop_6x6.json)
so later imu_gt/spline runs use it too. Dry run with --dry. Each gt dir keeps a history in board.json ("rescaled").
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from imu_gt import provenance  # noqa: E402

GT_DIRS = ["imu_gt", "imu_gt_rs", "imu_gt_spl", "imu_gt_spl_imu"]
POS_KEYS = ["p_bc"]                     # arrays in board coordinates (m)
VISIBLE_SQUARES_X = 7.07                # laptop screen: lit width = 7.07 squares (docs/overnight_input_audit.md)


def rescale_dir(d, new_sq, board_name, dry=False):
    f = d / "gt.npz"
    g = dict(np.load(f, allow_pickle=True))
    if "square_mm" not in g:
        return f"{d}: no square_mm recorded (made before 2026-10-20 board provenance) — skipped"
    bj = str(g.get("board_json", ""))
    if board_name and Path(bj).name != board_name:
        return f"{d}: board {Path(bj).name} ≠ {board_name} — skipped"
    old = float(g["square_mm"])
    k = new_sq / old
    if abs(k - 1) < 1e-9:
        return f"{d}: already {new_sq} mm"
    msg = f"{d}: square {old} → {new_sq} mm (positions × {k:.5f})"
    if dry:
        return msg + " [dry]"
    for key in POS_KEYS:
        g[key] = g[key] * k
    g["square_mm"] = np.float64(new_sq)
    np.savez(f, **g)
    bp = d / "board.json"
    info = json.load(open(bp)) if bp.exists() else {}
    hist = info.get("rescaled", [])
    hist.append(dict(at=time.strftime("%Y-%m-%d %H:%M:%S"), old_square_mm=old, new_square_mm=new_sq, factor=k,
                     code=provenance([__file__])["code_sha256"]))
    info.update(square_mm=new_sq, rescaled=hist)
    for key in ("dist_m_median",):
        if info.get(key) is not None:
            info[key] = info[key] * k
    if d.name == "imu_gt_spl_imu":
        info["stale"] = "IMU-aided spline is not scale-equivariant: re-run spline_gt.py --imu"
        msg += "  ⚠ imu_gt_spl_imu: positions scaled but the accelerometer fit is not scale-free → re-run spline_gt.py --imu"
    json.dump(info, open(bp, "w"), indent=1, ensure_ascii=False)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    grp = ap.add_mutually_exclusive_group(required=True)
    grp.add_argument("--square-mm", type=float)
    grp.add_argument("--screen-width-mm", type=float, help=f"lit laptop screen width; square = W / {VISIBLE_SQUARES_X}")
    ap.add_argument("--board-name", default="board_laptop_6x6.json", help="only GTs made with this board ('' = any)")
    ap.add_argument("--update-board", action="store_true", help="also write the new square into the board json")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--source", default=None, help="text for the board json square_mm_source (where the value comes from)")
    a = ap.parse_args()
    new_sq = a.square_mm if a.square_mm else a.screen_width_mm / VISIBLE_SQUARES_X
    new_sq = round(new_sq, 4)
    print(f"new square {new_sq} mm" + (f" (screen {a.screen_width_mm} mm / {VISIBLE_SQUARES_X})" if a.screen_width_mm else ""))
    boards = set()
    for r in a.recs:
        for gd in GT_DIRS:
            d = Path(r) / gd
            if (d / "gt.npz").exists():
                g = np.load(d / "gt.npz", allow_pickle=True)
                if "board_json" in g:
                    boards.add(str(g["board_json"]))
                print(rescale_dir(d, new_sq, a.board_name, a.dry))
    if a.update_board and not a.dry:
        for b in boards:
            if a.board_name and Path(b).name != a.board_name:
                continue
            spec = json.loads(Path(b).read_text())
            hist = spec.get("square_history", [])
            hist.append(dict(at=time.strftime("%Y-%m-%d %H:%M:%S"), square_mm=spec.get("square_mm"),
                             source=spec.get("square_mm_source")))
            spec.update(square_mm=new_sq, square_estimated=not a.screen_width_mm,
                        square_mm_source=a.source or (f"measured screen width {a.screen_width_mm} mm / {VISIBLE_SQUARES_X}"
                                                      if a.screen_width_mm else f"set by rescale_gt.py --square-mm {a.square_mm}"),
                        square_history=hist)
            Path(b).write_text(json.dumps(spec, indent=1, ensure_ascii=False) + "\n")
            print(f"board {b}: square_mm = {new_sq}")
    print("next: re-run imu_learn/scale_ls.py (check s ≈ method bias) and spline_gt.py --imu if imu_gt_spl_imu is used")


if __name__ == "__main__":
    main()
