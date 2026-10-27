"""calcap set(s) -> mrcal corners cache (lenskit 2026-10-27). Runs with venv_ffs (OpenCV 5 ChArUco).

  venv_ffs/bin/python lenskit_dump.py OUT_DIR TAG SET_DIR [SET_DIR ...] --boards board_print_A1_12x9

Detection = calcap_calib.load_views (the same views the OpenCV fit uses). Writes
  OUT_DIR/corners_<TAG>.vnl  "# filename x y level", one row per grid corner (row-major, nx x ny), "0 0 -" = not seen (level - = ignore)
  OUT_DIR/meta_<TAG>.json    per image: name, file, set, cap, slot, dist, tilt, turn, lean, roll, focus_dpt + board grid
Image names are virtual (<TAG>/<k>.pgm) - mrcal-calibrate-cameras only matches them against the cache.
Only one board per mrcal problem: views of other boards are dropped (reported).
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import calcap_calib as CC  # noqa: E402
import slots as SL  # noqa: E402


def focus_of(set_dir):
    """file (relative to the set) -> focus_dpt from the app's frames.jsonl (result metadata)."""
    out = {}
    for d in sorted(os.listdir(set_dir)):
        fj = os.path.join(set_dir, d, "frames.jsonl")
        if os.path.isfile(fj):
            for ln in open(fj, encoding="utf-8"):
                try:
                    r = json.loads(ln)
                except ValueError:
                    continue
                f = (r.get("result") or {}).get("focus_dpt")
                out[f"{d}/{r.get('file')}"] = f
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("tag")
    ap.add_argument("sets", nargs="+")
    ap.add_argument("--boards", default="board_print_A1_12x9")
    ap.add_argument("--charuco-dir", default=os.path.join(CC.S3D, "config", "charuco"))
    a = ap.parse_args()
    boards = SL.load_boards(a.charuco_dir, a.boards)
    b0 = boards[0]
    if b0.type == "acircles":
        sys.exit("acircles grid is not a regular grid - mrcal cannot use it (OpenCV fit only)")
    gK, gd = SL.load_lens(CC.DEFAULT)
    views = []
    for s in a.sets:
        v, _ = CC.load_views([s], boards, gK, gd, print)
        foc = focus_of(s)
        for x in v:
            x["set"] = os.path.abspath(s)
            x["focus_dpt"] = foc.get(x["file"])
        views += v
    keep = [v for v in views if v["board"] == b0.name]
    if len(keep) < len(views):
        print(f"dropped {len(views) - len(keep)} views of other boards (mrcal: one board per problem)")
    os.makedirs(a.out, exist_ok=True)
    L = ["# filename x y level"]
    meta = []
    for k, v in enumerate(keep):
        name = f"{a.tag}/{k:04d}.pgm"
        G = np.full((b0.ny * b0.nx, 2), np.nan)
        G[v["ids"]] = v["img"].reshape(-1, 2)
        for x, y in G:
            # missing corner: level "-" = ignore this point (mrcal). NOT "- - -": mrcal reads a row starting
            # with "- " as "no board in this image" and the row count goes wrong (found 2026-10-27).
            L.append(f"{name} 0 0 -" if np.isnan(x) else f"{name} {x:.4f} {y:.4f} 0")
        # pose from the guide lens (already in the view), plus roll / turn / lean
        p = SL.pose_of(b0, v["img"].reshape(-1, 2).astype(np.float64), v["ids"], gK, gd)
        meta.append(dict(name=name, file=v["file"], set=v["set"], cap=v["cap"], slot=v["slot"], n=int(len(v["ids"])),
                         focus_dpt=v["focus_dpt"],
                         **({k2: p[k2] for k2 in ("dist", "tilt", "turn", "lean", "roll", "pos")} if p else {})))
    open(os.path.join(a.out, f"corners_{a.tag}.vnl"), "w").write("\n".join(L) + "\n")
    sp = np.ptp(b0.obj[:, :2], 0)
    grid = dict(board=b0.name, nx=b0.nx, ny=b0.ny, spacing_m=float(b0.sq),
                spacing_xy_m=[float(sp[0] / (b0.nx - 1)), float(sp[1] / (b0.ny - 1))], scale_xy=list(b0.scale_xy),
                flatness_mm=b0.spec.get("flatness_mm"), measured_span_x_mm=b0.spec.get("measured_span_x_mm"),
                measured_span_y_mm=b0.spec.get("measured_span_y_mm"))
    json.dump(dict(tag=a.tag, sets=[os.path.abspath(s) for s in a.sets], grid=grid, images=meta),
              open(os.path.join(a.out, f"meta_{a.tag}.json"), "w"), indent=1)
    print(f"{a.tag}: {len(meta)} images, {len({m['cap'] for m in meta})} poses, {sum(m['n'] for m in meta)} corners -> {a.out}")


if __name__ == "__main__":
    main()
