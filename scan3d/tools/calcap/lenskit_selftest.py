"""lenskit self-test (PC only, no phone): default behaviour unchanged + new board types detect.

  ~/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/tools/calcap/lenskit_selftest.py [--pdf-png DIR]

1. Old slots.py (git calcap-20261025) vs new slots.py with the DEFAULT boards / 33 slots: for 400 random laptop-board
   poses -> pose_of values, fills() for every slot (tol 0 and 1), hint(), target_outline(), nearest_missing() equal.
2. Synthetic images (rendered with a known lens): ChArUco A1/A2/A3 print boards, a chessboard and a symmetric /
   asymmetric circle grid from temporary json files -> detected, pose within 0.5 deg / 5 mm of the truth.
3. Optional: the rasterised print PDFs (make_print_board.py --check PNGs) detected with slots.Board (OpenCV 5).
Prints PASS / FAIL lines, exit code 1 on any FAIL.
"""
import argparse
import importlib.util
import json
import math
import os
import subprocess
import sys
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import slots as SL  # noqa: E402

S3D = os.path.abspath(os.path.join(HERE, "..", ".."))
CH = os.path.join(S3D, "config", "charuco")
LENS = os.path.join(S3D, "config", "lens", "std_1920x1080_f0.50_cal20261024d.json")
FAIL = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  {info}" if info else ""), flush=True)
    if not ok:
        FAIL.append(name)


def old_slots():
    src = subprocess.run(["git", "-C", S3D, "show", "calcap-20261025:scan3d/tools/calcap/slots.py"],
                         capture_output=True, text=True, check=True).stdout
    d = tempfile.mkdtemp()
    p = os.path.join(d, "slots_old.py")
    open(p, "w", encoding="utf-8").write(src)
    spec = importlib.util.spec_from_file_location("slots_old", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def rand_pose(rng, board):
    turn, lean = rng.uniform(-55, 55), rng.uniform(-50, 50)
    R = SL._R_from(turn, lean, rng.uniform(-15, 15))
    d = rng.uniform(0.35, 1.3)
    ray = np.array([rng.uniform(-0.45, 0.45), rng.uniform(-0.25, 0.25), 1.0])
    t = ray / np.linalg.norm(ray) * d - R @ board.centre
    return R, t


def test_default(rng):
    O = old_slots()
    K, D = SL.load_lens(LENS)
    bn, bo = SL.load_boards(CH), O.load_boards(CH)
    check("default boards / slots", [b.name for b in bn] == [b.name for b in bo] and SL.SLOTS == O.SLOTS
          and all(np.array_equal(a.obj, b.obj) and a.min_corners == b.min_corners for a, b in zip(bn, bo)),
          f"{[b.name for b in bn]} {len(SL.SLOTS)} slots")
    nd = nf = nh = 0
    bad = []
    for k in range(400):
        b_new, b_old = bn[k % 2], bo[k % 2]
        R, t = rand_pose(rng, b_new)
        pts, _ = cv2.projectPoints(b_new.obj, cv2.Rodrigues(R)[0], t, K, D)
        pts = pts.reshape(-1, 2) + rng.normal(0, 0.2, (b_new.n, 2))
        ids = np.arange(b_new.n)
        pn = SL.pose_of(b_new, pts, ids, K, D)
        po = O.pose_of(b_old, pts, ids, K, D)
        if (pn is None) != (po is None):
            bad.append(("pose None", k)); continue
        if pn is None:
            continue
        nd += 1
        for key in po:
            if key == "edge":
                same = all(abs(po[key][s] - pn[key][s]) < 1e-9 for s in po[key])
            elif isinstance(po[key], float):
                same = abs(po[key] - pn[key]) < 1e-9
            else:
                same = po[key] == pn[key]
            if not same:
                bad.append(("pose " + key, k))
        for s_new, s_old in zip(SL.SLOTS, O.SLOTS):
            for tol in (0.0, 1.0):
                if SL.fills(s_new, pn, tol) != O.fills(s_old, po, tol):
                    bad.append(("fills", s_new["id"], k))
                nf += 1
            if SL.hint(s_new, pn) != O.hint(s_old, po):
                bad.append(("hint", s_new["id"], k))
            nh += 1
        mn = SL.nearest_missing(SL.SLOTS[k % 7:], pn)
        mo = O.nearest_missing(O.SLOTS[k % 7:], po)
        if mn["id"] != mo["id"]:
            bad.append(("nearest", k))
    for s_new, s_old in zip(SL.SLOTS, O.SLOTS):
        for b_new, b_old in zip(bn, bo):
            if not np.allclose(SL.target_outline(s_new, b_new, K, D), O.target_outline(s_old, b_old, K, D), atol=1e-9):
                bad.append(("outline", s_new["id"], b_new.name))
    check("default behaviour identical to calcap-20261025 slots.py", not bad,
          f"{nd} poses, {nf} fills, {nh} hints, 66 outlines; differences {bad[:5]}")


def render_plane(img_board, px_per_m, R, t, K, D, W=1920, H=1080, bg=170, origin=(0.0, 0.0), blur=0.0, noise=1.5, rng=None):
    """Board image (board metres * px_per_m + origin px) seen through K, D at pose (R, t)."""
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    nrm = cv2.undistortPoints(np.stack([uu, vv], -1).reshape(-1, 1, 2), K, D).reshape(H, W, 2)
    Hn = np.stack([R[:, 0], R[:, 1], t], 1)
    Hi = np.linalg.inv(Hn)
    p = np.einsum("ij,hwj->hwi", Hi, np.concatenate([nrm, np.ones((H, W, 1))], -1))
    X, Y = p[..., 0] / p[..., 2], p[..., 1] / p[..., 2]
    mx = (X * px_per_m + origin[0]).astype(np.float32)
    my = (Y * px_per_m + origin[1]).astype(np.float32)
    big = cv2.copyMakeBorder(img_board, 0, 0, 0, 0, cv2.BORDER_CONSTANT)
    # supersample-ish: board image is large, use area-ish linear sampling of a pre-blurred board
    out = cv2.remap(big, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=bg)
    out = out.astype(np.float32)
    if blur > 0:
        out = cv2.GaussianBlur(out, (0, 0), blur)
    if rng is not None and noise > 0:
        out += rng.normal(0, noise, out.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def board_image(b, px_per_m):
    """Board picture for type charuco (OpenCV generateImage), chessboard / circles (drawn), origin in px of board (0, 0)."""
    margin = int(round(1.5 * b.sq * px_per_m))
    if b.type == "charuco":
        w, h = int(round(b.sx * b.sq * px_per_m)), int(round(b.sy * b.sq * px_per_m))
        img = b.board.generateImage((w, h), marginSize=0, borderBits=1)
        img = cv2.copyMakeBorder(img, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=255)
        return img, (margin, margin)
    ext = np.ptp(b.obj[:, :2], 0)
    w, h = int(round(ext[0] * px_per_m)) + 2 * margin, int(round(ext[1] * px_per_m)) + 2 * margin
    img = np.full((h, w), 255, np.uint8)
    s = b.sq * px_per_m
    if b.type == "chessboard":
        for r in range(-1, b.ny):
            for c in range(-1, b.nx):
                if (r + c) % 2 == 0:
                    x0, y0 = margin + c * s, margin + r * s
                    cv2.rectangle(img, (int(round(x0)), int(round(y0))), (int(round(x0 + s)) - 1, int(round(y0 + s)) - 1), 0, -1)
    else:
        rad = s * (0.3 if b.type == "circles" else 0.25)
        for (x, y, _) in b.obj:
            cv2.circle(img, (int(round(margin * 16 + x * px_per_m * 16)), int(round(margin * 16 + y * px_per_m * 16))),
                       int(round(rad * 16)), 0, -1, lineType=cv2.LINE_AA, shift=4)
    return img, (margin, margin)


def test_types(rng):
    K, D = SL.load_lens(LENS)
    tmp = tempfile.mkdtemp()
    specs = {
        "tmp_chess": dict(type="chessboard", inner_x=9, inner_y=6, square_mm=40.0),
        "tmp_circles": dict(type="circles", cols=7, rows=5, spacing_mm=40.0),
        "tmp_acircles": dict(type="acircles", cols=4, rows=11, spacing_mm=20.0),
    }
    for n, j in specs.items():
        json.dump(j, open(os.path.join(tmp, n + ".json"), "w"))
    names = [("board_print_A1_12x9", CH, 1.0), ("board_print_A2_12x9", CH, 0.95), ("board_print_A3_12x9", CH, 0.9)] + \
            [(n, tmp, 0.7) for n in specs]
    for n, d, dist in names:
        b = SL.load_boards(d, (n,))[0]
        ppm = 8000.0 if b.type == "charuco" else 6000.0
        img, org = board_image(b, ppm)
        img = cv2.GaussianBlur(img, (0, 0), 1.2)
        for turn, lean, roll in ((0, 0, 0), (35, -20, 30)):
            R = SL._R_from(turn, lean, roll)
            t = np.array([0.0, 0.0, dist]) - R @ b.centre
            g = render_plane(img, ppm, R, t, K, D, origin=org, blur=0.6, rng=rng)
            c, i = b.detect(g)
            p = SL.pose_of(b, c, i, K, D) if len(i) >= b.min_corners else None
            ok = p is not None and len(i) == b.n
            info = f"{len(i)}/{b.n} corners"
            if p is not None:
                Rt = cv2.Rodrigues(np.array(p["rvec"]))[0]
                dang = math.degrees(math.acos(min(1.0, (np.trace(Rt.T @ R) - 1) / 2)))
                dt = float(np.linalg.norm(np.array(p["tvec"]) - t))
                # symmetric chessboards / circle grids may come back turned by 180 deg (same calibration data)
                if dang > 90:
                    Rz = np.diag([-1.0, -1.0, 1.0])
                    dang = math.degrees(math.acos(min(1.0, (np.trace((Rt @ Rz).T @ R) - 1) / 2)))
                    dt = float("nan")          # origin moves to the other end of the grid; translation not compared
                ok = ok and dang < 0.5 and (dt < 0.005 or b.type != "charuco")
                info += f", rot err {dang:.3f} deg, t err {dt * 1000:.1f} mm, turn {p['turn']:.1f} lean {p['lean']:.1f} roll {p['roll']:.1f}"
            check(f"detect {n} turn {turn} lean {lean} roll {roll}", ok, info)
    # the A1 detector must ignore the laptop 5x3 board (marker ids 0-6) in view
    a1 = SL.load_boards(CH, ("board_print_A1_12x9",))[0]
    lap = SL.load_boards(CH, ("board_laptop_5x3",))[0]
    img, org = board_image(lap, 4000.0)
    R = SL._R_from(0, 0); t = np.array([0.0, 0.0, 0.8]) - R @ lap.centre
    g = render_plane(img, 4000.0, R, t, K, D, origin=org, rng=rng)
    c1, i1 = a1.detect(g); c2, i2 = lap.detect(g)
    check("A1 detector ignores the laptop 5x3 board", len(i1) == 0 and len(i2) == lap.n, f"A1 {len(i1)}, laptop {len(i2)}")


def test_pdf_png(d):
    for n, f in (("board_print_A1_12x9", "charuco_print_A1_12x9_60mm_check_4pxmm.png"),
                 ("board_print_A2_12x9", "charuco_print_A2_12x9_42mm_check_4pxmm.png"),
                 ("board_print_A3_12x9", "charuco_print_A3_12x9_30mm_check_4pxmm.png")):
        p = os.path.join(d, f)
        if not os.path.exists(p):
            check("pdf png " + f, False, "missing"); continue
        b = SL.load_boards(CH, (n,))[0]
        g = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        c, i = b.detect(g)
        # corner spacing along x in the raster (4 px / mm) vs the board json
        ok = len(i) == b.n
        if ok:
            grid = c[np.argsort(i)].reshape(b.ny, b.nx, 2)
            sx = np.median(np.diff(grid[:, :, 0], axis=1)) / 4.0
            sy = np.median(np.diff(grid[:, :, 1], axis=0)) / 4.0
            ok = abs(sx - b.sq * 1000) < 0.02 and abs(sy - b.sq * 1000) < 0.02
            check("pdf raster detected by slots.Board (OpenCV 5) " + n, ok, f"{len(i)} corners, square {sx:.3f} x {sy:.3f} mm")
        else:
            check("pdf raster detected by slots.Board (OpenCV 5) " + n, False, f"{len(i)} corners")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pdf-png", default="")
    a = ap.parse_args()
    rng = np.random.default_rng(7)
    test_default(rng)
    test_types(rng)
    if a.pdf_png:
        test_pdf_png(a.pdf_png)
    print("RESULT:", "PASS" if not FAIL else f"FAIL {FAIL}")
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
