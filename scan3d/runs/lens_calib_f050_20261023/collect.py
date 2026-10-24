"""Detect ChArUco corners in EVERY frame of one tripod calibration recording (std60, 0.5 dpt, 2026-10-23).

One static pose per recording, so all frames are (nearly) the same view. We keep every frame's corners and pick the
steady ones later (select.py). Board: first 4 recordings 6x4, rest 5x3 (laptop screen).
  ../venv_ffs/bin/python runs/lens_calib_f050_20261023/collect.py REC_NAME
→ views/<rec>.pkl  {rec, board, size, frames: [(idx, t_mid_ns, ids (N,), corners (N,2), sharpness)], n_frames}
  peek/<rec>.jpg   (middle frame with detections, half size)
"""
import csv
import pickle
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
S3D = HERE.parents[1]
sys.path.insert(0, str(S3D))
import aruco_scale  # noqa: E402

BOARD_6x4 = {"hs_20261023_201830_std60", "hs_20261023_203443_std60", "hs_20261023_204636_std60",
             "hs_20261023_204819_std60"}


def board_for(rec):
    if rec >= "hs_20261024_1215":          # FAR set (2026-10-24 12:15-13:10): two big ArUco markers, 1.7-2.1 m
        # 121536 / 121711 were shot with layout v2 (commit c6a2d67 12:11: 600 px markers, gap 120 px; v3 at 12:19,
        # v4 = json at 12:23) — found from a +0.6 px radial residual on exactly these two (far_diag.py)
        return "board_laptop_2markers_v2" if rec < "hs_20261024_1219" else "board_laptop_2markers.json"
    return "board_laptop_6x4.json" if rec in BOARD_6x4 else "board_laptop_5x3.json"


EDGE_LINES = True   # 2026-10-24: L-corner cornerSubPix put corners 0.1-0.7 px INSIDE the marker (far_diag.py radial bias)


def edge_corners(lin, q, module):
    """Marker outer corners from edge lines: along each side sample 24 perpendicular profiles (0.15-0.85 of the side),
    find the 50 % crossing between the black border and the white quiet zone (linear light), fit a line (TLS),
    intersect neighbouring lines. Straight-edge 50 % crossing is unbiased for a symmetric blur, unlike an L-corner."""
    lines = []
    c = q.mean(0)
    for k in range(4):
        a, b = q[k], q[(k + 1) % 4]
        t = (b - a) / np.linalg.norm(b - a)
        n = np.array([-t[1], t[0]])
        if np.dot(n, (a + b) / 2 - c) < 0:
            n = -n                                          # outward normal
        ss = np.linspace(0.15, 0.85, 24)
        offs = np.linspace(-0.45 * module, 0.45 * module, 41)
        P = []
        for s_ in ss:
            base = a + s_ * (b - a)
            xy = base[None] + offs[:, None] * n[None]
            prof = cv2.remap(lin, xy[:, 0].astype(np.float32).reshape(1, -1), xy[:, 1].astype(np.float32).reshape(1, -1),
                             cv2.INTER_LINEAR).ravel()
            lo, hi = np.median(prof[:8]), np.median(prof[-8:])   # inside (black) .. outside (white)
            if hi - lo < 0.05:
                continue
            lev = (lo + hi) / 2
            j = np.where((prof[:-1] < lev) & (prof[1:] >= lev))[0]
            if len(j) != 1:
                continue
            j = j[0]
            f = (lev - prof[j]) / (prof[j + 1] - prof[j])
            P.append(base + (offs[j] + f * (offs[j + 1] - offs[j])) * n)
        if len(P) < 8:
            return q
        P = np.array(P)
        m = P.mean(0)
        _, _, vt = np.linalg.svd(P - m)
        lines.append((m, vt[0]))
    out = []
    for k in range(4):
        (m1, d1), (m2, d2) = lines[(k - 1) % 4], lines[k]      # corner k = side (k-1) x side k
        A = np.c_[d1, -d2]
        if abs(np.linalg.det(A)) < 1e-6:
            return q
        s1, _ = np.linalg.solve(A, m2 - m1)
        out.append(m1 + s1 * d1)
    out = np.array(out)
    return out if np.max(np.linalg.norm(out - q, axis=1)) < 2.0 else q


class TwoMarkers:
    """8 marker corners (id = marker*4 + k, OpenCV order TL TR BR BL). Dictionary as tools/board2_markers.py but WITHOUT
    the detector's own subpix (its 11x11 window, like a fixed 15x15 cornerSubPix, reaches the inner corner of the 1-module
    black border: markers are only 40-97 px here, module 5-12 px → single corners jumped 2-7 px between frames, first try
    2026-10-24). cornerSubPix window half-size = 0.4 module (2..5 px). Returns (corners (N,2), ids (N,), n_markers)."""
    def __init__(self, v2=False):
        sys.path.insert(0, str(S3D / "tools"))
        import board2_markers as b2
        self.b2 = b2
        self.obj = np.concatenate([b2.OBJ[0], b2.OBJ[1]])
        if v2:                                  # layout v2: side 600 px, gap 120 px (only relative geometry matters)
            s, g = 600 * b2.MM_PER_PX / 1000, 120 * b2.MM_PER_PX / 1000
            sq = np.array([[0, 0, 0], [s, 0, 0], [s, s, 0], [0, s, 0]], float)
            self.obj = np.concatenate([sq, sq + [s + g, 0, 0]])
        self.sy = 1
        p = cv2.aruco.DetectorParameters()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self.det = cv2.aruco.ArucoDetector(b2.D, p)

    def detect(self, gray):
        corners, ids, _ = self.det.detectMarkers(gray)
        if ids is None:
            return np.zeros((0, 2)), np.zeros(0, int), 0
        pts, pid = [], []
        for c, i in zip(corners, ids.ravel()):
            if int(i) in (0, 1):
                pts.append(c.reshape(4, 2)); pid += [int(i) * 4 + k for k in range(4)]
        if not pts:
            return np.zeros((0, 2)), np.zeros(0, int), 0
        out = []
        lin = None
        for q in pts:
            side = np.mean(np.linalg.norm(q - np.roll(q, 1, 0), axis=1))
            w = int(np.clip(round(0.4 * side / 8), 2, 5))
            q = q.astype(np.float32).reshape(-1, 1, 2)
            cv2.cornerSubPix(gray, q, (w, w), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 50, 0.001))
            q = q.reshape(-1, 2)
            if EDGE_LINES:
                if lin is None:
                    lin = ((gray.astype(np.float32) / 255.0) ** 2.2)      # ASSUMED display/video gamma 2.2 → linear light
                q = edge_corners(lin, q.astype(float), side / 8)
            out.append(q)
        pts = np.concatenate(out)
        o = np.argsort(pid)
        return pts.reshape(-1, 2)[o].astype(float), np.array(pid)[o], len(pid) // 4


def main(rec):
    bj = board_for(rec)
    if "2markers" in bj:
        board = TwoMarkers(v2=bj.endswith("_v2"))
    else:
        board = aruco_scale.Board(aruco_scale.load_board_spec(str(S3D / "config" / "charuco" / bj)))
    if board.sy % 2 == 0:
        # aruco_scale turns on the legacy pattern for even rows (printed 4.5 boards), but charuco_laptop_6x4.png was made
        # with the new pattern → 0 corners with legacy, 15 without (checked on all 4 recordings, 2026-10-23).
        board.board.setLegacyPattern(False)
        d = board.detector
        board.detector = cv2.aruco.CharucoDetector(board.board, d.getCharucoParameters(), d.getDetectorParameters(),
                                                   cv2.aruco.RefineParameters())
    rd = S3D / "recordings" / rec
    rows = list(csv.DictReader(open(rd / "frames_cam5.csv")))
    t = [int(r["t_mid_ns"]) for r in rows]
    cap = cv2.VideoCapture(str(rd / "cam5.mp4"))
    frames, i, size, mid = [], 0, None, None
    n_total = len(rows)
    while True:
        ok, f = cap.read()
        if not ok:
            break
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        size = g.shape[::-1]
        c, ids, nm = board.detect(g)
        sharp = float(cv2.Laplacian(g, cv2.CV_64F).var())
        frames.append((i, t[i] if i < len(t) else -1, np.asarray(ids, int).ravel(), np.asarray(c, float).reshape(-1, 2),
                       sharp, int(nm), float(g.mean()), float((g >= 250).mean())))
        if i == min(n_total, 10 ** 9) // 2:
            mid = (f.copy(), c, ids)
        i += 1
    (HERE / "views").mkdir(exist_ok=True)
    (HERE / "peek").mkdir(exist_ok=True)
    pickle.dump(dict(rec=rec, board=bj, size=size, frames=frames, n_frames=i, n_csv=len(rows)),
                open(HERE / "views" / f"{rec}.pkl", "wb"))
    if mid is not None:
        f, c, ids = mid
        for p in np.asarray(c).reshape(-1, 2):
            cv2.circle(f, (int(p[0]), int(p[1])), 8, (0, 0, 255), 2)
        cv2.imwrite(str(HERE / "peek" / f"{rec}.jpg"), cv2.resize(f, None, fx=0.5, fy=0.5), [cv2.IMWRITE_JPEG_QUALITY, 80])
    n = [len(fr[2]) for fr in frames]
    print(f"{rec} {bj}: frames {i} (csv {len(rows)}), with corners {sum(k > 0 for k in n)}, corners max {max(n or [0])}"
          f" median {int(np.median(n or [0]))}")


if __name__ == "__main__":
    main(sys.argv[1])
