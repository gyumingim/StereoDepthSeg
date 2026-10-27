"""Printable ChArUco boards for the lens re-calibration (lenskit, 2026-10-27).

  python make_print_board.py OUT_DIR [--boards board_print_A1_12x9 board_print_A2_12x9 board_print_A3_12x9]
         [--check]            # rasterise each PDF again (PyMuPDF) and detect it with OpenCV -> ids, corner spacing

Board geometry comes ONLY from config/charuco/<board>.json (dict, squares, square_mm, marker_ratio, ids_start, paper_mm).
The PDF is vector: every black square and every black marker module is a filled rectangle at its exact mm position
(no raster image, no scaling). Page size = the paper; print at "actual size / 100 %" (no "fit to page").
Margins carry: board name, dictionary, marker id range, square / marker size, and two 100 mm ruler bars (horizontal and
vertical) to check the print scale in BOTH directions before mounting.
Needs: OpenCV >= 4.7 (aruco CharucoBoard API) + matplotlib. --check also needs PyMuPDF (fitz).
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
from matplotlib.collections import PatchCollection  # noqa: E402
from matplotlib import font_manager  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
S3D = os.path.abspath(os.path.join(HERE, "..", ".."))
CHARUCO = os.path.join(S3D, "config", "charuco")
MM_PER_IN = 25.4


def korean_font():
    for p in (r"C:\Windows\Fonts\malgun.ttf", "/mnt/c/Windows/Fonts/malgun.ttf",
              "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"):
        if os.path.exists(p):
            font_manager.fontManager.addfont(p)
            return font_manager.FontProperties(fname=p).get_name()
    return None


def board_layout(j):
    """-> (black squares [(x, y, w, h)], marker list [(id, x0, y0, side, bits (n+2)x(n+2) 1=black)]), mm, y down."""
    sx, sy, sq = j["squares_x"], j["squares_y"], float(j["square_mm"])
    msq = sq * float(j["marker_ratio"])
    D = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, j["dict"]))
    nmark = (sx * sy) // 2
    ids = j.get("ids") or list(range(int(j.get("ids_start", 0)), int(j.get("ids_start", 0)) + nmark))
    b = cv2.aruco.CharucoBoard((sx, sy), sq / 1000.0, msq / 1000.0, D, np.asarray(ids, np.int32))
    b.setLegacyPattern(bool(j.get("legacy_pattern", sy % 2 == 0)))
    objp = b.getObjPoints()
    bid = np.asarray(b.getIds()).ravel()
    nb = D.markerSize
    marker_sq = set()
    markers = []
    for mid, c in zip(bid, objp):
        c = np.asarray(c).reshape(-1, 3) * 1000.0           # mm, corners TL TR BR BL
        assert abs((c[:, 0].max() - c[:, 0].min()) - msq) < 1e-3, (c, msq)    # float32 object points
        i, k = int(c[:, 0].mean() // sq), int(c[:, 1].mean() // sq)            # square that holds the marker
        x0, y0 = i * sq + (sq - msq) / 2, k * sq + (sq - msq) / 2              # exact mm (OpenCV centres markers)
        assert abs(x0 - c[:, 0].min()) < 1e-3 and abs(y0 - c[:, 1].min()) < 1e-3
        img = cv2.aruco.generateImageMarker(D, int(mid), nb + 2)   # 1 px per module incl. the black border
        markers.append((int(mid), x0, y0, msq, (img < 128).astype(np.uint8)))
        marker_sq.add((i, k))
    blacks = [(i * sq, k * sq, sq, sq) for k in range(sy) for i in range(sx) if (i, k) not in marker_sq]
    assert len(blacks) + len(markers) == sx * sy, (len(blacks), len(markers))
    return blacks, markers, ids


def ruler(ax, x0, y0, horizontal, fp, fs):
    """100 mm bar with ticks every 10 mm (long at 0 / 50 / 100) and 1 mm ticks in the first 10 mm."""
    L, T = 100.0, 1.6
    if horizontal:
        ax.add_patch(Rectangle((x0, y0), L, T, fc="k", ec="none"))
        for t in range(0, 101, 10):
            h = 4.0 if t % 50 == 0 else 2.5
            ax.add_patch(Rectangle((x0 + t - 0.15, y0 - h), 0.3, h, fc="k", ec="none"))
        for t in range(1, 10):
            ax.add_patch(Rectangle((x0 + t - 0.1, y0 - 1.2), 0.2, 1.2, fc="k", ec="none"))
        ax.text(x0 + 50, y0 + T + 0.8, "100 mm", ha="center", va="top", fontsize=fs, fontproperties=fp)
    else:
        ax.add_patch(Rectangle((x0, y0), T, L, fc="k", ec="none"))
        for t in range(0, 101, 10):
            h = 4.0 if t % 50 == 0 else 2.5
            ax.add_patch(Rectangle((x0 - h, y0 + t - 0.15), h, 0.3, fc="k", ec="none"))
        for t in range(1, 10):
            ax.add_patch(Rectangle((x0 - 1.2, y0 + t - 0.1), 1.2, 0.2, fc="k", ec="none"))
        ax.text(x0 + T + 0.8, y0 + 50, "100 mm", ha="left", va="center", rotation=270, fontsize=fs, fontproperties=fp)


def make_pdf(name, out_dir):
    j = json.load(open(os.path.join(CHARUCO, name + ".json"), encoding="utf-8"))
    PW, PH = [float(x) for x in j["paper_mm"].split("x")]
    sx, sy, sq = j["squares_x"], j["squares_y"], float(j["square_mm"])
    BW, BH = sx * sq, sy * sq
    ox, oy = (PW - BW) / 2, (PH - BH) / 2
    blacks, markers, ids = board_layout(j)
    kf = korean_font()
    fp = font_manager.FontProperties(family=kf) if kf else None
    fig = plt.figure(figsize=(PW / MM_PER_IN, PH / MM_PER_IN))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, PW); ax.set_ylim(PH, 0); ax.set_aspect("equal"); ax.axis("off")
    rects = [Rectangle((ox + x, oy + y), w, h) for x, y, w, h in blacks]
    for mid, x0, y0, side, bits in markers:
        n = bits.shape[0]
        m = side / n
        # black module runs per row AND per column (same union): the column runs cover the hairline seams between
        # rows that some viewers / printers draw where two rectangles only touch (exact geometry, no overlap outside)
        for tr in (False, True):
            B = bits.T if tr else bits
            for r in range(n):
                c = 0
                while c < n:
                    if B[r, c]:
                        c1 = c
                        while c1 < n and B[r, c1]:
                            c1 += 1
                        if tr:
                            rects.append(Rectangle((ox + x0 + r * m, oy + y0 + c * m), m, (c1 - c) * m))
                        else:
                            rects.append(Rectangle((ox + x0 + c * m, oy + y0 + r * m), (c1 - c) * m, m))
                        c = c1
                    else:
                        c += 1
    ax.add_collection(PatchCollection(rects, facecolor="k", edgecolor="none", linewidth=0, antialiased=False))
    mg = min(oy, ox)
    fs = 7 if mg < 16 else 9
    msq = sq * float(j["marker_ratio"])
    t1 = (f"{name}   ChArUco {sx}x{sy} squares, square {sq:.1f} mm, marker {msq:.1f} mm, {j['dict']}, "
          f"marker ids {ids[0]}-{ids[-1]}, inner corners {sx - 1}x{sy - 1}, legacy_pattern={j.get('legacy_pattern')}")
    t2 = ("인쇄: 실제 크기 100 % (\"페이지에 맞춤\" 끄기) · 무광 종이 · 인쇄 후 아래·오른쪽 100 mm 막대를 자로 확인 "
          f"(±0.2 mm) · 첫 안쪽 모서리→끝 안쪽 모서리: 가로 {10 * sq:g} mm (10칸), 세로 {7 * sq:g} mm (7칸) 을 재서 json 에 기록")
    ax.text(ox, oy - 1.5, t1, ha="left", va="bottom", fontsize=fs, fontproperties=fp)
    ax.text(ox, oy + BH + 1.5, t2, ha="left", va="top", fontsize=fs - 1, fontproperties=fp)
    # rulers: horizontal under the board (right part), vertical right of the board
    ruler(ax, ox + BW - 100.0, PH - max(4.0, oy * 0.35) - 1.6, True, fp, fs - 1)
    ruler(ax, min(PW - 12.0, ox + BW + max(6.0, ox * 0.4)), oy + (BH - 100.0) / 2, False, fp, fs - 1)
    os.makedirs(out_dir, exist_ok=True)
    pdf = os.path.join(out_dir, os.path.basename(j.get("print_pdf", f"{name}.pdf")))
    fig.savefig(pdf, format="pdf", dpi=72, metadata={"Title": f"{name} (print at 100 %)", "Creator": "lenskit make_print_board.py"})
    plt.close(fig)
    return pdf, j, (ox, oy)


def check_pdf(pdf, j, origin, px_per_mm=4.0):
    """Rasterise with PyMuPDF, detect with OpenCV ChArUco, compare corner positions with the json geometry."""
    import fitz
    doc = fitz.open(pdf)
    pg = doc[0]
    PW, PH = [float(x) for x in j["paper_mm"].split("x")]
    size_ok = abs(pg.rect.width / 72 * MM_PER_IN - PW) < 0.05 and abs(pg.rect.height / 72 * MM_PER_IN - PH) < 0.05
    zoom = px_per_mm * MM_PER_IN / 72.0
    pix = pg.get_pixmap(matrix=fitz.Matrix(zoom, zoom), colorspace=fitz.csGRAY, alpha=False)
    g = np.frombuffer(pix.samples, np.uint8).reshape(pix.height, pix.width).copy()
    sx, sy, sq = j["squares_x"], j["squares_y"], float(j["square_mm"])
    D = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, j["dict"]))
    ids = list(range(int(j["ids_start"]), int(j["ids_start"]) + (sx * sy) // 2))
    b = cv2.aruco.CharucoBoard((sx, sy), sq / 1000.0, sq * j["marker_ratio"] / 1000.0, D, np.asarray(ids, np.int32))
    b.setLegacyPattern(bool(j.get("legacy_pattern", False)))
    det = cv2.aruco.CharucoDetector(b)
    cc, ci, mc, mi = det.detectBoard(g)
    n = 0 if ci is None else len(ci)
    res = dict(pdf=os.path.basename(pdf), page_mm=[round(pg.rect.width / 72 * MM_PER_IN, 2), round(pg.rect.height / 72 * MM_PER_IN, 2)],
               page_size_ok=bool(size_ok), raster_px=[pix.width, pix.height], markers=0 if mi is None else int(len(mi)),
               corners=int(n), corners_expected=(sx - 1) * (sy - 1))
    if n:
        obj = np.asarray(b.getChessboardCorners()).reshape(-1, 3)[np.asarray(ci).ravel()] * 1000.0
        exp = (obj[:, :2] + np.array(origin)) * px_per_mm
        d = np.asarray(cc).reshape(-1, 2) - exp
        shift = d.mean(0)                     # constant offset = raster pixel-centre convention (<= 0.5 px)
        err = np.linalg.norm(d - shift, axis=1)
        sc = np.polyfit(exp[:, 0], np.asarray(cc).reshape(-1, 2)[:, 0], 1)[0], np.polyfit(exp[:, 1], np.asarray(cc).reshape(-1, 2)[:, 1], 1)[0]
        res["corner_shift_px"] = [round(float(x), 3) for x in shift]
        res["corner_pos_err_mm"] = dict(max=round(float(err.max() / px_per_mm), 4), mean=round(float(err.mean() / px_per_mm), 4))
        res["scale_x_y"] = [round(float(x), 6) for x in sc]
    # a wrong dictionary / id range must NOT detect this board (laptop 5x3 board ids 0-6)
    other = cv2.aruco.CharucoBoard((5, 3), 0.06, 0.035, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
    oc, oi, _, _ = cv2.aruco.CharucoDetector(other).detectBoard(g)
    res["laptop_5x3_detector_corners"] = 0 if oi is None else int(len(oi))
    return res, g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--boards", nargs="*", default=["board_print_A1_12x9", "board_print_A2_12x9", "board_print_A3_12x9"])
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    allres = []
    for nm in a.boards:
        pdf, j, org = make_pdf(nm, a.out)
        print("PDF", pdf, os.path.getsize(pdf), "bytes")
        if a.check:
            r, g = check_pdf(pdf, j, org)
            cv2.imwrite(os.path.splitext(pdf)[0] + "_check_4pxmm.png", g)
            print("  check", json.dumps(r, ensure_ascii=False))
            allres.append(r)
    if a.check:
        json.dump(allres, open(os.path.join(a.out, "print_check.json"), "w"), indent=1)
        bad = [r for r in allres if not (r["page_size_ok"] and r["corners"] == r["corners_expected"]
                                         and r.get("corner_pos_err_mm", {}).get("max", 9) < 0.05
                                         and max(abs(x) for x in r.get("corner_shift_px", [9])) <= 0.75
                                         and max(abs(x - 1) for x in r.get("scale_x_y", [9])) < 2e-4
                                         and r["laptop_5x3_detector_corners"] == 0)]
        print("CHECK", "PASS" if not bad else f"FAIL {[r['pdf'] for r in bad]}")
        sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
