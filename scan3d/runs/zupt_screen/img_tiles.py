#!/usr/bin/env python3
"""zupt_screen step 5 (exploratory): where in the image does still-period jitter come from?
Features spread over a 8x6 grid (like OpenVINS grid FAST, not board-concentrated), tracked between consecutive 60 Hz
fed frames (gray -> 640x360 -> equalizeHist). Per tile: median |displacement|. Mean over all features per pair ~ ZUPT
disparity stand-in. Writes PNG overlays runs/zupt_screen/tiles_<rec>_<a>.png and tiles.json."""
import sys, os, csv, json, time
from pathlib import Path
from multiprocessing import Pool
import numpy as np
import cv2
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "imu_learn")); os.chdir(ROOT)
W, H, GX, GY = 640, 360, 8, 6
JOBS = [("hs_20261021_214550", 0), ("hs_20261021_214550", 1), ("hs_20261021_214550", 2),
        ("hs_20261021_213341", 1), ("hs_20261021_215348", 0), ("hs_20261021_215348", 1), ("hs_20261021_213955", 1)]


def grid_feats(img):
    pts = []
    for gy in range(GY):
        for gx in range(GX):
            x0, x1 = gx * W // GX, (gx + 1) * W // GX
            y0, y1 = gy * H // GY, (gy + 1) * H // GY
            m = np.zeros((H, W), np.uint8); m[y0:y1, x0:x1] = 1
            p = cv2.goodFeaturesToTrack(img, 6, 0.001, 10, mask=m)
            if p is not None:
                pts.append(p.reshape(-1, 2))
    return np.concatenate(pts).astype(np.float32).reshape(-1, 1, 2) if pts else None


def run(job):
    import i3_confirm as ic
    rec, si = job
    x = ic.inputs(rec)
    a, b = x["stills_s"][si]
    a, b = a + 1.0, min(b - 0.5, a + 11.0)
    fr = list(csv.DictReader(open(ROOT / "recordings" / rec / "frames_cam5.csv")))
    tm = np.array([int(r["t_mid_ns"]) for r in fr]); trel = (tm - tm[0]) / 1e9
    ks = np.where((trel >= max(a, 0)) & (trel <= b))[0]
    grid = np.arange(trel[ks[0]], trel[ks[-1]], 1 / 60)
    j = np.clip(np.searchsorted(trel[ks], grid), 1, len(ks) - 1)
    j = np.where(np.abs(trel[ks[j - 1]] - grid) < np.abs(trel[ks[j]] - grid), j - 1, j)
    fed = sorted(set(ks[j].tolist()))
    cap = cv2.VideoCapture(str(ROOT / "recordings" / rec / "cam5.mp4"))
    frames, k, need = {}, 0, set(fed)
    color0 = None
    while k <= fed[-1]:
        if not cap.grab():
            break
        if k in need:
            _, f = cap.retrieve()
            if color0 is None:
                color0 = cv2.resize(f, (W, H), interpolation=cv2.INTER_AREA)
            frames[k] = cv2.equalizeHist(cv2.resize(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (W, H), interpolation=cv2.INTER_AREA))
        k += 1
    tiles = [[[] for _ in range(GX)] for _ in range(GY)]
    pair_mean = []
    for f0, f1 in zip(fed[:-1], fed[1:]):
        p0 = grid_feats(frames[f0])
        p1, st, _ = cv2.calcOpticalFlowPyrLK(frames[f0], frames[f1], p0, None, winSize=(15, 15), maxLevel=3)
        st = st.ravel() == 1
        d = np.linalg.norm((p1 - p0).reshape(-1, 2), axis=1)[st]
        pp = p0.reshape(-1, 2)[st]
        if len(d) < 20:
            continue
        pair_mean.append(float(d.mean()))
        for (xx, yy), dd in zip(pp, d):
            tiles[min(GY - 1, int(yy * GY / H))][min(GX - 1, int(xx * GX / W))].append(dd)
    med = np.array([[np.median(t) if t else np.nan for t in row] for row in tiles])
    p90 = np.array([[np.percentile(t, 90) if t else np.nan for t in row] for row in tiles])
    img = color0.copy()
    for gy in range(GY):
        for gx in range(GX):
            v = med[gy, gx]
            x0, y0 = gx * W // GX, gy * H // GY
            col = (0, 255, 0) if v < 0.1 else ((0, 200, 255) if v < 0.3 else (0, 0, 255))
            cv2.rectangle(img, (x0 + 1, y0 + 1), (x0 + W // GX - 2, y0 + H // GY - 2), col, 2)
            cv2.putText(img, f"{v:.2f}", (x0 + 6, y0 + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    cv2.putText(img, f"{rec[-6:]} still {si} {a:.1f}-{b:.1f}s pair-mean med {np.median(pair_mean):.3f}px", (6, H - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    png = ROOT / f"runs/zupt_screen/tiles_{rec[-6:]}_s{si}.png"
    cv2.imwrite(str(png), img)
    pm = np.array(pair_mean)
    res = dict(rec=rec, still=si, window_s=[round(a, 2), round(b, 2)], n_pairs=len(pm),
               pair_mean_pct_10_50_90=[round(float(v), 3) for v in np.percentile(pm, [10, 50, 90])],
               frac_pair_mean_lt05=round(float((pm < 0.5).mean()), 3),
               tile_median=np.round(med, 3).tolist(), tile_p90=np.round(p90, 3).tolist(), png=str(png.relative_to(ROOT)))
    print(json.dumps(dict((k_, v_) for k_, v_ in res.items() if k_ not in ("tile_p90",))), flush=True)
    return res


if __name__ == "__main__":
    with Pool(4) as p:
        res = p.map(run, JOBS)
    json.dump(dict(made=time.strftime("%F %T"), note="exploratory; no undistortion (가정)", jobs=res),
              open(ROOT / "runs/zupt_screen/tiles.json", "w"), indent=1)
