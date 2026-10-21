#!/usr/bin/env python3
"""zupt_screen step 4 (analysis only, exploratory): image-side cause of still-period ZUPT disparity noise.
For 213341 / 213955 / 214550 / 215348, each IMU still (i3_confirm.still_runs) >= 5 s, window [a+1, min(b-0.5, a+11)] s:
  - frame metadata (results_cam5.csv): exposure, ISO, frame duration, AE state, lens moving, focus
  - 240 fps timing gaps, and 60 Hz feed gaps (nearest frame to a 60 Hz grid, as 'fed by time')
  - brightness: global / board region / outside board, relative std and FFT peak on the 240 fps series; tile flicker map
  - KLT jitter on fed frames (gray -> 640x360 -> equalizeHist, goodFeatures 200, minDist 10, pyrLK): per-pair mean
    displacement (stand-in for OpenVINS ZUPT disparity), split board-region vs outside, and vs brightness change.
Board region = convex hull of detected ArUco (DICT_6X6_250, ids 100-121) corners on the window's first frame, grown 30 %.
No undistortion (the bench remaps first) -> displacement scale approximate (가정)."""
import sys, os, csv, json, time
from pathlib import Path
from multiprocessing import Pool
import numpy as np
import cv2
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "imu_learn")); os.chdir(ROOT)
RECS = ["hs_20261021_213341", "hs_20261021_213955", "hs_20261021_214550", "hs_20261021_215348"]
W, H = 640, 360


def meta(rec):
    R = ROOT / "recordings" / rec
    fr = list(csv.DictReader(open(R / "frames_cam5.csv")))
    md = {}
    for r in csv.DictReader(open(R / "results_cam5.csv")):
        md[int(r["sensor_ts_ns"])] = r
    return fr, md


def board_hull(img_full):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(img_full)
    if ids is None or len(ids) < 3:
        return None, 0
    pts = np.concatenate([c.reshape(-1, 2) for c, i in zip(corners, ids.ravel()) if 100 <= i <= 121]) * (W / img_full.shape[1])
    hull = cv2.convexHull(pts.astype(np.float32)).reshape(-1, 2)
    c = hull.mean(0)
    hull = c + 1.3 * (hull - c)
    mask = np.zeros((H, W), np.uint8)
    cv2.fillConvexPoly(mask, np.round(hull).astype(np.int32), 1)
    return mask.astype(bool), int(len(ids))


def peak(x, fs):
    x = np.asarray(x, float); x = x - x.mean()
    if len(x) < 64:
        return None, None
    F = np.abs(np.fft.rfft(x * np.hanning(len(x)))); f = np.fft.rfftfreq(len(x), 1 / fs)
    m = f > 2
    k = np.argmax(F[m])
    return round(float(f[m][k]), 2), round(float(F[m][k] / (F[m].mean() + 1e-12)), 1)


def run(rec):
    import i3_confirm as ic
    x = ic.inputs(rec)
    fr, md = meta(rec)
    tm = np.array([int(r["t_mid_ns"]) for r in fr]); ph = np.array([int(r["physical_ts_ns"]) for r in fr])
    trel = (tm - tm[0]) / 1e9
    wins = []
    for a, b in x["stills_s"]:
        if b - a >= 5:
            wins.append((a + 1.0, min(b - 0.5, a + 11.0)))
    # frames needed
    need = {}
    for wi, (a, b) in enumerate(wins):
        for k in np.where((trel >= a) & (trel <= b))[0]:
            need[int(k)] = wi
    cap = cv2.VideoCapture(str(ROOT / "recordings" / rec / "cam5.mp4"))
    n_vid = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames = {}
    full_first = {}
    kmax = max(need)
    k = 0
    while k <= kmax:
        ok = cap.grab()
        if not ok:
            break
        if k in need:
            _, f = cap.retrieve()
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            if need[k] not in full_first:
                full_first[need[k]] = g.copy()
            frames[k] = cv2.resize(g, (W, H), interpolation=cv2.INTER_AREA)
        k += 1
    out = dict(rec=rec, n_video=n_vid, n_csv=len(fr), windows=[])
    for wi, (a, b) in enumerate(wins):
        ks = sorted(kk for kk, w in need.items() if w == wi and kk in frames)
        if len(ks) < 100:
            continue
        mask, nmk = board_hull(full_first[wi])
        # metadata
        mrows = [md.get(int(ph[kk])) for kk in ks]
        mrows = [m for m in mrows if m]
        def uniq(col):
            v = [m[col] for m in mrows]
            u, c = np.unique(v, return_counts=True)
            return {str(a_): int(c_) for a_, c_ in zip(u, c)}
        dts = np.diff(tm[ks]) / 1e6
        gaps240 = int((dts > 1.5 * 4.1667).sum())
        # 60 Hz feed by time
        grid = np.arange(trel[ks[0]], trel[ks[-1]], 1 / 60)
        kk_arr = np.array(ks)
        j = np.clip(np.searchsorted(trel[kk_arr], grid), 1, len(kk_arr) - 1)
        j = np.where(np.abs(trel[kk_arr[j - 1]] - grid) < np.abs(trel[kk_arr[j]] - grid), j - 1, j)
        fed = list(dict.fromkeys(kk_arr[j].tolist()))
        fdt = np.diff(trel[fed]) * 1000
        # brightness at 240 fps
        G = np.array([frames[kk].mean() for kk in ks])
        Bm = np.array([frames[kk][mask].mean() for kk in ks]) if mask is not None else None
        Om = np.array([frames[kk][~mask].mean() for kk in ks]) if mask is not None else None
        fs = 1 / np.median(np.diff(trel[ks]))
        def rel(s):
            # frame-to-frame brightness change std, % of mean
            return None if s is None else round(float(np.std(np.diff(s)) / s.mean() * 100), 3)
        # tile flicker map (fed frames): std of frame-to-frame tile-mean change, % of tile mean
        T = np.array([cv2.resize(frames[kk].astype(np.float32), (16, 9), interpolation=cv2.INTER_AREA) for kk in fed])
        tf = (np.std(np.diff(T, axis=0), axis=0) / (T.mean(0) + 1e-6) * 100)
        tmask = cv2.resize(mask.astype(np.float32), (16, 9), interpolation=cv2.INTER_AREA) > 0.5 if mask is not None else None
        # KLT on fed frames
        prev = cv2.equalizeHist(frames[fed[0]])
        p0 = cv2.goodFeaturesToTrack(prev, 200, 0.01, 10)
        pair_disp, pair_in, pair_out, pair_dB, per_feat = [], [], [], [], []
        for a_, b_ in zip(fed[:-1], fed[1:]):
            cur = cv2.equalizeHist(frames[b_])
            if p0 is None or len(p0) < 50:
                p0 = cv2.goodFeaturesToTrack(prev, 200, 0.01, 10)
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, winSize=(15, 15), maxLevel=3)
            st = st.ravel() == 1
            d = np.linalg.norm((p1 - p0).reshape(-1, 2), axis=1)[st]
            pts = p0.reshape(-1, 2)[st]
            if len(d) > 20:
                pair_disp.append(float(d.mean()))
                if mask is not None:
                    xi = np.clip(pts[:, 0].astype(int), 0, W - 1); yi = np.clip(pts[:, 1].astype(int), 0, H - 1)
                    inb = mask[yi, xi]
                    pair_in.append(float(d[inb].mean()) if inb.any() else np.nan)
                    pair_out.append(float(d[~inb].mean()) if (~inb).any() else np.nan)
                    per_feat.append((int(inb.sum()), int((~inb).sum())))
                pair_dB.append(float(abs(frames[b_].mean() - frames[a_].mean())))
            p0 = p1[st].reshape(-1, 1, 2)
            prev = cur
        pd_ = np.array(pair_disp)
        win = dict(window_s=[round(a, 2), round(b, 2)], n_frames240=len(ks), n_fed=len(fed), aruco_markers=nmk,
                   board_area_frac=None if mask is None else round(float(mask.mean()), 3),
                   exposure_ns=uniq("exposure_ns"), iso=uniq("iso"), frame_duration_ns=uniq("frame_duration_ns"),
                   ae_mode=uniq("ae_mode"), ae_state=uniq("ae_state"), lens_moving=uniq("lens_moving"), focus=uniq("focus_dpt"),
                   gaps240=gaps240, gaps240_frac=round(gaps240 / max(1, len(dts)), 4), dt240_max_ms=round(float(dts.max()), 2),
                   fed_dt_ms_pct=[round(float(v), 2) for v in np.percentile(fdt, [5, 50, 95, 100])],
                   fed_gap_gt25ms=int((fdt > 25).sum()),
                   bright_mean=round(float(G.mean()), 1), flick_rel_global_pct=rel(G), flick_rel_board_pct=rel(Bm), flick_rel_out_pct=rel(Om),
                   fft_global=peak(G, fs), fft_board=peak(Bm, fs) if Bm is not None else None, fs=round(float(fs), 1),
                   tile_flicker_pct_board_med=None if tmask is None else round(float(np.median(tf[tmask])), 3),
                   tile_flicker_pct_out_med=None if tmask is None else round(float(np.median(tf[~tmask])), 3),
                   tile_flicker_pct_max=round(float(tf.max()), 3),
                   klt_disp_pct=[round(float(v), 3) for v in np.percentile(pd_, [10, 50, 90])] if len(pd_) else None,
                   klt_in_board_med=round(float(np.nanmedian(pair_in)), 3) if pair_in else None,
                   klt_out_board_med=round(float(np.nanmedian(pair_out)), 3) if pair_out else None,
                   feat_in_out_med=[int(np.median([p[0] for p in per_feat])), int(np.median([p[1] for p in per_feat]))] if per_feat else None,
                   corr_disp_dbright=round(float(np.corrcoef(pd_, pair_dB)[0, 1]), 3) if len(pd_) > 10 else None)
        out["windows"].append(win)
        print(rec, json.dumps(win), flush=True)
    return out


if __name__ == "__main__":
    with Pool(4) as p:
        res = p.map(run, RECS)
    json.dump(dict(made=time.strftime("%F %T"), note="exploratory; see docstring", recs=res),
              open(ROOT / "runs/zupt_screen/img_cause.json", "w"), indent=1)
