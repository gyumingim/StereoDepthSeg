#!/usr/bin/env python3
"""IMU 학습용 정답 만들기 (IMU_PLAN.md 4): 녹화 하나 → 보정판(ChArUco) 기준 카메라 자세 + IMU 시간차·회전.

  venv_ffs/bin/python scan3d/imu_learn/imu_gt.py scan3d/recordings/rec_YYYYMMDD_HHMMSS [--cam 5]

쓰는 것 (<녹화>/imu_gt/):
  gt.npz      t_ns (노출 가운데 + 롤링셔터로 보드가 찍힌 줄만큼 보정), R_bc (카메라→보드 회전, 3x3), p_bc (보드 좌표의 카메라 위치, m), rms_px, n_corners,
              ok (자세를 믿을 프레임). 보드 좌표 = aruco_scale.Board (판 평면 z=0).
  align.json  dt_ns: IMU 시각 + dt = 카메라 시각 (자이로 |ω| 와 정답 |ω| 의 상호상관), R_ci: IMU(기기) 축 → 카메라 축 회전
              (ω_cam = R_ci ω_imu, 같은 순간 ω 벡터들의 Kabsch), 잔차, 사용 표본 수.
  check.png   정답 |ω| 와 맞춘 자이로 |ω|, 축별 비교.
보정: calib_scale 의 렌즈 5 내부값(K2, dist2)을 녹화 크기로 비례 변환. 모서리는 왜곡을 편 뒤 자세를 푼다.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(SCAN3D))
sys.path.insert(0, str(SCAN3D.parent))
import aruco_scale  # noqa: E402
import calib_scale  # noqa: E402

BOARD_JSON = SCAN3D / "config" / "charuco" / "board_user_A4_6x6.json"
ACC, GYR = 1, 4   # Android Sensor.TYPE_*
ACC_RAW, GYR_RAW = 35, 16   # TYPE_ACCELEROMETER_UNCALIBRATED, TYPE_GYROSCOPE_UNCALIBRATED (read_imu(raw=True))


def lens_K(rec, cam, size):
    """렌즈 cam 의 (K, dist) — 보정 파일은 2+5 쌍: 렌즈 2 = K1/dist1, 렌즈 5 = K2/dist2."""
    g = calib_scale.load_geometry(calib_scale.default_calib("2+5"), ["2", "5"], size)
    st = g["st"]
    k = "1" if cam == "2" else "2"
    return np.asarray(st["K" + k], float), np.asarray(st["dist" + k], float).ravel()


def self_calibrate(video, n=80, min_corners=12):
    """4:3 가 아닌 녹화(예: 120 fps 고속 모드 1280x720·1920x1080 — 센서를 잘라 씀)는 렌즈 보정값을 비례 변환할 수 없어
    녹화 속 보드로 직접 보정: 고르게 뽑은 n 프레임의 ChArUco 모서리 → cv2.calibrateCamera. → (K, dist, rms)."""
    board = aruco_scale.Board(aruco_scale.load_board_spec(str(BOARD_JSON)))
    cap = cv2.VideoCapture(str(video))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
    want = set(np.linspace(0, total - 1, n * 3).astype(int))
    obj, img, size = [], [], None
    i = 0
    while len(obj) < n:
        r, f = cap.read()
        if not r:
            break
        if i in want:
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            size = g.shape[::-1]
            c, ids, _ = board.detect(g)
            if len(ids) >= min_corners:
                obj.append(board.obj[ids].astype(np.float32))
                img.append(c.astype(np.float32).reshape(-1, 1, 2))
        i += 1
    if len(obj) < 15:
        raise SystemExit(f"자기 보정에 쓸 보드 프레임이 부족 ({len(obj)})")
    rms, K, dist, _, _ = cv2.calibrateCamera(obj, img, size, None, None, flags=cv2.CALIB_FIX_K3)
    print(f"자기 보정: 프레임 {len(obj)}, rms {rms:.2f} px, f {K[0,0]:.1f} {K[1,1]:.1f}, c {K[0,2]:.1f} {K[1,2]:.1f}")
    return K, dist.ravel(), rms


def _so3_exp(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * Kx + (1 - np.cos(th)) * Kx @ Kx


def _rot_rays(om, tau, d):
    """Rotate each ray d[k] by exp([om * tau[k]]×) (Rodrigues, vectorized; same as _so3_exp per corner)."""
    w = np.outer(tau, om)                                            # (N,3) rotation vectors
    th = np.linalg.norm(w, axis=1, keepdims=True)
    k = np.where(th > 1e-12, w / np.maximum(th, 1e-12), 0.0)
    c, s = np.cos(th), np.sin(th)
    return d * c + np.cross(k, d) * s + k * np.sum(k * d, axis=1, keepdims=True) * (1 - c)


def gyro_cam(rec, al):
    """align.json (dt, R_ci, gyro bias) → function t_cam_ns → angular velocity in the camera frame (rad/s)."""
    _, gyr = read_imu(Path(rec) / "imu.csv", raw=bool(al.get("imu_raw", False)))   # same gyro type as the alignment
    tg = gyr[:, 0].astype(np.int64) + int(al["dt_ns"])
    wg = gyr[:, 1:4] - np.mean(al.get("gyro_bias_knots", [[0, 0, 0]]), 0)
    return GyroCam(tg, wg, np.asarray(al["R_ci"]))


class GyroCam:
    """t_cam_ns → ω in the camera frame (same numbers as the former lambda; a class so --chunks workers can pickle it)."""

    def __init__(self, tg, wg, R_ci):
        self.tg, self.wg, self.R_ci = tg, wg, R_ci

    def __call__(self, t):
        return self.R_ci @ np.array([np.interp(t, self.tg, self.wg[:, j]) for j in range(3)])


_CHUNK = {}   # board_poses(chunks=N): per-worker context (set by _chunk_init in each spawned worker)


def _chunk_init(ctx):
    cv2.setNumThreads(1)
    _CHUNK.clear()
    _CHUNK.update(ctx)
    _CHUNK["board"] = aruco_scale.Board(aruco_scale.load_board_spec(str(ctx["board_json"])))


def _chunk_worker(rng):
    """Frames [s, e) (e None = to the end of the video). Single-threaded: OpenCV 1 thread, FFmpeg decoder 1 thread.
    Frames before s are grab()bed (decoded, not converted) from frame 0 = exact frame indices (no seeking)."""
    s, e = rng
    c = _CHUNK
    cap = cv2.VideoCapture(str(c["video"]), cv2.CAP_FFMPEG, [cv2.CAP_PROP_N_THREADS, 1])
    for _ in range(s):
        if not cap.grab():
            return s, [], True
    res, i = [], s
    while e is None or i < e:
        r, f = cap.read()
        if not r:
            return s, res, True
        res.append(_frame_pose(i, f, c))
        i += 1
    return s, res, False


def _frame_pose(i, f, c):
    """One frame of board_poses: → (t_ns, R_bc, p_bc, rms_px, n_corners, ok, corners, ids)."""
    K, dist, Kn, w_cam, t_mid, skew = c["K"], c["dist"], c["Kn"], c["w_cam"], c["t_mid"], c["skew"]
    board, min_corners, max_rms = c["board"], c["min_corners"], c["max_rms"]
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = board.detect(g)
    R, p, rms, ok = np.eye(3), np.zeros(3), np.nan, False
    t_row = 0
    if len(ids) >= min_corners:
        if w_cam is not None and i in t_mid:
            n = cv2.undistortPoints(corners.reshape(-1, 1, 2), K, dist).reshape(-1, 2)
            d = np.c_[n, np.ones(len(n))]
            tau = (corners[:, 1] / g.shape[0] - 0.5) * skew.get(i, 0) / 1e9
            om = w_cam(t_mid[i])
            d = _rot_rays(om, tau, d)
            und = (d[:, :2] / d[:, 2:3]) * [Kn[0, 0], Kn[1, 1]] + [Kn[0, 2], Kn[1, 2]]
        else:
            und = cv2.undistortPoints(corners.reshape(-1, 1, 2), K, dist, P=Kn).reshape(-1, 2)
        pose = aruco_scale.solve_pose(board, und, ids, Kn)
        if pose["ok"]:
            R_cb, t_cb = pose["R"], pose["t"]
            R = R_cb.T
            p = -R_cb.T @ t_cb
            rms = pose["rms"]
            ok = rms <= max_rms
            if w_cam is None:
                t_row = int((corners[:, 1].mean() / g.shape[0] - 0.5) * skew.get(i, 0))
    t = t_mid.get(i, -1) + t_row if i in t_mid else -1
    return t, R, p, rms, len(ids), ok, corners.astype(np.float32), ids.astype(np.int16)


def _board_poses_chunked(video, rows, t_mid, skew, K, dist, min_corners, max_rms, w_cam, keep_corners, chunks, max_frames=0):
    """board_poses split into `chunks` contiguous frame ranges in spawned worker processes (2026-10-23, --chunks;
    same per-frame code as the serial loop up to the same stop rule: the first frame the decoder cannot read).
    spawn, not fork: a fork after OpenCV's thread pool has run deadlocks the children (seen 2026-10-23)."""
    import multiprocessing as mp
    ctx = dict(video=str(video), K=K, dist=dist, Kn=K.copy(), w_cam=w_cam, t_mid=t_mid, skew=skew,
               min_corners=min_corners, max_rms=max_rms, board_json=str(BOARD_JSON))
    n = len(rows) if max_frames <= 0 else min(len(rows), max_frames)
    b = [int(round(n * k / chunks)) for k in range(chunks + 1)]
    rngs = [(b[k], b[k + 1] if (k < chunks - 1 or max_frames > 0) else None) for k in range(chunks)]
    with mp.get_context("spawn").Pool(chunks, initializer=_chunk_init, initargs=(ctx,)) as pool:
        parts = sorted(pool.map(_chunk_worker, rngs, chunksize=1))
    frames = []
    for s, res, eof in parts:
        if len(frames) != s:          # an earlier chunk hit the end of the video: the serial loop would have stopped
            break
        frames.extend(res)
        if eof:
            break
    out = {k: np.asarray([fr[j] for fr in frames]) for j, k in enumerate(("t_ns", "R_bc", "p_bc", "rms_px", "n_corners", "ok"))}
    if keep_corners:
        out["corners"] = np.array([fr[6] for fr in frames], dtype=object)
        out["corner_ids"] = np.array([fr[7] for fr in frames], dtype=object)
    return out


def board_poses(video, frames_csv, K, dist, min_corners=8, max_rms=1.5, w_cam=None, keep_corners=False, chunks=0,
                max_frames=0):
    """w_cam (gyro_cam): rolling-shutter derotation — every corner is rotated from the moment its image row was read
    to the frame's middle row with the gyro, then one PnP at t_mid (gt_experiment.py, 4 recordings 2026-10-19: gyro
    0.2 s disagreement −20…−45 %, fast segments about halved, +9 % frames under the 1.5 px gate; flipped sign worse).
    chunks > 1 (--chunks, 2026-10-23; default 0 = this serial loop, unchanged): parallel frame ranges, same output.
    max_frames > 0: only the first max_frames frames (imu_gt_chunk_check.py; default 0 = all)."""
    board = aruco_scale.Board(aruco_scale.load_board_spec(str(BOARD_JSON)))
    rows = list(csv.DictReader(open(frames_csv)))
    t_mid = {int(r["frame_idx"]): int(r["t_mid_ns"]) for r in rows}
    skew = {int(r["frame_idx"]): int(r.get("rolling_shutter_skew_ns") or 0) for r in rows}
    if chunks and chunks > 1:
        return _board_poses_chunked(video, rows, t_mid, skew, K, dist, min_corners, max_rms, w_cam, keep_corners, chunks,
                                    max_frames)
    cap = cv2.VideoCapture(str(video))
    out = dict(t_ns=[], R_bc=[], p_bc=[], rms_px=[], n_corners=[], ok=[])
    cc_all, id_all = [], []
    Kn = K.copy()
    i = 0
    while not (max_frames > 0 and i >= max_frames):
        r, f = cap.read()
        if not r:
            break
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = board.detect(g)
        if keep_corners:                                             # for a later continuous-time (spline) fit
            cc_all.append(corners.astype(np.float32))
            id_all.append(ids.astype(np.int16))
        R, p, rms, ok = np.eye(3), np.zeros(3), np.nan, False
        t_row = 0                                                    # 롤링셔터: 보드가 찍힌 줄의 시각 보정
        if len(ids) >= min_corners:
            if w_cam is not None and i in t_mid:
                n = cv2.undistortPoints(corners.reshape(-1, 1, 2), K, dist).reshape(-1, 2)
                d = np.c_[n, np.ones(len(n))]
                tau = (corners[:, 1] / g.shape[0] - 0.5) * skew.get(i, 0) / 1e9   # s after the middle row
                om = w_cam(t_mid[i])
                d = _rot_rays(om, tau, d)                             # ray seen at t_mid = exp(-w (t_mid - t_c)) d
                und = (d[:, :2] / d[:, 2:3]) * [Kn[0, 0], Kn[1, 1]] + [Kn[0, 2], Kn[1, 2]]
            else:
                und = cv2.undistortPoints(corners.reshape(-1, 1, 2), K, dist, P=Kn).reshape(-1, 2)
            pose = aruco_scale.solve_pose(board, und, ids, Kn)
            if pose["ok"]:
                R_cb, t_cb = pose["R"], pose["t"]            # X_cam = R_cb X_board + t_cb
                R = R_cb.T                                   # 카메라 → 보드
                p = -R_cb.T @ t_cb                           # 보드 좌표의 카메라 중심
                rms = pose["rms"]
                ok = rms <= max_rms
                # 프레임 시각 = 노출 가운데 + 가운데 줄. 보드 줄(모서리 평균 y)은 (y/H - 0.5)·skew 만큼 다름
                # (S24 광각 skew 8.7 ms: 2 rad/s 회전이면 위·아래 끝에서 약 0.5° 어긋남)
                if w_cam is None:                                    # derotated corners already belong to t_mid
                    t_row = int((corners[:, 1].mean() / g.shape[0] - 0.5) * skew.get(i, 0))
        out["t_ns"].append(t_mid.get(i, -1) + t_row if i in t_mid else -1)
        out["R_bc"].append(R)
        out["p_bc"].append(p)
        out["rms_px"].append(rms)
        out["n_corners"].append(len(ids))
        out["ok"].append(ok)
        i += 1
    res = {k: np.asarray(v) for k, v in out.items()}
    if keep_corners:
        res["corners"] = np.array(cc_all, dtype=object)
        res["corner_ids"] = np.array(id_all, dtype=object)
    return res


def board_info(board_json):
    """Provenance of the board used for a GT: file, square size (mm) and where the square size came from.
    Positions scale linearly with square_mm (rotations, dt, R_ci do not) → rescale_gt.py fixes a GT for a new square."""
    spec = aruco_scale.load_board_spec(str(board_json))
    raw = json.loads(Path(board_json).read_text())
    return dict(board_json=str(Path(board_json).resolve()), board_name=Path(board_json).name,
                square_mm=round(spec["square_m"] * 1000, 6),
                square_mm_source=raw.get("square_mm_source") or raw.get("square_source") or spec.get("square_src"),
                square_estimated=bool(raw.get("square_estimated", False)))


def provenance(files=()):
    """git HEAD (+ dirty flag) and sha256 of the code files that made an output (CLAUDE.md 12 rule 4)."""
    import hashlib
    import subprocess
    import time
    out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"))
    try:
        out["git_head"] = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=SCAN3D, capture_output=True,
                                         text=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain", "--", "imu_learn", "aruco_scale.py", "config/charuco"],
                               cwd=SCAN3D, capture_output=True, text=True).stdout
        out["git_dirty"] = [ln[3:] for ln in dirty.splitlines() if ln.endswith(".py") or ln.endswith(".json")]
    except OSError:
        pass
    out["code_sha256"] = {Path(f).name: hashlib.sha256(Path(f).read_bytes()).hexdigest()[:16]
                          for f in [__file__, SCAN3D / "aruco_scale.py", *files] if Path(f).exists()}
    return out


def write_board_info(out_dir, board_json, extra=None, files=()):
    d = dict(board_info(board_json), **provenance(files), **(extra or {}))
    json.dump(d, open(Path(out_dir) / "board.json", "w"), indent=1, ensure_ascii=False)
    return d


def read_imu(path, raw=False):
    """(acc, gyr) rows [t_ns, x, y, z]. raw=False (default): Android-calibrated types 1/4. raw=True: uncalibrated
    types 35/16, values 0..2 only (3..5 = Android's bias estimate); recordings before 2026-10-20 have none -> ValueError."""
    a_type, g_type = (ACC_RAW, GYR_RAW) if raw else (ACC, GYR)
    acc, gyr = [], []
    bad = 0
    with open(path) as f:
        for r in csv.reader(f):
            if not r or r[0] == "type":
                continue
            if len(r) < 5:
                bad += 1
                continue
            try:                      # broken rows (app writes cut at a 4 KB boundary, e.g. hs_20261020_224323 row 318 '.0,0.0')
                t = int(r[0])
                if t == a_type:
                    acc.append([int(r[1])] + [float(x) for x in r[2:5]])
                elif t == g_type:
                    gyr.append([int(r[1])] + [float(x) for x in r[2:5]])
            except (ValueError, IndexError):
                bad += 1
    if bad:
        print(f"read_imu: {path}: skipped {bad} malformed row(s)", file=sys.stderr)
    if raw and (not acc or not gyr):
        raise ValueError(f"{path}: no uncalibrated IMU rows (type {ACC_RAW}: {len(acc)}, type {GYR_RAW}: {len(gyr)}) "
                         "- recorded before raw logging was added; run without the raw option")
    return np.asarray(acc), np.asarray(gyr)


def so3_log(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(th))
    return w * th


def cam_omega(gt):
    """연속한 믿을 만한 프레임 쌍의 카메라 각속도 (카메라 축, rad/s) 와 그 가운데 시각."""
    t, R, ok = gt["t_ns"], gt["R_bc"], gt["ok"]
    ts, ws = [], []
    for k in range(len(t) - 1):
        if ok[k] and ok[k + 1] and t[k + 1] > t[k]:
            dt = (t[k + 1] - t[k]) * 1e-9
            if dt > 0.1:
                continue
            ws.append(so3_log(R[k].T @ R[k + 1]) / dt)       # 카메라 축의 회전
            ts.append((t[k] + t[k + 1]) // 2)
    return np.asarray(ts), np.asarray(ws)


def kabsch(a, b):
    """b ≈ R a (행 = 표본) 인 회전 R."""
    H = a.T @ b
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T


def align(gt, gyr):
    tc, wc = cam_omega(gt)
    if len(tc) < 50:
        raise SystemExit(f"믿을 만한 연속 프레임이 너무 적음 ({len(tc)})")
    tg, wg = gyr[:, 0].astype(np.int64), gyr[:, 1:4]
    # |ω| 상호상관으로 시간차 (회전과 무관)
    grid = np.arange(tc[0], tc[-1], 5_000_000)               # 200 Hz
    nc = np.interp(grid, tc, np.linalg.norm(wc, axis=1))
    best = None
    for dt in range(-150_000_000, 150_001_000, 1_000_000):   # ±150 ms, 1 ms 간격
        ng = np.interp(grid, tg + dt, np.linalg.norm(wg, axis=1))
        e = np.mean((nc - ng) ** 2)
        if best is None or e < best[1]:
            best = (dt, e)
    dt = best[0]
    # 같은 순간의 ω 벡터로 회전 (움직이는 구간만)
    wg_at = np.stack([np.interp(tc, tg + dt, wg[:, j]) for j in range(3)], 1)
    m = np.linalg.norm(wc, axis=1) > 0.3
    R_ci = kabsch(wg_at[m], wc[m])
    res = wc[m] - (R_ci @ wg_at[m].T).T
    return dict(dt_ns=int(dt), R_ci=R_ci.tolist(), omega_rms=float(np.sqrt(np.mean(np.sum(res ** 2, 1)))),
                omega_med_norm=float(np.median(np.linalg.norm(wc[m], axis=1))), n=int(m.sum()),
                corr_mse=float(best[1])), (tc, wc, tg, wg)


def plot(out_png, al, data):
    tc, wc, tg, wg = data
    R = np.asarray(al["R_ci"])
    wg_c = (R @ wg.T).T
    t0 = tc[0]
    H, W = 600, 1200
    img = np.full((H, W, 3), 255, np.uint8)
    rows = [("|w|", np.linalg.norm(wc, axis=1), np.linalg.norm(wg_c, axis=1))] + \
           [(f"w{a}", wc[:, i], wg_c[:, i]) for i, a in enumerate("xyz")]
    span = (tc[-1] - t0) * 1e-9
    for r, (name, a, b) in enumerate(rows):
        y0 = 20 + r * (H - 20) // 4
        hh = (H - 20) // 4 - 10
        lim = max(1e-3, np.percentile(np.abs(np.r_[a, b]), 99))
        def pts(ts, v):
            x = ((ts - t0) * 1e-9 / span * (W - 20) + 10).astype(int)
            y = (y0 + hh / 2 - v / lim * hh / 2).astype(int)
            return np.stack([x, y], 1)
        sel = (tg + al["dt_ns"] >= tc[0]) & (tg + al["dt_ns"] <= tc[-1])
        cv2.polylines(img, [pts(tg[sel] + al["dt_ns"], b[sel])], False, (200, 120, 0), 1)
        cv2.polylines(img, [pts(tc, a)], False, (0, 0, 220), 1)
        cv2.putText(img, name, (12, y0 + 14), 0, 0.5, (0, 0, 0), 1)
    cv2.putText(img, f"red = board GT, blue = gyro (dt {al['dt_ns']/1e6:.1f} ms, rms {al['omega_rms']:.3f} rad/s)",
                (300, 14), 0, 0.5, (0, 0, 0), 1)
    cv2.imwrite(str(out_png), img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--cam", default="5")
    ap.add_argument("--board", default=None,
                    help="보드 json (기본 board_user_A4_6x6.json). 모니터에 띄운 보드: config/charuco/board_monitor_6x6.json")
    ap.add_argument("--lens", default=None,
                    help="렌즈 값 json (lens_calib.py). 없으면 imu_gt/lens.json → 공용 기본값 (lens_calib.shared_lens_path; 2.22 dpt = "
                         "hs_1920x1080_f2.22_cal20261022.json, 2026-10-23~) → 새로 구함")
    ap.add_argument("--rs", action="store_true",
                    help="롤링셔터 펴기 (자이로; imu_gt/align.json 의 dt·R_ci·바이어스 필요 → batch_sync 뒤에)")
    ap.add_argument("--out-dir", default="imu_gt",
                    help="결과 폴더 이름 (imu_gt 가 아니면 align.json 은 imu_gt 것을 쓰고 새로 만들지 않음)")
    ap.add_argument("--keep-corners", action="store_true", help="프레임별 모서리를 gt.npz 에 저장 (곡선 맞춤용)")
    ap.add_argument("--chunks", type=int, default=0,
                    help="보드 검출을 프레임 구간 N 개로 나눠 병렬 (프로세스마다 1 스레드; 0 = 예전 그대로 한 줄로). 결과 같음")
    ap.add_argument("--raw-imu", action="store_true",
                    help="원본 IMU (type 35/16) 로 맞춤 (align.json imu_raw=true; batch_sync·--rs 가 같은 종류를 씀). 기본 = 보정판 1/4")
    a = ap.parse_args()
    if a.board:
        global BOARD_JSON
        BOARD_JSON = Path(a.board)
    print(f"보드: {BOARD_JSON.name}")
    rec = Path(a.rec)
    meta = json.load(open(rec / "meta.json"))
    size = (int(meta["width"]), int(meta["height"]))
    out = rec / a.out_dir
    out.mkdir(exist_ok=True)
    w_cam = None
    if a.rs:
        al0 = json.load(open(rec / "imu_gt" / "align.json"))
        w_cam = gyro_cam(rec, al0)
        print(f"롤링셔터 펴기: align {al0.get('method', 'omega')}, dt {al0['dt_ns'] / 1e6:+.2f} ms")
    if size[0] * 3 == size[1] * 4:
        K, dist = lens_K(rec, a.cam, size)
    else:
        # 고속 녹화 (16:9, 센서를 잘라 씀): 공장 렌즈 값을 못 쓰므로 보드로 구한 값. 초점 고정 녹화는 같은 값을 같이 씀.
        import lens_calib
        fd = meta.get("focus_dpt_request") or -1
        shared = lens_calib.shared_lens_path(size[0], size[1], fd)   # 2026-10-23: 2.22 dpt → cal20261022 (전엔 fx 1267 파일)
        cand = [Path(a.lens)] if a.lens else [out / "lens.json", rec / "imu_gt" / "lens.json", shared]
        lp = next((p for p in cand if p.exists()), None)
        if lp is None:
            print("렌즈 값 새로 구함 (느린 프레임, 화면 전체에 퍼지게)")
            K, dist, info = lens_calib.calibrate([rec], BOARD_JSON)
            lp = shared if fd > 0 else out / "lens.json"
            lens_calib.save(lp, K, dist, info, focus_dpt=fd if fd > 0 else meta.get("focus_locked_dpt"))
        K, dist, info = lens_calib.load(lp)
        print(f"렌즈 값: {lp} ({info.get('model')}, rms {info.get('rms_px')} px, 사진 {info.get('n_views')} 장, "
              f"f {K[0, 0]:.1f} {K[1, 1]:.1f})")
    gt = board_poses(rec / f"cam{a.cam}.mp4", rec / f"frames_cam{a.cam}.csv", K, dist, w_cam=w_cam,
                     keep_corners=a.keep_corners, chunks=a.chunks)
    bi = board_info(BOARD_JSON)
    np.savez(out / "gt.npz", **gt, K=K, dist=dist, rs=bool(a.rs), board_json=bi["board_json"], square_mm=bi["square_mm"])
    ok = gt["ok"]
    write_board_info(out, BOARD_JSON, dict(
        rec=rec.name, rs=bool(a.rs), keep_corners=bool(a.keep_corners),
        align_used=(dict(dt_ns=al0["dt_ns"], imu_raw=bool(al0.get("imu_raw", False)), method=al0.get("method"))
                    if a.rs else None),
        frames=int(len(ok)), ok=int(ok.sum()),
        rms_px_median=float(np.nanmedian(gt["rms_px"][ok])) if ok.any() else None,
        dist_m_median=float(np.median(np.linalg.norm(gt["p_bc"][ok], axis=1))) if ok.any() else None,
        corners_median=float(np.median(gt["n_corners"])), corners_median_ok=float(np.median(gt["n_corners"][ok])) if ok.any() else None))
    print(f"정답: 프레임 {len(ok)}, 자세 OK {ok.sum()} ({ok.mean()*100:.0f} %), 재투영 중앙 {np.nanmedian(gt['rms_px'][ok]):.2f} px, "
          f"거리 중앙 {np.median(np.linalg.norm(gt['p_bc'][ok], axis=1)):.2f} m")
    if a.out_dir != "imu_gt":                       # variant GT: keep the (batch_sync) alignment in imu_gt untouched
        return
    acc, gyr = read_imu(rec / "imu.csv", raw=a.raw_imu)
    print(f"IMU ({'원본 35/16' if a.raw_imu else '보정판 1/4'}): 가속도 {len(acc)} ({len(acc)/((acc[-1,0]-acc[0,0])*1e-9):.0f} Hz), 자이로 {len(gyr)} ({len(gyr)/((gyr[-1,0]-gyr[0,0])*1e-9):.0f} Hz)")
    al, data = align(gt, gyr)
    al["imu_raw"] = bool(a.raw_imu)
    json.dump(al, open(out / "align.json", "w"), indent=1)
    plot(out / "check.png", al, data)
    R = np.asarray(al["R_ci"])
    print(f"맞춤: 시간차 {al['dt_ns']/1e6:.1f} ms, 회전 잔차 {al['omega_rms']:.3f} rad/s (움직임 중앙 {al['omega_med_norm']:.2f} rad/s, "
          f"표본 {al['n']})\nR_ci (IMU→카메라):\n{np.round(R, 3)}\n→ {out}")


if __name__ == "__main__":
    main()
