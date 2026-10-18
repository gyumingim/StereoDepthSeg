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


def board_poses(video, frames_csv, K, dist, min_corners=8, max_rms=1.5):
    board = aruco_scale.Board(aruco_scale.load_board_spec(str(BOARD_JSON)))
    rows = list(csv.DictReader(open(frames_csv)))
    t_mid = {int(r["frame_idx"]): int(r["t_mid_ns"]) for r in rows}
    skew = {int(r["frame_idx"]): int(r.get("rolling_shutter_skew_ns") or 0) for r in rows}
    cap = cv2.VideoCapture(str(video))
    out = dict(t_ns=[], R_bc=[], p_bc=[], rms_px=[], n_corners=[], ok=[])
    Kn = K.copy()
    i = 0
    while True:
        r, f = cap.read()
        if not r:
            break
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = board.detect(g)
        R, p, rms, ok = np.eye(3), np.zeros(3), np.nan, False
        t_row = 0                                                    # 롤링셔터: 보드가 찍힌 줄의 시각 보정
        if len(ids) >= min_corners:
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
                t_row = int((corners[:, 1].mean() / g.shape[0] - 0.5) * skew.get(i, 0))
        out["t_ns"].append(t_mid.get(i, -1) + t_row if i in t_mid else -1)
        out["R_bc"].append(R)
        out["p_bc"].append(p)
        out["rms_px"].append(rms)
        out["n_corners"].append(len(ids))
        out["ok"].append(ok)
        i += 1
    return {k: np.asarray(v) for k, v in out.items()}


def read_imu(path):
    acc, gyr = [], []
    with open(path) as f:
        for r in csv.reader(f):
            if not r or r[0] == "type":
                continue
            t = int(r[0])
            if t == ACC:
                acc.append([int(r[1])] + [float(x) for x in r[2:5]])
            elif t == GYR:
                gyr.append([int(r[1])] + [float(x) for x in r[2:5]])
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
    a = ap.parse_args()
    rec = Path(a.rec)
    meta = json.load(open(rec / "meta.json"))
    size = (int(meta["width"]), int(meta["height"]))
    if size[0] * 3 == size[1] * 4:
        K, dist = lens_K(rec, a.cam, size)
    else:
        K, dist, _ = self_calibrate(rec / f"cam{a.cam}.mp4")
    out = rec / "imu_gt"
    out.mkdir(exist_ok=True)
    gt = board_poses(rec / f"cam{a.cam}.mp4", rec / f"frames_cam{a.cam}.csv", K, dist)
    np.savez(out / "gt.npz", **gt, K=K, dist=dist)
    ok = gt["ok"]
    print(f"정답: 프레임 {len(ok)}, 자세 OK {ok.sum()} ({ok.mean()*100:.0f} %), 재투영 중앙 {np.nanmedian(gt['rms_px'][ok]):.2f} px, "
          f"거리 중앙 {np.median(np.linalg.norm(gt['p_bc'][ok], axis=1)):.2f} m")
    acc, gyr = read_imu(rec / "imu.csv")
    print(f"IMU: 가속도 {len(acc)} ({len(acc)/((acc[-1,0]-acc[0,0])*1e-9):.0f} Hz), 자이로 {len(gyr)} ({len(gyr)/((gyr[-1,0]-gyr[0,0])*1e-9):.0f} Hz)")
    al, data = align(gt, gyr)
    json.dump(al, open(out / "align.json", "w"), indent=1)
    plot(out / "check.png", al, data)
    R = np.asarray(al["R_ci"])
    print(f"맞춤: 시간차 {al['dt_ns']/1e6:.1f} ms, 회전 잔차 {al['omega_rms']:.3f} rad/s (움직임 중앙 {al['omega_med_norm']:.2f} rad/s, "
          f"표본 {al['n']})\nR_ci (IMU→카메라):\n{np.round(R, 3)}\n→ {out}")


if __name__ == "__main__":
    main()
