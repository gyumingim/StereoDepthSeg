"""정답(보드 자세) 만드는 방식 비교 — 같은 영상 조각에서 여러 방식으로 자세를 구해 '독립된 잣대' 두 개로 채점.

방식 (variants)
  base     : 지금 방식 (모서리 → 왜곡 펴기 → PnP), 시각 = 노출 가운데 + 보드 줄 보정
  rs       : 롤링셔터 펴기 — 모서리마다 그 줄이 찍힌 순간 → 사진 가운데 순간으로 자이로 회전만큼 되돌린 뒤 PnP, 시각 = 노출 가운데
  rs_neg   : rs 의 회전 방향을 뒤집은 것 (부호 확인용: rs 가 맞다면 rs_neg 는 나빠져야 함)
  sub3/sub7: 모서리를 cornerSubPix 로 한 번 더 다듬음 (창 3/7 px)
잣대 (재투영 오차는 자기 채점이라 안 씀)
  gyro_deg : 0.2 s 동안의 보드 자세 변화 vs 자이로 적분 회전 (align.json 의 dt·R_ci·바이어스) — 중앙값, 빠른 구간(>1 rad/s) 따로
  jit_mm / jit_deg : 사진만으로 그린 매끈한 곡선 (gt_smooth) 에서 점이 벗어난 정도 (RMS)
  ../venv_ffs/bin/python imu_learn/gt_experiment.py recordings/hs_A [...] --start 20 --secs 40
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import aruco_scale  # noqa: E402
import lens_calib  # noqa: E402
from imu_gt import read_imu  # noqa: E402
from gt_smooth import smooth_gt  # noqa: E402

VARIANTS = ["base", "rs", "rs_neg", "sub3", "sub7"]


def so3_exp(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def so3_log(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    return th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def lens_for(rec, meta):
    own = rec / "imu_gt" / "lens.json"
    fd = meta.get("focus_dpt_request") or -1
    shared = HERE.parent / "config" / "lens" / f"hs_{meta['width']}x{meta['height']}_f{fd:.2f}.json"
    return lens_calib.load(own if own.exists() else shared)[:2]


def run(rec, start_s, secs, board_json):
    rec = Path(rec)
    meta = json.load(open(rec / "meta.json"))
    K, dist = lens_for(rec, meta)
    H = int(meta["height"])
    board = aruco_scale.Board(aruco_scale.load_board_spec(board_json))
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    tmid = np.array([int(r["t_mid_ns"]) for r in rows])
    skew = np.array([int(r["rolling_shutter_skew_ns"]) for r in rows])
    al = json.load(open(rec / "imu_gt" / "align.json"))
    dt_ns, R_ci = al["dt_ns"], np.asarray(al["R_ci"])
    b = np.mean(al.get("gyro_bias_knots", [[0, 0, 0]]), 0)
    _, gyr = read_imu(rec / "imu.csv")
    tg = gyr[:, 0].astype(np.int64) + dt_ns                          # IMU time on the camera clock
    wg = gyr[:, 1:4] - b
    w_cam = lambda t: R_ci @ np.array([np.interp(t, tg, wg[:, j]) for j in range(3)])
    i0 = int(np.searchsorted(tmid, tmid[0] + start_s * 1e9))
    i1 = min(len(rows), int(np.searchsorted(tmid, tmid[0] + (start_s + secs) * 1e9)))
    cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
    out = {v: dict(t=[], R=[], p=[]) for v in VARIANTS}
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 40, 0.001)
    for i in range(i1):
        ok_r, f = cap.read()
        if not ok_r:
            break
        if i < i0:
            continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        c, ids, _ = board.detect(g)
        if len(ids) < 8:
            continue
        for v in VARIANTS:
            cc = c.copy()
            if v in ("sub3", "sub7"):
                w = 3 if v == "sub3" else 7
                cc = cv2.cornerSubPix(g, cc.astype(np.float32).reshape(-1, 1, 2), (w, w), (-1, -1), crit).reshape(-1, 2)
            if v.startswith("rs"):
                n = cv2.undistortPoints(cc.reshape(-1, 1, 2), K, dist).reshape(-1, 2)     # normalized rays at own row time
                sgn = 1.0 if v == "rs" else -1.0
                d = np.c_[n, np.ones(len(n))]
                tau = (cc[:, 1] / H - 0.5) * skew[i]                  # ns after the frame's middle row
                om = w_cam(tmid[i])
                for k in range(len(d)):                               # direction seen at t_mid = exp(-w * (t_mid - t_c)) d
                    d[k] = so3_exp(-sgn * om * (-tau[k] / 1e9)) @ d[k]
                und = (d[:, :2] / d[:, 2:3]) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
                t_ns = tmid[i]
            else:
                und = cv2.undistortPoints(cc.reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)
                t_ns = tmid[i] + int((cc[:, 1].mean() / H - 0.5) * skew[i])
            pose = aruco_scale.solve_pose(board, und, ids, K)
            if not pose["ok"] or pose["rms"] > 1.5:
                continue
            Rcb = pose["R"]
            out[v]["t"].append(t_ns)
            out[v]["R"].append(Rcb.T)                                 # camera → board
            out[v]["p"].append(-Rcb.T @ pose["t"])
    res = {}
    for v, o in out.items():
        t, R, p = np.array(o["t"]), np.array(o["R"]), np.array(o["p"])
        if len(t) < 100:
            continue
        # gyro consistency over 0.2 s (48 frames at 240 fps; 12 at 60)
        fps = float(meta.get("save_fps") or 60)
        n = int(round(0.2 * fps))
        errs, fast = [], []
        for k in range(len(t) - n):
            if t[k + n] - t[k] > 1.5 * 0.2e9:
                continue
            dRb = R[k].T @ R[k + n]                                    # camera rotation (camera frame) from board
            ts = np.linspace(t[k], t[k + n], 25)
            Q = np.eye(3)
            for a in range(24):
                Q = Q @ so3_exp(w_cam((ts[a] + ts[a + 1]) / 2) * (ts[a + 1] - ts[a]) / 1e9)
            e = np.degrees(np.linalg.norm(so3_log(dRb.T @ Q)))
            if np.degrees(np.linalg.norm(so3_log(dRb))) < 2:
                continue
            errs.append(e)
            if np.linalg.norm(w_cam(t[k])) > 1.0:
                fast.append(e)
        keep, Rs, ps = smooth_gt(t, R, p)
        jp = np.linalg.norm((p - ps)[keep], axis=1) * 1000
        jr = [np.degrees(np.linalg.norm(so3_log(R[k].T @ Rs[k]))) for k in np.where(keep)[0]]
        res[v] = dict(frames=int(len(t)), gyro_deg=round(float(np.median(errs)), 4) if errs else None,
                      gyro_fast_deg=round(float(np.median(fast)), 4) if fast else None, n_fast=len(fast),
                      jit_mm=round(float(np.sqrt(np.mean(jp ** 2))), 3), jit_deg=round(float(np.sqrt(np.mean(np.square(jr)))), 4))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    ap.add_argument("--start", type=float, default=20)
    ap.add_argument("--secs", type=float, default=40)
    ap.add_argument("--board", default=str(HERE.parent / "config" / "charuco" / "board_monitor_6x6.json"))
    a = ap.parse_args()
    allres = {}
    for rec in a.recs:
        r = run(rec, a.start, a.secs, a.board)
        allres[Path(rec).name] = r
        print(f"== {Path(rec).name} ({a.start:.0f}~{a.start + a.secs:.0f} s)")
        for v, m in r.items():
            print(f"  {v:7s} 사진 {m['frames']:5d}  자이로 0.2s {m['gyro_deg']}° (빠른 구간 {m['gyro_fast_deg']}°, {m['n_fast']})  "
                  f"곡선 흔들림 {m['jit_mm']} mm / {m['jit_deg']}°", flush=True)
        out = HERE.parent / "runs" / f"gt_experiment_{Path(rec).name}.json"
        json.dump(r, open(out, "w"), indent=1)


if __name__ == "__main__":
    main()
