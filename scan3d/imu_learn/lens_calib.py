"""고속 녹화 (1920x1080, 240 fps) 의 렌즈 값 (K, 왜곡) 을 녹화 속 보드로 제대로 구함.

imu_gt.py 의 예전 자기 보정은 영상 앞쪽에서 아무 프레임 80 장 (흐린 것 포함) 이라 녹화마다 f 가 3 % 달랐음.
여기서는:
  1. 자이로로 느린 프레임 (|ω| < 0.3 rad/s) 만 후보 → 흐림·롤링셔터가 작은 사진
  2. 모서리 20 개 이상, 화면 3x3 칸·기울기마다 고르게 퍼지게 최대 n 장 (기본 120) (같은 칸만 많으면 렌즈 가장자리를 모름)
  3. 왜곡 모델 3 가지 (k1 k2 p1 p2 / + k3 / 유리 모델) 를 반으로 나눈 사진으로 맞추고 나머지 반에서 오차 → 가장 좋은 모델
  4. 사진별 오차가 중앙의 2.5 배 넘는 사진은 빼고 다시 맞춤
결과: config/lens/hs_<w>x<h>_f<초점 dpt>.json (초점 고정 녹화끼리 같이 씀) 또는 녹화 imu_gt/lens.json.
  python imu_learn/lens_calib.py recordings/hs_A [recordings/hs_B ...] [--save config/lens/hs_1920x1080_f2.22_cal<날짜>.json]
기본으로 쓸 파일은 DEFAULT_LENS 표에 적는다 (2026-10-23 부터; 같은 이름에 덮어쓰지 않음).
"""
import argparse
import csv
import json
import pickle
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import aruco_scale  # noqa: E402
from imu_gt import read_imu  # noqa: E402

TERM = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 60, 1e-7)   # 250 장 유리 모델이 30 분 넘게 안 끝난 일 (2026-10-19)
MODELS = {"k1k2p": cv2.CALIB_FIX_K3, "k1k2p_k3": 0, "rational": cv2.CALIB_RATIONAL_MODEL}
LENS_DIR = HERE.parent / "config" / "lens"
# 공용 렌즈 기본값: 표에 있으면 그 파일, 없으면 hs_<W>x<H>_f<초점>.json (2026-10-23 사용자 결정, docs/lens_switch_20261023.md).
# 2.22 dpt: 새 2.22 노드 (fx 1303.9, 반복 ±0.2 %). 옛 hs_1920x1080_f2.22.json (fx 1267.1, 10-19 녹화 하나) 은
# hs_1920x1080_f2.22_cal20261019_old.json 으로 이름만 바꿔 보관.
# 2026-10-24 (main session): std60 (regular 60 fps session) 0.5 dpt = PROVISIONAL cal20261024d (fx 1286.2, fx +-0.3 %,
# pp +-2 px; half-split repeatability not met) until a larger-board far calibration. docs/lens_calib_f050_20261023.md.
DEFAULT_LENS = {"hs_1920x1080_f2.22": "hs_1920x1080_f2.22_cal20261022.json",
                "std_1920x1080_f0.50": "std_1920x1080_f0.50_cal20261024d.json"}


def shared_lens_path(width, height, focus_dpt, session=None):
    """초점 고정 녹화의 공용 렌즈 파일 (vio_bench_hs, imu_gt, gt_experiment 가 같이 씀).
    session = meta["session"]: "std" (일반 60 fps 세션) 이면 std_ 표, 아니면 (고속 240, 옛 녹화엔 없음) hs_ 표."""
    fd = focus_dpt or -1
    if session == "std":
        stem = f"std_{int(width)}x{int(height)}_f{fd:.2f}"
        if stem in DEFAULT_LENS or (LENS_DIR / (stem + ".json")).exists():
            return LENS_DIR / DEFAULT_LENS.get(stem, stem + ".json")
        # no std60 lens for this focus → old behaviour (HS table; e.g. std60 at 2.22 dpt → hs 2.22 node)
    stem = f"hs_{int(width)}x{int(height)}_f{fd:.2f}"
    return LENS_DIR / DEFAULT_LENS.get(stem, stem + ".json")


def collect(rec, board, max_w=0.3, cand=1500, min_corners=20):
    """느린 프레임 후보 (시간에 고르게 cand 개) 에서 보드 모서리. → [(obj, img, 무게중심 x,y, 기울기°)], 사진 크기."""
    rec = Path(rec)
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    t = np.array([int(r["t_mid_ns"]) for r in rows])
    _, gyr = read_imu(rec / "imu.csv")
    w = np.linalg.norm(np.stack([np.interp(t, gyr[:, 0], gyr[:, 1 + j]) for j in range(3)], 1), axis=1)
    slow = np.where(w < max_w)[0]
    want = set(slow[np.linspace(0, len(slow) - 1, min(cand, len(slow))).astype(int)].tolist())
    cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
    out, size, i = [], None, 0
    while True:
        r, f = cap.read()
        if not r:
            break
        if i in want:
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            size = g.shape[::-1]
            c, ids, _ = board.detect(g)
            if len(ids) >= min_corners:
                obj = board.obj[ids].astype(np.float32)
                img = c.astype(np.float32).reshape(-1, 1, 2)
                H, _ = cv2.findHomography(obj[:, :2], img.reshape(-1, 2))
                tilt = 0.0 if H is None else float(np.degrees(np.arctan(np.hypot(H[2, 0], H[2, 1]) * 0.3)))
                out.append((obj, img, *img.reshape(-1, 2).mean(0), tilt))
        i += 1
    return out, size, len(slow)


def spread(views, size, n):
    """화면 3x3 칸 x 기울기 3 구간으로 나눠 칸마다 돌아가며 뽑음 → 최대 n 장."""
    W, H = size
    tilts = np.array([v[4] for v in views])
    tq = np.quantile(tilts, [1 / 3, 2 / 3]) if len(views) > 3 else [0, 0]
    bins = {}
    for k, v in enumerate(views):
        key = (min(int(v[2] / W * 3), 2), min(int(v[3] / H * 3), 2), int(np.searchsorted(tq, v[4])))
        bins.setdefault(key, []).append(k)
    pick, lists = [], [np.array(b)[np.random.default_rng(0).permutation(len(b))].tolist() for b in bins.values()]
    while len(pick) < n and any(lists):
        for b in lists:
            if b and len(pick) < n:
                pick.append(b.pop())
    return sorted(pick), len(bins)


def fit(views, size, flags):
    rms, K, d, rv, tv = cv2.calibrateCamera([v[0] for v in views], [v[1] for v in views], size, None, None, flags=flags,
                                            criteria=TERM)
    return rms, K, d.ravel(), rv, tv


def holdout_err(views, size, flags):
    """짝수 사진으로 맞추고 홀수 사진은 렌즈 값 고정 상태로 자세만 풀어 재투영 오차 → 처음 보는 사진에서의 정확도."""
    a, b = views[0::2], views[1::2]
    _, K, d, _, _ = fit(a, size, flags)
    errs = []
    for v in b:
        ok, rv, tv = cv2.solvePnP(v[0], v[1], K, d, flags=cv2.SOLVEPNP_ITERATIVE)
        p, _ = cv2.projectPoints(v[0], rv, tv, K, d)
        errs.append(np.sqrt(np.mean(np.sum((p - v[1]) ** 2, -1))))
    return float(np.median(errs))


def calibrate(recs, board_json, n=120, log=print):
    board = aruco_scale.Board(aruco_scale.load_board_spec(str(board_json)))
    views, size, n_slow = [], None, 0
    for rec in recs:
        cache = Path(rec) / "imu_gt" / "lens_views.pkl"          # 모서리 찾기는 영상 전체를 읽어 느림 → 한 번만
        if cache.exists():
            v, size, ns = pickle.load(open(cache, "rb"))
        else:
            v, size, ns = collect(rec, board)
            cache.parent.mkdir(exist_ok=True)
            pickle.dump((v, size, ns), open(cache, "wb"))
        views += v
        n_slow += ns
        log(f"  {Path(rec).name}: 느린 프레임 {ns}, 보드 {len(v)} 장")
    if len(views) < 30:
        raise SystemExit(f"렌즈 보정에 쓸 보드 사진이 부족 ({len(views)})")
    pick, nb = spread(views, size, n)
    sel = [views[k] for k in pick]
    log(f"  고른 사진 {len(sel)} 장 (화면·기울기 칸 {nb} 개에 퍼지게)")
    scores = {m: holdout_err(sel, size, f) for m, f in MODELS.items()}
    best = min(scores, key=scores.get)
    log("  왜곡 모델 (처음 보는 사진의 재투영 오차 중앙): " + ", ".join(f"{m} {e:.3f} px" for m, e in scores.items())
        + f" → {best}")
    rms, K, d, rv, tv = fit(sel, size, MODELS[best])
    per = []
    for v, r_, t_ in zip(sel, rv, tv):
        p, _ = cv2.projectPoints(v[0], r_, t_, K, d)
        per.append(np.sqrt(np.mean(np.sum((p - v[1]) ** 2, -1))))
    per = np.array(per)
    keep = per < 2.5 * np.median(per)
    if (~keep).any():
        sel = [v for v, k in zip(sel, keep) if k]
        rms, K, d, _, _ = fit(sel, size, MODELS[best])
    log(f"  최종: 사진 {len(sel)} 장 (뺀 것 {(~keep).sum()}), rms {rms:.3f} px, f {K[0, 0]:.1f} {K[1, 1]:.1f}, "
        f"c {K[0, 2]:.1f} {K[1, 2]:.1f}")
    info = dict(model=best, rms_px=round(float(rms), 4), holdout_px={m: round(e, 4) for m, e in scores.items()},
                n_views=len(sel), n_bins=nb, recs=[Path(r).name for r in recs], size=list(size),
                board=Path(board_json).name, made=time.strftime("%Y-%m-%d %H:%M:%S"))
    return K, d, info


def save(path, K, d, info, focus_dpt=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(K=K.tolist(), dist=d.tolist(), focus_dpt=focus_dpt, **info), open(path, "w"), indent=1, ensure_ascii=False)


def load(path):
    j = json.load(open(path))
    return np.array(j["K"]), np.array(j["dist"]), j


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    ap.add_argument("--board", default=str(HERE.parent / "config" / "charuco" / "board_monitor_6x6.json"))
    ap.add_argument("--n", type=int, default=120)
    ap.add_argument("--save", default=None, help="렌즈 값 json (기본: 첫 녹화 imu_gt/lens.json)")
    a = ap.parse_args()
    K, d, info = calibrate(a.recs, a.board, a.n)
    m = json.load(open(Path(a.recs[0]) / "meta.json"))
    fd = m.get("focus_dpt_request")
    out = a.save or str(Path(a.recs[0]) / "imu_gt" / "lens.json")
    save(out, K, d, info, focus_dpt=fd if fd and fd > 0 else m.get("focus_locked_dpt"))
    print(f"→ {out}")


if __name__ == "__main__":
    main()
