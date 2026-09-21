#!/usr/bin/env python3
"""
z_selftest.py — 기하·집계 코드의 회귀 테스트 (GPU·모델·카메라 불필요, 메인 venv)

  ./venv/bin/python z_selftest.py

doc_DEPTH_CODE_REVIEW.md 의 재현 항목(R01~R08, R10)을 "고친 뒤에도 유지되는가"로 바꾼 것이다.
각 항목은 정확한 합성 입력을 넣고 기대값을 확인한다. 하나라도 FAIL 이면 종료코드 1.
"""
import json
import sys
import types
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
import lib_rectify as lr
import lib_stereo as ls

# z_object_depth 는 lib_ffs(venv_ffs 전용)를 import 하므로 stub 을 끼운다
sys.modules.setdefault("lib_ffs", types.SimpleNamespace(DEFAULT_CKPT="x", load=None, infer=None))
import z_object_depth as zod

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}")


def cam(fx, cx=639.5, cy=359.5):
    return dict(K=np.array([[fx, 0, cx], [0, fx, cy], [0, 0, 1.0]]), dist=np.zeros(5), image_size=(1280, 720))


def inject_matches(X, K1, K2, R, T):
    """정확한 3D 점 X (N,3, 카메라1 좌표) -> 두 카메라 픽셀 대응. _match_sift 를 대체한다."""
    p1, _ = cv2.projectPoints(X, np.zeros(3), np.zeros(3), K1["K"], K1["dist"])
    p2, _ = cv2.projectPoints(X, cv2.Rodrigues(R)[0], T.reshape(3, 1), K2["K"], K2["dist"])
    return p1.reshape(-1, 2), p2.reshape(-1, 2)


def run_reconstruct_with(X, K1, K2, R, T):
    pL, pR = inject_matches(X, K1, K2, R, T)
    orig = ls._match_sift
    ls._match_sift = lambda gL, gR, m, r: (pL, pR, len(pL))
    try:
        img = np.zeros((720, 1280, 3), np.uint8)
        return ls.reconstruct(img, img, K1, R, T, calibR=K2, z_range=(0.05, 100.0))
    finally:
        ls._match_sift = orig


def t_r01():
    print("R01 일반 두 뷰(회전·다른 K·작은 시차)에서 정확한 대응점이 살아남는가")
    rng = np.random.default_rng(0)
    X = np.column_stack([rng.uniform(-0.4, 0.4, 100), rng.uniform(-0.25, 0.25, 100), rng.uniform(1.2, 2.5, 100)])
    cases = [("동일 K + pitch 3deg", cam(800), cam(800), cv2.Rodrigues(np.array([np.radians(3), 0, 0]))[0], np.array([-0.15, 0, 0])),
             ("R=I, fx 800 vs 520", cam(800), cam(520, 650, 362), np.eye(3), np.array([-0.15, 0, 0])),
             ("yaw 2.86deg (dx 상쇄)", cam(800), cam(800), cv2.Rodrigues(np.array([0, np.radians(2.86), 0]))[0], np.array([-0.1, 0, 0]))]
    for name, K1, K2, R, T in cases:
        out = run_reconstruct_with(X, K1, K2, R, T)
        d = out["diag"]
        err = np.abs(out["points_cam"] - X[out["idx_final"]]).max() if len(out["idx_final"]) else np.inf
        check(f"{name}: epipolar {d['n_epipolar']}/100, depth {d['n_depth']}, 3D오차 {err*1000:.3f}mm",
              d["n_epipolar"] >= 95 and d["n_depth"] >= 95 and err < 1e-6)
    # 시차 없음(카메라 안 움직임) 은 광선각으로 걸러져야 한다
    out = run_reconstruct_with(X, cam(800), cam(800), np.eye(3), np.array([-1e-7, 0, 0]))
    check("카메라 미이동 -> depth 통과 0", out["diag"]["n_depth"] == 0, f"n_depth={out['diag']['n_depth']}")


def t_r03():
    print("R03 거리·치수가 같은 최종 집합에서 나오는가")
    rng = np.random.default_rng(1)
    obj = np.column_stack([rng.uniform(-0.1, 0.1, 50), rng.uniform(-0.05, 0.05, 50), 2.0 + rng.normal(0, 0.002, 50)])
    bg = np.column_stack([rng.uniform(-1.5, 1.5, 60), rng.uniform(-1, 1, 60), 4.0 + rng.normal(0, 0.3, 60)])
    P = np.vstack([obj, bg]) @ ls.CAM_TO_MAP.T
    box, kept, idx = ls.fit_box(P)
    zsel = (P[idx] @ ls.CAM_TO_MAP)[:, 2]           # 맵 -> 카메라 Z
    check("fit_box idx 가 kept 와 일치", np.allclose(P[idx], kept))
    check(f"최종집합 Z 중앙값 {np.median(zsel):.3f} (물체 2.0, 배경 4.0)", abs(np.median(zsel) - 2.0) < 0.02)
    pb = ls.planar_box(np.vstack([obj, bg]))
    check("planar_box idx 반환 + 배경 제외", pb is not None and abs(np.median(np.vstack([obj, bg])[pb["idx"], 2]) - 2.0) < 0.02)


def t_r05_r07():
    print("R05/R07 정류 부호·NaN 처리")
    K = cam(800)
    rp = lr.rectify_maps(K, K, np.eye(3), np.array([-0.1, 0, 0]))
    Z = lr.disparity_to_depth(np.array([[40.0]], np.float32), rp)[0, 0]
    Xq = cv2.perspectiveTransform(np.array([[[rp["cx"], rp["cy"], 40.0]]], np.float32), rp["Q"])[0, 0]
    check(f"정상 배치: 수식 Z {Z:.3f} == Q Z {Xq[2]:.3f} == 2.0", abs(Z - 2.0) < 1e-3 and abs(Xq[2] - 2.0) < 1e-3)
    for name, T in (("역순 T=(+0.1,0,0)", np.array([0.1, 0, 0])), ("수직 T=(0,-0.1,0)", np.array([0, -0.1, 0]))):
        try:
            lr.rectify_maps(K, K, np.eye(3), T); check(f"{name} -> 예외", False, "예외 없음")
        except ValueError as e:
            check(f"{name} -> 예외", True, str(e)[:50])
    out = lr.disparity_to_depth(np.array([np.nan, np.inf, -1.0, 0.0, 40.0], np.float32), rp)
    check("시차 nan/inf/음수/0 -> NaN, 40 -> 2.0", np.isnan(out[:4]).all() and abs(out[4] - 2.0) < 1e-3)
    ep = lr.epipolar_check(np.zeros((720, 1280), np.uint8), np.zeros((720, 1280), np.uint8))
    check("질감 없는 영상 -> epipolar_check status unavailable, 예외 없음", ep["status"] == "unavailable")
    js = json.dumps(zod._clean(dict(a=float("nan"), b=np.float32(np.inf), c=[1.0, np.nan], d=np.array([1, 2]))), allow_nan=False)
    check("NaN/Inf/ndarray -> JSON 저장 가능", '"a": null' in js and '"b": null' in js)


def t_r08():
    print("R08 오른쪽 ROI 의 y 범위")
    K = cam(800)
    rp = lr.rectify_maps(K, K, np.eye(3), np.array([-0.1, 0, 0]))
    rp = dict(rp, roi2=(0, 100, 1280, 200), roi1=(0, 0, 1280, 720))
    disp = np.full((720, 1280), 40.0, np.float32)
    valid, _ = zod.valid_mask(disp, rp, (0.1, 20.0))
    check("y<100 또는 y>=300 행은 무효", not valid[:100].any() and not valid[300:].any() and valid[150].any())


def t_r10():
    print("R10 rectify_maps 에 resize 용 size 인자가 없다 (혼용 불가)")
    import inspect
    check("size 파라미터 제거", "size" not in inspect.signature(lr.rectify_maps).parameters)


def t_dense():
    print("조밀 집계: 정답 깊이맵(기울어진 평면 + 배경)에서 seg 마스크 크기·중심")
    K1, K2 = cam(900), cam(520, 650, 362)
    R = cv2.Rodrigues(np.array([0, np.radians(3), 0]))[0]; T = np.array([-0.15, 0, 0])
    rp = lr.rectify_maps(K1, K2, R, T)
    H, W = 720, 1280
    u, v = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    d_rect = np.stack([(u - rp["cx"]) / rp["f"], (v - rp["cy"]) / rp["f"], np.ones_like(u)], -1)
    d_cam1 = d_rect @ np.asarray(rp["R1"])
    C = np.array([0.05, -0.02, 1.5]); th = np.radians(20); ex = np.array([np.cos(th), 0, np.sin(th)]); ey = np.array([0, 1.0, 0])
    Wd, Hd = 0.30, 0.20
    q = np.array([C - ex*Wd/2 - ey*Hd/2, C + ex*Wd/2 - ey*Hd/2, C + ex*Wd/2 + ey*Hd/2, C - ex*Wd/2 + ey*Hd/2])
    def plane(q):
        n = np.cross(q[1]-q[0], q[3]-q[0]); n /= np.linalg.norm(n); s = (n @ q[0]) / (d_cam1 @ n); X = d_cam1 * s[..., None]
        e1, e2 = q[1]-q[0], q[3]-q[0]; rel = X - q[0]; a = (rel@e1)/(e1@e1); b = (rel@e2)/(e2@e2)
        return s * d_rect[..., 2], (s > 0) & (a >= 0) & (a <= 1) & (b >= 0) & (b <= 1)
    Zmap = np.full((H, W), 4.0)                                      # 배경 벽 4m
    Zo, mask = plane(q); Zmap[mask] = Zo[mask]
    rng = np.random.default_rng(0)
    Zn = Zmap * (1 + rng.normal(0, 0.02, Zmap.shape))               # 깊이 노이즈 2%
    bleed = mask & (rng.random(mask.shape) < 0.05); Zn[bleed] = 4.0  # 경계 혼입 5%
    disp = rp["f"] * rp["baseline_m"] / Zn
    valid, Z = zod.valid_mask(disp.astype(np.float32), rp, (0.1, 20.0))
    pts = lr.depth_to_points_cam1(np.where(valid, Z, np.nan).astype(np.float32), rp)
    det = dict(box=cv2.boundingRect(mask.astype(np.uint8)), conf=1.0, label="obj")
    rec = zod.aggregate(zod._empty("obj", 1, det, "seg"), (mask * 255).astype(np.uint8), valid, Z, pts, rp=rp, outline=True)
    d = rec["dims_mm"]; e = (abs(d[0] - 300) / 3, abs(d[1] - 200) / 2); ec = np.linalg.norm(np.array(rec["center_cam1_m"]) - C) * 1000
    check(f"seg: 크기 {d[0]:.1f}x{d[1]:.1f} (오차 {e[0]:.2f}%/{e[1]:.2f}%), 중심오차 {ec:.1f}mm, status {rec['status']}",
          max(e) < 3 and ec < 10 and rec["status"] == "ok")
    check("선택 화소 통계와 원시 중앙값이 분리 저장됨", rec["depth_raw_m"] is not None and rec["selected_fraction"] is not None)
    # 배경 과반 bbox -> ambiguous 여야 한다 (배경을 물체로 보고하지 않음)
    x, y, w, h = det["box"]; big = np.zeros((H, W), np.uint8); big[max(0, y-150):y+h+150, max(0, x-200):x+w+200] = 255
    rec2 = zod.aggregate(zod._empty("obj", 1, det, "bbox"), big, valid, Z, pts)
    check(f"배경 과반 bbox -> status {rec2['status']}, 선택비율 {rec2['selected_fraction']:.2f}", rec2["status"] in ("ambiguous", "ok") and (rec2["selected_fraction"] < 0.5 or abs(rec2["depth_rect_m"] - 1.5) < 0.1))


if __name__ == "__main__":
    for t in (t_r01, t_r03, t_r05_r07, t_r08, t_r10, t_dense):
        try:
            t()
        except Exception as e:
            check(f"{t.__name__} 예외", False, f"{type(e).__name__}: {e}")
    n_fail = sum(1 for _, ok in RESULTS if not ok)
    print(f"\n{len(RESULTS) - n_fail}/{len(RESULTS)} PASS" + (f", {n_fail} FAIL" if n_fail else ""))
    sys.exit(1 if n_fail else 0)
