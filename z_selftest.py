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


def _R_to_q(m):
    """회전행렬 -> Android 쿼터니언 (x, y, z, w). Shepperd 분기 (180° 근방도 안전)."""
    tr = np.trace(m)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2; return np.array([(m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s, (m[1,0]-m[0,1])/s, s/4])
    i = int(np.argmax(np.diag(m)))
    if i == 0:
        s = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2; return np.array([s/4, (m[0,1]+m[1,0])/s, (m[0,2]+m[2,0])/s, (m[2,1]-m[1,2])/s])
    if i == 1:
        s = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2; return np.array([(m[0,1]+m[1,0])/s, s/4, (m[1,2]+m[2,1])/s, (m[0,2]-m[2,0])/s])
    s = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2; return np.array([(m[0,2]+m[2,0])/s, (m[1,2]+m[2,1])/s, s/4, (m[1,0]-m[0,1])/s])


def t_phone_calib():
    print("폰 두 렌즈: Android 포즈 규약(X_cam=R(X_w−t)) -> OpenCV 변환, 좌우 자동 순서, 장면 대응점 회전 보정")
    import lib_phone_calib as pc
    qA = [0.70710678, -0.70710678, 0.0, 0.0]                       # S24 광각과 같은 180° 회전
    RA = pc.quat_to_R(qA)
    check("쿼터니언 (0.7071,-0.7071,0,0) -> 180° 회전, det=1", abs(pc.rotation_deg(RA) - 180) < 1e-6 and abs(np.linalg.det(RA) - 1) < 1e-9)
    R_true = cv2.Rodrigues(np.radians([0.3, -0.2, 0.1]))[0]         # B 의 진짜 상대 회전 (A 카메라 프레임 기준)
    RB = R_true @ RA
    check("회전행렬 -> 쿼터니언 -> 회전행렬 왕복", np.abs(pc.quat_to_R(_R_to_q(RB)) - RB).max() < 1e-9)
    tB = np.array([0.0, 0.01576, 0.0])                            # 센서축: B 가 +y(폰 위쪽) 15.76mm
    size = (640, 480)
    inv = {"A": dict(id="A", intrinsics=[2780.0, 2784.0, 2048.0, 1547.0, 0], active_array="Rect(0, 0 - 4080, 3060)",
                     distortion=[0.01, -0.02, 0.001, 0.0005, -0.0003], pose_rotation=qA, pose_translation=[0, 0, 0]),
           "B": dict(id="B", intrinsics=[1637.0, 1639.0, 2025.0, 1492.0, 0], active_array="Rect(0, 0 - 4000, 3000)",
                     distortion=[-0.005, 0.04, -0.02, 0.0002, -0.0001], pose_rotation=_R_to_q(RB).tolist(), pose_translation=tB.tolist())}
    L, Rr = pc.order_left_right(inv, ["A", "B"], size)
    check(f"좌우 자동 순서: A 프레임에서 B 는 x=-15.76mm(왼쪽) -> (왼,오른)=({L},{Rr})", (L, Rr) == ("B", "A"))
    st = pc.factory_stereo(inv, "B", "A", size)
    check(f"factory T = {np.round(st['T']*1000, 3).tolist()} mm (x 성분만, 음수)", abs(st["T"][0] + 0.01576) < 1e-9 and abs(st["T"][1]) < 1e-9 and abs(st["T"][2]) < 1e-9)
    # 세계점(센서축): 카메라는 -z_sensor 를 본다. 두 카메라에 투영해 정확한 대응점을 만든다.
    rng = np.random.default_rng(1)
    Z = rng.uniform(0.5, 3.0, 400); X = np.column_stack([rng.uniform(-0.9, 0.9, 400) * Z, rng.uniform(-0.6, 0.6, 400) * Z, -Z])
    def project(c, Rc, tc):
        Xc = (Rc @ (X - tc).T).T
        px, _ = cv2.projectPoints(Xc.astype(np.float64), np.zeros(3), np.zeros(3), pc.K_from_characteristics(c, size), pc.dist_from_characteristics(c))
        return px.reshape(-1, 2), Xc
    pB, XB = project(inv["B"], RB, tB); pA, XA = project(inv["A"], RA, np.zeros(3))
    inside = np.all((pB > 0) & (pB < size), 1) & np.all((pA > 0) & (pA < size), 1) & (XA[:, 2] > 0) & (XB[:, 2] > 0)
    pA, pB, XA, XB = pA[inside], pB[inside], XA[inside], XB[inside]
    check(f"양쪽 카메라 앞·화면 안 점 {inside.sum()}개, X_A = R X_B + T 일치", inside.sum() > 150 and np.abs((st["R"] @ XB.T).T + st["T"] - XA).max() < 1e-9)
    nL, nR = ls._to_normalized(pB, st["K1"], st["dist1"]), ls._to_normalized(pA, st["K2"], st["dist2"])
    samp = ls.epipolar_sampson(nL, nR, st["R"], st["T"]) * st["K1"][0, 0]
    check(f"정확한 대응점의 Sampson 오차 최대 {samp.max():.2e}px", samp.max() < 1e-4)
    # 공장 roll/pitch 가 0.36° 틀리고 광각 fx 가 초점 이동으로 0.6% 다른 상황 + 0.3px 잡음 -> 기본 설정(yaw 고정, 스케일 적합)으로 회수되는가
    R_wrong = cv2.Rodrigues(np.radians([0.3, 0.0, -0.2]))[0] @ st["R"]
    K2_wrong = st["K2"].copy(); K2_wrong[0, 0] /= 1.006; K2_wrong[1, 1] /= 1.006      # 진짜 fx 는 이 K2 의 1.006배
    pBn, pAn = pB + rng.normal(0, 0.3, pB.shape), pA + rng.normal(0, 0.3, pA.shape)
    nLn, nRn = ls._to_normalized(pBn, st["K1"], st["dist1"]), ls._to_normalized(pAn, K2_wrong, st["dist2"])
    Rf, Tf, s_w, info = pc.refine_pose(nLn, nRn, R_wrong, st["T"], float(st["K1"][0, 0]))
    err = np.degrees(cv2.Rodrigues(Rf @ st["R"].T)[0].ravel())
    check(f"roll/pitch 0.36° + fx 0.6% 오차 -> 잔여 (roll,pitch,yaw)=({err[0]:+.3f},{err[1]:+.3f},{err[2]:+.3f})°, 스케일 {s_w:.4f}(정답 1.006), Sampson 중앙값 {info['sampson_med_px']:.2f}px",
          np.abs(err).max() < 0.05 and abs(s_w - 1.006) < 0.002 and info["sampson_med_px"] < 0.6)
    # yaw 를 자유로 두면 잡음만으로도 훨씬 크게 흔들린다 (관측 약함의 수치 근거)
    Rf2, _, _, info2 = pc.refine_pose(nLn, nRn, R_wrong, st["T"], float(st["K1"][0, 0]), fix_yaw=False)
    err2 = np.degrees(cv2.Rodrigues(Rf2 @ st["R"].T)[0].ravel())
    check(f"yaw 자유 적합 시 yaw 잔여 {err2[1]:+.3f}° (고정 시 0) — 합성 깨끗한 데이터라 작지만 실쌍에선 -0.87° 로 튐(STATUS 16)", abs(err2[1]) < 0.5)
    # yaw(y축) 만 0.1° 틀리면 시차 편향 f·δ ≈ 0.8px: 이 편향은 dy 로 거의 안 보인다는 한계를 수치로 남긴다
    R_yaw = cv2.Rodrigues(np.radians([0, 0.1, 0]))[0] @ st["R"]
    s_yaw = ls.epipolar_sampson(nL, nR, R_yaw, st["T"]) * st["K1"][0, 0]
    check(f"yaw 0.1° 오차의 Sampson 중앙값 {np.median(s_yaw):.3f}px (시차 편향 {st['K2'][0,0]*np.radians(0.1):.2f}px 인데 잔차는 작다 = yaw 는 dy 로 관측 약함)",
          np.median(s_yaw) < 0.3)


def t_phone_motion():
    print("폰 이동 스테레오: 회전벡터→R, R 고정 t̂, 스테레오 깊이로 스케일, 가속도 ZUPT 적분")
    import lib_phone_calib as pc, lib_phone_motion as pm
    rng = np.random.default_rng(3)
    Rs = pc.quat_to_R([0.70710678, -0.70710678, 0, 0])                      # 센서→왼쪽 렌즈 (S24 값)
    K = np.array([[262.0, 0, 324.0], [0, 262.0, 239.0], [0, 0, 1]]); d0 = np.zeros(5)
    st = dict(R_sensor_to_left=Rs, K1=K, dist1=d0)
    # 기기 자세: A 는 임의, B 는 A 에서 기기축 회전 (2°, -3°, 1°) ; 회전벡터 = 기기→세계
    RwA = cv2.Rodrigues(np.radians([10, -20, 35]))[0]
    R_dev = cv2.Rodrigues(np.radians([2, -3, 1]))[0]                        # X_devB = R_dev X_devA
    RwB = RwA @ R_dev.T
    q = lambda R: _R_to_q(R)
    imuA, imuB = {pm.ROTVEC_KEY: dict(values=q(RwA).tolist())}, {pm.ROTVEC_KEY: dict(values=q(RwB).tolist())}
    R = pm.relative_rotation_cam(st, imuA, imuB)
    R_true = Rs @ R_dev @ Rs.T
    check(f"회전벡터 두 개 → 카메라 프레임 상대회전 오차 {pc.rotation_deg(R @ R_true.T):.2e}°", pc.rotation_deg(R @ R_true.T) < 1e-6)
    # 장면: 깊이 0.5~3m, 카메라 A→B 이동 T_true (카메라 프레임) 0.15m 옆 + 약간 앞
    T_true = np.array([-0.15, 0.01, 0.02])
    Z = rng.uniform(0.5, 3.0, 300); XA = np.column_stack([rng.uniform(-1.0, 1.0, 300) * Z, rng.uniform(-0.7, 0.7, 300) * Z, Z])
    XB = XA @ R_true.T + T_true
    nA, nB = XA[:, :2] / XA[:, 2:3], XB[:, :2] / XB[:, 2:3]
    ok = np.all(np.abs(nA) < 1.2, 1) & np.all(np.abs(nB) < 1.2, 1) & (XB[:, 2] > 0.1)
    nA, nB, Z = nA[ok], nB[ok], Z[ok]
    nAn = nA + rng.normal(0, 0.3 / 262, nA.shape); nBn = nB + rng.normal(0, 0.3 / 262, nB.shape)
    t_hat, info = pm.translation_direction(nAn, nBn, R_true, 262.0)
    ang = np.degrees(np.arccos(np.clip(t_hat @ (T_true / np.linalg.norm(T_true)), -1, 1)))
    check(f"R 고정 이동방향 t̂ 각오차 {ang:.2f}° (점 {len(nA)}, 잡음 0.3px), cheirality {info['cheirality_frac']:.2f}", ang < 1.0 and info["cheirality_frac"] > 0.9)
    s, sinfo = pm.scale_from_depth(nAn, nBn, R_true, t_hat, Z * (1 + rng.normal(0, 0.05, len(Z))))   # 스테레오 깊이 잡음 5%
    check(f"깊이 5% 잡음으로 |T| {s*1000:.1f}mm (정답 {np.linalg.norm(T_true)*1000:.1f}), MAD {sinfo['mad_rel']*100:.0f}%", abs(s - np.linalg.norm(T_true)) / np.linalg.norm(T_true) < 0.02)
    # 가속도 적분: 1.2초 동안 세계 x 로 0.15m 부드럽게 이동 (sin² 프로파일) + 바이어스 0.05 m/s², 기기 자세 RwA 고정
    t = np.arange(0, 1.2, 0.008); tau = t / 1.2
    pos = 0.15 * (tau - np.sin(2 * np.pi * tau) / (2 * np.pi)); acc_w = np.gradient(np.gradient(pos, t), t)
    a_w = np.column_stack([acc_w, np.zeros_like(t), np.zeros_like(t)]) + np.array([0.05, -0.03, 0.02])
    acc_dev = [(int(ts * 1e9), (RwA.T @ a).tolist()) for ts, a in zip(t, a_w)]
    rot = [(int(ts * 1e9), q(RwA).tolist()) for ts in t]
    res = pm.accel_displacement(acc_dev, rot, 0, int(1.2e9), Rs)
    err = np.linalg.norm(np.array(res["d_world"]) - np.array([0.15, 0, 0])) * 1000
    check(f"가속도 2회 적분 + ZUPT: 변위 {np.round(np.array(res['d_world'])*1000,1).tolist()}mm (정답 150,0,0), 오차 {err:.1f}mm, 바이어스 0.05m/s² 제거", err < 10)


if __name__ == "__main__":
    for t in (t_r01, t_r03, t_r05_r07, t_r08, t_r10, t_dense, t_phone_calib, t_phone_motion):
        try:
            t()
        except Exception as e:
            check(f"{t.__name__} 예외", False, f"{type(e).__name__}: {e}")
    n_fail = sum(1 for _, ok in RESULTS if not ok)
    print(f"\n{len(RESULTS) - n_fail}/{len(RESULTS)} PASS" + (f", {n_fail} FAIL" if n_fail else ""))
    sys.exit(1 if n_fail else 0)
