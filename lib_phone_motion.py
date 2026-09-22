"""폰을 손으로 옆으로 옮긴 두 시점(A, B)을 넓은 baseline 스테레오로 쓰기 — 자이로가 회전, 15.76mm 스테레오가 스케일.

왜: 두 렌즈 baseline 15.76mm 는 1m 에서 시차 7px → 시차 0.5px 오차가 깊이 7% 다. 폰을 10~20cm 옮기면 baseline 이
7~13배라 같은 시차 오차가 깊이 0.5~1% 가 된다. 예전 1대(--mono) 모드의 약점은 두 촬영 사이 회전을 몰라 "순수 평행이동"
을 가정한 것(STATUS 문제점 9) — 회전벡터 센서(게임 회전벡터: 자이로+가속도 융합, 지자기 없음)가 그 회전을 준다.

역할 분담 (각 센서가 관측할 수 있는 것만 맡긴다):
  회전 R      : 게임 회전벡터 q_A, q_B → R = Rs · R(q_B)ᵀ R(q_A) · Rsᵀ (Rs = 센서→왼쪽 렌즈 카메라 회전, 공장값)
                 원시 자이로 적분과 비교해 불일치를 진단으로 남긴다.
  이동 방향 t̂ : 영상 대응점. R 을 알면 에피폴라 제약 (x_B × R x_A)ᵀ t = 0 이 t 에 선형 → 최소 특이벡터 (IRLS)
  이동 크기 |T|: A 시점의 metric 깊이(15.76mm 스테레오 삼각측량 또는 FFS 깊이맵) — X_B = R X_A + s t̂ 가 x_B 에 투영되도록 s
  가속도 적분  : 선가속도(중력 제거) 를 회전벡터로 세계 프레임에 옮겨 2번 적분, 양끝 정지 가정으로 바이어스 제거(ZUPT).
                 손으로 1~2초 이동하면 1~3cm 오차라 스케일의 **교차 검증**으로만 쓴다 (주 스케일 아님).

한계(정직하게): |T| 의 절대 정확도는 15.76mm 스테레오의 yaw/baseline 정확도를 그대로 물려받는다(편향은 같고 잡음만 준다).
--known 이나 --board 로 그 편향을 잡은 뒤에야 이 경로가 절대 미터로 정확해진다.
"""
import numpy as np
import cv2

import lib_phone_calib as pc
import lib_rectify as lr
import lib_stereo as ls

ROTVEC_KEY = "game_rotation_vector"


def _hom(n):
    return np.column_stack([n, np.ones(len(n))])


def R_world_from_device(rotvec_values):
    """Android 회전벡터 (x, y, z, [w]) -> R : v_world = R · v_device (SensorManager.getRotationMatrixFromVector 와 같은 뜻)."""
    v = np.asarray(rotvec_values, float)
    if len(v) >= 4 and abs(np.linalg.norm(v[:4]) - 1) < 0.05:
        q = v[:4]
    else:                                     # w 가 없으면 단위 사원수 조건으로 복원
        x, y, z = v[:3]; q = np.array([x, y, z, np.sqrt(max(0.0, 1 - x*x - y*y - z*z))])
    return pc.quat_to_R(q)


def relative_rotation_cam(st, imuA, imuB, key=ROTVEC_KEY):
    """A→B 기기 회전을 왼쪽 렌즈 카메라 프레임으로: X_camB = R · X_camA (방향벡터 기준)."""
    Rs = np.asarray(st["R_sensor_to_left"], float)
    RA, RB = R_world_from_device(imuA[key]["values"]), R_world_from_device(imuB[key]["values"])
    return Rs @ RB.T @ RA @ Rs.T


def gyro_integrated_rotation(gyro_samples, tA_ns, tB_ns):
    """원시 자이로(rad/s, 기기 프레임) 를 tA..tB 사이 적분한 회전 (기기 프레임, X_devB = R X_devA). 회전벡터와의 교차검증용."""
    s = [(t, np.asarray(v[:3], float)) for t, v in gyro_samples if tA_ns <= t <= tB_ns]
    if len(s) < 2:
        return None
    R = np.eye(3)
    for (t0, w0), (t1, w1) in zip(s[:-1], s[1:]):
        dt = (t1 - t0) * 1e-9
        R = cv2.Rodrigues(0.5 * (w0 + w1) * dt)[0] @ R      # 기기 프레임 각속도: 누적 R_{k+1} = exp(ω dt) R_k
    return R.T                                              # 위 R 은 A 프레임의 벡터를 B 로 보내는 것의 역 → 전치


def translation_direction(nA, nB, R, f_px, iters=60, cauchy_px=1.0):
    """R 이 주어졌을 때 t̂ (단위) : 대응점마다 (x_B × R x_A)ᵀ t = 0. Cauchy IRLS 최소 특이벡터. 부호는 cheirality.

    반환 t_hat, dict(residual_px(Sampson), inlier_frac, n, cheirality_frac)
    """
    xA, xB = _hom(nA), _hom(nB)
    M = np.cross(xB, xA @ R.T)
    w = np.ones(len(M))
    t = None
    for _ in range(iters):
        _, _, Vt = np.linalg.svd(M * np.sqrt(w)[:, None], full_matrices=False)
        t_new = Vt[-1]
        r = ls.epipolar_sampson(nA, nB, R, t_new) * f_px
        w_new = 1.0 / (1.0 + (r / cauchy_px) ** 2)
        if t is not None and np.allclose(w, w_new, atol=1e-7):
            break
        t, w = t_new, w_new
    # cheirality: 양 카메라 앞에 놓이는 점이 많은 부호
    def front_frac(tv):
        X, okw = ls.triangulate(nA, nB, R, tv)
        if len(X) == 0:
            return 0.0
        X = X[okw]
        XB = X @ R.T + tv
        return float(np.mean((X[:, 2] > 0) & (XB[:, 2] > 0)))
    fp, fn = front_frac(t), front_frac(-t)
    if fn > fp:
        t, fp = -t, fn
    r = ls.epipolar_sampson(nA, nB, R, t) * f_px
    return t, dict(residual_med_px=float(np.median(r)), inlier_frac=float((r < 2.0).mean()), n=int(len(r)),
                   cheirality_frac=fp, residual_px=r)


def scale_from_depth(nA, nB, R, t_hat, Z_A):
    """A 시점 metric 깊이 Z_A(대응점별) 로 |T|.  X_A = Z_A·x̃_A,  X_B = R X_A + s t̂,  x_B = X_B[:2]/X_B[2].
    점마다 두 선형식  s (t̂_x − x_B t̂_z) = x_B (RX_A)_z − (RX_A)_x ,  s (t̂_y − y_B t̂_z) = y_B (RX_A)_z − (RX_A)_y  를 LS 로 풀고
    중앙값을 취한다. 반환 s, dict(s_per_point, mad_rel, n)
    """
    XA = _hom(nA) * Z_A[:, None]
    RX = XA @ R.T
    a = np.column_stack([t_hat[0] - nB[:, 0] * t_hat[2], t_hat[1] - nB[:, 1] * t_hat[2]])
    b = np.column_stack([nB[:, 0] * RX[:, 2] - RX[:, 0], nB[:, 1] * RX[:, 2] - RX[:, 1]])
    s_pt = np.sum(a * b, 1) / np.maximum(np.sum(a * a, 1), 1e-12)
    ok = np.isfinite(s_pt) & (s_pt > 0)
    if ok.sum() < 8:
        raise ValueError(f"스케일을 줄 수 있는 대응점 {int(ok.sum())}개 — 이동이 너무 작거나 깊이가 없음")
    s = float(np.median(s_pt[ok]))
    mad = float(np.median(np.abs(s_pt[ok] - s)) * 1.4826 / s)
    return s, dict(s_per_point=s_pt[ok], mad_rel=mad, n=int(ok.sum()))


def accel_displacement(samples_acc, samples_rot, tA_ns, tB_ns, Rs):
    """선가속도(기기 프레임, 중력 제거) 를 세계 프레임으로 옮겨 2번 적분 + 양끝 정지(ZUPT) 바이어스 제거.

    반환 dict(d_world, d_camA (A 시점 왼쪽 렌즈 프레임), n, duration_s, v_end_before_zupt) 또는 None.
    """
    acc = [(t, np.asarray(v[:3], float)) for t, v in samples_acc if tA_ns <= t <= tB_ns]
    rot = [(t, R_world_from_device(v)) for t, v in samples_rot if tA_ns - 50_000_000 <= t <= tB_ns + 50_000_000]
    if len(acc) < 10 or len(rot) < 2:
        return None
    rot_t = np.array([t for t, _ in rot])
    def R_at(t):
        return rot[int(np.argmin(np.abs(rot_t - t)))][1]
    ts = np.array([t for t, _ in acc]) * 1e-9
    a_w = np.array([R_at(t) @ a for t, a in acc])
    dt = np.diff(ts)
    v = np.vstack([np.zeros(3), np.cumsum(0.5 * (a_w[1:] + a_w[:-1]) * dt[:, None], 0)])
    T = ts[-1] - ts[0]
    bias = v[-1] / max(T, 1e-6)                           # 끝에서 v=0 이 되도록 상수 가속도 바이어스
    v_c = v - bias * (ts - ts[0])[:, None]
    d_w = np.sum(0.5 * (v_c[1:] + v_c[:-1]) * dt[:, None], 0)
    RA = R_at(acc[0][0])                                  # 세계 → A 시점 기기 → 카메라
    d_cam = Rs @ RA.T @ d_w
    return dict(d_world=d_w.round(4).tolist(), d_camA=d_cam.round(4).tolist(), norm_m=float(np.linalg.norm(d_w)),
                n=len(acc), duration_s=float(T), v_end_before_zupt_mps=float(np.linalg.norm(v[-1])))


def rotation_deg(R):
    return pc.rotation_deg(R)


def analyze(st, imgA_L, imgA_R, imgB_L, imuA, imuB, sensors=None, tA_ns=None, tB_ns=None, ratio=0.75, min_flow_px=2.0):
    """두 시점 번들(왼쪽=초광각 영상 + IMU) → R, T(metric), 진단. sensors 는 {type: [(ts, values)]} (자이로/가속도 교차검증, 선택).

    스케일용 A 시점 깊이는 3-뷰 SIFT (UW_A ↔ W_A 스테레오 삼각측량, UW_A ↔ UW_B) 로 구한다 — FFS 없이(venv) 돈다.
    """
    K1, d1, K2, d2 = (np.asarray(st[k], float) for k in ("K1", "dist1", "K2", "dist2"))
    f_px = float(K1[0, 0])
    R = relative_rotation_cam(st, imuA, imuB)
    gA, gB, gAR = (cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) for im in (imgA_L, imgB_L, imgA_R))
    # UW_A ↔ UW_B (이동 쌍)
    sift = cv2.SIFT_create()
    kA, dA = sift.detectAndCompute(gA, None); kB, dB = sift.detectAndCompute(gB, None); kR, dR = sift.detectAndCompute(gAR, None)
    if dA is None or dB is None or dR is None:
        raise ValueError("SIFT 특징 부족")
    bf = cv2.BFMatcher(cv2.NORM_L2)
    mAB = [m for m, n in bf.knnMatch(dA, dB, k=2) if m.distance < ratio * n.distance]
    mAR = {m.queryIdx: m.trainIdx for m, n in bf.knnMatch(dA, dR, k=2) if m.distance < ratio * n.distance}
    pA = np.array([kA[m.queryIdx].pt for m in mAB], float).reshape(-1, 2)
    pB = np.array([kB[m.trainIdx].pt for m in mAB], float).reshape(-1, 2)
    nA, nB = ls._to_normalized(pA, K1, d1), ls._to_normalized(pB, K1, d1)
    if len(nA) < 20:
        raise ValueError(f"A↔B 대응점 {len(nA)}개 — 질감 있는 장면에서 다시")
    # 회전만 보정한 뒤 남는 흐름(px): 이동이 있으면 커야 한다. 작으면 baseline 판정 불가
    nA_rot = (_hom(nA) @ R.T); nA_rot = nA_rot[:, :2] / nA_rot[:, 2:3]
    flow = np.linalg.norm(ls._to_pixel(nB, K1) - ls._to_pixel(nA_rot, K1), axis=1)
    diag = dict(rotation_deg=rotation_deg(R), rotvec_deg_xyz=np.degrees(cv2.Rodrigues(R)[0].ravel()).round(3).tolist(),
                n_AB=int(len(nA)), flow_after_rotation_med_px=float(np.median(flow)), warnings=[])
    if tA_ns and tB_ns:
        diag["dt_s"] = (tB_ns - tA_ns) / 1e9
        if abs(diag["dt_s"]) > 10:      # 게임 회전벡터는 지자기가 없어 yaw 가 분당 ~0.1° 흐른다 → A,B 는 몇 초 안에
            diag["warnings"].append(f"A→B {diag['dt_s']:.0f}s: 회전벡터 yaw 드리프트 가능 (10초 안에 찍을 것)")
    if sensors and tA_ns and tB_ns and pc and 4 in sensors:
        Rg = gyro_integrated_rotation(sensors[4], tA_ns, tB_ns)
        if Rg is not None:
            Rs = np.asarray(st["R_sensor_to_left"], float)
            diag["gyro_vs_rotvec_deg"] = rotation_deg((Rs @ Rg @ Rs.T) @ R.T)
    if np.median(flow) < min_flow_px:
        diag["status"] = "baseline_undetectable"
        return dict(R=R, T=None, diag=diag)
    t_hat, tinfo = translation_direction(nA, nB, R, f_px)
    diag.update(t_hat=t_hat.round(4).tolist(), epipolar_residual_med_px=tinfo["residual_med_px"],
                epipolar_inlier_frac=tinfo["inlier_frac"], cheirality_frac=tinfo["cheirality_frac"])
    # 스케일: 3-뷰 점 (A 의 같은 SIFT 특징이 W_A 에도 매칭) → 15.76mm 스테레오 삼각측량 깊이
    idx3 = [i for i, m in enumerate(mAB) if m.queryIdx in mAR]
    if len(idx3) < 8:
        diag["status"] = "scale_unavailable"; diag["n_3view"] = len(idx3)
        return dict(R=R, T=None, t_hat=t_hat, diag=diag)
    pAR = np.array([kR[mAR[mAB[i].queryIdx]].pt for i in idx3], float)
    nAR = ls._to_normalized(pAR, K2, d2)
    Rst, Tst = np.asarray(st["R"], float), np.asarray(st["T"], float)
    samp = ls.epipolar_sampson(nA[idx3], nAR, Rst, Tst) * f_px
    good = samp < 2.0
    X, okw = ls.triangulate(nA[idx3][good], nAR[good], Rst, Tst)
    ok = okw & (X[:, 2] > 0.1) & (X[:, 2] < 20)
    sel = np.array(idx3)[good][ok]
    inl = tinfo["residual_px"][sel] < 2.0
    s, sinfo = scale_from_depth(nA[sel][inl], nB[sel][inl], R, t_hat, X[ok][inl][:, 2])
    T = s * t_hat
    diag.update(n_3view=int(len(idx3)), n_scale=sinfo["n"], scale_mad_rel=sinfo["mad_rel"], baseline_m=float(s), status="ok")
    # 정지한 폰 두 번들(실제 이동 0)에서 드리프트 0.44° 만으로 흐름 3.4px·baseline 16.6mm·MAD 42% 가 나왔다 → 게이트
    if sinfo["mad_rel"] > 0.25 or np.median(flow) < 5.0:
        diag["status"] = "baseline_unreliable"
        diag["warnings"].append(f"스케일 산포 {sinfo['mad_rel']*100:.0f}% / 흐름 {np.median(flow):.1f}px — 이동이 작거나(<5cm) 회전만 있음. 10~20cm 옆으로 옮길 것")
    if sensors and tA_ns and tB_ns and 10 in sensors and 15 in sensors:
        acc = accel_displacement(sensors[10], sensors[15], tA_ns, tB_ns, np.asarray(st["R_sensor_to_left"], float))
        if acc:
            # 카메라가 A→B 로 d 만큼 움직이면 X_B = R (X_A − d_camA) → T = −R d_camA
            T_acc = -R @ np.asarray(acc["d_camA"])
            acc["T_from_accel"] = T_acc.round(4).tolist()
            acc["angle_to_visual_deg"] = float(np.degrees(np.arccos(np.clip(T_acc @ T / max(np.linalg.norm(T_acc) * np.linalg.norm(T), 1e-9), -1, 1))))
            acc["norm_ratio_accel_over_visual"] = float(np.linalg.norm(T_acc) / max(np.linalg.norm(T), 1e-9))
            diag["accel"] = acc
    return dict(R=R, T=T, t_hat=t_hat, diag=diag)


def motion_rectify_order(st, R, T):
    """이동 쌍의 (왼, 오른) 순서: Tx<0 이 되는 쪽. 반환 ('AB'|'BA', R_lr, T_lr). 세로 이동이면 rectify_maps 가 거부."""
    if T[0] < 0:
        return "AB", R, T
    return "BA", R.T, -R.T @ T
