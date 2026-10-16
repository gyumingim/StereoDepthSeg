"""움직임 보정 정류 (키프레임 깊이용).

S24 두 렌즈(lens0 = 정류 왼쪽 렌즈 2, lens1 = 렌즈 5 또는 6 — scan3d 렌즈 쌍)는 같은 순간에 찍히지 않는다 (2·6 은 약 1~3 ms 로 거의 동시). 위상 차이 dt = t(lens1) − t(lens0)
(약 10~28 ms, 카메라를 열 때 정해짐)가 있어, 그 사이 폰이 움직이면 보정 외부 파라미터 (R, T) 가 실제 두 촬영 순간의
상대자세와 다르다 → 정류가 틀어지고 깊이가 틀린다.

이 모듈은 쌍마다 실제 상대자세를 구해 그 쌍만의 stereoRectify → remap 맵을 만든다.
  - 두 순간 사이 IMU 움직임 T_I0_I1 (I1 = lens1 시각의 IMU, 좌표는 I0 = lens0 시각의 IMU):
      회전 = 자이로 적분 (lens0 시각 ~ lens1 시각, OpenVINS 자이로 바이어스 bg 를 뺌) — 가장 정확.
      이동 = VIO 속도 × dt (v_G 를 lens0 시각 IMU 좌표로). 가속도 항 ½·a·dt² 은 뺌 (28 ms, 1 m/s² 에서 0.4 mm — 추정).
  - 카메라 cam1 (렌즈 2 원래 좌표) 로 옮김: ΔT = T_c1(t1)_c1(t0) = T_c_I · T_I0_I1⁻¹ · T_c_I⁻¹
      (T_c_I = IMU → cam1, VIO 추정 외부 파라미터 R_ItoC0·p_IinC0 를 쓰면 회전 때문에 생기는 렌즈 이동(레버암)까지 들어감).
  - 실제 두 카메라 상대자세: p_c2(t1) = R·p_c1(t1) + T,  p_c1(t1) = ΔR·p_c1(t0) + Δt
      → R_eff = R·ΔR,  T_eff = R·Δt + T  (= 보정 (R,T) ∘ ΔT). 이 값으로 stereoRectify.
  - 결과 깊이는 lens0 시각의 cam1 을 R1' 로 돌린 좌표(rect0'). 맵 적분은 기본 rect0 (R1) 좌표 기준이므로
      p_rect0 = R1 · R1'ᵀ · p_rect0' 로 돌려서 T_world_rect0 (lens0 시각 포즈) 를 곱한다 (scan3d._process_keyframe).
"""
import time

import cv2
import numpy as np

import lib_rectify as lr

# ΔT 가 이보다 작으면 쌍별 정류를 만들지 않고 기본 정류맵을 그대로 쓴다 (맵 만드는 시간 절약)
TINY_ROT_DEG = 0.01
TINY_TRANS_M = 2e-5
# ΔT 를 모를 때 (자이로 창이 비었거나 VIO 속도 없음) 고정 보정으로 깊이를 만들어도 되는 최대 |dt|.
# 손에 든 폰의 느린 회전 0.3 rad/s 기준 2 ms → 0.03° (640x480, f≈480 px 에서 약 0.3 px) — 그 밖은 깊이를 만들지 않는다.
FIXED_MAX_DT_MS = 2.0
GYRO_MAX_GAP_NS = 20_000_000     # 자이로 샘플 사이 이보다 큰 빈틈이 있으면 적분 안 함
GYRO_EDGE_TOL_NS = 6_000_000     # 창 끝에서 이만큼까지는 끝 샘플 값으로 이어 적분


def _expm_so3(phi):
    th = float(np.linalg.norm(phi))
    if th < 1e-12:
        return np.eye(3)
    R, _ = cv2.Rodrigues(np.asarray(phi, np.float64).reshape(3, 1))
    return R


def gyro_delta_R(samples, t_a, t_b, bg=None):
    """자이로 샘플 [(t_ns, (wx,wy,wz)) …] (시각 순) 로 [t_a, t_b] (t_a ≤ t_b) 적분 → R_Ia_Ib
    (t_b 의 IMU 축을 t_a 의 IMU 좌표로). 샘플 사이는 앞 샘플 값을 유지 (구간 상수). 창을 못 덮으면 None."""
    if t_b < t_a:
        raise ValueError("t_a ≤ t_b")
    if t_b == t_a:
        return np.eye(3)
    if not samples:
        return None
    ts = np.fromiter((s[0] for s in samples), np.int64, len(samples))
    if ts[0] > t_a + GYRO_EDGE_TOL_NS or ts[-1] < t_b - GYRO_EDGE_TOL_NS:
        return None
    b = np.zeros(3) if bg is None else np.asarray(bg, float).reshape(3)
    n = len(ts)
    i0 = max(0, int(np.searchsorted(ts, t_a, side="right")) - 1)     # t_a 직전 샘플 (없으면 첫 샘플)
    R = np.eye(3)
    for i in range(i0, n):
        lo = t_a if i == i0 else max(t_a, int(ts[i]))
        hi = min(t_b, int(ts[i + 1]) if i + 1 < n else t_b)
        if hi > lo:
            if i + 1 < n and int(ts[i + 1]) - int(ts[i]) > GYRO_MAX_GAP_NS:
                return None
            w = np.asarray(samples[i][1], float)[:3] - b
            R = R @ _expm_so3(w * ((hi - lo) * 1e-9))
        if hi >= t_b:
            break
    return R


def T_c1_I(st, rp, mot):
    """IMU → cam1 (렌즈 2 원래 좌표) 4x4. VIO 가 준 R_ItoC0·p_IinC0 (C0 = rect0) 가 있으면 그것 (레버암 포함),
    없으면 보정값 R_sensor_to_left (이동 0)."""
    T = np.eye(4)
    R1 = np.asarray(rp["R1"], float)
    if mot.get("R_ItoC0") is not None and mot.get("p_IinC0") is not None:
        R_C0_I = np.asarray(mot["R_ItoC0"], float).reshape(3, 3)
        p = np.asarray(mot["p_IinC0"], float).reshape(3)
        T[:3, :3] = R1.T @ R_C0_I
        T[:3, 3] = R1.T @ p
    else:
        T[:3, :3] = np.asarray(st["R_sensor_to_left"], float)
    return T


def motion_delta(st, rp, mot, gyro):
    """dict(dt_ns, t0_ns, v_G?, R_GtoI?, bg?, R_ItoC0?, p_IinC0?) + 자이로 샘플 → (ΔT 4x4 = T_c1(t1)_c1(t0), info) 또는
    (None, info). info: dt_ms, rot_deg, trans_mm, src ('gyro+vio' | 'gyro' | …), why (실패 이유)."""
    dt_ns = int(mot.get("dt_ns") or 0)
    t0 = int(mot["t0_ns"])
    t1 = t0 + dt_ns
    info = dict(dt_ms=round(dt_ns * 1e-6, 3))
    lo, hi = (t0, t1) if dt_ns >= 0 else (t1, t0)
    R_lohi = gyro_delta_R(gyro, lo, hi, mot.get("bg"))
    if R_lohi is None:
        info["why"] = "자이로 창 없음"
        return None, info
    R_I0_I1 = R_lohi if dt_ns >= 0 else R_lohi.T
    v_G, R_GtoI = mot.get("v_G"), mot.get("R_GtoI")
    if v_G is not None and R_GtoI is not None:
        v_I0 = np.asarray(R_GtoI, float).reshape(3, 3) @ np.asarray(v_G, float).reshape(3)
        p = v_I0 * (dt_ns * 1e-9)
        info["src"] = "gyro+vio"
        info["speed_mps"] = round(float(np.linalg.norm(v_G)), 4)
    elif mot.get("static"):
        p = np.zeros(3)                       # --ov none: 폰을 세워 둔다고 보고 이동 0 (회전은 자이로)
        info["src"] = "gyro"
    else:
        info["why"] = "VIO 속도 없음"
        return None, info
    T_I0_I1 = np.eye(4)
    T_I0_I1[:3, :3] = R_I0_I1
    T_I0_I1[:3, 3] = p
    Tc = T_c1_I(st, rp, mot)
    dT = Tc @ np.linalg.inv(T_I0_I1) @ np.linalg.inv(Tc)
    info["rot_deg"] = round(float(np.degrees(np.arccos(np.clip((np.trace(dT[:3, :3]) - 1) / 2, -1, 1)))), 4)
    info["trans_mm"] = round(float(np.linalg.norm(dT[:3, 3])) * 1000, 4)
    return dT, info


def rectify_with_delta(st, dT, alpha=0.0):
    """보정 (R,T) ∘ ΔT 로 그 쌍만의 정류맵 (lib_rectify.rectify_maps 와 같은 dict + rect_ms)."""
    R = np.asarray(st["R"], float)
    T = np.asarray(st["T"], float).reshape(3)
    R_eff = R @ dT[:3, :3]
    T_eff = R @ dT[:3, 3] + T
    cal = [dict(K=np.array(st[f"K{i}"], float), dist=np.array(st[f"dist{i}"], float), image_size=st["image_size"])
           for i in (1, 2)]
    t = time.perf_counter()
    rp = lr.rectify_maps(cal[0], cal[1], R_eff, T_eff, alpha=alpha)
    rp["rect_ms"] = 1000 * (time.perf_counter() - t)
    return rp


# 두 촬영 순간 사이 이동이 기선(15.5 mm)에 비해 크면 실제 기선 방향이 크게 돌아 (빠른 손 움직임 x dt 22 ms = 수~수십 mm)
# stereoRectify 가 상하 배치로 정류하거나 기선이 거의 0 이 된다 -> 그 키프레임은 깊이를 만들지 않는다 (2026-10-16 실시간 15:09 오류).
MAX_BASELINE_TURN_DEG = 25.0
MIN_BASELINE_RATIO = 0.6


def baseline_check(st, dT):
    """(ok, info): 보정 기선 T 대비 실제 기선 T_eff 의 방향 변화(deg)와 길이 비."""
    R = np.asarray(st["R"], float)
    T = np.asarray(st["T"], float).reshape(3)
    T_eff = R @ dT[:3, 3] + T
    n0, n1 = float(np.linalg.norm(T)), float(np.linalg.norm(T_eff))
    turn = float(np.degrees(np.arccos(np.clip(T @ T_eff / max(n0 * n1, 1e-12), -1, 1))))
    ratio = n1 / max(n0, 1e-12)
    ok = turn <= MAX_BASELINE_TURN_DEG and ratio >= MIN_BASELINE_RATIO
    return ok, dict(base_turn_deg=round(turn, 2), base_ratio=round(ratio, 3))


def R_rect0_from(rp_nominal, rp_pair):
    """쌍별 정류 좌표 rect0' → 기본 rect0 회전 (p_rect0 = R1 · R1'ᵀ · p_rect0')."""
    return np.asarray(rp_nominal["R1"], float) @ np.asarray(rp_pair["R1"], float).T


def tiny(info):
    return info.get("rot_deg", 1.0) < TINY_ROT_DEG and info.get("trans_mm", 1.0) < TINY_TRANS_M * 1000
