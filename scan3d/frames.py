"""좌표계와 변환 (scan3d 전체에서 이 파일만 축 규약을 정의한다).

축 규약
- I (IMU) = Android 센서(기기) 좌표: 폰을 세로로 정면에서 볼 때 x 오른쪽, y 위, z 화면 밖.
- cam1 = 렌즈 2(초광각) 원래 카메라 좌표, OpenCV 규약: x 영상 오른쪽, y 영상 아래, z 광축 앞.
- rect0 = cam1 을 stereoRectify 의 R1 로 돌린 정류 왼쪽 좌표 (p_rect0 = R1 @ p_cam1). 깊이·FFS·YOLO 모두 이 영상 기준.
- G (world) = OpenVINS 월드: 중력 정렬 z 위, yaw·원점은 VIO 초기화 시점.
- T_a_b (4x4) 는 "b 좌표의 점 → a 좌표" (p_a = T_a_b @ p_b). 회전 R_a_b 도 같은 뜻.
- OpenVINS 쿼터니언은 JPL [x,y,z,w], q_GtoI → R_GtoI (G 의 벡터를 I 좌표로).

맵 적분: p_world = T_world_rect0 @ p_rect0,  T_world_rect0 = T_world_I @ T_I_rect0.
"""
import numpy as np


def make_T(R, p):
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float)
    T[:3, 3] = np.asarray(p, float).reshape(3)
    return T


def inv_T(T):
    R, p = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ p)


def jpl_quat_to_R(q):
    """JPL q=[x,y,z,w] → R (OpenVINS ov_core::quat_2_Rot 와 같은 식). q_GtoI 를 주면 R_GtoI."""
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q)
    v, w = q[:3], q[3]
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return (2 * w * w - 1) * np.eye(3) - 2 * w * vx + 2 * np.outer(v, v)


def R_I_to_rect0(st, rp):
    """보정 초기값: IMU(센서) → rect0 회전 = R1 @ R_sensor_to_left (명세 4.2)."""
    return np.asarray(rp["R1"], float) @ np.asarray(st["R_sensor_to_left"], float)


def T_I_rect0_default(st, rp, p_rect0_in_I=(0.0, 0.0, 0.0)):
    """rect0 → IMU 변환 (보정값, 평행이동은 모름 = 0)."""
    R_C_I = R_I_to_rect0(st, rp)            # I → C
    return make_T(R_C_I.T, p_rect0_in_I)     # C → I


def T_world_I_from_ov(pose):
    """OpenVINS 상태 (q_GtoI JPL, p_IinG) → T_world_I. 없으면 None."""
    q, p = pose.get("q_GtoI_jpl"), pose.get("p_IinG", pose.get("p"))   # 브리지는 p_IinG 를 "p" 로 보냄
    if q is None or p is None:
        return None
    R_GtoI = jpl_quat_to_R(q)
    return make_T(R_GtoI.T, p)


def T_I_cam0_from_ov(pose):
    """브리지가 추정 외부 파라미터(R_ItoC0, p_IinC0)를 주면 cam0 → IMU 변환. 없으면 None."""
    R, p = pose.get("R_ItoC0"), pose.get("p_IinC0")
    if R is None or p is None:
        return None
    R_ItoC = np.asarray(R, float).reshape(3, 3)
    p_IinC = np.asarray(p, float).reshape(3)
    R_CtoI = R_ItoC.T
    return make_T(R_CtoI, -R_CtoI @ p_IinC)


def T_cam1_rect0(rp):
    """rect0 → cam1 (원래 렌즈 2) : p_cam1 = R1ᵀ p_rect0."""
    return make_T(np.asarray(rp["R1"], float).T, np.zeros(3))


VIO_CAM0_RECT0 = "rect0"     # VIO cam0 = 정류 왼쪽 (stereo 설정, 옛 mono 설정)
VIO_CAM0_LENS0U = "lens0u"   # VIO cam0 = 렌즈 2 왜곡만 편 핀홀, cam1 과 같은 축 (mono 설정 기본, calib_scale.vio_lens0_K)


def pose_cam0_to_rect0(msg, rp, cam0=VIO_CAM0_RECT0):
    """브리지 pose 의 cam0 기준 값을 rect0 기준으로 바꾼 사본 (cam0 가 rect0 이면 그대로).

    lens0u 는 cam1 과 같은 축이라 p_rect0 = R1 @ p_lens0u (같은 광학 중심):
      T_world_rect0 = T_world_lens0u @ T_cam1_rect0,  R_ItoC0(rect0) = R1 @ R_ItoC0(lens0u),  p_IinC0(rect0) = R1 @ p_IinC0.
    이렇게 해 두면 T_world_rect0()·motion_rect.T_c1_I() 는 cam0 = rect0 전제 그대로 쓴다."""
    if cam0 == VIO_CAM0_RECT0 or not msg:
        return msg
    if cam0 != VIO_CAM0_LENS0U:
        raise ValueError(f"모르는 VIO cam0 종류: {cam0!r}")
    out = dict(msg)
    R1 = np.asarray(rp["R1"], float)
    try:
        if msg.get("T_world_cam0") is not None:
            T = np.asarray(msg["T_world_cam0"], float).reshape(4, 4) @ T_cam1_rect0(rp)
            out["T_world_cam0"] = T.reshape(-1).tolist()
        if msg.get("R_ItoC0") is not None:
            out["R_ItoC0"] = (R1 @ np.asarray(msg["R_ItoC0"], float).reshape(3, 3)).reshape(-1).tolist()
        if msg.get("p_IinC0") is not None:
            out["p_IinC0"] = (R1 @ np.asarray(msg["p_IinC0"], float).reshape(3)).tolist()
    except (TypeError, ValueError):
        out["T_world_cam0"] = None
        out.pop("R_ItoC0", None)
        out.pop("p_IinC0", None)
    return out


def _finite_T(T):
    return T if T is not None and T.shape == (4, 4) and np.isfinite(T).all() else None


def T_world_rect0(pose, st, rp, cam0_is_rect=True):
    """VIO 출력 1개 → T_world_rect0 (4x4) 또는 None (NaN·null·형식 오류도 None, REVIEW 1).

    pose 의 cam0 는 rect0 기준이어야 한다. mono 설정의 cam0 = lens0u (왜곡만 편 렌즈 2) 이면 scan3d 가 먼저
    pose_cam0_to_rect0() 로 바꿔서 넘긴다 (2026-10-16).
    우선순위
    1) pose["T_world_cam0"] (브리지가 직접 계산).
    2) T_world_I (q_GtoI, p_IinG) × T_I_cam0 (브리지 추정 외부 파라미터 R_ItoC0/p_IinC0 = 설정과 같은 회전 출처).
    3) 그것도 없으면 보정 초기값 R1 @ R_sensor_to_left (설정의 Camera2 pose_rotation 과 조금 다를 수 있음, REVIEW 12).
    cam0_is_rect 는 호환용 인자 (False 를 주면 ValueError).
    """
    if not cam0_is_rect:
        raise ValueError("raw VIO 입력은 지원하지 않는다 (make_ov_config.py 는 정류 rect0 설정만 만든다)")
    if not pose or not pose.get("init") or pose.get("status") == "lost":
        return None                          # 초기화 전·추적 실패 포즈는 맵에 쓰지 않는다
    try:
        T = pose.get("T_world_cam0")
        if T is not None:
            return _finite_T(np.asarray(T, float).reshape(4, 4))
        T_wI = T_world_I_from_ov(pose)
        if T_wI is None:
            return None
        T_Ic = T_I_cam0_from_ov(pose)
        if T_Ic is None:
            return _finite_T(T_wI @ T_I_rect0_default(st, rp))
        return _finite_T(T_wI @ T_Ic)
    except (TypeError, ValueError):
        return None


# VIO 없이(--ov none) 깊이만 볼 때: rect0 를 z-up 월드로 눕혀 둔다 (x 오른쪽, y 앞, z 위).
T_WORLD_RECT0_STATIC = make_T(np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float), np.zeros(3))


def rect0_points(Z, rp, us, vs):
    """정류 왼쪽 화소 (us, vs) 와 깊이 Z → rect0 점 (N,3). p = [(u-cx)/f·Z, (v-cy)/f·Z, Z]."""
    f, cx, cy = rp["f"], rp["cx"], rp["cy"]
    return np.stack([(us - cx) / f * Z, (vs - cy) / f * Z, Z], axis=1).astype(np.float32)


def transform(T, pts):
    return (pts @ T[:3, :3].T.astype(np.float32)) + T[:3, 3].astype(np.float32)


def rot_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))
