"""모션 스테레오 (같은 렌즈 2, 시간차 두 프레임) + 두 렌즈 깊이와 합치기.

왜
- 렌즈 2·5 기선 15.5 mm → 깊이 오차 σ_Z = Z²·σ_d/(f·B) (σ_d ≈ 0.5 px, f ≈ 463): 1 m ±7 cm, 2 m ±28 cm.
- 폰을 움직이면 같은 렌즈 2 의 과거 프레임 j 와 지금 키프레임 i 사이에 수 cm~수십 cm 기선이 생긴다.
  VIO(mono, cam0 = lens0u = 렌즈 2 축) 자세로 짝을 고르고, 그 쌍만의 정류 → FFS → 깊이.

좌표 (frames.py 규약)
- cam1 = 렌즈 2 원래 카메라 (= lens0u 와 같은 축·같은 광학 중심). VIO T_world_cam0(lens0u) = T_world_cam1.
- rect0 = R1 · cam1 (기본 정류 왼쪽). 출력 점·깊이 격자는 모두 키프레임 i 의 rect0 (scan3d 맵 적분과 같은 기준).
- 모션 쌍 정류 좌표 "mrect" = Rm · cam1(i). Rm 의 x 축 = 기선 방향 (영상 x 성분이 양수가 되도록 부호 고름).
  rect0·mrect·쌍별 rect0' 는 모두 i 의 같은 광학 중심 → 회전 호모그래피로 깊이 격자를 시차 없이 옮긴다 (warp_depth).

정류 (lib_rectify 는 "2번 카메라가 오른쪽" 만 받으므로 직접 만든다)
- c = j 의 광학 중심 (i 좌표), B = |c|. e1 = ±c/B, e2 = ẑ×e1, e3 = e1×e2.
- e1 = −c/B (j 가 왼쪽) 이면 두 정류 영상을 좌우 뒤집어 FFS(뒤집은 i, 뒤집은 j) 후 시차를 다시 뒤집는다.
- 기선이 영상에서 비스듬하면 정류 영상이 돌아간다 (roll). 이 녹화에서 roll 중앙값 60° 인데도 FFS 결과는 두 렌즈와 맞았다.

자세 (실측, rec_20261016_143702)
- VIO 상대자세만으로 정류하면 대응점 세로 어긋남 |dy| 중앙값 5~14 px (초기화 직후) → FFS 깊이 쓰레기.
- SIFT 대응 + 기본행렬 E 로 R·기선 방향을 다시 구하면 |dy| 0.5~0.9 px. 기선 길이는 VIO.
- 하지만 E·VIO·자이로 적분 회전이 서로 0.3~1° 다르다 → 정류 뒤 '가로' 오차 = 시차 상수 치우침 δ (f=463 에서 1° ≈ 8 px).
  dy 검사로는 안 보인다. 그래서 두 렌즈 깊이로 쌍마다 시차를 맞춘다 (anchor):
    d_two = f·B/Z₂ (두 렌즈 깊이를 모션 시차로 바꾼 값) 를 모션 시차 d_m 에 대해 d_two = c·d_m + e 로 강건 회귀
    (Z₂ 가 믿을 만한 0.35~1.5 m 화소만, 잡음이 작은 d_m 을 설명 변수로 → 기울기 감쇠 없음).
    고친 시차 d' = c·d_m + e, Z = f·B/d'. c 는 VIO 기선 길이 오차, e 는 회전 오차를 흡수한다.
  즉 '크기·치우침은 두 렌즈 (보정된 15.5 mm), 정밀도(잡음)는 긴 기선' 으로 나눠 맡긴다.
  가까운 두 렌즈 깊이가 부족하면 (anchor 실패) 모션 깊이는 짝 둘이 서로 맞는 화소만 쓴다.

합치기 (fuse_depth)
- 화소마다 σ 로 가중 평균 (역분산). 차이가 k_sigma·√(σ₂²+σm²) 를 넘으면 그 화소를 버린다 (움직이는 물체·반사·오대응).
- 모션 깊이만 있는 화소: anchor 된 쌍이고 (두 쌍이 서로 맞거나 Z ≥ motion_only_min_z) 이면 남긴다.

scan3d 통합은 MotionStereo.add_frame (매 VIO 포즈) + MotionStereo.fuse_keyframe (키프레임) 두 곳 — 맨 아래 주석 참고.
"""
from collections import deque
from dataclasses import dataclass
import time

import cv2
import numpy as np

import frames

SIGMA_D_PX = 0.5          # FFS 시차 오차 (1 m ±7 cm, 2 m ±28 cm 에서 역산한 값, 두 렌즈·모션 공통 가정)


@dataclass
class MSConfig:
    # 짝 고르기
    dt_min_s: float = 0.15
    dt_max_s: float = 2.5
    b_min_m: float = 0.04
    b_max_m: float = 0.30
    b_target_m: float = 0.12
    # (55°, 15°, roll 40°) 이면 이 녹화 키프레임 12 % 만 짝이 있고 (40°, 25°, roll 90°) 이면 69 % (전진·회전 위주 손동작).
    # 정류 검사(dy ≤ 1.2 px)까지 통과하는 키프레임은 29/118 (25 %), 그중 anchor 성공 ~50 % (eval_motion_stereo, 2026-10-16).
    min_perp_deg: float = 40.0        # 기선과 광축 사이 각 하한 (90 = 완전 옆걸음, 0 = 전진)
    max_rot_deg: float = 25.0         # 두 프레임 회전 차이 상한 (겹침)
    max_roll_deg: float = 90.0        # 정류 영상 회전 상한 (90 = 제한 없음)
    n_pairs: int = 1                  # 2 = 두 쌍 일관성 검사 (FFS 2 회)
    pair2_min_dir_deg: float = 25.0   # 둘째 쌍은 첫 쌍과 기선 방향이 이만큼 달라야
    max_tries: int = 4                # 자세 검사에 떨어지면 다음 후보 (최대 몇 개)
    # 자세
    pose: str = "auto"                # auto = E 와 VIO 중 정류 검사(dy) 가 좋은 쪽, "e", "vio"
    e_min_inliers: int = 60
    e_min_inlier_frac: float = 0.35
    e_max_rot_deg: float = 3.0        # E 와 VIO 회전 차이 상한
    e_max_dir_deg: float = 45.0       # 기선 방향 차이 상한 (VIO 방향 오차 20~40° 구간이 있음)
    gate_dy_px: float = 1.2           # 정류 뒤 대응점 |dy| 중앙값 상한
    gate_dx_pos: float = 0.9          # 대응점 시차 양수 비율 하한
    sift_n: int = 1500
    # 정류·FFS
    f_scale: float = 1.0              # 모션 정류 f = rect0 f × f_scale
    ffs_max_disp: int = 192           # FFS 모델 max_disp (축소 입력 기준)
    ffs_scale_min: float = 0.35
    z_near_default: float = 0.5
    z_min: float = 0.3
    z_max: float = 5.0
    lr_check: bool = False            # FFS 를 반대 방향으로 한 번 더 (좌우 확인)
    lr_tol_px: float = 1.0
    # 두 렌즈로 시차 맞추기
    anchor: str = "affine"            # affine (c·d+e) | scale (c·d) | none
    anchor_z: tuple = (0.35, 1.5)
    anchor_min_px: int = 2000
    anchor_spread: float = 1.3        # p90/p10 (시차) 이 이보다 작으면 e 를 못 정함 → scale 만
    anchor_c: tuple = (0.6, 1.6)
    anchor_max_e_px: float = 25.0
    anchor_max_mad: float = 0.08      # 회귀 잔차 MAD / 중앙 시차 (두 렌즈가 엉망이거나 모션이 엉망이면 큼)
    # 오차 모델
    sigma_d_px: float = SIGMA_D_PX
    sigma_rot_deg_free: float = 0.5   # anchor 없을 때 회전 오차 (E·VIO·자이로 차이 0.3~1°, 실측)
    sigma_b_rel_free: float = 0.1
    # 합치기
    k_sigma: float = 3.0
    motion_only_min_z: float = 1.0
    keep_unverified: bool = True
    stride: int = 3
    history_s: float = 3.0


# ---------------------------------------------------------------- 기하
def rel_pose(T_w_ci, T_w_cj):
    """p_j = R p_i + t (i 좌표 점 → j 좌표)."""
    T = np.linalg.inv(T_w_cj) @ T_w_ci
    return T[:3, :3].copy(), T[:3, 3].copy()


def pair_geometry(R, t):
    """상대자세 → dict(B, c(i 좌표 j 중심), perp_deg(기선-광축 각, 90 = 옆), rot_deg, roll_deg, flip)."""
    c = -R.T @ t
    B = float(np.linalg.norm(c))
    if B < 1e-9:
        return dict(B=0.0, c=c, perp_deg=0.0, rot_deg=0.0, roll_deg=90.0, flip=False)
    u = c / B
    perp = float(np.degrees(np.arccos(np.clip(abs(u[2]), 0, 1))))
    rot = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    flip = bool(u[0] < 0)
    e = -u if flip else u
    roll = float(np.degrees(np.arctan2(e[1], e[0])))       # (-90, 90]
    return dict(B=B, c=c, perp_deg=perp, rot_deg=rot, roll_deg=roll, flip=flip)


def rect_rotation(R, t):
    """(Rm, flip, B): Rm = cam1(i) → mrect. e1 = 기선 (영상 x 양수 방향), e2 ≈ 아래, e3 ≈ 두 광축 평균."""
    g = pair_geometry(R, t)
    e1 = g["c"] / g["B"]
    if g["flip"]:
        e1 = -e1
    zm = np.array([0.0, 0.0, 1.0]) + R.T @ np.array([0.0, 0.0, 1.0])
    e2 = np.cross(zm, e1)
    e2 /= np.linalg.norm(e2)
    e3 = np.cross(e1, e2)
    return np.stack([e1, e2, e3]), g["flip"], g["B"]


def K_of(f, cx, cy):
    return np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]])


def rect0_K(rp):
    return K_of(rp["f"], rp["cx"], rp["cy"])


def rot_grid_map(K_src, R_src_from_dst, K_dst, size_dst):
    """회전만 다른 두 격자 (같은 광학 중심): dst 화소 → src 화소 맵 + src z 성분 zs.
    dst 광선 r=(x,y,1), r_src = R r. src 깊이 Z_src 인 점의 dst 깊이 = Z_src / zs."""
    W, H = size_dst
    u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    x = (u - K_dst[0, 2]) / K_dst[0, 0]
    y = (v - K_dst[1, 2]) / K_dst[1, 1]
    r = np.stack([x, y, np.ones_like(x)], -1) @ np.asarray(R_src_from_dst, float).T
    zs = r[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        mx = K_src[0, 0] * r[..., 0] / zs + K_src[0, 2]
        my = K_src[1, 1] * r[..., 1] / zs + K_src[1, 2]
    bad = zs <= 1e-6
    mx[bad] = -1
    my[bad] = -1
    return mx.astype(np.float32), my.astype(np.float32), zs.astype(np.float32)


def warp_depth(Z_src, valid_src, K_src, R_src_from_dst, K_dst, size_dst, extra=()):
    """깊이 격자 옮기기 (최근접, 회전만). 반환 (Z_dst (무효 NaN), valid_dst, [extra 맵 (최근접)])."""
    mx, my, zs = rot_grid_map(K_src, R_src_from_dst, K_dst, size_dst)
    Zs = np.where(valid_src, Z_src, 0).astype(np.float32)
    Zd = cv2.remap(Zs, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    with np.errstate(divide="ignore", invalid="ignore"):
        Zd = Zd / zs
    V = np.isfinite(Zd) & (Zd > 0)
    Zd = np.where(V, Zd, np.nan).astype(np.float32)
    outs = [cv2.remap(np.asarray(e, np.float32), mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT,
                      borderValue=-1) for e in extra]
    return Zd, V, outs


# ---------------------------------------------------------------- 짝 고르기
def score_pair(g, dt, cfg):
    """작을수록 좋음. 조건 밖이면 (None, 이유)."""
    if not (cfg.dt_min_s <= dt <= cfg.dt_max_s):
        return None, "dt"
    if not (cfg.b_min_m <= g["B"] <= cfg.b_max_m):
        return None, "baseline"
    if g["perp_deg"] < cfg.min_perp_deg:
        return None, "forward"
    if g["rot_deg"] > cfg.max_rot_deg:
        return None, "rotation"
    if abs(g["roll_deg"]) > cfg.max_roll_deg:
        return None, "roll"
    s = abs(np.log(g["B"] / cfg.b_target_m))
    s += 1.5 * (1 - np.sin(np.radians(g["perp_deg"])))
    s += 0.5 * g["rot_deg"] / cfg.max_rot_deg
    s += 0.3 * abs(g["roll_deg"]) / 90.0
    s += 0.2 * dt / cfg.dt_max_s
    return float(s), "ok"


def select_partners(T_w_ci, t_i_ns, history, cfg):
    """history: [dict(t_ns, T_w_c1, ...)] → ([(index, geom+dt, score)] 좋은 순, 거절 이유 집계)."""
    cands, why = [], {}
    for k, h in enumerate(history):
        dt = (t_i_ns - h["t_ns"]) * 1e-9
        R, t = rel_pose(T_w_ci, h["T_w_c1"])
        g = pair_geometry(R, t)
        s, w = score_pair(g, dt, cfg)
        why[w] = why.get(w, 0) + 1
        if s is not None:
            cands.append((s, k, dict(g, dt=dt)))
    cands.sort(key=lambda x: x[0])
    return [(k, g, s) for s, k, g in cands], why


def dir_differs(g0, g1, min_deg):
    u0, u1 = g0["c"] / g0["B"], g1["c"] / g1["B"]
    return np.degrees(np.arccos(np.clip(abs(u0 @ u1), 0, 1))) >= min_deg


# ---------------------------------------------------------------- 자세 (E) 와 정류 검사
_SIFT = {}


def sift_feats(gray, n):
    s = _SIFT.get(n)
    if s is None:
        s = _SIFT[n] = cv2.SIFT_create(n)
    kp, des = s.detectAndCompute(gray, None)
    return np.array([k.pt for k in kp], np.float32).reshape(-1, 2), des


def match_norm(feat_i, feat_j, K1, dist1):
    """SIFT 비율 검사 대응 → 정규화 좌표 (a: i, b: j) 또는 None."""
    (pi, di), (pj, dj) = feat_i, feat_j
    if di is None or dj is None or len(pi) < 20 or len(pj) < 20:
        return None
    m = [p[0] for p in cv2.BFMatcher(cv2.NORM_L2).knnMatch(di, dj, k=2) if len(p) == 2 and p[0].distance < 0.8 * p[1].distance]
    if len(m) < 20:
        return None
    a = cv2.undistortPoints(pi[[x.queryIdx for x in m]].reshape(-1, 1, 2), K1, dist1).reshape(-1, 2)
    b = cv2.undistortPoints(pj[[x.trainIdx for x in m]].reshape(-1, 1, 2), K1, dist1).reshape(-1, 2)
    return a, b


def pose_from_E(a, b, f, R_vio, t_vio, cfg):
    """(R, t (길이 = VIO), info) 또는 (None, None, info)."""
    info = dict(n_match=int(len(a)))
    E, msk = cv2.findEssentialMat(a, b, np.eye(3), cv2.RANSAC, 0.999, 1.0 / f)
    if E is None or E.shape != (3, 3):
        info["why"] = "E 실패"
        return None, None, info
    n_in, Re, te, _ = cv2.recoverPose(E, a, b, np.eye(3), mask=msk)
    te = te.reshape(3)
    info["n_in"] = int(n_in)
    if n_in < cfg.e_min_inliers or n_in < cfg.e_min_inlier_frac * len(a):
        info["why"] = f"인라이어 {n_in}/{len(a)}"
        return None, None, info
    B = float(np.linalg.norm(t_vio))
    info["e_rot_diff_deg"] = round(frames.rot_deg(Re, R_vio), 3)
    info["e_dir_diff_deg"] = round(float(np.degrees(np.arccos(np.clip(te @ (t_vio / B), -1, 1)))), 2)
    if info["e_rot_diff_deg"] > cfg.e_max_rot_deg or info["e_dir_diff_deg"] > cfg.e_max_dir_deg:
        info["why"] = "E 와 VIO 불일치"
        return None, None, info
    return Re, te * B, info


def rect_match_stats(a, b, R, t, f):
    """정규화 대응을 이 자세의 모션 정류로 옮겨 dy·dx 통계 (정류 검사 + 가까운 쪽 시차)."""
    Rm, flip, _ = rect_rotation(R, t)
    pi = np.c_[a, np.ones(len(a))] @ Rm.T
    pj = np.c_[b, np.ones(len(b))] @ (Rm @ R.T).T
    ui = f * pi[:, :2] / pi[:, 2:]
    uj = f * pj[:, :2] / pj[:, 2:]
    dy = ui[:, 1] - uj[:, 1]
    dx = ui[:, 0] - uj[:, 0]
    if flip:
        dx = -dx
    ok = np.abs(dy) < 3
    return dict(n=int(len(a)), dy_med=float(np.median(np.abs(dy))), dy_ok_frac=float(ok.mean()),
                dx_pos_frac=float((dx[ok] > 0).mean()) if ok.any() else 0.0,
                dx_p98=float(np.percentile(dx[ok], 98)) if ok.sum() > 5 else None)


def gate_ok(gs, cfg):
    return gs is not None and gs["dy_med"] <= cfg.gate_dy_px and gs["dx_pos_frac"] >= cfg.gate_dx_pos


# ---------------------------------------------------------------- 두 렌즈로 시차 맞추기
def fit_anchor(d_m, ok, Z2m, V2m, f, B, cfg):
    """d_two = f·B/Z₂ ≈ c·d_m + e (강건, Huber IRLS). 반환 dict(c, e, mode, n, mad_rel, ok) 또는 None (표본 부족)."""
    lo, hi = cfg.anchor_z
    with np.errstate(invalid="ignore"):
        m = ok & V2m & (Z2m >= lo) & (Z2m <= hi)
    n = int(m.sum())
    if n < cfg.anchor_min_px:
        return dict(ok=False, n=n, why="가까운 두 렌즈 화소 부족")
    y = (f * B / Z2m[m]).astype(np.float64)           # 두 렌즈 깊이 → 모션 시차 단위 (잡음 큼 → 종속 변수)
    x = d_m[m].astype(np.float64)                      # 모션 시차 (잡음 작음 → 설명 변수)
    if n > 40000:
        sel = np.random.default_rng(0).choice(n, 40000, replace=False)
        x, y = x[sel], y[sel]
    spread = float(np.percentile(x, 90) / max(np.percentile(x, 10), 1e-6))
    mode = "affine" if cfg.anchor == "affine" and spread >= cfg.anchor_spread else "scale"
    w = np.ones_like(x)
    c, e = 1.0, 0.0
    for _ in range(8):
        if mode == "affine":
            A = np.c_[x, np.ones_like(x)] * np.sqrt(w)[:, None]
            c, e = np.linalg.lstsq(A, y * np.sqrt(w), rcond=None)[0]
        else:
            c, e = float(np.sum(w * x * y) / max(np.sum(w * x * x), 1e-12)), 0.0
        r = y - (c * x + e)
        s = 1.4826 * np.median(np.abs(r)) + 1e-3
        w = np.minimum(1.0, 1.5 * s / np.maximum(np.abs(r), 1e-9))
    mad_rel = float(1.4826 * np.median(np.abs(r)) / max(np.median(y), 1e-6))
    res = dict(c=float(c), e=float(e), mode=mode, n=n, spread=round(spread, 3), mad_rel=round(mad_rel, 4))
    good = cfg.anchor_c[0] <= c <= cfg.anchor_c[1] and abs(e) <= cfg.anchor_max_e_px and mad_rel <= cfg.anchor_max_mad
    res["ok"] = bool(good)
    if not good:
        res["why"] = "맞춤 값이 범위 밖 / 잔차 큼"
    return res


# ---------------------------------------------------------------- 한 쌍의 깊이
def _uncertain(disp, rel=0.15, abs_px=1.0, k=5):
    d = np.nan_to_num(disp, nan=0.0).astype(np.float32)
    ker = np.ones((k, k), np.uint8)
    return (cv2.dilate(d, ker) - cv2.erode(d, ker)) > np.maximum(abs_px, rel * d)


def pair_depth(model, infer, bgr_i, bgr_j, R, t, K1, dist1, rp, cfg, Z2=None, V2=None, z_near=None, iters=8,
               d_hint=None):
    """두 원본 렌즈 2 영상 + 상대자세 (p_j = R p_i + t) → 키프레임 i 의 rect0 격자 깊이.

    Z2, V2: 두 렌즈 깊이 (기본 rect0 격자) — 있으면 시차 맞추기 (anchor). infer = lib_ffs.infer.
    반환 dict(Z, valid, sigma (rect0 격자), anchor, B, f, scale, flip, roll_deg, ms ...)."""
    t0 = time.perf_counter()
    Rm, flip, B = rect_rotation(R, t)
    g = pair_geometry(R, t)
    W0, H0 = (int(x) for x in rp["size"])
    f = float(rp["f"]) * cfg.f_scale
    W, H = (H0, W0) if abs(g["roll_deg"]) > 45 else (W0, H0)
    R1 = np.asarray(rp["R1"], float)
    a = Rm @ R1.T @ np.array([0.0, 0.0, 1.0])          # rect0 광축을 mrect 격자 가운데로
    Km = K_of(f, W / 2 - f * a[0] / a[2], H / 2 - f * a[1] / a[2])
    mi = cv2.initUndistortRectifyMap(K1, dist1, Rm, Km, (W, H), cv2.CV_32FC1)
    mj = cv2.initUndistortRectifyMap(K1, dist1, Rm @ R.T, Km, (W, H), cv2.CV_32FC1)
    ri = cv2.remap(bgr_i, *mi, cv2.INTER_LINEAR)
    rj = cv2.remap(bgr_j, *mj, cv2.INTER_LINEAR)
    one = np.full(bgr_i.shape[:2], 255, np.uint8)
    vi = cv2.remap(one, *mi, cv2.INTER_NEAREST, borderValue=0) > 0
    vj = cv2.remap(one, *mj, cv2.INTER_NEAREST, borderValue=0) > 0
    t1 = time.perf_counter()
    zn = float(z_near or cfg.z_near_default)
    d_need = f * B / max(zn, 0.05)
    if d_hint:
        d_need = max(d_need if z_near else 0.0, 1.15 * d_hint * cfg.f_scale)
    s = round(float(np.clip(0.92 * cfg.ffs_max_disp / max(d_need, 1.0), cfg.ffs_scale_min, 1.0)), 3)
    d_cap = 0.95 * cfg.ffs_max_disp / s
    L, Rr = (ri[:, ::-1], rj[:, ::-1]) if flip else (ri, rj)
    rgbL = cv2.cvtColor(np.ascontiguousarray(L), cv2.COLOR_BGR2RGB)
    rgbR = cv2.cvtColor(np.ascontiguousarray(Rr), cv2.COLOR_BGR2RGB)
    disp = infer(model, rgbL, rgbR, valid_iters=iters, scale=s)
    dR = None
    if cfg.lr_check:                                       # 오른쪽(j) 기준 시차: 뒤집어서 j 를 왼쪽으로
        dR = infer(model, np.ascontiguousarray(rgbR[:, ::-1]), np.ascontiguousarray(rgbL[:, ::-1]),
                   valid_iters=iters, scale=s)[:, ::-1]
    if flip:
        disp = disp[:, ::-1]
        if dR is not None:
            dR = dR[:, ::-1]
    t2 = time.perf_counter()
    sgn = -1.0 if flip else 1.0                            # j 화소 u_j = u − sgn·d
    u = np.arange(W, dtype=np.float32)[None, :]
    vv = np.repeat(np.arange(H, dtype=np.int32)[:, None], W, 1)
    uj = u - sgn * disp
    uji = np.clip(np.round(uj).astype(np.int32), 0, W - 1)
    ok = np.isfinite(disp) & (disp > 0.5) & (disp < d_cap) & (uj >= 0) & (uj <= W - 1) & vi
    ok &= vj[vv, uji]
    lr_frac = None
    if dR is not None:
        lr = np.abs(dR[vv, uji] - disp) < cfg.lr_tol_px
        lr_frac = float(lr[ok].mean()) if ok.any() else None
        ok &= lr
    ok &= ~_uncertain(disp)
    R_m_from_0 = Rm @ R1.T                                 # p_mrect = Rm R1ᵀ p_rect0
    anc = dict(ok=False, why="두 렌즈 깊이 없음")
    dcor = disp
    if Z2 is not None and cfg.anchor != "none":
        Z2m, V2m, _ = warp_depth(Z2, V2, rect0_K(rp), R_m_from_0.T, Km, (W, H))
        anc = fit_anchor(disp, ok, Z2m, V2m, f, B, cfg)
        if anc.get("ok"):
            dcor = anc["c"] * disp + anc["e"]
    with np.errstate(divide="ignore", invalid="ignore"):
        Zm = (f * B / dcor).astype(np.float32)
    ok &= np.isfinite(Zm) & (dcor > 0.5) & (Zm >= cfg.z_min) & (Zm <= cfg.z_max)
    Z0, V0, _ = warp_depth(Zm, ok, Km, R_m_from_0, rect0_K(rp), (W0, H0))
    # 오차: anchor 되면 시차 잡음 (FFS + 맞춤 잔차), 아니면 회전·기선 오차 포함
    Zs = np.nan_to_num(Z0, nan=1.0)
    Beff = B * (anc["c"] if anc.get("ok") else 1.0)
    if anc.get("ok"):
        # 맞춤 불확실성: 두 렌즈 잡음은 공간적으로 이어져 있어 유효 표본 ≈ 50 개로 봄 (추정)
        sd = np.hypot(cfg.sigma_d_px, anc["mad_rel"] * float(np.median(dcor[ok])) / np.sqrt(50) if ok.any() else 0.0)
        sig = Zs * Zs * sd / (f * Beff)
    else:
        sdp = np.hypot(cfg.sigma_d_px, f * np.radians(cfg.sigma_rot_deg_free))
        sig = np.hypot(Zs * Zs * sdp / (f * B), Zs * cfg.sigma_b_rel_free)
    t3 = time.perf_counter()
    return dict(Z=Z0, valid=V0, sigma=sig.astype(np.float32), anchor=anc, B=B, f=f, Km=Km, Rm=Rm, scale=s, flip=flip,
                roll_deg=g["roll_deg"], perp_deg=g["perp_deg"], rot_deg=g["rot_deg"], d_cap=float(d_cap), z_near=zn,
                rect_mrect=ri, lr_frac=lr_frac,
                ms=dict(rect=1000 * (t1 - t0), ffs=1000 * (t2 - t1), post=1000 * (t3 - t2)))


def sigma_two_lens(Z, f, B=0.0155, sd=SIGMA_D_PX):
    return Z * Z * sd / (f * B)


# ---------------------------------------------------------------- 합치기
def combine_motion(results, cfg):
    """여러 모션 쌍 (rect0 격자) → (Z, valid, sigma, verified). 둘 이상이면 서로 맞는 화소가 verified, 어긋나면 버림."""
    if not results:
        return None
    if len(results) == 1:
        r = results[0]
        return r["Z"], r["valid"], r["sigma"], np.zeros_like(r["valid"])
    shape = results[0]["Z"].shape
    num, den = np.zeros(shape), np.zeros(shape)
    nv = np.zeros(shape, np.int32)
    for r in results:
        w = np.where(r["valid"], 1.0 / np.maximum(r["sigma"], 1e-6) ** 2, 0)
        num += np.where(r["valid"], r["Z"], 0) * w
        den += w
        nv += r["valid"]
    with np.errstate(invalid="ignore", divide="ignore"):
        Zw = np.where(den > 0, num / den, np.nan)
        Sw = np.where(den > 0, 1.0 / np.sqrt(den), np.inf)
        agree = nv >= 2
        for r in results:
            agree &= ~(r["valid"] & (nv >= 2) & (np.abs(r["Z"] - Zw) > cfg.k_sigma * np.hypot(r["sigma"], Sw)))
    V = (nv >= 1) & ~((nv >= 2) & ~agree)
    return Zw.astype(np.float32), V, Sw.astype(np.float32), agree


def fuse_depth(Z2, V2, S2, Zm, Vm, Sm, verified, anchored, cfg):
    """두 렌즈 (Z2) + 모션 (Zm), 같은 rect0 격자. 반환 (Z, valid, src, 집계). src: 0 없음, 1 두 렌즈, 2 모션, 3 가중 평균."""
    Z2 = np.where(V2, Z2, np.nan)
    Zm = np.where(Vm, Zm, np.nan)
    both = V2 & Vm
    with np.errstate(invalid="ignore"):
        agree = both & (np.abs(Z2 - Zm) <= cfg.k_sigma * np.hypot(S2, Sm))
    w2 = 1.0 / np.maximum(S2, 1e-6) ** 2
    wm = 1.0 / np.maximum(Sm, 1e-6) ** 2
    Zf = np.full(Z2.shape, np.nan, np.float32)
    src = np.zeros(Z2.shape, np.uint8)
    Zf[agree] = ((Z2 * w2 + Zm * wm) / (w2 + wm))[agree]
    src[agree] = 3
    only2 = V2 & ~Vm
    Zf[only2] = Z2[only2]
    src[only2] = 1
    with np.errstate(invalid="ignore"):
        keep_m = verified | (anchored & cfg.keep_unverified & (Zm >= cfg.motion_only_min_z))
    onlym = Vm & ~V2 & keep_m
    Zf[onlym] = Zm[onlym]
    src[onlym] = 2
    return Zf, src > 0, src, dict(both=int(both.sum()), agree=int(agree.sum()), conflict=int((both & ~agree).sum()),
                                  only2=int(only2.sum()), onlym=int(onlym.sum()))


# ---------------------------------------------------------------- 상태 있는 래퍼 (scan3d 용)
class MotionStereo:
    """VIO 포즈가 붙은 렌즈 2 프레임을 모아 두고 키프레임마다 모션 스테레오 깊이 + 두 렌즈 깊이 합치기.

    st, rp = calib_scale.load_geometry(...)["st"], ["rp"] (기본 정류). model = DepthSeg.model (FFS 객체 공유).
    fuse_keyframe 은 깊이 워커 스레드에서만 (FFS 호출). add_frame 은 다른 스레드에서 불러도 됨 (deque append)."""

    def __init__(self, st, rp, model, cfg=None, iters=8, infer=None, log=print):
        self.st, self.rp, self.model, self.iters, self.log = st, rp, model, iters, log
        self.cfg = cfg or MSConfig()
        self.K1 = np.asarray(st["K1"], float)
        self.dist1 = np.asarray(st["dist1"], float)
        if infer is None:
            import lib_ffs
            infer = lib_ffs.infer
        self.infer = infer
        self.hist = deque()
        self.T_r0_c1 = np.linalg.inv(frames.T_cam1_rect0(rp))
        self.stats = dict(kf=0, with_pair=0, used_E=0, used_vio=0, anchored=0, why={}, rejects=[])
        self._last = None

    def reset(self):
        self.hist.clear()

    def add_frame(self, t_ns, bgr_lens2, T_world_rect0):
        """매 VIO 포즈 (추적 중) 마다. T_world_rect0 = frames.T_world_rect0(...) (scan3d._on_pose 의 T)."""
        if T_world_rect0 is None:
            return
        T_w_c1 = np.asarray(T_world_rect0, float) @ self.T_r0_c1
        self.hist.append(dict(t_ns=int(t_ns), bgr=bgr_lens2, T_w_c1=T_w_c1, feat=None))
        while self.hist and (t_ns - self.hist[0]["t_ns"]) * 1e-9 > self.cfg.history_s:
            self.hist.popleft()

    def _feat(self, h):
        if h.get("feat") is None:
            h["feat"] = sift_feats(cv2.cvtColor(h["bgr"], cv2.COLOR_BGR2GRAY), self.cfg.sift_n)
        return h["feat"]

    def _choose_pose(self, fi, h, R, t):
        """(R, t, info, gate) 또는 (None, ...) — E 와 VIO 중 정류 검사를 통과하고 dy 가 작은 쪽."""
        cfg = self.cfg
        ab = match_norm(fi, self._feat(h), self.K1, self.dist1)
        if ab is None:
            return None, None, dict(why="대응 부족"), None
        a, b = ab
        f = float(self.rp["f"]) * cfg.f_scale
        opts = []
        info = {}
        if cfg.pose in ("auto", "e"):
            Re, te, info = pose_from_E(a, b, 0.5 * (self.K1[0, 0] + self.K1[1, 1]), R, t, cfg)
            if Re is not None:
                opts.append(("E", Re, te))
        if cfg.pose in ("auto", "vio"):
            opts.append(("vio", R, t))
        best = None
        for name, RR, tt in opts:
            gs = rect_match_stats(a, b, RR, tt, f)
            gs["pose"] = name
            if gate_ok(gs, cfg) and (best is None or gs["dy_med"] < best[3]["dy_med"]):
                best = (RR, tt, dict(info, used=name), gs)
            info[f"dy_{name}"] = round(gs["dy_med"], 2)
        if best is None:
            return None, None, dict(info, why="정류 검사 실패 " + str({k: v for k, v in info.items() if k.startswith('dy_')})), None
        return best

    def motion_depth(self, t_ns, bgr_i, T_world_rect0, Z2=None, V2=None, z_near=None):
        """키프레임 i 의 모션 깊이 (rect0 격자). 반환 dict(pairs=[pair_depth 결과 + dt·e·gate], ...) 또는 None."""
        cfg = self.cfg
        T_w_ci = np.asarray(T_world_rect0, float) @ self.T_r0_c1
        hist = [h for h in self.hist if h["t_ns"] < t_ns]
        sel, why = select_partners(T_w_ci, t_ns, hist, cfg)
        self.stats["kf"] += 1
        for k, v in why.items():
            self.stats["why"][k] = self.stats["why"].get(k, 0) + v
        if not sel:
            return None
        t0 = time.perf_counter()
        fi = sift_feats(cv2.cvtColor(bgr_i, cv2.COLOR_BGR2GRAY), cfg.sift_n)
        res, tried, rejects, ms_pose = [], 0, [], 0.0
        for k, g, s in sel:
            if len(res) >= cfg.n_pairs or tried >= cfg.max_tries:
                break
            if res and not dir_differs(res[0]["geom"], g, cfg.pair2_min_dir_deg):
                continue
            tried += 1
            h = hist[k]
            R, t = rel_pose(T_w_ci, h["T_w_c1"])
            tp = time.perf_counter()
            R, t, einfo, gate = self._choose_pose(fi, h, R, t)
            ms_pose += 1000 * (time.perf_counter() - tp)
            if R is None:
                rejects.append(einfo.get("why"))
                continue
            r = pair_depth(self.model, self.infer, bgr_i, h["bgr"], R, t, self.K1, self.dist1, self.rp, cfg,
                           Z2=Z2, V2=V2, z_near=z_near, iters=self.iters, d_hint=gate.get("dx_p98"))
            r.update(dt=g["dt"], score=s, e=einfo, gate=gate, t_j_ns=h["t_ns"], geom=pair_geometry(R, t))
            self.stats["used_E" if einfo["used"] == "E" else "used_vio"] += 1
            self.stats["anchored"] += bool(r["anchor"].get("ok"))
            res.append(r)
        self.stats["rejects"] = (self.stats["rejects"] + rejects)[-20:]
        if not res:
            return None
        self.stats["with_pair"] += 1
        return dict(pairs=res, tried=tried, rejects=rejects, ms_pose=ms_pose,
                    ms_all=1000 * (time.perf_counter() - t0))

    def fuse_keyframe(self, t_ns, bgr_lens2, T_world_rect0, r2=None, rp_pair=None, R_fix=None):
        """키프레임 하나: 모션 깊이 + 두 렌즈 깊이 (DepthSeg 결과 r2) 합치기 → DepthSeg 와 같은 꼴의 dict.

        r2      : DepthSeg.__call__ 결과 (Z, valid, dets … rp_pair 격자) 또는 None (모션 깊이만).
        rp_pair : r2 를 만든 정류 (scan3d._pair_rect 의 rp), R_fix: 같은 함수의 R_rect0_from_pair (None = 기본 정류).
        반환 dict(pts_rect0 (기본 rect0 — R_fix 이미 반영), rgb, cls, det_idx, dets, n, Z, valid, src, rectL, info)."""
        cfg, rp = self.cfg, self.rp
        t0 = time.perf_counter()
        W, H = (int(x) for x in rp["size"])
        K0 = rect0_K(rp)
        info = {}
        det_map = np.full((H, W), -1, np.int32)
        if r2 is not None:
            rpp = rp_pair or rp
            for k, d in enumerate(r2.get("dets") or []):
                if d.get("mask") is not None:
                    det_map[d["mask"] > 0] = k
            if R_fix is not None:                                      # p_rect0' = R_fixᵀ p_rect0
                Z2, V2, (dm,) = warp_depth(r2["Z"], r2["valid"], rect0_K(rpp), np.asarray(R_fix, float).T, K0, (W, H),
                                           extra=(det_map.astype(np.float32),))
                det_map = np.where(V2, dm, -1).astype(np.int32)
            else:
                V2 = r2["valid"].copy()
                Z2 = np.where(V2, r2["Z"], np.nan).astype(np.float32)
            S2 = sigma_two_lens(np.nan_to_num(Z2, nan=1.0), rp["f"], float(rpp.get("baseline_m", 0.0155)), cfg.sigma_d_px)
            zv = Z2[V2]
            z_near = float(np.percentile(zv, 2)) if zv.size > 500 else None
        else:
            Z2 = np.full((H, W), np.nan, np.float32)
            V2 = np.zeros((H, W), bool)
            S2 = np.full((H, W), np.inf, np.float32)
            z_near = None
        md = self.motion_depth(t_ns, bgr_lens2, T_world_rect0, Z2=Z2 if r2 is not None else None, V2=V2,
                               z_near=z_near)
        self._last = md
        if md is None:
            Zf, Vf, src = Z2, V2, np.where(V2, 1, 0).astype(np.uint8)
            info.update(pairs=0)
        else:
            Zm, Vm, Sm, ver = combine_motion(md["pairs"], cfg)
            anchored = any(p["anchor"].get("ok") for p in md["pairs"])
            Zf, Vf, src, fst = fuse_depth(Z2, V2, S2, Zm, Vm, Sm, ver, anchored, cfg)
            info.update(pairs=len(md["pairs"]), fuse=fst, anchored=anchored,
                        B=[round(p["B"], 4) for p in md["pairs"]], dt=[round(p["dt"], 3) for p in md["pairs"]],
                        pose=[p["e"].get("used") for p in md["pairs"]], ffs_scale=[p["scale"] for p in md["pairs"]],
                        anchor=[{k: v for k, v in p["anchor"].items()} for p in md["pairs"]],
                        ms_ffs=sum(p["ms"]["ffs"] for p in md["pairs"]), ms_pose=md["ms_pose"], ms_motion=md["ms_all"])
            md.update(Zm=Zm, Vm=Vm, Sm=Sm, verified=ver)
        # 점 뽑기 (DepthSeg 와 같은 규칙: stride 격자, 물체 마스크 안은 두 배 촘촘, 마스크별 깊이 ±20 % 밖은 배경)
        rect0 = cv2.remap(bgr_lens2, *rp["mapL"], cv2.INTER_LINEAR)
        s = cfg.stride
        sub = np.zeros((H, W), bool); sub[::s, ::s] = True
        s2 = max(1, s // 2)
        sub2 = np.zeros((H, W), bool); sub2[::s2, ::s2] = True
        take = Vf & (sub | (sub2 & (det_map >= 0)))
        vs, us = np.nonzero(take)
        di = det_map[vs, us].copy()
        Zs = Zf[vs, us]
        for k in np.unique(di[di >= 0]):
            sel = di == k
            med = np.median(Zs[sel])
            di[sel & (np.abs(Zs - med) > 0.2 * med)] = -1
        dets = (r2.get("dets") if r2 is not None else None) or []
        cls = np.where(di >= 0, np.array([d["class_id"] for d in dets] + [-1], np.int16)[di], -1).astype(np.int16)
        pts = frames.rect0_points(Zs, rp, us.astype(np.float32), vs.astype(np.float32))
        rgb = rect0[vs, us][:, ::-1].copy()
        info["ms_total"] = 1000 * (time.perf_counter() - t0)
        return dict(pts_rect0=pts, rgb=rgb, cls=cls, det_idx=di, dets=dets, n=len(pts), Z=Zf, valid=Vf, src=src,
                    rectL=rect0, Z2=Z2, V2=V2, info=info)


# ---------------------------------------------------------------- scan3d 통합 (이 파일 밖은 고치지 않음 — 통합 담당이 넣을 것)
# 1) _depth_worker: self.ds = depth_seg.DepthSeg(...) 직후
#        import motion_stereo
#        self.ms = motion_stereo.MotionStereo(self.geom["st"], self.geom["rp"], self.ds.model, iters=self.ds.iters, log=self.log)
#    apply_geometry (해상도 바뀜) 에서 새로 만들고, on_vio_event (VIO 리셋) 에서 self.ms.reset().
#    iters 를 바꾸면 self.ms.model = self.ds.model; self.ms.iters = self.ds.iters.
# 2) _on_pose: T = frames.T_world_rect0(...) 구한 뒤 (stale 아님, mode != "static"):
#        got = self._pair_for(t_ns)
#        if got is not None and getattr(self, "ms", None): self.ms.add_frame(t_ns, got[0][0].bgr, T)
#    (키프레임이 아닌 포즈도 매번. 프레임 참조만 보관: 3 s ≈ 45 장 × 0.9 MB.)
# 3) _process_keyframe: r = self.ds(fL.bgr, fR.bgr, rp=rp, scale=...) 다음
#        out = self.ms.fuse_keyframe(mot["t0_ns"], fL.bgr, T, r2=r, rp_pair=rp, R_fix=R_fix)
#        pts = out["pts_rect0"]                 # R_fix 이미 반영 → 'if R_fix is not None: pts = pts @ R_fix.T' 는 건너뜀
#        pts_w = frames.transform(T, pts)
#        r["det_idx"], r["rgb"], r["cls"] 대신 out["det_idx"], out["rgb"], out["cls"]  (dets 목록은 r 과 같음)
#        패널: rect=out["rectL"], Z=out["Z"], valid=out["valid"]
#    두 렌즈 깊이를 못 만든 키프레임 (rp None) 은 지금처럼 건너뛴다 (anchor 가 없으면 모션 깊이 크기·치우침을 못 믿음).
