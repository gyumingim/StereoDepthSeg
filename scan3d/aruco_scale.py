"""ChArUco 보드로 축척 확인·보정 (깊이 시차 오프셋/비례, 두 렌즈 외부 파라미터 잔차, VIO 축척).

왜 필요한가
- 실측: 3분 사이 FFS 시차가 전체적으로 +0.08~0.2 px 밀려 깊이가 약 1.5 % 변함 (열로 추정). 지도 축척은 검증 안 됨.
- 벽에 크기를 자로 잰 ChArUco 보드를 붙여 두면, 보드가 보이는 키프레임마다 "정답 깊이"가 생긴다.

원리 (키프레임 1장, 정류 왼쪽 rect0 기준)
1) rect0 (회색) 에서 ChArUco 모서리 검출 → solvePnP (정류 K = rp f, cx, cy, 왜곡 없음) → 보드 자세 T_cam_board.
2) 보드 평면 위 촘촘한 점(칸의 1/4 간격)을 그 자세로 투영 → 화소 (u, v) 와 그 점의 보드 기준 깊이 Z_board.
   d_true = f·B / Z_board (B = rp baseline_m).  d_ffs = 그 화소의 FFS 시차 (양선형 보간).
   참고: Z_board ≈ f·L/l_px (L 보드 실제 크기, l_px 화소 크기) 이므로 d_true ≈ B·l_px/L — 초점거리 f 오차와 거의 무관.
   즉 이 확인이 믿는 것은 "보드 실제 크기(자로 잰 값)"와 "기선 B" 둘이다.
3) 키프레임마다 (d_true, d_ffs) 요약을 쌓고, 시간 감쇠 가중(= EWMA, 기본 τ=60 s)으로 강건 회귀
      d_ffs = a·d_true + b    (보드를 여러 거리에서 봐서 d_true 범위가 1.4배 이상일 때)
      d_ffs = d_true + b      (한 거리뿐이면 오프셋만, a = 1 고정)
   보정: d_corr = s·d_ffs + Δd,  s = 1/a,  Δd = −b/a.   깊이 Z_corr = f·B / d_corr.
4) (정보) rect1 에서도 보드가 보이면 같은 id 모서리끼리 비교 → 모서리 시차 u0−u1 vs d_true (= 외부 파라미터 yaw 잔차),
   세로 어긋남 v1−v0 (pitch·roll 잔차). FFS 와 무관한 기하 확인이라 "FFS 탓인지 렌즈 자세 탓인지" 가를 수 있다.
5) (정보) VIO 축척: 보드 자세를 두 키프레임에서 얻으면 그 사이 카메라 이동 거리(보드 기준) vs VIO 이동 거리 비.
6) (정보) 위치 기준점: 보드의 월드 위치 T_world_board 를 쌓아 처음 본 위치에서 얼마나 떠내려갔는지 (VIO 드리프트 지표).

이 모듈은 FFS·scan3d 를 import 하지 않는다 (cv2, numpy 만). 통합 방법은 scan3d/ARUCO_SCALE.md "통합".
OpenCV 4.5 (옛 aruco API) 와 4.7+/5.x (CharucoDetector) 둘 다 지원. 보드 행 수는 홀수로 (4.6 에서 짝수 행 무늬가 바뀜).
"""
import json
import math
import threading
from collections import deque
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------- 보드 명세

BOARD_DEFAULT = dict(dict="DICT_4X4_50", squares_x=7, squares_y=5, square_mm_nominal=34.0, marker_ratio=0.75,
                     measured_width_mm=None, measured_height_mm=None, square_mm=None)


def load_board_spec(path=None, **override):
    """보드 json (make_charuco.py 가 만든 board_*.json) → dict + square_m, marker_m, measured(bool).

    칸 크기 우선순위: square_mm (직접 입력) > measured_width_mm/squares_x 와 measured_height_mm/squares_y 평균
    > square_mm_nominal (인쇄 의도 값, measured=False — 축척 확인용으로는 믿지 말 것)."""
    spec = dict(BOARD_DEFAULT)
    if path:
        spec.update(json.loads(Path(path).read_text()))
    spec.update({k: v for k, v in override.items() if v is not None})
    sx, sy = int(spec["squares_x"]), int(spec["squares_y"])
    sq, src = None, "nominal"
    if spec.get("square_mm"):
        sq, src = float(spec["square_mm"]), "square_mm"
    else:
        vals = []
        if spec.get("measured_width_mm"):
            vals.append(float(spec["measured_width_mm"]) / sx)
        if spec.get("measured_height_mm"):
            vals.append(float(spec["measured_height_mm"]) / sy)
        if vals:
            sq, src = float(np.mean(vals)), "measured"
    if sq is None:
        sq = float(spec["square_mm_nominal"])
    ratio = float(spec.get("marker_ratio") or 0.75)
    spec.update(squares_x=sx, squares_y=sy, square_m=sq / 1000.0, marker_m=ratio * sq / 1000.0,
                square_src=src, measured=src != "nominal")
    return spec


def _dict_of(name):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


NEW_API = hasattr(cv2.aruco, "CharucoDetector")


class Board:
    """ChArUco 보드 + 검출기 (OpenCV 버전 차이를 숨김)."""

    def __init__(self, spec):
        self.spec = spec
        self.sx, self.sy = spec["squares_x"], spec["squares_y"]
        self.square_m, self.marker_m = spec["square_m"], spec["marker_m"]
        self.dictionary = _dict_of(spec["dict"])
        if NEW_API:
            ids_start = int(spec.get("ids_start") or 0)       # different marker ids per board → the video tells which board
            if ids_start:
                n_mk = (self.sx * self.sy) // 2
                self.board = cv2.aruco.CharucoBoard((self.sx, self.sy), self.square_m, self.marker_m, self.dictionary,
                                                    np.arange(ids_start, ids_start + n_mk, dtype=np.int32))
            else:
                self.board = cv2.aruco.CharucoBoard((self.sx, self.sy), self.square_m, self.marker_m, self.dictionary)
            # 짝수 행: 4.5 로 인쇄한 보드와 같은 무늬 (기본). 새 OpenCV 로 만든 보드 그림은 board json 에
            # "legacy_pattern": false 를 넣는다 (예: board_laptop_6x4.json — legacy 로는 모서리 0 개, 2026-10-23).
            legacy = spec.get("legacy_pattern")
            if legacy is None:
                legacy = self.sy % 2 == 0
            if legacy and hasattr(self.board, "setLegacyPattern"):
                self.board.setLegacyPattern(True)
            cp = cv2.aruco.CharucoParameters()
            cp.minMarkers = 1
            cp.tryRefineMarkers = True                   # 놓친 마커를 보드 배치로 다시 찾음
            dp = cv2.aruco.DetectorParameters()
            dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            # 작은 마커 (1440 폭에서 기본 0.03 = 둘레 약 43 px 미만은 버림) 도 받기. rec_20261018_002349 실측:
            # 두 렌즈 모두 모서리 8개 이상 97 -> 105 쌍 (+8 %), 검출 16 -> 24 ms/장. 창 크기를 넓히면 108 쌍인데 40 ms 라 안 씀.
            dp.minMarkerPerimeterRate = 0.01
            self.detector = cv2.aruco.CharucoDetector(self.board, cp, dp, cv2.aruco.RefineParameters())
            corners = self.board.getChessboardCorners()
        else:
            self.board = cv2.aruco.CharucoBoard_create(self.sx, self.sy, self.square_m, self.marker_m, self.dictionary)
            self.params = cv2.aruco.DetectorParameters_create()
            self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            corners = self.board.chessboardCorners
        self.obj = np.asarray(corners, np.float64).reshape(-1, 3)   # 모서리 id → 보드 좌표 (m), z = 0
        self.n_corners = len(self.obj)
        self.size_m = (self.sx * self.square_m, self.sy * self.square_m)

    def image(self, w_px, h_px, margin_px=0):
        """보드 그림 (회색 uint8). make_charuco.py·합성 시험용."""
        if NEW_API:
            return self.board.generateImage((int(w_px), int(h_px)), marginSize=int(margin_px), borderBits=1)
        return self.board.draw((int(w_px), int(h_px)), marginSize=int(margin_px), borderBits=1)

    def detect(self, gray):
        """회색 영상 → (corners (N,2) float64, ids (N,) int, n_markers). 못 찾으면 N = 0."""
        if gray.ndim == 3:
            gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
        if NEW_API:
            cc, cid, mc, mid = self.detector.detectBoard(gray)
            nm = 0 if mid is None else len(mid)
        else:
            mc, mid, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
            nm = 0 if mid is None else len(mid)
            cc = cid = None
            if nm:
                n, cc, cid = cv2.aruco.interpolateCornersCharuco(mc, mid, gray, self.board)
                if not n:
                    cc = cid = None
        if cc is None or cid is None or len(cid) == 0:
            return np.zeros((0, 2)), np.zeros(0, int), nm
        return np.asarray(cc, np.float64).reshape(-1, 2), np.asarray(cid, int).reshape(-1), nm


# ---------------------------------------------------------------- 자세

def K_of(rp):
    return np.array([[rp["f"], 0.0, rp["cx"]], [0.0, rp["f"], rp["cy"]], [0.0, 0.0, 1.0]])


def solve_pose(board, corners, ids, K):
    """→ dict(R, t, rms, n, ok, why) (보드 → 카메라: X_cam = R·X_board + t)."""
    n = len(ids)
    if n < 6:
        return dict(ok=False, n=n, why="모서리 부족")
    obj = board.obj[ids]
    span = np.ptp(obj[:, :2], axis=0)
    if min(span) < 0.5 * board.square_m:                 # 한 줄에 몰림 → 자세 불안정
        return dict(ok=False, n=n, why="모서리가 한 줄")
    img = corners.reshape(-1, 1, 2)
    ok, rv, tv = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return dict(ok=False, n=n, why="solvePnP 실패")
    rv, tv = cv2.solvePnPRefineLM(obj, img, K, None, rv, tv)
    proj, _ = cv2.projectPoints(obj, rv, tv, K, None)
    err = proj.reshape(-1, 2) - corners
    R, _ = cv2.Rodrigues(rv)
    t = tv.reshape(3)
    if t[2] <= 0:
        return dict(ok=False, n=n, why="보드가 카메라 뒤")
    return dict(ok=True, R=R, t=t, rms=float(np.sqrt(np.mean(np.sum(err ** 2, 1)))), n=n, why="")


def board_quality(board, pose, corners, K):
    """신뢰도 숫자: 보드 화소 크기(볼록 껍질 넓이의 √), 보는 각도(보드 법선 vs 보드 중심 방향), 거리."""
    hull = cv2.convexHull(corners.astype(np.float32))
    px = float(math.sqrt(max(cv2.contourArea(hull), 0.0)))
    R, t = pose["R"], pose["t"]
    c = R @ np.array([board.size_m[0] / 2, board.size_m[1] / 2, 0.0]) + t
    nrm = R[:, 2]
    ang = float(np.degrees(np.arccos(min(1.0, abs(nrm @ c) / np.linalg.norm(c)))))
    return dict(px=px, angle_deg=ang, z_m=float(c[2]))


# ---------------------------------------------------------------- 표본

def bilinear(img, u, v):
    m = cv2.remap(np.ascontiguousarray(img, np.float32), u.astype(np.float32).reshape(1, -1),
                  v.astype(np.float32).reshape(1, -1), cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                  borderValue=np.nan)
    return m.reshape(-1)


def board_samples(board, pose, ids, K, shape, step_frac=0.25):
    """보드 평면 위 점 (검출된 모서리의 보드 좌표 범위 안, 칸의 step_frac 간격) → (u, v, Z_board)."""
    obj = board.obj[ids]
    (x0, y0), (x1, y1) = obj[:, :2].min(0), obj[:, :2].max(0)
    st = board.square_m * step_frac
    xs = np.arange(x0, x1 + 1e-9, st)
    ys = np.arange(y0, y1 + 1e-9, st)
    X, Y = np.meshgrid(xs, ys)
    P = np.column_stack([X.ravel(), Y.ravel(), np.zeros(X.size)])
    C = P @ pose["R"].T + pose["t"]
    Z = C[:, 2]
    u = K[0, 0] * C[:, 0] / Z + K[0, 2]
    v = K[1, 1] * C[:, 1] / Z + K[1, 2]
    H, W = shape[:2]
    m = (Z > 0.05) & (u >= 2) & (u < W - 3) & (v >= 2) & (v < H - 3)
    return u[m], v[m], Z[m]


def _wmedian(x, w):
    o = np.argsort(x)
    cw = np.cumsum(w[o])
    return float(x[o][np.searchsorted(cw, 0.5 * cw[-1])])


def robust_affine(x, y, w, fit_slope=True, iters=10, k=1.5):
    """가중 Huber IRLS: y ≈ a·x + b (fit_slope=False 면 a = 1). → (a, b, 잔차 MAD)."""
    x, y, w = (np.asarray(v, float) for v in (x, y, w))
    a, b = 1.0, _wmedian(y - x, w)
    for _ in range(iters):
        r = y - (a * x + b)
        sc = max(1.4826 * _wmedian(np.abs(r - _wmedian(r, w)), w), 1e-3)
        hw = w * np.minimum(1.0, k * sc / np.maximum(np.abs(r), 1e-12))
        if fit_slope:
            A = np.column_stack([x, np.ones_like(x)]) * np.sqrt(hw)[:, None]
            sol = np.linalg.lstsq(A, y * np.sqrt(hw), rcond=None)[0]
            a, b = float(sol[0]), float(sol[1])
        else:
            b = float(np.sum(hw * (y - x)) / np.sum(hw))
    r = y - (a * x + b)
    return a, b, float(1.4826 * _wmedian(np.abs(r - _wmedian(r, w)), w))


def rotvec_deg(R):
    rv, _ = cv2.Rodrigues(np.asarray(R, float))
    return np.degrees(rv.reshape(3))


# ---------------------------------------------------------------- 누적기

class ScaleConfig:
    """기본값. 키워드로 덮어쓰기: ArucoScale(spec, cfg=ScaleConfig(tau_s=60))."""

    def __init__(self, **kw):
        self.min_corners = 8            # ChArUco 모서리 최소 개수
        self.min_px = 60.0              # 보드 화소 크기 (볼록 껍질 넓이 √) 최소
        self.max_angle_deg = 60.0       # 보드 법선 vs 보는 방향 최대
        self.max_rms_px = 0.8           # solvePnP 재투영 RMS 최대 (1440 폭 기준; 640 폭이면 비례로 줄일 것)
        self.z_range = (0.25, 3.0)      # 보드 중심 거리 (m)
        self.min_samples = 40           # 한 키프레임 유효 표본 최소
        self.tau_s = 60.0               # 시간 감쇠 (EWMA) 시간 상수. 열 드리프트(3분에 0.1~0.2 px)를 따라가도록 (합성 E3: 120 s 는 늦음)
        self.slope_ratio = 1.4          # d_true 범위 (최대/최소) 가 이 이상이면 비례 a 도 맞춤
        self.min_obs_apply = 3          # 보정 적용에 필요한 최소 관측 키프레임 수
        self.max_abs_dd = 2.0           # 보정 안전 한계 |Δd| (px)
        self.s_range = (0.9, 1.1)       # 보정 안전 한계 s
        self.max_obs = 400              # 보관할 관측 수
        self.seen_hold_s = 2.0          # 이 시간 안에 본 적 있으면 board_seen
        self.motion_rot_deg = 0.10      # 움직임 보정 정류 회전이 이만하면 가중치 절반 (잔차가 Δd 에 섞임)
        self.motion_trans_mm = 0.3      # 같은 뜻의 이동 (mm)
        self.vio_min_move_m = 0.15      # VIO 축척 비교에 쓸 최소 카메라 이동
        for k, v in kw.items():
            if not hasattr(self, k):
                raise KeyError(f"모르는 설정 {k}")
            setattr(self, k, v)


class ArucoScale:
    """보드 관측 누적 → 시차 보정 + 상태. 여러 스레드에서 불러도 됨 (lock).

    쓰는 법 (scan3d 키프레임마다):
        obs = A.observe(rectL_bgr, disp_raw, rp, t, valid=None, T_world_cam=T_cam, motion=info, rect1=rectR_bgr)
        disp = A.apply_disp(disp_raw, rp)        # 보정된 시차 (준비 안 됐으면 그대로)
        st = A.status()                          # 웹·로그용 dict
    """

    def __init__(self, spec, cfg=None, log=None):
        self.spec = spec
        self.board = Board(spec)
        self.cfg = cfg or ScaleConfig()
        self.log = log or (lambda m: None)
        self.lock = threading.Lock()
        self.enabled = True             # False 면 apply_* 가 그대로 돌려줌 (관측은 계속)
        self.reset()

    def reset(self):
        with getattr(self, "lock", threading.Lock()):
            self.obs = deque(maxlen=self.cfg.max_obs)
            self.ext = deque(maxlen=self.cfg.max_obs)
            self.vio_pairs = deque(maxlen=200)
            self.fit = dict(mode="none", a=1.0, b=0.0, s=1.0, dd=0.0, mad=None, n=0, ready=False)
            self.last = None
            self.t_seen = None
            self.board_world0 = None

    # ---------------- 관측

    def detect(self, img):
        """rect0 → (corners, ids, n_markers). 검출만 (FFS 전에 보드가 있는지 싸게 확인할 때)."""
        return self.board.detect(img)

    def observe(self, rect0, disp, rp, t, valid=None, T_world_cam=None, motion=None, rect1=None, det=None):
        """키프레임 1장 관측. rect0 = 정류 왼쪽 (BGR/회색, disp 와 같은 크기), disp = FFS 원시 시차 (보정 전! None 이면 자세·VIO·외부만),
        rp = 이 쌍의 정류 dict (f, cx, cy, baseline_m), t = 시각 (s), valid = 유효 화소 마스크 (없으면 disp>0),
        T_world_cam = 이 정류 카메라의 월드 자세 4x4 (VIO, 쌍별 정류면 T_world_rect0 @ R_fix 꼴), motion = 움직임 보정
        정보 dict(rot_deg, trans_mm), rect1 = 정류 오른쪽 (주면 외부 파라미터 잔차), det = detect() 결과 재사용.
        → 관측 dict (used=True/False, why, 숫자들)."""
        cfg = self.cfg
        K = K_of(rp)
        f, B = float(rp["f"]), float(rp["baseline_m"])
        shape = (rect0.shape[0], rect0.shape[1])
        corners, ids, nm = det if det is not None else self.board.detect(rect0)
        o = dict(t=float(t), n_markers=int(nm), n=int(len(ids)), used=False, why="", fB=f * B)
        if len(ids) < cfg.min_corners:
            o["why"] = "보드 안 보임" if len(ids) == 0 else f"모서리 {len(ids)} < {cfg.min_corners}"
            return self._done(o, seen=len(ids) > 0)
        pose = solve_pose(self.board, corners, ids, K)
        if not pose["ok"]:
            o["why"] = pose["why"]
            return self._done(o, seen=True)
        q = board_quality(self.board, pose, corners, K)
        o.update(rms_px=round(pose["rms"], 3), px=round(q["px"], 1), angle_deg=round(q["angle_deg"], 1),
                 z_m=round(q["z_m"], 4))
        bad = []
        if q["px"] < cfg.min_px:
            bad.append(f"작음 {q['px']:.0f}px")
        if q["angle_deg"] > cfg.max_angle_deg:
            bad.append(f"각도 {q['angle_deg']:.0f}°")
        if pose["rms"] > cfg.max_rms_px:
            bad.append(f"재투영 {pose['rms']:.2f}px")
        if not cfg.z_range[0] <= q["z_m"] <= cfg.z_range[1]:
            bad.append(f"거리 {q['z_m']:.2f}m")
        T_cb = np.eye(4)
        T_cb[:3, :3], T_cb[:3, 3] = pose["R"], pose["t"]
        o["T_cam_board"] = T_cb
        # 위치 기준점·VIO 축척 (깊이 표본과 별개; 자세만 좋으면 됨)
        if not bad and T_world_cam is not None:
            self._vio_and_anchor(o, T_cb, np.asarray(T_world_cam, float), t)
        # 외부 파라미터 잔차 (rect1)
        if rect1 is not None and not bad:
            e = self._extrinsic(corners, ids, pose, rect1, K, f, B, disp)
            if e:
                o["ext"] = e
        if bad:
            o["why"] = ", ".join(bad)
            return self._done(o, seen=True)
        if disp is None:
            o["why"] = "시차 없음 (자세만)"
            return self._done(o, seen=True)
        u, v, Zb = board_samples(self.board, pose, ids, K, shape)
        dt = f * B / Zb
        dm = bilinear(disp, u, v)
        ok = np.isfinite(dm) & (dm > 0)
        if valid is not None:
            ok &= bilinear(valid.astype(np.float32), u, v) > 0.99
        if ok.sum() < cfg.min_samples:
            o["why"] = f"유효 표본 {int(ok.sum())} < {cfg.min_samples}"
            return self._done(o, seen=True)
        dt, dm = dt[ok], dm[ok]
        r = dm - dt
        med = float(np.median(r))
        mad = float(1.4826 * np.median(np.abs(r - med)))
        keep = np.abs(r - med) <= max(0.3, 4 * mad)
        dt, dm = dt[keep], dm[keep]
        # 한 장 안 affine (보드가 기울어 깊이 범위가 있을 때 참고용)
        a1, b1, mad1 = robust_affine(dt, dm, np.ones_like(dt), fit_slope=np.ptp(dt) > 0.15 * np.mean(dt))
        w = (min(1.0, len(ids) / 24.0) * min(1.0, q["px"] / 150.0) * math.cos(math.radians(q["angle_deg"]))
             / (1.0 + (pose["rms"] / 0.3) ** 2))
        if motion:
            w /= 1.0 + (float(motion.get("rot_deg") or 0) / cfg.motion_rot_deg) ** 2 \
                + (float(motion.get("trans_mm") or 0) / cfg.motion_trans_mm) ** 2
        off = float(np.median(dm - dt))
        o.update(used=True, w=round(w, 4), n_samples=int(len(dt)), d_true=float(np.mean(dt)), d_ffs=float(np.mean(dm)),
                 offset_px=round(off, 4), mad_px=round(mad, 4), frame_a=round(a1, 5), frame_b=round(b1, 4),
                 depth_err_pct=round(100.0 * (off / float(np.mean(dt))), 3),
                 dt_min=float(dt.min()), dt_max=float(dt.max()))
        with self.lock:
            self.obs.append(o)
            self._refit(t)
        return self._done(o, seen=True)

    def _done(self, o, seen):
        with self.lock:
            if seen:
                self.t_seen = o["t"]
            self.last = {k: v for k, v in o.items() if k != "T_cam_board"}
        return o

    def _refit(self, t_now):
        """lock 안에서. 관측 요약 (d_true 평균, d_ffs 평균, w) 에 시간 감쇠 가중 → affine 또는 오프셋."""
        cfg = self.cfg
        L = list(self.obs)
        x = np.array([o["d_true"] for o in L])
        y = np.array([o["d_ffs"] for o in L])
        w = np.array([o["w"] * math.exp(-max(0.0, t_now - o["t"]) / cfg.tau_s) for o in L])
        live = w > 1e-3 * max(w.max(), 1e-12)
        x, y, w = x[live], y[live], w[live]
        # 각 관측의 d_true 범위 (기운 보드) 도 비례 판단에 넣음
        lo = min(o["dt_min"] for o, k in zip(L, live) if k)
        hi = max(o["dt_max"] for o, k in zip(L, live) if k)
        slope = len(x) >= 3 and hi / max(lo, 1e-6) >= cfg.slope_ratio and np.ptp(x) / max(np.mean(x), 1e-6) > 0.25
        a, b, mad = robust_affine(x, y, w, fit_slope=slope)
        if slope and not (1.0 / cfg.s_range[1] <= a <= 1.0 / cfg.s_range[0]):
            a, b, mad = robust_affine(x, y, w, fit_slope=False)
            slope = False
        s, dd = 1.0 / a, -b / a
        ready = len(x) >= cfg.min_obs_apply and abs(dd) <= cfg.max_abs_dd and cfg.s_range[0] <= s <= cfg.s_range[1]
        self.fit = dict(mode="affine" if slope else "offset", a=a, b=b, s=s, dd=dd, mad=mad, n=int(len(x)),
                        n_total=len(L), ready=bool(ready), t=float(t_now), d_range=[round(lo, 2), round(hi, 2)])

    def _vio_and_anchor(self, o, T_cb, T_wc, t):
        T_wb = T_wc @ T_cb
        c_board = -T_cb[:3, :3].T @ T_cb[:3, 3]                 # 카메라 중심 (보드 좌표)
        with self.lock:
            if self.board_world0 is None:
                self.board_world0 = T_wb
            o["board_drift_m"] = round(float(np.linalg.norm(T_wb[:3, 3] - self.board_world0[:3, 3])), 4)
            prev = [e for e in self.ext if "c_board" in e][-60:]
            for p in prev:
                db = float(np.linalg.norm(c_board - p["c_board"]))
                if db < self.cfg.vio_min_move_m:
                    continue
                dv = float(np.linalg.norm(T_wc[:3, 3] - p["T_wc"][:3, 3]))
                # 회전 일치 확인: 보드로 본 상대 회전 vs VIO 상대 회전
                Rb = T_cb[:3, :3] @ p["T_cb"][:3, :3].T
                Rv = T_wc[:3, :3].T @ p["T_wc"][:3, :3]
                rot_err = float(np.linalg.norm(rotvec_deg(Rb.T @ Rv)))
                self.vio_pairs.append(dict(t=t, ratio=dv / db, move_m=db, rot_err_deg=rot_err))
            self.ext.append(dict(t=t, c_board=c_board, T_wc=T_wc, T_cb=T_cb))

    def _extrinsic(self, corners0, ids0, pose0, rect1, K, f, B, disp):
        """rect1 에서도 검출 → 같은 id 모서리: 모서리 시차 vs d_true (yaw), 세로 어긋남 (pitch/roll), PnP 상대 회전.

        yaw/pitch/roll_mrad = 정류 좌표에서 "실제 오른쪽 카메라가 보정값보다 더 돈 각도" (작은 각, x·y·z 축 회전).
        yaw +0.1 mrad → 모서리 시차가 f·0.1e-3 px (1440 폭 f≈1040 이면 0.10 px) 작아짐 = 깊이가 길게 나옴."""
        c1, id1, _ = self.board.detect(rect1)
        common, i0, i1 = np.intersect1d(ids0, id1, return_indices=True)
        if len(common) < 6:
            return None
        u0, v0 = corners0[i0, 0], corners0[i0, 1]
        u1, v1 = c1[i1, 0], c1[i1, 1]
        P = self.board.obj[common] @ pose0["R"].T + pose0["t"]
        d_true = f * B / P[:, 2]
        d_corner = u0 - u1
        dy = v1 - v0
        A = np.column_stack([np.ones_like(u0), (u0 - K[0, 2]) / f])
        cdy = np.linalg.lstsq(A, dy, rcond=None)[0]
        e = dict(n=int(len(common)), corner_offset_px=round(float(np.median(d_corner - d_true)), 4),
                 yaw_mrad=round(-1000.0 * float(np.median(d_corner - d_true)) / f, 4),
                 dy_px=round(float(np.median(dy)), 4), pitch_mrad=round(-1000.0 * float(cdy[0]) / f, 4),
                 roll_mrad=round(1000.0 * float(cdy[1]) / f, 4))
        dm = bilinear(disp, u0, v0) if disp is not None else np.full(len(u0), np.nan)
        okm = np.isfinite(dm) & (dm > 0)
        if okm.sum() >= 4:                                     # FFS 시차 vs 모서리 시차 (FFS 자체 치우침)
            e["ffs_minus_corner_px"] = round(float(np.median(dm[okm] - d_corner[okm])), 4)
        p1 = solve_pose(self.board, c1, id1, K)
        if p1["ok"]:
            Rrel = p1["R"] @ pose0["R"].T
            trel = p1["t"] - Rrel @ pose0["t"]
            e["pnp_rot_deg"] = [round(float(x), 4) for x in rotvec_deg(Rrel)]     # 이상 = 0 (PnP 잡음 큼, 참고)
            e["pnp_baseline_mm"] = round(-1000.0 * float(trel[0]), 2)              # 이상 = B
        return e

    # ---------------- 보정 적용

    def correction(self):
        """현재 보정 (s, Δd) 또는 None (준비 안 됨·꺼짐). d_corr = s·d + Δd."""
        with self.lock:
            fit = dict(self.fit)
        if not (self.enabled and fit["ready"]):
            return None
        return fit["s"], fit["dd"]

    def apply_disp(self, disp, rp=None):
        """시차 지도 보정 사본 (무효 화소 ≤0·NaN 은 그대로). 준비 안 됐으면 원본 그대로 돌려줌."""
        c = self.correction()
        if c is None:
            return disp
        s, dd = c
        out = np.array(disp, np.float32, copy=True)
        m = np.isfinite(out) & (out > 0)
        out[m] = s * out[m] + dd
        out[m & (out <= 0)] = 0.0
        return out

    def apply_depth(self, Z, rp):
        """깊이 지도 보정 사본 (Z = f·B/d → d 로 바꿔 보정 후 다시 Z). 준비 안 됐으면 그대로."""
        c = self.correction()
        if c is None:
            return Z
        s, dd = c
        fB = float(rp["f"]) * float(rp["baseline_m"])
        Z = np.asarray(Z, np.float32)
        out = np.full(Z.shape, np.nan, np.float32)
        m = np.isfinite(Z) & (Z > 0)
        d = s * (fB / Z[m]) + dd
        out[m] = np.where(d > 0, fB / np.maximum(d, 1e-6), np.nan)
        return out

    def depth_factor(self, Z, rp):
        """깊이 Z 에서 보정 배율 Z_corr/Z (점 구름을 rect0 원점 기준으로 늘이고 줄일 때)."""
        c = self.correction()
        if c is None:
            return np.ones_like(np.asarray(Z, np.float32))
        s, dd = c
        fB = float(rp["f"]) * float(rp["baseline_m"])
        d = fB / np.asarray(Z, np.float32)
        return d / (s * d + dd)

    # ---------------- 상태

    def status(self, t_now=None):
        """웹·로그용 dict (JSON 가능)."""
        with self.lock:
            fit, last = dict(self.fit), self.last
            vp = list(self.vio_pairs)
            ex = [o.get("ext") for o in self.obs if o.get("ext")]
            n_obs = len(self.obs)
            t_seen = self.t_seen
        t_now = t_now if t_now is not None else (last or {}).get("t")
        st = dict(enabled=self.enabled, board=dict(dict=self.spec["dict"], squares=[self.spec["squares_x"],
                  self.spec["squares_y"]], square_mm=round(self.spec["square_m"] * 1000, 3),
                  measured=self.spec["measured"]),
                  board_seen=bool(t_seen is not None and t_now is not None and t_now - t_seen <= self.cfg.seen_hold_s),
                  t_last_seen=t_seen, n_obs=n_obs, mode=fit["mode"], ready=fit["ready"],
                  delta_d_px=round(fit["dd"], 4), s=round(fit["s"], 5), n_fit=fit["n"],
                  resid_mad_px=None if fit["mad"] is None else round(fit["mad"], 4), d_range_px=fit.get("d_range"),
                  last=last)
        # 보정이 깊이에 주는 변화 (1 m, 2 m) — 사람이 읽기 쉬운 숫자. fB 는 마지막 관측 기준이 없어 비율로만
        if last and last.get("fB"):
            fB = last.get("fB")
            if fB:
                st["depth_change_pct"] = {f"{z}m": round(100.0 * ((fB / z) / (fit["s"] * fB / z + fit["dd"]) - 1), 3)
                                          for z in (0.5, 1.0, 2.0)}
        if vp:
            r = np.array([p["ratio"] for p in vp])
            st["vio_scale"] = dict(ratio_median=round(float(np.median(r)), 4), n_pairs=len(vp),
                                   spread=round(float(1.4826 * np.median(np.abs(r - np.median(r)))), 4),
                                   rot_err_deg_median=round(float(np.median([p["rot_err_deg"] for p in vp])), 3))
        if ex:
            st["extrinsic"] = {k: round(float(np.median([e[k] for e in ex if k in e])), 4)
                               for k in ("yaw_mrad", "pitch_mrad", "roll_mrad", "corner_offset_px", "dy_px",
                                         "ffs_minus_corner_px") if any(k in e for e in ex)}
            st["extrinsic"]["n"] = len(ex)
        if not self.spec["measured"]:
            st["warn"] = "보드 칸 크기를 자로 잰 값이 없음 (인쇄 의도 값 사용) — 축척 확인 결과를 믿지 말 것"
        return st


# ---------------------------------------------------------------- 칸 크기 재기 (자 없이, 사진 1장)

# 기준 물체 실제 크기 (mm, 긴 변 × 짧은 변).
#  a4   = ISO 216 A4 210 × 297. 규격 허용 오차는 ±2 mm (297 쪽) 이지만 보통 종이는 그보다 훨씬 작다 (추정 ±0.5 mm ≈ 0.2 %).
#  card = ISO/IEC 7810 ID-1 (신용·체크·교통 카드) 85.60 × 53.98 mm. 두께 0.76 mm → 종이 위에 올리면 높이만큼 크게 보임
#         (40 cm 에서 찍으면 약 +0.2 %) → 종이 옆 같은 탁자 위에 놓을 것.
REF_SIZES_MM = {"a4": (297.0, 210.0), "card": (85.60, 53.98), "letter": (279.4, 215.9)}


def _order_quad(q):
    """4점 → 화면 기준 시계 방향 (왼쪽 위부터)."""
    q = np.asarray(q, np.float64).reshape(4, 2)
    c = q.mean(0)
    a = np.arctan2(q[:, 1] - c[1], q[:, 0] - c[0])
    q = q[np.argsort(a)]
    k = int(np.argmin(q.sum(1)))
    return np.roll(q, -k, axis=0)


def _refine_quad(cnt, quad):
    """윤곽 점들로 네 변마다 직선 맞춤 (가운데 80 %) → 이웃 직선 교점 = 서브화소 모서리."""
    P = cnt.reshape(-1, 2).astype(np.float64)
    lines = []
    for i in range(4):
        a, b = quad[i], quad[(i + 1) % 4]
        ab = b - a
        L = np.linalg.norm(ab)
        if L < 5:
            return quad
        n = np.array([-ab[1], ab[0]]) / L
        s = (P - a) @ ab / L ** 2
        d = np.abs((P - a) @ n)
        m = (s > 0.1) & (s < 0.9) & (d < max(3.0, 0.01 * L))
        if m.sum() < 10:
            return quad
        vx, vy, x0, y0 = cv2.fitLine(P[m].astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
        lines.append((np.array([x0, y0], float), np.array([vx, vy], float)))
    out = []
    for i in range(4):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        M = np.column_stack([d1, -d2])
        if abs(np.linalg.det(M)) < 1e-9:
            return quad
        u = np.linalg.solve(M, p2 - p1)
        out.append(p1 + u[0] * d1)
    out = np.array(out)
    return out if np.max(np.linalg.norm(out - quad, axis=1)) < 0.05 * np.linalg.norm(quad[0] - quad[2]) else quad


def _edge_refine(gray, quad, half=6, n=60):
    """변마다 법선 방향 밝기 단면의 기울기 최대점 (포물선 서브화소) → 직선 맞춤 → 교점.
    문턱값·팽창으로 생기는 윤곽 치우침 (카드 360 px 에서 1 px = 0.3 %) 을 없앤다."""
    g = cv2.GaussianBlur(gray, (0, 0), 1.0).astype(np.float32)
    lines = []
    for i in range(4):
        a, b = quad[i], quad[(i + 1) % 4]
        ab = b - a
        L = float(np.linalg.norm(ab))
        if L < 20:
            return quad
        tdir = ab / L
        nrm = np.array([-tdir[1], tdir[0]])
        s = np.linspace(0.1, 0.9, n)
        base = a[None, :] + s[:, None] * ab[None, :]
        off = np.arange(-half, half + 1, dtype=np.float64)
        P = base[:, None, :] + off[None, :, None] * nrm[None, None, :]
        prof = cv2.remap(g, P[..., 0].astype(np.float32), P[..., 1].astype(np.float32), cv2.INTER_LINEAR)
        d = np.abs(np.diff(prof, axis=1))
        k = np.argmax(d, axis=1)
        pts = []
        for j in range(n):
            kk = int(k[j])
            if d[j, kk] < 8 or kk == 0 or kk == d.shape[1] - 1:
                continue
            y0, y1, y2 = d[j, kk - 1], d[j, kk], d[j, kk + 1]
            den = y0 - 2 * y1 + y2
            sub = 0.5 * (y0 - y2) / den if abs(den) > 1e-6 else 0.0
            o = off[kk] + 0.5 + float(np.clip(sub, -0.5, 0.5))      # diff 칸 k 는 off[k]~off[k+1] 사이
            pts.append(base[j] + o * nrm)
        if len(pts) < n // 3:
            return quad
        vx, vy, x0, y0 = cv2.fitLine(np.array(pts, np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).reshape(4)
        lines.append((np.array([x0, y0], float), np.array([vx, vy], float)))
    out = []
    for i in range(4):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        M = np.column_stack([d1, -d2])
        if abs(np.linalg.det(M)) < 1e-9:
            return quad
        u = np.linalg.solve(M, p2 - p1)
        out.append(p1 + u[0] * d1)
    out = np.array(out)
    return out if np.max(np.linalg.norm(out - quad, axis=1)) < 10 else quad


def _quads(mask, min_area, max_area):
    """이진 영상의 바깥 윤곽 중 볼록 사각형 → [(quad (4,2), 윤곽, 넓이)]."""
    cs, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    out = []
    for c in cs:
        A = cv2.contourArea(c)
        if not (min_area <= A <= max_area):
            continue
        h = cv2.convexHull(c)
        per = cv2.arcLength(h, True)
        for eps in (0.01, 0.02, 0.03, 0.05):
            ap = cv2.approxPolyDP(h, eps * per, True)
            if len(ap) == 4:
                break
        if len(ap) != 4 or not cv2.isContourConvex(ap):
            continue
        if A < 0.85 * cv2.contourArea(ap):                  # 사각형이 아닌 덩어리
            continue
        q = _order_quad(ap)
        out.append((_refine_quad(c, q), c, A))
    return out


def find_reference(gray, ref="a4", board_pts=None):
    """사진에서 기준 사각형 (a4: 어두운 배경 위 흰 종이, card: 카드) 찾기 → (quad (4,2) 시계 방향, info) 또는 (None, info).
    board_pts (보드 모서리 화소) 를 주면 a4 는 그 점을 모두 품은 사각형만, card 는 보드와 겹치지 않는 것만."""
    H, W = gray.shape[:2]
    g = cv2.GaussianBlur(gray, (5, 5), 0)
    cands = []
    _, th = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    masks = [cv2.morphologyEx(th, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)), 255 - th]
    if ref != "a4":
        e = cv2.Canny(g, 40, 120)
        masks.append(cv2.dilate(e, np.ones((3, 3), np.uint8)))
    big = ref == "a4"
    for m in masks:
        cands += _quads(m, (0.08 if big else 0.002) * H * W, (0.98 if big else 0.4) * H * W)
    long_mm, short_mm = REF_SIZES_MM[ref]
    best, bs = None, 1e9
    for q, c, A in cands:
        sides = [np.linalg.norm(q[(i + 1) % 4] - q[i]) for i in range(4)]
        ratio = max(sides[0] + sides[2], sides[1] + sides[3]) / max(min(sides[0] + sides[2], sides[1] + sides[3]), 1e-6)
        if board_pts is not None and len(board_pts):
            inside = [cv2.pointPolygonTest(q.astype(np.float32), (float(x), float(y)), False) >= 0 for x, y in board_pts]
            if big and not all(inside):
                continue
            if not big and any(inside):
                continue
        score = abs(ratio / (long_mm / short_mm) - 1.0)
        if score > 0.25:                                    # 원근으로 비율이 꽤 달라질 수 있어 넉넉히
            continue
        score -= (0.05 * A / (H * W)) if big else 0.0      # 종이: 같은 비율이면 큰 것
        if score < bs:
            best, bs = q, score
    return best, dict(n_candidates=len(cands), ratio_err=None if best is None else round(bs, 3))


def ref_homography(quad, ref):
    """시계 방향 quad → 기준 물체 평면 (mm) 호모그래피. 화면에서 긴 쪽 변 쌍을 긴 변 (mm) 으로."""
    long_mm, short_mm = REF_SIZES_MM[ref]
    s01 = np.linalg.norm(quad[1] - quad[0]) + np.linalg.norm(quad[2] - quad[3])
    s12 = np.linalg.norm(quad[2] - quad[1]) + np.linalg.norm(quad[3] - quad[0])
    w, h = (long_mm, short_mm) if s01 >= s12 else (short_mm, long_mm)
    dst = np.array([[0, 0], [w, 0], [w, h], [0, h]], np.float64)
    return cv2.getPerspectiveTransform(quad.astype(np.float32), dst.astype(np.float32))


def fit_grid_scale(grid, mm):
    """보드 격자 (칸 단위) → mm 점: 닮음 변환 배율 (= 칸 크기 mm) + 축별 배율 (affine) + 잔차 RMS mm."""
    g = np.asarray(grid, float)
    p = np.asarray(mm, float)
    gc, pc = g - g.mean(0), p - p.mean(0)
    U, S, Vt = np.linalg.svd(pc.T @ gc)
    D = np.diag([1.0, np.sign(np.linalg.det(U @ Vt))])
    Rm = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / np.sum(gc ** 2))
    res = pc - s * gc @ Rm.T
    M = np.linalg.lstsq(np.column_stack([gc, np.ones(len(gc))]), pc, rcond=None)[0][:2].T
    return dict(square_mm=s, sx_mm=float(np.linalg.norm(M[:, 0])), sy_mm=float(np.linalg.norm(M[:, 1])),
                rms_mm=float(np.sqrt(np.mean(np.sum(res ** 2, 1)))))


def measure_square_mm(img, board, ref="a4", ref_quad=None):
    """자 없이 칸 크기 재기: 보드가 인쇄된 종이 전체 (a4) 또는 옆에 둔 카드 (card) 가 보이게 위에서 찍은 사진 1장.
    기준 사각형 4 모서리 → 호모그래피 (사진 → 기준 평면 mm) → 보드 모서리를 mm 로 옮겨 칸 크기 맞춤.
    → dict(ok, square_mm, sx_mm, sy_mm, rms_mm, n, ref, quad, warn[]). ref_quad (4,2) 를 주면 자동 찾기 대신 그것."""
    gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, nm = board.detect(gray)
    out = dict(ok=False, ref=ref, n=int(len(ids)), n_markers=int(nm), warn=[])
    if len(ids) < 6:
        out["why"] = f"보드 모서리 {len(ids)} 개 (사진에 보드 전체가 보이게, 반사 없이)"
        return out
    if ref_quad is None:
        q, info = find_reference(gray, ref, corners)
        out.update(info)
    else:
        q = _order_quad(ref_quad)
    if q is None:
        out["why"] = ("종이 네 모서리 못 찾음 (종이보다 어두운 바닥 위에, 종이 전체가 보이게)" if ref == "a4" else
                      "카드 못 찾음 (카드를 종이 옆 어두운/무늬 없는 바닥에)")
        return out
    q = _edge_refine(gray, q)
    q = _edge_refine(gray, q, half=3)
    out["quad_edge"] = True
    Hm = ref_homography(q, ref)
    mm = cv2.perspectiveTransform(corners.reshape(-1, 1, 2).astype(np.float64), Hm).reshape(-1, 2)
    grid = board.obj[ids][:, :2] / board.square_m
    f = fit_grid_scale(grid, mm)
    out.update(ok=True, quad=q.round(2).tolist(), **{k: round(v, 3) for k, v in f.items()})
    if abs(f["sx_mm"] / f["sy_mm"] - 1) > 0.01:
        out["warn"].append(f"가로·세로 칸 크기가 {100 * abs(f['sx_mm'] / f['sy_mm'] - 1):.1f} % 다름 (프린터 비율·사진 왜곡)")
    if f["rms_mm"] > 0.4:
        out["warn"].append(f"잔차 {f['rms_mm']:.2f} mm 큼 (종이가 휘었거나 기준 모서리가 틀림)")
    return out


def make(board_json=None, log=None, **cfg):
    """편의: 보드 json → ArucoScale."""
    return ArucoScale(load_board_spec(board_json), ScaleConfig(**cfg), log=log)


if __name__ == "__main__":
    b = Board(load_board_spec())
    print(f"OpenCV {cv2.__version__} new_api={NEW_API} 모서리 {b.n_corners} 보드 {b.size_m} m")
