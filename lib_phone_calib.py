"""폰 두 렌즈(예: S24 초광각 2 + 광각 5) 스테레오 캘리브레이션.

세 가지 출처를 지원한다.
  1) factory : Camera2 CameraCharacteristics(공장 캘리브레이션)를 OpenCV 규약으로 변환
  2) refine  : 장면 SIFT 대응점으로 공장 회전(+T 방향)만 재추정. |T| 는 공장값 고정
  3) board   : 두 렌즈가 동시에 본 체커보드로 cv2.stereoCalibrate (권장 — 유일하게 baseline
               스케일과 yaw 를 실제 데이터로 잡는다)

Android 규약 (NdkCameraMetadataTags.h, ACAMERA_LENS_POSE_* / ACAMERA_LENS_INTRINSIC_CALIBRATION 주석 인용):
  poseRotation   : "quaternion rotation from the Android sensor coordinate system to a
                    camera-aligned coordinate system" , p' = R p (p 는 센서 좌표계, p' 는 카메라 좌표계)
  poseTranslation: "position of the camera optical center ... relative to the reference
                    (PRIMARY_CAMERA) ... needs to be negated to convert it to a translation
                    from the camera to the origin"  =>  X_cam_i = R_i (X_w - t_i)
  intrinsics     : [fx, fy, cx, cy, s] , 좌표계는 pre-correction active array 픽셀
  distortion     : [κ1, κ2, κ3, κ4, κ5] 이며 κ1..3 방사, κ4,κ5 접선 => OpenCV 순서 [κ1, κ2, κ4, κ5, κ3]
따라서 두 물리 렌즈 A(왼), B(오른) 의 OpenCV 상대 포즈(X_B = R X_A + T) 는
  R = R_B R_Aᵀ ,  T = R_B (t_A − t_B)
이다. 이 규약은 실제 쌍에서 (1) T 가 영상 x 축과 나란하고 (2) B 가 A 오른쪽(Tx<0) 인 것으로 확인했다
(STATUS.md 문제점 16). 공장 회전 자체는 0.3~0.6° 어긋나 있어 그대로 쓰면 dy 4~6px 가 남는다.

주의 — baseline 15.76mm 의 한계: 1m 에서 시차가 f·b/Z ≈ 451·0.01576/1 ≈ 7px 뿐이다. yaw 0.1° 오차
(≈0.8px 시차 편향)만으로 1m 깊이가 11% 틀어진다. 에피폴라 잔차(dy)만으로는 yaw 가 거의 관측되지
않으므로 refine 결과는 "보정 안 된 스케일" 이고, 체커보드 stereoCalibrate 또는 실측 거리 검증이 필요하다.
"""
import json
from pathlib import Path

import cv2
import numpy as np

import lib_calib
import lib_rectify as lr
import lib_stereo as ls

_ROOT = Path(__file__).resolve().parent
# --list 가 방금 저장한 것을 우선, 없으면 저장소에 넣어둔 S24(SM-S921N) 사본
DEFAULT_INVENTORY = next((p for p in (_ROOT / "phone_stereo/runs/cameras.json", _ROOT / "phone_stereo/cameras_s24.json") if p.exists()),
                         _ROOT / "phone_stereo/cameras_s24.json")


# ══════════════════════════════════════════════════════════════════════════════
# 1) 공장 캘리브레이션 -> OpenCV
# ══════════════════════════════════════════════════════════════════════════════
def load_inventory(path=DEFAULT_INVENTORY):
    """z_phone_stereo.py --list 가 저장한 cameras.json -> {id: characteristics}.

    논리 카메라 항목 안의 physical[] (물리 5, 6, 2) 를 펼쳐 넣는다. 같은 id 가 최상위에도 있으면
    (예: 단독 노출된 초광각 '2') 값은 동일하므로 어느 쪽이든 된다.
    """
    inv = {}
    for c in json.loads(Path(path).read_text()):
        for p in c.get("physical") or []:
            if "intrinsics" in p:
                inv[str(p["id"])] = p
        inv.setdefault(str(c["id"]), c)
    return inv


def parse_rect(s):
    """'Rect(0, 0 - 4000, 3000)' -> (w, h)."""
    v = [int(x) for x in s.replace("Rect(", "").replace(")", "").replace(" - ", ",").replace(" ", "").split(",")]
    return v[2] - v[0], v[3] - v[1]


def K_from_characteristics(c, size):
    """active array 픽셀 기준 [fx, fy, cx, cy, s] -> 출력 해상도 size=(w,h) 의 K.

    물리 스트림은 센서 active array 전체 시야를 4:3 그대로 축소해 내보낸다(S24 4080x3060, 4000x3000,
    3392x2544 모두 4:3). 4:3 이 아닌 size 를 요구하면 크롭이 섞이므로 거부한다.
    """
    fx, fy, cx, cy, s = [float(v) for v in c["intrinsics"]]
    if fx <= 1 or fy <= 1:
        raise ValueError(f"카메라 {c.get('id')} 에 공장 intrinsics 없음: {c['intrinsics']}")
    aw, ah = parse_rect(c["active_array"])
    sx, sy = size[0] / aw, size[1] / ah
    if abs(sx / sy - 1) > 0.01:
        raise ValueError(f"출력 {size} 의 비율이 active array {aw}x{ah} 와 다름 — 크롭 규약을 알 수 없다")
    return np.array([[fx * sx, s * sx, cx * sx], [0.0, fy * sy, cy * sy], [0.0, 0.0, 1.0]])


def dist_from_characteristics(c):
    """Android [κ1, κ2, κ3, κ4, κ5] -> OpenCV [k1, k2, p1, p2, k3]."""
    k1, k2, k3, k4, k5 = [float(v) for v in c["distortion"]]
    return np.array([k1, k2, k4, k5, k3])


def quat_to_R(q):
    """Android 쿼터니언 (x, y, z, w) -> 회전행렬. NDK 문서의 행렬식 그대로."""
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                     [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
                     [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]])


def relative_pose(cA, cB):
    """X_B = R X_A + T (cv2.stereoCalibrate 규약). 모듈 docstring 의 유도 참조."""
    RA, RB = quat_to_R(cA["pose_rotation"]), quat_to_R(cB["pose_rotation"])
    tA, tB = np.asarray(cA["pose_translation"], float), np.asarray(cB["pose_translation"], float)
    return RB @ RA.T, RB @ (tA - tB)


def rotation_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def factory_stereo(inv, idL, idR, size, logical="0"):
    """공장값만으로 스테레오 캘리 dict. Tx>0 이면 (L,R 이 바뀐 것) 예외."""
    cL, cR = inv[str(idL)], inv[str(idR)]
    R, T = relative_pose(cL, cR)
    st = dict(K1=K_from_characteristics(cL, size), dist1=dist_from_characteristics(cL),
              K2=K_from_characteristics(cR, size), dist2=dist_from_characteristics(cR),
              R=R, T=T, image_size=[int(size[0]), int(size[1])],
              phone=dict(logical=str(logical), physical=[str(idL), str(idR)]),
              baseline_m=float(np.linalg.norm(T)), factory_baseline_m=float(np.linalg.norm(T)),
              factory_rotation_deg=rotation_deg(R),
              # Android 센서 좌표계(SensorEvent: 세로 기준 x 오른쪽, y 위, z 화면 밖) -> 각 렌즈 카메라 좌표계.
              # 중력/자이로 값을 카메라 프레임으로 옮길 때 쓴다 (lib_phone_depth.gravity_frame).
              R_sensor_to_left=quat_to_R(cL["pose_rotation"]), R_sensor_to_right=quat_to_R(cR["pose_rotation"]),
              pose_source="factory:CameraCharacteristics", scale_status="factory_unverified",
              warnings=["공장 회전은 실제 쌍에서 dy 4~6px 남음 — refine 또는 board 필요",
                        "yaw/baseline 은 실측 검증 전"])
    if T[0] >= 0:
        raise ValueError(f"물리 {idR} 가 {idL} 의 왼쪽에 있음 (Tx={T[0]*1000:+.2f}mm). --physical {idR} {idL} 순서로")
    return st


def order_left_right(inv, ids, size):
    """두 물리 id 를 (왼, 오른) 순서로. 규약: 오른쪽 카메라 T.x < 0."""
    a, b = str(ids[0]), str(ids[1])
    R, T = relative_pose(inv[a], inv[b])
    return (a, b) if T[0] < 0 else (b, a)


# ══════════════════════════════════════════════════════════════════════════════
# 2) 장면 대응점으로 회전 재추정 (|T| 공장 고정)
# ══════════════════════════════════════════════════════════════════════════════
def load_pairs(dirs, idL, idR):
    """번들 디렉터리(camera_<id>.png 포함) 또는 그 부모(captures/<ts>/…) 목록 -> [(name, imgL, imgR)]."""
    out = []
    for d in dirs:
        d = Path(d)
        cands = [d] if (d / f"camera_{idL}.png").exists() else sorted(p for p in d.glob("*/") if (p / f"camera_{idL}.png").exists())
        cands += sorted(p for p in d.glob("captures/*/") if (p / f"camera_{idL}.png").exists())
        for p in cands:
            imL, imR = cv2.imread(str(p / f"camera_{idL}.png")), cv2.imread(str(p / f"camera_{idR}.png"))
            if imL is None or imR is None:
                raise FileNotFoundError(f"{p}: camera_{idL}.png / camera_{idR}.png 둘 다 필요")
            out.append((str(p), imL, imR))
    if not out:
        raise FileNotFoundError(f"{dirs}: camera_{idL}.png 가 있는 번들 없음")
    return out


def common_focus(pair_names):
    """번들들의 pair.json focus_m 이 하나로 일치하면 그 값, 없거나 섞여 있으면 None (+ 사유)."""
    vals = set()
    for n in pair_names:
        f = Path(n) / "pair.json"
        if f.exists():
            vals.add(json.loads(f.read_text()).get("focus_m"))
    if len(vals) == 1 and None not in vals:
        return vals.pop(), None
    return None, ("focus_m 기록 없음(구버전 번들)" if not vals or vals == {None} else f"focus_m 이 섞여 있음 {sorted(v for v in vals if v is not None)}")


def collect_matches(pairs, st, ratio=0.75):
    """모든 쌍의 SIFT 대응점 -> 정규화 좌표 (nL, nR), 쌍별 개수."""
    nL, nR, counts = [], [], []
    for _, imL, imR in pairs:
        gL, gR = cv2.cvtColor(imL, cv2.COLOR_BGR2GRAY), cv2.cvtColor(imR, cv2.COLOR_BGR2GRAY)
        pL, pR, _ = ls._match_sift(gL, gR, None, ratio)
        nL.append(ls._to_normalized(pL, st["K1"], st["dist1"]))
        nR.append(ls._to_normalized(pR, st["K2"], st["dist2"]))
        counts.append(len(pL))
    return np.vstack(nL), np.vstack(nR), counts


def refine_pose(nL, nR, R0, T0, f_px, fit_t=False, fix_yaw=True, fit_scale=True, iters=80, cauchy_px=1.0):
    """Sampson 에피폴라 오차(px) 의 Cauchy 로버스트 최소화 (Levenberg–Marquardt, 수치 야코비안). scipy 불필요.

    파라미터
      회전 보정 w : R = exp(w) R0. fix_yaw=True 면 x,z 축(roll/pitch) 2개만, False 면 3개
      광각 초점거리 스케일 s (fit_scale) : 오른쪽 정규화 좌표 nR/s  ⇔  K2 의 fx,fy × s
      T 방향 보정 v (fit_t) : T = exp(v) T0. |T| 는 절대 바꾸지 않는다 (에피폴라 제약은 스케일 불가)
    기본값의 근거 (S24 초광각2+광각5, 초점 1m 고정 번들 8쌍 적합 → 별도 6쌍 평가, STATUS 문제점 16):
      - yaw(y축) 를 자유로 두면 −0.87° 로 튀며 시차가 음수로 뒤집힘(dx>0 비율 0.48). dy 로는 yaw 가 관측되지 않아
        스케일 불일치를 yaw 로 흡수한 것. → 기본 공장 yaw 고정. yaw 는 실측 거리(--known) 나 체커보드(--board) 로만 잡는다.
      - 광각은 AF 렌즈라 초점 위치에 따라 fx 가 변한다(초점 1m 에서 공장값의 0.991). 스케일을 넣어야 dy 의
        y-기울기가 사라진다: 평가쌍 dy 0.57~0.89 → 0.43~0.56px.
      - T 방향은 16° 움직여도 비용이 거의 안 줄어 기본 끔.
    반환 R, T, s, info
    """
    idx = [0, 2] if fix_yaw else [0, 1, 2]
    n_rot = len(idx)
    n_par = n_rot + (1 if fit_scale else 0) + (3 if fit_t else 0)
    def unpack(x):
        w = np.zeros(3); w[idx] = x[:n_rot]
        R = cv2.Rodrigues(w)[0] @ R0
        s = float(np.exp(x[n_rot])) if fit_scale else 1.0
        T = cv2.Rodrigues(x[-3:])[0] @ T0 if fit_t else T0
        return R, T, s
    def res(x):
        R, T, s = unpack(x)
        return ls.epipolar_sampson(nL, nR / s, R, T) * f_px
    def cost(r):
        return float(np.sum(cauchy_px**2 * np.log1p((r / cauchy_px) ** 2)))
    x, lam, eps = np.zeros(n_par), 1e-3, 1e-7
    r = res(x); c = cost(r)
    for _ in range(iters):
        w = 1.0 / (1.0 + (r / cauchy_px) ** 2)
        J = np.empty((len(r), n_par))
        for k in range(n_par):
            dx = np.zeros(n_par); dx[k] = eps
            J[:, k] = (res(x + dx) - r) / eps
        JW = J * w[:, None]
        step = -np.linalg.solve(J.T @ JW + lam * np.eye(n_par), JW.T @ r)
        r_new = res(x + step)
        if cost(r_new) < c:
            x, r, c, lam = x + step, r_new, cost(r_new), max(lam / 3, 1e-9)
            if np.linalg.norm(step) < 1e-9:
                break
        else:
            lam *= 10
            if lam > 1e6:
                break
    R, T, s = unpack(x)
    rot = np.zeros(3); rot[idx] = np.degrees(x[:n_rot])
    return R, T, s, dict(n=int(len(r)), sampson_med_px=float(np.median(r)), inlier_frac_1px=float((r < 1.0).mean()),
                         inlier_frac_2px=float((r < 2.0).mean()), delta_rot_deg_xyz=rot.round(4).tolist(),
                         delta_rot_deg=float(np.degrees(np.linalg.norm(x[:n_rot]))), fix_yaw=bool(fix_yaw),
                         focal_scale_wide=s, delta_t_dir_deg=float(np.degrees(np.linalg.norm(x[-3:]))) if fit_t else 0.0)


def apply_scale(st, s):
    """광각(오른쪽, K2) fx,fy 에 스케일 s 적용. cx,cy 불변."""
    K2 = np.asarray(st["K2"], float).copy(); K2[0, 0] *= s; K2[1, 1] *= s
    new = dict(st, K2=K2); new["focal_scale_wide"] = float(s) * float(st.get("focal_scale_wide", 1.0))
    return new


def roi_to_source(rp, roi):
    """정류 왼쪽 영상의 ROI(x,y,w,h) -> 원본 왼쪽 영상의 마스크. remap 테이블(dst->src) 값의 범위를 쓴다."""
    x, y, w, h = [int(v) for v in roi]
    mx, my = rp["mapL"][0][y:y + h, x:x + w], rp["mapL"][1][y:y + h, x:x + w]
    W, H = rp["size"]
    x0, x1 = int(np.clip(mx.min(), 0, W - 1)), int(np.clip(mx.max(), 0, W - 1))
    y0, y1 = int(np.clip(my.min(), 0, H - 1)), int(np.clip(my.max(), 0, H - 1))
    mask = np.zeros((H, W), np.uint8); mask[y0:y1 + 1, x0:x1 + 1] = 255
    return mask


def pin_yaw_known_distance(st, imL, imR, roi, known_m, iters=8):
    """실측 거리 하나로 yaw 를 고정: ROI 의 SIFT 중앙 시차가 f·b/Z_known 이 되도록 오른쪽 카메라를 y축 회전.

    dy 로 관측되지 않는 yaw 가 시차에는 f·δ 의 편향으로 그대로 들어가므로, 거리를 아는 물체 하나면 δ 가 정해진다.
    대응점은 원본 영상에서 **한 번만** 찾고(ROI 는 정류→원본으로 되돌려 마스크), 후보 δ 마다 cv2.undistortPoints(R=R1/R2,
    P=P1/P2) 로 정류 좌표를 해석적으로 다시 계산한다 — 매번 SIFT 를 다시 돌리면 ±0.3px 잡음으로 할선법이 흔들렸다.
    반환 (st_new, info).
    """
    cal = [dict(K=np.asarray(st[f"K{i}"], float), dist=np.asarray(st[f"dist{i}"], float), image_size=tuple(st["image_size"])) for i in (1, 2)]
    R0, T0 = np.asarray(st["R"], float), np.asarray(st["T"], float)
    rp0 = lr.rectify_maps(*cal, R0, T0)
    mask = roi_to_source(rp0, roi)
    pL, pR, _ = ls._match_sift(cv2.cvtColor(imL, cv2.COLOR_BGR2GRAY), cv2.cvtColor(imR, cv2.COLOR_BGR2GRAY), mask, 0.75)
    if len(pL) < 8:
        raise ValueError(f"ROI 안 대응점 {len(pL)}개 — 질감 있는 물체를 ROI 로 잡을 것")
    def disparity(delta_rad):
        R = cv2.Rodrigues(np.array([0.0, delta_rad, 0.0]))[0] @ R0
        rp = lr.rectify_maps(*cal, R, T0)
        qL = cv2.undistortPoints(pL.reshape(-1, 1, 2), cal[0]["K"], cal[0]["dist"], R=rp["R1"], P=rp["P1"]).reshape(-1, 2)
        qR = cv2.undistortPoints(pR.reshape(-1, 1, 2), cal[1]["K"], cal[1]["dist"], R=rp["R2"], P=rp["P2"]).reshape(-1, 2)
        ok = np.abs(qL[:, 1] - qR[:, 1]) < 2.0
        if ok.sum() < 8:
            raise ValueError(f"정류 후 dy<2px 대응점 {int(ok.sum())}개 — 정류가 맞지 않음 (--check 먼저)")
        return float(np.median(qL[ok, 0] - qR[ok, 0])), rp, R, int(ok.sum())
    d0, rp, R, n = disparity(0.0)
    target = rp["f"] * rp["baseline_m"] / known_m
    hist = [(0.0, d0)]
    x0, f0 = 0.0, d0 - target
    x1 = -f0 / rp["f"]                       # 1차 근사: 시차 편향 ≈ f·δ
    for _ in range(iters):
        d1, rp, R, n = disparity(x1); f1 = d1 - target; hist.append((x1, d1))
        if abs(f1) < 0.01 or abs(f1 - f0) < 1e-12:
            break
        x0, f0, x1 = x1, f1, x1 - f1 * (x1 - x0) / (f1 - f0)
    d_fin, rp, R, n = disparity(x1)
    new = dict(st, R=R)
    new.update(yaw_pin=dict(known_m=float(known_m), roi=[int(v) for v in roi], n_matches=int(n), disparity_before_px=d0,
                            disparity_after_px=d_fin, target_px=float(target), yaw_delta_deg=float(np.degrees(x1)),
                            depth_before_m=float(rp["f"] * rp["baseline_m"] / d0) if d0 > 0 else None,
                            history=[(float(np.degrees(a)), float(b)) for a, b in hist]))
    return new, new["yaw_pin"]


# ══════════════════════════════════════════════════════════════════════════════
# 3) 체커보드 stereoCalibrate
# ══════════════════════════════════════════════════════════════════════════════
def board_stereo_calibrate(pairs, st, square_m, pattern=lib_calib.PATTERN, fix_intrinsics=True):
    """두 렌즈가 동시에 본 체커보드 번들들로 R, T (옵션: K, dist 도) 를 추정.

    fix_intrinsics=True : 공장 K/dist 고정 (CALIB_FIX_INTRINSIC) — 뷰가 적어도 안정적
    False               : 공장값을 초기값으로 K/dist 까지 재추정 (뷰 20장 이상 권장)
    반환 st 갱신본 + 진단(rms, per_view, n_views, used, baseline 비교).
    """
    objp = lib_calib.object_points(pattern, square_m)
    obj, cL_list, cR_list, used, skipped = [], [], [], [], []
    for name, imL, imR in pairs:
        gL, gR = cv2.cvtColor(imL, cv2.COLOR_BGR2GRAY), cv2.cvtColor(imR, cv2.COLOR_BGR2GRAY)
        cL, cR = lib_calib.find_corners(gL, pattern), lib_calib.find_corners(gR, pattern)
        if cL is None or cR is None:
            skipped.append((name, "L" if cL is None else "", "R" if cR is None else ""))
            continue
        obj.append(objp); cL_list.append(cL.astype(np.float32)); cR_list.append(cR.astype(np.float32)); used.append(name)
    if len(obj) < 5:
        raise ValueError(f"체커보드가 양쪽 다 검출된 쌍 {len(obj)}개 — 최소 5, 권장 15+. 건너뜀: {skipped}")
    size = tuple(st["image_size"])
    K1, d1, K2, d2 = (np.asarray(st[k], np.float64).copy() for k in ("K1", "dist1", "K2", "dist2"))
    flags = cv2.CALIB_FIX_INTRINSIC if fix_intrinsics else cv2.CALIB_USE_INTRINSIC_GUESS
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7)
    out = cv2.stereoCalibrateExtended(obj, cL_list, cR_list, K1, d1, K2, d2, size,
                                      np.asarray(st["R"], np.float64), np.asarray(st["T"], np.float64).reshape(3, 1),
                                      flags=flags, criteria=crit)
    rms, K1n, d1n, K2n, d2n, R, T = out[0], out[1], out[2], out[3], out[4], out[5], out[6]
    per_view = np.asarray(out[-1]).reshape(-1, 2).tolist() if np.asarray(out[-1]).size == 2 * len(obj) else None
    T = np.asarray(T, float).ravel()
    new = dict(st)
    new.update(K1=K1n, dist1=np.asarray(d1n).ravel(), K2=K2n, dist2=np.asarray(d2n).ravel(), R=R, T=T,
               baseline_m=float(np.linalg.norm(T)), pose_source="board:cv2.stereoCalibrate",
               board=dict(rms_px=float(rms), n_views=len(obj), used=used, skipped=skipped, per_view_px=per_view,
                          square_m=float(square_m), pattern=list(pattern), fix_intrinsics=bool(fix_intrinsics)))
    return new


# ══════════════════════════════════════════════════════════════════════════════
# 저장/불러오기
# ══════════════════════════════════════════════════════════════════════════════
def save(path, st):
    d = {}
    for k, v in st.items():
        d[k] = np.asarray(v).tolist() if isinstance(v, np.ndarray) else v
    Path(path).write_text(json.dumps(d, indent=2, ensure_ascii=False))


def load(path):
    st = json.loads(Path(path).read_text())
    for k in ("K1", "K2", "dist1", "dist2", "R", "T"):
        st[k] = np.asarray(st[k], np.float64)
    return st
