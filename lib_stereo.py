"""
lib_stereo.py — 삼각측량 코어 (카메라 1대 버전과 2대 버전이 공용)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
왜 1벌인가
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  "1대를 좌우로 옮겨 2장"과 "2대로 동시에 2장"은 같은 문제다.
  둘 다 "상대 포즈 (R,T)를 아는 두 뷰"일 뿐이고, 다른 건 (R,T)를 어디서 얻느냐뿐이다.
    1대: pose_from_baseline(b)      -> R=I, T=(-b,0,0)
    2대: cv2.stereoCalibrate() 결과 -> R, T
  두 규약이 실제로 같은지 확인했다. opencv 4.13 calib3d.hpp:1933-1934 은
  stereoCalibrate 의 (R,T)를 X2 = R*X1 + T 로 정의한다. 아래 유도와 동일하다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
좌표계와 부호 (숫자로 검증한 유도)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  월드 = 왼쪽 촬영 위치의 카메라 좌표계 (X 오른쪽, Y 아래, Z 전방).
  오른쪽 촬영 위치는 월드에서 (+b, 0, 0) 에 있다. 회전은 없다.
  따라서 월드 점을 오른쪽 카메라 좌표로 옮기면  X2 = X1 - (b,0,0)
  즉  R = I,  T = (-b, 0, 0).

    P1 = [ I | 0 ]                 (정규화 좌표계에서 쓰므로 K가 들어가지 않는다)
    P2 = [ I | (-b,0,0)^T ]

  픽셀 시차:  xL = fx*X/Z + cx,   xR = fx*(X-b)/Z + cx
              d  = xL - xR = fx*b/Z  > 0     ->   Z = fx*b/d

  숫자 검증 (fx=900px, b=0.20m, 점 (0,0,2.0)m):
      xL = cx,  xR = 900*(-0.2)/2 + cx = cx - 90   ->  d = +90px
      Z  = 900*0.20/90 = 2.00 m   ✓ 부호·크기 모두 일치

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
왜곡 처리 방식
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  이미지 전체를 undistort 하지 않고, 매칭된 점 좌표만 undistortPoints 한다.
  이미지를 리샘플링하지 않아 특징점이 뭉개지지 않고 계산도 가볍다.
  undistortPoints(P=None) 은 정규화 좌표를 주므로 P1/P2 에 K가 필요 없어진다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
이 방법으로 알 수 있는 것과 없는 것 (반드시 알고 쓸 것)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  두 시점이 가까우므로 복원되는 것은 물체의 "보이는 앞면"뿐이다.
    - 중심 위치(X,Y,Z), 폭, 높이  -> 관측 가능, 신뢰할 수 있음
    - 앞뒤 두께(깊이)             -> 관측 불가. OBB의 가장 얇은 축은
                                    "물체의 두께"가 아니라 "복원된 표면의 두께"다.
  박스 치수를 볼 때 이 축은 하한으로만 읽어야 한다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
단위 규약
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  전부 미터. 각도는 도(deg)로 출력만 한다.
"""
import cv2
import numpy as np
import open3d as o3d

# 카메라 광학 좌표계(X 우, Y 하, Z 전방) -> 맵 좌표계(X 우, Y 전방, Z 상).
# 3D 맵에서 Y가 아래인 채로 보면 상하가 뒤집혀 보이므로 표시 전에 한 번 돌린다.
#   v_map = CAM_TO_MAP @ v_cam  =>  (x, z, -y)
CAM_TO_MAP = np.array([[1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0],
                       [0.0, -1.0, 0.0]])


# ══════════════════════════════════════════════════════════════════════════════
# 포즈 공급자 — 여기만 1대/2대가 다르다
# ══════════════════════════════════════════════════════════════════════════════
def pose_from_baseline(baseline_m):
    """1대 카메라를 오른쪽으로 baseline_m 만큼 평행이동한 경우의 (R, T).

    회전이 없다고 가정한다. 이 가정이 맞는지는 reconstruct() 의 dy_rms 로 검증한다.
    """
    if baseline_m <= 0:
        raise ValueError("baseline은 양수 — 왼쪽에서 오른쪽으로 옮긴 거리(m)")
    return np.eye(3), np.array([[-baseline_m], [0.0], [0.0]])


def pose_from_board(imgL, imgR, calibL, calibR=None, square_m=None, pattern=None):
    """두 사진 모두에 보이는 체커보드로 두 촬영 위치의 상대 포즈를 직접 "측정"한다.

    pose_from_baseline 은 R=I 를 "가정"한다. 그 가정이 깨지면(특히 yaw)
    거리가 통째로 틀어지는데, 눈으로도 dy 로도 안 보인다.
    이 함수는 가정하는 대신 재기 때문에:
      - 카메라를 손으로 아무렇게나 옮겨도 된다 (회전이 섞여도 그 회전을 측정)
      - 이동거리를 자로 잴 필요가 없다 (스케일은 체커보드 사각형에서 나온다)
      - 평면 퇴화와 무관하다 (Essential 행렬을 쓰지 않는다)
    대신 체커보드가 두 사진에 모두 보여야 하고, 두 촬영 사이에 보드가
    움직이면 안 된다.

    유도:
      solvePnP 는 보드 좌표 Xb 를 카메라 좌표로 옮기는 (R_i, t_i) 를 준다.
          X1 = R1 Xb + t1      ->   Xb = R1^T (X1 - t1)
          X2 = R2 Xb + t2 = R2 R1^T (X1 - t1) + t2
      따라서 X2 = R X1 + T 규약에서
          R = R2 R1^T
          T = t2 - R t1
      이 규약은 stereoCalibrate(calib3d.hpp:1933-1934) 및 pose_from_baseline 과
      같으므로 reconstruct() 는 그대로 쓸 수 있다.

    반환: (R, T, diag). 보드가 한쪽이라도 안 보이면 (None, None, diag).
    """
    import lib_calib as _lc            # 순환 없음 (lib_calib 은 lib_stereo 를 안 쓴다)
    pattern = pattern or _lc.PATTERN
    if square_m is None:
        raise ValueError("square_m 필수 — calib.json 의 square_m 을 넘길 것")
    calibR = calibR or calibL

    gL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    gR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    cL, cR = _lc.find_corners(gL, pattern), _lc.find_corners(gR, pattern)
    diag = dict(found_L=cL is not None, found_R=cR is not None)
    if cL is None or cR is None:
        return None, None, diag

    objp = _lc.object_points(pattern, square_m)
    poses, errs = [], []
    for c, cal in ((cL, calibL), (cR, calibR)):
        ok, rv, tv = cv2.solvePnP(objp, c, cal["K"], cal["dist"],
                                  flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            diag["solvepnp_failed"] = True
            return None, None, diag
        pr, _ = cv2.projectPoints(objp, rv, tv, cal["K"], cal["dist"])
        errs.append(float(np.sqrt(np.mean(np.sum(
            (pr.reshape(-1, 2) - c.reshape(-1, 2)) ** 2, axis=1)))))
        poses.append((cv2.Rodrigues(rv)[0], tv.reshape(3)))

    (R1, t1), (R2, t2) = poses
    R = R2 @ R1.T
    T = (t2 - R @ t1).reshape(3, 1)

    diag.update(
        baseline_m=float(np.linalg.norm(T)),
        rot_deg=float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))),
        board_z_m=(float(t1[2]), float(t2[2])),
        pnp_reproj_px=(errs[0], errs[1]),
        # 2번 카메라가 1번의 오른쪽에 있으면 T의 x가 음수다 (모듈 docstring 유도 참조)
        second_is_right=bool(T[0, 0] < 0))
    return R, T, diag


# ══════════════════════════════════════════════════════════════════════════════
# 매칭
# ══════════════════════════════════════════════════════════════════════════════
def _match_sift(grayL, grayR, maskL, ratio):
    """왼쪽은 ROI 안에서만, 오른쪽은 전체에서 SIFT 검출 후 ratio test."""
    sift = cv2.SIFT_create()
    kL, dL = sift.detectAndCompute(grayL, maskL)
    kR, dR = sift.detectAndCompute(grayR, None)
    if dL is None or dR is None or len(kL) < 2 or len(kR) < 2:
        return np.zeros((0, 2), np.float64), np.zeros((0, 2), np.float64), 0
    knn = cv2.BFMatcher(cv2.NORM_L2).knnMatch(dL, dR, k=2)
    good = [m for m, n in knn if m.distance < ratio * n.distance]
    pL = np.array([kL[m.queryIdx].pt for m in good], np.float64).reshape(-1, 2)
    pR = np.array([kR[m.trainIdx].pt for m in good], np.float64).reshape(-1, 2)
    return pL, pR, len(knn)


def _to_normalized(pts, K, dist):
    """픽셀 좌표 -> 왜곡 보정된 정규화 좌표 (N,2)."""
    if len(pts) == 0:
        return pts.reshape(0, 2)
    return cv2.undistortPoints(pts.reshape(-1, 1, 2), K, dist).reshape(-1, 2)


def _to_pixel(norm, K):
    """정규화 좌표 -> 왜곡 없는 픽셀 좌표. 진단값을 px 단위로 읽기 위함."""
    return np.column_stack([norm[:, 0] * K[0, 0] + K[0, 2],
                            norm[:, 1] * K[1, 1] + K[1, 2]])


# ══════════════════════════════════════════════════════════════════════════════
# 삼각측량 + 박스
# ══════════════════════════════════════════════════════════════════════════════
def triangulate(nL, nR, R, T):
    """정규화 좌표 두 벌 + (R,T) -> 왼쪽 카메라 좌표계의 3D 점 (N,3).

    P는 정규화 좌표계 기준이므로 K가 들어가지 않는다.
    """
    # 빈 입력 가드: 에피폴라 필터가 점을 전부 떨어내면 nL 이 (0,2) 가 되는데
    # cv2.triangulatePoints 는 빈 행렬을 받으면 예외를 던진다.
    # 질감 없는 물체나 시점 변화가 큰 촬영에서 실제로 발생한다.
    if len(nL) == 0:
        return np.zeros((0, 3)), np.zeros(0, bool)

    P1 = np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = np.hstack([R, T.reshape(3, 1)])
    h = cv2.triangulatePoints(P1, P2, nL.T, nR.T)      # (4,N)
    w = h[3]
    ok = np.abs(w) > 1e-12                              # 무한원점 방지
    return (h[:3] / np.where(ok, w, 1.0)).T, ok


def fit_box(pts_map, nb_neighbors=20, std_ratio=2.0, cluster=True):
    """맵 좌표계 점군 -> 배경 분리 + 이상치 제거 후 최소부피 OBB.

    반환: (dict 박스정보 | None, 살아남은 점 (M,3), 살아남은 점의 입력 인덱스 (M,))
    호출측은 거리·중심·치수를 **같은 인덱스 집합**에서 계산해야 한다. 예전엔 거리는
    필터 전 점군의 중앙값, 치수는 필터 후 점군에서 냈는데, 2m 물체 50점 + 4m 배경 60점
    합성에서 거리 3.85m / 치수는 2m 덩어리 것으로 섞여 나왔다 (doc_DEPTH_CODE_REVIEW R03).

    ── 왜 배경 분리(DBSCAN)가 필요한가 ──────────────────────────────────────
    사람이 손으로 그린 ROI 에는 물체 뒤 배경이 딸려 들어온다. 통계적 이상치
    제거는 배경 점이 몇 개뿐일 때만 통하고, 덩어리로 들어오면 못 막는다.
    합성 실험(참값 300x200mm, 물체 Z=1.5m, 배경벽 Z=4m)에서 ROI 마진을 키우자:
        마진 0~50px  -> 287 x 186 mm   (오차 4~7%)
        마진 90px    -> 2683 x 1291 mm (오차 794%) — 박스가 배경까지 감쌌다
    그래서 3D 공간에서 덩어리를 나눠 가장 큰 덩어리만 남긴다.
    eps 는 점 밀도에서 자동으로 잡는다(최근접거리 중앙값의 3배).

    ※ 한계: "가장 큰 덩어리"를 고르므로, 물체보다 배경에 특징점이 훨씬 많으면
      배경을 고를 수 있다. 찾은 덩어리 목록을 box["clusters"]에 남겨 확인할 수
      있게 한다. 애매하면 z_range 로 좁힐 것.

    ── 왜 get_minimal_oriented_bounding_box 인가 ───────────────────────────
    get_oriented_bounding_box 는 BoundingVolume.cpp:232-284 에서 "볼록껍질
    꼭짓점만으로 PCA"를 한다. 노이즈가 있는 표면 점군에서는 축이 면내에서 틀어져
    extent 가 부풀어 오른다. 합성 검증(참값 300x200mm):
        get_oriented_bounding_box          302.7 x 212.6 mm  (오차 0.89% / 6.32%)
        get_minimal_oriented_bounding_box  304.2 x 198.8 mm  (오차 1.41% / 0.61%)
    최소부피 OBB 의 퇴화 처리 (직접 실행해 확인한 동작):
        robust=False -> 완전 평면 입력에서 RuntimeError(Qhull) 발생
        robust=True  -> 예외는 안 나지만 extent [0,0,0] 을 "조용히" 반환
    robust=True 로 부르되 결과를 반드시 검사하고, 퇴화면 껍질 PCA 로 물러선 뒤
    그 사실을 box["method"] 에 남긴다.
    앞뒤 두께는 관측 불가하다는 점은 모듈 docstring 참조.
    """
    P = np.asarray(pts_map, np.float64)
    idx = np.arange(len(P))
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P))
    n_before = len(P)
    if n_before >= nb_neighbors:
        pc, ind = pc.remove_statistical_outlier(nb_neighbors, std_ratio)
        idx = idx[np.asarray(ind, int)]

    clusters = []
    if cluster and len(pc.points) >= 10:
        nn = np.asarray(pc.compute_nearest_neighbor_distance())
        eps = 3.0 * float(np.median(nn[nn > 0])) if np.any(nn > 0) else 0.0
        if eps > 0:
            lab = np.asarray(pc.cluster_dbscan(eps=eps, min_points=5))
            ids = [l for l in np.unique(lab) if l >= 0]
            if ids:
                pts_all = np.asarray(pc.points)
                clusters = sorted(
                    ({"n": int((lab == l).sum()),
                      "z_med": float(np.median(pts_all[lab == l][:, 1]))} for l in ids),
                    key=lambda c: -c["n"])
                best = max(ids, key=lambda l: (lab == l).sum())
                sel = np.where(lab == best)[0]
                pc = pc.select_by_index(sel)
                idx = idx[sel]

    pts = np.asarray(pc.points)
    if len(pts) < 4:
        return None, pts, idx

    method = "minimal"
    try:
        obb = pc.get_minimal_oriented_bounding_box(robust=True)
        ext = np.asarray(obb.extent)
        if not np.all(np.isfinite(ext)) or ext.max() <= 0:
            raise ValueError("최소부피 OBB가 퇴화 결과를 반환")
    except (RuntimeError, ValueError):
        obb = pc.get_oriented_bounding_box(robust=True)
        ext = np.asarray(obb.extent)
        method = "hull-pca(fallback)"

    return dict(center=np.asarray(obb.center), extent=ext,
                R=np.asarray(obb.R), dims_sorted=np.sort(ext)[::-1],
                method=method, clusters=clusters,
                n_before=n_before, n_after=len(pts)), pts, idx


# ══════════════════════════════════════════════════════════════════════════════
def _skew(t):
    t = np.asarray(t, float).ravel()
    return np.array([[0.0, -t[2], t[1]], [t[2], 0.0, -t[0]], [-t[1], t[0], 0.0]])


def epipolar_sampson(nL, nR, R, T):
    """정규화 좌표 대응점의 Sampson 에피폴라 오차 (정규화 단위, 길이 N).

    X2 = R X1 + T 규약에서 E = [T]x R 이고 정확한 대응점은 xR^T E xL = 0 이다.
    """
    if len(nL) == 0:
        return np.zeros(0)
    E = _skew(T) @ np.asarray(R, float)
    xL = np.column_stack([nL, np.ones(len(nL))])
    xR = np.column_stack([nR, np.ones(len(nR))])
    ExL = xL @ E.T                      # 행마다 E @ xL
    EtxR = xR @ E                       # 행마다 E^T @ xR
    num = np.sum(xR * ExL, axis=1)
    den = ExL[:, 0] ** 2 + ExL[:, 1] ** 2 + EtxR[:, 0] ** 2 + EtxR[:, 1] ** 2
    return np.abs(num) / np.sqrt(np.maximum(den, 1e-18))


def triangulation_angle_deg(nL, nR, R):
    """두 광선이 이루는 각(도). 0 에 가까우면 시차가 없어 깊이가 발산한다.

    카메라1 좌표계에서 광선1 = xL, 광선2 = R^T xR.
    """
    if len(nL) == 0:
        return np.zeros(0)
    a = np.column_stack([nL, np.ones(len(nL))])
    b = np.column_stack([nR, np.ones(len(nR))]) @ np.asarray(R, float)   # (R^T xR)^T
    cos = np.sum(a * b, axis=1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1))
    return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))


def reconstruct(imgL, imgR, calibL, R, T, roi=None, calibR=None,
                ratio=0.75, max_epi_px=2.0, z_range=(0.1, 20.0), std_ratio=2.0,
                min_tri_deg=0.1, mask=None, mask_dilate_px=8):
    """두 이미지 + 상대포즈 -> ROI 안 객체의 3D 점군과 박스.

    imgL/imgR  : BGR. imgL이 왼쪽(또는 1번) 카메라.
    calibL     : lib_calib.load_calib() 결과. calibR 생략 시 같은 카메라로 본다.
    roi        : (x, y, w, h) — imgL 기준. None이면 전체.
    mask       : HxW uint8 — 주면 roi 대신 이 영역(세그멘테이션 마스크) 안에서만
                 특징점을 찾는다. 가장자리 특징점이 잘리지 않게 mask_dilate_px 만큼
                 팽창시키므로, 이 영역은 "탐색 영역"이지 측정 영역이 아니다.
                 배경이 섞이면 fit_box 의 DBSCAN 이 거른다.
    max_epi_px : Sampson 에피폴라 잔차 허용치 (왼쪽 카메라 픽셀 단위 근사).
    min_tri_deg: 두 광선 사이 최소 각. 이보다 작으면 시차가 없어 깊이가 발산한다.

    ── 왜 dy 필터가 아니라 Sampson 인가 (R01) ─────────────────────────────
    예전 구현은 |yL-yR| < 2px 와 |xL-xR| >= 1px 로 걸렀다. 그건 "같은 K, R=I"
    (1대 평행이동)에서만 맞는다. 회전이 섞이거나(board 모드) 두 카메라의 K 가
    다르면(웹캠+폰) 정확한 대응점의 y 가 어긋나 전부 탈락했다 (재현: research/
    depth_review_20260921/reproduce.py):
        동일 K + pitch 3°  : 정확한 대응 100개 -> 통과 0개
        R=I, fx 800 vs 520 : 100개 -> 12개
        yaw 2.86°, Z=2m    : 회전이 baseline 의 x 이동을 상쇄해 dx<1px -> 시차 하한에서 전부 탈락
    Sampson 오차는 (R,T)가 정하는 에피폴라 기하 자체로 검사하므로 구성과 무관하고,
    "시차 없음"은 광선 각도로 판정한다 (카메라가 안 움직이면 각도 ~0).
    양쪽 카메라에서 Z>0 (cheirality) 도 검사한다.

    반환: dict(points_map, points_cam, box, diag, idx_final)
      points_cam / z_m / center_cam_m 은 fit_box 가 남긴 최종 집합에서만 낸다 (R03).
      z_raw_m 은 필터 전 참고값이다.
    """
    calibR = calibR or calibL
    KL, dL_, KR, dR_ = calibL["K"], calibL["dist"], calibR["K"], calibR["dist"]
    R = np.asarray(R, float)
    Tv = np.asarray(T, float).reshape(3)

    for img, cal, name in ((imgL, calibL, "L"), (imgR, calibR, "R")):
        hw = (img.shape[1], img.shape[0])
        if tuple(cal["image_size"]) != hw:
            raise ValueError(
                f"{name} 이미지 {hw} 가 캘리브레이션 해상도 {tuple(cal['image_size'])} 와 다름 "
                f"— K는 해상도 종속이라 그대로 쓰면 결과가 통째로 틀린다")

    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    maskL = None
    if mask is not None:
        k = np.ones((2 * mask_dilate_px + 1,) * 2, np.uint8)
        maskL = cv2.dilate((np.asarray(mask) > 0).astype(np.uint8) * 255, k)
    elif roi is not None:
        x, y, w, h = (int(v) for v in roi)
        maskL = np.zeros(grayL.shape, np.uint8)
        maskL[y:y + h, x:x + w] = 255

    pL, pR, n_knn = _match_sift(grayL, grayR, maskL, ratio)
    diag = dict(n_knn=n_knn, n_ratio=len(pL), n_epipolar=0, n_depth=0, n_final=0)

    nL, nR = _to_normalized(pL, KL, dL_), _to_normalized(pR, KR, dR_)
    fx = float(KL[0, 0])

    # --- 에피폴라 필터 (Sampson, 픽셀 단위 근사) ---------------------------------
    samp = epipolar_sampson(nL, nR, R, Tv) * fx
    diag["epi_px_all"] = float(np.sqrt(np.mean(samp ** 2))) if len(samp) else float("nan")
    keep = samp < max_epi_px
    nL, nR, samp = nL[keep], nR[keep], samp[keep]
    diag["n_epipolar"] = int(keep.sum())

    # --- 삼각측량 + cheirality + 광선각 -----------------------------------------
    pts_cam, ok = triangulate(nL, nR, R, Tv)
    ang = triangulation_angle_deg(nL, nR, R)
    zmin, zmax = z_range
    if len(pts_cam):
        cam2 = pts_cam @ R.T + Tv
        good = (ok & (pts_cam[:, 2] > zmin) & (pts_cam[:, 2] < zmax)
                & (cam2[:, 2] > 0) & (ang >= min_tri_deg))
    else:
        good = np.zeros(0, bool)
    pts_cam, nL, nR, samp, ang = pts_cam[good], nL[good], nR[good], samp[good], ang[good]
    diag["n_depth"] = int(good.sum())
    if len(pts_cam) == 0:
        return dict(points_map=np.zeros((0, 3)), points_cam=pts_cam, box=None,
                    diag=diag, idx_final=np.zeros(0, int))

    diag["epi_px"] = float(np.sqrt(np.mean(samp ** 2)))
    diag["tri_deg"] = (float(ang.min()), float(np.median(ang)), float(ang.max()))
    z_raw = pts_cam[:, 2]
    diag["z_raw_m"] = (float(z_raw.min()), float(np.median(z_raw)), float(z_raw.max()))

    # 재투영 오차: 삼각측량된 점을 두 뷰로 되쏘아 원래 매칭 좌표와 비교
    projL = pts_cam[:, :2] / pts_cam[:, 2:3]
    cam2 = pts_cam @ R.T + Tv
    projR = cam2[:, :2] / cam2[:, 2:3]
    eL = np.linalg.norm(_to_pixel(projL, KL) - _to_pixel(nL, KL), axis=1)
    eR = np.linalg.norm(_to_pixel(projR, KR) - _to_pixel(nR, KR), axis=1)
    diag["reproj_px"] = float(np.sqrt(np.mean(np.concatenate([eL, eR]) ** 2)))

    # --- 배경 분리 + 박스 (최종 집합 = idx) --------------------------------------
    pts_map = pts_cam @ CAM_TO_MAP.T
    box, pts_kept, idx = fit_box(pts_map, std_ratio=std_ratio)
    diag["n_final"] = int(len(idx))
    if len(idx):
        zf = pts_cam[idx, 2]
        diag["z_m"] = (float(zf.min()), float(np.median(zf)), float(zf.max()))
        diag["center_cam_m"] = np.median(pts_cam[idx], axis=0).tolist()
    return dict(points_map=pts_kept, points_cam=pts_cam[idx], box=box, diag=diag, idx_final=idx)


def planar_box(pts, min_pts=20, mad_k=3.0, boundary_rays=None, max_thick_ratio=0.6):
    """조밀 점군(보이는 표면)용 크기·중심 추정 — fit_box 의 최소부피 OBB 대신 쓴다.

    조밀 스테레오로 얻은 마스크 안 점군은 거의 평면이다. 그런 점군에는
    get_minimal_oriented_bounding_box(Qhull) 가 퇴화해 축이 틀어진다
    (합성 검증: 정답 0.30x0.20 평면이 308x217mm 로 나옴, 높이 8.3% 과대).

    방법: 깊이 Z 를 중앙값±max(5*MAD, 5%) 로 먼저 절단(배경 혼입 제거; 3% 혼입만으로
          SVD 법선이 뒤집혀 2603x746mm 로 붕괴했었다)
          -> SVD 로 평면 적합 -> 법선 방향 잔차 3*MAD 밖 제거 -> 재적합
          -> 평면 위 2D 좌표에서 cv2.minAreaRect (볼록껍질 기반이라 점 밀도와 무관)
          -> 폭·높이 = 사각형 변, 두께 = 잔차 p2~p98 폭, 중심 = 사각형 중심을 3D 로.
    boundary_rays: (M,3) 마스크 경계 화소의 카메라1 광선(lib_rectify.pixel_rays_cam1).
          주면 그 광선과 적합 평면의 교점(노이즈 없는 3D 윤곽)으로 minAreaRect 를 잰다.
          점군 껍질은 "가장 튄 점"이라 깊이 노이즈가 커지면 크기가 부푼다
          (합성 검증: 노이즈 2% 에서 폭 300 -> 347mm). 평면은 수만 점으로 적합해
          노이즈가 평균화되므로, 평면 + 윤곽 조합이 노이즈에 둔감하다
          (같은 조건 303.6mm). 없으면(bbox 모드) 점군 껍질로 잰다.

    ※ 이 모델은 "보이는 면이 하나의 평면"이라고 가정한다. 구·병·여러 면이 보이는
      물체에는 성립하지 않는다. planarity.ratio(법선 방향 강건 폭 / 짧은 변)가
      max_thick_ratio 를 넘으면 planar_ok=False 로 표시한다. 깊이 노이즈도 폭으로 잡히므로
      이 지표는 노이즈와 비평면을 완전히 구분하지 못한다 — 크게 비평면인 경우만 걸러낸다.

    반환: dict(center, dims_sorted=[W,H,thick], normal, n, size_from, idx, planarity,
               planar_ok) 또는 None
      idx : 입력 pts 기준 최종 사용 점의 인덱스. 거리·중심·치수를 같은 집합에서 내기 위함.
    """
    P0 = np.asarray(pts, np.float64)
    fin = np.isfinite(P0).all(axis=1)
    idx = np.where(fin)[0]
    P = P0[idx]
    if len(P) < min_pts:
        return None
    z = P[:, 2]
    zm = np.median(z); zmad = np.median(np.abs(z - zm))
    tol = max(5.0 * zmad, 0.05 * abs(zm))
    k = np.abs(z - zm) <= tol
    P, idx = P[k], idx[k]
    if len(P) < min_pts:
        return None
    for _ in range(2):                                   # 적합 -> 경계점 제거 -> 재적합
        c = P.mean(axis=0)
        _, _, Vt = np.linalg.svd(P - c, full_matrices=False)
        n = Vt[2]
        res = (P - c) @ n
        mad = np.median(np.abs(res - np.median(res)))
        keep = np.abs(res - np.median(res)) <= mad_k * max(mad, 1e-4)
        if keep.sum() < min_pts:
            break
        P, idx = P[keep], idx[keep]
    c = P.mean(axis=0)
    _, _, Vt = np.linalg.svd(P - c, full_matrices=False)
    e1, e2, n = Vt[0], Vt[1], Vt[2]
    res = (P - c) @ n
    thick = float(np.percentile(res, 98) - np.percentile(res, 2))       # 보고용 (꼬리 포함)
    rms = float(np.sqrt(np.mean(res ** 2)))
    robust_w = float(2.0 * 1.4826 * np.median(np.abs(res - np.median(res))))   # 판정용 (꼬리 무시)

    size_from = "hull"
    Q = P
    if boundary_rays is not None and len(boundary_rays) >= 8:
        d = np.asarray(boundary_rays, np.float64)
        denom = d @ n
        ok = np.abs(denom) > 1e-9
        s_ = (n @ c) / np.where(ok, denom, 1.0)
        ok &= s_ > 0
        if ok.sum() >= 8:
            Q = d[ok] * s_[ok, None]                     # 경계 광선 ∩ 평면
            size_from = "mask-outline"
    uv = np.column_stack([(Q - c) @ e1, (Q - c) @ e2]).astype(np.float32)
    (rx, ry), (w, h), _ = cv2.minAreaRect(uv)
    center = c + rx * e1 + ry * e2
    dims = np.array(sorted([float(w), float(h)], reverse=True) + [thick])
    # 평면성: 법선 방향 강건 폭 / 짧은 변. p2~p98 폭은 깊이 노이즈 꼬리에 부풀어 작은 물체
    # (폰, 짧은 변 100mm, 노이즈 1%@1.5m -> 꼬리 폭 46mm) 를 비평면으로 오판했다.
    ratio = robust_w / max(min(float(w), float(h)), 1e-6)
    return dict(center=center, dims_sorted=dims, normal=n, n=int(len(P)), size_from=size_from,
                idx=idx,
                planarity=dict(thick_m=thick, robust_width_m=robust_w, ratio=float(ratio), rms_m=rms,
                               n_in=int(fin.sum()), n_used=int(len(P))),
                planar_ok=bool(ratio < max_thick_ratio))


def check_pure_translation(imgL, imgR, calib, ratio=0.75):
    """1대 촬영이 정말 "회전 없는 평행이동"이었는지 검증한다.

    이 프로젝트의 1대 방식은 R=I 라는 가정 위에 통째로 서 있다. 가정을 믿지 않고
    두 사진에서 실제 상대 회전을 추정해 각도로 확인한다.
      - Essential 행렬 -> recoverPose 로 (R, t) 추정. t는 크기를 모르는 단위벡터라
        스케일은 못 주지만, 회전각과 이동 방향은 스케일과 무관하게 나온다.
      - 이동 방향 기대값은 t = (-1, 0, 0)  (오른쪽으로 옮겼을 때. 모듈 docstring 유도 참조)
      - 정규화 좌표를 쓰므로 RANSAC 임계값도 정규화 단위여야 한다 -> 1px / fx

    rot_deg 는 세 축 회전을 모두 정확히 잡는다 (합성 검증):
        순수 평행이동 -> 0.031도,  yaw 0.5도 -> 0.411도,  yaw 2도 -> 2.028도,
        pitch 2도 -> 2.032도,      roll 2도 -> 2.026도
    특히 yaw 는 dy 로는 절대 못 잡는다. yaw 는 시차에 거의 균일한 offset 만 더하고
    y 는 건드리지 않기 때문이다. 2도 yaw 에서 dy_med 는 0.014 -> 0.078px 로
    거의 그대로였지만 시차에는 fx*tan(2도)=31px 가 더해져
    Z=1.50m 가 1.19m 로(21%) 틀어졌다. 그래서 회전 판정은 rot_deg 로만 한다.

    ※ 단 하나의 예외 — 장면이 평면 하나에 가까우면 Essential 행렬이 퇴화해서
      rot_deg 를 믿을 수 없다(평면 모호성). 단일 평면 합성 장면에서 참값 0도인데
      14.2도가 나오기도 하고, recoverPose 가 인라이어를 0개 주기도 했다.
      그래서 호모그래피가 전체 매칭 중 몇 %를 설명하는지(planar_ratio)를 함께 내고,
      E 추정이 사실상 실패하면 reliable=False 로 분명히 표시한다. 조용히 넘어가지 않는다.
      -> 촬영할 때 깊이가 다른 대상이 여러 개 보이게 할 것.

    disp_med/disp_p5 는 회전 판정용이 아니라 z_range 를 정하는 참고값이다.

    반환: dict 또는 None(매칭 부족)
    """
    KL, dL_ = calib["K"], calib["dist"]
    gL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    gR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    pL, pR, _ = _match_sift(gL, gR, None, ratio)
    if len(pL) < 8:
        return None

    nL, nR = _to_normalized(pL, KL, dL_), _to_normalized(pR, KL, dL_)
    fx = KL[0, 0]
    thr = 1.0 / fx                      # 정규화 좌표계에서 약 1px

    # dy/시차는 ratio test 를 통과한 전체 매칭에서 본다. E 인라이어에 의존하면
    # E 가 실패하는 바로 그 상황에서 값이 통째로 사라진다.
    uL, uR = _to_pixel(nL, KL), _to_pixel(nR, KL)
    dy = uL[:, 1] - uR[:, 1]
    disp = uL[:, 0] - uR[:, 0]

    # 평면 퇴화 지표: 호모그래피 하나가 전체 매칭의 몇 %를 설명하는가.
    # E 인라이어 수로 나누지 않는다 — E 가 실패하면 정의 자체가 무너지기 때문.
    _, hmask = cv2.findHomography(nL, nR, cv2.RANSAC, thr)
    planar_ratio = (int(hmask.sum()) if hmask is not None else 0) / len(pL)

    out = dict(n_match=len(pL), planar_ratio=float(planar_ratio),
               dy_rms=float(np.sqrt(np.mean(dy ** 2))),
               dy_med=float(np.median(np.abs(dy))),
               disp_med=float(np.median(disp)),
               disp_p5=float(np.percentile(disp, 5)),
               n_inlier=0, reliable=False,
               rot_deg=float("nan"), trans_deg=float("nan"), t_unit=None)

    E, mask = cv2.findEssentialMat(nL, nR, np.eye(3), method=cv2.RANSAC,
                                   prob=0.999, threshold=thr)
    if E is None or E.shape != (3, 3):
        return out
    _, R, t, mask = cv2.recoverPose(E, nL, nR, np.eye(3), mask=mask)
    n_inl = int(mask.ravel().astype(bool).sum())
    out["n_inlier"] = n_inl
    if n_inl < 8:                        # 퇴화 — 각도를 내면 거짓말이 된다
        return out

    tv = t.ravel()
    out.update(reliable=True, t_unit=tv,
               rot_deg=float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))),
               trans_deg=float(np.degrees(np.arccos(
                   np.clip(tv @ np.array([-1.0, 0, 0]), -1, 1)))))
    return out


def check_board_consistency(imgL, imgR, calib, R_board, T_board, ratio=0.75):
    """보드가 말하는 카메라 이동과 "배경 장면"이 말하는 이동이 일치하는가.

    pose_from_board 는 보드가 고정돼 있다고 믿는다. 그 믿음이 깨지는 두 경우가
    조용히 틀린 답을 낸다 (합성 검증, 참값 물체 200x150mm / 거리 0.800m):
        카메라만 이동(정상) -> 195.0x145.7mm, 0.794m   오차 2.5%
        폰만 이동           -> 12573x5966mm, 48.87m    오차 6186%
        둘 다 이동          -> 240.5x174.8mm, 0.952m   오차 20.2%  <- 제일 위험
    마지막은 그럴듯한 숫자라 눈으로 못 걸러낸다. 그래서 기계가 걸러야 한다.

    방법: 보드 영역을 뺀 배경 특징점으로 Essential 행렬을 풀어 카메라 이동을
    독립적으로 구하고 보드가 준 포즈와 대조한다.
      - 카메라만 움직였으면 둘이 일치한다
      - 폰만 움직였으면 배경 점이 아예 안 움직인다 (bg_shift_px ~ 0)
      - 둘 다 움직였으면 서로 다른 회전/방향을 말한다
    bg_shift_px 는 Essential 행렬에 기대지 않으므로, 카메라가 전혀 안 움직여
    E 가 퇴화하는 경우에도 판정할 수 있다.

    반환: dict 또는 None(배경 매칭 부족)
    """
    import lib_calib as _lc
    K, dist = calib["K"], calib["dist"]
    gL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    gR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    # 보드 영역을 마스크에서 제외 — 보드는 판정 대상이지 증거가 아니다
    masks = []
    for g in (gL, gR):
        m = np.full(g.shape, 255, np.uint8)
        c = _lc.find_corners(g)
        if c is not None:
            hull = cv2.convexHull(c.astype(np.int32))
            pad = int(0.6 * np.sqrt(cv2.contourArea(hull) / max(1, len(c))))
            cv2.fillConvexPoly(m, hull, 0)
            if pad > 0:
                m = cv2.erode(m, np.ones((pad, pad), np.uint8))
        masks.append(m)

    sift = cv2.SIFT_create()
    kL, dL = sift.detectAndCompute(gL, masks[0])
    kR, dR = sift.detectAndCompute(gR, masks[1])
    if dL is None or dR is None or len(kL) < 8 or len(kR) < 8:
        return None
    knn = cv2.BFMatcher(cv2.NORM_L2).knnMatch(dL, dR, k=2)
    good = [m for m, n in knn if m.distance < ratio * n.distance]
    if len(good) < 8:
        return None
    pL = np.array([kL[m.queryIdx].pt for m in good], np.float64)
    pR = np.array([kR[m.trainIdx].pt for m in good], np.float64)

    # 배경 점이 실제로 얼마나 움직였는가 — E 없이도 나오는 1차 증거
    bg_shift = float(np.median(np.linalg.norm(pL - pR, axis=1)))
    out = dict(n_bg=len(pL), bg_shift_px=bg_shift, reliable=False,
               rot_diff_deg=float("nan"), trans_dir_deg=float("nan"),
               planar_ratio=float("nan"))

    nL, nR = _to_normalized(pL, K, dist), _to_normalized(pR, K, dist)
    thr = 1.0 / K[0, 0]
    _, hmask = cv2.findHomography(nL, nR, cv2.RANSAC, thr)
    out["planar_ratio"] = float((int(hmask.sum()) if hmask is not None else 0) / len(pL))

    E, mask = cv2.findEssentialMat(nL, nR, np.eye(3), method=cv2.RANSAC,
                                   prob=0.999, threshold=thr)
    if E is None or E.shape != (3, 3):
        return out
    _, Rs, ts, mask = cv2.recoverPose(E, nL, nR, np.eye(3), mask=mask)
    if int(mask.ravel().astype(bool).sum()) < 8:
        return out

    tb = np.asarray(T_board).ravel()
    tb = tb / max(1e-12, np.linalg.norm(tb))
    out.update(
        reliable=True,
        rot_diff_deg=float(np.degrees(np.arccos(np.clip(
            (np.trace(Rs.T @ np.asarray(R_board)) - 1) / 2, -1, 1)))),
        trans_dir_deg=float(np.degrees(np.arccos(np.clip(ts.ravel() @ tb, -1, 1)))))
    return out


# 보드 일관성 게이트 — 전부 합성 실험에서 관측된 값으로 정했다.
#   정상(카메라만 이동): 배경이동 106px, 회전차 0.27deg, 방향차 0.91deg, 평면비 0.53
#   폰만 이동          : 배경이동 0.00px  <- BG_SHIFT 로 잡힘
#   둘 다 이동(직교)   : 방향차 6.68deg   <- TRANS_DIR 로 잡힘
BG_SHIFT_GATE_PX = 2.0
ROT_DIFF_GATE_DEG = 2.0
TRANS_DIR_GATE_DEG = 3.0
BOARD_PLANAR_GATE = 0.90


def board_check_lines(cc):
    """check_board_consistency 결과 -> 판정 문자열 목록. 출력은 호출측이 한다.

    z_capture 와 z_reconstruct 가 같은 판정을 써야 하므로 여기 한 벌만 둔다.
    """
    if cc is None:
        return ["  보드 일관성 검사 불가 — 배경 특징점이 8개 미만",
                "       배경에 질감이 있어야 카메라가 실제로 움직였는지 확인할 수 있다."]
    L = [f"  배경 특징점 {cc['n_bg']}개, 배경 이동 중앙값 {cc['bg_shift_px']:.2f} px"]

    moved = cc["bg_shift_px"] >= BG_SHIFT_GATE_PX
    L.append(f"  [{'PASS' if moved else 'FAIL'}] 카메라가 실제로 움직였는가 "
             f"({cc['bg_shift_px']:.2f} >= {BG_SHIFT_GATE_PX} px)")
    if not moved:
        L += ["       배경이 그대로다 = 카메라는 안 움직이고 폰만 움직였다.",
              "       시차가 없으므로 삼각측량이 원리적으로 불가능하다. 다시 찍을 것."]
        return L

    if cc["planar_ratio"] >= BOARD_PLANAR_GATE or not cc["reliable"]:
        L += [f"  [WARN] 배경 평면비 {cc['planar_ratio']:.2f} — 배경이 평면 한 장에 가까워",
              "       보드 포즈와의 대조가 성립하지 않는다. 깊이가 다른 배경이 보이게 찍을 것."]
        return L

    rot_ok = cc["rot_diff_deg"] < ROT_DIFF_GATE_DEG
    dir_ok = cc["trans_dir_deg"] < TRANS_DIR_GATE_DEG
    L.append(f"  [{'PASS' if rot_ok else 'FAIL'}] 보드와 배경의 회전 일치 "
             f"{cc['rot_diff_deg']:.2f} deg (< {ROT_DIFF_GATE_DEG})")
    L.append(f"  [{'PASS' if dir_ok else 'FAIL'}] 보드와 배경의 이동방향 일치 "
             f"{cc['trans_dir_deg']:.2f} deg (< {TRANS_DIR_GATE_DEG})")
    if not (rot_ok and dir_ok):
        L += ["       보드와 배경이 서로 다른 움직임을 말한다 = 촬영 사이에 폰이 움직였다.",
              "       폰과 물체는 고정하고 카메라만 옮겨서 다시 찍을 것."]
    else:
        L += ["       주의: 폰이 카메라와 '같은 방향'으로 밀렸다면 이 검사로는 못 잡는다.",
              "       그건 순수한 스케일 오차인데 스케일의 출처가 보드뿐이라 대조할 기준이 없다.",
              "       오차 크기는 대략 (폰이 밀린 거리 / baseline) 이다 —",
              "       실측: 30mm 밀림 / 150mm baseline -> 16.8%,  5mm 밀림 -> 1.1%.",
              "       폰을 확실히 고정하고 baseline 을 크게 잡는 것이 유일한 대비책이다."]
    return L


def depth_error_m(z, baseline_m, fx_px, disp_err_px=0.5):
    """이 구성에서 이론상 깊이 오차. dZ = Z^2 * dd / (fx * b)

    촬영 전에 baseline을 얼마로 잡을지 판단하는 데 쓴다.
    """
    return z ** 2 * disp_err_px / (fx_px * baseline_m)
