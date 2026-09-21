"""
lib_rectify.py — 서로 다른 두 카메라의 이미지를 스테레오 정류(rectify)한다

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
왜 필요한가
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  조밀 스테레오(Fast-FoundationStereo 등)는 "정류된, 왜곡 없는" 쌍을 요구한다:
  에피폴라선이 수평이고 같은 행에 대응점이 놓여야 한다 (repo README 명시).
  우리 리그는 노트북 웹캠(fx 794, 77.7°)과 폰 초광각(fx 520, 101.8°)이라
  K 도 왜곡도 다르고 광축도 평행이 아니다. cv2.stereoRectify 가 두 카메라를
  공통 평면으로 회전시키고(R1, R2) 공통 투영행렬(P1, P2)을 준다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
입력 규약
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  (R, T) 는 X2 = R*X1 + T (stereoCalibrate / lib_stereo 와 동일). 단위 m.
  두 캘리브레이션의 image_size 가 같아야 한다 (한 remap 크기로 두 영상을 정류한다).
  해상도를 바꾸고 싶으면 입력 영상과 K 를 함께 바꾼 캘리브레이션을 만들어 넣을 것 —
  이 모듈은 resize 를 하지 않는다 (예전 size 인자는 stereoRectify 의 원본 크기와 출력
  크기를 혼용해 f 가 안 바뀐 채 잘리는 버그였다. doc_DEPTH_CODE_REVIEW R10).

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
좌우 방향과 부호 (R05)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  정류 후 P2[0,3] = f * Tx (Tx: 정류 좌표계에서 카메라2 의 x 위치). 카메라2 가
  오른쪽이면 Tx < 0 이고 "왼쪽 x - 오른쪽 x" 시차가 양수다. 조밀 스테레오 모델은 이
  규약(왼쪽 영상, 양의 시차)만 안다. Tx >= 0 (2번이 왼쪽) 이거나 P2[1,3] != 0 (상하
  배치) 이면 abs 로 숨기지 않고 즉시 실패시킨다 — 그대로 진행하면 Q 재투영은 -Z 를,
  수식은 +Z 를 내는 모순이 생긴다 (재현: reproduce.py rectify_reversed / rectify_vertical).

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
깊이
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Z_rect = f * baseline / d,  baseline = -Tx.  d 가 유한·양수가 아니면 NaN.
  alpha=0 : 유효 화소만 남게 자른다. 화각이 크게 다른 두 카메라면 좁은 쪽 시야로 잘린다.
"""
import cv2
import numpy as np


def assert_size(img, calib, name):
    hw = (img.shape[1], img.shape[0])
    if tuple(calib["image_size"]) != hw:
        raise ValueError(f"{name} 이미지 {hw} 가 캘리 해상도 {tuple(calib['image_size'])} 와 다름")


def rectify_maps(calibL, calibR, R, T, alpha=0.0):
    """정류 파라미터와 remap 테이블. 한 번 만들어 계속 쓴다."""
    size = tuple(calibL["image_size"])
    if tuple(calibR["image_size"]) != size:
        raise ValueError(f"두 캘리 해상도가 다름 L {size} R {tuple(calibR['image_size'])}")
    R = np.asarray(R, np.float64); T = np.asarray(T, np.float64).reshape(3, 1)
    if not np.linalg.norm(T) > 0:
        raise ValueError("T 가 0 — baseline 없음")
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        calibL["K"], calibL["dist"], calibR["K"], calibR["dist"], size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=alpha)
    f = float(P1[0, 0])
    tx = float(P2[0, 3]) / f
    ty = float(P2[1, 3]) / float(P2[1, 1])
    if abs(ty) > 1e-9 and abs(tx) < 1e-9:
        raise ValueError("상하 배치(수직 스테레오)로 정류됨 — 조밀 스테레오 입력 규약(수평)에 맞지 않음")
    if tx >= 0:
        raise ValueError(
            f"정류 후 2번 카메라가 왼쪽에 있음 (Tx={tx*1000:+.1f}mm). 시차 규약은 2번이 오른쪽. "
            f"--index-l/--index-r 을 바꿔 찍거나 촬영 위치를 바꿀 것")
    mapLx, mapLy = cv2.initUndistortRectifyMap(calibL["K"], calibL["dist"], R1, P1, size, cv2.CV_32FC1)
    mapRx, mapRy = cv2.initUndistortRectifyMap(calibR["K"], calibR["dist"], R2, P2, size, cv2.CV_32FC1)
    return dict(R1=R1, R2=R2, P1=P1, P2=P2, Q=Q, roi1=tuple(int(v) for v in roi1),
                roi2=tuple(int(v) for v in roi2), size=size,
                mapL=(mapLx, mapLy), mapR=(mapRx, mapRy),
                f=f, cx=float(P1[0, 2]), cy=float(P1[1, 2]), tx_m=tx, baseline_m=-tx)


def rectify_pair(imgL, imgR, rp):
    """두 이미지를 정류한다."""
    interp = cv2.INTER_NEAREST if imgL.ndim == 2 else cv2.INTER_LINEAR
    return (cv2.remap(imgL, *rp["mapL"], interpolation=interp),
            cv2.remap(imgR, *rp["mapR"], interpolation=interp))


def rectify_mask(mask, rp):
    """왼쪽 이미지 좌표의 HxW 마스크를 정류 왼쪽 좌표로 옮긴다 (최근접 보간)."""
    return cv2.remap(mask, *rp["mapL"], interpolation=cv2.INTER_NEAREST)


def disparity_to_depth(disp, rp):
    """정류 좌표계 시차(px) -> 깊이(m). 유한·양수가 아닌 시차는 NaN (inf 도 NaN, 0 이 아니다)."""
    d = np.asarray(disp, np.float32)
    Z = np.full(d.shape, np.nan, np.float32)
    m = np.isfinite(d) & (d > 0)
    Z[m] = rp["f"] * rp["baseline_m"] / d[m]
    return Z


def depth_to_points_cam1(depth, rp):
    """정류-왼쪽 좌표의 깊이맵 -> 원래 카메라1 좌표계의 (H,W,3) 점.

    정류 좌표계의 점 X_rect 를 R1^T 로 돌리면 카메라1 좌표계가 된다
    (stereoRectify 의 R1 은 카메라1 -> 정류 회전이므로).
    """
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    X = (u - rp["cx"]) / rp["f"] * depth
    Y = (v - rp["cy"]) / rp["f"] * depth
    pts = np.stack([X, Y, depth], axis=-1)
    return pts @ np.asarray(rp["R1"], np.float32)      # (R1^T @ p)^T = p^T @ R1


def pixel_rays_cam1(rp, uv):
    """정류-왼쪽 화소 (M,2) -> 원 카메라1 좌표계의 광선 방향 (M,3)."""
    uv = np.asarray(uv, np.float64).reshape(-1, 2)
    d = np.column_stack([(uv[:, 0] - rp["cx"]) / rp["f"], (uv[:, 1] - rp["cy"]) / rp["f"], np.ones(len(uv))])
    return d @ np.asarray(rp["R1"], np.float64)


def epipolar_check(rectL, rectR, ratio=0.75, n_max=400):
    """정류가 맞는지 수치로: 대응점 dy 가 0 에 가까워야 한다.

    반환 dict(status, n, dy_med, dy_rms, dy_rms_inlier, inlier_frac, pos_dx_frac)
      status : "ok" | "unavailable" (대응점 8개 미만 — 텍스처 부족. 실패가 아니라 판정 불가)
      dy 는 ratio test 통과점 중 dx>0 인 것만 쓴다. 오매칭 몇 개에 RMS 가 민감하므로
      dy_med 를 우선 보고 RMS 는 보조로 본다. 이 검사는 회전·정류 일치만 보며
      baseline 의 미터 스케일은 검증하지 못한다.
    """
    sift = cv2.SIFT_create(n_max)
    gL = rectL if rectL.ndim == 2 else cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    gR = rectR if rectR.ndim == 2 else cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)
    kL, dL = sift.detectAndCompute(gL, None)
    kR, dR = sift.detectAndCompute(gR, None)
    out = dict(status="unavailable", n=0, dy_med=None, dy_rms=None, dy_rms_inlier=None,
               inlier_frac=None, pos_dx_frac=None)
    if dL is None or dR is None or len(kL) < 2 or len(kR) < 2:
        return out
    good = [p[0] for p in cv2.BFMatcher(cv2.NORM_L2).knnMatch(dL, dR, k=2)
            if len(p) == 2 and p[0].distance < ratio * p[1].distance]     # k=2 인데 1개만 올 수 있다
    if len(good) < 8:
        out["n"] = len(good)
        return out
    dy = np.array([kL[m.queryIdx].pt[1] - kR[m.trainIdx].pt[1] for m in good])
    dx = np.array([kL[m.queryIdx].pt[0] - kR[m.trainIdx].pt[0] for m in good])
    pos = dx > 0
    out["pos_dx_frac"] = float(pos.mean())
    if pos.sum() >= 8:
        dy = dy[pos]
    inl = np.abs(dy) < 2.0
    out.update(status="ok", n=int(len(dy)), dy_med=float(np.median(np.abs(dy))),
               dy_rms=float(np.sqrt(np.mean(dy ** 2))),
               # 오매칭 몇 개가 RMS 를 끌어올린다 (합성 정류쌍에서 중앙값 0.23px 인데 RMS 3.6px).
               # 합격 판정은 중앙값 + 인라이어 비율로 하고 RMS 는 참고로만 둔다.
               dy_rms_inlier=float(np.sqrt(np.mean(dy[inl] ** 2))) if inl.any() else None,
               inlier_frac=float(inl.mean()))
    return out
