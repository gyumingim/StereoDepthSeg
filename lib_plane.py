"""
lib_plane.py — 체커보드가 정의한 평면 위 물체의 3D 위치·크기 (카메라 1대, 실시간)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
왜 이게 카메라 1대로 되는가
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  삼각측량은 두 시점이 필요하다. 고정된 카메라 1대로는 시차가 0이라 깊이가 안 나온다.
  그런데 "물체가 어떤 평면 위에 있다"는 조건이 하나 더 있으면 얘기가 다르다.
  픽셀 하나는 3D 공간의 "광선" 하나를 정하는데, 그 광선이 평면과 만나는 점은
  단 하나로 결정된다. 미지수 3개(X,Y,Z)에 식이 3개(광선 2 + 평면 1)가 되는 셈이다.
  체커보드는 실치수를 아는 평면이므로 그 평면을 미터 단위로 정의해 준다.

  -> 물체가 책상(=체커보드와 같은 평면)에 놓여 있는 한, 한 장으로 정확히 나온다.
  -> 공중에 뜬 물체나 다른 평면 위의 물체는 이 방법으로 못 잰다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
유도
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  solvePnP 가 보드->카메라 (R, t) 를 준다. 보드 평면은 보드 좌표계에서 z=0 이므로
  카메라 좌표계에서:
      법선   n = R @ [0,0,1]
      평면 위 한 점 p = t                      (보드 원점)
      평면   n . X = n . p

  픽셀 (u,v) 의 광선 방향 (카메라 좌표계):
      d = [ (u-cx)/fx, (v-cy)/fy, 1 ]
      X = s*d  이고  n.(s*d) = n.p  ->  s = (n.p) / (n.d)
      s > 0 이어야 카메라 앞이다.

  높이: 평면에서 위로 향하는 단위벡터 u_up (아래 부호 결정 참조).
      물체 밑점 B 위로 h 만큼 올라간 점 P(h) = B + h*u_up 이 박스 윗변에 투영된다.
          y_top = fy * P_y / P_z + cy
      정리하면
          h = ((y_top-cy)*B_z - fy*B_y) / (fy*u_y - (y_top-cy)*u_z)
      분모가 0에 가까우면 높이가 관측 불가다(카메라가 평면을 정면으로 내려다볼 때).
      그래서 분모 크기를 height_cond 로 함께 돌려준다.

  u_up 부호: n 은 보드의 +z 방향인데 보드를 뒤집어 놓았을 수도 있다.
      카메라는 원점에 있고 보드는 t 에 있으므로 보드에서 카메라로 가는 방향은 -t.
      n 이 카메라 쪽을 향하면(n . (-t) > 0) 그쪽이 "위"다. 아니면 -n 이 위다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
단위 규약
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  전부 미터. 카메라 광학 좌표계(X 우, Y 하, Z 전방) 기준.
"""
import cv2
import numpy as np


def board_plane(calib, corners, square_m, pattern):
    """검출된 체커보드 코너 -> 카메라 좌표계에서의 평면.

    반환: dict 또는 None(solvePnP 실패)
    """
    import lib_calib as _lc
    objp = _lc.object_points(pattern, square_m)
    ok, rvec, tvec = cv2.solvePnP(objp, corners, calib["K"], calib["dist"],
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R = cv2.Rodrigues(rvec)[0]
    t = tvec.reshape(3)
    n = R[:, 2]                                  # R @ [0,0,1]
    up = n if float(n @ (-t)) > 0 else -n        # 카메라 쪽을 향하는 쪽이 "위"

    proj, _ = cv2.projectPoints(objp, rvec, tvec, calib["K"], calib["dist"])
    err = float(np.sqrt(np.mean(np.sum(
        (proj.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1))))
    # 표시용 거리는 보드 "중심" 기준. t 는 보드 원점(좌상단 코너)이라
    # 그대로 쓰면 보드 크기의 절반만큼 어긋나 보인다.
    center = R @ objp.mean(axis=0) + t
    return dict(n=n, p=t, up=up, R=R, t=t,
                board_dist_m=float(np.linalg.norm(center)),
                pnp_reproj_px=err)


def _normalized(K, dist, uv):
    """픽셀 -> 왜곡 보정된 정규화 좌표 (x, y). dist 가 None 이면 핀홀로만 본다."""
    if dist is None:
        return (uv[0] - K[0, 2]) / K[0, 0], (uv[1] - K[1, 2]) / K[1, 1]
    n = cv2.undistortPoints(np.array([[[float(uv[0]), float(uv[1])]]], np.float64), K, dist)
    return float(n[0, 0, 0]), float(n[0, 0, 1])


def ray_plane(K, uv, plane, dist=None):
    """픽셀 -> 평면 위 3D 점 (카메라 좌표계). 카메라 뒤거나 평행하면 None.

    dist 를 주면 왜곡을 보정한 광선을 쓴다. 안 주면 원영상 화소를 핀홀로 해석해
    가장자리에서 편향이 생긴다 (doc_DEPTH_CODE_REVIEW R12).
    """
    xn, yn = _normalized(K, dist, uv)
    d = np.array([xn, yn, 1.0])
    nd = float(plane["n"] @ d)
    if abs(nd) < 1e-9:
        return None
    s = float(plane["n"] @ plane["p"]) / nd
    if s <= 0:
        return None
    return s * d


def object_from_box(K, box, plane, dist_coeffs=None):
    """YOLO 박스 -> 평면 위 물체의 3D 위치와 크기.

    박스의 아랫변이 물체가 평면에 닿는 선이라고 본다.
    반환: dict 또는 None(평면과 안 만남)
    """
    x, y, w, h = (float(v) for v in box)
    yb = y + h
    pts = {}
    for name, uv in (("base_c", (x + w / 2, yb)), ("base_l", (x, yb)),
                     ("base_r", (x + w, yb))):
        P = ray_plane(K, uv, plane, dist_coeffs)
        if P is None:
            return None
        pts[name] = P

    B = pts["base_c"]
    fy = K[1, 1]
    up = plane["up"]
    # 높이식을 정규화 좌표로: yn = (y_top-cy)/fy (왜곡 보정 포함)
    #   h = (yn*B_z - B_y) / (u_y - yn*u_z)     (docstring 유도식의 분자·분모를 fy 로 나눈 것)
    _, yn = _normalized(K, dist_coeffs, (x + w / 2, y))
    denom = up[1] - yn * up[2]
    height = float((yn * B[2] - B[1]) / denom) if abs(denom) > 1e-9 else float("nan")

    return dict(
        base=B,
        distance_m=float(np.linalg.norm(B)),          # 카메라 중심에서의 거리
        depth_m=float(B[2]),                          # 광축 방향 깊이
        width_m=float(np.linalg.norm(pts["base_r"] - pts["base_l"])),
        height_m=height,
        height_cond=float(abs(denom) * fy),           # 예전 스케일(px) 유지. 작을수록 높이를 못 믿는다
    )
