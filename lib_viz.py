"""
lib_viz.py — 복원 결과를 Open3D 3D 맵에 배치해 보여준다

맵 좌표계는 lib_stereo.CAM_TO_MAP 로 변환된 Z-up 계다 (X 오른쪽, Y 전방, Z 위).
여러 번 복원한 객체를 리스트로 넘기면 같은 맵에 함께 배치된다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
카메라 프러스텀의 extrinsic 유도
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  create_camera_visualization(w, h, intrinsic, extrinsic, scale) 의 extrinsic 은
  월드->카메라 4x4 다 (lineset.cpp:85-90).
  여기서 월드는 맵 좌표계이고, X_map = CAM_TO_MAP @ X_cam1 이므로
      X_cam1 = CAM_TO_MAP^T @ X_map                  -> E1 = [CAM_TO_MAP^T | 0]
      X_cam2 = R @ X_cam1 + T = R CAM_TO_MAP^T X_map + T
                                                     -> E2 = [R CAM_TO_MAP^T | T]
"""
import numpy as np
import open3d as o3d

from lib_stereo import CAM_TO_MAP

CAM_COLORS = ((0.10, 0.60, 1.00), (1.00, 0.55, 0.10))   # 왼쪽/오른쪽 촬영 위치
OBJ_COLORS = ((0.95, 0.25, 0.25), (0.25, 0.85, 0.35), (0.85, 0.75, 0.20),
              (0.60, 0.40, 0.95), (0.25, 0.80, 0.85))


def ground_grid(half=2.0, step=0.25, z=0.0):
    """맵 바닥 격자. 빈 3D 뷰에서는 격자가 없으면 거리·크기를 눈으로 읽을 수 없다."""
    n = int(half / step)
    pts, lines = [], []
    for i in range(-n, n + 1):
        t = i * step
        pts += [[t, -half, z], [t, half, z], [-half, t, z], [half, t, z]]
        lines += [[len(pts) - 4, len(pts) - 3], [len(pts) - 2, len(pts) - 1]]
    ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(np.array(pts, float)),
                              o3d.utility.Vector2iVector(np.array(lines, int)))
    ls.paint_uniform_color((0.75, 0.75, 0.75))
    return ls


def camera_frustums(K, image_size, R, T, scale=0.12):
    """두 촬영 위치를 맵 좌표계에 그린다 (파랑=왼쪽/1번, 주황=오른쪽/2번)."""
    w, h = image_size
    E1, E2 = np.eye(4), np.eye(4)
    E1[:3, :3] = CAM_TO_MAP.T
    E2[:3, :3] = np.asarray(R) @ CAM_TO_MAP.T
    E2[:3, 3] = np.asarray(T).ravel()
    out = []
    for E, c in ((E1, CAM_COLORS[0]), (E2, CAM_COLORS[1])):
        ls = o3d.geometry.LineSet.create_camera_visualization(
            int(w), int(h), np.asarray(K), E, scale)
        ls.paint_uniform_color(c)
        out.append(ls)
    return out


def build(objects, K=None, image_size=None, R=None, T=None,
          grid=True, axis=0.2):
    """표시할 Open3D 지오메트리 목록을 만든다.

    objects : [dict(points_map=(N,3), box=dict|None), ...]
    K/image_size/R/T 를 주면 촬영 위치 두 개도 함께 그린다.
    """
    geoms = []
    if grid:
        geoms.append(ground_grid())
    if axis:
        geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis))
    if K is not None and R is not None:
        geoms += camera_frustums(K, image_size, R, T)

    for i, o in enumerate(objects):
        col = OBJ_COLORS[i % len(OBJ_COLORS)]
        pts = np.asarray(o["points_map"])
        if len(pts):
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
            pc.paint_uniform_color(col)
            geoms.append(pc)
        if o.get("box") is not None:
            b = o["box"]
            obb = o3d.geometry.OrientedBoundingBox(
                np.asarray(b["center"]).reshape(3, 1),
                np.asarray(b["R"]), np.asarray(b["extent"]).reshape(3, 1))
            obb.color = col
            geoms.append(obb)
    return geoms


def show(objects, K=None, image_size=None, R=None, T=None, title="3D map"):
    """3D 맵 창을 띄운다. 창을 닫으면 반환된다."""
    geoms = build(objects, K, image_size, R, T)
    o3d.visualization.draw_geometries(
        geoms, window_name=title, width=1280, height=800,
        # 맵 좌표계는 Z-up 이므로 up 벡터를 +Z로 주고 뒤에서 앞(+Y)을 보게 한다
        lookat=[0, 1, 0], up=[0, 0, 1], front=[0, -1, 0.35], zoom=0.7)
