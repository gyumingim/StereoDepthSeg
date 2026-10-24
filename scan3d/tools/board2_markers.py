# Two big ArUco markers (laptop 1920x1200): detection + pose for the calibration pages (2026-10-24, user: "큰마커 2개").
# Layout: DICT_6X6_250 ids 0 (left) and 1 (right), side 700 px, gap 160 px, top-left at (180,250) and (1040,250).
# Physical: 700 px x 0.1570 mm/px (Lenovo 14" 1920x1200 panel, spec estimate) = 109.9 mm at 1:1 pixels.
import cv2, numpy as np, math

SIDE_PX, GAP_PX, X0, Y0 = 700, 160, 180, 250
MM_PER_PX = 0.1570
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)


def make_image():
    img = np.full((1200, 1920), 255, np.uint8)
    for i, x in enumerate([X0, X0 + SIDE_PX + GAP_PX]):
        img[Y0:Y0 + SIDE_PX, x:x + SIDE_PX] = cv2.aruco.generateImageMarker(D, i, SIDE_PX, borderBits=1)
    return img


def obj_points():
    """Board-frame 3D corners (metres) per marker id, OpenCV corner order (TL, TR, BR, BL), y down like the image."""
    s = SIDE_PX * MM_PER_PX / 1000.0
    out = {}
    for i, x in enumerate([X0, X0 + SIDE_PX + GAP_PX]):
        x0, y0 = x * MM_PER_PX / 1000.0, Y0 * MM_PER_PX / 1000.0
        out[i] = np.array([[x0, y0, 0], [x0 + s, y0, 0], [x0 + s, y0 + s, 0], [x0, y0 + s, 0]], np.float64)
    return out


_P = cv2.aruco.DetectorParameters()
_P.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
_DET = cv2.aruco.ArucoDetector(D, _P)
OBJ = obj_points()
CENTRE = np.array([(X0 + (2 * SIDE_PX + GAP_PX) / 2) * MM_PER_PX / 1000.0, (Y0 + SIDE_PX / 2) * MM_PER_PX / 1000.0, 0.0])


def detect(gray, K):
    """-> None or dict(n_markers, img_pts (N,2), tilt, turn, lean, dist_m, poly (outline of both markers))."""
    corners, ids, _ = _DET.detectMarkers(gray)
    if ids is None:
        return None
    obj, img = [], []
    for c, i in zip(corners, ids.ravel()):
        if int(i) in OBJ:
            obj.append(OBJ[int(i)]); img.append(c.reshape(4, 2))
    if not obj:
        return None
    obj = np.concatenate(obj); img = np.concatenate(img).astype(np.float64)
    ok, rv, tv = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rv); nr = R[:, 2]
    tilt = math.degrees(math.acos(abs(nr[2])))
    turn = math.degrees(math.atan2(nr[0], abs(nr[2]))); lean = math.degrees(math.atan2(nr[1], abs(nr[2])))
    dist = float(np.linalg.norm(R @ CENTRE + tv.ravel()))
    return dict(n_markers=len(obj) // 4, img_pts=img, tilt=tilt, turn=turn, lean=lean, dist_m=dist, rv=rv, tv=tv)


if __name__ == "__main__":
    import sys
    cv2.imwrite(sys.argv[1], make_image())
