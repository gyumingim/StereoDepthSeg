# Two big ArUco markers (laptop 1920x1200): detection + pose for the calibration pages (2026-10-24, user: "큰마커 2개").
# Layout: DICT_6X6_250 ids 0 (left) and 1 (right), side 780 px, gap 120 px, top-left at (120,100) and (1020,100).
# Physical: 780 px x 0.1570 mm/px (Lenovo 14" 1920x1200 panel, spec estimate) = 122.5 mm at 1:1 pixels.
import cv2, numpy as np, math

SIDE_PX, GAP_PX, X0, Y0 = 780, 120, 120, 100   # full width, top; bottom 220 px = live view + text
MM_PER_PX = 0.15756   # ruler 7 x 272 px = 30 cm (user, 2026-10-21)
try:      # set from the board page's card check (credit card 85.60 mm on the screen) -> board2_scale.json next to this file
    import json as _j, os as _o
    MM_PER_PX = float(_j.load(open(_o.path.join(_o.path.dirname(_o.path.abspath(__file__)), "board2_scale.json")))["mm_per_px"])
except Exception:
    pass
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


def set_scale(mm_per_px):
    """Change the physical scale at runtime (the page's card check)."""
    global MM_PER_PX, OBJ, CENTRE
    MM_PER_PX = float(mm_per_px); OBJ = obj_points()
    CENTRE = np.array([(X0 + (2 * SIDE_PX + GAP_PX) / 2) * MM_PER_PX / 1000.0, (Y0 + SIDE_PX / 2) * MM_PER_PX / 1000.0, 0.0])


_P = cv2.aruco.DetectorParameters()
_P.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
_DET = cv2.aruco.ArucoDetector(D, _P)
OBJ = obj_points()
CENTRE = np.array([(X0 + (2 * SIDE_PX + GAP_PX) / 2) * MM_PER_PX / 1000.0, (Y0 + SIDE_PX / 2) * MM_PER_PX / 1000.0, 0.0])


def detect(gray, K, prev_normal=None):
    """-> None or dict(n_markers, img_pts (N,2), tilt, turn, lean, dist_m, ambiguous, normal).
    Planar PnP has two mirror solutions (tilt +a vs -a); when the 2nd solution's reprojection error is within 2x of the
    best, the pose is ambiguous -> pick the one closer to prev_normal (temporal consistency) and flag it."""
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
    n, rvs, tvs, errs = cv2.solvePnPGeneric(obj, img, K, None, flags=cv2.SOLVEPNP_IPPE)
    if not n:
        return None
    errs = np.asarray(errs).ravel()
    sols = []
    for rv, tv, e in zip(rvs, tvs, errs):
        R, _ = cv2.Rodrigues(rv); sols.append((float(e), R, rv, tv))
    sols.sort(key=lambda x: x[0])
    ambiguous = len(sols) > 1 and sols[1][0] < 2.0 * max(sols[0][0], 1e-6)
    pick = sols[0]
    if ambiguous and prev_normal is not None:
        pick = max(sols[:2], key=lambda x: float(np.dot(x[1][:, 2], prev_normal)))
    e, R, rv, tv = pick; nr = R[:, 2]
    tilt = math.degrees(math.acos(min(1.0, abs(nr[2]))))
    turn = math.degrees(math.atan2(nr[0], abs(nr[2]))); lean = math.degrees(math.atan2(nr[1], abs(nr[2])))
    dist = float(np.linalg.norm(R @ CENTRE + tv.ravel()))
    return dict(n_markers=len(obj) // 4, img_pts=img, tilt=tilt, turn=turn, lean=lean, dist_m=dist, rv=rv, tv=tv,
                ambiguous=bool(ambiguous), normal=nr.copy(), err=e)


if __name__ == "__main__":
    import sys
    cv2.imwrite(sys.argv[1], make_image())
