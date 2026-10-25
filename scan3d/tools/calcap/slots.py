"""Coverage slots + board pose for the automatic lens-calibration capture (calcap, 2026-10-25).

All pixel numbers are in FULL-RES 1920x1080 image units (the VIO stream). Shared by calcap_server.py (live guidance on
the 960x540 preview), calcap_calib.py (re-check on the saved stills) and make_dry_set.py (synthetic test set).

Slot = (where the board's inner corners sit in the image) x (board tilt) x (distance bin):
  F-<pos>       frontal (tilt <= 12 deg) at 9 positions (C, T, B, L, R, TL, TR, BL, BR), distance 0.55-1.35 m
  T20-<ax><s>-<near|far>  centre, ~20 deg (14-30) turn+/turn-/lean+/lean- (other axis <= 12), near 0.55-0.95 / far 0.95-1.35
  T40-<ax><s>   centre, ~40 deg (32-55) turn+/-, lean+/-  (other axis <= 15)
  E40-<pos>-<ax><s>  edges: L/R with turn+/-, T/B with lean+/-  (~40 deg)
  K40-<pos>     corners TL/TR/BL/BR, tilt 30-55 in any direction
Position: centre of the detected-corner bounding box in the 3x3 grid (thirds of 1920 / 1080); every non-centre position
must also reach the named frame edge(s): gap between the corner bbox and that edge <= EDGE_PX.
Distance window: 0.55-1.35 m (focus 0.5 dpt; estimated depth of field ~0.9 m..inf, so near shots are slightly soft -
the calibration report splits fx by distance bin to show any effect).
turn = atan2(n_x, |n_z|), lean = atan2(n_y, |n_z|), tilt = acos(|n_z|) with n = board normal in camera axes
(same convention as the old board server shoot_server.py).
"""
import json
import math
import os

import cv2
import numpy as np

W, H = 1920, 1080
EDGE_PX = 250
GOAL_GAP = 150.0      # target outline: inner-corner bbox this far from the named edges
DMIN, DMAX, DSPLIT = 0.55, 1.35, 0.95
POS_NAME = {"C": "가운데", "T": "위 가장자리", "B": "아래 가장자리", "L": "왼쪽 가장자리", "R": "오른쪽 가장자리",
            "TL": "왼쪽 위 귀퉁이", "TR": "오른쪽 위 귀퉁이", "BL": "왼쪽 아래 귀퉁이", "BR": "오른쪽 아래 귀퉁이"}
AX_NAME = {"turn": "좌우로 돌림", "lean": "앞뒤로 젖힘"}


def make_slots():
    s = []
    for p in ["C", "T", "B", "L", "R", "TL", "TR", "BL", "BR"]:
        s.append(dict(id=f"F-{p}", pos=p, kind="F", label=f"정면 · {POS_NAME[p]}"))
    for ax in ("turn", "lean"):
        for sg in (1, -1):
            for db in ("near", "far"):
                s.append(dict(id=f"T20-{ax}{'+' if sg > 0 else '-'}-{db}", pos="C", kind="T20", ax=ax, sign=sg, dbin=db,
                              label=f"가운데 · {AX_NAME[ax]} 20° ({'+' if sg > 0 else '−'}) · {'가까이' if db == 'near' else '멀리'}"))
    for ax in ("turn", "lean"):
        for sg in (1, -1):
            s.append(dict(id=f"T40-{ax}{'+' if sg > 0 else '-'}", pos="C", kind="T40", ax=ax, sign=sg,
                          label=f"가운데 · {AX_NAME[ax]} 40° ({'+' if sg > 0 else '−'})"))
    for p, ax in (("L", "turn"), ("R", "turn"), ("T", "lean"), ("B", "lean")):
        for sg in (1, -1):
            s.append(dict(id=f"E40-{p}-{ax}{'+' if sg > 0 else '-'}", pos=p, kind="T40", ax=ax, sign=sg,
                          label=f"{POS_NAME[p]} · {AX_NAME[ax]} 40° ({'+' if sg > 0 else '−'})"))
    for p in ("TL", "TR", "BL", "BR"):
        s.append(dict(id=f"K40-{p}", pos=p, kind="K40", label=f"{POS_NAME[p]} · 아무 방향 40°"))
    return s


SLOTS = make_slots()
SLOT_BY_ID = {s["id"]: s for s in SLOTS}


# ---------------------------------------------------------------- boards
class Board:
    """ChArUco board from config/charuco/<name>.json (same spec as aruco_scale: legacy_pattern key honoured)."""

    def __init__(self, path):
        j = json.load(open(path, encoding="utf-8"))
        self.name = os.path.splitext(os.path.basename(path))[0]
        self.spec = j
        self.sx, self.sy = j["squares_x"], j["squares_y"]
        self.sq = j["square_mm"] / 1000.0
        D = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, j["dict"]))
        self.board = cv2.aruco.CharucoBoard((self.sx, self.sy), self.sq, self.sq * j["marker_ratio"], D)
        legacy = j.get("legacy_pattern")
        if legacy is None:
            legacy = self.sy % 2 == 0
        self.board.setLegacyPattern(bool(legacy))
        cp = cv2.aruco.CharucoParameters(); cp.minMarkers = 1; cp.tryRefineMarkers = True
        dp = cv2.aruco.DetectorParameters(); dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        dp.minMarkerPerimeterRate = 0.01
        self.det = cv2.aruco.CharucoDetector(self.board, cp, dp, cv2.aruco.RefineParameters())
        self.obj = np.asarray(self.board.getChessboardCorners(), np.float64).reshape(-1, 3)
        self.n = len(self.obj)
        self.centre = np.array([self.sx * self.sq / 2, self.sy * self.sq / 2, 0.0])
        nx, ny = self.sx - 1, self.sy - 1           # inner-corner grid; extreme corners for the outline
        self.outline_ids = [0, nx - 1, nx * ny - 1, nx * (ny - 1)]
        self.min_corners = max(6, int(math.ceil(0.7 * self.n)))

    def detect(self, gray):
        cc, ci, mc, mi = self.det.detectBoard(gray)
        if ci is None or len(ci) == 0:
            return np.zeros((0, 2)), np.zeros(0, int)
        return np.asarray(cc, np.float64).reshape(-1, 2), np.asarray(ci, int).reshape(-1)


def load_boards(charuco_dir, names=("board_laptop_5x3", "board_laptop_6x4")):
    return [Board(os.path.join(charuco_dir, n + ".json")) for n in names]


def load_lens(path):
    j = json.load(open(path, encoding="utf-8"))
    return np.array(j["K"], np.float64), np.array(j["dist"], np.float64)


# ---------------------------------------------------------------- pose
def pose_of(board, corners, ids, K, dist, scale=1.0):
    """corners in an image scaled by `scale` vs full res -> dict in FULL-RES units, or None.
    K / dist are full-res; corners are mapped to full res first (x / scale)."""
    if len(ids) < board.min_corners:
        return None
    c = corners / scale
    obj = board.obj[ids]
    if min(np.ptp(obj[:, 0]), np.ptp(obj[:, 1])) < 0.5 * board.sq:
        return None
    try:
        n, rvs, tvs, errs = cv2.solvePnPGeneric(obj, c.reshape(-1, 1, 2), K, dist, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None
    if not n:
        return None
    best = int(np.argmin(np.asarray(errs).ravel()))
    rv, tv = cv2.solvePnPRefineLM(obj, c.reshape(-1, 1, 2), K, dist, rvs[best], tvs[best])
    R, _ = cv2.Rodrigues(rv)
    t = tv.ravel()
    if t[2] <= 0:
        return None
    nr = R[:, 2]
    tilt = math.degrees(math.acos(min(1.0, abs(nr[2]))))
    turn = math.degrees(math.atan2(nr[0], abs(nr[2])))
    lean = math.degrees(math.atan2(nr[1], abs(nr[2])))
    d = float(np.linalg.norm(R @ board.centre + t))
    x0, y0 = c.min(0)
    x1, y1 = c.max(0)
    proj, _ = cv2.projectPoints(obj, rv, tv, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - c) ** 2, 1))))
    return dict(board=board.name, n=int(len(ids)), tilt=tilt, turn=turn, lean=lean, dist=d,
                bbox=[float(x0), float(y0), float(x1), float(y1)], cx=float((x0 + x1) / 2), cy=float((y0 + y1) / 2),
                edge=dict(L=float(x0), R=float(W - x1), T=float(y0), B=float(H - y1)), rms=rms,
                rvec=rv.ravel().tolist(), tvec=t.tolist(), pos=pos_of(float((x0 + x1) / 2), float((y0 + y1) / 2)))


def pos_of(cx, cy):
    v = "T" if cy < H / 3 else ("B" if cy > 2 * H / 3 else "")
    h = "L" if cx < W / 3 else ("R" if cx > 2 * W / 3 else "")
    return (v + h) or "C"


def detect_pose(boards, gray, K, dist, scale=1.0):
    """First board that gives a pose -> (pose, corners_in_given_image, ids, board) or (None, ...)."""
    best = (None, None, None, None)
    for b in boards:
        c, i = b.detect(gray)
        if len(i) == 0:
            continue
        p = pose_of(b, c, i, K, dist, scale)
        if p is not None:
            return p, c, i, b
        if best[1] is None or len(i) > len(best[2]):
            best = (None, c, i, b)
    return best


# ---------------------------------------------------------------- slot rules
def fills(slot, p, tol=0.0):
    """Does pose p fill the slot? tol widens every window (degrees / 100 px / 0.05 m per 1.0) for the still re-check."""
    if p is None:
        return False
    if not (DMIN - 0.05 * tol <= p["dist"] <= DMAX + 0.05 * tol):
        return False
    if p["pos"] != slot["pos"]:
        return False
    for side in slot["pos"].replace("C", ""):
        if p["edge"][side] > EDGE_PX + 100 * tol:
            return False
    k = slot["kind"]
    t = 3 * tol
    if k == "F":
        return p["tilt"] <= 12 + t
    if k == "K40":
        return 30 - t <= p["tilt"] <= 55 + t
    a, o = (p["turn"], p["lean"]) if slot["ax"] == "turn" else (p["lean"], p["turn"])
    a *= slot["sign"]
    if k == "T20":
        if not (14 - t <= a <= 30 + t and abs(o) <= 12 + t):
            return False
        return (p["dist"] < DSPLIT + 0.05 * tol) if slot["dbin"] == "near" else (p["dist"] >= DSPLIT - 0.05 * tol)
    if k == "T40":
        return 32 - t <= a <= 55 + t and abs(o) <= 15 + t
    return False


def target_pose(slot):
    """A representative pose for the slot (for the yellow target outline): (turn, lean, dist, pixel goal of the bbox)."""
    k = slot["kind"]
    turn = lean = 0.0
    if k in ("T20", "T40"):
        ang = 22.0 if k == "T20" else 40.0
        if slot["ax"] == "turn":
            turn = ang * slot["sign"]
        else:
            lean = ang * slot["sign"]
    elif k == "K40":
        turn = 30.0 * (1 if "L" in slot["pos"] else -1)
        lean = 28.0 * (1 if "T" in slot["pos"] else -1)
    d = 0.75 if slot.get("dbin") == "near" else (1.15 if slot.get("dbin") == "far" else 0.85)
    return turn, lean, d


def _R_from(turn, lean):
    """Board rotation whose normal has the given turn / lean (degrees), normal pointing along +z (away from camera)."""
    nx, ny = math.tan(math.radians(turn)), math.tan(math.radians(lean))
    n = np.array([nx, ny, 1.0]); n /= np.linalg.norm(n)
    x = np.cross([0.0, 1.0, 0.0], n); x /= np.linalg.norm(x)
    y = np.cross(n, x)
    return np.stack([x, y, n], 1)


def target_Rt(slot, board, K, dist):
    """Board pose (R, t: X_cam = R X_board + t) at the slot's target: target_pose angles / distance, outline pushed to
    GOAL_GAP px from the named frame edges (the outer squares / markers stay mostly in view)."""
    turn, lean, d = target_pose(slot)
    R = _R_from(turn, lean)
    rv = cv2.Rodrigues(R)[0]
    goal = dict(L=GOAL_GAP, R=W - GOAL_GAP, T=GOAL_GAP, B=H - GOAL_GAP)
    px, py = W / 2, H / 2
    for _ in range(5):
        ray = np.linalg.solve(K, [px, py, 1.0]); ray /= np.linalg.norm(ray)
        t = ray * d - R @ board.centre
        pts, _ = cv2.projectPoints(board.obj, rv, t, K, dist)
        pts = pts.reshape(-1, 2)
        x0, y0 = pts.min(0); x1, y1 = pts.max(0)
        p = slot["pos"]
        if "L" in p: px += goal["L"] - x0
        if "R" in p: px += goal["R"] - x1
        if "T" in p: py += goal["T"] - y0
        if "B" in p: py += goal["B"] - y1
    return R, t


def target_outline(slot, board, K, dist):
    """Projected outline (4 extreme inner corners, full-res px) of the board at the slot's target pose."""
    R, t = target_Rt(slot, board, K, dist)
    pts, _ = cv2.projectPoints(board.obj[board.outline_ids], cv2.Rodrigues(R)[0], t, K, dist)
    return pts.reshape(-1, 2)


def nearest_missing(missing, p):
    """Missing slot closest to the current pose (position first, then angle) - the one to suggest."""
    if not missing:
        return None
    if p is None:
        return missing[0]

    def cost(s):
        c = 0.0 if s["pos"] == p["pos"] else (1.0 if (set(s["pos"]) & set(p["pos"])) else 2.0)
        tu, le, d = target_pose(s)
        c += (abs(tu - p["turn"]) + abs(le - p["lean"])) / 25.0 + abs(d - p["dist"]) / 0.4
        return c
    return min(missing, key=cost)


def hint(slot, p):
    """Short Korean hint for the phone / laptop text."""
    if p is None:
        return "보드가 안 보여요"
    out = []
    if p["dist"] < DMIN or (slot.get("dbin") == "far" and p["dist"] < DSPLIT):
        out.append("더 멀리")
    elif p["dist"] > DMAX or (slot.get("dbin") == "near" and p["dist"] >= DSPLIT):
        out.append("더 가까이")
    if p["pos"] != slot["pos"]:
        out.append("노란 틀 위치로")
    else:
        for side in slot["pos"].replace("C", ""):
            if p["edge"][side] > EDGE_PX:
                out.append("화면 끝까지 더")
                break
    k = slot["kind"]
    if k == "F" and p["tilt"] > 12:
        out.append(f"정면으로 (지금 {p['tilt']:.0f}°)")
    elif k == "K40" and not (30 <= p["tilt"] <= 55):
        out.append(f"40° 기울이기 (지금 {p['tilt']:.0f}°)")
    elif k in ("T20", "T40"):
        a = (p["turn"] if slot["ax"] == "turn" else p["lean"]) * slot["sign"]
        lo, hi = (14, 30) if k == "T20" else (32, 55)
        if not (lo <= a <= hi):
            out.append(f"노란 틀 모양처럼 기울이기 ({a:+.0f}° → {lo}~{hi}°)")
    return " · ".join(out) if out else "그대로 멈추기"
