"""Slot list for the printed-board lens calibration (lenskit 2026-10-27) -> config/calcap_slots/print_A1_55.json.

  ~/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/tools/calcap/make_slots_print.py [--board board_print_A1_12x9]

Source: runs/lenscal_20261027/README.md section 2 pose table (N-C, N-E, N-K, M-C, M-EK, F, N6; ~55 poses), changed:
  * distance >= 0.9 m everywhere (focus fixed at 0.5 dpt: depth of field estimated ~0.9 m .. inf, calcap README /
    lenscal research notes - an ESTIMATE, the calibration checks it with C7 near/far). README groups at 0.6-0.7 m move to
    0.9-1.15 m ("N"), 1.0 m -> 1.15-1.5 m ("M"), 1.5 m -> 1.6-2.1 m ("F"); README N6 (0.6 m) becomes N-R (roll-heavy
    poses at 0.9-1.15 m) so roll 0 / 45 / 90 deg are all present.
  * the board stands on a support (easel / tripod + ball head), the phone is fixed on a tripod (README: never hand-held).
Angles: turn = left/right (yaw), lean = up/down (pitch), roll = in-plane rotation (sign / meaning: slots.py).
Each slot keeps windows of about +-12 deg around its target and its distance band; edge / corner slots need the board
to reach that frame edge (partly outside the frame is fine for ChArUco: min_corner_frac 0.3).
Self-check: the target pose of every slot (as drawn by the guide) is projected through the lens, cropped to the frame,
re-detected as a pose (slots.pose_of) and must fill its own slot; goal_gap is lowered (board pushed further out) until
it does. A slot that cannot be filled is an error.
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import slots as SL  # noqa: E402

S3D = os.path.abspath(os.path.join(HERE, "..", ".."))
DN = dict(N=(0.9, 1.15, 1.0), M=(1.15, 1.5, 1.3), F=(1.6, 2.1, 1.8))
PN = SL.POS_NAME
WIN = 12.0


def ang_win(a):
    if a == 0:
        return [-WIN, WIN]
    lo, hi = a - WIN, a + WIN
    return [max(lo, 6.0), hi] if a > 0 else [lo, min(hi, -6.0)]


def side_text(turn, lean):
    t = []
    if turn:
        t.append(("오른쪽" if turn > 0 else "왼쪽") + f" 끝이 가깝게 {abs(turn):.0f}°")
    if lean:
        t.append(("아래쪽" if lean > 0 else "위쪽") + f" 끝이 가깝게 {abs(lean):.0f}°")
    return " + ".join(t) if t else "정면"


def slot(group, dband, pos, turn=0.0, lean=0.0, roll=0.0, extra=""):
    lo, hi, d = DN[dband]
    sid = f"{group}-{pos}-t{turn:+.0f}l{lean:+.0f}" + (f"r{roll:+.0f}" if roll else "")
    s = dict(id=sid, kind="G", group=group, pos=pos, dist=[lo, hi], turn=ang_win(turn), lean=ang_win(lean),
             target=dict(turn=turn, lean=lean, roll=roll, dist=d))
    if turn == 0 and lean == 0:
        s["tilt"] = [0.0, 12.0]
        s.pop("turn"); s.pop("lean")
    if roll == 0:
        s["roll"] = [-20.0, 20.0]
    elif abs(roll) >= 80:
        s["roll_abs"] = [68.0, 90.0]
    else:
        s["roll"] = [roll - WIN, roll + WIN]
    rl = "" if roll == 0 else f" · 판을 {abs(roll):.0f}° {'시계' if roll > 0 else '반시계'} 방향으로 돌림"
    s["label"] = f"{group} · {PN[pos]} · {lo:.2f}–{hi:.2f} m · {side_text(turn, lean)}{rl}{extra}"
    return s


def build():
    S = []
    # N-C: centre, near (0.9-1.15 m): yaw +-45, pitch +-45, 4 diagonals, roll 90 + yaw +-45, roll +-30 + pitch +-40  (12)
    for a in (45, -45):
        S.append(slot("NC", "N", "C", turn=a))
        S.append(slot("NC", "N", "C", lean=a))
    for a in (35, -35):
        for b in (35, -35):
            S.append(slot("NC", "N", "C", turn=a, lean=b))
    for a in (45, -45):
        S.append(slot("NC", "N", "C", turn=a, roll=90))
    S.append(slot("NC", "N", "C", lean=40, roll=30))
    S.append(slot("NC", "N", "C", lean=-40, roll=-30))
    # N-E: edges L R T B, two opposite 35 deg tilts each (8)
    for p, ax in (("L", "turn"), ("R", "turn"), ("T", "lean"), ("B", "lean")):
        for a in (35, -35):
            S.append(slot("NE", "N", p, **{ax: a}))
    # N-K: 4 corners, two opposite diagonal 30 deg tilts each (21 + 21 deg ~ 30 deg) (8)
    for p in ("TL", "TR", "BL", "BR"):
        sx = -1 if "L" in p else 1
        sy = -1 if "T" in p else 1
        for g in (1, -1):
            S.append(slot("NK", "N", p, turn=21 * sx * g, lean=21 * sy * g))
    # M-C: centre 1.15-1.5 m: frontal, yaw +-40, pitch +-40, 2 diagonals, roll 45 + tilt 20 (8)
    S.append(slot("MC", "M", "C"))
    for a in (40, -40):
        S.append(slot("MC", "M", "C", turn=a))
        S.append(slot("MC", "M", "C", lean=a))
    S.append(slot("MC", "M", "C", turn=30, lean=-30))
    S.append(slot("MC", "M", "C", turn=-30, lean=30))
    S.append(slot("MC", "M", "C", turn=20, roll=45))
    # M-EK: 4 edges + 4 corners at 30 deg (8)
    # inner side (towards the image centre) nearer, same as the M-EK corners (audit 10-27: T/B signs were flipped)
    for p, ax, a in (("L", "turn", 30), ("R", "turn", -30), ("T", "lean", 30), ("B", "lean", -30)):
        S.append(slot("MEK", "M", p, **{ax: a}))
    for p in ("TL", "TR", "BL", "BR"):
        sx = -1 if "L" in p else 1
        sy = -1 if "T" in p else 1
        S.append(slot("MEK", "M", p, turn=-21 * sx, lean=-21 * sy))
    # F: centre 1.6-2.1 m: frontal, yaw +-35, pitch +-35 (5)
    S.append(slot("F", "F", "C"))
    for a in (35, -35):
        S.append(slot("F", "F", "C", turn=a))
        S.append(slot("F", "F", "C", lean=a))
    # N-R (README N6 moved to >= 0.9 m): roll 90 + pitch +-45, roll 45 + yaw +-45, roll -45 + pitch +-45 (6)
    for a in (45, -45):
        S.append(slot("NR", "N", "C", lean=a, roll=90))
    for a in (45, -45):
        S.append(slot("NR", "N", "C", turn=a, roll=45))
    for a in (45, -45):
        S.append(slot("NR", "N", "C", lean=a, roll=-45))
    return S


def visible_pose(s, board, K, D, gg):
    s = dict(s); s["target"] = dict(s["target"], goal_gap=gg)
    R, t = SL.target_Rt(s, board, K, D)
    pts, _ = cv2.projectPoints(board.obj, cv2.Rodrigues(R)[0], t, K, D)
    pts = pts.reshape(-1, 2)
    cam = (R @ board.obj.T).T + t
    ok = (pts[:, 0] >= 8) & (pts[:, 0] <= SL.W - 9) & (pts[:, 1] >= 8) & (pts[:, 1] <= SL.H - 9) & (cam[:, 2] > 0)
    ids = np.nonzero(ok)[0]
    return SL.pose_of(board, pts[ids], ids, K, D), int(len(ids))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", default="board_print_A1_12x9")
    ap.add_argument("--lens", default=os.path.join(S3D, "config", "lens", "std_1920x1080_f0.50_cal20261024d.json"))
    ap.add_argument("--out", default=os.path.join(S3D, "config", "calcap_slots", "print_A1_55.json"))
    a = ap.parse_args()
    board = SL.load_boards(os.path.join(S3D, "config", "charuco"), (a.board,))[0]
    K, D = SL.load_lens(a.lens)
    S = build()
    bad = []
    for s in S:
        done = False
        for gg in (150, 100, 40, 0, -60, -120, -180, -240, -300):
            p, nvis = visible_pose(s, board, K, D, gg)
            if p is not None and SL.fills(s, p):
                s["target"]["goal_gap"] = gg
                s["check"] = dict(visible_corners=nvis, turn=round(p["turn"], 1), lean=round(p["lean"], 1),
                                  roll=round(p["roll"], 1), tilt=round(p["tilt"], 1), dist=round(p["dist"], 3), pos=p["pos"])
                done = True
                break
        if not done:
            bad.append(s["id"])
    lean_pos = sum(1 for s in S if s["target"]["lean"] > 5)
    lean_neg = sum(1 for s in S if s["target"]["lean"] < -5)
    diag = sum(1 for s in S if abs(s["target"]["turn"]) > 5 and abs(s["target"]["lean"]) > 5)
    rolls = sorted({abs(s["target"]["roll"]) for s in S})
    groups = {}
    for s in S:
        groups[s["group"]] = groups.get(s["group"], 0) + 1
    j = dict(name=os.path.splitext(os.path.basename(a.out))[0], made=time.strftime("%Y-%m-%d %H:%M:%S"),
             board=a.board, lens_for_check=a.lens, n=len(S), groups=groups,
             counts=dict(lean_plus=lean_pos, lean_minus=lean_neg, diagonal=diag, roll_abs_values=rolls),
             note=("Printed-board lens calibration slots (lenskit 2026-10-27). Source lenscal_20261027 README section 2; "
                   "distance >= 0.9 m (0.5 dpt focus, depth of field estimate); board on a support, phone on a tripod. "
                   "Use: calcap_server.py --slots <this> --boards board_print_A1_12x9 (A2 / A3: same file, the board covers "
                   "less of the frame, more poses may fail the corner coverage)."),
             unfillable=bad, slots=S)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(j, open(a.out, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    print(a.out, len(S), "slots", groups, j["counts"], "unfillable:", bad)
    for s in S:
        print(f"  {s['id']:24s} gg {s['target'].get('goal_gap')!s:>5} {s.get('check')}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
