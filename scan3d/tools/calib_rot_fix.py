#!/usr/bin/env python3
"""Two-lens calibration rotation fix: rotate the right camera by small (pitch, yaw, roll) measured in the RECTIFIED
right-camera frame (rect1), the same convention as aruco_scale extrinsic residuals and tools/eval_aruco_scale.py:

    X_true_rect1 = expm([w]x) X_calib_rect1,   w = (pitch, yaw, roll) rad
    yaw > 0  -> corner disparity smaller than truth by f*yaw px (depth too long);  pitch > 0 -> v1 - v0 = -f*pitch px.

New R = Rerr @ R, T = Rerr @ T with Rerr = R2^T expm([w]x) R2 (rotation of the right camera about its own centre).
Intrinsics, distortion and everything else are copied. The old file is never modified.

    venv_ffs/bin/python scan3d/tools/calib_rot_fix.py calib_phone_pair.json calib_phone_pair_v2.json         --yaw-mrad 1.5 --pitch-mrad 2.0 --roll-mrad 0 --why 'board rec_20261018_002349 + SfM 165645/175446'
"""
import argparse
import datetime as dt
import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(1, str(HERE.parent.parent))


def fix_rotation(st, pitch, yaw, roll):
    """st = calibration dict (K1,dist1,K2,dist2,R,T,image_size). Angles in rad. Returns (R_new, T_new, Rerr)."""
    K1, d1, K2, d2 = (np.array(st[k], float) for k in ('K1', 'dist1', 'K2', 'dist2'))
    R, T = np.array(st['R'], float), np.array(st['T'], float).reshape(3, 1)
    W, H = (int(x) for x in st['image_size'])
    _, R2, *_ = cv2.stereoRectify(K1, d1, K2, d2, (W, H), R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    Rd, _ = cv2.Rodrigues(np.array([pitch, yaw, roll], float))
    Rerr = R2.T @ Rd @ R2
    return Rerr @ R, (Rerr @ T).reshape(3), Rerr


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--yaw-mrad', type=float, default=0.0)
    ap.add_argument('--pitch-mrad', type=float, default=0.0)
    ap.add_argument('--roll-mrad', type=float, default=0.0)
    ap.add_argument('--why', default='', help='evidence text stored in the new file')
    a = ap.parse_args(argv)
    src, dst = Path(a.src), Path(a.dst)
    if src.resolve() == dst.resolve():
        sys.exit('dst must differ from src (the original calibration is kept)')
    st = json.loads(src.read_text())
    Rn, Tn, Rerr = fix_rotation(st, 1e-3 * a.pitch_mrad, 1e-3 * a.yaw_mrad, 1e-3 * a.roll_mrad)
    out = dict(st)
    out['R'], out['T'] = Rn.tolist(), Tn.tolist()
    out['baseline_m'] = float(np.linalg.norm(Tn))
    fix = dict(created=dt.datetime.now().isoformat(timespec='seconds'), tool='scan3d/tools/calib_rot_fix.py',
               base_file=src.name, rect1_mrad=dict(pitch=a.pitch_mrad, yaw=a.yaw_mrad, roll=a.roll_mrad),
               R_err=Rerr.tolist(), evidence=a.why)
    out['rotation_fix'] = fix
    out['history'] = list(st.get('history') or []) + [dict(pose_source='rotation_fix', base=src.name,
                                                           rect1_mrad=fix['rect1_mrad'])]
    dst.write_text(json.dumps(out, ensure_ascii=False, indent=2) + '\n')
    print(f'{src.name} -> {dst.name}: rect1 pitch/yaw/roll mrad = {a.pitch_mrad}/{a.yaw_mrad}/{a.roll_mrad}, '
          f'|R_err| = {np.degrees(np.linalg.norm(cv2.Rodrigues(Rerr)[0])):.4f} deg')


if __name__ == '__main__':
    main()
