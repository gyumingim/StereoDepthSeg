#!/usr/bin/env python3
"""viewcrafter 2026-11-25: virtual side-orbit trajectory for recording 165506 (our BA world frame, OpenCV RDF c2w).

  ~/scanner/gsmap_env/venv/bin/python scan3d/viewcrafter/vc_traj.py --dump DUMP --fuse FUSE_DA3 --out traj.npz \
      [--anchor 24] [--inputs 62 24 55] [--phi 50] [--n 25]

Frame 0 = anchor keyframe pose exactly. Frame k = anchor pose rotated by phi_k (0 -> +-phi, linear) about the world-up axis
through the look-at point P = C_a + d * z_a, d = median fused depth in the central 1/3 of the anchor image.
World up = minus the mean image-down axis of all keyframes (same as gsm_render.py).
The sign of phi is chosen so the path leaves the captured camera path the most (larger mean 'off-path angle' = angle at P
between the virtual camera and the nearest captured keyframe camera).
Intrinsics: images are centre-cropped 960x540 -> 864x540 (16:10) and resized to 512x320 (ViewCrafter-512 input size).
  K864 = Kr with cx - 48 (splat render), K512 = K864 * 512/864 (ViewCrafter condition render).
"""
import argparse, json, time
from pathlib import Path

import numpy as np


def rot_axis(a, ang):
    a = a / np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", required=True); ap.add_argument("--fuse", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--anchor", type=int, default=24); ap.add_argument("--inputs", type=int, nargs="+", default=[62, 24, 55])
    ap.add_argument("--phi", type=float, default=50.0); ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--target", type=int, default=-1, help="check path: anchor -> this real keyframe (slerp + lerp), last frame = its pose")
    a = ap.parse_args()
    cams = np.load(Path(a.dump) / "cams.npz"); T, kf, Kr = cams["T"], cams["kf"], cams["Kr"]
    W0, H0 = (int(x) for x in cams["size"]); crop = (W0 - 864) // 2
    K864 = Kr.copy(); K864[0, 2] -= crop
    s = 512 / 864; K512 = K864.copy(); K512[:2] *= s
    up = -T[:, :3, 1].mean(0); up /= np.linalg.norm(up)
    Ta = T[a.anchor]
    Z = np.load(Path(a.fuse) / f"{kf[a.anchor]:05d}.npz")["Z"]
    c = Z[H0 // 3: 2 * H0 // 3, W0 // 3: 2 * W0 // 3]; d = float(np.nanmedian(c[np.isfinite(c) & (c > 0)]))
    P = Ta[:3, 3] + d * Ta[:3, 2]
    best = None
    for sign in (+1, -1):
        phis = np.radians(np.linspace(0, sign * a.phi, a.n)); c2w = []
        for ph in phis:
            R = rot_axis(up, ph); M = np.eye(4); M[:3, :3] = R; M[:3, 3] = P - R @ P
            c2w.append(M @ Ta)
        c2w = np.stack(c2w)
        # off-path angle at P to nearest captured camera
        v = c2w[:, :3, 3] - P; v /= np.linalg.norm(v, axis=1, keepdims=True)
        u = T[:, :3, 3] - P; u /= np.linalg.norm(u, axis=1, keepdims=True)
        off = np.degrees(np.arccos(np.clip(v @ u.T, -1, 1)).min(1))
        if best is None or off.mean() > best[2].mean():
            best = (sign, c2w, off)
    sign, c2w, off = best
    if a.target >= 0:
        from scipy.spatial.transform import Rotation, Slerp
        Tb = T[a.target]; w = np.linspace(0, 1, a.n)
        Rk = Slerp([0, 1], Rotation.from_matrix(np.stack([Ta[:3, :3], Tb[:3, :3]])))(w).as_matrix()
        c2w = np.tile(np.eye(4), (a.n, 1, 1)); c2w[:, :3, :3] = Rk; c2w[:, :3, 3] = (1 - w)[:, None] * Ta[:3, 3] + w[:, None] * Tb[:3, 3]
        v = c2w[:, :3, 3] - P; v /= np.linalg.norm(v, axis=1, keepdims=True)
        u = np.delete(T, [a.target], 0)[:, :3, 3] - P; u /= np.linalg.norm(u, axis=1, keepdims=True)
        off = np.degrees(np.arccos(np.clip(v @ u.T, -1, 1)).min(1)); sign = 0
    meta = dict(target=a.target, target_kf=int(kf[a.target]) if a.target >= 0 else None,made=time.strftime("%Y-%m-%d %H:%M:%S"), dump=str(Path(a.dump).resolve()), anchor=a.anchor, anchor_kf=int(kf[a.anchor]),
                inputs=a.inputs, inputs_kf=[int(kf[i]) for i in a.inputs], phi_deg=sign * a.phi, n=a.n, look_dist_m=d,
                P=P.tolist(), up=up.tolist(), off_path_deg_last=float(off[-1]), off_path_deg=off.round(1).tolist(), crop_px=crop)
    np.savez(a.out, c2w=c2w, K512=K512, K864=K864, P=P, up=up, meta=json.dumps(meta))
    print(json.dumps(meta, indent=1))


if __name__ == "__main__":
    main()
