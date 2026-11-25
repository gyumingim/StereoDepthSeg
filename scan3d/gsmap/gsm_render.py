#!/usr/bin/env python3
"""gsmap 2026-11-25: render a trained splat (gaussians.pt from gsm_train.py) headless.

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_render.py RUN_DIR [--fps 30] [--back 0.35] [--up 0.22]

Outputs in RUN_DIR:
  chase.mp4   virtual camera behind (--back m against the phone's view direction) and above (--up m, world up = -y of the
              board frame, the phone's mean 'down' axis) the phone, smoothed, looking at a point 0.6 m in front of the phone.
              Phone drawn as a small wireframe frustum. Path = keyframe poses (BA, all 134) slerp-interpolated to --fps
              over the keyframe timestamps.
  side_<kf>.jpg  original keyframe | splat render at the same pose, for the held-out keyframes.
  slice.png   top view (x-z) of Gaussian centres (opacity > 0.3) in 3 horizontal bands 2 cm thick -> double surfaces show
              as two parallel lines.
"""
import argparse, sys, time
from pathlib import Path

import cv2
import numpy as np


def quat_from_R(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("--fps", type=float, default=30)
    ap.add_argument("--back", type=float, default=0.35); ap.add_argument("--up", type=float, default=0.22)
    ap.add_argument("--look", type=float, default=0.6); ap.add_argument("--smooth", type=float, default=0.6, help="s, Gaussian sigma")
    ap.add_argument("--dump", default=None)
    a = ap.parse_args()
    import torch
    from gsplat import rasterization
    from scipy.spatial.transform import Rotation, Slerp
    run = Path(a.run)
    ck = torch.load(run / "gaussians.pt", weights_only=False)
    p = {k: v.cuda() for k, v in ck["params"].items()}
    T, Kr, (W, H), kf, test = ck["T"], ck["Kr"], ck["size"], ck["kf"], ck["test"]
    dump = Path(a.dump or ck.get("args", {}).get("dump", ""))
    cams = np.load(dump / "cams.npz"); tns = cams["t_ns"].astype(np.float64) * 1e-9
    K = torch.from_numpy(Kr).float().cuda()

    def render(vm):
        rc, ra, _ = rasterization(p["means"], torch.nn.functional.normalize(p["quats"], dim=-1), torch.exp(p["scales"]),
                                  torch.sigmoid(p["opacities"]), torch.cat([p["sh0"], p["shN"]], 1),
                                  torch.from_numpy(vm).float().cuda()[None], K[None], W, H, sh_degree=ck["sh_degree"],
                                  near_plane=0.02, far_plane=20.0, packed=False)
        im = rc[0] + (1 - ra[0]) * 0.08
        return cv2.cvtColor((im.clamp(0, 1).cpu().numpy() * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)

    # ---- side-by-side stills on held-out views
    with torch.no_grad():
        for c in test:
            im = cv2.imread(str(dump / "img" / f"{kf[c]:05d}.png"))
            r = render(np.linalg.inv(T[c]))
            s = np.hstack([im, r])
            cv2.putText(s, "original (held out)", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            cv2.putText(s, "splat render", (W + 12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            cv2.imwrite(str(run / f"side_{kf[c]:05d}.jpg"), s, [cv2.IMWRITE_JPEG_QUALITY, 88])

    # ---- chase path
    t = tns - tns[0]
    tt = np.arange(0, t[-1], 1.0 / a.fps)
    rots = Rotation.from_matrix(T[:, :3, :3])
    Rq = Slerp(t, rots)(tt)
    pos = np.stack([np.interp(tt, t, T[:, i, 3]) for i in range(3)], 1)
    # smooth positions and view directions (Gaussian over time)
    sig = a.smooth * a.fps; k = np.arange(-int(3 * sig), int(3 * sig) + 1); g = np.exp(-k ** 2 / (2 * sig ** 2)); g /= g.sum()
    sm = lambda x: np.stack([np.convolve(np.pad(x[:, i], len(k) // 2, mode="edge"), g, mode="valid") for i in range(x.shape[1])], 1)  # noqa: E731
    fwd = sm(Rq.as_matrix()[:, :, 2]); fwd /= np.linalg.norm(fwd, axis=1, keepdims=True)
    up = -T[:, :3, 1].mean(0); up /= np.linalg.norm(up)          # world up = minus the phone's mean image-down axis
    pos_s = sm(pos)
    fwd_h = fwd - (fwd @ up)[:, None] * up; fwd_h /= np.linalg.norm(fwd_h, axis=1, keepdims=True)
    eye = pos_s - a.back * fwd_h + a.up * up
    tgt = pos_s + a.look * fwd
    vw = cv2.VideoWriter(str(run / "chase_raw.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W, H))
    # phone frustum in phone frame (12 cm deep)
    d = 0.06; fr = np.array([[0, 0, 0], [-0.5, -0.3, 1], [0.5, -0.3, 1], [0.5, 0.3, 1], [-0.5, 0.3, 1]]) * [d * 1.4, d * 1.4, d]
    edges = [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]
    t0 = time.time()
    with torch.no_grad():
        for j in range(len(tt)):
            z = tgt[j] - eye[j]; z /= np.linalg.norm(z)
            x = np.cross(-up, z); x /= np.linalg.norm(x); y = np.cross(z, x)
            c2w = np.eye(4); c2w[:3, :3] = np.stack([x, y, z], 1); c2w[:3, 3] = eye[j]
            w2c = np.linalg.inv(c2w)
            img = render(w2c)
            Rp = Rq[j].as_matrix(); Pw = fr @ Rp.T + pos[j]
            Pc = Pw @ w2c[:3, :3].T + w2c[:3, 3]
            if (Pc[:, 2] > 0.02).all():
                uv = (Pc[:, :2] / Pc[:, 2:] * [Kr[0, 0], Kr[1, 1]] + [Kr[0, 2], Kr[1, 2]]).astype(int)
                for e0, e1 in edges:
                    cv2.line(img, tuple(uv[e0]), tuple(uv[e1]), (40, 200, 255), 2, cv2.LINE_AA)
            vw.write(img)
    vw.release()
    import subprocess, imageio_ffmpeg
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-i", str(run / "chase_raw.mp4"), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-crf", "23", "-movflags", "+faststart", str(run / "chase.mp4")], check=True)
    (run / "chase_raw.mp4").unlink()
    print(f"chase {len(tt)} frames {time.time() - t0:.1f}s", flush=True)

    # ---- slices of Gaussian centres
    M = p["means"].cpu().numpy(); op = torch.sigmoid(p["opacities"]).cpu().numpy(); C = (p["sh0"][:, 0].cpu().numpy() * 0.2821 + 0.5).clip(0, 1)
    s = op > 0.3; M, C = M[s], C[s]
    h = M @ up
    qs = np.percentile(h, [30, 50, 70]); panels = []
    x0, x1 = np.percentile(M[:, 0], [1, 99]); z0, z1 = np.percentile(M[:, 2], [1, 99])
    S = 500
    for q in qs:
        b = np.abs(h - q) < 0.01
        im = np.full((S, S, 3), 20, np.uint8)
        u = ((M[b, 0] - x0) / (x1 - x0) * (S - 1)).astype(int); v = ((M[b, 2] - z0) / (z1 - z0) * (S - 1)).astype(int)
        ok = (u >= 0) & (u < S) & (v >= 0) & (v < S)
        im[v[ok], u[ok]] = (C[b][ok][:, ::-1] * 255).astype(np.uint8)
        # cameras
        cu = ((T[:, 0, 3] - x0) / (x1 - x0) * (S - 1)).astype(int); cv_ = ((T[:, 2, 3] - z0) / (z1 - z0) * (S - 1)).astype(int)
        for uu, vv in zip(cu, cv_):
            if 0 <= uu < S and 0 <= vv < S:
                cv2.circle(im, (uu, vv), 2, (0, 0, 255), -1)
        cv2.putText(im, f"band {q:+.2f} m, {b.sum()} pts", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        panels.append(im)
    sl = np.hstack(panels)
    cv2.putText(sl, f"top view x-z, {(x1 - x0) * 100:.0f} x {(z1 - z0) * 100:.0f} cm; red = cameras", (8, S - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imwrite(str(run / "slice.png"), sl)


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gsplat"))
    import gpu_guard
    with gpu_guard.job("gsmap render " + sys.argv[1], torch_cap=True):
        main()
