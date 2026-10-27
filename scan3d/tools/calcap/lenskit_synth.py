"""Synthetic calcap set for the printed-board pipeline test (lenskit 2026-10-27, PC only, no phone).

  ~/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/tools/calcap/lenskit_synth.py OUT_SET_DIR
      [--slots config/calcap_slots/print_A1_55.json] [--board board_print_A1_12x9] [--stills 2] [--seed 1]
      [--true-lens L] [--sag-mm 0] [--focus-dpt 0.5] [--drop-slots N]

Renders every slot's target pose (+ small random jitter inside the slot windows) through a KNOWN true lens
(default: calcap20261026 k1 k2 lens with fx, fy x 1.003, cx + 2, cy - 1.5 px, k1 + 0.004) and writes the same files the
real capture writes: captures.json (ok captures, slot, files), <session>/cap_NNN_k.pgm, <session>/frames.jsonl (focus_dpt
etc.), set.json, true_lens.json. Each still is checked with slots.fills() on the re-detected pose like the server does.
--sag-mm s: the board is NOT flat - centre pushed away from the camera by s mm along a paraboloid (both axes), with the
sign flipped for lean < 0 (self-weight sag on a tilted support), to see C5 / C8 react.
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
from lenskit_selftest import board_image  # noqa: E402

S3D = os.path.abspath(os.path.join(HERE, "..", ".."))
W, H = 1920, 1080


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--slots", default=os.path.join(S3D, "config", "calcap_slots", "print_A1_55.json"))
    ap.add_argument("--board", default="board_print_A1_12x9")
    ap.add_argument("--stills", type=int, default=2)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--true-lens", default=os.path.join(S3D, "config", "lens", "std_1920x1080_f0.50_cal20261024d.json"))
    ap.add_argument("--perturb", type=int, default=1, help="1: true lens = file lens x1.003 fx fy, cx +2, cy -1.5, k1 +0.004")
    ap.add_argument("--sag-mm", type=float, default=0.0)
    ap.add_argument("--focus-dpt", type=float, default=0.5)
    ap.add_argument("--noise", type=float, default=2.0, help="pixel noise sd (grey levels)")
    ap.add_argument("--drop-slots", type=int, default=0, help="leave out N random slots (incomplete set)")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    board = SL.load_boards(os.path.join(S3D, "config", "charuco"), (a.board,))[0]
    slots, sj = SL.load_slot_file(a.slots)
    K0, D0 = SL.load_lens(a.true_lens)
    KT, DT = K0.copy(), np.zeros(5)
    DT[:len(D0)] = D0[:5]
    if a.perturb:
        KT[0, 0] *= 1.003; KT[1, 1] *= 1.003; KT[0, 2] += 2.0; KT[1, 2] -= 1.5; DT[0] += 0.004
    os.makedirs(a.out, exist_ok=True)
    sess = time.strftime("%Y%m%d_%H%M%S")
    sd = os.path.join(a.out, sess)
    os.makedirs(sd, exist_ok=True)
    json.dump(dict(K=KT.tolist(), dist=DT.tolist(), note="synthetic true lens (lenskit_synth)", sag_mm=a.sag_mm,
                   focus_dpt=a.focus_dpt), open(os.path.join(a.out, "true_lens.json"), "w"), indent=1)
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    nrm = cv2.undistortPoints(np.stack([uu, vv], -1).reshape(-1, 1, 2), KT, DT).reshape(H, W, 2)
    ray = np.concatenate([nrm, np.ones((H, W, 1))], -1)
    ppm = 8000.0
    img0, org = board_image(board, ppm)
    blurred = {}
    use = list(range(len(slots)))
    if a.drop_slots:
        use = sorted(rng.permutation(len(slots))[a.drop_slots:].tolist())
    caps, frames = [], []
    half = np.ptp(board.obj[:, :2], 0) / 2
    for ci, si in enumerate(use, 1):
        s = slots[si]
        for attempt in range(20):
            tg = dict(s["target"])
            j = 1
            tg["turn"] = tg.get("turn", 0) + j * rng.uniform(-3, 3)
            tg["lean"] = tg.get("lean", 0) + j * rng.uniform(-3, 3)
            tg["roll"] = tg.get("roll", 0) + j * rng.uniform(-3, 3)
            tg["dist"] = tg.get("dist") + j * rng.uniform(-0.04, 0.04)
            s2 = dict(s, target=tg)
            R, t = SL.target_Rt(s2, board, KT, DT)
            # ray-plane intersection (flat) or ray-paraboloid (sag) in board coordinates
            n = R[:, 2]
            lam = (n @ t) / (ray @ n)
            P = ray * lam[..., None]
            B = (P - t) @ R                          # board coords (x, y, z)
            if a.sag_mm:
                sg = 1.0 if tg["lean"] >= 0 else -1.0
                for _ in range(3):                   # z(x, y) = sag * (1 - u^2) (1 - v^2) pushed along +z (away)
                    u = (B[..., 0] - board.centre[0]) / half[0]
                    v = (B[..., 1] - board.centre[1]) / half[1]
                    z = sg * a.sag_mm / 1000.0 * np.clip(1 - u * u, 0, None) * np.clip(1 - v * v, 0, None)
                    # move along the ray so the point lies on the bent surface (z grows with lam along n)
                    dl = (z - B[..., 2]) / (ray @ n)
                    lam = lam + dl
                    P = ray * lam[..., None]
                    B = (P - t) @ R
            valid = lam > 0
            dist_c = float(np.linalg.norm(R @ board.centre + t))
            key = round(dist_c, 1)
            if key not in blurred:
                sig = 0.6 * ppm * dist_c / KT[0, 0]
                blurred[key] = cv2.GaussianBlur(img0, (0, 0), sig)
            mx = (B[..., 0] * ppm + org[0]).astype(np.float32)
            my = (B[..., 1] * ppm + org[1]).astype(np.float32)
            mx[~valid] = -1; my[~valid] = -1
            g0 = cv2.remap(blurred[key], mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=170)
            g0 = cv2.GaussianBlur(g0.astype(np.float32), (0, 0), 0.7)
            stills, ok_all = [], True
            for k in range(a.stills):
                g = np.clip(g0 + rng.normal(0, a.noise, g0.shape), 0, 255).astype(np.uint8)
                c, i = board.detect(g)
                p = SL.pose_of(board, c, i, KT, DT) if len(i) >= board.min_corners else None
                if p is None or not SL.fills(s, p, tol=1.0):
                    ok_all = False
                    break
                stills.append((g, p))
            if ok_all:
                break
        if not ok_all:
            print("slot not filled after 20 tries:", s["id"], flush=True)
            continue
        files = []
        for k, (g, p) in enumerate(stills):
            fn = f"cap_{ci:03d}_{k}.pgm"
            cv2.imwrite(os.path.join(sd, fn), g)
            files.append(f"{sess}/{fn}")
            frames.append(dict(file=fn, cap=ci, k=k, tag=s["id"], written=True, width=W, height=H,
                               result=dict(focus_dpt=a.focus_dpt, exposure_ns=2000000, iso=800, lens_state=0, ois=0,
                                           noise_reduction=0, edge=0, distortion_correction=0),
                               pc=dict(slot=s["id"], synthetic=True)))
        caps.append(dict(cap=ci, slot=s["id"], label=s["label"], dir=sess, files=files, ok=True, why="synthetic",
                         motion_px=0.0, still_poses=[{k2: v for k2, v in p.items() if k2 not in ("rvec", "tvec")} for _, p in stills]))
        print(f"{ci:3d} {s['id']:24s} turn {stills[0][1]['turn']:+6.1f} lean {stills[0][1]['lean']:+6.1f} roll {stills[0][1]['roll']:+6.1f} "
              f"dist {stills[0][1]['dist']:.2f} n {stills[0][1]['n']}", flush=True)
    with open(os.path.join(sd, "frames.jsonl"), "w") as f:
        for fr in frames:
            f.write(json.dumps(fr) + "\n")
    json.dump(dict(updated=time.strftime("%Y-%m-%d %H:%M:%S"), n_ok=len(caps), n_total=len(slots),
                   missing=len(slots) - len(caps), captures=caps), open(os.path.join(a.out, "captures.json"), "w"), indent=1)
    json.dump(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), synthetic=True, boards=[board.name], slots_file=a.slots,
                   board_specs={board.name: board.spec}, frames_per_slot=a.stills, seed=a.seed, sag_mm=a.sag_mm),
              open(os.path.join(a.out, "set.json"), "w"), indent=1, ensure_ascii=False)
    print("set", a.out, "captures", len(caps), "/", len(slots))


if __name__ == "__main__":
    main()
