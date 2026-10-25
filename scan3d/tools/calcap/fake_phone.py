"""Fake phone for testing calcap_server.py + calcap_calib.py without the phone (2026-10-25).

Speaks the app's calcap TCP protocol (PREV / CAP / STAT / MSG / OVL / IMG / STOP) and renders the laptop 5x3 ChArUco board
(the real PNG) through a KNOWN "true" lens (default: cal20261024d with fx, fy x1.004 and cx +3, cy -2 px, i.e. NOT the
lens the server uses for guidance) at a scripted pose sequence that visits every slot's target pose (held ~1.6 s, with
moves in between). CAP writes PGM stills + frames.jsonl like the app into <dir>/<session>/.
  python fake_phone.py --dir <phone files dir> [--port 8812] [--hold 1.6]
The true lens is written to <dir>/true_lens.json so the calibration result can be compared with it.
"""
import argparse
import json
import math
import os
import socket
import sys
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import slots as SL  # noqa: E402

S3D = os.path.abspath(os.path.join(HERE, "..", ".."))
ap = argparse.ArgumentParser()
ap.add_argument("--dir", required=True)
ap.add_argument("--port", type=int, default=8812)
ap.add_argument("--hold", type=float, default=1.6)
ap.add_argument("--move", type=float, default=0.7)
ap.add_argument("--fps", type=float, default=10)
ap.add_argument("--lens", default=os.path.join(S3D, "config", "lens", "std_1920x1080_f0.50_cal20261024d.json"))
ap.add_argument("--noise", type=float, default=2.0)
A = ap.parse_args()

K0, D0 = SL.load_lens(A.lens)
KT = K0.copy(); KT[0, 0] *= 1.004; KT[1, 1] *= 1.004; KT[0, 2] += 3.0; KT[1, 2] -= 2.0
DT = D0.copy()
BOARD = SL.load_boards(os.path.join(S3D, "config", "charuco"), ("board_laptop_5x3",))[0]
PNG = cv2.imread(os.path.join(S3D, "config", "charuco", "charuco_laptop_5x3.png"), cv2.IMREAD_GRAYSCALE)
PX_PER_M = 384.0 / BOARD.sq          # 384 px per square in the PNG, board rows start at y = 24
Y_OFF = (PNG.shape[0] - 384 * BOARD.sy) / 2.0
W, H = 1920, 1080
uu, vv = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
NORM = cv2.undistortPoints(np.stack([uu, vv], -1).reshape(-1, 1, 2), KT, DT).reshape(H, W, 2)
rng = np.random.default_rng(1)

# pose script: every slot target, plus a "no board" gap at the start
SEQ = []
for s in SL.SLOTS:
    R, t = SL.target_Rt(s, BOARD, KT, DT)
    SEQ.append((s["id"], R, t))


def pose_at(tsec):
    period = A.hold + A.move
    if tsec < 1.0:
        return None
    k = int((tsec - 1.0) // period)
    ph = (tsec - 1.0) - k * period
    if k >= len(SEQ):
        k = len(SEQ) - 1; ph = 0.0
    _, R, t = SEQ[k]
    if ph < A.hold:
        return R, t
    # moving to the next pose: big shift (blurless but different) -> not steady, not matching
    a = (ph - A.hold) / A.move
    _, R2, t2 = SEQ[min(k + 1, len(SEQ) - 1)]
    rv = (1 - a) * cv2.Rodrigues(R)[0] + a * cv2.Rodrigues(R2)[0]
    return cv2.Rodrigues(rv)[0], (1 - a) * t + a * t2 + np.array([0.05 * math.sin(a * math.pi), 0, 0])


def render(pose):
    img = np.full((H, W), 170, np.uint8)
    if pose is None:
        return img
    R, t = pose
    Hn = np.stack([R[:, 0], R[:, 1], t], 1)
    Hi = np.linalg.inv(Hn)
    x = NORM[..., 0]; y = NORM[..., 1]
    X = Hi[0, 0] * x + Hi[0, 1] * y + Hi[0, 2]
    Y = Hi[1, 0] * x + Hi[1, 1] * y + Hi[1, 2]
    Z = Hi[2, 0] * x + Hi[2, 1] * y + Hi[2, 2]
    mx = (X / Z * PX_PER_M).astype(np.float32)
    my = (Y / Z * PX_PER_M + Y_OFF).astype(np.float32)
    out = cv2.remap(PNG, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=170)
    out = cv2.GaussianBlur(out, (0, 0), 0.7)
    out = np.clip(out.astype(np.float32) + rng.normal(0, A.noise, out.shape), 0, 255).astype(np.uint8)
    return out


SESSION = time.strftime("%Y%m%d_%H%M%S")
SDIR = os.path.join(A.dir, SESSION)
os.makedirs(SDIR, exist_ok=True)
json.dump(dict(K=KT.tolist(), dist=DT.tolist(), note="fake_phone true lens = guidance lens with fx,fy x1.004, cx +3, cy -2"),
          open(os.path.join(A.dir, "true_lens.json"), "w"), indent=1)
T0 = time.time()
stats = dict(frames=0, caps=0, imgs=0, msgs=0)


def frame_now():
    k = int((time.time() - T0) * A.fps)
    return k, int(k * 1e9 / A.fps)


def handle(c):
    f = c.makefile("rb")
    cap_id = 0
    while True:
        line = f.readline()
        if not line:
            return
        a = line.decode("utf-8").strip().split(" ", 3)
        cmd = a[0]
        if cmd == "PREV":
            k, ts = frame_now()
            img = render(pose_at(time.time() - T0))[::2, ::2]
            ok, j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            c.sendall(f"JPG {len(j)} {ts}\n".encode() + j.tobytes())
            stats["frames"] += 1
            time.sleep(max(0.0, 1.0 / A.fps - 0.02))
        elif cmd == "CAP":
            n = int(a[1]); tag = a[2]; info = a[3] if len(a) > 3 else "{}"
            cap_id += 1
            files = []
            for i in range(n):
                time.sleep(1.0 / A.fps)
                k, ts = frame_now()
                img = render(pose_at(time.time() - T0))
                name = f"cap_{cap_id:03d}_{i}.pgm"
                cv2.imwrite(os.path.join(SDIR, name), img)
                with open(os.path.join(SDIR, "frames.jsonl"), "a") as fj:
                    fj.write(json.dumps(dict(file=name, cap=cap_id, k=i, tag=tag, sensor_ts_ns=ts, written=True, width=W, height=H,
                                             result=dict(exposure_ns=2000000, iso=500, focus_dpt=0.5, frame_duration_ns=100000000,
                                                         crop="0 0 4000 2252", fake=True), pc=json.loads(info))) + "\n")
                files.append(name)
            stats["caps"] += 1
            c.sendall(("DONE " + json.dumps(dict(cap=cap_id, tag=tag, files=files, n=n, skipped=0, dir="/fake/" + SESSION)) + "\n").encode())
        elif cmd == "STAT":
            c.sendall(("STAT " + json.dumps(dict(frames=stats["frames"], fps_measured=A.fps, dir="/fake/" + SESSION,
                                                 temps=dict(skin_c=30.0, ap_c=33.0), captures=stats["caps"], fake=True)) + "\n").encode())
        elif cmd in ("MSG", "IMG", "OVL"):
            if cmd == "OVL":
                json.loads(line[4:])          # must be valid json for the app (0.6.25)
            k = dict(MSG="msgs", IMG="imgs", OVL="ovls")[cmd]
            stats[k] = stats.get(k, 0) + 1
            c.sendall(b"OK\n")
        elif cmd == "STOP":
            c.sendall(b"OK\n")
            json.dump(dict(stop_reason="pc stop", stat_end=dict(fps_measured=A.fps, frames=stats["frames"]), fake=True, stats=stats),
                      open(os.path.join(SDIR, "meta.json"), "w"))
            open(os.path.join(SDIR, "temps.csv"), "w").write("t_rel_s,skin_c\n0,30\n")
            print("fake phone: STOP", stats, flush=True)
            os._exit(0)
        else:
            c.sendall(b"ERR\n")


srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", A.port)); srv.listen(1)
print(f"fake phone on 127.0.0.1:{A.port}, session {SDIR}, {len(SEQ)} poses x {A.hold + A.move:.1f} s", flush=True)
while True:
    c, _ = srv.accept()
    threading.Thread(target=handle, args=(c,), daemon=True).start()
