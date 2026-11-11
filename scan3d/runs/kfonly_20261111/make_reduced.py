#!/usr/bin/env python3
"""kfonly 2026-11-11: build the reduced recording that variant B ("phone sends only poses + IMU + keyframes") reads.

  python make_reduced.py REC VIO_POSES --out RED_DIR [--quality 90] [--gt imu_gt_spl_cal1022] [--check A_poses_ba.jsonl.gz]

Keyframes = the kf_ba rule (kf_ba.py --kf-from-vio --match-ms 1 --kf-step 10): every 10th recording row whose time has a VIO
'tracking' pose within 1 ms. Uses only the live VIO poses + frame times, i.e. what the phone knows while recording.
Per keyframe: decode cam5.mp4 (only stand-in for the phone's camera image) -> resize 960x540 INTER_AREA (= the PC pipeline's
first step) -> JPEG at --quality -> RED/frames_kf/NNNNN.jpg (NNNNN = recording frame index).
RED also gets: meta.json, frames_cam5.csv, imu.csv (sent by the phone), phone_live_poses.jsonl.gz (copy of VIO_POSES),
GT dir (lens K/dist + scoring only, not "sent"), manifest.json (bytes, frames, made, inputs). RED never holds cam5.mp4.
"""
import argparse, gzip, hashlib, json, shutil, sys, time
from pathlib import Path

import cv2
import numpy as np


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec"); ap.add_argument("vio"); ap.add_argument("--out", required=True)
    ap.add_argument("--quality", type=int, default=90)
    ap.add_argument("--gt", default="imu_gt_spl_cal1022")
    ap.add_argument("--kf-step", type=int, default=10)
    ap.add_argument("--match-ms", type=float, default=1.0)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--check", default="", help="A's poses_ba.jsonl.gz: its 'frame' list must equal the keyframes here")
    ap.add_argument("--full-res", action="store_true", help="store the decoded 1920x1080 frame (no resize); the PC downscales like A")
    ap.add_argument("--size-sample", type=int, default=40, help="keyframes also encoded at other qualities / PNG (sizes only)")
    a = ap.parse_args()
    t0 = time.time()
    rec, out = Path(a.rec), Path(a.out)
    if out.exists():
        shutil.rmtree(out)
    (out / "frames_kf").mkdir(parents=True)
    g = np.load(rec / a.gt / "gt.npz")
    t_ns = g["t_ns"]
    vt = []
    for line in gzip.open(a.vio, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            vt.append(int(d["t_ns"]))
    vt = np.array(vt, np.int64)
    idx = np.searchsorted(vt, t_ns).clip(1, len(vt) - 1)
    idx = np.where(np.abs(vt[idx - 1] - t_ns) < np.abs(vt[idx] - t_ns), idx - 1, idx)
    has = np.abs(vt[idx] - t_ns) < a.match_ms * 1e6
    kfs = [int(i) for i in np.flatnonzero(has)[::a.kf_step]]
    check = None
    if a.check:
        fa = [json.loads(l)["frame"] for l in gzip.open(a.check, "rt")]
        check = dict(file=a.check, n=len(fa), equal=(sorted(fa) == kfs), subset=set(fa) <= set(kfs))
        if not check["subset"]:
            sys.exit(f"A's BA frames are not a subset of the keyframes here ({len(fa)} vs {len(kfs)})")
    W0, H0 = int(round(1920 * a.scale)), int(round(1080 * a.scale))
    want = set(kfs)
    cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
    fi, nb, got = 0, 0, 0
    samp = set(kfs[:: max(1, len(kfs) // a.size_sample)][: a.size_sample])
    ss = {"q80": 0, "q90": 0, "q95": 0, "png": 0, "q90_full1080": 0, "n": 0}
    while fi <= kfs[-1]:
        r, f = cap.read()
        if not r:
            break
        if fi in want:
            im = cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", f if a.full_res else im, [cv2.IMWRITE_JPEG_QUALITY, a.quality])
            (out / "frames_kf" / f"{fi:05d}.jpg").write_bytes(buf.tobytes())
            nb += len(buf); got += 1
            if fi in samp:
                for q in (80, 90, 95):
                    ss[f"q{q}"] += len(cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, q])[1])
                ss["png"] += len(cv2.imencode(".png", im)[1])
                ss["q90_full1080"] += len(cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 90])[1])
                ss["n"] += 1
        fi += 1
    cap.release()
    if got != len(kfs):
        sys.exit(f"decoded {got} of {len(kfs)} keyframes")
    for n in ("meta.json", "frames_cam5.csv", "imu.csv"):
        shutil.copy(rec / n, out / n)
    shutil.copy(a.vio, out / "phone_live_poses.jsonl.gz")
    shutil.copytree(rec / a.gt, out / a.gt)
    meta = json.loads((rec / "meta.json").read_text())
    dur_s = (meta["end_elapsed_ns"] - meta["start_elapsed_ns"]) * 1e-9
    imu_b = (rec / "imu.csv").stat().st_size
    import zlib
    imu_gz = len(zlib.compress((rec / "imu.csv").read_bytes(), 6))
    pose_b = Path(a.vio).stat().st_size
    n_pose = int(sum(1 for _ in gzip.open(a.vio, "rt")))
    man = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), rec=str(rec), vio=a.vio, vio_sha256=sha(a.vio), args=vars(a),
               mp4_bytes=(rec / "cam5.mp4").stat().st_size, mp4_sha256_first_64MB=hashlib.sha256(open(rec / "cam5.mp4", "rb").read(1 << 26)).hexdigest(),
               duration_s=dur_s, frames_in_video=int(meta.get("frames", 0)), save_fps=meta.get("save_fps"),
               n_keyframes=len(kfs), keyframes=kfs, jpeg_quality=a.quality, image_size=[f.shape[1], f.shape[0]] if a.full_res else [W0, H0], jpeg_bytes=nb,
               imu_csv_bytes=imu_b, imu_csv_zlib_bytes=imu_gz, live_pose_file_bytes=pose_b, live_pose_rows=n_pose,
               frames_csv_bytes=(rec / "frames_cam5.csv").stat().st_size, size_sample=ss, check_A=check,
               runtime_s=round(time.time() - t0, 1))
    (out / "manifest.json").write_text(json.dumps(man, indent=1))
    print(json.dumps({k: v for k, v in man.items() if k != "keyframes"}, indent=1))


if __name__ == "__main__":
    main()
