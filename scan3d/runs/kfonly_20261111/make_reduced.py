#!/usr/bin/env python3
"""kfonly 2026-11-11: build the reduced recording that variant B ("phone sends only poses + IMU + keyframes") reads.

  python make_reduced.py REC VIO_POSES --out RED_DIR [--quality 90] [--gt imu_gt_spl_cal1022] [--check A_poses_ba.jsonl.gz]

Keyframes = the kf_ba rule (kf_ba.py --kf-from-vio --match-ms 1 --kf-step 10): every 10th recording row whose time has a VIO
'tracking' pose within 1 ms. Uses only the live VIO poses + frame times, i.e. what the phone knows while recording.
Per keyframe: decode cam5.mp4 (only stand-in for the phone's camera image) -> resize 960x540 INTER_AREA (= the PC pipeline's
first step) -> JPEG at --quality -> RED/frames_kf/NNNNN.jpg (NNNNN = recording frame index).
RED also gets: meta.json, frames_cam5.csv, imu.csv (sent by the phone), phone_live_poses.jsonl.gz (copy of VIO_POSES),
GT dir (lens K/dist + scoring only, not "sent"), manifest.json (bytes, frames, made, inputs). RED never holds cam5.mp4.
kfsel 11-11: --kf-select motion = scanA/kf_select.py MotionSelector (new keyframe after >= --kf-dist m or >= --kf-rot deg since the
last keyframe, sharpest of a --kf-win frame window by --kf-blur lap|gyro|none); per-keyframe log RED/kf_select.json.
--kf-select interval (default) = the rule above, unchanged.
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
    ap.add_argument("--kf-select", default="interval", choices=["interval", "motion"])
    ap.add_argument("--kf-dist", type=float, default=0.03, help="motion: metres moved since the last keyframe (VIO world)")
    ap.add_argument("--kf-rot", type=float, default=4.0, help="motion: degrees rotated since the last keyframe")
    ap.add_argument("--kf-win", type=int, default=4, help="motion: window of VIO-fed frames (due frame + following) to pick from")
    ap.add_argument("--kf-blur", default="lap", choices=["lap", "gyro", "none"], help="motion: blur score that picks the window frame")
    ap.add_argument("--kf-max-gyro", type=float, default=0.0, help="motion: rad/s, frames above rank last in the window (0 = off)")
    a = ap.parse_args()
    t0 = time.time()
    rec, out = Path(a.rec), Path(a.out)
    if out.exists():
        shutil.rmtree(out)
    (out / "frames_kf").mkdir(parents=True)
    g = np.load(rec / a.gt / "gt.npz")
    t_ns = g["t_ns"]
    vt, vT = [], []
    for line in gzip.open(a.vio, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            vt.append(int(d["t_ns"])); vT.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    vt = np.array(vt, np.int64)
    idx = np.searchsorted(vt, t_ns).clip(1, len(vt) - 1)
    idx = np.where(np.abs(vt[idx - 1] - t_ns) < np.abs(vt[idx] - t_ns), idx - 1, idx)
    has = np.abs(vt[idx] - t_ns) < a.match_ms * 1e6
    kfs = [int(i) for i in np.flatnonzero(has)[::a.kf_step]]
    if a.kf_select == "motion":
        return motion(a, rec, out, t_ns, has, idx, vT, t0)
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
    finish(a, rec, out, kfs, nb, f, W0, H0, ss, check, t0)


def finish(a, rec, out, kfs, nb, f, W0, H0, ss, check, t0, kfsel=None):
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
               runtime_s=round(time.time() - t0, 1), kf_select=kfsel or dict(rule="interval", kf_step=a.kf_step))
    (out / "manifest.json").write_text(json.dumps(man, indent=1))
    print(json.dumps({k: v for k, v in man.items() if k != "keyframes"}, indent=1))



def motion(a, rec, out, t_ns, has, idx, vT, t0):
    """kfsel: one causal pass over the video = what the phone does live (VIO pose + gyro + image of a few-frame window)."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scanA"))
    import kf_select as KS
    sel = KS.MotionSelector(a.kf_dist, a.kf_rot, a.kf_win, a.kf_blur, a.kf_max_gyro)
    gy = KS.Gyro(rec / "imu.csv")                            # gyro always logged (and used by --kf-blur gyro)
    fc = {}
    with open(rec / "frames_cam5.csv") as fh:                 # t_mid_ns -> (exposure, rolling-shutter skew)
        next(fh)
        for line in fh:
            p = line.split(",")
            fc[int(p[1])] = (int(p[4]), int(p[5]))
    ex_med = np.median([v[0] for v in fc.values()]); sk_med = np.median([v[1] for v in fc.values()])
    W0, H0 = int(round(1920 * a.scale)), int(round(1080 * a.scale))
    hv = np.flatnonzero(has); last = int(hv[-1])
    cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
    fi, nb, kfs, infos, n_scored, f = 0, 0, [], [], 0, None

    def emit(res):
        nonlocal nb
        for k, info, img in res:
            im = img if a.full_res else cv2.resize(img, (W0, H0), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, a.quality])
            (out / "frames_kf" / f"{k:05d}.jpg").write_bytes(buf.tobytes())
            nb += len(buf); kfs.append(int(k)); infos.append(info)
    while fi <= last:
        r, f_ = cap.read()
        if not r:
            break
        f = f_
        if has[fi]:
            T = vT[idx[fi]]
            if sel.wants_image(T):
                ex, sk = fc.get(int(t_ns[fi]), (ex_med, sk_med))
                h = (ex + sk) / 2
                g_ = gy.rate(int(t_ns[fi] - h), int(t_ns[fi] + h))
                lp = KS.lap_var(f) if a.kf_blur == "lap" else None
                n_scored += 1
                emit(sel.feed(fi, T, lp, g_, payload=f))
        fi += 1
    cap.release()
    emit(sel.flush())
    if fi <= last:
        sys.exit(f"video ended at frame {fi}, last VIO frame {last}")
    (out / "kf_select.json").write_text(json.dumps(infos))
    pk = np.array([i["pos_in_window"] for i in infos]); gk = np.array([i["gyro"] for i in infos], float)
    gw = np.array([np.mean(i["gyro_window"]) for i in infos], float)
    kfsel = dict(rule="motion", dist_m=a.kf_dist, rot_deg=a.kf_rot, win=a.kf_win, blur=a.kf_blur, max_gyro=a.kf_max_gyro,
                 n_vio_frames=int(len(hv)), n_scored=n_scored, n_kf=len(kfs),
                 pos_in_window_hist=np.bincount(pk, minlength=max(1, a.kf_win)).tolist(),
                 gyro_kf_median=float(np.median(gk)), gyro_window_mean_median=float(np.median(gw)),
                 moved_m_median=float(np.median([i["moved_m"] for i in infos[1:]])) if len(infos) > 1 else None,
                 rot_deg_median=float(np.median([i["rot_deg"] for i in infos[1:]])) if len(infos) > 1 else None,
                 exposure_ns_median=float(ex_med), skew_ns_median=float(sk_med))
    print("[kf_select]", json.dumps(kfsel), flush=True)
    finish(a, rec, out, kfs, nb, f, W0, H0, {"n": 0, "note": "not sampled in motion mode"}, None, t0, kfsel)


if __name__ == "__main__":
    main()
