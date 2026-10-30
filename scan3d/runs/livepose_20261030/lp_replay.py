#!/usr/bin/env python3
"""livepose 2026-10-30: PC replay of ONE recording through the LIVE phone configuration (app 0.6.38c, profile rec60).

  python lp_replay.py REC CFG_DIR OUT_PREFIX [options]

Live path on the phone (read from MainActivity.java / OvRunner.java on appfix-20261029, not from READMEs):
  ov_cam main, 1920x1080 Y plane @ 60 fps, focus 0.5 dpt, exp 2 ms + auto ISO, IMU = UNCALIBRATED types 35/16 values 0..2
  (no Android bias, no rebias), frame time = SENSOR_TIMESTAMP + (exposure + RS skew)/2 = mid exposure, no per-recording
  time shift (yaml timeshift_cam_imu 2.172 ms + online calibration), preset "bench" bridge args (--max-speed 8
  --lost-frames 1e6 --startup-lost-frames 1e6), --remap files/ov/c4f60/remap_1920x1080.bin (raw 1920x1080 -> 960x540),
  profile rec60 flags (ovopt all_exact,...,gyro_klt=5, drop-late 50, loop 1 BoW, health, predict 2, idle spin 2).
This replay (differences, all written to OUT.json "input_path"):
  - frames: cam5.mp4 (H.264, 160 Mbit/s for std60) decoded by OpenCV/FFmpeg -> BGR -> GRAY (phone: camera Y plane, no codec)
  - remap: the bridge itself with the phone remap file (same code path as the phone)
  - selection: vio_bench_hs.fed_selection (60 fps grid; HS 240 fps recordings: nearest frame per 1/60 s)
  - IMU: --imu raw (default, = phone) or cal; acc interpolated to gyro times (= OvRunner.flushImu)
  - time: --time live (default): frames at t_mid, no shift (= phone); --time oracle: frames shifted by the recording's
    GT align dt and yaml timeshift set to 0 (= bench; uses GT knowledge, not available live)
  - offline: --drop-late 0 (wall-clock rule, nondeterministic offline), --loop-sync 1 (deterministic loop closure; phone =
    async thread, map pose corrections arrive later), --idle-spin 0 --predict 0 (timing only, poses unchanged; saves PC CPU)
Writes OUT.poses.jsonl.gz, OUT.loop.jsonl, OUT.bridge.err, OUT.json (score from lp_score.py).
"""
import argparse, gzip, hashlib, json, os, re, struct, subprocess, sys, tempfile, threading, time
from pathlib import Path
import numpy as np, cv2

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parents[1]
sys.path.insert(0, str(SCAN3D / "imu_learn")); sys.path.insert(0, str(SCAN3D)); sys.path.insert(0, str(HERE))
import vio_bench_hs as vb  # noqa: E402
from imu_gt import read_imu  # noqa: E402
import lp_score  # noqa: E402

HDR, MAGIC = struct.Struct("<III"), 0x4F564252
TMP = Path.home() / "scanner/tmp_livepose"


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec"); ap.add_argument("cfg"); ap.add_argument("out")
    ap.add_argument("--bridge", default=str(TMP / "bin/ov_bridge_pc"))
    ap.add_argument("--vocab", default=str(TMP / "bin/voc_k10L4.voc"))
    ap.add_argument("--imu", choices=["raw", "cal"], default="raw")
    ap.add_argument("--time", choices=["live", "oracle"], default="live")
    ap.add_argument("--yaml", default="", help="estimator_config.yaml overrides k=v,k=v")
    ap.add_argument("--chain", default="", help="kalibr chain overrides k=v (imu noise keys / timeshift_cam_imu)")
    ap.add_argument("--noise-scale", type=float, default=1.0, help="x all 4 IMU noise values")
    ap.add_argument("--remap", default="", help="other remap_1920x1080.bin (lens ablation); needs --intr")
    ap.add_argument("--intr", default="", help="fx,fy,cx,cy for the imucam yaml with --remap")
    ap.add_argument("--extra", default="", help="extra bridge args after the profile (override)")
    ap.add_argument("--loop", type=int, default=1)
    ap.add_argument("--start-off", type=float, default=0.0)
    ap.add_argument("--max-secs", type=float, default=0.0)
    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--decode-threads", type=int, default=2)
    ap.add_argument("--drop-every", type=int, default=0, help="drop every Nth fed frame (timing emulation)")
    ap.add_argument("--gt", default="auto")
    x = ap.parse_args()

    rec = Path(x.rec).resolve()
    meta = json.load(open(rec / "meta.json"))
    out = Path(x.out); out.parent.mkdir(parents=True, exist_ok=True)
    # ---- config: copy of the phone folder (= OvRunner.prepareConfig: output paths local), overrides applied
    cfg = Path(tempfile.mkdtemp(prefix="lpcfg_", dir=str(TMP / "cfgtmp") if (TMP / "cfgtmp").is_dir() else None))
    src = Path(x.cfg).resolve()
    for f in src.iterdir():
        if f.suffix == ".yaml":
            (cfg / f.name).write_text(f.read_text())
    remap = Path(x.remap) if x.remap else src / "remap_1920x1080.bin"
    est = (cfg / "estimator_config.yaml").read_text()
    est = re.sub(r'"/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov/[A-Za-z0-9_]+/', f'"{cfg}/', est)
    for kv in filter(None, x.yaml.split(",")):
        k, v = kv.split("=", 1)
        est, n = re.subn(rf"(?m)^{re.escape(k)}:.*$", f"{k}: {v}", est)
        if n != 1:
            sys.exit(f"yaml key {k}: {n} matches")
    (cfg / "estimator_config.yaml").write_text(est)
    imu_y = (cfg / "kalibr_imu_chain.yaml").read_text()
    for k in ("accelerometer_noise_density", "accelerometer_random_walk", "gyroscope_noise_density", "gyroscope_random_walk"):
        if x.noise_scale != 1.0:
            m = re.search(rf"(?m)^(\s*{k}:\s*)([-0-9.eE+]+)", imu_y)
            imu_y = imu_y[:m.start(2)] + f"{float(m.group(2)) * x.noise_scale:.4e}" + imu_y[m.end(2):]
    cam_y = (cfg / "kalibr_imucam_chain.yaml").read_text()
    al = json.load(open(rec / "imu_gt" / "align.json"))
    dt_gt = int(al["dt_ns"])
    if x.time == "oracle":
        cam_y = re.sub(r"(?m)^(\s*timeshift_cam_imu:).*$", r"\1 0.0", cam_y)
    for kv in filter(None, x.chain.split(",")):
        k, v = kv.split("=", 1)
        tgt = "cam" if k in ("timeshift_cam_imu",) else "imu"
        if tgt == "cam":
            cam_y, n = re.subn(rf"(?m)^(\s*{k}:).*$", rf"\1 {v}", cam_y)
        else:
            imu_y, n = re.subn(rf"(?m)^(\s*{k}:).*$", rf"\1 {v}", imu_y)
        if n != 1:
            sys.exit(f"chain key {k}: {n} matches")
    if x.intr:
        cam_y, n = re.subn(r"(?m)^(\s*intrinsics:).*$", r"\1 [" + ", ".join(x.intr.split(",")) + "]", cam_y)
        assert n == 1
    elif x.remap:
        sys.exit("--remap needs --intr (rectified pinhole of that remap)")
    (cfg / "kalibr_imu_chain.yaml").write_text(imu_y)
    (cfg / "kalibr_imucam_chain.yaml").write_text(cam_y)
    assert "/storage/" not in est

    cmd = [x.bridge, str(cfg / "estimator_config.yaml"), "--stdio", "--max-speed", "8", "--lost-frames", "1000000",
           "--startup-lost-frames", "1000000", "--remap", str(remap), "--profile", "rec60", "--drop-late", "0",
           "--idle-spin", "0", "--predict", "0"]
    if x.loop:
        cmd += ["--loop", "1", "--loop-sync", "1", "--loop-vocab", x.vocab, "--loop-log", str(out) + ".loop.jsonl"]
    else:
        cmd += ["--loop", "0"]
    cmd += x.extra.split()

    # ---- inputs
    rows = list(__import__("csv").DictReader(open(rec / "frames_cam5.csv")))
    fs = vb.fed_selection(rec, meta, rows, x.fps, x.start_off, x.max_secs)
    t_frames, t_end, sel = fs["t_frames"], fs["t_end"], fs["sel"]
    selected = np.zeros(len(t_frames), bool); selected[sel] = True
    acc, gyr = read_imu(rec / "imu.csv", raw=(x.imu == "raw"))
    a_i = np.stack([np.interp(gyr[:, 0], acc[:, 0], acc[:, 1 + j]) for j in range(3)], 1)
    g_i = gyr[:, 1:4].copy()
    t_imu = gyr[:, 0].astype(np.int64)
    shift = dt_gt if x.time == "oracle" else 0
    errf = open(str(out) + ".bridge.err", "w")
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=errf, bufsize=0)
    poses = []

    def reader():
        for line in p.stdout:
            try:
                m = json.loads(line)
            except Exception:
                continue
            if m.get("type") == "pose":
                poses.append(m)
    th = threading.Thread(target=reader, daemon=True); th.start()
    cap = cv2.VideoCapture(str(rec / "cam5.mp4"), cv2.CAP_FFMPEG, [cv2.CAP_PROP_N_THREADS, x.decode_threads])
    W, H = int(meta["width"]), int(meta["height"])
    ii = int(np.searchsorted(t_imu, fs["t_start"] - shift - int(2.0 * 1e9)))
    n_fed, n_sel, crashed, t0 = 0, 0, None, time.time()
    try:
        for fi in range(len(rows)):
            if not cap.grab():
                break
            tc = t_frames[fi]
            if tc > t_end:
                break
            if not selected[fi]:
                continue
            n_sel += 1
            tci = tc - shift
            while ii < len(t_imu) and t_imu[ii] <= tci + 5_000_000:
                p.stdin.write(HDR.pack(MAGIC, 1, 56) + struct.pack("<q3d3d", int(t_imu[ii]), *g_i[ii], *a_i[ii]))
                ii += 1
            if x.drop_every and n_sel % x.drop_every == 0:
                continue
            _, f = cap.retrieve()
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            assert g.shape == (H, W)
            body = struct.pack("<qIIB3x", int(tci), W, H, 1) + np.ascontiguousarray(g).tobytes()
            p.stdin.write(HDR.pack(MAGIC, 2, len(body)) + body)
            n_fed += 1
        for _ in range(200):
            if ii >= len(t_imu):
                break
            p.stdin.write(HDR.pack(MAGIC, 1, 56) + struct.pack("<q3d3d", int(t_imu[ii]), *g_i[ii], *a_i[ii]))
            ii += 1
        p.stdin.write(HDR.pack(MAGIC, 4, 0)); p.stdin.close()
    except BrokenPipeError:
        crashed = f"bridge exited rc {p.poll()} after {n_fed} frames"
    p.wait(timeout=300); th.join(timeout=10); errf.close()
    wall = time.time() - t0
    with gzip.open(str(out) + ".poses.jsonl.gz", "wt") as f:
        for m in poses:
            f.write(json.dumps(m, sort_keys=True) + "\n")
    res = dict(rec=rec.name, made=time.strftime("%F %T"), args=vars(x), bridge_cmd=" ".join(cmd),
               bridge_sha=sha(x.bridge), remap_sha=sha(remap), cfg_src=str(src),
               yaml_sha={f.name: sha(f) for f in sorted(cfg.glob("*.yaml"))},
               imucam_intrinsics=re.search(r"intrinsics:.*", cam_y).group(0),
               timeshift_yaml=re.search(r"timeshift_cam_imu:.*", cam_y).group(0),
               input_path=dict(decode="cam5.mp4 OpenCV-FFmpeg H.264 -> BGR -> GRAY 1920x1080 (phone: camera Y plane, no codec)",
                               remap="bridge --remap " + remap.name + " sha " + sha(remap),
                               imu=("uncalibrated 35/16 v0..2, no rebias (= phone)" if x.imu == "raw" else "Android calibrated 1/4 (!= phone)"),
                               time=("t_mid, no shift (= phone)" if x.time == "live" else f"t_mid - GT dt {dt_gt / 1e6:.3f} ms, yaml timeshift 0 (oracle)"),
                               offline="drop-late 0, loop-sync 1, idle-spin 0, predict 0",
                               selection=f"{x.fps:g} fps grid from save_fps {meta.get('save_fps')}, start_off {x.start_off}"),
               frames_selected=n_sel, frames_fed=n_fed, poses=len(poses), wall_s=round(wall, 1), crashed=crashed,
               focus_dpt_rec=meta.get("focus_dpt_request"), save_fps=meta.get("save_fps"), gt_align_dt_ms=dt_gt / 1e6,
               t_first_fed=int(t_frames[sel[0]] - shift), t_last_fed=int(t_frames[sel[-1]] - shift))
    res["score"] = lp_score.score(rec, poses, shift, res["t_first_fed"], res["t_last_fed"], gt=x.gt)
    json.dump(res, open(str(out) + ".json", "w"), indent=1)
    s = res["score"]
    print(json.dumps(dict(rec=rec.name, fed=n_fed, poses=len(poses), wall=res["wall_s"], crashed=crashed,
                          **{k: s.get(k) for k in ("status", "init_s", "ate_vio_cm", "ate_map_cm", "path_m", "ate_pct",
                                                    "drift_cm_per_m", "loop_err_cm", "dt_final_ms")})))


if __name__ == "__main__":
    main()
