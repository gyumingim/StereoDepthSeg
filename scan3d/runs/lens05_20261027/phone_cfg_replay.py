#!/usr/bin/env python3
"""lens05: replay a std60 recording through the PHONE config folder (estimator yaml + remap_1920x1080.bin, exactly the
files pushed to files/ov/<cfg>/) with the PC bridge, using vio_bench_hs for frame/IMU feeding and GT scoring.

  python phone_cfg_replay.py REC CFG_DIR OUT.json [--fps 60] [--max-secs S]

How: vio_bench_hs.run() normally remaps frames itself with its own lens; here undistort_maps is replaced by an identity
1920x1080 map (raw gray frames go to the bridge, like the phone sends the raw Y plane), and BRIDGE by a wrapper that
ignores the bench yaml and runs the bridge with CFG_DIR/estimator_config.yaml + --remap CFG_DIR/remap_1920x1080.bin
--profile rec60 --drop-late 0 --loop 0 (+ the bench's own args). Differences from the phone (both lenses the same):
frames are on the IMU clock (bench shifts by the recording's dt) while the phone yaml also has timeshift 2.17 ms;
R_ci / lever = phone values (mean), not per recording. Scoring = vio_bench_hs.evaluate (ATE vs camera GT) = reference only.
"""
import argparse, json, os, stat, sys, tempfile
from pathlib import Path
import numpy as np, cv2

L = Path.home() / "scanner/StereoDepthSeg-lens05/scan3d"
MAIN = Path.home() / "scanner/StereoDepthSeg/scan3d"
BR = Path.home() / "scanner/tmp_integ2l/ate/bin/ov_bridge_81e547e9dafa"
sys.path.insert(0, str(L / "imu_learn")); sys.path.insert(0, str(L))
os.chdir(MAIN)
import vio_bench_hs as vb  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("rec"); ap.add_argument("cfg"); ap.add_argument("out")
ap.add_argument("--fps", type=float, default=60); ap.add_argument("--max-secs", type=float, default=0)
x = ap.parse_args()
src_cfg = Path(x.cfg).resolve()
# run-dir copy like OvRunner.prepareConfig: OpenVINS output paths (phone /storage/...) -> a local dir; remap + yaml unchanged
cfg = Path(tempfile.mkdtemp(prefix="lens05cfg_"))
for f in src_cfg.iterdir():
    if f.suffix == ".yaml" or f.name == "remap_1920x1080.bin":
        (cfg / f.name).write_bytes(f.read_bytes())
t = (cfg / "estimator_config.yaml").read_text()
import re as _re
t = _re.sub(r'"/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov/[A-Za-z0-9_]+/', f'"{cfg}/', t)
(cfg / "estimator_config.yaml").write_text(t)
assert "/storage/" not in t


def ident_maps(rec, meta, size):
    W, H = int(meta["width"]), int(meta["height"])
    mx, my = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    m1, m2 = cv2.convertMaps(mx, my, cv2.CV_16SC2)
    return m1, m2, np.array([[1000.0, 0, W / 2], [0, 1000.0, H / 2], [0, 0, 1]]), (W, H)


vb.undistort_maps = ident_maps
wrap = Path(tempfile.mkdtemp(prefix="lens05_")) / "br.sh"
wrap.write_text(f'#!/bin/bash\nshift\nexec {BR} {cfg}/estimator_config.yaml --remap {cfg}/remap_1920x1080.bin '
                f'--profile rec60 --drop-late 0 --loop 0 "$@" 2> {x.out}.bridge.err\n')
wrap.chmod(wrap.stat().st_mode | stat.S_IEXEC)
vb.BRIDGE = wrap
a = argparse.Namespace(rec=x.rec, fps=x.fps, size="1920x1080", npts=200, clones=15, noise="cfg", noise_scale=1.0,
                       imu_calib=None, init_cfg="strict", start_off=0.0, imu_lead=2.0, init_window=None, init_disp=None,
                       decode_threads=0, dropout="", max_secs=x.max_secs, imu_raw=False, cfg_over="", still_hold=None,
                       gt="auto", focus_comp=False, focus_k=0.008, focus_tol_px=0.05, drop_lens_moving=False,
                       focus_table=None, gyro_rebias=None, frame_cache=False, lever="config/imu_cam_lever.json",
                       lens_file=None, out=None)
r = vb.run(a)
r["phone_cfg"] = str(src_cfg)
r.pop("traj", None)
json.dump(r, open(x.out, "w"), indent=1)
print(json.dumps({k: r.get(k) for k in ("rec", "frames_fed", "poses", "tracked", "tracked_frac", "ate_cm", "ate_sim3_cm", "scale",
                                         "rot_med_deg", "rpe1s_cm", "error")}))
