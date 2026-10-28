#!/usr/bin/env python3
"""npunight 2026-10-29 (copy of appnpu_replay.py; the bridge stdin is also tee-d to OUT.stream.bin for the phone replay): replay a std60 recording through the PHONE config folder with a given PC bridge (lens05
phone_cfg_replay.py method: raw 1920x1080 gray + IMU via vio_bench_hs, the bridge remaps with the phone remap), profile
rec60 + loop closure (--loop 1 --loop-sync 1 = deterministic), and save every pose line + the loop log.

  python appnpu_replay.py REC CFG_DIR BRIDGE OUTPREFIX [--extra "ARGS"] [--max-secs S]
Writes OUTPREFIX.poses.jsonl, OUTPREFIX.loop.jsonl, OUTPREFIX.json (score = reference only), OUTPREFIX.bridge.err
"""
import argparse, json, os, re, stat, sys, tempfile, time, hashlib
from pathlib import Path
import numpy as np, cv2

L = Path.home() / "scanner/StereoDepthSeg-lens05/scan3d"
MAIN = Path.home() / "scanner/StereoDepthSeg/scan3d"
sys.path.insert(0, str(L / "imu_learn")); sys.path.insert(0, str(L))
os.chdir(MAIN)
import vio_bench_hs as vb  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("rec"); ap.add_argument("cfg"); ap.add_argument("bridge"); ap.add_argument("out")
ap.add_argument("--extra", default=""); ap.add_argument("--max-secs", type=float, default=0)
ap.add_argument("--vocab", default=str(Path.home() / "scanner/StereoDepthSeg-appnpu/scan3d/ov_bridge/loop/voc_k10L4.voc"))
x = ap.parse_args()
src_cfg = Path(x.cfg).resolve()
cfg = Path(tempfile.mkdtemp(prefix="appnpucfg_"))
for f in src_cfg.iterdir():
    if f.suffix == ".yaml" or f.name == "remap_1920x1080.bin":
        (cfg / f.name).write_bytes(f.read_bytes())
t = (cfg / "estimator_config.yaml").read_text()
t = re.sub(r'"/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov/[A-Za-z0-9_]+/', f'"{cfg}/', t)
(cfg / "estimator_config.yaml").write_text(t)
assert "/storage/" not in t


def ident_maps(rec, meta, size):
    W, H = int(meta["width"]), int(meta["height"])
    mx, my = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    m1, m2 = cv2.convertMaps(mx, my, cv2.CV_16SC2)
    return m1, m2, np.array([[1000.0, 0, W / 2], [0, 1000.0, H / 2], [0, 0, 1]]), (W, H)


vb.undistort_maps = ident_maps
wrap = Path(tempfile.mkdtemp(prefix="appnpu_")) / "br.sh"
wrap.write_text(f'#!/bin/bash\nshift\ntee {x.out}.stream.bin | {x.bridge} {cfg}/estimator_config.yaml --remap {cfg}/remap_1920x1080.bin '
                f'--profile rec60 --drop-late 0 --loop 1 --loop-sync 1 --loop-vocab {x.vocab} --loop-log {x.out}.loop.jsonl '
                f'{x.extra} "$@" 2> {x.out}.bridge.err\n')
wrap.chmod(wrap.stat().st_mode | stat.S_IEXEC)
vb.BRIDGE = wrap
saved = {}
orig = vb.evaluate


def ev(rec_, a_, poses, *rest):
    saved["poses"] = poses
    return orig(rec_, a_, poses, *rest)


vb.evaluate = ev
a = argparse.Namespace(rec=x.rec, fps=60.0, size="1920x1080", npts=200, clones=15, noise="cfg", noise_scale=1.0,
                       imu_calib=None, init_cfg="strict", start_off=0.0, imu_lead=2.0, init_window=None, init_disp=None,
                       decode_threads=0, dropout="", max_secs=x.max_secs, imu_raw=False, cfg_over="", still_hold=None,
                       gt="auto", focus_comp=False, focus_k=0.008, focus_tol_px=0.05, drop_lens_moving=False,
                       focus_table=None, gyro_rebias=None, frame_cache=False, lever="config/imu_cam_lever.json",
                       lens_file=None, out=None)
t0 = time.time()
r = vb.run(a)
r.pop("traj", None)
r.update(appnpu_made=time.strftime("%F %T"), bridge=x.bridge, bridge_sha256=hashlib.sha256(open(x.bridge, "rb").read()).hexdigest(),
         extra=x.extra, phone_cfg=str(src_cfg), secs=round(time.time() - t0, 1))
with open(x.out + ".poses.jsonl", "w") as f:
    for m in saved.get("poses") or []:
        f.write(json.dumps(m, sort_keys=True) + "\n")
json.dump(r, open(x.out + ".json", "w"), indent=1)
print(json.dumps({k: r.get(k) for k in ("rec", "frames_fed", "poses", "tracked", "ate_cm", "rot_med_deg", "error", "secs")}))
