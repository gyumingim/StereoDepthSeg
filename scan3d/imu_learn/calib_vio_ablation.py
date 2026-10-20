#!/usr/bin/env python3
"""VIO ablation of the physical IMU calibration on the fixed bench (vio_bench_hs.py, used unchanged as a module).

  ../venv_ffs/bin/python imu_learn/calib_vio_ablation.py recordings/hs_... --calib runs/on_X_<rec>/result.json \
      --parts raw|lever|bias|bias_G|bias_G_A|full [--start-off 0.5] [--noise meas --noise-scale 10] --out runs/vio_calibcheck/NAME.json

parts: raw = no calibration (bench default, p_CinI = 0); lever = only r (camera position in IMU axes -> OpenVINS
p_CinI); bias = bg, ba only; bias_G = + gyro matrix G; bias_G_A = + accel matrix A; full = all incl. lever r.
--start-off s: the first VIO frame is s seconds later (IMU feed unchanged). OpenVINS is deterministic for identical
input (same JSON twice = same numbers), so a 'repeat' is a run with another start offset — it shows how fragile
initialization is. The pose stream (p, v, biases, extrinsic, time offset) is saved next to the JSON (.poses.npz) for
diagnosing collapses.
"""
import argparse
import json
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vio_bench_hs as vb  # noqa: E402

PARTS = {"raw": None, "lever": ("r",), "bias": ("bg", "ba"), "bias_G": ("bg", "ba", "G"),
         "bias_G_A": ("bg", "ba", "G", "A"), "full": ("bg", "ba", "G", "A", "r")}


def ablated(cal, keep):
    out = dict(G=np.eye(3).tolist(), A=np.eye(3).tolist(), bg=[0.0] * 3, ba=[0.0] * 3, r=[0.0] * 3)
    for k in keep:
        out[k] = cal[k]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--calib", required=True)
    ap.add_argument("--parts", required=True, choices=list(PARTS))
    ap.add_argument("--start-off", type=float, default=0.0)
    ap.add_argument("--fps", type=float, default=60)
    ap.add_argument("--size", default="640x360")
    ap.add_argument("--noise", default="cfg", choices=["cfg", "meas"])
    ap.add_argument("--noise-scale", type=float, default=1.0)
    ap.add_argument("--max-secs", type=float, default=60)
    ap.add_argument("--out", required=True)
    o = ap.parse_args()
    a = argparse.Namespace(rec=o.rec, fps=o.fps, size=o.size, npts=200, clones=11, noise=o.noise, noise_scale=o.noise_scale,
                           imu_calib=None, dropout="", max_secs=o.max_secs, imu_raw=False, gt="auto", out=None)
    cal_full = json.load(open(o.calib))["calib"]
    used = None
    if PARTS[o.parts] is not None:
        used = ablated(cal_full, PARTS[o.parts])
        tf = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, dir=str(vb.SCAN3D / "runs"))
        json.dump(dict(calib=used), tf)
        tf.close()
        a.imu_calib = tf.name
    if o.start_off:
        orig_pick = vb.pick_by_time
        vb.pick_by_time = lambda t_frames, t_start, t_end, fps: orig_pick(t_frames, t_start + int(o.start_off * 1e9), t_end, fps)
        if o.max_secs:                                   # keep the scored length the same
            a.max_secs = o.max_secs + o.start_off
    saved = {}
    orig_eval = vb.evaluate

    def ev(rec, a_, poses, *rest):
        saved["poses"] = poses
        return orig_eval(rec, a_, poses, *rest)
    vb.evaluate = ev
    r = vb.run(a)
    if a.imu_calib:
        Path(a.imu_calib).unlink(missing_ok=True)
    r.update(parts=o.parts, start_off=o.start_off, calib_file=o.calib, calib_used=used)
    out = Path(o.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    json.dump(r, open(out, "w"), indent=1, ensure_ascii=False)
    P = saved.get("poses", [])

    def col(k, n):
        return np.array([m[k] if m.get(k) is not None else [np.nan] * n for m in P], float).reshape(len(P), n)
    np.savez_compressed(out.with_suffix(".poses.npz"), t_ns=np.array([m.get("t_ns", 0) for m in P], np.int64),
                        status=np.array([m.get("status", "") for m in P]), p=col("p", 3),
                        v=col("v", 3), bg=col("bg", 3), ba=col("ba", 3), p_IinC=col("p_IinC0", 3),
                        dt_cam_imu=np.array([m.get("dt_cam_imu", np.nan) or np.nan for m in P], float),
                        T=np.array([np.asarray(m["T_world_cam0"]).reshape(16) if m.get("T_world_cam0") else np.full(16, np.nan) for m in P]))
    print(json.dumps({k: r.get(k) for k in ("rec", "parts", "start_off", "noise", "noise_scale", "ate_cm", "scale", "tracked_frac")}))


if __name__ == "__main__":
    main()
