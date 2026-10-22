#!/usr/bin/env python3
"""Collapse-detection run: the fixed bench (vio_bench_hs.py, unchanged, as a module) with the same settings as
calib_vio_ablation.py (runs/vio_calibcheck), but with the --diag bridge (tools/ov_bridge_diag/build/ov_bridge, source tools/ov_bridge_diag/main.cpp: pose lines also
carry the IMU-state standard deviations sig15 and the cam0 tracked-feature count n_track; linked to a COPY of OpenVINS
~/scanner/src/open_vins_diag -> ~/scanner/src/ovbuild_diag whose only change is log lines: [CHI2] per MSCKF/SLAM update =
features tested / rejected / mean chi2 ratio, [DIAGT] frame time) and the FULL pose stream saved
(jsonl.gz: p, q, v, bg, ba, sig15, n_track, n_slam, n_msckf, ...) plus the OpenVINS DEBUG log (init method, chi2, ZUPT).
The extra output does not change the estimate (same input -> same numbers; checked bit-identical against the calibcheck
.poses.npz of 145816 bias). CAUTION: dynamic init has a wall-clock budget (init_dyn_mle_max_time 0.05 s), so under heavy
CPU load the same input can initialize differently (seen: 145816 full).
--start-off s: first VIO frame s seconds later (IMU unchanged) = another initialization on the same recording.

  ../venv_ffs/bin/python imu_learn/collapse_run.py recordings/hs_... --parts raw [--calib runs/on_mid_s0_<rec>/result.json] \
      --noise meas --noise-scale 10 --max-secs 30 --out runs/collapse/NAME
writes NAME.json (bench summary over the run), NAME.poses.jsonl.gz, NAME.stderr.log.gz
"""
import argparse
import gzip
import json
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vio_bench_hs as vb  # noqa: E402
from calib_vio_ablation import PARTS, ablated  # noqa: E402

DIAG_BRIDGE = vb.SCAN3D / "tools" / "ov_bridge_diag" / "build" / "ov_bridge"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--calib", default=None)
    ap.add_argument("--parts", default="raw", choices=list(PARTS))
    ap.add_argument("--fps", type=float, default=60)
    ap.add_argument("--size", default="640x360")
    ap.add_argument("--noise", default="meas", choices=["cfg", "meas"])
    ap.add_argument("--noise-scale", type=float, default=10.0)
    ap.add_argument("--max-secs", type=float, default=30)
    ap.add_argument("--start-off", type=float, default=0.0)
    ap.add_argument("--out", required=True, help="output prefix (no extension)")
    o = ap.parse_args()
    out = Path(o.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    a = argparse.Namespace(rec=o.rec, fps=o.fps, size=o.size, npts=200, clones=11, noise=o.noise,
                           noise_scale=o.noise_scale, imu_calib=None, dropout="", max_secs=o.max_secs, imu_raw=False,
                           gt="auto", out=None, gyro_rebias=False)
    used = None
    if PARTS[o.parts] is not None:
        used = ablated(json.load(open(o.calib))["calib"], PARTS[o.parts])
        tf = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, dir=str(vb.SCAN3D / "runs"))
        json.dump(dict(calib=used), tf)
        tf.close()
        a.imu_calib = tf.name
    if o.start_off:                               # first VIO frame s seconds later (IMU feed unchanged) -> another init
        orig_pick = vb.pick_by_time
        vb.pick_by_time = lambda tf, t0, t1, fps, per: orig_pick(tf, t0 + int(o.start_off * 1e9), t1, fps, per)
        a.max_secs = o.max_secs + o.start_off
    errlog = open(str(out) + ".stderr.log", "w")
    orig_popen = vb.subprocess.Popen

    def popen(args, **kw):                      # diag bridge + --diag + DEBUG log (logging only, estimate unchanged)
        cfg = Path(args[1])
        cfg.write_text(cfg.read_text().replace('verbosity: "WARNING"', 'verbosity: "DEBUG"'))
        args = [str(DIAG_BRIDGE)] + list(args[1:]) + ["--diag"]
        kw["stderr"] = errlog
        return orig_popen(args, **kw)
    vb.subprocess.Popen = popen
    saved = {}
    orig_eval = vb.evaluate

    def ev(rec, a_, poses, *rest):
        saved["poses"] = poses
        return orig_eval(rec, a_, poses, *rest)
    vb.evaluate = ev
    r = vb.run(a)
    vb.subprocess.Popen = orig_popen              # vb.subprocess IS the subprocess module: restore before git calls
    errlog.close()
    if a.imu_calib:
        Path(a.imu_calib).unlink(missing_ok=True)
    import subprocess as sp
    git = lambda *c: sp.run(['git', '-C', str(vb.SCAN3D), *c], capture_output=True, text=True).stdout.strip()
    r.update(parts=o.parts, calib_file=o.calib, calib_used=used, bridge=str(DIAG_BRIDGE), start_off=o.start_off,
             git_head=git('rev-parse', '--short', 'HEAD'), bench_blob=git('hash-object', 'imu_learn/vio_bench_hs.py'))
    json.dump(r, open(str(out) + ".json", "w"), indent=1, ensure_ascii=False)
    with gzip.open(str(out) + ".poses.jsonl.gz", "wt") as f:
        for m in saved.get("poses", []):
            f.write(json.dumps(m) + "\n")
    raw = Path(str(out) + ".stderr.log")
    with open(raw, "rb") as fi, gzip.open(str(raw) + ".gz", "wb") as fo:
        fo.write(fi.read())
    raw.unlink()
    print(json.dumps({k: r.get(k) for k in ("rec", "parts", "ate_cm", "scale", "tracked_frac", "wall_s")}))


if __name__ == "__main__":
    main()
