#!/usr/bin/env python3
"""C5 test run (docs/c5_test_plan.md): the bench (vio_bench_hs.py, as a module) with one OpenVINS init setting
(--init-cfg strict | strict-dyn | loose), clone window and start offset, through the diag bridge (tools/ov_bridge_diag,
same estimate as the shared bridge, see collapse_run.py) with the DEBUG log (init method / bias) and the full pose stream.
No learned IMU calibration (parts=raw), Android types 1/4 IMU, as in runs/collapse.

  ../venv_ffs/bin/python imu_learn/c5_run.py recordings/hs_... --init-cfg strict --clones 11 --start-off 0 \
      --max-secs 60 --out runs/c5/NAME
writes NAME.json (bench result + code version), NAME.poses.jsonl.gz, NAME.stderr.log.gz
"""
import argparse
import gzip
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vio_bench_hs as vb  # noqa: E402

DIAG_BRIDGE = vb.SCAN3D / "tools" / "ov_bridge_diag" / "build" / "ov_bridge"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--init-cfg", default="strict", choices=["strict", "strict-dyn", "loose"])
    ap.add_argument("--clones", type=int, default=11)
    ap.add_argument("--start-off", type=float, default=0.0)
    ap.add_argument("--fps", type=float, default=60)
    ap.add_argument("--size", default="640x360")
    ap.add_argument("--noise", default="cfg", choices=["cfg", "meas"],
                    help="cfg = app values (primary, plan change 1); meas (x noise-scale) = tuned on eval recs")
    ap.add_argument("--noise-scale", type=float, default=1.0)
    ap.add_argument("--max-secs", type=float, default=60)
    ap.add_argument("--out", required=True, help="output prefix (no extension)")
    o = ap.parse_args()
    out = Path(o.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    a = argparse.Namespace(rec=o.rec, fps=o.fps, size=o.size, npts=200, clones=o.clones, noise=o.noise,
                           noise_scale=o.noise_scale, imu_calib=None, dropout="", max_secs=o.max_secs, imu_raw=False,
                           gt="auto", out=None, init_cfg=o.init_cfg, start_off=o.start_off)
    load0 = open("/proc/loadavg").read().split()[:3]
    errlog = open(str(out) + ".stderr.log", "w")
    orig_popen = vb.subprocess.Popen

    def popen(args, **kw):                      # diag bridge + --diag + DEBUG log (logging only, estimate unchanged)
        if not str(args[0]).endswith("ov_bridge"):
            return orig_popen(args, **kw)
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
    try:
        r = vb.run(a)
    finally:
        vb.subprocess.Popen = orig_popen
        errlog.close()
    r.update(loadavg_start=load0, loadavg_end=open("/proc/loadavg").read().split()[:3], parts="raw", calib_used=None, bridge=str(DIAG_BRIDGE), runner="imu_learn/c5_run.py")
    json.dump(r, open(str(out) + ".json", "w"), indent=1, ensure_ascii=False)
    with gzip.open(str(out) + ".poses.jsonl.gz", "wt") as f:
        for m in saved.get("poses", []):
            f.write(json.dumps(m) + "\n")
    raw = Path(str(out) + ".stderr.log")
    with open(raw, "rb") as fi, gzip.open(str(raw) + ".gz", "wb") as fo:
        fo.write(fi.read())
    raw.unlink()
    print(json.dumps({k: r.get(k) for k in ("rec", "init_cfg", "noise", "noise_scale", "clones", "start_off", "t_init_clip_s", "covered_frac",
                                            "ate_cm", "scale", "wall_s", "bench_blob", "error")}))


if __name__ == "__main__":
    main()
