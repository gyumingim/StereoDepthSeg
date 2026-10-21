#!/usr/bin/env python3
"""C5 test run (docs/c5_test_plan.md): the bench (vio_bench_hs.py, as a module) with one OpenVINS init setting
(--init-cfg strict | strict-dyn | loose), clone window and start offset, through the diag bridge (tools/ov_bridge_diag,
same estimate as the shared bridge, see collapse_run.py) with the DEBUG log (init method / bias) and the full pose stream.
Default: no learned IMU calibration (parts=raw), Android types 1/4 IMU, as in runs/collapse. Overnight 2026-10-20
options: --imu-raw (types 16/35), --imu-calib JSON (+ --calib-label), --noise allan (docs/overnight_vio_prereg.md).

  ../venv_ffs/bin/python imu_learn/c5_run.py recordings/hs_... --init-cfg strict --clones 11 --start-off 0 \
      --max-secs 60 --out runs/c5/NAME
writes NAME.json (bench result + code version), NAME.poses.jsonl.gz, NAME.stderr.log.gz
"""
import argparse
import gzip
import hashlib
import os
import subprocess
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vio_bench_hs as vb  # noqa: E402

DIAG_BRIDGE = vb.SCAN3D / "tools" / "ov_bridge_diag" / "build" / "ov_bridge"


def sha256(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def binary_hashes():
    """sha256 of the diag bridge and the OpenVINS shared libraries it loads (ldd), so mixed builds are visible."""
    out = {}
    libs = [str(DIAG_BRIDGE)]
    try:
        for line in subprocess.run(["ldd", str(DIAG_BRIDGE)], capture_output=True, text=True).stdout.splitlines():
            if "=>" in line and ("ov_" in line or "open_vins" in line or "ovbuild" in line):
                libs.append(line.split("=>")[1].split("(")[0].strip())
    except OSError:
        pass
    for f in libs:
        try:
            out[f] = hashlib.sha256(open(f, "rb").read()).hexdigest()
        except OSError as e:
            out[f] = f"unreadable: {e}"
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--init-cfg", default="strict", choices=["strict", "strict-dyn", "loose"])
    ap.add_argument("--clones", type=int, default=11)
    ap.add_argument("--start-off", type=float, default=0.0)
    ap.add_argument("--fps", type=float, default=60)
    ap.add_argument("--size", default="640x360")
    ap.add_argument("--noise", default="cfg", choices=["cfg", "meas", "allan"],
                    help="cfg = app values (primary, plan change 1); meas (x noise-scale) = tuned on eval recs; "
                         "allan = long still recording floor (overnight 2026-10-20, secondary)")
    ap.add_argument("--noise-scale", type=float, default=1.0)
    ap.add_argument("--max-secs", type=float, default=60)
    ap.add_argument("--decode-threads", type=int, default=2, help="video decode threads (load control only)")
    ap.add_argument("--imu-lead", type=float, default=2.0, help="s of IMU before the first frame (bench default 2)")
    ap.add_argument("--imu-raw", action="store_true", help="uncalibrated IMU types 16/35 (new recordings only)")
    ap.add_argument("--imu-calib", default=None, help="calibration JSON {'calib': G, bg, A, ba, r} applied by the bench")
    ap.add_argument("--calib-label", default="none", help="name of the calibration (recorded only)")
    ap.add_argument("--init-window", type=float, default=None, help="override init_window_time s (bench)")
    ap.add_argument("--init-disp", type=float, default=None, help="override init_max_disparity px (bench)")
    ap.add_argument("--out", required=True, help="output prefix (no extension)")
    o = ap.parse_args()
    out = Path(o.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    a = argparse.Namespace(rec=o.rec, fps=o.fps, size=o.size, npts=200, clones=o.clones, noise=o.noise,
                           noise_scale=o.noise_scale, imu_calib=o.imu_calib, dropout="", max_secs=o.max_secs,
                           imu_raw=o.imu_raw, gt="auto", out=None, init_cfg=o.init_cfg, start_off=o.start_off,
                           decode_threads=o.decode_threads, imu_lead=o.imu_lead,
                           init_window=o.init_window, init_disp=o.init_disp)
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
    saved = {}

    def popen_rec(args, **kw):                  # also keep the exact config files the bridge reads (2026-10-21 verdict)
        p = popen(args, **kw)
        if str(args[0]).endswith("ov_bridge"):
            d = Path(args[1]).parent
            saved["ov_config"] = {f.name: f.read_text() for f in sorted(d.glob("*.yaml"))}
        return p
    vb.subprocess.Popen = popen_rec
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
    r.update(binary_sha256=binary_hashes(), decode_threads=o.decode_threads,
             loadavg_start=load0, loadavg_end=open("/proc/loadavg").read().split()[:3],
             parts="raw" if not o.imu_calib else "calib", calib_label=o.calib_label,
             calib_used=json.load(open(o.imu_calib))["calib"] if o.imu_calib else None,
             calib_file=o.imu_calib, calib_sha256=sha256(o.imu_calib) if o.imu_calib else None,
             allan_sha256=sha256(vb.ALLAN_JSON) if o.noise == "allan" else None,
             bridge=str(DIAG_BRIDGE), runner="imu_learn/c5_run.py",
             runner_blob=subprocess.run(["git", "-C", str(vb.SCAN3D), "hash-object", "imu_learn/c5_run.py"],
                                        capture_output=True, text=True).stdout.strip())
    with gzip.open(str(out) + ".poses.jsonl.gz", "wt") as f:
        for m in saved.get("poses", []):
            f.write(json.dumps(m) + "\n")
    raw = Path(str(out) + ".stderr.log")
    with open(raw, "rb") as fi, gzip.open(str(raw) + ".gz", "wb") as fo:
        fo.write(fi.read())
    raw.unlink()
    # actual config + initial state, per run (2026-10-21 verdict 11 / today item 5)
    cfg = saved.get("ov_config", {})
    r["ov_config_files"] = cfg                                   # exact yaml text the bridge read (after the DEBUG edit)
    r["ov_config_sha256"] = {k: hashlib.sha256(v.encode()).hexdigest() for k, v in cfg.items()}
    try:
        import collapse_signals as cs
        r["init_info"] = cs.parse_log(str(raw) + ".gz")["init_info"]   # method, disparity, bg/v/ba from the DEBUG log (4 dp)
    except Exception as e:  # noqa: BLE001
        r["init_info"] = {"error": repr(e)}
    first = next((m for m in saved.get("poses", []) if m.get("status") == "tracking"), None)
    # full precision, but AFTER the first update (differs slightly from init_info bg_init/ba_init = value at init)
    r["state_first_tracking_pose"] = ({k: first.get(k) for k in ("t_ns", "bg", "ba", "v")} if first else None)
    json.dump(r, open(str(out) + ".json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps({k: r.get(k) for k in ("rec", "init_cfg", "noise", "noise_scale", "clones", "start_off", "t_init_clip_s", "covered_frac",
                                            "ate_cm", "scale", "wall_s", "bench_blob", "error")}))


if __name__ == "__main__":
    main()
