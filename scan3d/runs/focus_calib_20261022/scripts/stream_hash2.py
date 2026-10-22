"""Feed-stream identity test: run the bench (old HEAD copy vs new) with a fake bridge that only sha256s the exact byte
stream (IMU + frames) the bench sends. usage: stream_hash.py OLD_COPY CASE..."""
import argparse, importlib.util, json, os, shutil, sys, time
from pathlib import Path

S3 = Path.home() / "scanner/StereoDepthSeg/scan3d"
sys.path[:0] = [str(S3), str(S3 / "imu_learn")]
TMP = Path.home() / "fc_tmp"
FAKE = TMP / "fakebridge.sh"
FAKE.write_text('#!/bin/sh\nexec sha256sum > "$FAKE_OUT"\n')
os.chmod(FAKE, 0o755)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    m.SCAN3D = S3
    m.TEMPLATE = S3 / "config/mono/estimator_config.yaml"
    m.BRIDGE = FAKE
    return m


def ns(rec, **kw):
    a = dict(rec=str(S3 / "recordings" / rec), fps=60, size="640x360", npts=200, clones=11, noise="cfg", noise_scale=1.0,
             imu_calib=None, dropout="", max_secs=0, imu_raw=False, gt="auto", out=None, init_cfg="strict",
             start_off=0.0, decode_threads=2, imu_lead=2.0, init_window=None, init_disp=None, cfg_over="")
    a.update(kw)
    return argparse.Namespace(**a)


def go(mod, a, tag):
    out = TMP / f"h2_{tag}.txt"
    os.environ["FAKE_OUT"] = str(out)
    before = set((S3 / "runs").glob("vio_*"))
    t = time.time()
    r = mod.run(a)
    for d in set((S3 / "runs").glob("vio_*")) - before:      # the bench's temp config dir
        shutil.rmtree(d, ignore_errors=True)
    h = out.read_text().split()[0]
    keep = {k: r.get(k) for k in ("frames_fed", "focus_comp", "focus_d_ref", "frames_rescaled", "image_scale_min",
                                  "image_scale_max", "focus_fed_min", "focus_fed_max", "frames_dropped_lens_moving",
                                  "focus_table", "frames_focus_clamped", "table_map_shift_px_max")}
    print(json.dumps(dict(tag=tag, sha256=h[:16], secs=round(time.time() - t), **keep)), flush=True)
    return h



old = load("vb_old", sys.argv[1])
new = load("vb_new", S3 / "imu_learn/vio_bench_hs.py")
TAB = str(S3 / "config/lens/hs_1920x1080_focus_table_sweep.json")
o1 = dict(init_window=1.0, init_disp=15, start_off=1.0, imu_raw=True, cfg_over="num_opencv_threads=2")
for rec, kw in (("hs_20261021_215348", o1), ("hs_20261022_111836", {}), ("hs_20261022_111605", {})):
    if rec[-6:] not in sys.argv[2:]:
        continue
    t = rec[-6:]
    ho = go(old, ns(rec, **kw), f"{t}_old_off")
    hn = go(new, ns(rec, **kw), f"{t}_new_off")
    print(rec, "OFF identical:", ho == hn, flush=True)
    if t == "215348":
        continue
    lo = go(old, ns(rec, focus_comp=True, focus_k=0.008, drop_lens_moving=True, **kw), f"{t}_old_lin")
    ln = go(new, ns(rec, focus_comp=True, focus_k=0.008, drop_lens_moving=True, **kw), f"{t}_new_lin")
    print(rec, "linear ON(+drop) identical:", lo == ln, flush=True)
    ht = go(new, ns(rec, focus_comp=True, focus_table=TAB, drop_lens_moving=True, **kw), f"{t}_new_table")
    print(rec, "table ON == OFF:", ht == hn, flush=True)
