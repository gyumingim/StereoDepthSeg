#!/usr/bin/env python3
"""Files for live OpenVINS on the phone (arlat app, mode "ov"): config for raw lens-2 640x480 frames + undistort map.

  venv_ffs/bin/python scan3d/ov_bridge/android/make_phone_ov.py [OUT_DIR] [--fps 30] [--size 640x480]
OUT_DIR (default ~/scanner/android/phone_ov) gets estimator_config.yaml, kalibr_*.yaml (make_ov_config --mono, lens0u)
and lens0u_remap.bin (int32 w, h + float32 mapx + mapy, calib_scale.vio_lens0_maps) for ov_bridge --remap.
Output file paths inside the config point at PHONE_DIR so OpenVINS never writes outside the app folder.
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

SCAN3D = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCAN3D))
sys.path.insert(0, str(SCAN3D.parent))   # lib_rectify 등 저장소 최상위 모듈
import calib_scale  # noqa: E402

PHONE_DIR = "/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default=str(Path.home() / "scanner/android/phone_ov"))
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--size", default="640x480")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(SCAN3D / "make_ov_config.py"), "--mono", "--fps", str(a.fps),
                    "--size", a.size, "--out", str(out)], check=True)
    cfg = out / "estimator_config.yaml"
    s = cfg.read_text(encoding="utf-8")
    s = re.sub(r'(_filepath[a-z_]*|filepath_[a-z]+):\s*"[^"]*/([^"/]+)"', lambda m: f'{m.group(1)}: "{PHONE_DIR}/{m.group(2)}"', s)
    cfg.write_text(s, encoding="utf-8")
    g = calib_scale.load_geometry(calib_scale.default_calib("2+5"), ["2", "5"], calib_scale.parse_size(a.size))
    v = g["vio"]
    mx, my = (np.ascontiguousarray(m, np.float32) for m in v["map"])
    W, H = v["size"]
    with open(out / "lens0u_remap.bin", "wb") as f:
        f.write(np.array([W, H], np.int32).tobytes())
        f.write(mx.tobytes())
        f.write(my.tobytes())
    print(f"K lens0u {np.round(v['K'][[0, 0, 1], [0, 2, 2]], 3)} size {W}x{H} -> {out}")
    print(re.findall(r"intrinsics:.*", (out / "kalibr_imucam_chain.yaml").read_text(encoding="utf-8")))


if __name__ == "__main__":
    main()
