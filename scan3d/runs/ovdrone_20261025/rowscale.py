import sys, json, os
from pathlib import Path
MAIN = Path("/home/system/scanner/StereoDepthSeg/scan3d")
sys.path.insert(0, str(MAIN / "imu_learn")); sys.path.insert(0, str(MAIN)); os.chdir(MAIN)
import vio_bench_hs as vb
sys.path.insert(0, str(MAIN / "imu_learn"))
from phone60_run import RAW, OLD
for rec in RAW + OLD:
    R = Path("recordings") / rec
    meta = json.load(open(R / "meta.json"))
    K, _ = vb.lens_for(R, meta)
    for size in ("960x540", "640x360"):
        _, _, Knew, (Wd, H) = vb.undistort_maps(R, meta, size)
        print(rec, size, round(float(K[1, 1] * H / (Knew[1, 1] * int(meta["height"]))), 6), round(float(K[1,1]),2), round(float(Knew[1,1]),2))
