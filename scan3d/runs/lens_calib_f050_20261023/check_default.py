"""Check lens default resolution after registering std60 0.5 dpt (2026-10-24)."""
import json, sys
from pathlib import Path
S3D = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(S3D)); sys.path.insert(0, str(S3D / "imu_learn"))
import lens_calib
import vio_bench_hs
for rec in ["hs_20261022_162218", "hs_20261022_171455", "hs_20261024_121536_std60", "hs_20261023_201830_std60", "hs_20261022_212058_std60"]:
    r = S3D / "recordings" / rec
    if not (r / "meta.json").exists():
        print(rec, "missing"); continue
    m = json.load(open(r / "meta.json"))
    p = lens_calib.shared_lens_path(m["width"], m["height"], m.get("focus_dpt_request"), m.get("session"))
    own = (r / "imu_gt" / "lens.json").exists()
    print(rec, "session", m.get("session"), "focus", m.get("focus_dpt_request"), "->", p.name, "exists", p.exists(),
          "| vio_bench_hs.lens_path ->", vio_bench_hs.lens_path(r, m).name, "(own imu_gt/lens.json)" if own else "")
