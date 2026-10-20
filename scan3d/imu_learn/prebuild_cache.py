#!/usr/bin/env python3
"""Build train_imu's prepared-recording cache for one recording (run one per recording in parallel before several
folds start, so the folds don't all rebuild the same caches).
  IMU_CACHE_DIR=runs/train_cache ../venv_ffs/bin/python imu_learn/prebuild_cache.py REC GT_DIR [--smooth]
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from train_imu import load_rec  # noqa: E402

r = load_rec(sys.argv[1], smooth="--smooth" in sys.argv, gt_dir=sys.argv[2])
print(sys.argv[1], sys.argv[2], "frames", r["n_gt"], "dropped", r["n_bad"])
