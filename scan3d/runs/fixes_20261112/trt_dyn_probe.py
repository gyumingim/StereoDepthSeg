"""unit 13b: probe the dynamic TRT engine at one input size in a fresh process (CUDA error 720 poisons the context).
python trt_dyn_probe.py ENGDIR H W -> prints OK + disparity stats + ms, or the error; also PyTorch fp16 on the same random-ish image."""
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))           # scan3d/
sys.path.insert(0, os.environ.get("FFS_ROOT", "/home/system/scanner/StereoDepthSeg"))
import ffs_trt  # noqa: E402

eng, H, W = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
rng = np.random.default_rng(0)
base = (rng.random((H // 8 + 1, W // 8 + 1, 3)) * 255).astype(np.uint8)
import cv2  # noqa: E402
L = cv2.resize(base, (W, H), interpolation=cv2.INTER_CUBIC)
R = np.roll(L, -12, axis=1)                                             # 12 px disparity everywhere
import torch  # noqa: E402
t = ffs_trt.TrtFFS(eng)
try:
    for k in range(3):
        t0 = time.perf_counter(); d = t.infer(None, L, R); torch.cuda.synchronize(); ms = 1000 * (time.perf_counter() - t0)
    print(f"TRT {H}x{W}: OK {ms:.0f} ms, disp median {np.median(d):.2f} (expect ~12), GPU max alloc {torch.cuda.max_memory_allocated() / 2**30:.2f} GB")
except Exception as e:
    print(f"TRT {H}x{W}: FAIL {e!r}"[:300])
