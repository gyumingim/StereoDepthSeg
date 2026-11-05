#!/usr/bin/env python3
"""scalehunt: livepose lp_replay.py unchanged + two data perturbations (README 3: G_s, A_s).

  python sh_replay.py [--gyro-scale S] [--acc-scale S] <lp_replay.py args...>
The IMU reader used by lp_replay (imu_gt.read_imu) is wrapped to multiply gyro / accelerometer values. Everything else
(config copy, bridge args, frame feeding, scoring) is lp_replay.main() as committed.
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "livepose_20261030"))
import lp_replay as L  # noqa: E402

gs = as_ = 1.0
argv = sys.argv[1:]
cache = None
if "--cache" in argv:
    i = argv.index("--cache"); cache = Path(argv[i + 1]); del argv[i:i + 2]
for flag in ("--gyro-scale", "--acc-scale"):
    if flag in argv:
        i = argv.index(flag); v = float(argv[i + 1]); del argv[i:i + 2]
        if flag == "--gyro-scale":
            gs = v
        else:
            as_ = v
_read = L.read_imu


def read_imu(*a, **k):
    acc, gyr = _read(*a, **k)
    acc = acc.copy(); gyr = gyr.copy()
    acc[:, 1:4] *= as_
    gyr[:, 1:4] *= gs
    return acc, gyr


L.read_imu = read_imu


class _Cap:
    """stands in for cv2.VideoCapture: serves the gray frames sh_cache.py decoded (same decoder, same selection)"""
    def __init__(self, path, *a):
        d = cache / Path(path).parent.name
        self.idx = {int(f): k for k, f in enumerate(np.load(d / "idx.npy"))}
        self.gray = np.load(d / "gray.npy", mmap_mode="r")
        self.fi = -1

    def grab(self):
        self.fi += 1
        return True

    def retrieve(self):
        if self.fi not in self.idx:
            raise SystemExit(f"cache miss frame {self.fi}")
        return True, ("cached", self.fi)


class _CV:
    def __getattr__(self, k):
        return getattr(_cv2, k)

    VideoCapture = _Cap

    @staticmethod
    def cvtColor(f, code):
        if isinstance(f, tuple):
            return np.asarray(_Cap_inst[0].gray[_Cap_inst[0].idx[f[1]]])
        return _cv2.cvtColor(f, code)


if cache is not None:
    import cv2 as _cv2
    import numpy as np
    _Cap_inst = []
    _orig = _Cap.__init__

    def _init(self, *a, **k):
        _orig(self, *a, **k); _Cap_inst[:] = [self]
    _Cap.__init__ = _init
    L.cv2 = _CV()
    print(f"scalehunt frame cache: {cache}", file=sys.stderr)
sys.argv = [sys.argv[0]] + argv
if gs != 1.0 or as_ != 1.0:
    print(f"scalehunt data scale: gyro x{gs} acc x{as_}", file=sys.stderr)
L.main()
