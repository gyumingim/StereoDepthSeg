#!/usr/bin/env python3
"""vioscale: livepose lp_replay.py unchanged + IMU input perturbations (same idea as scalehunt sh_replay.py).

  python vs_replay.py [--acc-scale S] [--gyro-scale S] [--gyro-cal] [--imu-decim N] <lp_replay.py args...>
  --acc-scale / --gyro-scale  multiply the values (positive control / measured correction)
  --gyro-cal                  gyro = Android-calibrated type 4 (bias removed by the OS), accel stays raw type 35
  --imu-decim N               keep every Nth gyro row (acc is interpolated to the kept gyro times by lp_replay)
Everything else (config copy, bridge args, frame feeding, scoring) is lp_replay.main() as committed.
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "livepose_20261030"))
import lp_replay as L  # noqa: E402

argv = sys.argv[1:]
opt = {"--acc-scale": 1.0, "--gyro-scale": 1.0, "--imu-decim": 1}
gcal = False
for flag in list(opt):
    if flag in argv:
        i = argv.index(flag); opt[flag] = type(opt[flag])(argv[i + 1]); del argv[i:i + 2]
if "--gyro-cal" in argv:
    argv.remove("--gyro-cal"); gcal = True
_read = L.read_imu


def read_imu(path, raw=False):
    acc, gyr = _read(path, raw=raw)
    if gcal:
        _, gyr = _read(path, raw=False)       # type 4 (acc type 1 is not recorded -> keep raw acc)
    acc = acc.copy(); gyr = gyr.copy()
    acc[:, 1:4] *= opt["--acc-scale"]
    gyr[:, 1:4] *= opt["--gyro-scale"]
    if opt["--imu-decim"] > 1:
        gyr = gyr[::opt["--imu-decim"]]
    print(f"vs_replay: acc x{opt['--acc-scale']} gyro x{opt['--gyro-scale']} gyro_cal {gcal} decim {opt['--imu-decim']} "
          f"-> acc {len(acc)} gyr {len(gyr)}", file=sys.stderr)
    return acc, gyr


L.read_imu = read_imu
sys.argv = [sys.argv[0]] + argv
L.main()
