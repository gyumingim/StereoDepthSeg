#!/usr/bin/env python3
"""ovbaro identity helper: copy an ov_bridge record stream to stdout and insert a type-7 BARO record (<qd t_ns, hPa)
every 80 ms of IMU time (constant 1013.25 hPa + 0.01 hPa sine), right after the IMU record that crosses the grid.
    python3 inject_baro.py IN.bin | ov_bridge ... """
import math
import struct
import sys

HDR = struct.Struct("<III")
MAGIC = 0x4F564252
src = open(sys.argv[1], "rb")
out = sys.stdout.buffer
nxt, n = None, 0
while True:
    h = src.read(12)
    if len(h) < 12:
        break
    m, t, ln = HDR.unpack(h)
    body = src.read(ln)
    out.write(h)
    out.write(body)
    if t == 1 and ln == 56:
        ti = struct.unpack_from("<q", body)[0]
        if nxt is None:
            nxt = ti
        if ti >= nxt:
            out.write(HDR.pack(MAGIC, 7, 16) + struct.pack("<qd", ti, 1013.25 + 0.01 * math.sin(n * 0.1)))
            n += 1
            nxt += 80_000_000
print(f"inject_baro: {n} baro records", file=sys.stderr)
