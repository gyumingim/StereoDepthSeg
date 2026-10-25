#!/usr/bin/env python3
"""Emulate the arlat 0.6.19 imu_prio send order on a recorded ov_bridge input stream (PC identity check).

  python3 reorder.py IN.bin OUT.bin --lead S [--chunk 65536] [--imu-per-chunk K] [--no-chunk]

Every image record (type 2) is moved behind all IMU records with t <= t_img + lead (IMU arrives earlier than the image,
as when IMU no longer waits behind image writes), then written as CHUNK (5) / CHUNK_END (6) pieces of --chunk bytes
with the following IMU records interleaved (one IMU after every K chunks) like the app's writer does. --no-chunk keeps
the image as one type-2 record. Prints counts and the order statistics.
"""
import argparse
import struct
import sys
from collections import deque

H = struct.Struct("<III")
MAGIC = 0x4F564252


def records(f):
    while True:
        h = f.read(12)
        if len(h) < 12:
            return
        m, t, n = H.unpack(h)
        assert m == MAGIC, "bad magic"
        yield t, h + f.read(n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp")
    ap.add_argument("out")
    ap.add_argument("--lead", type=float, required=True)
    ap.add_argument("--chunk", type=int, default=65536)
    ap.add_argument("--imu-per-chunk", type=int, default=4)
    ap.add_argument("--no-chunk", action="store_true")
    a = ap.parse_args()
    lead_ns = int(a.lead * 1e9)
    pend = deque()          # images waiting for the IMU stream to pass t_img + lead
    cur = deque()           # chunk records of the image being written
    n = dict(imu=0, img=0, chunks=0, imu_inside=0, other=0)
    first_img_t = None
    with open(a.inp, "rb") as fi, open(a.out, "wb") as fo:
        def start_image(rec):
            payload = rec[12:]
            if a.no_chunk:
                fo.write(rec)
                return
            parts = [payload[i:i + a.chunk] for i in range(0, len(payload), a.chunk)]
            for k, p in enumerate(parts):
                cur.append(H.pack(MAGIC, 6 if k == len(parts) - 1 else 5, len(p)) + p)
            n["chunks"] += len(parts)
            for _ in range(a.imu_per_chunk):
                if cur:
                    fo.write(cur.popleft())

        def finish_image():
            while cur:
                fo.write(cur.popleft())

        for t, rec in records(fi):
            if t == 1:
                ti = struct.unpack_from("<q", rec, 12)[0]
                while pend and ti > pend[0][0] + lead_ns:
                    finish_image()
                    start_image(pend.popleft()[1])
                fo.write(rec)
                n["imu"] += 1
                if cur:
                    n["imu_inside"] += 1
                    for _ in range(a.imu_per_chunk):
                        if cur:
                            fo.write(cur.popleft())
            elif t == 2:
                n["img"] += 1
                pend.append((struct.unpack_from("<q", rec, 12)[0], rec))
            else:
                finish_image()
                while pend:
                    start_image(pend.popleft()[1])
                    finish_image()
                fo.write(rec)
                n["other"] += 1
        finish_image()
        while pend:
            start_image(pend.popleft()[1])
            finish_image()
    print(f"{a.inp} -> {a.out}: lead {a.lead} s chunk {a.chunk} imu/chunk {a.imu_per_chunk} no_chunk {a.no_chunk} | {n}")


if __name__ == "__main__":
    main()
