#!/usr/bin/env python3
"""ovpipe chain-break test stream (2026-10-26): copy an ov_bridge record stream to stdout and inject
  - after every Nth image record (type 2): a copy of the PREVIOUS image record (older timestamp -> the bridge drops it
    deterministically when the current frame is still queued, else feeds it) and
  - one RESET record (type 3) after image record R (0 = none).
  - MODE (4th argument, default dup): dup = the copy as is; swap = the copy with width/height swapped in its header
    (same bytes, other size -> the bridge rejects it: size differs from the first frame -> pipe drop path)
Usage: inject.py IN.bin N R [dup|swap] > out   (stdout streamed; nothing stored)
"""
import struct
import sys

MAGIC = 0x4F564252
src, every, reset_at = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
mode = sys.argv[4] if len(sys.argv) > 4 else 'dup'
out = sys.stdout.buffer
n_img = 0
prev = None
with open(src, 'rb') as f:
    while True:
        h = f.read(12)
        if len(h) < 12:
            break
        magic, typ, ln = struct.unpack('<III', h)
        assert magic == MAGIC, magic
        pl = f.read(ln)
        out.write(h)
        out.write(pl)
        if typ == 2:
            n_img += 1
            if every > 0 and n_img % every == 0 and prev is not None:
                out.write(prev[0])
                if mode == 'swap':
                    pl2 = bytearray(prev[1])
                    pl2[8:12], pl2[12:16] = prev[1][12:16], prev[1][8:12]
                    out.write(bytes(pl2))
                else:
                    out.write(prev[1])
            if reset_at > 0 and n_img == reset_at:
                out.write(struct.pack('<III', MAGIC, 3, 0))
            prev = (h, pl)
out.flush()
sys.stderr.write(f'inject: images {n_img} every {every} reset_at {reset_at}\n')
