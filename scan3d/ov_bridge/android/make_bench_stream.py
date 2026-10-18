#!/usr/bin/env python3
"""Phone VIO benchmark input: shrink a captured ov_bridge input stream (PROTOCOL.md records) to WxH and write a matching config.

  venv_ffs/bin/python scan3d/ov_bridge/android/make_bench_stream.py IN.bin CFG_DIR OUT_DIR [--size 640x480] [--max-cams N]
OUT_DIR gets stream.bin and estimator_config.yaml + kalibr_*.yaml (intrinsics and resolution scaled; everything else as is).
Capture IN.bin by putting a tee in front of the bridge (stdin copy) during a replay.
"""
import argparse
import re
import shutil
import struct
import sys
from pathlib import Path

import cv2
import numpy as np

MAGIC = 0x4F564252


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('inp'); ap.add_argument('cfg'); ap.add_argument('out')
    ap.add_argument('--size', default='640x480'); ap.add_argument('--max-cams', type=int, default=0)
    a = ap.parse_args()
    W, H = map(int, a.size.split('x'))
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    src_wh, n_imu, n_cam = None, 0, 0
    with open(a.inp, 'rb') as f, open(out / 'stream.bin', 'wb') as g:
        while True:
            hdr = f.read(12)
            if len(hdr) < 12:
                break
            magic, typ, n = struct.unpack('<III', hdr)
            if magic != MAGIC:
                sys.exit(f'bad magic at {f.tell() - 12}')
            pl = f.read(n)
            if typ == 2:
                if a.max_cams and n_cam >= a.max_cams:
                    break
                t_ns, w, h, nc = struct.unpack('<qIIB3x', pl[:20])
                src_wh = (w, h)
                imgs = [np.frombuffer(pl, np.uint8, w * h, 20 + i * w * h).reshape(h, w) for i in range(nc)]
                imgs = [cv2.resize(im, (W, H), interpolation=cv2.INTER_AREA) for im in imgs]
                pl = struct.pack('<qIIB3x', t_ns, W, H, nc) + b''.join(im.tobytes() for im in imgs)
                n_cam += 1
            elif typ == 1:
                n_imu += 1
            g.write(struct.pack('<III', MAGIC, typ, len(pl)) + pl)
    sx, sy = W / src_wh[0], H / src_wh[1]
    for p in Path(a.cfg).glob('*.yaml'):
        s = p.read_text(encoding='utf-8')
        if p.name == 'kalibr_imucam_chain.yaml':
            def fix_k(m):
                k = [float(v) for v in m.group(2).split(',')]
                k = [k[0] * sx, k[1] * sy, k[2] * sx, k[3] * sy]
                return m.group(1) + ', '.join(f'{v:.6f}' for v in k) + ']'
            s = re.sub(r'(intrinsics:\s*\[)([^\]]*)\]', fix_k, s)
            s = re.sub(r'resolution:\s*\[[^\]]*\]', f'resolution: [{W}, {H}]', s)
        (out / p.name).write_text(s, encoding='utf-8')
    print(f'{n_cam} cams, {n_imu} imu, {src_wh} -> {W}x{H}, scale {sx:.4f}/{sy:.4f} -> {out}')


if __name__ == '__main__':
    main()
