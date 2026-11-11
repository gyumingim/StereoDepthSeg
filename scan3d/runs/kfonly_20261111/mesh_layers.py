"""kfonly 11-11 diagnostic (not in the pre-registered rule): board-mesh layer structure from mesh.ply.
layers(mesh_dir) -> board vertices, fraction > 0.5 cm from the median sheet ("ghost layer"), RMS of the near sheet (cm)."""
import json
import numpy as np


def layers(f):
    m = json.load(open(f"{f}/summary.json")); BW, BH = [x / 1000 for x in m["board_mm"]]
    b = open(f"{f}/mesh.ply", "rb").read(); e = b.index(b"end_header\n") + 11; h = b[:e].decode()
    L = h.split("\n"); i = [k for k, l in enumerate(L) if l.startswith("element vertex")][0]
    nv = int(L[i].split()[-1]); vp = []; i += 1
    while L[i].startswith("property"):
        vp.append(L[i].split()); i += 1
    tm = {"float": "<f4", "double": "<f8", "uchar": "u1", "int": "<i4"}
    V = np.frombuffer(b[e:e + nv * np.dtype([(p[2], tm[p[1]]) for p in vp]).itemsize], np.dtype([(p[2], tm[p[1]]) for p in vp]))
    x, y, z = V["x"], V["y"], V["z"]
    mb = (x > 0.02) & (x < BW - 0.02) & (y > 0.02) & (y < BH - 0.02) & (np.abs(z) < 0.10)
    zz = z[mb] * 100; med = np.median(zz); far = np.abs(zz - med) > 0.5
    return dict(board_v=int(mb.sum()), ghost_frac=float(far.mean()), near_rms_cm=float(np.sqrt(np.mean((zz[~far] - med) ** 2))))
