#!/usr/bin/env python3
"""npuopt 10-28 evening (11th pre-reg): official XFeat torch post on official torch raw (13 live-path frames, 960x544, y>=540 dropped)
vs C++ xfpost_soft.h on the softmax-in-model output. Reuses off()/cmp() of score960.py.
venv_ffs:  python score960soft.py OUT.json NAME=KPFILE [NAME=KPFILE ...]   (golden mode: --gold NAME SIMKP PCKP)"""
import json, sys, time, os
import numpy as np
sys.argv_saved = list(sys.argv)
import importlib.util
here = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(here, "score960.py")).read().split("ref = np.fromfile")[0]
g = {}; exec(compile(src, "score960_head", "exec"), g)
off, cmp = g["off"], g["cmp"]
T = "/home/system/scanner/tmp_npu/xf960"


def readkp(fn, n_frames):
    bb = open(fn, "rb").read(); o = 0; out = []
    for _ in range(n_frames):
        n = int(np.frombuffer(bb, np.int32, 1, o)[0]); o += 4
        xy = np.frombuffer(bb, np.int32, 2 * n, o).reshape(n, 2).astype(np.float32); o += 8 * n
        de = np.frombuffer(bb, np.float32, 64 * n, o).reshape(n, 64); o += 256 * n
        out.append((xy, de))
    return out


res = {"made": time.strftime("%F %T"), "ref": "official XFeatModel torch raw (ref960_raw130_fp32.bin) + official torch post, top-1024, y<540"}
args = sys.argv[2:]
if args and args[0] == "--gold":
    for a in args[1:]:
        nm, sim, pc = a.split(","); s, p = readkp(sim, 1)[0], readkp(pc, 1)[0]
        q = cmp(p[0], p[1], s[0], s[1])
        res[nm] = {"overlap": q[0], "cos_med": q[1], "n_pc": len(p[0]), "n_sim": len(s[0]), "input": "Lab random golden (not a real image)", "post": "C++ post_soft both"}
else:
    ref = np.fromfile(f"{T}/ref960_raw130_fp32.bin", np.float32).reshape(-1, 130, 68, 120)
    R = [off(ref[i], 540) for i in range(len(ref))]
    for a in args:
        nm, fn = a.split("=", 1); K = readkp(fn, len(ref)); ov, cs, nn = [], [], []
        for (ra, rd), (xy, de) in zip(R, K):
            q = cmp(ra, rd, xy, de); ov.append(q[0]); cs.append(q[1]); nn.append((len(ra), len(xy)))
        res[nm] = {"file": fn, "overlap_med": float(np.median(ov)), "overlap_min": float(min(ov)), "cos_med": float(np.median(cs)),
                   "cos_min_frame": float(min(cs)), "n_ref_cpp": nn,
                   "pass_rule": bool(np.median(ov) >= 0.99 and np.median(cs) >= 0.999)}
print(json.dumps({k: (v if not isinstance(v, dict) else {kk: vv for kk, vv in v.items() if kk != "n_ref_cpp"}) for k, v in res.items()}, indent=1))
json.dump(res, open(sys.argv[1], "w"), indent=1)
