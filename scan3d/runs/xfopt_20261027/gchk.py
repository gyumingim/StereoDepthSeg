#!/usr/bin/env python3
"""xfopt: bridge gates == simulator. (1) GCHK run (gates inside the bridge, new build 38934de7) vs sim.py on the X1t3 log of
the same build with the same gates; (2) X1t3 new build vs X1t3 old build (493de926): loop logs identical without ms."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim  # noqa: E402
import xfopt_run as XR  # noqa: E402

G = Path("/home/system/scanner/tmp_xfopt/gchk")
GATES = dict(pick="x", inl=40, ratio=0.3, cells=10, rp=2.0, yaw=3.0, ntri_frac=0.9, consist_n=2)
out = []
n1 = n2 = b1 = b2 = 0
for r, o in XR.cells("dev"):
    k = XR.key_of(r, o, "dev")
    if (G / "GCHK" / f"{k}_GCHK.json").exists() and (G / "X1t3" / f"{k}_X1t3.json").exists():
        a = sim.simulate(sim.Cell("X1t3", f"{k}_X1t3", out=G, gt=False), GATES)
        act = sim.Cell("GCHK", f"{k}_GCHK", out=G, gt=False).loops
        s1 = sorted((s, c, e["o"], e["ni"]) for s, c, e in a)
        s2 = sorted((x["session"], x["cur"], x["old"], x["n_inl"]) for x in act)
        n1 += 1
        b1 += s1 != s2
        out.append(f"{k} bridge gates vs sim: {'SAME' if s1 == s2 else 'DIFF'} {len(s2)} / {len(s1)}")
    fa, fb = G / "X1t3" / f"{k}_X1t3.loop.jsonl", XR.OUT / "X1t3" / f"{k}_X1t3.loop.jsonl"
    if fa.exists() and fb.exists() and (G / "X1t3" / f"{k}_X1t3.json").exists():
        def L(f):
            r = []
            for x in open(f):
                d = {kk: v for kk, v in json.loads(x).items() if kk != "ms"}
                if "xf" in d:
                    d["xf"] = d["xf"][:3]   # [3:] = server / match times
                r.append(d)
            return r
        n2 += 1
        same = L(fa) == L(fb)
        b2 += not same
        out.append(f"{k} X1t3 new vs old build loop log: {'IDENTICAL' if same else 'DIFFERENT'}")
out.append(f"RESULT gates {n1 - b1}/{n1} SAME, build {n2 - b2}/{n2} IDENTICAL")
(Path(__file__).resolve().parent / "ident").mkdir(exist_ok=True)
open(Path(__file__).resolve().parent / "ident" / "gchk.txt", "w").write("\n".join(out) + "\n")
print("\n".join(out))
