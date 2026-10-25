#!/usr/bin/env python3
"""Score a cell subset with acc_run.score (same scorer): python3 score_sub.py o0 L0,F10,F20 [--ref L0]
subset 'o0' = start offset 0 of the 15 recordings (screening), 'all' = every cell."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import acc_run as A  # noqa: E402

sub, conds = sys.argv[1], sys.argv[2].split(",")
ref = sys.argv[sys.argv.index("--ref") + 1] if "--ref" in sys.argv else "L0"
if sub != "all":
    off = int(sub[1:])
    _c = A.cells
    A.cells = lambda: ([(r, o) for r, o in _c()[0] if o == off], _c()[1], _c()[2])
orig_out = A.OUT
A.score(conds, ref)
p = A.OUT / f"score_{'_'.join(conds)}.json"
if sub != "all" and p.exists():
    p.rename(A.OUT / f"score_{sub}_{'_'.join(conds)}.json")
    print("renamed ->", A.OUT / f"score_{sub}_{'_'.join(conds)}.json")
