# Group phone runs (phone_an.json of several out_* dirs) into config x variant and print a compact table.
#   python3 phone_sum.py out_L1_ident out_L2_timing [...]
# Variant from the tag: I_none* / T_none* / N_none* = A (no options), *_exact* = B (all_exact), *_near* = W
# (all_exact + ekf_gemm + qr_compress + ekf_chol + par_cov), I_ref = 0.6.17 APK binary. Config: N_* = c4n400s100c20,
# others c4. Each run's values are per-run percentiles over its tracking frames; the table shows every run (start cpu9
# clock cap and SKIN = thermal state) and the median over runs.
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

rows = []
for od in sys.argv[1:]:
    p = Path(od) / "phone_an.json"
    if p.exists():
        rows += json.load(open(p))


def group(tag):
    cfg = "c4n400s100c20" if tag.startswith("N_") else "c4"
    if "ref" in tag:
        v = "ref_0617"
    elif "none" in tag:
        v = "A_none"
    elif "exact" in tag:
        v = "B_all_exact"
    elif "near" in tag:
        v = "W_near"
    else:
        v = tag
    return cfg, v


g = defaultdict(list)
for r in rows:
    g[group(r["tag"])].append(r)
print("| config | variant | runs (tag: proc p50/p95, >33 %, cpu9 cap MHz, SKIN start) | median proc p50 | median p95 | median >33 % |")
print("|---|---|---|---|---|---|")
for (cfg, v), rs in sorted(g.items()):
    det = "; ".join(f"{r['tag']}: {r['proc_p50']}/{r['proc_p95']}, {r['over33']}%, {int(r['cap9']) // 1000 if r['cap9'] else '?'}, "
                    f"{r['skin0']}" for r in rs)
    med = lambda k: round(float(np.median([x[k] for x in rs if x[k] is not None])), 2)
    print(f"| {cfg} | {v} | {det} | {med('proc_p50')} | {med('proc_p95')} | {med('over33')} |")
