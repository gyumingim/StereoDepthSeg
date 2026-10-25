#!/usr/bin/env python3
"""Near-exact pose comparison (ovupd 2026-10-26): first differing pose, position difference (cm) between two ov_bridge
pose streams matched by timestamp (tracking poses only).  python3 near.py REF.jsonl OTHER.jsonl"""
import json, sys
import numpy as np

def load(p):
    d = {}
    for line in open(p):
        if '"pose"' not in line or '"p":' not in line:
            continue
        j = json.loads(line)
        d[round(float(j["t"]), 6)] = np.array(j["p"][:3], float)
    return d

a, b = load(sys.argv[1]), load(sys.argv[2])
ts = sorted(set(a) & set(b))
name = sys.argv[2].split("/")[-1]
if not ts:
    print(f"{name}: no common poses (ref {len(a)}, other {len(b)})"); sys.exit()
diff = np.array([np.linalg.norm(a[t] - b[t]) * 100 for t in ts])
first = next((i for i, t in enumerate(ts) if a[t].tolist() != b[t].tolist()), None)
print(f"{name}: common {len(ts)} (ref {len(a)}, other {len(b)}), identical {int((diff == 0).sum())}, first diff idx {first}, "
      f"|dp| cm median {np.median(diff):.4f} p95 {np.percentile(diff, 95):.4f} max {diff.max():.4f} last {diff[-1]:.4f}; "
      f"path ref {sum(np.linalg.norm(a[ts[i+1]]-a[ts[i]]) for i in range(len(ts)-1))*100:.1f} cm")
