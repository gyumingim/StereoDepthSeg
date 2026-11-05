#!/usr/bin/env python3
"""identity gate (README 3.2): pose streams equal except timing fields.  python sh_ident.py A.poses.jsonl.gz B.poses.jsonl.gz"""
import gzip, json, sys
DROP = {"ms", "wall_ms", "t_wall", "wall", "proc_ms", "lat_ms", "remap_ms"}
def rd(p):
    out = []
    for l in gzip.open(p, "rt"):
        m = json.loads(l)
        out.append({k: v for k, v in m.items() if k not in DROP and not k.endswith("_ms_wall")})
    return out
a, b = rd(sys.argv[1]), rd(sys.argv[2])
diff = [i for i, (x, y) in enumerate(zip(a, b)) if x != y]
keys = sorted(set(k for i in diff[:3] for k in a[i] if a[i].get(k) != b[i].get(k)))
print(dict(n_a=len(a), n_b=len(b), n_diff=len(diff), first_diff=diff[:3], keys=keys))
sys.exit(0 if len(a) == len(b) and not diff else 1)
