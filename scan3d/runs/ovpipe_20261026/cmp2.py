#!/usr/bin/env python3
"""Compare ov_bridge pose lines (like ovopt cmp.py) excluding timing fields and optionally more keys; also counts pred lines.
  python3 cmp2.py [--ex imu_held,...] REF.jsonl OTHER.jsonl [...]
"""
import hashlib
import json
import sys

args = sys.argv[1:]
EX = {"ms", "remap_ms", "wait_ms"}
if args and args[0] == "--ex":
    EX |= set(args[1].split(","))
    args = args[2:]


def load(p):
    P, h, npred = [], hashlib.sha256(), 0
    for line in open(p):
        if '"pred"' in line:
            npred += 1
            continue
        if '"pose"' not in line:
            continue
        j = json.loads(line)
        for k in EX:
            j.pop(k, None)
        P.append(j)
        h.update(json.dumps(j, sort_keys=True).encode())
    return P, h.hexdigest()[:16], npred


ref, ref_h, rp = load(args[0])
print(f"ref {args[0]}: poses {len(ref)} tracking {sum(r.get('status') == 'tracking' for r in ref)} hash {ref_h} pred {rp} (excluded {sorted(EX)})")
bad = 0
for p in args[1:]:
    P, h, npred = load(p)
    same = sum(1 for a, b in zip(ref, P) if a == b)
    first = next((i for i, (a, b) in enumerate(zip(ref, P)) if a != b), None)
    ok = len(P) == len(ref) and same == len(ref)
    bad += 0 if ok else 1
    diffk = sorted(k for k in set(ref[first]) | set(P[first]) if ref[first].get(k) != P[first].get(k)) if first is not None else []
    print(f"{p}: poses {len(P)} identical {same}/{len(ref)} hash {h} pred {npred} first_diff {first} {diffk[:6]} "
          f"{'IDENTICAL' if ok else 'DIFFERENT'}")
sys.exit(1 if bad else 0)
