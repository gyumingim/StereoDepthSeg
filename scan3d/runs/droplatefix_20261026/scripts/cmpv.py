# verifier comparator: pose lines only (pred lines are asynchronous), timing / output-only keys removed
import json, sys, hashlib
EX = {"ms", "remap_ms", "wait_ms", "klt"}
def load(p):
    P = []
    for l in open(p):
        if '"type":"pose"' not in l: continue
        j = json.loads(l)
        for k in EX: j.pop(k, None)
        P.append(j)
    return P
ref = load(sys.argv[1])
print(f"ref {sys.argv[1].split('/')[-1]}: poses {len(ref)} tracking {sum(1 for r in ref if r.get('status')=='tracking')}")
for p in sys.argv[2:]:
    P = load(p)
    same = sum(1 for a, b in zip(ref, P) if a == b)
    first = next((i for i, (a, b) in enumerate(zip(ref, P)) if a != b), None)
    keys = sorted(k for k in set(ref[first]) | set(P[first]) if ref[first].get(k) != P[first].get(k)) if first is not None else []
    print(f"  {p.split('/')[-1]}: poses {len(P)} identical {same}/{len(ref)} first_diff {first} {keys[:5]} -> {'IDENTICAL' if (same == len(ref) == len(P)) else 'DIFFERENT'}")
