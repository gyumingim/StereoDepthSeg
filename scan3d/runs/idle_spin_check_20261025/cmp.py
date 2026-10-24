# compare pose lines of bridge runs: all fields except timing (ms, remap_ms, wait_ms)
import json, sys
TIMING = {"ms", "remap_ms", "wait_ms"}


def load(p):
    P = []
    for l in open(p):
        if '"pose"' not in l:
            continue
        j = json.loads(l)
        for k in TIMING:
            j.pop(k, None)
        P.append(j)
    return P


ref_p = sys.argv[1]
ref = load(ref_p)
keys = sorted(set().union(*[set(r.keys()) for r in ref]))
print("ref", ref_p, "poses", len(ref), "tracking", sum(r.get("status") == "tracking" for r in ref))
print("compared keys", keys)
for p in sys.argv[2:]:
    P = load(p)
    same = sum(1 for a, b in zip(ref, P) if a == b)
    st = sum(1 for a, b in zip(ref, P) if all(a.get(k) == b.get(k) for k in ("p", "q", "v", "bg", "ba", "T_world_cam0")))
    print(f"{p}: poses {len(P)} identical_lines {same}/{len(ref)} identical_state(p,q,v,bg,ba,T) {st}/{len(ref)}")
