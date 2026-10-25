# Compare ov_bridge pose lines: all fields except timing (ms, remap_ms, wait_ms). Prints identical-line counts and a
# hash of the non-timing content per file (same hash = identical printed poses, counters and calibration at the
# printed precision: 9 significant digits by default, 17 = round-trip exact doubles with ov_bridge --pose-precision 17).
#   python3 cmp.py REF.jsonl OTHER.jsonl [...]
import hashlib
import json
import sys

TIMING = {"ms", "remap_ms", "wait_ms"}


def load(p):
    P, h = [], hashlib.sha256()
    for line in open(p):
        if '"pose"' not in line:
            continue
        j = json.loads(line)
        for k in TIMING:
            j.pop(k, None)
        P.append(j)
        h.update(json.dumps(j, sort_keys=True).encode())
    return P, h.hexdigest()[:16]


ref_p = sys.argv[1]
ref, ref_h = load(ref_p)
print(f"ref {ref_p}: poses {len(ref)} tracking {sum(r.get('status') == 'tracking' for r in ref)} hash {ref_h}")
bad = 0
for p in sys.argv[2:]:
    P, h = load(p)
    same = sum(1 for a, b in zip(ref, P) if a == b)
    st = sum(1 for a, b in zip(ref, P) if all(a.get(k) == b.get(k) for k in ("p", "q", "v", "bg", "ba", "T_world_cam0")))
    first = next((i for i, (a, b) in enumerate(zip(ref, P)) if a != b), None)
    ok = len(P) == len(ref) and same == len(ref)
    bad += 0 if ok else 1
    print(f"{p}: poses {len(P)} identical_lines {same}/{len(ref)} identical_state {st}/{len(ref)} hash {h}"
          f" first_diff {first} {'IDENTICAL' if ok else 'DIFFERENT'}")
sys.exit(1 if bad else 0)
