"""integ2 0.6.28 identity on std60 (vio_bench feed, --pose-precision 17): O60 = 0.6.27 build --profile rec60,
N60 = 0.6.28 --profile rec60 --loop 0, R60 = 0.6.28 --profile rec60 (loop on, --loop-sync 1), A60 = same, --loop-sync 0 (live).
Compares every pose line, all keys except timing keys and "map"."""
import gzip, json, os, sys
B = os.environ.get("OVLOOP_OUT", os.path.expanduser("~/scanner/tmp_integ2l/ate_id"))
TIMING = {"ms", "remap_ms", "t_out", "age_ms", "lat_ms", "feed_ms", "map"}
def load(c, cell):
    f = f"{B}/{c}/{cell}_{c}.poses.jsonl.gz"
    return [json.loads(x) for x in gzip.open(f, "rt")] if os.path.exists(f) else None
for rec in ("s165506", "s173432"):
    for o in (0, 1, 2):
        cell = f"{rec}_o{o}"
        ref = load("O60", cell)
        for c in ("N60", "R60", "A60"):
            x = load(c, cell)
            if ref is None or x is None: print(cell, c, "missing"); continue
            nd = sum(1 for a, b in zip(ref, x) if {k: v for k, v in a.items() if k not in TIMING} != {k: v for k, v in b.items() if k not in TIMING})
            print(f"{cell} O60 vs {c}: lines {len(ref)}/{len(x)} differing {nd} map lines {sum('map' in b for b in x)} keys_extra {sorted(set().union(*[set(b) for b in x]) - set().union(*[set(a) for a in ref]))}")
