# safety check: prediction lines with horizon h > 0.5 s that still say "tracking" (should be 0 with drop-late mode 1 / pred-stale 500)
import gzip, json, sys, csv, os
O = sys.argv[1]
for t in sys.argv[2:]:
    n = big = big_trk = 0; hmax_trk = 0.0
    for l in gzip.open(f"{O}/{t}/po.jsonl.gz", "rt"):
        if not l.startswith('{"type":"pred"'): continue
        m = json.loads(l); n += 1
        if m["h"] > 0.5:
            big += 1
            if m.get("status") == "tracking": big_trk += 1
        if m.get("status") == "tracking": hmax_trk = max(hmax_trk, m["h"])
    v = f"{O}/{t}/viop.csv"; vs = ""
    if os.path.exists(v):
        V = list(csv.DictReader(open(v)))
        vs = f" | viop rows {len(V)}, age_ms>500 & status 1: {sum(1 for r in V if float(r['age_ms']) > 500 and r['status'] == '1')}, max age_ms {max(float(r['age_ms']) for r in V):.0f}"
    print(f"{t}: pred {n}, h>0.5 s {big}, of those 'tracking' {big_trk}, max h among tracking {hmax_trk:.3f} s{vs}")
