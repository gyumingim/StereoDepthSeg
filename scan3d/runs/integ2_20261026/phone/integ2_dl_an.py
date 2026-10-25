# integ2 phone drop-late analysis (adapted from runs/droplatefix_20261026/scripts/dlf_an.py; phone layout out_I_1/<tag>/)
import json, re, sys, csv, math, os, gzip
O = sys.argv[1]; names = sys.argv[2:]
def load(d):
    P, R = [], []
    f = f"{d}/po.jsonl.gz"
    op = gzip.open(f, "rt") if os.path.exists(f) else open(f"{d}/po.jsonl")
    for l in op:
        if l.startswith('{"type":"pose"'): P.append(json.loads(l))
        elif l.startswith('{"type":"pred"'): R.append(json.loads(l))
    return P, R
for n in names:
    d = f"{O}/{n}"
    if not os.path.isdir(d): print(n, "missing"); continue
    P, R = load(d)
    err = open(f"{d}/pe.txt").read()
    fd = open(f"{d}/feeder.txt").read().strip().splitlines()
    m = re.search(r"dropped (\d+) frames", err); drops = int(m.group(1)) if m else -1
    m1 = re.search(r"forced (\d+) \| re-anchors (\d+) \| (?:late-lost (\d+) \| )?max late run (\d+)", err)
    stl = re.search(r"pred thread stale \(--pred-stale-ms\): (\d+)", err)
    rean = re.findall(r"re-anchor[^\n]*", err)[:2]
    mode = re.search(r"drop-late [^\n]*mode[^\n]*", err)
    after = [p for p in P if p["t"] >= 25.0]
    trk_after = sum(1 for p in after if p.get("status") == "tracking")
    hs = {}
    for p in after:
        s = (p.get("h") or {}).get("state", "-"); hs[s] = hs.get(s, 0) + 1
    rl = [r for r in R if r["t"] >= 25.0]
    hmax = max((r["h"] for r in R), default=float("nan"))
    lost = sum(1 for r in rl if r["status"] == "lost")
    last = R[-1] if R else None
    vlast = math.sqrt(sum(x * x for x in last["v"])) if last and "v" in last else float("nan")
    vstr = ""
    vi = f"{d}/viop.csv"
    if os.path.exists(vi):
        V = list(csv.DictReader(open(vi)))
        cols = list(V[0].keys()) if V else []
        vstr = f"viop rows {len(V)} cols {cols[:12]}"
        for c in cols:
            if c.lower() in ("state", "status", "tracking", "hb_state"):
                vals = {}
                for v in V: vals[v[c]] = vals.get(v[c], 0) + 1
                vstr += f" | {c} counts {dict(sorted(vals.items())[:8])}"
    print(f"{n}: {mode.group(0) if mode else '?'}")
    print(f"   poses {len(P)} (t>=25: {len(after)}, tracking {trk_after}) | dropped {drops} | forced/re-anchors/late-lost/max-late-run {m1.groups() if m1 else '-'} | health t>=25 {hs}")
    print(f"   re-anchor lines: {rean}")
    print(f"   pred {len(R)} max h {round(hmax,3)} s, lost(t>=25) {lost}, stale-marked {stl.group(1) if stl else '-'}, last |v| {round(vlast,2)} m/s")
    print(f"   {vstr}")
    print(f"   last pose t {P[-1]['t'] if P else None} | feeder: {fd[-1] if fd else ''}")
