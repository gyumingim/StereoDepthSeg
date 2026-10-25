# droplatefix analysis of paced runs: drops, forced, re-anchor, recovery, health, pred horizon / lost, VIOP flags, end-pose vs no-stall run
import json, re, sys, csv, math, os
O = sys.argv[1]
names = sys.argv[2:]
def load(n):
    P, R = [], []
    for l in open(f"{O}/{n}.jsonl"):
        if l.startswith('{"type":"pose"'):
            P.append(json.loads(l))
        elif l.startswith('{"type":"pred"'):
            R.append(json.loads(l))
    return P, R
ref_end = {}
def pos(j):
    T = j.get("T_world_cam0")
    if not T: return None
    T = T if len(T) == 16 else sum(T, [])
    return (T[3], T[7], T[11])
rows = []
for n in names:
    if not os.path.exists(f"{O}/{n}.jsonl"): continue
    P, R = load(n)
    err = open(f"{O}/{n}.err").read()
    fd = open(f"{O}/{n}.feeder").read().strip().splitlines()
    m = re.search(r"dropped (\d+) frames", err); drops = int(m.group(1)) if m else -1
    m1 = re.search(r"forced (\d+) \| re-anchors (\d+) \| (?:late-lost \d+ \| )?max late run (\d+)", err)
    forced, rean, mlr = (m1.groups() if m1 else ("-", "-", "-"))
    stl = re.search(r"pred thread stale \(--pred-stale-ms\): (\d+)", err)
    t0 = P[0]["t"] if P else 0
    after = [p for p in P if p["t"] >= 25.0]
    hs = {}
    for p in after:
        s = p.get("h", {}).get("state", "-"); hs[s] = hs.get(s, 0) + 1
    rl = [r for r in R if r["t"] >= 25.0]
    hmax = max((r["h"] for r in R), default=float("nan"))
    hmax_a = max((r["h"] for r in rl), default=float("nan"))
    lost = sum(1 for r in rl if r["status"] == "lost")
    last = R[-1] if R else None
    vlast = math.sqrt(sum(x * x for x in last["v"])) if last else float("nan")
    vi = f"{O}/{n}.viop.csv"
    vstr = ""
    if os.path.exists(vi):
        V = list(csv.DictReader(open(vi)))
        k = [c for c in (V[0].keys() if V else []) if c.lower() in ("status", "tracking", "track")]
        if V and k:
            st = k[0]
            vstr = f"viop rows {len(V)} not-tracking {sum(1 for v in V if v[st] in ('0', 'lost'))} last {V[-1][st]}/q{V[-1].get('quality')}"
    ends = pos(P[-1]) if P else None
    rows.append((n, len(P), len(after), drops, forced, rean, mlr, hs, len(R), round(hmax, 3), lost, stl.group(1) if stl else "-",
                 round(vlast, 2), vstr, P[-1]["t"] if P else None, ends, fd[-1] if fd else ""))
for r in rows:
    n, npose, naft, drops, forced, rean, mlr, hs, npred, hmax, lost, stale, vlast, vstr, tl, ends, fdl = r
    print(f"{n}: poses {npose} (t>=25: {naft}) | dropped {drops} forced {forced} re-anchors {rean} max-late-run {mlr} | health t>=25 {hs}")
    print(f"     pred {npred} max h {hmax} s, lost(t>=25) {lost}, stale-marked {stale}, last |v| {vlast} m/s | {vstr}")
    print(f"     last pose t {tl} p {tuple(round(x, 3) for x in ends) if ends else None}")
