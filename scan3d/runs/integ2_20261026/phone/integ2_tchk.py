import gzip,json,re,os,sys,csv
O=sys.argv[1]
for t in sys.argv[2:]:
    d=f"{O}/{t}"
    P=[json.loads(l) for l in gzip.open(d+"/po.jsonl.gz","rt") if l.startswith('{"type":"pose"')]
    trk=[p for p in P if p.get("status")=="tracking"]
    err=open(d+"/pe.txt").read()
    m=re.search(r"dropped (\d+) frames",err)
    resets=len(re.findall(r"reset|re-init|reinit",err,re.I))
    T=list(csv.DictReader(open(d+"/tl.csv")))
    init1=[r for r in T if r.get("init")=="1"]
    print(t, "poses",len(P),"tracking",len(trk),"first trk t",trk[0]["t"] if trk else None,"dropped",m.group(1) if m else None,"tl rows",len(T),"init1",len(init1),"reset-ish lines",resets)
    print("   tl cols", list(T[0].keys())[:14])
    print("   ", [l for l in err.splitlines() if 'drop-late' in l and 'skipped' not in l][:3])
