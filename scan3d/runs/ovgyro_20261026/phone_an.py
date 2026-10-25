# ovgyro phone replay summary: per run feed ms p50/p95/max, track (S_TRACK) ms p50/p95, frames > 33 ms, late drops, SKIN, battery
import csv,json,glob,os,sys,re
import numpy as np
root=sys.argv[1]
rows=[]
for d in sorted(glob.glob(os.path.join(root,'*/'))):
    tag=os.path.basename(d.rstrip('/'))
    pf=os.path.join(d,'prof.csv')
    if not os.path.exists(pf): continue
    R=[r for r in csv.reader(open(pf)) if r and r[0][0].isdigit()]
    ms=np.array([float(r[2]) for r in R]); tr=np.array([float(r[6]) for r in R]); init=np.array([int(r[3]) for r in R])
    run=open(os.path.join(d,'run.txt')).read() if os.path.exists(os.path.join(d,'run.txt')) else ''
    g=lambda k: (re.search(k+r'=(\S+)',run) or [None,None])[1]
    bat=open(os.path.join(d,'battery_start.txt')).read() if os.path.exists(os.path.join(d,'battery_start.txt')) else ''
    ac='AC' if re.search(r'AC powered: true',bat) else ('USB' if re.search(r'USB powered: true',bat) else 'none')
    po=[]; bad=0
    for l in (open(os.path.join(d,'po.jsonl')) if os.path.exists(os.path.join(d,'po.jsonl')) else []):
        if '"pose"' not in l: continue
        try:
            j=json.loads(l)
        except Exception:
            bad+=1; continue
        if 'klt' in j or 'status' in j: po.append(j)
    ld=max([p.get('late_dropped',0) for p in po] or [0])
    K=np.array([p['klt'][:9] for p in po if 'klt' in p and p['klt'][0]>0],float)
    rows.append(dict(tag=tag,frames=len(ms),feed_p50=round(float(np.median(ms)),2),feed_p95=round(float(np.percentile(ms,95)),2),
        feed_max=round(float(ms.max()),1),over33=int((ms>33).sum()),track_p50=round(float(np.median(tr)),2),track_p95=round(float(np.percentile(tr,95)),2),
        bad_lines=bad,late_dropped=ld,tracking=sum(p.get('status')=='tracking' for p in po),
        kept_pct=round(100*K[:,2].sum()/K[:,0].sum(),2) if len(K) else None, res_px=round(float(np.average(K[:,7],weights=K[:,1])),2) if len(K) else None,
        skin_start=g('skin_start'),skin_max=g('skin_max'),cpu9_cap=g('cpu9_cap_start'),screen=g('screen_start'),aborted=g('aborted'),charging=ac))
for r in rows: print(json.dumps(r))
json.dump(rows,open(os.path.join(root,'summary.json'),'w'),indent=1)
