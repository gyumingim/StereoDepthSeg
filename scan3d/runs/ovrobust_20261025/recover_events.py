import gzip,json,glob,os
B='/tmp/ovrobust/ate'
import sys
for c,d in [(x,0) for x in (sys.argv[1:] or ['BLR1b','BLR2','BLR2b','R'])]:
    out=[]
    for f in sorted(glob.glob(f'{B}/{c}/*.poses.jsonl.gz')):
        L=[json.loads(x) for x in gzip.open(f,'rt') if x.strip()]
        t0=int(L[0]['t_ns']); prev=0; ev=[]
        rk=[k for k in L[-1] if k.startswith('n_reinit')]
        for m in L:
            n=sum(m.get(k,0) for k in ('n_reinit_state','n_reinit_full'))
            if n>prev: ev.append(round((int(m['t_ns'])-t0)/1e9,2)); prev=n
        segs=max(m.get('seg',0) for m in L)
        if ev: out.append((os.path.basename(f)[:9],ev,segs))
    print(c,'cells with events',len(out),'keys',rk)
    for o in out: print('  ',o)
