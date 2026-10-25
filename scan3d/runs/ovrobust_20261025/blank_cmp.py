import json,gzip,glob,os
B='/tmp/ovrobust/ate'
A=json.load(open(f'{B}/ana_H_BL1_BLR1b_BL2_BLR2b.json'))
def rec_t(path):
    L=[json.loads(x) for x in gzip.open(path,'rt') if x.strip()]; t0=int(L[0]['t_ns']); p=0; ev=[]
    for m in L:
        n=m.get('n_reinit_state',0)
        if n>p: ev.append(round((int(m['t_ns'])-t0)/1e9,1)); p=n
    # recovery time: first OK after blank end
    return L,t0,ev
for bl,br,d in [('BL1','BLR1b',1),('BL2','BLR2b',2)]:
    print(f'== {bl} vs {br}')
    rt=[]
    for r in A['cells']:
        if bl not in r: continue
        L,t0,ev=rec_t(f"{B}/{br}/{r['key']}_{br}.poses.jsonl.gz")
        Lb,_,_=rec_t(f"{B}/{bl}/{r['key']}_{bl}.poses.jsonl.gz")
        # time from blank end to first OK state (no-recover run)
        te=15+d; tok=None
        for m in Lb:
            t=(int(m['t_ns'])-t0)/1e9
            if t>=te and m.get('h',{}).get('state')=='OK': tok=round(t-te,2); break
        if r.get('B0_init_s') and r['B0_init_s']<=13: rt.append(tok)
        if ev or r[bl]['hash']!=r[br]['hash']:
            a=(r[bl].get('blank') or {}).get('ate_after_cm'); b=(r[br].get('blank') or {}).get('ate_after_cm')
            ha=(r.get('H_blank_ref',{}).get(str(d)) or {}).get('ate_after_cm')
            print(r['key'],'H',ha,bl,a,br,b,'ev',ev,'same',r[bl]['hash']==r[br]['hash'])
    v=sorted(x for x in rt if x is not None); print('time blank-end->OK (no recover) n',len(v),'med',v[len(v)//2],'max',v[-1],'never',sum(x is None for x in rt))
