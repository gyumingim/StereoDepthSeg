# recording-level + same-cell statistics for gyro_klt conditions vs PRK (score json from ate_run.py score)
import json,sys,math
import numpy as np
J=json.load(open(sys.argv[1])); ref=sys.argv[2]; conds=sys.argv[3].split(',')
drop='hs_20261021_213955'
cells=[c for c in J['cells'] if c['rec']!=drop]
for c in conds:
    both=[x for x in cells if ref in x and c in x and not x[ref]['div'] and not x[c]['div']]
    med_ref=np.median([x[ref]['ate_w'] for x in both]); med_c=np.median([x[c]['ate_w'] for x in both])
    recs={}
    for x in both: recs.setdefault(x['rec'],[]).append(x[c]['ate_w']-x[ref]['ate_w'])
    rd=[np.mean(v) for v in recs.values()]; nb=sum(d<0 for d in rd); n=len(rd)
    p=min(1,2*sum(math.comb(n,k) for k in range(0,min(nb,n-nb)+1))/2**n)
    div=[x['key'] for x in cells if x[c]['div']]
    same=sum(1 for x in cells if c in x and 'PG3' in x and x[c]['poses_hash']==x['PG3']['poses_hash']) if 'PG3' in cells[0] else None
    k=[x for x in cells if x['key']=='222747_o0'][0]
    print(f"{c}: div {len(div)} {div} | same {len(both)} cells med {med_ref:.4f} -> {med_c:.4f} | recordings {nb}/{n} better, sign p {p:.2f} | 222747_o0 {k[ref]['ate_w']:.2f} -> {k[c]['ate_w']:.2f} | identical to PG3 cells {same}")
