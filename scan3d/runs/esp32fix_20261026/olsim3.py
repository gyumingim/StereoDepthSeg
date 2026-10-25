from olsim import run, old
from olsim2 import mk
import sys
def show(name, f):
    print('==',name)
    W=dict(vimp=0,spin=0,hi=0,cut_h=0,tc=0)
    for h0 in [0.2,0.5,1.0,2.0,3.0]:
        row=[]
        for e in [-0.03,-0.015,0,0.015,0.03]:
            p,tc=f(h0)
            r=run(h0,1+e,p)
            row.append("%+.1f%%: v%.2f sp%.2f hi%.2f%s"%(e*100,r['vimp'],r['spin'],r['spin_hi'],'' if r['cut_h'] is None else ' CUT@%.2fm'%r['cut_h']))
            W['vimp']=max(W['vimp'],r['vimp']);W['spin']=max(W['spin'],r['spin']);W['hi']=max(W['hi'],r['spin_hi']);W['cut_h']=max(W['cut_h'],r['cut_h'] or 0);W['tc']=max(W['tc'],tc)
        print(' h0 %.1f | '%h0+' | '.join(row))
    print(' WORST',{k:round(v,2) for k,v in W.items()})
def oldf(h0):
    t1=0.4; d1=0.08; te=t1+(h0-d1)/0.4; tc=min(te*1.2+0.3,6.0); return old(h0),tc
show('old',oldf)
for kw in [dict(v1=0.4,r2=0.97,rend=0.7,tramp=0.4),dict(v1=0.6,r2=0.95,rend=0.7,tramp=0.4),dict(v1=0.6,r2=0.97,rend=0.7,tramp=0.4),dict(v1=0.5,r2=0.96,rend=0.7,tramp=0.4)]:
    show(str(kw), lambda h0: (lambda r:(r[0],r[3]))(mk(h0,**kw)))
