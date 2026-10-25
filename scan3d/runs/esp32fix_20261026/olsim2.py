from olsim import run, old, G
import sys
def mk(h0, a1=1.0, v1=0.4, r2=0.97, rend=0.7, tramp=0.4, emax=0.03, hold=0.2):
    t1=v1/a1
    def prof_ramp(ts):
        def p(t):
            r = (1-a1/G) if t<t1 else r2
            if ts is not None and t>=ts:
                f=min(1,(t-ts)/tramp); r=r*(1-f)+rend*f
            return r
        return p
    tf=run(h0,1-emax,prof_ramp(None))['tg']
    ts=run(h0,1+emax,prof_ramp(tf))['tg']
    if ts is None: ts=6
    tc=min(ts+hold,6.0)
    pr=prof_ramp(tf)
    return (lambda t: None if t>=tc else pr(t)), tf, ts, tc
def table(**kw):
    worst=dict(vimp=0,spin_hi=0,cut_h=0)
    for h0 in [0.2,0.5,1.0,2.0,3.0]:
        for e in [-0.03,-0.015,0,0.015,0.03]:
            p,tf,ts,tc=mk(h0,**kw)
            r=run(h0,1+e,p)
            worst['vimp']=max(worst['vimp'],r['vimp']); worst['spin_hi']=max(worst['spin_hi'],r['spin_hi'])
            worst['cut_h']=max(worst['cut_h'],r['cut_h'] or 0)
            if kw.get('verbose'): pass
    return worst
if __name__=='__main__':
    best=[]
    for v1 in [0.4,0.6,0.8]:
     for r2 in [0.95,0.97,0.99]:
      for rend in [0.6,0.7,0.8]:
       for tramp in [0.2,0.4]:
        w=table(v1=v1,r2=r2,rend=rend,tramp=tramp)
        best.append((w['vimp'],w,v1,r2,rend,tramp))
    best.sort(key=lambda x:x[0])
    for b in best[:12]: print(b)
    print('old worst:')
    w=dict(vimp=0,spin_hi=0,cut_h=0)
    for h0 in [0.2,0.5,1.0,2.0,3.0]:
        for e in [-0.03,-0.015,0,0.015,0.03]:
            r=run(h0,1+e,old(h0)); w['vimp']=max(w['vimp'],r['vimp']); w['spin_hi']=max(w['spin_hi'],r['spin_hi']); w['cut_h']=max(w['cut_h'],r['cut_h'] or 0)
    print(w)
