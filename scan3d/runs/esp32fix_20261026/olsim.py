import itertools
G=9.81
def run(h0, m, prof, dt=0.001):
    # prof(t)-> (ratio or None=cut)
    z=h0; v=0; t=0; tg=None; vimp=0; spin=0; spin_hi=0; cut_h=None
    while t<8:
        r=prof(t)
        if r is None:
            if tg is None: cut_h=z
            break
        a=(r*m-1)*G
        if tg is None:
            v+=a*dt; z+=v*dt
            if z<=0: tg=t; vimp=-v; z=0; v=0
        else:
            if a>0: pass  # would lift off; ignore (assume stays)
            spin+=dt
            if r*m>0.85: spin_hi+=dt
        t+=dt
    if tg is None:  # fell from cut_h with v
        vimp=(v*v+2*G*z)**0.5 if z>0 else -v
    return dict(tg=tg, vimp=vimp, spin=spin, spin_hi=spin_hi, cut_h=cut_h, tcut=t)

def old(h0):
    t1=0.4; d1=0.08
    te=t1+(h0-d1)/0.4; tc=te*1.2+0.3
    return lambda t: None if t>=tc else (1-1/G if t<t1 else 0.99)

def nominal_tg(h0, prof):
    return run(h0,1.0,prof)['tg']

def new(h0, r2=0.97, rend=0.6, tramp=0.5, hold=0.25, pre=0.25):
    t1=0.4
    base=lambda t: (1-1/G if t<t1 else r2)
    tn=nominal_tg(h0, base)
    ts=tn-pre
    def p(t):
        if t>=tn+hold: return None
        if t<ts: return base(t)
        f=min(1,(t-ts)/tramp); return base(t)*(1-f)+rend*f
    return p, tn

if __name__=="__main__":
   for name in ['old','new']:
    print('==',name)
    for h0 in [0.3,1.0,2.0,3.0]:
      row=[]
      for e in [-0.03,-0.015,0,0.015,0.03]:
          if name=='old': p=old(h0)
          else: p,_=new(h0)
          r=run(h0,1+e,p)
          row.append("e%+.3f: vimp %.2f spin %.2f hi %.2f cut_h %s"%(e,r['vimp'],r['spin'],r['spin_hi'],None if r['cut_h'] is None else round(r['cut_h'],2)))
      print('h0',h0); print('\n'.join('   '+x for x in row))
