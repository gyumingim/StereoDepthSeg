import json,sys,numpy as np
for p in sys.argv[1:]:
    K=[]
    for l in open(p):
        if '"pose"' not in l or '"klt"' not in l: continue
        j=json.loads(l); k=j['klt']
        if k[0]>0 and len(k)>=13: K.append(k)
    K=np.array(K,float)
    px,py,fx,fy=K[:,9],K[:,10],K[:,11],K[:,12]
    print(p.split('/')[-1], 'frames',len(K))
    for lag in range(-3,4):
        a=slice(max(0,lag),len(K)+min(0,lag)); b=slice(max(0,-lag),len(K)-max(0,lag))
        cx=np.corrcoef(px[a],fx[b])[0,1]; cy=np.corrcoef(py[a],fy[b])[0,1]
        print(f'  lag {lag:+d}: corr x {cx:.3f} y {cy:.3f}')
    sx=np.sum(px*fx)/np.sum(px*px); sy=np.sum(py*fy)/np.sum(py*py)
    print(f'  gain flow/pred x {sx:.3f} y {sy:.3f}; corr px-fy {np.corrcoef(px,fy)[0,1]:.3f} py-fx {np.corrcoef(py,fx)[0,1]:.3f}')
    print('  mean |pred| %.2f |flow| %.2f'%(np.mean(np.hypot(px,py)),np.mean(np.hypot(fx,fy))))
