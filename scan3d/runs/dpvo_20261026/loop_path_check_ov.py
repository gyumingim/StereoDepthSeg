import json, numpy as np
def r(p):
    L=np.linalg.norm(np.diff(p[::6],axis=0),axis=1).sum(); d=np.linalg.norm(p.max(0)-p.min(0)); return round(L/d,2), round(L,3)
for k in ("l142148","l142610"):
    for f in (60,30):
        t=np.array([x[1:4] for x in json.load(open(f"/home/system/scanner/tmp_dpvo/ov/GL{f}/{k}_o0_GL{f}.json"))["traj"]],float)
        print(k,f,"OV path/extent, path_m", r(t))
