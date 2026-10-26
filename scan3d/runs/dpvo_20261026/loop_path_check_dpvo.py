# scale-free sanity of loop_pct: path length (every 6th pose, as loop_eval) / bbox diagonal, DPVO vs OpenVINS
import gzip, json, numpy as np
def P(f):
    tr=[json.loads(x) for x in gzip.open(f,'rt') if x.strip()]
    return np.array([np.asarray(m["T_world_cam0"]).reshape(4,4)[:3,3] for m in tr if m.get("status")=="tracking" and m.get("T_world_cam0")])
def r(p):
    L=np.linalg.norm(np.diff(p[::6],axis=0),axis=1).sum(); d=np.linalg.norm(p.max(0)-p.min(0)); return round(L/d,2), round(L,3)
for k in ("l142148","l142610"):
    for f in (60,30):
        out=[]
        try: out.append(("OV", r(P(f"/home/system/scanner/tmp_dpvo/ov/GL{f}/{k}_o0_GL{f}.json").__class__ and P(f"/home/system/scanner/tmp_dpvo/ov/GL{f}/{k}_o0_GL{f}.poses.jsonl.gz"))))
        except Exception as e: out.append(("OV",str(e)[:60]))
        for cfg in ("base","slam"):
            try: out.append((cfg, r(P(f"/home/system/scanner/tmp_dpvo/out/{cfg}/{k}_o0_f{f}_{cfg}.final.poses.jsonl.gz"))))
            except Exception as e: out.append((cfg,str(e)[:40]))
        print(k,f,out)
