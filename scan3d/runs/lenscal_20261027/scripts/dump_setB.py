import sys, os, json, pickle, numpy as np
S3D=os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d")
sys.path.insert(0,S3D)
import aruco_scale
P=pickle.load(open(os.path.expanduser("~/scanner/StereoDepthSeg/scan3d/runs/lens_calib_f050_20261023/views/poses.pkl"),"rb"))
print(P.keys(), len(P["poses"]))
from collections import Counter
print(Counter(p["board"] for p in P["poses"]))
p=P["poses"][5]; print({k:(type(v),len(v) if hasattr(v,"__len__") else v) for k,v in p.items()})
cl={p["rec"]:p["rec"] for p in P["poses"]}
for a,b,_ in P["near"]:
    if a in cl and b in cl:
        ra,rb=cl[a],cl[b]
        for k in cl:
            if cl[k]==rb: cl[k]=ra
out=[]; boards={}
for p in P["poses"]:
    if "5x3" not in p["board"]: continue
    if p["board"] not in boards:
        boards[p["board"]]=aruco_scale.Board(aruco_scale.load_board_spec(os.path.join(S3D,"config","charuco",p["board"])))
    b=boards[p["board"]]; vs=p["views"]
    sel=[vs[int(round(x))] for x in np.linspace(0,len(vs)-1,min(4,len(vs)))]
    for fi,ids,c2 in sel:
        ids=np.asarray(ids).ravel()
        out.append(dict(file=p["rec"]+"#%d"%fi,cap=cl[p["rec"]],slot=p["rec"],board=p["board"],ids=ids.tolist(),obj=b.obj[ids].tolist(),img=np.asarray(c2).reshape(-1,2).tolist()))
print(len(out), len(set(o["cap"] for o in out)), Counter(len(o["ids"]) for o in out), set(o["board"] for o in out))
print("sq", [ (k, getattr(v,"sq",None)) for k,v in boards.items()])
json.dump(dict(src="lens_calib_f050_20261023/views/poses.pkl 5x3 only, 4 frames/rec evenly",views=out),open(os.path.expanduser("~/scanner/tmp_lenscal/views_setB.json"),"w"))
