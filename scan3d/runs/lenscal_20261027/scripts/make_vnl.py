import json, numpy as np, os, sys
T=os.path.expanduser("~/scanner/tmp_lenscal")
def grid(v):
    obj=np.array(v["obj"]); img=np.array(v["img"])
    sq=np.min(np.diff(np.unique(np.round(obj[:,0],6))))
    c=np.round(obj[:,0]/sq).astype(int); r=np.round(obj[:,1]/sq).astype(int)
    c-=1; r-=1
    return sq,r,c,img
for tag,fn in [("A","views_calcap.json"),("B","views_setB.json")]:
    d=json.load(open(os.path.join(T,fn)))
    L=["# filename x y level"]; meta=[]
    sqs=set()
    for k,v in enumerate(d["views"]):
        sq,r,c,img=grid(v); sqs.add(round(sq,6))
        assert r.min()>=0 and r.max()<=1 and c.min()>=0 and c.max()<=3, (r,c)
        G=np.full((2,4,2),np.nan); G[r,c]=img
        name="%s/%04d.pgm"%(tag,k)
        for i in range(2):
            for j in range(4):
                x,y=G[i,j]
                L.append(f"{name} - - -" if np.isnan(x) else f"{name} {x:.4f} {y:.4f} 0")
        meta.append(dict(name=name,file=v["file"],cap=v["cap"],slot=v["slot"]))
    open(os.path.join(T,f"corners_{tag}.vnl"),"w").write("\n".join(L)+"\n")
    json.dump(meta,open(os.path.join(T,f"meta_{tag}.json"),"w"))
    print(tag,len(meta),"sq",sqs)
