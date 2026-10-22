import gzip, json, sys
import numpy as np
from pathlib import Path
B = Path("/home/system/scanner/StereoDepthSeg/scan3d")
RUNS = B/"runs/board_rotation"
RECS = ["hs_20261022_133820","hs_20261022_134204","hs_20261022_134550"]

def acc(path):
    t=[];a=[]
    for line in open(path):
        if line.startswith("35,"):
            p=line.split(",")
            t.append(int(p[1])); a.append([float(p[2]),float(p[3]),float(p[4])])
    return np.array(t),np.array(a)

def umeyama(src,dst):
    ms,md=src.mean(0),dst.mean(0); A=src-ms; Bm=dst-md
    U,S,Vt=np.linalg.svd(Bm.T@A/len(src))
    D=np.eye(3); D[2,2]=np.sign(np.linalg.det(U@Vt))
    R=U@D@Vt; s=np.trace(np.diag(S)@D)/((A**2).sum(1).mean())
    return s,R,md-s*R@ms

def movavg(t,x,w):
    tg=np.arange(t[0],t[-1],1/240.); xg=np.column_stack([np.interp(tg,t,x[:,k]) for k in range(3)])
    n=int(round(240*w)); h=n//2
    pad=np.vstack([np.repeat(xg[:1],h,0),xg,np.repeat(xg[-1:],n-1-h,0)])
    c=np.vstack([np.zeros((1,3)),np.cumsum(pad,0)])
    yg=(c[n:]-c[:-n])/n
    return np.column_stack([np.interp(t,tg,yg[:,k]) for k in range(3)])

def ang(a,b): return float(np.degrees(np.arccos(np.clip(abs(a@b)/np.linalg.norm(a)/np.linalg.norm(b),0,1))))

out={}
for rec in RECS:
    R=B/"recordings"/rec
    al=json.load(open(R/"imu_gt/align.json")); Rci=np.array(al["R_ci"])
    g=np.load(R/"imu_gt_spl/gt.npz",allow_pickle=True)
    ok=g["ok"].astype(bool)&(np.linalg.norm(g["p_bc"],axis=1)<1.2)
    gt_t,gp,gR=g["t_ns"][ok],g["p_bc"][ok],g["R_bc"][ok]
    grs=np.load(R/"imu_gt_rs/gt.npz",allow_pickle=True)
    okrs=(grs["ok"].astype(bool)&(np.linalg.norm(grs["p_bc"],axis=1)<1.2)).mean()
    ta,a=acc(R/"imu.csv"); ta=ta+int(al["dt_ns"])
    ai=np.column_stack([np.interp(gt_t,ta,a[:,k]) for k in range(3)])
    fb=np.einsum('nij,jk,nk->ni',gR,Rci,ai); fbT=np.einsum('nij,kj,nk->ni',gR,Rci,ai)
    ug=fb.mean(0); ug/=np.linalg.norm(ug); ugT=fbT.mean(0); ugT/=np.linalg.norm(ugT)
    # still start (first 3 s of GT ok frames)
    ts=(gt_t-gt_t[0])/1e9; st=ts<3
    ug_st=fb[st].mean(0); ug_st/=np.linalg.norm(ug_st)
    # spread of per-frame direction (unit) - correct convention should be tighter
    def spread(f):
        u=f/np.linalg.norm(f,axis=1,keepdims=True); m=u.mean(0); m/=np.linalg.norm(m)
        return float(np.degrees(np.median(np.arccos(np.clip(u@m,-1,1)))))
    # camera axis of gravity: mean of R_ci a in camera frame (start still)
    gcam=(Rci@ai[st].mean(0)); gcam/=np.linalg.norm(gcam)
    r=dict(okrs=round(float(okrs),3), ok_spl=round(float(ok.mean()),3), dist_med_m=round(float(np.median(np.linalg.norm(gp,axis=1))),3),
           u_g=ug.round(3).tolist(), ang_y_ug=round(ang(ug,np.eye(3)[1]),2), ang_x_ug=round(ang(ug,np.eye(3)[0]),2),
           ug_vs_still=round(ang(ug,ug_st),2), ang_y_still=round(ang(ug_st,np.eye(3)[1]),2),
           spread_Rci=round(spread(fb),2), spread_RciT=round(spread(fbT),2), ang_y_ugT=round(ang(ugT,np.eye(3)[1]),2),
           gcam_start=gcam.round(3).tolist(), gt_std_cm=dict(), runs={})
    # motion std per axis
    E=np.eye(3); h1=np.cross(ug,E[2]); h1/=np.linalg.norm(h1); h2=np.cross(ug,h1)
    pm=gp[(ts>4)&(ts<ts[-1]-4)]
    r["gt_std_cm"]={"x":(pm[:,0].std()*100).round(1),"y":(pm[:,1].std()*100).round(1),"z":(pm[:,2].std()*100).round(1),
                    "v":((pm@ug).std()*100).round(1),"h1":((pm@h1).std()*100).round(1),"h2":((pm@h2).std()*100).round(1)}
    for k in range(3):
        j=json.load(open(RUNS/f"{rec[-6:]}_base_o{k}.json"))
        T=[];P=[];Rw=[]
        for line in gzip.open(RUNS/f"{rec[-6:]}_base_o{k}.poses.jsonl.gz","rt"):
            if not line.strip(): continue
            m=json.loads(line)
            if m.get("status")=="tracking" and m.get("T_world_cam0"):
                M=np.asarray(m["T_world_cam0"]).reshape(4,4)
                T.append(m["t_ns"]+int(round(j["align_dt_ms"]*1e6))); P.append(M[:3,3]); Rw.append(M[:3,:3])
        T=np.array(T);P=np.array(P);Rw=np.array(Rw)
        i=np.clip(np.searchsorted(gt_t,T),1,len(gt_t)-1); i-= (np.abs(gt_t[i-1]-T)<=np.abs(gt_t[i]-T))
        sel=np.abs(gt_t[i]-T)<=2e6
        P,Pg,Rw,ii=P[sel],gp[i[sel]],Rw[sel],i[sel]; tt=(T[sel]-T[sel][0])/1e9
        s,Rr,tr=umeyama(P,Pg); e=s*P@Rr.T+tr-Pg
        ate=float(np.sqrt((e**2).sum(1).mean())*100)
        # VIO up in board frame two ways
        up_sim=Rr@np.array([0,0,1.])
        upc=np.einsum('nij,nji->ni',gR[ii],Rw[:,:,2:3].transpose(0,2,1).repeat(1,1)) if False else None
        up_cam=np.einsum('nji,j->ni',Rw,np.array([0,0,1.]))      # R_wc^T ez in cam
        up_b=np.einsum('nij,nj->ni',gR[ii],up_cam); up_b=up_b.mean(0); up_b/=np.linalg.norm(up_b)
        d=dict(n=int(sel.sum()),ate_sim3_cm=round(ate,2),scale=round(s,3),json_ate=j.get("ate_cm"),json_scale=j.get("scale"),
               rebias=j.get("gyro_rebias"),ang_simup_ug=round(ang(up_sim,ug),2),ang_vioup_ug=round(ang(up_b,ug),2))
        collapsed = j["ate_cm"]>10 or not 0.5<=j["scale"]<=2
        d["collapsed"]=collapsed
        for lab,w in (("L30",30.),("L5",5.)):
            L=movavg(tt,e,w); tot=(L**2).sum()
            fx,fy,fz=[(L[:,q]**2).sum()/tot for q in range(3)]
            fv=((L@ug)**2).sum()/tot; f1=((L@h1)**2).sum()/tot; f2=((L@h2)**2).sum()/tot
            d[lab]=dict(rms=round(float(np.sqrt((L**2).sum(1).mean())*100),3),fv=round(fv,3),fy=round(fy,3),fx=round(fx,3),fz=round(fz,3),
                        fh1=round(f1,3),fh2=round(f2,3),r=round(fy/fv,3))
            if lab=="L30": d["_L"]=(tt,L)
        r["runs"][k]=d
    good=[d for d in r["runs"].values() if not d["collapsed"]]
    for lab in ("L30","L5"):
        if good:
            r[lab+"_med"]={q:round(float(np.median([d[lab][q] for d in good])),3) for q in ("rms","fv","fy","fx","fz","fh1","fh2","r")}
    # common component
    if len(good)>=2:
        t0=max(d["_L"][0][0] for d in good); t1=min(d["_L"][0][-1] for d in good)
        tc=np.arange(t0,t1,0.01)
        Lc=np.mean([np.column_stack([np.interp(tc,*[d["_L"][0],d["_L"][1][:,q]]) for q in range(3)]) for d in good],0)
        tot=(Lc**2).sum(); r["common"]=dict(fv=round(float(((Lc@ug)**2).sum()/tot),3),fy=round(float((Lc[:,1]**2).sum()/tot),3))
    for d in r["runs"].values(): d.pop("_L",None)
    out[rec]=r
print(json.dumps(out,indent=1,default=float))
