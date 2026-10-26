import sys, os, json, numpy as np
sys.path.insert(0, os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d/runs/lenscal_20261027/scripts"))
from lenscal_lib import *
import mrcal
V = json.load(open(os.path.expanduser("~/scanner/StereoDepthSeg-integ2/scan3d/config/phone_ov_main/variants.json")))
c4 = mrcal.cameramodel(intrinsics=opencv_to_mrcal(V["K_raw_used"], V["dist"]), imagersize=(W,H))
L24d = json.load(open(os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d/config/lens/std_1920x1080_f0.50_cal20261024d.json")))
m24 = mrcal.cameramodel(intrinsics=opencv_to_mrcal(L24d["K"], L24d["dist"]), imagersize=(W,H))
def solved(S):
    m, oi, names = base(S, "OPENCV4"); o = subset(oi, np.arange(len(names))); solve(o); return model_of(o)
mods = {S: solved(S) for S in ["A","B","AB"]}
P = test_points()
def look(difflen, pts):
    gy, gx = difflen.shape
    return max(difflen[int(round(p[1]/(H-1)*(gy-1))), int(round(p[0]/(W-1)*(gx-1)))] for p in pts)
for name, mf in [("c4f60", c4), ("24d", m24)]:
    for S in ["A","B","AB"]:
        row = {}
        for lab, fr in [("r180", -1.0), ("r540", 540.), ("whole", 1e5)]:
            dl, _, _, Rt = mrcal.projection_diff([mf, mods[S]], gridn_width=61, use_uncertainties=False, focus_radius=fr)
            row[lab] = [round(float(look(dl, P[k])),2) for k in ["r0.5","r0.8","r1.0"]] + [round(float(np.degrees(np.linalg.norm(mrcal.r_from_R(Rt[:3,:])))),3)]
        print(name, "vs", S, row, flush=True)
# A vs B with different fit radius
for lab, fr in [("r180", -1.0), ("r540", 540.), ("whole", 1e5)]:
    dl, _, _, Rt = mrcal.projection_diff([mods["A"], mods["B"]], gridn_width=61, use_uncertainties=False, focus_radius=fr)
    print("A vs B", lab, [round(float(look(dl, P[k])),2) for k in ["centre","r0.5","r0.8","r1.0"]])
# remap check: c4 remap from StereoDepthSeg/c4
fn = os.path.expanduser("~/scanner/StereoDepthSeg/scan3d/config/phone_ov_main/c4/remap_1920x1080.bin")
b = open(fn,"rb").read(); w,h = np.frombuffer(b[:8],np.int32); n=w*h
mx = np.frombuffer(b[8:8+4*n],np.float32).reshape(h,w); my = np.frombuffer(b[8+4*n:8+8*n],np.float32).reshape(h,w)
src = np.frombuffer(b[8+8*n:],np.int32)
Kn = np.array(V["variant_sizes"]["c4"]["Knew"])
uu, vv = np.meshgrid(np.arange(0,w,7), np.arange(0,h,7))
pts = np.c_[uu.ravel(), vv.ravel()].astype(float)
ray = np.c_[(pts[:,0]-Kn[0,2])/Kn[0,0], (pts[:,1]-Kn[1,2])/Kn[1,1], np.ones(len(pts))]
for nm, mf in [("c4f60 K_raw_used+dist", c4), ("24d", m24), ("AB", mods["AB"])]:
    qexp = mrcal.project(ray, *mf.intrinsics())
    qmap = np.c_[mx[pts[:,1].astype(int), pts[:,0].astype(int)], my[pts[:,1].astype(int), pts[:,0].astype(int)]]
    e = np.linalg.norm(qexp-qmap,axis=1)
    print("remap", (w,h), src, nm, "max err %.3f med %.3f" % (e.max(), np.median(e)))
