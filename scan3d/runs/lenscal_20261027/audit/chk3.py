import sys, os, numpy as np
sys.path.insert(0, os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d/runs/lenscal_20261027/scripts"))
from lenscal_lib import *
import mrcal
for S in ["A","B"]:
    m, oi, names = base(S, "OPENCV4")
    Rt = mrcal.Rt_from_rt(oi["frames_rt_toref"])
    z = Rt[:, :3, 2]; t = Rt[:, 3, :]
    s = np.einsum("ij,ij->i", z, t)   # >0: board +z points away from camera
    print(S, "frac board +z pointing away from camera: %.2f" % np.mean(s > 0), "median dist m %.2f" % np.median(np.linalg.norm(t,axis=1)))
