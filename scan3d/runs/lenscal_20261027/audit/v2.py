import sys, os, numpy as np
sys.path.insert(0, os.path.expanduser("~/scanner/StereoDepthSeg-calcap/scan3d/runs/lenscal_20261027/scripts"))
from lenscal_lib import *
import mrcal
m, oi, names = base("A", "OPENCV4")
o = subset(oi, np.arange(20), outlier=False)
mrcal.optimize(**o)
w = 0.0005
o["calobject_warp"] = np.array([w, 0.0])
x = mrcal.optimizer_callback(**o, no_jacobian=True)[1] if False else mrcal.optimizer_callback(**o)[1]
obj = mrcal.ref_calibration_object(4, 2, o["calibration_object_spacing"], calobject_warp=o["calobject_warp"])
q = mrcal.project(mrcal.transform_point_rt(o["frames_rt_toref"][:, None, None, :], obj[None]), o["lensmodel"], o["intrinsics"][0])
d = (q - o["observations_board"][..., :2]).ravel()
print("C-core residual vs python ref_calibration_object(warp) max diff", np.max(np.abs(x[:d.size] - d)))
print("obj z per corner (mm)", np.round(obj[..., 2] * 1000, 3))
# camera-frame depth change of centre corners due to +warp
Rt = mrcal.Rt_from_rt(o["frames_rt_toref"])
p0 = mrcal.transform_point_rt(o["frames_rt_toref"][:, None, None, :], mrcal.ref_calibration_object(4, 2, o["calibration_object_spacing"])[None])
p1 = mrcal.transform_point_rt(o["frames_rt_toref"][:, None, None, :], obj[None])
print("range change of inner corners for +0.5mm warp (mm): min %.3f max %.3f" % ((np.linalg.norm(p1,axis=-1)-np.linalg.norm(p0,axis=-1))[:, :, 1:3].min()*1000, (np.linalg.norm(p1,axis=-1)-np.linalg.norm(p0,axis=-1))[:, :, 1:3].max()*1000))
