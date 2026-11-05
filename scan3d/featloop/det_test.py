# featloop: is pycolmap SIFT extraction / matching+verification deterministic? (CPU only, no GPU lock)
import pycolmap, numpy as np, sys, json, os
from pathlib import Path
img = Path(os.path.expanduser("~/scanner/featloop_ext/cache/hs_20261024_165506_std60/kf10_s0.5"))
pl = Path(os.path.expanduser("~/scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec/165506/ba_S/pairs.txt"))
names = sorted({n for l in pl.read_text().split() for n in [l]})
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
Kr = json.load(open(os.path.expanduser("~/scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec/165506/ba_S/ba.json")))["Kr"]
res = {}
for rep in range(2):
    db = out / f"db{rep}.db"
    if db.exists(): db.unlink()
    eo = pycolmap.FeatureExtractionOptions(); eo.sift.max_num_features = 6000; eo.max_image_size = 960
    ro = pycolmap.ImageReaderOptions(); ro.camera_model = "PINHOLE"
    ro.camera_params = f"{Kr[0][0]:.10g},{Kr[1][1]:.10g},{Kr[0][2]+0.5:.10g},{Kr[1][2]+0.5:.10g}"
    pycolmap.extract_features(db, img, image_names=names, camera_mode=pycolmap.CameraMode.SINGLE, reader_options=ro, extraction_options=eo, device=pycolmap.Device.cpu)
    po = pycolmap.ImportedPairingOptions(); po.match_list_path = str(pl)
    pycolmap.match_image_pairs(str(db), matching_options=pycolmap.FeatureMatchingOptions(), pairing_options=po, device=pycolmap.Device.cpu)
    d = pycolmap.Database.open(str(db)); ids, g = d.read_two_view_geometries()
    kp = {im.name: np.asarray(d.read_keypoints(im.image_id)) for im in d.read_all_images()}
    d.close()
    res[rep] = (kp, {(tuple(i) if isinstance(i, tuple) else int(i)): len(np.asarray(x.inlier_matches)) for i, x in zip(ids, g)})
    print("rep", rep, "inliers median", np.median(list(res[rep][1].values())), flush=True)
k0, k1 = res[0][0], res[1][0]
print("keypoints identical:", all(np.array_equal(k0[n], k1[n]) for n in k0))
i0, i1 = res[0][1], res[1][1]
print("inlier counts identical:", i0 == i1, "n differ", sum(i0[k] != i1.get(k) for k in i0))
