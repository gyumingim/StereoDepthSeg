#!/usr/bin/env python3
"""gpuba 11-11 task 2: is pycolmap SIFT extraction repeatable? Extract the same cached keyframe images N times per variant
(settings as kf_ba: PINHOLE single camera, max_image_size = 960) and compare keypoints + descriptors image by image.

  python sift_det.py IMG_DIR OUT_JSON [--reps 3] [--variants base,thr1,gpu0,nolimit,cpu] [--limit N]

Per variant: images whose (keypoints, descriptors) differ from run 0 (exact, in stored order), the same after sorting rows
(= same set, different order), keypoint-count differences, wall time per run.
"""
import argparse, json, os, sys, tempfile, time
from pathlib import Path

import numpy as np


def extract(img_dir, names, db, variant, max_feat):
    import pycolmap
    eo = pycolmap.FeatureExtractionOptions()
    eo.sift.max_num_features = max_feat
    eo.max_image_size = 960
    dev = pycolmap.Device.auto
    if variant == "thr1":
        eo.num_threads = 1
    elif variant == "gpu0":
        eo.gpu_index = "0"
    elif variant == "thr1gpu0":
        eo.num_threads = 1; eo.gpu_index = "0"
    elif variant == "nolimit":
        eo.sift.max_num_features = 100000
    elif variant == "cpu":
        eo.use_gpu = False; dev = pycolmap.Device.cpu
    ro = pycolmap.ImageReaderOptions(); ro.camera_model = "PINHOLE"
    t = time.time()
    pycolmap.extract_features(db, img_dir, image_names=names, camera_mode=pycolmap.CameraMode.SINGLE, reader_options=ro,
                              extraction_options=eo, device=dev)
    dt = time.time() - t
    d = pycolmap.Database.open(str(db))
    out = {}
    for im in d.read_all_images():
        out[im.name] = (np.asarray(d.read_keypoints(im.image_id)), np.asarray(d.read_descriptors(im.image_id).data
                        if hasattr(d.read_descriptors(im.image_id), "data") else d.read_descriptors(im.image_id)))
    d.close()
    return out, dt


def rows(kp, de):
    return np.concatenate([kp.astype(np.float64), de.astype(np.float64)], 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("img_dir"); ap.add_argument("out")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--variants", default="base,thr1,gpu0,nolimit,cpu")
    ap.add_argument("--max-features", type=int, default=6000)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--gpu-lock", action="store_true", help="hold ~/scanner/gpu.lock (gpu_guard) while extracting")
    a = ap.parse_args()
    names = sorted(p.name for p in Path(a.img_dir).glob("*.jpg"))
    if a.limit:
        names = names[:a.limit]
    import contextlib
    if a.gpu_lock:
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scanA"))
        import gpu_guard
        ctx = gpu_guard.job("gpuba sift_det", torch_cap=False)
    else:
        ctx = contextlib.nullcontext()
    res = dict(made=time.strftime("%F %T"), host=os.uname().nodename, img_dir=a.img_dir, n_images=len(names),
               max_features=a.max_features, variants={})
    with ctx, tempfile.TemporaryDirectory() as td:
        for v in a.variants.split(","):
            runs, times = [], []
            for r in range(a.reps):
                db = Path(td) / f"{v}_{r}.db"
                o, dt = extract(a.img_dir, names, db, v, a.max_features)
                runs.append(o); times.append(round(dt, 2)); db.unlink()
            n_exact = n_set = n_cnt = 0
            per = []
            for nm in names:
                k0, d0 = runs[0][nm]
                diff_exact = diff_set = diff_cnt = False
                for o in runs[1:]:
                    k1, d1 = o[nm]
                    if k1.shape != k0.shape or d1.shape != d0.shape:
                        diff_cnt = diff_exact = diff_set = True; continue
                    if not (np.array_equal(k1, k0) and np.array_equal(d1, d0)):
                        diff_exact = True
                        a0, a1 = rows(k0, d0), rows(k1, d1)
                        if not np.array_equal(a0[np.lexsort(a0.T[::-1])], a1[np.lexsort(a1.T[::-1])]):
                            diff_set = True
                n_exact += diff_exact; n_set += diff_set; n_cnt += diff_cnt
                if diff_exact:
                    per.append(dict(name=nm, n=[int(o[nm][0].shape[0]) for o in runs], set_differs=bool(diff_set)))
            res["variants"][v] = dict(times_s=times, images_differ_exact=n_exact, images_differ_as_set=n_set,
                                      images_differ_count=n_cnt, kp_median=float(np.median([runs[0][nm][0].shape[0] for nm in names])),
                                      examples=per[:10])
            print(v, json.dumps({k: x for k, x in res["variants"][v].items() if k != "examples"}), flush=True)
    Path(a.out).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
