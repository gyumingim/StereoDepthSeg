#!/usr/bin/env python3
"""nncrun 2026-10-28: LighterGlue NPU (n3t256_a16) phone inputs + PC fp32 reference.
Pairs: lgev512_* (13 eval pairs: eval frame + frame 30 later, same recording; eval recordings are not the calib ones),
first 256 points (XFeat fp32 top-k order), kpts normalised (kornia, size 640x352).
Reference = original LighterGlue fp32, 6 layers, exact GELU (lg_clean.LGClean(net, False)), same 256 points.
Writes OUT/: lg_x13.bin (packed NPU input [13,256,512] f32, lgnpu.pack), lg_pairs13.bin (per pair kpts0 256x2, kpts1,
desc0 256x64, desc1 f32 -> phone C++ does the positional encoding), lg_ref.npz (ref matches + pixel kpts).
venv_ffs python (torch + kornia + cv2):  python lg_prep.py OUT"""
import json, os, sys, time
import numpy as np, torch
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/npu_20261027")
sys.path.insert(0, "/mnt/c/Users/a3162/scan3d_handoff/npu_results/lgnpu")
from lg_clean import LGClean, load_net, post as lc_post, kornia_ref  # noqa
import lgnpu  # noqa  (pack, post)
torch.set_grad_enabled(False)
O = sys.argv[1]
N = 256
B = "/home/system/scanner/tmp_npu/data"
d = {k: np.load(f"{B}/lgev512_{k}.npy")[:, :N].astype(np.float32) for k in ("kpts0", "kpts1", "desc0", "desc1")}
net = load_net()
ref = LGClean(net, False).eval()
xs, refs, kref = [], [], []
pairs = open(f"{O}/lg_pairs13.bin", "wb")
for i in range(len(d["kpts0"])):
    a = [torch.from_numpy(d[k][i:i + 1]) for k in ("kpts0", "kpts1", "desc0", "desc1")]
    P, z0, z1 = [t.numpy() for t in ref(*a)]
    m0, m1, s0 = lc_post(P, z0, z1)
    ok = (m1[m0] == np.arange(N)) & (s0 > 0.1)
    refs.append(np.array([(j, int(m0[j])) for j in np.nonzero(ok)[0]], np.int32).reshape(-1, 2))
    k0, k1, s0k = kornia_ref(net, *a)
    okk = (k1[k0] == np.arange(N)) & (s0k > 0.1)
    kref.append({(j, int(k0[j])) for j in np.nonzero(okk)[0]})
    xs.append(lgnpu.pack(net, *a).numpy().astype(np.float32))
    for k in ("kpts0", "kpts1", "desc0", "desc1"):
        pairs.write(d[k][i].tobytes())
pairs.close()
np.concatenate(xs).tofile(f"{O}/lg_x13.bin")
pix = lambda k: k * 320.0 + np.array([320.0, 176.0], np.float32)  # kornia normalize inverse (size 640x352)
np.savez(f"{O}/lg_ref.npz", **{f"m{i}": r for i, r in enumerate(refs)}, k0=pix(d["kpts0"]), k1=pix(d["kpts1"]))
agree_k = [len({tuple(p) for p in r} & kk) / max(1, len(r)) for r, kk in zip(refs, kref)]
info = {"made": time.strftime("%F %T"), "pairs": len(refs), "N": N, "ref_matches": [len(r) for r in refs],
        "LGClean_vs_kornia_agree": agree_k, "x_shape": [len(xs), N, 512]}
print(json.dumps(info))
json.dump(info, open(f"{O}/lg_prep_info.json", "w"), indent=1)
