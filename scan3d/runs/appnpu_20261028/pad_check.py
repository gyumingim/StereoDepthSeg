#!/usr/bin/env python3
"""appnpu 2026-10-28: unmasked LighterGlue L6 (fixed N = 512) with fewer real points: which padding is closest to running
the same network on exactly the real points? (PC torch fp32 = the L6_a16mc graph before quantisation; NOT the phone NNC.)
Real live-path frames (cam5 -> c4f60 remap 960x540), XFeat official top-512 (padded 544), first n points kept.
  pad0 = zero descriptor at the image centre (server lg_pad 0, = lgpost masked-model padding convention)
  pad1 = repeat the real points cyclically, argmax over the real points only (first idea, no copy sum)
  pad2 = repeat cyclically AND sum the dual-softmax probabilities over the copies of each point (server lg_pad 1): copies
         stay identical through every layer, so with N = c * n this equals the network on the n points (attention weight
         per distinct key unchanged); otherwise the multiplicities differ by 1
Score = Jaccard of the match set vs the exact-n reference.   python pad_check.py OUT.json"""
import json, os, sys, time, importlib.util
import numpy as np, cv2, torch
import torch.nn.functional as F

XF_SRC = "/home/system/scanner/tmp_ovxfeat/xfeat_src"
sys.path.insert(0, XF_SRC)
torch.set_grad_enabled(False)
from modules.xfeat import XFeat  # noqa: E402
from modules.interpolator import InterpolateSparse2d  # noqa: E402

R = "/home/system/scanner/StereoDepthSeg/scan3d/recordings"
PAIRS = [("hs_20261024_165506_std60", 300, 330), ("hs_20261024_165506_std60", 900, 960), ("hs_20261024_165506_std60", 1300, 1420),
         ("hs_20261024_173432_std60", 400, 430), ("hs_20261024_173432_std60", 1000, 1090)]
RMP = "/home/system/scanner/tmp_lens05/new_gen/c4f60/remap_1920x1080.bin"
_b = open(RMP, "rb").read(); vw, vh = np.frombuffer(_b, np.int32, 2, 0)
MX = np.frombuffer(_b, np.float32, vw * vh, 8).reshape(vh, vw); MY = np.frombuffer(_b, np.float32, vw * vh, 8 + 4 * vw * vh).reshape(vh, vw)
xf = XFeat(weights=os.path.join(XF_SRC, "weights", "xfeat.pt"), top_k=512); net = xf.net.cpu().eval()
bic = InterpolateSparse2d("bicubic"); bil = InterpolateSparse2d("bilinear"); near = InterpolateSparse2d("nearest")
spec = importlib.util.spec_from_file_location("lgnpu", "/mnt/c/Users/a3162/scan3d_handoff/npu_results/lgnpu/lgnpu.py")
lgm = importlib.util.module_from_spec(spec); spec.loader.exec_module(lgm)
lgnet = lgm.load_net(); LGM = lgm.LGNpu(lgnet, "tanh", 6).eval()


def grab(rec, fi):
    cap = cv2.VideoCapture(f"{R}/{rec}/cam5.mp4"); cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
    ok, fr = cap.read(); assert ok
    return cv2.remap(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), MX, MY, cv2.INTER_LINEAR)


def feat(g):
    x = cv2.copyMakeBorder(g, 0, 4, 0, 0, cv2.BORDER_REPLICATE).astype(np.float32)[None, None] / 255.0
    xn = (x - x.mean()) / np.sqrt(x.var() + 1e-5)
    f, k, h = net(torch.from_numpy(xn.astype(np.float32)))
    Kh = xf.get_kpts_heatmap(k); mk = xf.NMS(Kh, threshold=0.05, kernel_size=5)
    sc = (near(Kh, mk, 544, 960) * bil(h, mk, 544, 960)).squeeze(-1)
    sc[torch.all(mk == 0, dim=-1)] = -1; sc[mk[..., 1] >= 540] = -1
    idx = torch.argsort(-sc, stable=True)[:, :512]
    mk = torch.stack([torch.gather(mk[..., 0], -1, idx), torch.gather(mk[..., 1], -1, idx)], -1)
    de = F.normalize(bic(F.normalize(f, dim=1), mk, 544, 960), dim=-1)
    return mk[0].float().numpy(), de[0].numpy()


norm = lambda p: (p - np.array([480.0, 270.0], np.float32)) / 480.0


def run(k0, d0, k1, d1, n0, n1, N, pad):
    if pad == 2:
        i0, i1 = np.arange(N) % n0, np.arange(N) % n1
        o = LGM(lgm.pack(lgnet, *[torch.from_numpy(np.ascontiguousarray(t))[None] for t in (norm(k0[i0]), norm(k1[i1]), d0[i0], d1[i1])])).numpy()
        A = np.zeros((n0, n1)); BT = np.zeros((n1, n0))
        np.add.at(A.T, i1, o[0, :n0, :N].T)          # A2[i, j] = sum over the copies j2 of j of A[i, j2]
        np.add.at(BT.T, i0, o[0, :n1, N:2 * N].T)    # BT2[j, i] = sum over the copies i2 of i of BT[j, i2]
        return lgm.ref_post(A * BT.T, o[0, :n0, 2 * N], o[0, :n1, 2 * N + 1])
    K0 = np.zeros((N, 2), np.float32); K1 = np.zeros((N, 2), np.float32); D0 = np.zeros((N, 64), np.float32); D1 = np.zeros((N, 64), np.float32)
    if pad == 1:
        i0, i1 = np.arange(N) % n0, np.arange(N) % n1
        K0, D0, K1, D1 = norm(k0[i0]), d0[i0], norm(k1[i1]), d1[i1]
    else:
        K0[:n0], D0[:n0], K1[:n1], D1[:n1] = norm(k0[:n0]), d0[:n0], norm(k1[:n1]), d1[:n1]
    o = LGM(lgm.pack(lgnet, *[torch.from_numpy(np.ascontiguousarray(t))[None] for t in (K0, K1, D0, D1)])).numpy()
    A, BT, z0, z1 = o[0, :n0, :N][:, :n1], o[0, :n1, N:2 * N][:, :n0], o[0, :n0, 2 * N], o[0, :n1, 2 * N + 1]
    return lgm.ref_post(A * BT.T, z0, z1)


LGC = lgm.LGClean(lgnet, True).eval()   # same network (tanh GELU) with separate point sets of any size = the exact reference
NS = [(128, 128), (200, 200), (256, 256), (384, 384), (448, 448), (512, 300), (448, 200), (300, 512)]


def run_exact(k0, d0, k1, d1, n0, n1):
    out = LGC(*[torch.from_numpy(np.ascontiguousarray(t))[None] for t in (norm(k0[:n0]), norm(k1[:n1]), d0[:n0], d1[:n1])])
    return lgm.ref_post(*[t[0].numpy() for t in out])


rows = []
t0 = time.time()
# self-check: the exact reference == LGNpu at 512 x 512 (no padding)

for rec, fa, fb in PAIRS:
    ka, da = feat(grab(rec, fa)); kb, db = feat(grab(rec, fb))
    if not rows:
        e = run_exact(ka, da, kb, db, 512, 512); q = run(ka, da, kb, db, 512, 512, 512, 0)
        selfcheck = dict(exact_vs_lgnpu_512=len(e & q) / max(1, len(e | q)), n=len(e))
        print("selfcheck", selfcheck, flush=True)
    for n0, n1 in NS:
        n = f"{n0}x{n1}"
        ref = run_exact(ka, da, kb, db, n0, n1)       # exactly n0 / n1 points (no padding at all)
        p0 = run(ka, da, kb, db, n0, n1, 512, 0)
        p1 = run(ka, da, kb, db, n0, n1, 512, 1)
        p2 = run(ka, da, kb, db, n0, n1, 512, 2)
        j = lambda a, b: len(a & b) / max(1, len(a | b))
        rows.append(dict(rec=rec, frames=[fa, fb], n=n, n_ref=len(ref), n_pad0=len(p0), n_pad1=len(p1), n_pad2=len(p2),
                         jac_pad0=round(j(ref, p0), 4), jac_pad1=round(j(ref, p1), 4), jac_pad2=round(j(ref, p2), 4)))
        print(rows[-1], flush=True)
summ = {f"{n[0]}x{n[1]}": {f"jac_pad{k}_mean": round(float(np.mean([r[f"jac_pad{k}"] for r in rows if r["n"] == f"{n[0]}x{n[1]}"])), 4) for k in (0, 1, 2)}
        for n in NS}
json.dump(dict(selfcheck=selfcheck, made=time.strftime("%F %T"), secs=round(time.time() - t0, 1), pairs=PAIRS, rows=rows, summary=summ,
               note="PC torch fp32 L6 tanh (pre-quantisation graph); reference = same net on exactly n points"), open(sys.argv[1], "w"), indent=1)
print(json.dumps(summ, indent=1))
