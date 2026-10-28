#!/usr/bin/env python3
"""appnpu 2026-10-28: check the C++ xfnpu server (PC test build + torch stand-in) against an independent Python reference
on real live-path frames (cam5.mp4 -> gray -> phone c4f60 remap -> 960x540, as npuopt10 xf960_eval.py):
  1. 'F': keypoints == official XFeat (padded 544, top-k, y < 540; xf960_eval.official) on the same frame
  2. 'M' mode 1: mutual-NN pairs == Python mutual NN on reference descriptors (queries: bicubic at the query pixel)
  3. 'M' mode 2: LighterGlue pairs == Python lgnpu pack -> LGNpu(tanh, 6) -> lgnpu.post on the same points (x512)
Reference descriptors are fp32, the server stores fp16 -> small set differences possible; reported, not hidden.
  python test_server.py SOCKNAME OUT.json [REC FRAME_A FRAME_B NQ]"""
import json, os, socket, struct, sys, time, importlib.util
import numpy as np, cv2, torch
import torch.nn.functional as F

XF_SRC = os.environ.get("XF_SRC", "/home/system/scanner/tmp_ovxfeat/xfeat_src")
sys.path.insert(0, XF_SRC)
torch.set_grad_enabled(False)
from modules.xfeat import XFeat  # noqa: E402
from modules.interpolator import InterpolateSparse2d  # noqa: E402

sock, outp = sys.argv[1], sys.argv[2]
rec = sys.argv[3] if len(sys.argv) > 3 else "/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261024_165506_std60"
fa, fb = (int(sys.argv[4]), int(sys.argv[5])) if len(sys.argv) > 5 else (600, 640)
NQ = int(sys.argv[6]) if len(sys.argv) > 6 else 150
TOPK = int(os.environ.get("TOPK", "512"))   # = the server's topk (fewer than 512 -> LighterGlue padding path)
PAD = int(os.environ.get("PAD", "2"))       # = the server's lg_pad (0 zero-centre, 2 repeat + copy sum)
RMP = "/home/system/scanner/tmp_lens05/new_gen/c4f60/remap_1920x1080.bin"
_b = open(RMP, "rb").read(); vw, vh = np.frombuffer(_b, np.int32, 2, 0)
MX = np.frombuffer(_b, np.float32, vw * vh, 8).reshape(vh, vw); MY = np.frombuffer(_b, np.float32, vw * vh, 8 + 4 * vw * vh).reshape(vh, vw)


def grab(fi):
    cap = cv2.VideoCapture(f"{rec}/cam5.mp4"); cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
    ok, fr = cap.read(); assert ok
    return cv2.remap(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), MX, MY, cv2.INTER_LINEAR)


xf = XFeat(weights=os.path.join(XF_SRC, "weights", "xfeat.pt"), top_k=TOPK)
net = xf.net.cpu().eval()
bic = InterpolateSparse2d("bicubic"); bil = InterpolateSparse2d("bilinear"); near = InterpolateSparse2d("nearest")


def ref_feat(g, q):
    """official post (xf960_eval.official, ymax 540) + query descriptors (xf_server: bicubic on the normalised map)"""
    x = cv2.copyMakeBorder(g, 0, 4, 0, 0, cv2.BORDER_REPLICATE).astype(np.float32)[None, None] / 255.0
    xn = (x - x.mean()) / np.sqrt(x.var() + 1e-5)
    f, k, h = net(torch.from_numpy(xn.astype(np.float32)))
    Kh = xf.get_kpts_heatmap(k)
    mk = xf.NMS(Kh, threshold=0.05, kernel_size=5)
    sc = (near(Kh, mk, 544, 960) * bil(h, mk, 544, 960)).squeeze(-1)
    sc[torch.all(mk == 0, dim=-1)] = -1
    sc[mk[..., 1] >= 540] = -1
    idx = torch.argsort(-sc, stable=True)[:, :TOPK]
    mk = torch.stack([torch.gather(mk[..., 0], -1, idx), torch.gather(mk[..., 1], -1, idx)], -1)
    s = torch.gather(sc, -1, idx)
    fn = F.normalize(f, dim=1)
    de = F.normalize(bic(fn, mk, 544, 960), dim=-1)
    v = s[0] > 0
    kp, de = mk[0][v].float().numpy(), de[0][v].numpy()
    dq = F.normalize(bic(fn, torch.from_numpy(q)[None], 544, 960), dim=-1)[0].numpy() if len(q) else np.zeros((0, 64), np.float32)
    return kp, np.concatenate([q, kp], 0), np.concatenate([dq, de], 0)


def mnn(da, db, mc=0.82):
    S = da @ db.T
    m12, m21 = S.argmax(1), S.argmax(0)
    ok = (m21[m12] == np.arange(len(da))) & (S.max(1) >= mc)
    return {(int(i), int(m12[i])) for i in np.nonzero(ok)[0]}


spec = importlib.util.spec_from_file_location("lgnpu", "/mnt/c/Users/a3162/scan3d_handoff/npu_results/lgnpu/lgnpu.py")
lgm = importlib.util.module_from_spec(spec); spec.loader.exec_module(lgm)
lgnet = lgm.load_net(); LGM = lgm.LGNpu(lgnet, "tanh", 6).eval()


def ref_lg(pa, da, pb, db, nqa, nqb):
    """x512: XFeat points only (after the queries); padding PAD 0 = zero desc at the centre, 2 = repeat + copy sum"""
    ia, ib = np.arange(nqa, len(pa))[:512], np.arange(nqb, len(pb))[:512]
    na, nb = len(ia), len(ib)
    norm = lambda p: (p - np.array([480.0, 270.0], np.float32)) / 480.0
    k0 = np.zeros((512, 2), np.float32); k1 = np.zeros((512, 2), np.float32)
    d0 = np.zeros((512, 64), np.float32); d1 = np.zeros((512, 64), np.float32)
    if PAD == 2:
        r0, r1 = ia[np.arange(512) % na], ib[np.arange(512) % nb]
        k0, d0, k1, d1 = norm(pa[r0]), da[r0], norm(pb[r1]), db[r1]
    else:
        k0[:na], d0[:na] = norm(pa[ia]), da[ia]; k1[:nb], d1[:nb] = norm(pb[ib]), db[ib]
    x = lgm.pack(lgnet, *[torch.from_numpy(np.ascontiguousarray(t))[None] for t in (k0, k1, d0, d1)])
    o = LGM(x).numpy()
    if PAD == 2:
        A = np.zeros((na, nb)); BT = np.zeros((nb, na))
        np.add.at(A.T, np.arange(512) % nb, o[0, :na, :512].T)
        np.add.at(BT.T, np.arange(512) % na, o[0, :nb, 512:1024].T)
    else:
        A, BT = o[0, :na, :512][:, :nb], o[0, :nb, 512:1024][:, :na]
    z0, z1 = o[0, :na, 1024], o[0, :nb, 1025]
    pairs = lgm.ref_post(A * BT.T, z0, z1)
    return {(int(ia[i]), int(ib[j])) for i, j in pairs}


def connect():
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect("\0" + sock); return s


def rd(s, n):
    b = b""
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise EOFError
        b += c
    return b


def F_(s, kid, g, q):
    s.sendall(b"F" + struct.pack("<qii", kid, g.shape[1], g.shape[0]) + g.tobytes() + struct.pack("<i", len(q)) + q.astype(np.float32).tobytes())
    nk, = struct.unpack("<i", rd(s, 4))
    kp = np.frombuffer(rd(s, 8 * nk), np.float32).reshape(nk, 2)
    ms, = struct.unpack("<f", rd(s, 4))
    return kp, ms


def M_(s, a, b, mode):
    s.sendall(b"M" + struct.pack("<qqi", a, b, mode))
    n, = struct.unpack("<i", rd(s, 4))
    if n < 0:   # 0.6.31 explicit server error code (-3 = keyframe not cached), then the server closes the session
        ms, = struct.unpack("<f", rd(s, 4))
        return n, ms
    r = np.frombuffer(rd(s, 12 * n), dtype=[("a", "<i4"), ("b", "<i4"), ("s", "<f4")])
    ms, = struct.unpack("<f", rd(s, 4))
    return {(int(x), int(y)) for x, y in zip(r["a"], r["b"])}, ms


ga, gb = grab(fa), grab(fb)
rng = np.random.default_rng(0)
qa = np.c_[rng.uniform(5, 955, NQ), rng.uniform(5, 535, NQ)].astype(np.float32)
qb = np.c_[rng.uniform(5, 955, NQ), rng.uniform(5, 535, NQ)].astype(np.float32)
s = connect()
ka, msa = F_(s, 1, ga, qa)
kb, msb = F_(s, 2, gb, qb)
m1, ms1 = M_(s, 1, 2, 1)
m2, ms2 = M_(s, 1, 2, 2)
m0, _ = M_(s, 1, 99, 2)   # unknown id -> explicit error -3 (not "0 matches"), session closed by the server
try:
    s.sendall(b"Q")
except OSError:
    pass
s.close()
rka, pa, da = ref_feat(ga, qa)
rkb, pb, db = ref_feat(gb, qb)
r1 = mnn(da, db)
r2 = ref_lg(pa, da, pb, db, NQ, NQ)
same_kp_a = len(ka) == len(rka) and bool(np.all(ka == rka))
same_kp_b = len(kb) == len(rkb) and bool(np.all(kb == rkb))
j = lambda a, b: len(a & b) / max(1, len(a | b))
res = dict(made=time.strftime("%F %T"), rec=rec, frames=[fa, fb], nq=NQ, topk=TOPK,
           kp=dict(server=[len(ka), len(kb)], ref=[len(rka), len(rkb)], identical=[same_kp_a, same_kp_b],
                   set_overlap=[len({tuple(x) for x in ka.tolist()} & {tuple(x) for x in rka.tolist()}) / max(1, len(rka)),
                                len({tuple(x) for x in kb.tolist()} & {tuple(x) for x in rkb.tolist()}) / max(1, len(rkb))]),
           nn=dict(server=len(m1), ref=len(r1), common=len(m1 & r1), jaccard=round(j(m1, r1), 4)),
           lg=dict(server=len(m2), ref=len(r2), common=len(m2 & r2), jaccard=round(j(m2, r2), 4),
                   server_pairs_in_xfeat_part=all(a >= NQ and b >= NQ for a, b in m2)),
           unknown_id_answer=m0 if isinstance(m0, int) else len(m0), server_ms=dict(F=[msa, msb], M1=ms1, M2=ms2),
           note="reference = Python fp32 (descriptors fp32); server stores descriptors fp16 -> tiny NN / LG set differences possible")
json.dump(res, open(outp, "w"), indent=1)
print(json.dumps(res, indent=1))
