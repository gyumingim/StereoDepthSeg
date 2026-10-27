"""npu 2026-10-27: accuracy metrics (numpy only, no torch) shared by acc_pc.py / acc_phone.py.
XFeat: same post as ovxfeat acc_compare (softmax over 65 ch, drop dustbin, pixel-shuffle 8, NMS 5 (max-pool), thr 0.05,
score = K * H(nearest), top 1024); overlap = share of reference top-1024 points with a test point within 1 px;
desc cos = cosine of L2-normalised 64-d feats at the reference points (nearest cell of the 1/8 grid).
LighterGlue: mutual matches with score > 0.1 (kornia filter_matches); agreement = |ref ∩ test| / |ref|."""
import numpy as np
H, W = 352, 640
h, w = H // 8, W // 8


def kp_heat(k):  # k (65,h,w) logits -> (H,W) heat
    e = np.exp(k - k.max(0, keepdims=True)); s = (e / e.sum(0, keepdims=True))[:64]
    return s.reshape(8, 8, h, w).transpose(2, 0, 3, 1).reshape(H, W)


def topk(f, k, hm, n=1024, thr=0.05):
    K = kp_heat(k)
    from numpy.lib.stride_tricks import sliding_window_view
    P = np.pad(K, 2, constant_values=-1)
    mx = sliding_window_view(P, (5, 5)).max((-1, -2))
    ys, xs = np.nonzero((K == mx) & (K > thr))
    sc = K[ys, xs] * hm[ys // 8, xs // 8]
    o = np.argsort(-sc, kind="stable")[:n]
    return np.stack([xs[o], ys[o]], 1).astype(np.float32)


def xf_compare(ref, tst):
    """ref/tst = (feats (64,h,w), keypoints (65,h,w), heatmap (h,w))"""
    a, b = topk(*ref), topk(*tst)
    if len(b) == 0:
        return 0.0, float("nan"), len(a), 0, float("nan")
    d = np.sqrt(((a[:, None] - b[None]) ** 2).sum(-1)).min(1)
    fr = ref[0] / np.linalg.norm(ref[0], axis=0, keepdims=True); ft = tst[0] / np.linalg.norm(tst[0], axis=0, keepdims=True)
    gx, gy = (a[:, 0] // 8).astype(int), (a[:, 1] // 8).astype(int)
    cos = (fr[:, gy, gx] * ft[:, gy, gx]).sum(0)
    return float((d <= 1.0).mean()), float(np.median(cos)), len(a), len(b), float(cos.min())


def lg_post(P, z0, z1):
    ls = lambda z: -np.logaddexp(0, -z)
    sc = np.log(np.maximum(P.astype(np.float64), 1e-30)) + ls(z0.reshape(-1, 1).astype(np.float64)) + ls(z1.reshape(1, -1).astype(np.float64))
    m0, m1 = sc.argmax(1), sc.argmax(0)
    mut = m1[m0] == np.arange(len(m0))
    s0 = np.exp(sc.max(1))
    ok = mut & (s0 > 0.1)
    return {(i, int(m0[i])) for i in np.nonzero(ok)[0]}


def lg_compare(ref, tst):
    r, t = lg_post(*ref), lg_post(*tst)
    return (len(r & t) / max(1, len(r))), len(r), len(t)
