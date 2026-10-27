#!/usr/bin/env python3
"""(lgopt 2026-10-27 copy of runs/xfopt_20261027/xf_server.py: + --lg-width / --lg-depth / --lg-prune-min, defaults = xfopt behaviour)
ovxfeat 2026-10-26: XFeat (+ LighterGlue) feature / matching server for ov_bridge --loop-xf (PC prototype).

The bridge's loop thread starts this as a child process and talks over stdin/stdout (binary, little endian):
  'F' i64 id, i32 W, i32 H, u8[W*H] gray image, i32 nq, f32[2*nq] query pixels (OpenVINS tracked points)
      -> i32 nk, f32[2*nk] detected keypoints (pixels of the given image), f32 ms
      the server keeps descriptors of the point list [queries (nq), keypoints (nk)] under id.
  'M' i64 idA, i64 idB, i32 mode (1 = mutual nearest neighbour, cos >= --min-cos; 2 = LighterGlue)
      -> i32 n, n x (i32 ia, i32 ib, f32 score), f32 ms      (indices into the point lists above)
  'Q' -> exit
Options: --topk (sparse XFeat keypoints), --w/--h (resize the keyframe before XFeat; 0 = as given; XFeat then rounds to
multiples of 32), --dense 1 (XFeat* semi-dense coarse points, dual scale, no refinement), --min-cos, --lg-conf, --log FILE.
Source: https://github.com/verlab/accelerated_features e92685f (Apache-2.0); LighterGlue = kornia 0.7.2 LightGlue.
"""
import argparse
import json
import os
import struct
import sys
import time

os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")
import numpy as np  # noqa: E402
import torch  # noqa: E402
import torch.nn.functional as F  # noqa: E402

XF_SRC = os.environ.get("XF_SRC", "/home/system/scanner/tmp_ovxfeat/xfeat_src")
sys.path.insert(0, XF_SRC)
import warnings  # noqa: E402
warnings.filterwarnings("ignore")
from modules.xfeat import XFeat  # noqa: E402
from modules.interpolator import InterpolateSparse2d  # noqa: E402


def rd(f, n):
    b = f.read(n)
    if len(b) != n:
        raise EOFError
    return b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topk", type=int, default=1024)
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=360)
    ap.add_argument("--dense", type=int, default=0)
    ap.add_argument("--min-cos", type=float, default=0.82)
    ap.add_argument("--lg-conf", type=float, default=0.1)
    ap.add_argument("--det-th", type=float, default=0.05)
    ap.add_argument("--dev", default="cuda")
    ap.add_argument("--log", default="")
    # xfopt 2026-10-27: keypoint position refinement (descriptors stay at the detected pixel; only the returned / matched
    # positions move). 0 = off (ovxfeat), 1 = 3x3 heatmap centroid at the XFeat input resolution, 2 = cv2.cornerSubPix on the
    # given (960x540) image, win 2 (5x5), moves > 1.5 px rejected (original kept)
    ap.add_argument("--subpix", type=int, default=0)
    # lgopt 2026-10-27: LightGlue adaptive width (point pruning) / depth (early exit). Defaults = LighterGlue (width 0.95,
    # depth -1). kornia prunes only when a side has more points than pruning_keypoint_thresholds[device] (cuda+flash 1536,
    # cpu -1 = always): on the PC GPU with ~1100-1250 points pruning never ran in xfopt, on a phone CPU it would.
    # --lg-prune-min N overrides that threshold (-1 = always prune, phone-like; default None = kornia table).
    ap.add_argument("--lg-width", type=float, default=0.95)
    ap.add_argument("--lg-depth", type=float, default=-1.0)
    ap.add_argument("--lg-prune-min", type=int, default=None)
    o = ap.parse_args()
    if o.subpix == 2:
        import cv2  # noqa: F401
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.use_deterministic_algorithms(True, warn_only=True)
    dev = torch.device(o.dev if torch.cuda.is_available() else "cpu")
    xf = XFeat(weights=os.path.join(XF_SRC, "weights", "xfeat.pt"), top_k=o.topk, detection_threshold=o.det_th)
    xf.dev = dev
    xf.net = xf.net.to(dev).eval()
    lg = None
    bil = InterpolateSparse2d("bicubic")
    cache = {}
    fin = sys.stdin.buffer
    fout = sys.stdout.buffer
    logf = open(o.log, "w") if o.log else None
    stats = {"F": [], "M1": [], "M2": []}
    sys.stderr.write(f"[xf_server] ready dev {dev} topk {o.topk} size {o.w}x{o.h} dense {o.dense} min_cos {o.min_cos} lg_conf {o.lg_conf}\n")
    sys.stderr.flush()
    while True:
        try:
            c = rd(fin, 1)
        except EOFError:
            break
        if c == b"Q":
            break
        if c == b"F":
            kid, W, H = struct.unpack("<qii", rd(fin, 16))
            img = np.frombuffer(rd(fin, W * H), np.uint8).reshape(H, W)
            nq, = struct.unpack("<i", rd(fin, 4))
            q = np.frombuffer(rd(fin, 8 * nq), np.float32).reshape(nq, 2) if nq else np.zeros((0, 2), np.float32)
            if dev.type == "cuda":
                torch.cuda.synchronize()
            t0 = time.perf_counter()
            with torch.inference_mode():
                x = torch.from_numpy(img.copy()).to(dev).float()[None, None] / 255.0
                Wt, Ht = (o.w or W), (o.h or H)
                if (Wt, Ht) != (W, H):
                    x = F.interpolate(x, (Ht, Wt), mode="bilinear", align_corners=False, antialias=True)
                sx, sy = W / Wt, H / Ht   # given-image pixel = XFeat-input pixel * s
                if o.dense:
                    d = xf.detectAndComputeDense(x, top_k=o.topk)
                    kp = d["keypoints"][0]
                    de = F.normalize(d["descriptors"][0], dim=-1)
                else:
                    d = xf.detectAndCompute(x, top_k=o.topk)[0]
                    kp, de = d["keypoints"], d["descriptors"]
                # descriptors at the query pixels: same dense map (single scale), same interpolation as XFeat keypoints
                xp, rh, rw = xf.preprocess_tensor(x)
                M1, K1, _ = xf.net(xp)
                M1 = F.normalize(M1, dim=1)
                _H1, _W1 = xp.shape[-2:]
                if o.subpix == 1 and not o.dense and len(kp):
                    hm = xf.get_kpts_heatmap(K1)[0, 0]              # (_H1, _W1)
                    ij = torch.round(kp / torch.tensor([rw, rh], device=dev)).long()   # heatmap pixel (exact before scale)
                    ix = ij[:, 0].clamp(1, _W1 - 2)
                    iy = ij[:, 1].clamp(1, _H1 - 2)
                    off = torch.arange(-1, 2, device=dev)
                    wy = hm[(iy[:, None, None] + off[None, :, None]), (ix[:, None, None] + off[None, None, :])]  # N,3,3
                    ws = wy.sum((1, 2)).clamp_min(1e-12)
                    dx = (wy * off[None, None, :]).sum((1, 2)) / ws
                    dy = (wy * off[None, :, None]).sum((1, 2)) / ws
                    kp = torch.stack([(ix.float() + dx) * rw, (iy.float() + dy) * rh], -1)
                if nq:
                    qt = torch.from_numpy(q.copy()).to(dev)
                    qx = torch.stack([qt[:, 0] / sx / rw, qt[:, 1] / sy / rh], -1)[None]
                    dq = F.normalize(bil(M1, qx, H=_H1, W=_W1)[0], dim=-1)
                else:
                    dq = torch.zeros((0, 64), device=dev)
                kpp = kp * torch.tensor([sx, sy], device=dev)
                if o.subpix == 2 and len(kpp):
                    import cv2
                    k0 = kpp.float().cpu().numpy().astype(np.float32).reshape(-1, 1, 2)
                    k1 = cv2.cornerSubPix(img, k0.copy(), (2, 2), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 10, 0.01))
                    mv = np.linalg.norm((k1 - k0).reshape(-1, 2), axis=1) > 1.5
                    k1[mv] = k0[mv]
                    kpp = torch.from_numpy(k1.reshape(-1, 2)).to(dev)
                pts = torch.cat([torch.from_numpy(q.copy()).to(dev), kpp], 0)
                desc = torch.cat([dq, de], 0)
            if dev.type == "cuda":
                torch.cuda.synchronize()
            ms = (time.perf_counter() - t0) * 1000
            cache[kid] = dict(pts=pts, desc=desc, size=(W, H), nq=nq)
            kpn = kpp.float().cpu().numpy().astype(np.float32)
            fout.write(struct.pack("<i", len(kpn)) + kpn.tobytes() + struct.pack("<f", ms))
            fout.flush()
            stats["F"].append(ms)
            if logf:
                logf.write(json.dumps(dict(op="F", id=kid, nq=nq, nk=len(kpn), ms=round(ms, 3))) + "\n")
        elif c == b"M":
            a, b, mode = struct.unpack("<qqi", rd(fin, 20))
            A, B = cache.get(a), cache.get(b)
            if dev.type == "cuda":
                torch.cuda.synchronize()
            t0 = time.perf_counter()
            pairs = np.zeros((0, 2), np.int32)
            sc = np.zeros((0,), np.float32)
            if A is not None and B is not None and len(A["desc"]) and len(B["desc"]):
                with torch.inference_mode():
                    if mode == 1:
                        S = A["desc"] @ B["desc"].T
                        m12 = S.argmax(1)
                        m21 = S.argmax(0)
                        i0 = torch.arange(len(m12), device=dev)
                        best = S.max(1).values
                        ok = (m21[m12] == i0) & (best >= o.min_cos)
                        pairs = torch.stack([i0[ok], m12[ok]], -1).int().cpu().numpy()
                        sc = best[ok].float().cpu().numpy()
                    else:
                        if lg is None:
                            from modules.lighterglue import LighterGlue
                            lg = LighterGlue(weights=os.path.join(XF_SRC, "weights", "xfeat-lighterglue.pt"))
                            lg.net = lg.net.to(dev).eval()
                            lg.net.conf.width_confidence = o.lg_width
                            lg.net.conf.depth_confidence = o.lg_depth
                            if o.lg_prune_min is not None:
                                lg.net.pruning_keypoint_thresholds = {k: o.lg_prune_min for k in lg.net.pruning_keypoint_thresholds}
                            sys.stderr.write(f"[xf_server] LG width {o.lg_width} depth {o.lg_depth} prune_min {lg.net.pruning_min_kpts(dev)} filter {o.lg_conf}\n")
                        WA, HA = A["size"]
                        WB, HB = B["size"]
                        out = lg({"keypoints0": A["pts"][None].float(), "keypoints1": B["pts"][None].float(),
                                  "descriptors0": A["desc"][None].float(), "descriptors1": B["desc"][None].float(),
                                  "image_size0": torch.tensor([[WA, HA]], device=dev).float(),
                                  "image_size1": torch.tensor([[WB, HB]], device=dev).float()}, min_conf=o.lg_conf)
                        m = out["matches"][0]
                        pairs = m.int().cpu().numpy().reshape(-1, 2)
                        sc = out["scores"][0].float().cpu().numpy().reshape(-1)
            if dev.type == "cuda":
                torch.cuda.synchronize()
            ms = (time.perf_counter() - t0) * 1000
            buf = struct.pack("<i", len(pairs))
            if len(pairs):
                rec = np.zeros(len(pairs), dtype=[("a", "<i4"), ("b", "<i4"), ("s", "<f4")])
                rec["a"], rec["b"], rec["s"] = pairs[:, 0], pairs[:, 1], sc
                buf += rec.tobytes()
            fout.write(buf + struct.pack("<f", ms))
            fout.flush()
            stats["M%d" % mode].append(ms)
            if logf:
                logf.write(json.dumps(dict(op="M", a=a, b=b, mode=mode, nA=len(A["desc"]) if A else 0,
                                           nB=len(B["desc"]) if B else 0, n=len(pairs), ms=round(ms, 3))) + "\n")
        else:
            sys.stderr.write(f"[xf_server] bad request {c!r}\n")
            break
    for k, v in stats.items():
        if v:
            sys.stderr.write(f"[xf_server] {k}: n {len(v)} ms p50 {np.percentile(v, 50):.2f} p95 {np.percentile(v, 95):.2f}\n")
    if logf:
        logf.close()


if __name__ == "__main__":
    main()
