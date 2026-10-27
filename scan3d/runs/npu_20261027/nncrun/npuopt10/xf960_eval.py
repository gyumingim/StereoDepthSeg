#!/usr/bin/env python3
"""npuopt 10-28: XFeat 960x544 (VIO res, 540->544 bottom 4 rows edge-replicated) vs 640x352 on the 13 eval pairs
(eval frame + 30 frames later; frames grabbed at 960x540 from cam5.mp4 exactly like make_calib.py).
PC fp32 official XFeatModel (InstanceNorm inside) + official post (NMS 5 thr 0.05, score nearest K * bilinear H, top-k,
bicubic desc + L2), mutual NN cos > 0.82, MAGSAC F in 960x540 pixels.
Writes OUT/xf960_eval.json, OUT/in960_norm.npy (13 frames, normalised padded input for the ONNX / phone),
OUT/ref960_out.npy (official raw outputs: feats|kpt logits|heat logit-> here heat after sigmoid is stored separately).
venv_ffs python:  python xf960_eval.py OUT"""
import json, os, sys, time
import cv2, numpy as np, torch, torch.nn.functional as F
XF = "/home/system/scanner/tmp_ovxfeat/xfeat_src"; sys.path.insert(0, XF)
from modules.model import XFeatModel
from modules.xfeat import XFeat
from modules.interpolator import InterpolateSparse2d
torch.set_grad_enabled(False); torch.set_num_threads(8)
OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
R = "/home/system/scanner/StereoDepthSeg/scan3d/recordings"
D = "/home/system/scanner/tmp_npu/data"
net = XFeatModel().eval(); net.load_state_dict(torch.load(f"{XF}/weights/xfeat.pt", map_location="cpu"))
xf = XFeat(weights=None); xf.dev = torch.device("cpu")
near, bil, bic = InterpolateSparse2d("nearest"), InterpolateSparse2d("bilinear"), InterpolateSparse2d("bicubic")
names = open(f"{D}/eval_list.txt").read().split("eval:\n")[1].split()
if os.environ.get("XF960_MODE") == "long":   # pre-registered supplement 2: 5 long moving recordings x 9 points, +30 frames
    names = []
    for rec in ("hs_20261024_142148_std60", "hs_20261024_142610_std60", "hs_20261024_151646_std60", "hs_20261024_165506_std60", "hs_20261024_173432_std60"):
        nfr = int(cv2.VideoCapture(f"{R}/{rec}/cam5.mp4").get(cv2.CAP_PROP_FRAME_COUNT))
        names += [f"{rec}#{int(q * nfr)}" for q in np.linspace(0.1, 0.9, 9)]


RMP = "/home/system/scanner/StereoDepthSeg-lens05/scan3d/config/phone_ov_main/c4f60/remap_1920x1080.bin"   # app file (live c4f60)
_b = open(RMP, "rb").read(); _vw, _vh = np.frombuffer(_b, np.int32, 2, 0)
MX = np.frombuffer(_b, np.float32, _vw * _vh, 8).reshape(_vh, _vw); MY = np.frombuffer(_b, np.float32, _vw * _vh, 8 + 4 * _vw * _vh).reshape(_vh, _vw)
SRC_SZ = tuple(np.frombuffer(_b, np.int32, 2, 8 + 8 * _vw * _vh)); assert (_vw, _vh) == (960, 540) and SRC_SZ == (1920, 1080), (_vw, _vh, SRC_SZ)


def grab(rec, fi):
    """live path: 1920x1080 BGR -> GRAY -> remap INTER_LINEAR (app c4f60 maps) -> 960x540 undistorted"""
    cap = cv2.VideoCapture(f"{R}/{rec}/cam5.mp4"); cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
    ok, fr = cap.read(); assert ok, (rec, fi)
    src = fr.shape[:2]; assert src == (1080, 1920), src
    return cv2.remap(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), MX, MY, cv2.INTER_LINEAR), src


def official(feats, kl, heat, H, W, n, ymax=None):
    Kh = xf.get_kpts_heatmap(kl)
    mk = xf.NMS(Kh, threshold=0.05, kernel_size=5)
    sc = (near(Kh, mk, H, W) * bil(heat, mk, H, W)).squeeze(-1)
    sc[torch.all(mk == 0, dim=-1)] = -1
    if ymax is not None:
        sc[mk[..., 1] >= ymax] = -1
    idx = torch.argsort(-sc)[:, :n]
    mk = torch.stack([torch.gather(mk[..., 0], -1, idx), torch.gather(mk[..., 1], -1, idx)], -1)
    s = torch.gather(sc, -1, idx)
    de = F.normalize(bic(F.normalize(feats, dim=1), mk, H, W), dim=-1)
    v = s[0] > 0
    return mk[0][v].float().numpy(), de[0][v].numpy()


def run960(g):
    x = cv2.copyMakeBorder(g, 0, 4, 0, 0, cv2.BORDER_REPLICATE).astype(np.float32)[None, None] / 255.0
    f, k, h = net(torch.from_numpy(x))
    xn = (x - x.mean()) / np.sqrt(x.var() + 1e-5)
    return (f, k, h), xn.astype(np.float32)


def run640(g):
    x = cv2.resize(g, (640, 352), interpolation=cv2.INTER_AREA).astype(np.float32)[None, None] / 255.0
    return net(torch.from_numpy(x)), x


def kp960(o, n):
    return official(*o, 544, 960, n, ymax=540)


def kp640(o, n):
    k, d = official(*o, 352, 640, n)
    k = k.copy(); k[:, 0] = (k[:, 0] + 0.5) * 1.5 - 0.5; k[:, 1] = (k[:, 1] + 0.5) * (540 / 352) - 0.5
    return k, d


def sampson(Fm, a, b):
    a1 = np.c_[a, np.ones(len(a))]; b1 = np.c_[b, np.ones(len(b))]
    Fa = a1 @ Fm.T; Ftb = b1 @ Fm
    num = (b1 * Fa).sum(1) ** 2
    return np.sqrt(num / (Fa[:, 0] ** 2 + Fa[:, 1] ** 2 + Ftb[:, 0] ** 2 + Ftb[:, 1] ** 2))


def geo(k0, d0, k1, d1):
    i0, i1 = xf.match(torch.from_numpy(d0), torch.from_numpy(d1), min_cossim=0.82)
    a, b = k0[i0.numpy()].astype(np.float64), k1[i1.numpy()].astype(np.float64)
    r = {"n_kp0": len(k0), "n_match": len(a), "disp_med": float(np.median(np.linalg.norm(a - b, axis=1))) if len(a) else 0.0}
    if len(a) < 15:
        return r
    cv2.setRNGSeed(0)
    F3, m3 = cv2.findFundamentalMat(a, b, cv2.USAC_MAGSAC, 3.0, 0.9999, 20000)
    cv2.setRNGSeed(0)
    F1, m1 = cv2.findFundamentalMat(a, b, cv2.USAC_MAGSAC, 1.0, 0.9999, 20000)
    s3 = sampson(F3, a, b); in3 = s3 < 3.0
    r.update(sampson_med_in3=float(np.median(s3[in3])), n_in3=int(in3.sum()),
             n_in1=int((sampson(F1, a, b) < 1.0).sum()), inl_ratio1=float((sampson(F1, a, b) < 1.0).mean()))
    return r


t0 = time.time()
GAPS = (30,) if os.environ.get("XF960_MODE") == "long" else (30, 180)
rows, ins, outs, srcs = [], [], [], []
for i, nm in enumerate(names):
    rec, fi = nm.split("#"); fi = int(fi)
    g0, s0 = grab(rec, fi); srcs.append(list(s0))
    o0, xn0 = run960(g0); p0, _ = run640(g0)
    ins.append(xn0[0]); outs.append(torch.cat([o0[0], o0[1], o0[2]], 1)[0].numpy())
    row = {"name": nm}
    for gap in GAPS:
        nfr = int(cv2.VideoCapture(f"{R}/{rec}/cam5.mp4").get(cv2.CAP_PROP_FRAME_COUNT))
        f1 = fi + gap if fi + gap < nfr - 1 else (fi - gap if fi - gap >= 0 else (nfr - 2 if nfr - 2 - fi >= fi else 0))
        row[f"g{gap}_frame1"] = [f1, nfr]
        g1, _ = grab(rec, f1); o1, _ = run960(g1); p1, _ = run640(g1)
        for n in (1024, 2048):
            row[f"g{gap}_r960_{n}"] = geo(*kp960(o0, n), *kp960(o1, n))
            row[f"g{gap}_r640_{n}"] = geo(*kp640(p0, n), *kp640(p1, n))
    rows.append(row); print(nm, json.dumps({k: (round(v.get("sampson_med_in3", -1), 4), v.get("n_in1"), round(v["disp_med"], 1)) for k, v in row.items() if isinstance(v, dict)}), flush=True)
if not os.environ.get("XF960_MODE"): np.save(f"{OUT}/in960_norm.npy", np.stack(ins)); np.save(f"{OUT}/ref960_out_sigheat.npy", np.stack(outs))
summ = {}
for gap in GAPS:
    for n in (1024, 2048):
        a = [r[f"g{gap}_r960_{n}"] for r in rows]; b = [r[f"g{gap}_r640_{n}"] for r in rows]
        ok = [i for i in range(len(rows)) if "sampson_med_in3" in a[i] and "sampson_med_in3" in b[i] and min(a[i]["disp_med"], b[i]["disp_med"]) >= 5.0]
        med = lambda L, k: float(np.median([L[i][k] for i in ok])) if ok else None
        summ[f"gap{gap}_top{n}"] = {"pairs_valid_moving": len(ok), "pairs_excluded": len(rows) - len(ok),
                        "pairs_960_lower_sampson": int(sum(a[i]["sampson_med_in3"] < b[i]["sampson_med_in3"] for i in ok)),
                        "sampson_med_in3_960": med(a, "sampson_med_in3"), "sampson_med_in3_640": med(b, "sampson_med_in3"),
                        "n_in1_960": med(a, "n_in1"), "n_in1_640": med(b, "n_in1"),
                        "inl_ratio1_960": med(a, "inl_ratio1"), "inl_ratio1_640": med(b, "inl_ratio1"),
                        "n_match_960": med(a, "n_match"), "n_match_640": med(b, "n_match"), "disp_med_960": med(a, "disp_med")}
res = {"made": time.strftime("%F %T"), "sec": round(time.time() - t0, 1), "src_video_hw": srcs, "remap": RMP,
       "summary": summ, "rows": rows,
       "notes": "PC fp32 official XFeatModel; frames = live path (gray -> c4f60 remap -> 960x540); 960 = + 4 bottom rows replicated, kps y>=540 dropped; "
                "640 = INTER_AREA 960x540->640x352, coords scaled to 960x540; MAGSAC seed 0; pairs with match disp median < 5 px excluded"}
json.dump(res, open(f"{OUT}/xf960_eval{os.environ.get('XF960_MODE', '')}.json", "w"), indent=1)
print(json.dumps(summ, indent=1))
