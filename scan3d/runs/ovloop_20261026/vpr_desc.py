#!/usr/bin/env python3
"""Learned VPR global descriptors (BoQ-DINOv2 / SALAD-DINOv2) for ovloop bridge.

usage:
  venv_ffs/bin/python vpr_desc.py --model boq|salad --in DIR --out FILE.ovpr
                                  [--size HxW] [--batch 8] [--fp16] [--timing-json F]
DIR: kf_<t_ns>.pgm (8-bit gray) files (+ index.csv, not required).
Output (little-endian): b"OVPR", uint32 dim, then per image (sorted by t_ns):
  int64 t_ns, float32[dim]  (L2-normalised).
Sources are pinned local clones in ./vpr_src (see README_vpr.md).
"""
import argparse, glob, json, os, re, struct, sys, time

# venv_ffs has xformers whose kernels do not support sm_120 (RTX 5080) / fp32;
# DINOv2 hub code then falls back to plain PyTorch attention.
os.environ.setdefault("XFORMERS_DISABLED", "1")

import cv2
import numpy as np
import torch
import torch.nn.functional as F

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "vpr_src")
MEAN = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
STD = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)

BOQ_URL = "https://github.com/amaralibey/Bag-of-Queries/releases/download/v1.0/dinov2_12288.pth"
SALAD_URL = "https://github.com/serizba/salad/releases/download/v1.0.0/dino_salad.ckpt"


def weights_file(url):
    d = os.path.join(torch.hub.get_dir(), "checkpoints")
    return os.path.join(d, os.path.basename(url))


def build_boq():
    sys.path.insert(0, os.path.join(SRC, "Bag-of-Queries", "src"))
    from backbones import DinoV2
    from boq import BoQ

    class VPRModel(torch.nn.Module):  # same as Bag-of-Queries/hubconf.py
        def __init__(self, backbone, aggregator):
            super().__init__()
            self.backbone = backbone
            self.aggregator = aggregator

        def forward(self, x):
            x = self.backbone(x)
            x, _ = self.aggregator(x)
            return x

    bb = DinoV2()
    agg = BoQ(in_channels=bb.out_channels, proj_channels=384, num_queries=64,
              num_layers=2, row_dim=12288 // 384)
    m = VPRModel(bb, agg)
    sd = torch.hub.load_state_dict_from_url(BOQ_URL, map_location="cpu")
    m.load_state_dict(sd)  # strict
    # official recipe: ToTensor -> Resize(322,322) BICUBIC antialias -> ImageNet norm
    return m, "bicubic", (322, 322), weights_file(BOQ_URL)


def build_salad():
    # avoid salad/vpr_model.py (needs pytorch_lightning); rebuild the same modules
    sys.path.insert(0, os.path.join(SRC, "salad"))
    from models import helper

    class VPRModel(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.backbone = helper.get_backbone(
                "dinov2_vitb14", {"num_trainable_blocks": 4, "return_token": True, "norm_layer": True})
            self.aggregator = helper.get_aggregator(
                "SALAD", {"num_channels": 768, "num_clusters": 64, "cluster_dim": 128, "token_dim": 256})

        def forward(self, x):
            return self.aggregator(self.backbone(x))

    m = VPRModel()
    sd = torch.hub.load_state_dict_from_url(SALAD_URL, map_location="cpu")
    if "state_dict" in sd:
        sd = sd["state_dict"]
    res = m.load_state_dict(sd, strict=False)
    if res.missing_keys:
        raise RuntimeError(f"SALAD missing keys: {res.missing_keys[:10]}")
    if res.unexpected_keys:
        print(f"[salad] ignored unexpected keys: {res.unexpected_keys[:10]}", file=sys.stderr)
    # official eval.py: Resize(322,322) BILINEAR -> ImageNet norm
    return m, "bilinear", (322, 322), weights_file(SALAD_URL)


def list_frames(d):
    out = []
    for p in glob.glob(os.path.join(d, "kf_*.pgm")):
        mt = re.match(r"kf_(-?\d+)\.pgm$", os.path.basename(p))
        if mt:
            out.append((int(mt.group(1)), p))
    out.sort()
    return out


def preprocess(grays, size, mode, dev):
    x = torch.from_numpy(np.stack(grays)).to(dev).float().div_(255.0)  # B,H,W
    x = x.unsqueeze(1).expand(-1, 3, -1, -1)  # gray -> RGB (R=G=B)
    x = F.interpolate(x, size=size, mode=mode, align_corners=False, antialias=True)
    if mode == "bicubic":
        x = x.clamp_(0.0, 1.0)  # torchvision tensor bicubic also clamps for float? keep in [0,1]
    return (x - MEAN.to(dev)) / STD.to(dev)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, choices=["boq", "salad"])
    ap.add_argument("--in", dest="inp", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--size", default=None, help="HxW, multiples of 14 (default 322x322 = official)")
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--fp16", action="store_true", help="autocast fp16 (default fp32)")
    ap.add_argument("--timing-json", default=None)
    a = ap.parse_args()

    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    t0 = time.time()
    model, mode, size, wfile = (build_boq if a.model == "boq" else build_salad)()
    model = model.eval().to(dev)
    if a.size:
        h, w = (int(v) for v in a.size.lower().split("x"))
        if h % 14 or w % 14:
            sys.exit("--size must be multiples of 14")
        size = (h, w)
    nparam = sum(p.numel() for p in model.parameters())
    t_load = time.time() - t0

    frames = list_frames(a.inp)
    if not frames:
        sys.exit(f"no kf_*.pgm in {a.inp}")

    def run(batch_imgs):
        x = preprocess(batch_imgs, size, mode, dev)
        with torch.no_grad(), torch.autocast(dev.type, dtype=torch.float16, enabled=a.fp16):
            d = model(x)
        return F.normalize(d.float(), p=2, dim=1)

    # warm-up (2 passes on first batch)
    first = [cv2.imread(p, cv2.IMREAD_GRAYSCALE) for _, p in frames[: a.batch]]
    for _ in range(2):
        run(first)
    if dev.type == "cuda":
        torch.cuda.synchronize()

    descs, gpu_t = [], 0.0
    for i in range(0, len(frames), a.batch):
        imgs = []
        for _, p in frames[i:i + a.batch]:
            g = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
            if g is None:
                sys.exit(f"cannot read {p}")
            imgs.append(g)
        if dev.type == "cuda":
            torch.cuda.synchronize()
        ts = time.perf_counter()
        d = run(imgs)
        if dev.type == "cuda":
            torch.cuda.synchronize()
        gpu_t += time.perf_counter() - ts
        descs.append(d.cpu().numpy().astype("<f4"))
    D = np.concatenate(descs, 0)
    dim = D.shape[1]

    with open(a.out, "wb") as f:
        f.write(b"OVPR")
        f.write(struct.pack("<I", dim))
        for (t, _), v in zip(frames, D):
            f.write(struct.pack("<q", t))
            f.write(v.tobytes())

    info = {
        "model": a.model, "dim": int(dim), "n_images": len(frames), "input_size_hw": list(size),
        "resize_mode": mode, "fp16": a.fp16, "batch": a.batch, "device": str(dev),
        "gpu_name": torch.cuda.get_device_name(0) if dev.type == "cuda" else None,
        "ms_per_image_gpu_incl_preproc": 1000.0 * gpu_t / len(frames),
        "params": int(nparam), "weights_file": wfile,
        "weights_bytes": os.path.getsize(wfile) if os.path.exists(wfile) else None,
        "peak_gpu_mem_MB": (torch.cuda.max_memory_allocated() / 2**20) if dev.type == "cuda" else None,
        "load_s": t_load, "out": os.path.abspath(a.out), "input_dir": os.path.abspath(a.inp),
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    print(json.dumps(info, indent=1))
    if a.timing_json:
        with open(a.timing_json, "w") as f:
            json.dump(info, f, indent=1)


if __name__ == "__main__":
    main()
