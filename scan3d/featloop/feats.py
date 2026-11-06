"""featloop 2026-11-05: XFeat + LighterGlue (PyTorch, PC GPU) and SALAD global descriptors for kf_ba.py.

Sources (pinned, downloaded to ~/scanner/featloop_ext, not committed):
  XFeat / LighterGlue: github.com/verlab/accelerated_features e92685f (Apache-2.0), weights/xfeat.pt sha256 0f5187fd...,
    weights/xfeat-lighterglue.pt sha256 766102df...; LightGlue code = kornia (venv_ffs kornia 0.8.3).
  SALAD: github.com/serizba/salad 6aede13 (GPL-3.0) models/aggregators/salad.py + models/backbones/dinov2.py,
    weights dino_salad.ckpt (torch hub cache), DINOv2 vitb14 from the local hub cache (facebookresearch_dinov2_main).
All functions run inside the caller's gpu_guard job (torch memory cap set by the caller).
"""
import os
import sys
import time

import numpy as np

EXT = os.environ.get("FEATLOOP_EXT", os.path.expanduser("~/scanner/featloop_ext"))


def _torch():
    import torch
    torch.backends.cudnn.benchmark = False
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    return torch


class XFLG:
    """XFeat sparse keypoints (top-k) + LighterGlue matcher. Images: 8-bit gray, any size; padded at the bottom/right
    (last row/column repeated) to a multiple of 32 so XFeat does not resample (keypoints in the pad dropped)."""

    def __init__(self, topk=4096, det_th=0.05, lg_conf=0.1, subpix=0):
        torch = _torch()
        self.torch = torch
        sys.path.insert(0, os.path.join(EXT, "xf"))
        import warnings
        warnings.filterwarnings("ignore")
        from modules.xfeat import XFeat
        from modules.lighterglue import LighterGlue
        self.dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.xf = XFeat(weights=os.path.join(EXT, "xf", "weights", "xfeat.pt"), top_k=topk, detection_threshold=det_th)
        self.xf.dev = self.dev
        self.xf.net = self.xf.net.to(self.dev).eval()
        self.lg = LighterGlue(weights=os.path.join(EXT, "xf", "weights", "xfeat-lighterglue.pt"))
        self.lg.net.eval()
        self.topk, self.lg_conf, self.subpix = topk, lg_conf, subpix
        self.ms_f, self.ms_m = [], []

    def extract(self, gray):
        torch = self.torch
        H, W = gray.shape
        t0 = time.perf_counter()
        with torch.inference_mode():
            x = torch.from_numpy(gray.copy()).to(self.dev).float()[None, None] / 255.0
            Hp = ((H + 31) // 32) * 32
            if Hp != H:
                x = torch.cat([x, x[:, :, -1:, :].expand(1, 1, Hp - H, W)], 2)
            Wp = ((W + 31) // 32) * 32
            if Wp != W:
                x = torch.cat([x, x[:, :, :, -1:].expand(1, 1, x.shape[2], Wp - W)], 3)
            d = self.xf.detectAndCompute(x, top_k=self.topk + 256)[0]
            keep = (d["keypoints"][:, 1] < H) & (d["keypoints"][:, 0] < W)
            kp, de, sc = d["keypoints"][keep][:self.topk], d["descriptors"][keep][:self.topk], d["scores"][keep][:self.topk]
            if self.subpix == 1 and len(kp):          # 3x3 heatmap centroid (xfopt --subpix 1); descriptors unchanged
                xp, rh, rw = self.xf.preprocess_tensor(x)
                _, K1, _ = self.xf.net(xp)
                hm = self.xf.get_kpts_heatmap(K1)[0, 0]
                H1, W1 = hm.shape
                ix = torch.round(kp[:, 0] / rw).long().clamp(1, W1 - 2)
                iy = torch.round(kp[:, 1] / rh).long().clamp(1, H1 - 2)
                off = torch.arange(-1, 2, device=self.dev)
                wy = hm[(iy[:, None, None] + off[None, :, None]), (ix[:, None, None] + off[None, None, :])]
                ws = wy.sum((1, 2)).clamp_min(1e-12)
                dx = (wy * off[None, None, :]).sum((1, 2)) / ws
                dy = (wy * off[None, :, None]).sum((1, 2)) / ws
                kp = torch.stack([(ix.float() + dx) * rw, (iy.float() + dy) * rh], -1)
        if self.dev.type == "cuda":
            torch.cuda.synchronize()
        self.ms_f.append((time.perf_counter() - t0) * 1000)
        return dict(kp=kp.float().cpu().numpy(), desc=de.float().cpu(), score=sc.float().cpu().numpy(), size=(W, H))

    def match(self, A, B):
        torch = self.torch
        t0 = time.perf_counter()
        if len(A["kp"]) < 2 or len(B["kp"]) < 2:
            return np.zeros((0, 2), np.uint32)
        with torch.inference_mode():
            out = self.lg({"keypoints0": torch.from_numpy(A["kp"])[None].to(self.dev),
                           "keypoints1": torch.from_numpy(B["kp"])[None].to(self.dev),
                           "descriptors0": A["desc"][None].to(self.dev), "descriptors1": B["desc"][None].to(self.dev),
                           "image_size0": torch.tensor([A["size"]], device=self.dev).float(),
                           "image_size1": torch.tensor([B["size"]], device=self.dev).float()}, min_conf=self.lg_conf)
            m = out["matches"][0].int().cpu().numpy().reshape(-1, 2).astype(np.uint32)
        if self.dev.type == "cuda":
            torch.cuda.synchronize()
        self.ms_m.append((time.perf_counter() - t0) * 1000)
        return m


def salad_descriptors(gray_list, batch=8):
    """SALAD (DINOv2-B/14) global descriptors, 322x322 bilinear (official eval), gray -> 3 channels, L2-normalised."""
    torch = _torch()
    F = torch.nn.functional
    os.environ.setdefault("XFORMERS_DISABLED", "1")
    hub = os.path.join(torch.hub.get_dir(), "facebookresearch_dinov2_main")
    _load = torch.hub.load
    torch.hub.load = lambda repo, name, *a, **k: _load(hub, name, source="local", pretrained=True)   # no network
    import importlib.util
    spec = importlib.util.spec_from_file_location("salad_agg", os.path.join(EXT, "salad", "models", "aggregators", "salad.py"))
    agg_mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(agg_mod)
    spec = importlib.util.spec_from_file_location("salad_bb", os.path.join(EXT, "salad", "models", "backbones", "dinov2.py"))
    bb_mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(bb_mod)

    class VPR(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.backbone = bb_mod.DINOv2(model_name="dinov2_vitb14", num_trainable_blocks=4, return_token=True, norm_layer=True)
            self.aggregator = agg_mod.SALAD(num_channels=768, num_clusters=64, cluster_dim=128, token_dim=256)

        def forward(self, x):
            return self.aggregator(self.backbone(x))
    try:
        m = VPR()
    finally:
        torch.hub.load = _load
    sd = torch.load(os.path.join(torch.hub.get_dir(), "checkpoints", "dino_salad.ckpt"), map_location="cpu")
    sd = sd.get("state_dict", sd)
    res = m.load_state_dict(sd, strict=False)
    if res.missing_keys:
        raise RuntimeError(f"SALAD missing keys {res.missing_keys[:5]}")
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    m = m.eval().to(dev)
    mean = torch.tensor([0.485, 0.456, 0.406], device=dev).view(1, 3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225], device=dev).view(1, 3, 1, 1)
    out = []
    with torch.inference_mode():
        for i in range(0, len(gray_list), batch):
            x = torch.from_numpy(np.stack(gray_list[i:i + batch])).to(dev).float().div_(255.0)[:, None].expand(-1, 3, -1, -1)
            x = F.interpolate(x, size=(322, 322), mode="bilinear", align_corners=False, antialias=True)
            out.append(F.normalize(m((x - mean) / std).float(), dim=1).cpu().numpy())
    del m
    if dev.type == "cuda":
        torch.cuda.empty_cache()
    return np.concatenate(out, 0)
