"""Convert naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt model.safetensors (HF mirror) into the .pth layout that
dust3r.inference.load_model expects ({'args': Namespace(model=<ctor string>), 'model': state_dict}).
Run from the ViewCrafter folder: ../venv/bin/python ../hf_dust3r_to_pth.py"""
import argparse, sys
sys.path.insert(0, "extern/dust3r")
import torch
from safetensors.torch import load_file
from dust3r.model import AsymmetricCroCo3DStereo  # noqa: F401
from math import inf  # noqa: F401

S = ("AsymmetricCroCo3DStereo(pos_embed='RoPE100', patch_embed_cls='ManyAR_PatchEmbed', img_size=(512, 512), head_type='dpt', "
     "output_mode='pts3d', depth_mode=('exp', -inf, inf), conf_mode=('exp', 1, inf), enc_embed_dim=1024, enc_depth=24, "
     "enc_num_heads=16, dec_embed_dim=768, dec_depth=12, dec_num_heads=12)")
sd = load_file("checkpoints/dust3r_hf/model.safetensors")
# HF export dropped the aliased ModuleList names (scratch.layer_rn[i] is scratch.layer{i+1}_rn, same tensor)
for h in (1, 2):
    for i in range(4):
        sd[f"downstream_head{h}.dpt.scratch.layer_rn.{i}.weight"] = sd[f"downstream_head{h}.dpt.scratch.layer{i + 1}_rn.weight"]
net = eval(S.replace("ManyAR_PatchEmbed", "PatchEmbedDust3R"))
print(net.load_state_dict(sd, strict=True))
torch.save({"args": argparse.Namespace(model=S), "model": sd}, "checkpoints/dust3r_hf/DUSt3R_512_dpt_from_hf.pth")
print("saved")
