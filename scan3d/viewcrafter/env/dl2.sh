#!/bin/bash
# open_clip ViT-H-14 (laion2b_s32b_b79k) weights into the HF cache + DUSt3R safetensors from HF (faster mirror)
cd /home/system/scanner/viewcrafter_env || exit 1
date
venv/bin/python -c "
import open_clip
for a in ['ViT-H-14']:
    m,_,_ = open_clip.create_model_and_transforms(a, device='cpu', pretrained='laion2b_s32b_b79k'); print('clip ok', a)
"
date
mkdir -p ViewCrafter/checkpoints/dust3r_hf
wget -q -c -O ViewCrafter/checkpoints/dust3r_hf/model.safetensors https://huggingface.co/naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt/resolve/main/model.safetensors && echo "dust3r hf ok $(date)"
wget -q -O ViewCrafter/checkpoints/dust3r_hf/config.json https://huggingface.co/naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt/resolve/main/config.json
date
