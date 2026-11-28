#!/bin/bash
# ViewCrafter reinstall 2026-11-28: weights (512 model + DUSt3R HF safetensors). 1024/sparse model NOT downloaded (lean).
C=/home/system/scanner/viewcrafter_env/ckpt_dl
mkdir -p $C && cd $C || exit 1
date
wget -q -O README_512.md https://huggingface.co/Drexubery/ViewCrafter_25_512/raw/main/README.md
wget -q -O README_dust3r_hf.md https://huggingface.co/naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt/raw/main/README.md
wget -q -O dust3r_config.json https://huggingface.co/naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt/resolve/main/config.json
# HF API: file listing with sha256 (lfs oid) for verification
wget -q -O api_512.json "https://huggingface.co/api/models/Drexubery/ViewCrafter_25_512?blobs=true"
wget -q -O api_dust3r.json "https://huggingface.co/api/models/naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt?blobs=true"
wget -q -c -O model_512.ckpt https://huggingface.co/Drexubery/ViewCrafter_25_512/resolve/main/model.ckpt && echo "m512 ok $(date)"
wget -q -c -O dust3r_model.safetensors https://huggingface.co/naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt/resolve/main/model.safetensors && echo "dust3r ok $(date)"
ls -la
sha256sum model_512.ckpt dust3r_model.safetensors | tee sha256.txt
date
echo DL_DONE
