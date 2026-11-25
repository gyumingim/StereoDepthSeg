#!/bin/bash
# download ViewCrafter weights from Hugging Face (own folder only)
cd /home/system/scanner/viewcrafter_env/ViewCrafter/checkpoints || exit 1
date
wget -q -O README_512.md https://huggingface.co/Drexubery/ViewCrafter_25_512/raw/main/README.md
wget -q -O README_sparse.md https://huggingface.co/Drexubery/ViewCrafter_25_sparse/raw/main/README.md
wget -q -O README_dust3r_hf.md https://huggingface.co/naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt/raw/main/README.md
wget -q -c -O model_512.ckpt https://huggingface.co/Drexubery/ViewCrafter_25_512/resolve/main/model.ckpt && echo "m512 ok $(date)"
wget -q -c -O model_sparse.ckpt https://huggingface.co/Drexubery/ViewCrafter_25_sparse/resolve/main/model_sparse.ckpt && echo "sparse ok $(date)"
ls -la
date
