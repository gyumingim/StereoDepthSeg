# ViewCrafter (512 model) on the desktop — install and run

Reinstalled 2026-11-28 (the 11-25 env was gone). Copy of this file: `~/scanner/viewcrafter_env/README.md` and
`scan3d/viewcrafter/README.md` (branch gsmap-20261125, worktree `~/scanner/StereoDepthSeg-gsmap`).

## Layout (WSL Ubuntu-22.04, user `system`)
| path | what |
|---|---|
| `~/scanner/viewcrafter_env/ViewCrafter/` | upstream https://github.com/Drexubery/ViewCrafter @ `97cdb8634b28708cbd50912d57c52f23c5ffbf09` + `env/viewcrafter_local.patch` (xformers -> torch SDPA shim) |
| `~/scanner/viewcrafter_env/venv/` | uv venv, CPython 3.12.15, torch 2.11.0+cu128 (sm_120 in arch list), torchvision 0.26.0+cu128, pytorch3d 0.7.9 (source build, commit 33824be, sm_120), open-clip-torch 2.17.1, pytorch-lightning 1.9.5, transformers 4.40.2. Exact list = `env/requirements_freeze.txt` |
| `~/scanner/viewcrafter_env/pytorch3d/` | pytorch3d source + build dir (CUDA 12.8 toolkit from `~/scanner/gsplat_env/cuda128`) |
| `ViewCrafter/checkpoints/model_512.ckpt` | HF `Drexubery/ViewCrafter_25_512` `model.ckpt` (repo sha 8db619b), 10,437,386,907 B, sha256 `e15528ca4fade935222ba159356708294bf788c38e22b366597701372ed7f8df` (= HF LFS oid), Apache-2.0 |
| `ViewCrafter/checkpoints/dust3r_hf/DUSt3R_512_dpt_from_hf.pth` | HF `naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt` `model.safetensors` (repo sha 61c5744, sha256 `7c300a89534113436bde52732d3151212bcbd90f0aa3c8d1496f86d84bfe4b42` = HF LFS oid) converted by `env/hf_dust3r_to_pth.py` (all keys matched); pth sha256 `e4896c81ad881b3b4812b9bbcd20c968ed11ff0d4da598f5a3897868cabd6f8f`. Safetensors deleted after conversion. CC BY-NC-SA 4.0. Only needed for `--pc dust3r` |
| `~/.cache/huggingface/hub/models--laion--CLIP-ViT-H-14-laion2B-s32B-b79K` | open_clip ViT-H-14 (shared cache, already present) |
| `~/scanner/viewcrafter_env/work/` | outputs (not in git) |
| `~/scanner/viewcrafter_env/logs/` | install logs + the install scripts as run |

Not installed: ViewCrafter_25_sparse / 1024 models (official 23.5 GB VRAM, do not fit 16 GB as is). Disk: env 21 GB.

## Reinstall from scratch (~15 min on 2026-11-28, mostly the 10.4 GB download)
```
bash scan3d/viewcrafter/env/install1.sh   # clone @97cdb86 + patch, venv, torch cu128, freeze (--no-deps)
bash scan3d/viewcrafter/env/dl.sh         # weights + HF API json (sha256) into ~/scanner/viewcrafter_env/ckpt_dl
bash scan3d/viewcrafter/env/install2.sh   # pytorch3d build (build_p3d.sh) + move weights + DUSt3R conversion
```
Gotchas: freeze needs `--no-deps` (protobuf 3.20.3 vs tensorboard 2.21 resolver conflict); open-clip-torch must stay 2.17.1;
the SDPA shim must get 4-D tensors (else math-kernel OOM); torch >= 2.6 `weights_only` is handled inside vc_run.py.

## Run
```
cd ~/scanner/viewcrafter_env/ViewCrafter
../venv/bin/python ~/scanner/StereoDepthSeg-gsmap/scan3d/viewcrafter/vc_run.py --traj TRAJ.npz --dump DUMP --fuse FUSE \
    --mode single|sparse --pc ours|dust3r --out OUT [--seed 123] [--steps 50]
```
Smoke test: `bash scan3d/viewcrafter/run_smoke.sh` (one clip, 165506 kf 540, our FFS+DA3 point cloud, 60° side orbit).
Uses `scan3d/gsplat/gpu_guard.py` (gpu.lock, torch cap SCAN3D_GPU_FRAC=0.85). One heavy job at a time.

## Smoke result 2026-11-28 11:32-11:34 (n=1, seed 123)
model load 60 s, diffusion 41.0 s (50 DDIM), total 112 s (wall 1:56); peak nvidia-smi 10,890 MiB, torch alloc 9.97 GB;
25 x 320x512 frames, output `~/scanner/viewcrafter_env/work/smoke_165506_single_ours/diffusion.mp4`. Same as 11-25 (10.9 GB / 41 s).
Log notes: "cannot find cuda-compiled version of RoPE2D" = DUSt3R's optional kernel, harmless here (no DUSt3R in this run).
