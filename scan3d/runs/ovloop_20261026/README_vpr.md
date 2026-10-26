# vpr_desc.py — learned VPR global descriptors (BoQ / SALAD, DINOv2 ViT-B/14)

Made 2026-10-26 (ovloop-vpr helper). PC GPU (RTX 5080, WSL Ubuntu-22.04), python = `StereoDepthSeg/venv_ffs` (torch 2.11+cu128, torchvision 0.26, cv2, timm). Nothing pip-installed; no extra venv needed.

## Run
```bash
P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
$P vpr_desc.py --model boq   --in DIR --out boq.ovpr   [--size 322x322] [--batch 8] [--fp16] [--timing-json t.json]
$P vpr_desc.py --model salad --in DIR --out salad.ovpr [--size 322x322] [--batch 8] [--fp16]
```
- DIR: `kf_<t_ns>.pgm` (8-bit gray). `index.csv` is not read; t_ns comes from the file name.
- Preprocess: gray -> R=G=B, /255, resize on GPU (antialias) to 322x322 (official default for both; `--size HxW` multiples of 14, e.g. 322x574 keeps 16:9), ImageNet mean/std, eval + no_grad, L2-normalise.
  BoQ = bicubic (README recipe), SALAD = bilinear (eval.py). Default fp32; `--fp16` = autocast (cos(fp32,fp16) = 1.0000 on test).
- Output (little-endian): `"OVPR"`, uint32 dim, then per image sorted by t_ns: int64 t_ns, float32[dim]. Timing JSON (created time, input dir, sizes) printed and optionally saved.
- `XFORMERS_DISABLED=1` is set inside the script: venv_ffs has xformers whose kernels do not support sm_120 / fp32 -> DINOv2 uses plain PyTorch attention.
- SALAD is rebuilt from `salad/models` without `vpr_model.py` (that needs pytorch_lightning); same modules/args as hubconf.py, state dict loads with no missing keys.

## Sources (pinned clones in `vpr_src/`)
| item | URL | commit | license |
|---|---|---|---|
| BoQ code | https://github.com/amaralibey/Bag-of-Queries | 1a4965ea7dfd9bd0dd846adf7a0e430f68101d12 | MIT |
| BoQ weights | https://github.com/amaralibey/Bag-of-Queries/releases/download/v1.0/dinov2_12288.pth | release v1.0 | MIT (repo) |
| SALAD code | https://github.com/serizba/salad | 6aede13a3f6c25750bf7fde10209c06cb73060bb | **GPL-3.0** |
| SALAD weights | https://github.com/serizba/salad/releases/download/v1.0.0/dino_salad.ckpt | release v1.0.0 | GPL-3.0 (repo) |
| DINOv2 code (torch.hub main) | https://github.com/facebookresearch/dinov2 | 7764ea0f912e53c92e82eb78a2a1631e92725fc8 (HEAD 2026-10-26) | Apache-2.0 |
| DINOv2 ViT-B/14 weights | https://dl.fbaipublicfiles.com/dinov2/dinov2_vitb14/dinov2_vitb14_pretrain.pth | — | Apache-2.0 |

Note: SALAD is GPL-3 -> shipping it inside a distributed app forces GPL on that app; BoQ (MIT) + DINOv2 (Apache) do not.

Weights (in `~/.cache/torch/hub/checkpoints/`): dinov2_12288.pth 380.9 MB, dino_salad.ckpt 352.0 MB, dinov2_vitb14_pretrain.pth 346.4 MB (loaded first, then overwritten by the VPR checkpoints, which contain the backbone).

## Model facts (measured)
| | BoQ-DINOv2 | SALAD-DINOv2 |
|---|---|---|
| descriptor dim | 12288 (48 KB/frame fp32) | 8448 (33 KB/frame fp32) |
| params | 95.2 M | 88.0 M |
| GPU ms/img, steady, batch 8, fp32 / fp16 | 5.8 / 2.9 | 6.0 / 2.4 |
| GPU ms/img, batch 1 (WSL launch-bound) | ~17 | ~17 |
| ms/img in real file run (100 imgs, imread between batches) | 14–21 | 18–27 |
| peak GPU mem (b=8, fp32) | 0.47 GB | 0.61 GB |

Test (`vpr_test/`): 5 frames from `recordings/hs_20261024_142148_std60/cam5.mp4` (frames 30, 31, 588, 882, 1734 -> gray 960x540 PGM, script `mk_test_pgm.py`, check `check_ovpr.py`).
cos(frame30, frame31): BoQ 0.983 (322x574: 0.996), SALAD 0.991 (322x574: 0.996). Different places 0.10–0.62. Frame 1734 vs 30: 0.91–0.98 (probably same spot at end of recording — not verified). Files parse, sorted, norms = 1.
Steady-state benchmark: `bench_vpr.py` (random input, 30 iterations).

## Phone notes (all 추정 unless stated)
- Size: ~90 M params -> ~350 MB fp32, ~175 MB fp16, ~90 MB int8 (추정). ViT-B/14 at 322x322 = 529 tokens, 12 blocks, roughly 45–50 GFLOPs/frame (추정).
- ONNX export: should work with `torch.onnx.export` at fixed 322x322 (DINOv2 pos-embed interpolation becomes a constant at fixed size; set XFORMERS_DISABLED). SALAD uses a Sinkhorn optimal-transport loop (fixed iterations, exportable, 추정); BoQ uses nn.TransformerEncoderLayer + MultiheadAttention (exportable, 추정).
- TFLite/LiteRT: via ai-edge-torch or onnx2tf; fp16 or int8 quantisation needed. LayerNorm/GELU/softmax on ViT generally run on the S24 GPU delegate; full NPU (Exynos/Qualcomm) int8 ViT often falls back partially (추정).
- Expected S24 latency: ViT-B fp16 on mobile GPU ~100–300 ms/frame (추정) -> fine for keyframes at ~1–2 Hz, not per frame. Smaller option: retrain/distil with ViT-S/14 (not provided by these repos' released weights).
- Alternative: compute descriptors on the PC (this script) and only send keyframes; phone does nothing.
