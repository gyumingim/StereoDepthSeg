"""
lib_ffs.py — Fast-FoundationStereo 추론 어댑터 (정류된 좌/우 -> 시차맵)

※ 이 모듈은 venv_ffs (python3.12, torch 2.6.0+cu124, xformers) 에서만 동작한다.
   메인 venv(torch 2.14) 와는 고정 버전이 달라 같은 프로세스에 못 넣는다.
   실행:  ./venv_ffs/bin/python z_object_depth.py ...

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
공식 scripts/run_demo.py 와 똑같이 한다 (직접 읽고 옮김, commit master 2026-09)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  model = torch.load(ckpt, map_location='cpu', weights_only=False)   # 직렬화된 모델 객체
  model.args.valid_iters / max_disp 덮어쓰기 -> .cuda().eval()
  입력: RGB uint8 -> float 텐서 [1,3,H,W] (정규화 없음, 0~255 그대로)
  InputPadder(divis_by=32) 로 패딩 -> autocast fp16 ->
  model.forward(l, r, iters, test_mode=True, optimize_build_volume='pytorch1')
  -> unpad -> HxW 시차(px), 0 아래는 clip.
  scale<1 로 축소해 넣으면 시차도 scale 배가 되므로 되돌릴 때 /scale 한다.

  직렬화된 체크포인트라 저장소의 core/ 패키지가 import 가능해야 언피클된다.
  → tools/Fast-FoundationStereo 를 sys.path 에 넣는다. 체크포인트와 코드 버전을 함께 고정할 것.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
라이선스
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  코드 LICENSE.txt: "non-commercially means for research purposes only" (54~57행).
  가중치 C-Fast-FoundationStereo (HF nvidia/c-fast-foundationstereo): NVIDIA Open Model
  Agreement — 상업 허용. 상업 배포는 README 가 안내하는 TAO/TensorRT 경로를 써야 한다.
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
FFS_DIR = HERE / "tools" / "Fast-FoundationStereo"
DEFAULT_CKPT = FFS_DIR / "weights" / "c" / "model_best_bp2_serialize.pth"

_MODEL = {}


def triton_usable():
    """Triton 이 첫 실행에 C 확장(cuda_utils)을 컴파일하려면 Python.h 가 있어야 한다.

    이 PC 는 deadsnakes python3.12 만 있고 python3.12-dev 가 없어 컴파일이 실패했다
    (gcc: fatal error: Python.h: No such file or directory). 헤더가 없으면 triton 을 못 쓴다.
    """
    import sysconfig
    return (Path(sysconfig.get_paths()["include"]) / "Python.h").exists()


def _import_ffs():
    """저장소 코드를 import 한다. 필요하면 triton 없는 순수 PyTorch 경로로 우회한다.

    core/submodule.py 는 `try: import triton ... except: triton = None` 이고 triton 커널은
    triton 이 있을 때만 정의된다. 우리는 forward 에 optimize_build_volume='pytorch1' 을 넘기므로
    triton 이 없어도 동작한다. 단 같은 파일의 @torch.compile 은 CUDA 에서 inductor->triton 을
    요구하므로 TORCH_COMPILE_DISABLE=1 로 무력화한다 (eager 실행, 느려질 뿐 결과는 같다).
    Python.h 가 생기면(python3.12-dev 설치) 자동으로 triton 경로를 쓴다.
    """
    if str(FFS_DIR) not in sys.path:
        sys.path.insert(0, str(FFS_DIR))
    import torch                                   # venv_ffs 의 torch 2.6
    if not triton_usable():
        # torch 가 이미 import 돼 있어도 된다: triton 은 core/submodule.py 가 import 할 때 막히고,
        # torch.compile 은 dynamo 설정으로 직접 끈다 (env 는 import 시점에 따라 안 먹을 수 있다).
        sys.modules["triton"] = None               # import triton -> ImportError -> triton = None
        import torch._dynamo
        torch._dynamo.config.disable = True
        _import_ffs.mode = "pytorch-only (Python.h 없음: triton/torch.compile 비활성)"
    else:
        _import_ffs.mode = "triton"
    from core.utils.utils import InputPadder       # 저장소 코드
    return torch, InputPadder


_import_ffs.mode = "triton"


def load(ckpt=DEFAULT_CKPT, valid_iters=8, max_disp=192):
    """직렬화된 모델을 불러 GPU 에 올린다. 한 번만 하고 캐시한다."""
    key = (str(ckpt), valid_iters, max_disp)
    if key in _MODEL:
        return _MODEL[key]
    torch, _ = _import_ffs()
    if not Path(ckpt).exists():
        raise FileNotFoundError(f"체크포인트 없음: {ckpt}")
    model = torch.load(str(ckpt), map_location="cpu", weights_only=False)
    # 직렬화된 args 에 빠진 키를 채운다. C 체크포인트의 args 에는 `normalize` 가 없어서
    # forward(foundation_stereo.py:204) 가 ConfigAttributeError 로 죽었다. 저장소 자체 기본값은
    # True (make_plugin_onnx.py:123 getattr(model.args,'normalize',True), submodule.py:377 normalize=True).
    # cfg.yaml(체크포인트 옆) 의 키도 run_demo.py 와 같이 반영한다.
    try:
        from omegaconf import OmegaConf
        OmegaConf.set_struct(model.args, False)
    except Exception:
        pass
    cfg_p = Path(ckpt).with_name("cfg.yaml")
    filled = []
    if cfg_p.exists():
        import yaml
        for k, v in (yaml.safe_load(cfg_p.read_text()) or {}).items():
            if not hasattr(model.args, k):
                setattr(model.args, k, v); filled.append(k)
    if not hasattr(model.args, "normalize"):
        model.args.normalize = True; filled.append("normalize=True(저장소 기본값)")
    load.filled_keys = filled
    model.args.valid_iters = valid_iters
    model.args.max_disp = max_disp
    model = model.cuda().eval()
    _MODEL[key] = model
    return model


def infer(model, rgbL, rgbR, valid_iters=8, scale=1.0, hiera=False):
    """정류된 RGB 쌍 -> 시차(px, float32 HxW, 원본 해상도 기준). 시차<=0 은 무효."""
    torch, InputPadder = _import_ffs()
    import cv2
    assert rgbL.shape == rgbR.shape and rgbL.ndim == 3 and rgbL.shape[2] == 3, "RGB HxWx3 두 장"
    if not (0.0 < scale <= 1.0):
        raise ValueError(f"scale 은 (0,1] 이어야 함: {scale}")
    H0, W0 = rgbL.shape[:2]
    l, r = rgbL, rgbR
    if scale != 1.0:
        l = cv2.resize(l, None, fx=scale, fy=scale)
        r = cv2.resize(r, (l.shape[1], l.shape[0]))
    H, W = l.shape[:2]
    tl = torch.as_tensor(np.ascontiguousarray(l)).cuda().float()[None].permute(0, 3, 1, 2)
    tr = torch.as_tensor(np.ascontiguousarray(r)).cuda().float()[None].permute(0, 3, 1, 2)
    padder = InputPadder(tl.shape, divis_by=32, force_square=False)
    tl, tr = padder.pad(tl, tr)
    try:
        with torch.no_grad(), torch.amp.autocast("cuda", enabled=True, dtype=torch.float16):
            if hiera:
                disp = model.run_hierachical(tl, tr, iters=valid_iters, test_mode=True, small_ratio=0.5)
            else:
                disp = model.forward(tl, tr, iters=valid_iters, test_mode=True,
                                     optimize_build_volume="pytorch1")
    except torch.OutOfMemoryError as e:
        # 순수 PyTorch 경로(triton 없음)는 코스트 볼륨을 6D 텐서로 통째로 만들어 메모리를 많이 쓴다.
        # 실측: 960x540 은 VRAM 4.4GB, 1280x720 은 8GB 초과. --scale 0.75(960x540) 또는 0.5 를 쓸 것.
        # python3.12-dev 가 설치되면 triton 경로가 자동으로 켜져 훨씬 적게 쓴다.
        raise RuntimeError(f"CUDA 메모리 부족 (입력 {W}x{H}, {_import_ffs.mode}). "
                           f"--scale 0.75 또는 0.5 로 줄일 것. 원문: {str(e)[:120]}") from e
    disp = padder.unpad(disp.float()).data.cpu().numpy().reshape(H, W).clip(0, None)
    if (H, W) != (H0, W0):
        # 요청 scale 이 아니라 실제로 일어난 비율(W/W0)로 되돌린다. 정수 반올림 때문에
        # 둘이 다를 수 있다. 시차는 가로 화소 단위이므로 가로 비율을 쓴다.
        disp = cv2.resize(disp, (W0, H0), interpolation=cv2.INTER_LINEAR) / (W / W0)
    return disp.astype(np.float32)
