"""unit 13d diagnostic: where does the TensorRT fp16 FFS engine first produce NaN / inf?
python trt_nan_bisect.py ONNX H W [N_MARKS] [--fp32-from NAME_SUBSTR,...]
Parses the ONNX, marks N_MARKS evenly spaced float layer outputs as extra network outputs, builds an fp16 engine (marking
outputs blocks some fusions: if NaN disappears that is reported too), runs the 12 px synthetic pair of trt_dyn_probe.py and
prints per marked tensor: layer index, name, type, max |x|, NaN / inf fraction. Read-only on the repo; engine kept in memory."""
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import ffs_trt  # noqa: E402

onnx_p, H, W = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
NM = int(sys.argv[4]) if len(sys.argv) > 4 else 60
fp32_from = []
if "--fp32-from" in sys.argv:
    fp32_from = sys.argv[sys.argv.index("--fp32-from") + 1].split(",")
import cv2  # noqa: E402
import torch  # noqa: E402

trt = ffs_trt._trt()
lg = trt.Logger(trt.Logger.WARNING)
b = trt.Builder(lg)
net = b.create_network(0)
p = trt.OnnxParser(net, lg)
assert p.parse_from_file(onnx_p), [str(p.get_error(i)) for i in range(p.num_errors)]
n = net.num_layers
cand = []
for i in range(n):
    L = net.get_layer(i)
    if L.type in (trt.LayerType.CONSTANT, trt.LayerType.SHAPE, trt.LayerType.IDENTITY):
        continue
    if L.num_outputs and L.get_output(0).dtype == trt.float32 and len(L.get_output(0).shape) >= 3:
        cand.append(i)
if "--range" in sys.argv:                                 # --range A,B: marks only between layer indices A and B
    a0, b0 = (int(v) for v in sys.argv[sys.argv.index("--range") + 1].split(","))
    cand = [i for i in cand if a0 <= i <= b0]
pick = sorted(set(cand[int(k)] for k in np.linspace(0, len(cand) - 1, NM)))
names = {}
for i in pick:
    t = net.get_layer(i).get_output(0)
    if not t.is_network_output:
        t.name = f"dbg_{i}"
        net.mark_output(t)
        names[t.name] = i
cfg = b.create_builder_config()
cfg.set_flag(trt.BuilderFlag.FP16)
cfg.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 4 << 30)
if fp32_from:
    for i in range(n):
        L = net.get_layer(i)
        if any(s in L.name for s in fp32_from):
            L.precision = trt.float32
    cfg.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)
print(f"{n} layers, {len(names)} marked, building ...", flush=True)
ser = b.build_serialized_network(net, cfg)
eng = trt.Runtime(lg).deserialize_cuda_engine(ser)
ctx = eng.create_execution_context()
rng = np.random.default_rng(0)
base = (rng.random((H // 8 + 1, W // 8 + 1, 3)) * 255).astype(np.uint8)
Li = cv2.resize(base, (W, H), interpolation=cv2.INTER_CUBIC)
Ri = np.roll(Li, -12, axis=1)
mean = torch.tensor(ffs_trt.MEAN, device="cuda").view(1, 3, 1, 1)
std = torch.tensor(ffs_trt.STD, device="cuda").view(1, 3, 1, 1)
bufs = {}
for k, img in (("left_image", Li), ("right_image", Ri)):
    x = torch.as_tensor(img).cuda().float()[None].permute(0, 3, 1, 2)
    bufs[k] = ((x - mean) / std).contiguous()
TD = {trt.float32: torch.float32, trt.float16: torch.float16, trt.int32: torch.int32, trt.int64: torch.int64, trt.bool: torch.bool}
for j in range(eng.num_io_tensors):
    nm = eng.get_tensor_name(j)
    if eng.get_tensor_mode(nm) == trt.TensorIOMode.OUTPUT:
        bufs[nm] = torch.empty(tuple(eng.get_tensor_shape(nm)), dtype=TD[eng.get_tensor_dtype(nm)], device="cuda")
for nm, t in bufs.items():
    ctx.set_tensor_address(nm, t.data_ptr())
assert ctx.execute_async_v3(torch.cuda.current_stream().cuda_stream)
torch.cuda.synchronize()
d = bufs["disparity"].float()
print(f"disparity: median {d.median().item():.3f}, nan frac {torch.isnan(d).float().mean().item():.4f}")
first = None
for nm, i in sorted(names.items(), key=lambda kv: kv[1]):
    x = bufs[nm].float()
    fin = torch.isfinite(x)
    nanf = (~fin).sum().item() / fin.numel()
    mx = x[fin].abs().max().item() if fin.any() else float("nan")
    L = net.get_layer(i)
    flag = "  <-- first non-finite" if nanf > 0 and first is None else ""
    if nanf > 0 and first is None:
        first = i
    print(f"{i:6d} {str(L.type).split('.')[-1]:14s} max|x| {mx:12.4g}  nonfinite {nanf:.2e}  {L.name[:90]}{flag}")
