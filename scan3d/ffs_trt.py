"""ffs_trt.py — Fast-FoundationStereo on TensorRT (opt-in: scan_offline --ffs-backend trt). Default stays PyTorch (lib_ffs).

Same job as lib_ffs.infer (rectified RGB pair -> disparity px at the input resolution), same resize / un-resize maths:
  - resize both images to dsize = round(W0*s), round(H0*s) (INTER_AREA), un-resize with the REAL ratio W/W0  (= lib_ffs.infer)
  - padding: lib_ffs pads (replicate, 'sintel' split) to the next multiple of 32. A TensorRT engine has ONE fixed input size,
    so here the image is padded (replicate, same split rule) to the smallest engine "bucket" that holds it, and cropped back.
    The image content / pixel grid is NOT resized for the bucket -> disparity scale maths unchanged; only the replicated border
    is wider than in PyTorch (border context differs a little: measured, runs/fixes_20261112/README.md unit 11).
  - ImageNet normalisation is done here in fp32 (the single-ONNX export strips it from the graph, upstream make_single_onnx.py).
Loud failures, no fallback: no engine dir / manifest, engine built for another TensorRT version, GPU, checkpoint, iters or
max_disp, or an input larger than every bucket -> RuntimeError.

Build (laptop, once per GPU / TensorRT version; ~minutes per bucket):
  venv_ffs/bin/python scan3d/ffs_trt.py build --out /home/system/scanner/ffs_trt/engines --sizes 544x960,960x544,...  (HxW, /32)
TensorRT Python wheels live OUTSIDE venv_ffs (pip --target, FFS_TRT_SITE, default /home/system/scanner/ffs_trt/site) so the
shared venv is not touched; that dir is appended to sys.path only when this module is used.
Engine = derived from C-Fast-FoundationStereo weights (NVIDIA Open Model Agreement); not committed to git.
"""
import hashlib
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

TRT_SITE = os.environ.get("FFS_TRT_SITE", "/home/system/scanner/ffs_trt/site")
ENGINE_DIR = os.environ.get("FFS_TRT_ENGINES", "/home/system/scanner/ffs_trt/engines")
MEAN = (123.675, 116.28, 103.53)      # ImageNet in 0-255 (= core/foundation_stereo.normalize_image)
STD = (58.395, 57.12, 57.375)
EXPORT_VERSION = 1                    # bump when the export recipe below changes (engines carry it)


def _trt():
    if TRT_SITE not in sys.path:
        sys.path.append(TRT_SITE)
    import tensorrt as trt
    return trt


def _sha16(p, n=None):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        h.update(f.read() if n is None else f.read(n))
    return h.hexdigest()[:16]


def _ckpt_id(ckpt):
    st = Path(ckpt).stat()
    return f"{Path(ckpt).name}:{st.st_size}:{_sha16(ckpt, 64 << 20)}"   # first 64 MB + size: fast, catches a swapped file


def _gpu_name():
    import torch
    return torch.cuda.get_device_name(0)


def pad_split(h, w, Hb, Wb):
    """InputPadder 'sintel' split for a total pad of (Hb-h, Wb-w): [left, right, top, bottom]."""
    ph, pw = Hb - h, Wb - w
    return [pw // 2, pw - pw // 2, ph // 2, ph - ph // 2]


class TrtFFS:
    """Engines of one directory (manifest.json). infer(rgbL, rgbR, scale) like lib_ffs.infer."""

    def __init__(self, engine_dir=ENGINE_DIR, valid_iters=8, max_disp=192, ckpt=None, log=print):
        import torch
        trt = _trt()
        self.torch, self.trt = torch, trt
        d = Path(engine_dir)
        man_p = d / "manifest.json"
        if not man_p.exists():
            raise RuntimeError(f"--ffs-backend trt: no engine manifest {man_p} (build: python scan3d/ffs_trt.py build ...)")
        man = json.loads(man_p.read_text())
        if ckpt is None:
            sys.path.insert(0, os.environ.get("FFS_ROOT", str(Path(__file__).resolve().parents[1])))
            import lib_ffs
            ckpt = lib_ffs.DEFAULT_CKPT
        want = dict(trt=trt.__version__, gpu=_gpu_name(), ckpt=_ckpt_id(ckpt), valid_iters=valid_iters, max_disp=max_disp,
                    export_version=EXPORT_VERSION)
        bad = {k: (man.get(k), v) for k, v in want.items() if man.get(k) != v}
        if bad:
            raise RuntimeError(f"--ffs-backend trt: engines in {d} do not match this run (manifest, wanted): {bad}. Rebuild them.")
        self.logger = trt.Logger(trt.Logger.WARNING)
        rt = trt.Runtime(self.logger)
        self.eng = []
        mem = 0
        for e in man["engines"]:
            p = d / e["file"]
            if not p.exists() or _sha16(p) != e["sha"]:
                raise RuntimeError(f"--ffs-backend trt: engine {p} missing or changed (sha != manifest)")
            engine = rt.deserialize_cuda_engine(p.read_bytes())
            if engine is None:
                raise RuntimeError(f"--ffs-backend trt: could not deserialize {p}")
            ctx = engine.create_execution_context(trt.ExecutionContextAllocationStrategy.USER_MANAGED)
            mem = max(mem, engine.device_memory_size_v2)
            self.eng.append(dict(H=e["H"], W=e["W"], engine=engine, ctx=ctx, n=0))
        # one shared activation buffer for all contexts (only one runs at a time), owned by torch
        self.mem = torch.empty(max(mem, 1), dtype=torch.uint8, device="cuda")
        for e in self.eng:
            e["ctx"].set_device_memory(self.mem.data_ptr(), mem)
        self.mean = torch.tensor(MEAN, device="cuda").view(1, 3, 1, 1)
        self.std = torch.tensor(STD, device="cuda").view(1, 3, 1, 1)
        self.manifest = man
        self.shapes = {}                  # (h, w, Hb, Wb) -> count, for the summary
        log(f"[ffs-trt] {len(self.eng)} engines {[(e['H'], e['W']) for e in self.eng]} from {d}, TensorRT {trt.__version__}, "
            f"shared activation memory {mem / 2 ** 20:.0f} MB")

    def bucket(self, h, w):
        fit = [e for e in self.eng if e["H"] >= h and e["W"] >= w]
        if not fit:
            raise RuntimeError(f"--ffs-backend trt: input {w}x{h} is larger than every engine "
                               f"{[(e['W'], e['H']) for e in self.eng]} (build a bigger bucket)")
        return min(fit, key=lambda e: e["H"] * e["W"])

    def run(self, tl, tr):
        """tl, tr: float32 cuda [1,3,Hb,Wb] 0-255 -> disparity [Hb,Wb] float32 cuda."""
        torch = self.torch
        Hb, Wb = tl.shape[-2:]
        e = [x for x in self.eng if x["H"] == Hb and x["W"] == Wb][0]
        xl = ((tl / 255.0 - self.mean / 255.0) / (self.std / 255.0)).contiguous()   # = (img/255 - mean)/std of normalize_image
        xr = ((tr / 255.0 - self.mean / 255.0) / (self.std / 255.0)).contiguous()
        out = torch.empty((1, 1, Hb, Wb), dtype=torch.float32, device="cuda")
        ctx = e["ctx"]
        ctx.set_tensor_address("left_image", xl.data_ptr())
        ctx.set_tensor_address("right_image", xr.data_ptr())
        ctx.set_tensor_address("disparity", out.data_ptr())
        st = torch.cuda.current_stream()
        if not ctx.execute_async_v3(st.cuda_stream):
            raise RuntimeError("--ffs-backend trt: execute_async_v3 failed")
        e["n"] += 1
        return out[0, 0]

    def infer(self, model_unused, rgbL, rgbR, valid_iters=8, scale=1.0, hiera=False):
        """Drop-in for lib_ffs.infer (model argument ignored). Same resize and un-resize maths."""
        torch = self.torch
        import cv2
        if hiera:
            raise RuntimeError("--ffs-backend trt: hiera not supported")
        assert rgbL.shape == rgbR.shape and rgbL.ndim == 3 and rgbL.shape[2] == 3, "RGB HxWx3 two images"
        if not (0.0 < scale <= 1.0):
            raise ValueError(f"scale must be in (0,1]: {scale}")
        H0, W0 = rgbL.shape[:2]
        l, r = rgbL, rgbR
        if scale != 1.0:
            dsize = (max(1, int(round(W0 * scale))), max(1, int(round(H0 * scale))))
            l = cv2.resize(l, dsize, interpolation=cv2.INTER_AREA)
            r = cv2.resize(r, dsize, interpolation=cv2.INTER_AREA)
        H, W = l.shape[:2]
        e = self.bucket(H, W)
        k = (H, W, e["H"], e["W"]); self.shapes[k] = self.shapes.get(k, 0) + 1
        pad = pad_split(H, W, e["H"], e["W"])
        tl = torch.as_tensor(np.ascontiguousarray(l)).cuda().float()[None].permute(0, 3, 1, 2)
        tr = torch.as_tensor(np.ascontiguousarray(r)).cuda().float()[None].permute(0, 3, 1, 2)
        F = torch.nn.functional
        tl = F.pad(tl, pad, mode="replicate"); tr = F.pad(tr, pad, mode="replicate")
        with torch.no_grad():
            d = self.run(tl, tr)
        disp = d[pad[2]:e["H"] - pad[3], pad[0]:e["W"] - pad[1]].cpu().numpy().reshape(H, W).clip(0, None)
        if (H, W) != (H0, W0):
            disp = cv2.resize(disp, (W0, H0), interpolation=cv2.INTER_LINEAR) / (W / W0)
        return disp.astype(np.float32)

    def stats(self):
        return dict(engines=[dict(H=e["H"], W=e["W"], calls=e["n"]) for e in self.eng], trt=self.trt.__version__,
                    shapes={f"{h}x{w}->{hb}x{wb}": n for (h, w, hb, wb), n in sorted(self.shapes.items())},
                    manifest_made=self.manifest.get("made"))


# ------------------------------------------------------------------ build (export ONNX per bucket -> TensorRT fp16 engine)
def build(out, sizes, valid_iters=8, max_disp=192, workspace_gb=4.0, ckpt=None):
    os.environ.setdefault("TORCH_COMPILE_DISABLE", "1")
    root = Path(os.environ.get("FFS_ROOT", str(Path(__file__).resolve().parents[1])))
    sys.path.insert(0, str(root))
    _trt()                                     # adds TRT_SITE (holds onnx, needed by torch.onnx.export) to sys.path
    import importlib.util
    import torch
    import torch._dynamo
    torch._dynamo.config.disable = True
    import lib_ffs
    ckpt = ckpt or lib_ffs.DEFAULT_CKPT
    model = lib_ffs.load(ckpt, valid_iters=valid_iters, max_disp=max_disp)       # same key filling as the PyTorch path
    model.args.mixed_precision = False                                           # fp32 graph; TensorRT picks fp16 (= upstream)
    spec = importlib.util.spec_from_file_location("ffs_make_single_onnx", lib_ffs.FFS_DIR / "scripts" / "make_single_onnx.py")
    mso = importlib.util.module_from_spec(spec); spec.loader.exec_module(mso)
    import core.foundation_stereo as fsm
    fsm.normalize_image = lambda img: img
    fsm.build_gwc_volume_optimized_pytorch1 = mso._build_gwc_volume_onnx
    fsm.build_concat_volume_optimized_pytorch1 = mso._build_concat_volume_onnx
    wrapper = mso.FastFoundationStereoSingleOnnx(model).cuda().eval()
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    # 1) export every bucket to ONNX here (torch on the GPU), 2) build each engine in a FRESH process: the TensorRT builder
    #    in the export process ran out of GPU memory (torch cache held 7.35 of 8 GB on the laptop, 11-12 first try)
    names, times = [], {}
    for (H, W) in sizes:
        assert H % 32 == 0 and W % 32 == 0, (H, W)
        name = f"ffs_c_it{valid_iters}_md{max_disp}_{H}x{W}"
        t0 = time.time()
        with torch.no_grad():
            torch.onnx.export(wrapper, (torch.randn(1, 3, H, W, device="cuda"), torch.randn(1, 3, H, W, device="cuda")),
                              str(out / f"{name}.onnx"), opset_version=17, input_names=["left_image", "right_image"],
                              output_names=["disparity"], do_constant_folding=True, dynamo=False)
        times[f"{H}x{W}"] = dict(onnx_s=round(time.time() - t0, 1))
        names.append((H, W, name))
        torch.cuda.empty_cache()
    gpu = _gpu_name()
    del wrapper, model
    lib_ffs._MODEL.clear()
    torch.cuda.empty_cache()
    import subprocess
    trt = _trt()
    engines = []
    for (H, W, name) in names:
        t1 = time.time()
        subprocess.run([sys.executable, str(Path(__file__).resolve()), "engine", str(out / f"{name}.onnx"),
                        str(out / f"{name}.engine"), str(workspace_gb)], check=True)
        ep = out / f"{name}.engine"
        times[f"{H}x{W}"]["engine_s"] = round(time.time() - t1, 1)
        print(f"[ffs-trt build] {H}x{W}: {times[f'{H}x{W}']}, {ep.stat().st_size / 2 ** 20:.0f} MB", flush=True)
        engines.append(dict(H=H, W=W, file=ep.name, sha=_sha16(ep)))
        (out / f"{name}.onnx").unlink()                                          # big; re-exportable
    man = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), trt=trt.__version__, torch=torch.__version__, gpu=gpu,
               ckpt=_ckpt_id(ckpt), valid_iters=valid_iters, max_disp=max_disp, export_version=EXPORT_VERSION, precision="fp16",
               export="upstream scripts/make_single_onnx.py volume builders, opset 17, normalisation outside", engines=engines,
               build_times=times, workspace_gb=workspace_gb)
    (out / "manifest.json").write_text(json.dumps(man, indent=1))
    print(json.dumps(man, indent=1))


def build_engine(onnx_p, engine_p, workspace_gb):
    trt = _trt()
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    net = builder.create_network(0)
    parser = trt.OnnxParser(net, logger)
    if not parser.parse_from_file(str(onnx_p)):
        raise RuntimeError("ONNX parse failed: " + "; ".join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    cfg = builder.create_builder_config()
    cfg.set_flag(trt.BuilderFlag.FP16)
    cfg.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, int(workspace_gb * 2 ** 30))
    ser = builder.build_serialized_network(net, cfg)
    if ser is None:
        raise RuntimeError(f"TensorRT build failed for {onnx_p}")
    Path(engine_p).write_bytes(bytes(ser))


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("--out", default=ENGINE_DIR)
    b.add_argument("--sizes", required=True, help="HxW,HxW,... padded engine sizes (multiples of 32)")
    b.add_argument("--iters", type=int, default=8)
    b.add_argument("--max-disp", type=int, default=192)
    b.add_argument("--workspace-gb", type=float, default=4.0)
    e = sub.add_parser("engine", help="internal: one ONNX -> fp16 engine (fresh process, no torch on the GPU)")
    e.add_argument("onnx"); e.add_argument("engine"); e.add_argument("workspace_gb", type=float)
    a = ap.parse_args()
    if a.cmd == "engine":
        build_engine(a.onnx, a.engine, a.workspace_gb)
    if a.cmd == "build":
        build(a.out, [tuple(int(v) for v in s.split("x")) for s in a.sizes.split(",")], a.iters, a.max_disp, a.workspace_gb)
