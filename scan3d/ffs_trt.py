"""ffs_trt.py — Fast-FoundationStereo on TensorRT (opt-in: scan_offline --ffs-backend trt). Default stays PyTorch (lib_ffs).

Same job as lib_ffs.infer (rectified RGB pair -> disparity px at the input resolution), same resize / un-resize maths:
  - resize both images to dsize = round(W0*s), round(H0*s) (INTER_AREA), un-resize with the REAL ratio W/W0  (= lib_ffs.infer)
  - padding: lib_ffs pads (replicate, 'sintel' split) to the next multiple of 32; here the same /32 size and split (unit 13d):
    a static engine of exactly that size (544x960 / 960x544 = the full 960x540 frame), else a dynamic-shape engine whose
    profile holds it. Only without either: the smallest static "bucket" that holds it (wider replicate border -> on the
    downscaled pairs p95 |dZ| 28 mm vs PyTorch, README unit 13b; avoid).
  - ImageNet normalisation is done here in fp32 (the single-ONNX export strips it from the graph, upstream make_single_onnx.py).
Loud failures, no fallback: no engine dir / manifest, engine built for another TensorRT version, GPU, checkpoint, iters,
max_disp or export version, or an input larger than every engine -> RuntimeError.

Build (once per GPU / TensorRT version; RTX 5080 ~7 min, RTX 4060 laptop ~1 h):
  venv_ffs/bin/python scan3d/ffs_trt.py build --out /home/system/scanner/ffs_trt/engines_v3 --sizes 544x960,960x544 \
      --dynamic "128x128,384x672,544x960;128x128,672x384,960x544"
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
ENGINE_DIR = os.environ.get("FFS_TRT_ENGINES", "/home/system/scanner/ffs_trt/engines_v3")   # export v3 (unit 13d)
MEAN = (123.675, 116.28, 103.53)      # ImageNet in 0-255 (= core/foundation_stereo.normalize_image)
STD = (58.395, 57.12, 57.375)
EXPORT_VERSION = 3                    # bump when the export recipe below changes (engines carry it)
# v3 (unit 13d): InstanceNorm decomposed + fp32, every ReduceL2 fp32, gwc F.normalize eps 1e-3 (fp16-safe). v1 engines
# gave NaN on the RTX 5080: gwc normalisation 0/0 on the zero-padded shifted columns (eps 1e-12 = 0 in fp16).


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


def pt_infer_variant(model, rgbL, rgbR, valid_iters=8, scale=1.0, buckets=None, fp32=False):
    """lib_ffs.infer with two switches (diagnostic, unit 13b): buckets = [(H, W), ...] -> pad to the smallest bucket like TrtFFS
    (else the next /32 like lib_ffs); fp32 = no autocast (caller also sets model.args.mixed_precision False)."""
    import cv2
    import torch
    H0, W0 = rgbL.shape[:2]
    l, r = rgbL, rgbR
    if scale != 1.0:
        dsize = (max(1, int(round(W0 * scale))), max(1, int(round(H0 * scale))))
        l = cv2.resize(l, dsize, interpolation=cv2.INTER_AREA)
        r = cv2.resize(r, dsize, interpolation=cv2.INTER_AREA)
    H, W = l.shape[:2]
    if buckets:
        fit = [b for b in buckets if b[0] >= H and b[1] >= W]
        Hb, Wb = min(fit, key=lambda b: b[0] * b[1])
    else:
        Hb, Wb = -(-H // 32) * 32, -(-W // 32) * 32
    pad = pad_split(H, W, Hb, Wb)
    F = torch.nn.functional
    tl = F.pad(torch.as_tensor(np.ascontiguousarray(l)).cuda().float()[None].permute(0, 3, 1, 2), pad, mode="replicate")
    tr = F.pad(torch.as_tensor(np.ascontiguousarray(r)).cuda().float()[None].permute(0, 3, 1, 2), pad, mode="replicate")
    with torch.no_grad(), torch.amp.autocast("cuda", enabled=not fp32, dtype=torch.float16):
        d = model.forward(tl, tr, iters=valid_iters, test_mode=True, optimize_build_volume="pytorch1")
    disp = d.float()[0, 0, pad[2]:Hb - pad[3], pad[0]:Wb - pad[1]].cpu().numpy().reshape(H, W).clip(0, None)
    if (H, W) != (H0, W0):
        disp = cv2.resize(disp, (W0, H0), interpolation=cv2.INTER_LINEAR) / (W / W0)
    return disp.astype(np.float32)


BUCKETS = [(544, 960), (448, 768), (352, 576), (256, 448), (960, 544), (768, 448), (576, 352), (448, 256)]


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
            self.eng.append(dict(H=e["H"], W=e["W"], engine=engine, ctx=ctx, n=0, dynamic=bool(e.get("dynamic")),
                                 min=e.get("min"), max=e.get("max")))
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
        """static engines: smallest bucket that holds h x w. dynamic engine (unit 13b): h, w padded to the next multiple of 32
        exactly like lib_ffs (InputPadder divis_by 32) -> same padding as PyTorch."""
        H32, W32 = -(-h // 32) * 32, -(-w // 32) * 32
        for e in self.eng:                   # unit 13d: a static engine of exactly the /32 size (e.g. 544x960) = PyTorch padding
            if not e["dynamic"] and (e["H"], e["W"]) == (H32, W32):
                return e
        for e in self.eng:
            if e["dynamic"] and e["min"][0] <= H32 <= e["max"][0] and e["min"][1] <= W32 <= e["max"][1]:
                return dict(H=H32, W=W32, dyn=e)
        fit = [e for e in self.eng if not e["dynamic"] and e["H"] >= h and e["W"] >= w]
        if not fit:
            raise RuntimeError(f"--ffs-backend trt: input {w}x{h} is larger than every engine "
                               f"{[(e['W'], e['H']) for e in self.eng]} (build a bigger bucket)")
        return min(fit, key=lambda e: e["H"] * e["W"])

    def run(self, tl, tr):
        """tl, tr: float32 cuda [1,3,Hb,Wb] 0-255 -> disparity [Hb,Wb] float32 cuda."""
        torch = self.torch
        Hb, Wb = tl.shape[-2:]
        e = getattr(self, "_cur", None) or [x for x in self.eng if x["H"] == Hb and x["W"] == Wb][0]
        xl = ((tl / 255.0 - self.mean / 255.0) / (self.std / 255.0)).contiguous()   # = (img/255 - mean)/std of normalize_image
        xr = ((tr / 255.0 - self.mean / 255.0) / (self.std / 255.0)).contiguous()
        out = torch.empty((1, 1, Hb, Wb), dtype=torch.float32, device="cuda")
        ctx = e["ctx"]
        if e["dynamic"]:
            ctx.set_input_shape("left_image", (1, 3, Hb, Wb)); ctx.set_input_shape("right_image", (1, 3, Hb, Wb))
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
        self._cur = e.get("dyn")
        with torch.no_grad():
            d = self.run(tl, tr)
        self._cur = None
        disp = d[pad[2]:e["H"] - pad[3], pad[0]:e["W"] - pad[1]].cpu().numpy().reshape(H, W).clip(0, None)
        if (H, W) != (H0, W0):
            disp = cv2.resize(disp, (W0, H0), interpolation=cv2.INTER_LINEAR) / (W / W0)
        return disp.astype(np.float32)

    def stats(self):
        return dict(engines=[dict(H=e["H"], W=e["W"], calls=e["n"], dynamic=e["dynamic"], min=e["min"], max=e["max"])
                             for e in self.eng], trt=self.trt.__version__,
                    shapes={f"{h}x{w}->{hb}x{wb}": n for (h, w, hb, wb), n in sorted(self.shapes.items())},
                    manifest_made=self.manifest.get("made"))


# ------------------------------------------------------------------ build (export ONNX per bucket -> TensorRT fp16 engine)
def build(out, sizes, valid_iters=8, max_disp=192, workspace_gb=4.0, ckpt=None, dynamic=None):
    """sizes = static engines; dynamic = [(min, opt, max) HxW, ...]: one dynamic-shape engine per profile, traced at opt."""
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
    F = torch.nn.functional

    def _gwc_fp16_safe(refimg_fea, targetimg_fea, maxdisp, num_groups, normalize=True):
        """= upstream _build_gwc_volume_onnx, but F.normalize eps 1e-3 instead of 1e-12 (unit 13d, export v2). The shifted target
        volume has zero-padded columns (norm 0): eps 1e-12 is 0 in fp16 -> 0/0 = NaN (10 % of /Mul_1 on the RTX 5080). Zero
        vectors still give 0 (0 / 1e-3); a real feature vector has norm >> 1e-3, so the values equal the PyTorch path."""
        B, C, H, W = refimg_fea.shape
        cpg = C // num_groups
        ref_volume = refimg_fea.unsqueeze(2).expand(B, C, maxdisp, H, W)
        target_volume = torch.stack([F.pad(targetimg_fea, (d, 0, 0, 0), "constant", 0.0)[:, :, :, :W] for d in range(maxdisp)], dim=2)
        ref_volume = ref_volume.view(B, num_groups, cpg, maxdisp, H, W)
        target_volume = target_volume.view(B, num_groups, cpg, maxdisp, H, W)
        if normalize:
            ref_volume = F.normalize(ref_volume.float(), dim=2, eps=1e-3).to(refimg_fea.dtype)
            target_volume = F.normalize(target_volume.float(), dim=2, eps=1e-3).to(refimg_fea.dtype)
        return (ref_volume * target_volume).sum(dim=2).contiguous()
    fsm.build_gwc_volume_optimized_pytorch1 = _gwc_fp16_safe
    fsm.build_concat_volume_optimized_pytorch1 = mso._build_concat_volume_onnx
    wrapper = mso.FastFoundationStereoSingleOnnx(model).cuda().eval()
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    # 1) export every bucket to ONNX here (torch on the GPU), 2) build each engine in a FRESH process: the TensorRT builder
    #    in the export process ran out of GPU memory (torch cache held 7.35 of 8 GB on the laptop, 11-12 first try)
    names, times = [], {}
    # unit 13d (export v2): InstanceNorm decomposed into mean / var / sqrt in the ONNX (no TensorRT InstanceNormalization layer:
    # its Myelin kernel __myl_Inor hit CUDA error 720 in the 4060 dynamic-shape engine) + those layers kept fp32.
    # FFS_TRT_DECOMP_IN=0 = old native layer (diagnostics only).
    decomp = os.environ.get("FFS_TRT_DECOMP_IN", "1") == "1"
    in_scopes = []
    if decomp:
        import torch.nn.modules.instancenorm as inm

        def _in_forward(self, x):
            dims = tuple(range(2, x.dim()))
            xf = x.float()
            m = xf.mean(dims, keepdim=True)
            c = xf - m
            v = (c * c).mean(dims, keepdim=True)
            y = c / torch.sqrt(v + self.eps)
            if self.affine:
                sh = (1, -1) + (1,) * (x.dim() - 2)
                y = y * self.weight.view(sh) + self.bias.view(sh)
            return y.to(x.dtype)
        inm._InstanceNorm.forward = _in_forward
        def _scope(n):                   # torch.onnx scope names: wrapper.forward calls model.forward (no "model" level);
            out, prev = [], ""             # a numbered child of a Sequential is "<parent>.<i>" (e.g. /stem_2/stem_2.0/IN/)
            for c in n.split("."):
                out.append(f"{prev}.{c}" if c.isdigit() else c)
                prev = out[-1]
            return "/" + "/".join(out) + "/"
        in_scopes = [_scope(n) for n, mod in wrapper.model.named_modules() if isinstance(mod, inm._InstanceNorm)]
        print(f"[ffs-trt build] InstanceNorm decomposed for export: {len(in_scopes)} modules, kept fp32", flush=True)
    # dynamic = list of (min, opt, max) profiles, one engine each (unit 13d: landscape + portrait; ONE 960x960 box needed > 4 GB
    # builder workspace and sizes the shared activation memory for 960x960)
    jobs = [(H, W, None) for (H, W) in sizes] + [(p[1][0], p[1][1], p) for p in (dynamic or [])]
    recipe = dict(export_version=EXPORT_VERSION, inorm="decomposed_fp32" if decomp else "native", fp32_layers="ReduceL2*",
                  precision="fp32" if os.environ.get("FFS_TRT_FP32") == "1" else "fp16", valid_iters=valid_iters,
                  max_disp=max_disp, ckpt=_ckpt_id(ckpt))
    for (H, W, dyn) in jobs:
        assert H % 32 == 0 and W % 32 == 0, (H, W)
        name = f"ffs_c_it{valid_iters}_md{max_disp}_{H}x{W}" + ("_dyn" if dyn else "")
        rec = dict(recipe, profile=[list(x) for x in dyn] if dyn else None)
        side = out / f"{name}.engine.json"
        if (out / f"{name}.engine").exists() and side.exists() and json.loads(side.read_text()) == rec:
            print(f"[ffs-trt build] {name}: engine of the same recipe exists, kept", flush=True)
            times[name] = dict(kept=True)
            names.append((H, W, name, dyn))
            continue
        t0 = time.time()
        dax = {k: {2: "H", 3: "W"} for k in ("left_image", "right_image", "disparity")} if dyn else None
        with torch.no_grad():
            torch.onnx.export(wrapper, (torch.randn(1, 3, H, W, device="cuda"), torch.randn(1, 3, H, W, device="cuda")),
                              str(out / f"{name}.onnx"), opset_version=17, input_names=["left_image", "right_image"],
                              output_names=["disparity"], do_constant_folding=True, dynamo=False, dynamic_axes=dax)
        if decomp:
            (out / f"{name}.fp32scopes.json").write_text(json.dumps(in_scopes))
        times[name] = dict(onnx_s=round(time.time() - t0, 1))
        names.append((H, W, name, dyn))
        torch.cuda.empty_cache()
    gpu = _gpu_name()
    del wrapper, model
    lib_ffs._MODEL.clear()
    torch.cuda.empty_cache()
    import subprocess
    trt = _trt()
    engines = []
    for (H, W, name, dyn) in names:
        ep = out / f"{name}.engine"
        if not times[name].get("kept"):
            t1 = time.time()
            prof = [] if not dyn else ["--profile", ",".join(f"{h}x{w}" for h, w in dyn)]
            subprocess.run([sys.executable, str(Path(__file__).resolve()), "engine", str(out / f"{name}.onnx"),
                            str(ep), str(workspace_gb)] + prof, check=True)
            times[name]["engine_s"] = round(time.time() - t1, 1)
            (out / f"{name}.engine.json").write_text(json.dumps(dict(recipe, profile=[list(x) for x in dyn] if dyn else None)))
            (out / f"{name}.onnx").unlink()                                      # big; re-exportable
            (out / f"{name}.fp32scopes.json").unlink(missing_ok=True)
        print(f"[ffs-trt build] {name}: {times[name]}, {ep.stat().st_size / 2 ** 20:.0f} MB", flush=True)
        engines.append(dict(H=H, W=W, file=ep.name, sha=_sha16(ep)) if not dyn else
                       dict(H=H, W=W, file=ep.name, sha=_sha16(ep), dynamic=True, min=list(dyn[0]), opt=list(dyn[1]),
                            max=list(dyn[2])))
    man = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), trt=trt.__version__, torch=torch.__version__, gpu=gpu,
               ckpt=_ckpt_id(ckpt), valid_iters=valid_iters, max_disp=max_disp, export_version=EXPORT_VERSION, precision=("fp32" if os.environ.get("FFS_TRT_FP32") == "1" else "fp16"),
               export="upstream scripts/make_single_onnx.py volume builders, opset 17, normalisation outside", engines=engines,
               build_times=times, workspace_gb=workspace_gb,
               inorm=("decomposed_fp32" if decomp else "native"), fp32_layers="ReduceL2*")
    (out / "manifest.json").write_text(json.dumps(man, indent=1))
    print(json.dumps(man, indent=1))


def build_engine(onnx_p, engine_p, workspace_gb, profile=None):
    trt = _trt()
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    net = builder.create_network(0)
    parser = trt.OnnxParser(net, logger)
    if not parser.parse_from_file(str(onnx_p)):
        raise RuntimeError("ONNX parse failed: " + "; ".join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    cfg = builder.create_builder_config()
    if os.environ.get("FFS_TRT_FP32") != "1":                 # FFS_TRT_FP32=1: diagnostic all-fp32 engine (unit 13d)
        cfg.set_flag(trt.BuilderFlag.FP16)
    cfg.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, int(workspace_gb * 2 ** 30))
    # unit 13d: every ReduceL2 (its pow / reduce / sqrt layers carry the node name) in fp32. RTX 5080 v1 engines: /ReduceL2_1 of
    # the gwc feature normalisation reached ~2000 -> squares overflow fp16 -> /Mul_1 10 % non-finite -> all-NaN disparity
    # (runs/fixes_20261112/trt_nan_bisect.py). The 4060 static v1 tactics happened to accumulate in fp32.
    nl2 = 0
    for li in range(net.num_layers):
        L = net.get_layer(li)
        if "ReduceL2" in L.name:
            L.precision = trt.float32
            for oi in range(L.num_outputs):
                if L.get_output_type(oi) in (trt.float32, trt.float16):
                    L.set_output_type(oi, trt.float32)
            nl2 += 1
    if nl2:
        cfg.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)
    print(f"[ffs-trt build] {nl2} ReduceL2 layers forced fp32", flush=True)
    sp = Path(str(onnx_p)[:-len(".onnx")] + ".fp32scopes.json")
    if sp.exists():                                             # unit 13d: decomposed InstanceNorm layers -> fp32
        scopes = tuple(json.loads(sp.read_text()))
        n32, hit = 0, {sc: 0 for sc in scopes}
        for li in range(net.num_layers):
            L = net.get_layer(li)
            sc = next((sc for sc in scopes if L.name.startswith(sc)), None)
            if sc is not None:
                hit[sc] += 1
                L.precision = trt.float32
                for oi in range(L.num_outputs):
                    if L.get_output_type(oi) in (trt.float32, trt.float16):
                        L.set_output_type(oi, trt.float32)
                n32 += 1
        miss = [sc for sc, k in hit.items() if k == 0]
        if n32 == 0:
            raise RuntimeError(f"no layer in the InstanceNorm scopes (names changed?): {scopes}")
        if miss:                         # modules that exist but are not run in this graph have no layers
            print(f"[ffs-trt build] WARNING InstanceNorm scopes without layers: {miss}", flush=True)
        cfg.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)
        print(f"[ffs-trt build] {n32} of {net.num_layers} layers in {len(scopes)} InstanceNorm scopes forced fp32", flush=True)
    if profile:
        (h0, w0), (h1, w1), (h2, w2) = profile
        op = builder.create_optimization_profile()
        for nm in ("left_image", "right_image"):
            op.set_shape(nm, (1, 3, h0, w0), (1, 3, h1, w1), (1, 3, h2, w2))
        cfg.add_optimization_profile(op)
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
    b.add_argument("--sizes", default="", help="HxW,HxW,... padded engine sizes (multiples of 32)")
    b.add_argument("--iters", type=int, default=8)
    b.add_argument("--max-disp", type=int, default=192)
    b.add_argument("--workspace-gb", type=float, default=4.0)
    e = sub.add_parser("engine", help="internal: one ONNX -> fp16 engine (fresh process, no torch on the GPU)")
    e.add_argument("onnx"); e.add_argument("engine"); e.add_argument("workspace_gb", type=float)
    e.add_argument("--profile", default="", help="min,opt,max HxW (dynamic engine)")
    b.add_argument("--dynamic", default="", help="min,opt,max HxW[;min,opt,max ...]: dynamic-shape engines (one per profile), "
                                                 "used for every input whose /32 size has no exact static engine")
    a = ap.parse_args()
    hw = lambda txt: [tuple(int(v) for v in t.split("x")) for t in txt.split(",")] if txt else []
    if a.cmd == "engine":
        build_engine(a.onnx, a.engine, a.workspace_gb, hw(a.profile) or None)
    if a.cmd == "build":
        build(a.out, hw(a.sizes), a.iters, a.max_disp, a.workspace_gb,
              dynamic=[hw(p) for p in a.dynamic.split(";") if p] or None)
