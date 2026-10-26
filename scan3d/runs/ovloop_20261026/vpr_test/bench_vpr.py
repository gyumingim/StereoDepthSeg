import sys, time, torch
sys.path.insert(0, sys.argv[1]); import vpr_desc as V
for name in ["boq", "salad"]:
    m, mode, size, _ = (V.build_boq if name == "boq" else V.build_salad)(); m = m.eval().cuda()
    for b in [1, 8]:
        for fp16 in [False, True]:
            x = torch.randn(b, 3, *size, device="cuda")
            with torch.no_grad(), torch.autocast("cuda", dtype=torch.float16, enabled=fp16):
                for _ in range(5): m(x)
                torch.cuda.synchronize(); t = time.perf_counter(); n = 30
                for _ in range(n): m(x)
                torch.cuda.synchronize()
            print(f"{name} b={b} fp16={fp16}: {1000*(time.perf_counter()-t)/(n*b):.2f} ms/img (steady)", flush=True)
    del m; torch.cuda.empty_cache()
    for k in [k for k in list(sys.modules) if k.split('.')[0] in ("models", "backbones", "boq")]: del sys.modules[k]
