import numpy as np, onnxruntime as ort, json, time
R = "C:/Users/a3162/scan3d_handoff/npu_results/xf960soft"; U = "C:/Users/a3162/scan3d_handoff/npu_upload/xf960soft"
res = {"made": time.strftime("%F %T")}
snr = lambda a, b: float(10 * np.log10((b ** 2).sum() / ((a - b) ** 2).sum()))
for n, ch in (("x960psmc", 3), ("x960p1mc", 1)):
    x = np.fromfile(f"{R}/{n}/input_data_float32.bin", np.float32).reshape(1, ch, 544, 960)
    g = np.fromfile(f"{R}/{n}/golden_data_float32.bin", np.float32).reshape(129, 68, 120)
    S = ort.InferenceSession(f"{U}/xf960_{ch}ch_postsoft.onnx", providers=["CPUExecutionProvider"])
    p = S.run(None, {"image": x})[0][0]
    p.astype(np.float32).tofile(f"{R}/{n}/gold_pc_fp32.bin")
    res[n] = {"in_range": [float(x.min()), float(x.max())], "snr_feats": snr(g[:64], p[:64]), "snr_prob": snr(g[64:128], p[64:128]),
              "snr_heat": snr(g[128], p[128]), "prob_maxabs": float(np.abs(g[64:128] - p[64:128]).max())}
print(json.dumps(res, indent=1)); json.dump(res, open(f"{R}/gold_snr.json", "w"), indent=1)
