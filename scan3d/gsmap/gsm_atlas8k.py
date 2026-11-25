#!/usr/bin/env python3
"""gsmap 2026-11-25: downscale a GLB texture atlas to <= 8192 px (xatlas may pack slightly larger). In place."""
import json, struct, sys
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gsm_texture import write_glb
p = sys.argv[1]; b = open(p, "rb").read(); jl = struct.unpack("<I", b[12:16])[0]; j = json.loads(b[20:20 + jl]); bl = b[20 + jl + 8:]
bv = j["bufferViews"]; g = lambda i: bl[bv[i]["byteOffset"]:bv[i]["byteOffset"] + bv[i]["byteLength"]]  # noqa: E731
V = np.frombuffer(g(0), np.float32).reshape(-1, 3); UV = np.frombuffer(g(1), np.float32).reshape(-1, 2); F = np.frombuffer(g(2), np.uint32).reshape(-1, 3)
tex = cv2.imdecode(np.frombuffer(g(3), np.uint8), 1)
if max(tex.shape[:2]) > 8192:
    tex = cv2.resize(tex, (8192, 8192), interpolation=cv2.INTER_AREA); ok, jpg = cv2.imencode(".jpg", tex, [cv2.IMWRITE_JPEG_QUALITY, 88])
    write_glb(p, V, UV, F, jpg.tobytes())
print("atlas", tex.shape[:2], "glb MB", round(Path(p).stat().st_size / 2 ** 20, 2))
