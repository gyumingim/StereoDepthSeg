#!/usr/bin/env python3
"""kfsel 11-11: copy the new-default 133820 mesh to the handoff viewer and append one "[새]" entry to scanA/meshes.json
(backup first, re-read just before writing, append only if the file is not listed yet). Same format as hybrid/publish.py."""
import json, shutil, time
from pathlib import Path
H = Path(__file__).resolve().parent
OUT = Path("/mnt/c/Users/a3162/scan3d_handoff/changes_0620/kfsel")
MJ = Path("/mnt/c/Users/a3162/scan3d_handoff/changes_0620/scanA/meshes.json")
OUT.mkdir(parents=True, exist_ok=True)
src = H.parent / "laptop" / "20261022_133820" / "newdefault" / "scan" / "mesh" / "mesh.ply"
name = "scanner_133820_newdefault.ply"
shutil.copy2(src, OUT / name)
e = {"file": f"../kfsel/{name}", "label": "[새] 133820 · 새 기본값 (움직임 키프레임 + 넓은 짝)", "rec": "133820", "board_mm": [569.646, 316.47]}
shutil.copy2(MJ, MJ.with_name(f"meshes.json.bak_kfsel_{time.strftime('%Y%m%d_%H%M%S')}"))
cur = json.loads(MJ.read_text(encoding="utf-8"))
if e["file"] not in {x["file"] for x in cur}:
    cur.append(e)
MJ.write_text(json.dumps(cur, ensure_ascii=False, indent=1), encoding="utf-8")
print("added", e)
