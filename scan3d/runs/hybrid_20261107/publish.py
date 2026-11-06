#!/usr/bin/env python3
"""hybrid: copy meshes to C:\\Users\\a3162\\scan3d_handoff\\changes_0620\\hybrid\\ and append "[새]" viewer entries to scanA/meshes.json
(backup first, re-read just before writing, append only entries whose file is not listed yet). Same format as mvggt/publish.py."""
import json
import shutil
import time
from pathlib import Path

H = Path(__file__).resolve().parent
OUT = Path("/mnt/c/Users/a3162/scan3d_handoff/changes_0620/hybrid")
MJ = Path("/mnt/c/Users/a3162/scan3d_handoff/changes_0620/scanA/meshes.json")
REC = {"165506": dict(tag="165506", rec=None, board=None),
       "20261022_134204": dict(tag="134204", rec="134204", board=[569.646, 316.47]),
       "20261020_222747": dict(tag="222747", rec="222747", board=[385.74, 214.3])}
LAB = {"H1": "섞기 H1: FFS 깊이 + FFS 없는 키프레임만 VGGT 깊이",
       "H2": "섞기 H2: H1 + VGGT 축척을 이웃 FFS 키프레임에 맞춤",
       "H3": "섞기 H3: H2 + FFS 빈 화소도 VGGT 로 채움",
       "H1_fs": "섞기 H1 · 정답 축척 (오라클)",
       "H2_fs": "섞기 H2 · 정답 축척 (오라클)",
       "H3_fs": "섞기 H3 · 정답 축척 (오라클)"}
OUT.mkdir(parents=True, exist_ok=True)
new = []
for rid, r in REC.items():
    for v, lab in LAB.items():
        src = H / "rec" / rid / f"mesh_{v}" / "mesh.ply"
        if not src.exists():
            continue
        name = f"hybrid_{r['tag']}_{v}.ply"
        shutil.copy2(src, OUT / name)
        e = {"file": f"../hybrid/{name}", "label": f"[새] {r['tag'] + ' · ' if r['rec'] else ''}{lab}"}
        if r["rec"]:
            e.update(rec=r["rec"], board_mm=r["board"])
        new.append(e)
bak = MJ.with_name(f"meshes.json.bak_hybrid_{time.strftime('%Y%m%d_%H%M%S')}")
shutil.copy2(MJ, bak)
cur = json.loads(MJ.read_text(encoding="utf-8"))          # re-read just before writing
have = {e["file"] for e in cur}
add = [e for e in new if e["file"] not in have]
cur += add
MJ.write_text(json.dumps(cur, ensure_ascii=False, indent=1), encoding="utf-8")
print(f"copied {len(new)}, appended {len(add)}, backup {bak.name}, total entries {len(cur)}")
