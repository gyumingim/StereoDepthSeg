#!/usr/bin/env python3
"""u16 (11-13): synthetic test input = a reduced keyframe folder whose live VIO poses are scaled (positions x S about the origin).

  python u16_scale_part.py SRC_RED_DIR DST_DIR 1.05

frames_kf/ is hard-linked, the other files copied; phone_live_poses.jsonl.gz positions multiplied by S (rotations unchanged),
lensonly/gt.npz copied unchanged (merge_align uses only its K, dist, t_ns). Writes DST/u16_scaled.json (made, source, S).
"""
import gzip, json, os, shutil, sys, time
from pathlib import Path

src, dst, S = Path(sys.argv[1]), Path(sys.argv[2]), float(sys.argv[3])
if dst.exists():
    shutil.rmtree(dst)
dst.mkdir(parents=True)
for f in src.iterdir():
    if f.name == "frames_kf":
        (dst / "frames_kf").mkdir()
        for j in f.iterdir():
            os.link(j, dst / "frames_kf" / j.name)
    elif f.is_dir():
        shutil.copytree(f, dst / f.name)
    elif f.name != "phone_live_poses.jsonl.gz":
        shutil.copy2(f, dst / f.name)
n = 0
with gzip.open(src / "phone_live_poses.jsonl.gz", "rt") as fi, gzip.open(dst / "phone_live_poses.jsonl.gz", "wt") as fo:
    for line in fi:
        m = json.loads(line)
        if m.get("T_world_cam0"):
            T = m["T_world_cam0"]
            for i in (3, 7, 11):
                T[i] *= S
            n += 1
        fo.write(json.dumps(m) + "\n")
(dst / "u16_scaled.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), source=str(src), scale=S, poses_scaled=n), indent=1))
print(f"scaled {n} poses x {S} -> {dst}")
