#!/usr/bin/env python3
"""gsmap 2026-11-26 W2: run the R ruler (gsm_rjudge.py geo / photo, unchanged code) on another map set, into another folder.
  python gsm_w2judge.py geo|photo OUTDIR name=MESH(.ply|.glb):POSES [...]      POSES = bap | ba2 | ba3
PAIRS = every map minus the first one (the baseline). Never writes into r_judge."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gsm_rjudge as rj  # noqa: E402


def main():
    mode, out = sys.argv[1], Path(sys.argv[2]); assert out.resolve() != rj.OUT.resolve(), "never overwrite r_judge"
    maps = {}
    for s in sys.argv[3:]:
        nm, rest = s.split("=", 1); p, poses = rest.rsplit(":", 1); maps[nm] = (Path(p), poses)
    names = list(maps); rj.MAPS = maps; rj.OUT = out; rj.PAIRS = tuple((m, names[0]) for m in names[1:])
    out.mkdir(parents=True, exist_ok=True)
    dict(geo=rj.cmd_geo, photo=rj.cmd_photo)[mode]()


if __name__ == "__main__":
    main()
