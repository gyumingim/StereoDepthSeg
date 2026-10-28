#!/usr/bin/env python3
"""lgin 2026-10-29: LighterGlue input A/B on the live path (PC replay through the phone c4f60 config + PC bridge, PC fp32 server).
  python3 lgin_run.py run SET [--par 4]      SET = main | expl | smoke
  python3 lgin_run.py score                  (score_rep.py pc + cand_stats.py on every finished job)
Jobs: (recording, window, condition, loop-sync, repeat). Output WSL ~/scanner/tmp_lgin/<cond>/<rec>_s<sync>_r<k>.*
Refuses to run unless README.md + this file + xf_server_lgin.py are committed and clean (pre-registration gate)."""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HOME = Path.home()
WT = HOME / "scanner/StereoDepthSeg-lgin"
RD = WT / "scan3d/runs/lgin_20261029"
OUT = Path(os.environ.get("LGIN_OUT", str(HOME / "scanner/tmp_lgin")))
PY = str(HOME / "scanner/StereoDepthSeg/venv_ffs/bin/python")
RECS = HOME / "scanner/StereoDepthSeg/scan3d/recordings"
CFG = HOME / "scanner/tmp_lens05/new_gen/c4f60"                 # phone c4f60 folder (= npunight PC reference)
BRIDGE = HOME / "scanner/tmp_appnpu/ident/ov_bridge_new"         # PC build of appnpu ov_bridge (= npunight PC reference)
VOC = WT / "scan3d/ov_bridge/loop/voc_k10L4.voc"
SRV = RD / "xf_server_lgin.py"
XFPP = "PYTHONPATH=/home/system/scanner/tmp_ovxfeat/pylib"
GATES = "xf_mode=2,xf_tri=1,xf_tri_deg=3,xf_tri_mode=1,xf_min_inl=40,xf_rp_deg=2,max_cand=1,log_cand=1"   # app BRIDGE_CFG (= FLGc) + log
SRV_COMMON = "--topk 512 --min-cos 0.82 --lg-conf 0.1 --det-th 0.05 --lg-width -1 --lg-depth -1 --dev cuda"   # NPU-like: no pruning, no early exit

# recordings and windows (s, 0 = whole) = npunight PC reference windows
MAIN = [("hs_20261024_165506_std60", 0), ("hs_20261021_213341", 40), ("hs_20261021_214550", 40), ("hs_20261021_215348", 30),
        ("hs_20261020_221925", 30), ("hs_20261020_222747", 30), ("hs_20261020_223815", 30), ("hs_20261020_224323", 30)]
EXPL_FULL = [(r, 0) for r, _ in MAIN[1:]]          # HS recordings, whole length (larger revisits), exploratory

CONDS = {
    "A": "--lg-in x512 --pad32 1",                 # XFeat 512 only (app x512, 960x544)
    "B": "--lg-in trk --pad32 1",                  # tracks + XFeat 512, all points (FLGc input, 960x544)
    "Bm": "--lg-in trkm --lg-cap 768 --pad32 1",   # app trk768 masked-model semantics (both sides min(nA, nB), cap 768) - exploratory
    "A640": "--lg-in x512 --w 640 --h 360",        # FLGc XFeat resolution - exploratory
    "B640": "--lg-in trk --w 640 --h 360",
}


def jobs(which):
    J = []
    if which == "smoke":
        return [("hs_20261024_165506_std60", 12, c, s, 1) for c in ("A", "B") for s in (1,)]
    if which == "main":
        for rec, w in MAIN:
            # ABBA in time per recording: sync 0 A1 B1 B2 A2, then sync 1 A B (deterministic reference)
            J += [(rec, w, "A", 0, 1), (rec, w, "B", 0, 1), (rec, w, "B", 0, 2), (rec, w, "A", 0, 2), (rec, w, "A", 1, 1), (rec, w, "B", 1, 1)]
    if which == "expl":
        for rec, w in MAIN:
            J += [(rec, w, c, 1, 1) for c in ("Bm", "A640", "B640")]
        for rec, w in EXPL_FULL:
            J += [(rec, w, c, 1, 1) for c in ("A", "B")]
    return J


def tag(rec, w, c, s, k):
    short = rec.split("_")[2]
    return OUT / c / f"{short}_w{w}_s{s}_r{k}"


def git(*a):
    return subprocess.run(["git", "-C", str(WT), *a], capture_output=True, text=True).stdout.strip()


def gate():
    for f in ("scan3d/runs/lgin_20261029/README.md", "scan3d/runs/lgin_20261029/lgin_run.py", "scan3d/runs/lgin_20261029/xf_server_lgin.py",
              "scan3d/runs/lgin_20261029/cand_stats.py"):
        if not (WT / f).exists() or git("status", "--porcelain", f) or not git("log", "-1", "--format=%H", "--", f):
            raise SystemExit(f"gate: {f} missing / uncommitted")
    return git("rev-parse", "HEAD")


def one(j):
    rec, w, c, s, k = j
    pre = tag(rec, w, c, s, k)
    if (Path(str(pre) + ".json")).exists():
        return f"skip {pre}"
    pre.parent.mkdir(parents=True, exist_ok=True)
    srv = f"{XFPP} {PY} {SRV} {SRV_COMMON} {CONDS[c]} --log {pre}.xf.jsonl"
    extra = f'--loop-sync {s} --loop-xf "{srv}" --loop-cfg {GATES}'
    cmd = [PY, str(RD / "appnpu_replay.py"), str(RECS / rec), str(CFG), str(BRIDGE), str(pre), "--extra", extra, "--vocab", str(VOC)]
    if w:
        cmd += ["--max-secs", str(w)]
    t0 = time.time()
    with open(str(pre) + ".log", "w") as lf:
        p = subprocess.run(["nice", "-n", "5", *cmd], stdout=lf, stderr=subprocess.STDOUT)
    json.dump(dict(job=list(j), cmd=cmd, exit=p.returncode, wall_s=round(time.time() - t0, 1), made=time.strftime("%F %T")),
              open(str(pre) + ".job.json", "w"), indent=1)
    return f"{'ok' if p.returncode == 0 else 'FAILED'} {pre} {time.time() - t0:.0f}s"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what")
    ap.add_argument("set", nargs="?", default="main")
    ap.add_argument("--par", type=int, default=4)
    a = ap.parse_args()
    if a.what == "run":
        head = gate() if a.set != "smoke" else git("rev-parse", "HEAD")
        OUT.mkdir(parents=True, exist_ok=True)
        meta = dict(set=a.set, head=head, bridge=str(BRIDGE), bridge_sha256=hashlib.sha256(open(BRIDGE, "rb").read()).hexdigest(),
                    server_sha256=hashlib.sha256(open(SRV, "rb").read()).hexdigest(), gates=GATES, srv_common=SRV_COMMON, conds=CONDS,
                    started=time.strftime("%F %T"))
        json.dump(meta, open(OUT / f"run_{a.set}.json", "w"), indent=1)
        J = jobs(a.set)
        print(f"{len(J)} jobs, par {a.par}", flush=True)
        # keep per-recording ABBA order: jobs of one recording run in sequence; recordings in parallel
        by = {}
        for j in J:
            by.setdefault((j[0], j[1]), []).append(j)

        def seq(L):
            return [one(j) for j in L]
        with ThreadPoolExecutor(a.par) as ex:
            for res in ex.map(seq, by.values()):
                for r in res:
                    print(time.strftime("%T"), r, flush=True)
        print("ALL DONE", flush=True)
    elif a.what == "score":
        for d in sorted(OUT.glob("*/")):
            for jf in sorted(d.glob("*.job.json")):
                pre = str(jf)[:-len(".job.json")]
                job = json.load(open(jf))["job"]
                if not os.path.exists(pre + ".json"):
                    print("no result", pre)
                    continue
                subprocess.run([PY, str(WT / "scan3d/runs/npunight_20261029/score_rep.py"), "pc", pre, str(RECS / job[0])])
                subprocess.run([PY, str(RD / "cand_stats.py"), pre, str(RECS / job[0])])


if __name__ == "__main__":
    main()
