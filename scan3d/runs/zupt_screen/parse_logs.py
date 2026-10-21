#!/usr/bin/env python3
"""zupt_screen step 1 (post-hoc, analysis only, 2026-10-22): parse every OpenVINS stderr under runs/ that has per-frame
ZUPT disparity lines. Output runs/zupt_screen/frames.pkl: per run rec, config, t_imu_ns, disp, nf, passed."""
import gzip, json, re, glob, os, time, pickle
from pathlib import Path
from collections import Counter
from multiprocessing import Pool
import numpy as np

ROOT = Path(__file__).resolve().parents[2]          # scan3d
ANSI = re.compile(r"\x1b\[[0-9;]*m")
RX = re.compile(r"\(([-0-9.]+) [<>] ([0-9.]+), (\d+) features\)")


def one(err):
    pre = err[:-len(".stderr.log.gz")] if err.endswith(".gz") else err[:-len(".stderr.log")]
    js, ps = pre + ".json", pre + ".poses.jsonl.gz"
    if not (os.path.exists(js) and os.path.exists(ps)):
        return dict(run=pre, skip="no json/poses")
    try:
        d = json.load(open(js))
    except Exception as e:
        return dict(run=pre, skip=f"json {e}")
    op = gzip.open if err.endswith(".gz") else open
    rows, cur, thr = [], None, set()
    try:
        for line in op(err, "rt", errors="replace"):
            if "[DIAGT]" in line:
                try:
                    cur = float(ANSI.sub("", line).split("[DIAGT]")[1])
                except ValueError:
                    cur = None
            elif cur is not None and "[ZUPT]:" in line and "disparity (" in line:
                m = RX.search(ANSI.sub("", line))
                if m:
                    rows.append((cur, "passed" in line, float(m[1]), int(m[3])))
                    thr.add(float(m[2]))
    except (EOFError, OSError) as e:
        if not rows:
            return dict(run=pre, skip=f"read {e}")
    if not rows:
        return dict(run=pre, skip="no-zupt-lines")
    tc, tn = [], []
    try:
        for x in gzip.open(ps, "rt"):
            m = json.loads(x)
            if m.get("type", "pose") == "pose" and m.get("t") is not None:
                tc.append(m["t"]); tn.append(m["t_ns"])
    except Exception as e:
        return dict(run=pre, skip=f"poses {e}")
    if len(tc) < 2:
        return dict(run=pre, skip="no-poses")
    tc, tn = np.array(tc, float), np.array(tn, float)
    o = np.argsort(tc)
    a = np.array(rows, float)
    t_imu = np.interp(a[:, 0], tc[o], tn[o])
    held = bool(d.get("frames_held")) or bool(d.get("still_hold")) or bool(d.get("z2i_still_hold")) \
        or bool(d.get("hold_segments_imu_ns"))
    cu = d.get("config_used") or {}
    return dict(run=os.path.relpath(pre, ROOT / "runs"), rec=d.get("rec"), fps=d.get("fps"), size=d.get("size"),
                npts=d.get("npts"), dropout=d.get("dropout") or "", held=held, thr=sorted(thr),
                align_dt_ms=d.get("align_dt_ms"), cfg_over=d.get("cfg_over"), cfg=str(cu)[:300],
                t=t_imu.astype(np.int64), disp=a[:, 2].astype(np.float32), nf=a[:, 3].astype(np.int32), ok=a[:, 1] > 0.5)


if __name__ == "__main__":
    errs = sorted(glob.glob(str(ROOT / "runs" / "**" / "*.stderr.log*"), recursive=True))
    errs = [e for e in errs if e.endswith(".stderr.log.gz") or e.endswith(".stderr.log")]
    print(len(errs), "stderr files", flush=True)
    with Pool(8) as p:
        res = p.map(one, errs, chunksize=4)
    keep = [r for r in res if "skip" not in r]
    skip = [r for r in res if "skip" in r]
    print("parsed", len(keep), "skipped", len(skip))
    print(Counter(s["skip"][:20] for s in skip))
    print("skipped dirs", Counter(os.path.relpath(s["run"], ROOT / "runs").split("/")[0] for s in skip))
    pickle.dump(dict(made=time.strftime("%F %T"), runs=keep, skipped=[(s["run"], s["skip"]) for s in skip]),
                open(ROOT / "runs/zupt_screen/frames.pkl", "wb"))
    c = Counter((r["rec"], r["fps"], r["size"], r["npts"], r["dropout"], r["held"], tuple(r["thr"])) for r in keep)
    for k, v in sorted(c.items(), key=lambda kv: str(kv[0])):
        print(v, k)
