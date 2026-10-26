#!/usr/bin/env python3
"""gyrocheck 2026-10-27: NL60 vs GL60 with small start-offset shifts (PLAN.md).
Imports runs/integ2_20261026/integ2_ate.py UNCHANGED (read only) and reuses its CONDS / job arguments; the only change is
start_off (+k frames) and frame_cache=False. Bridge = frozen copy of ov_bridge_c640f5344f35 (both conditions).
usage: gc_run.py freeze | job <key>_k<k>_<cond> | run <par> | score"""
import os, sys, json, gzip, time, subprocess, hashlib, shutil
sys.dont_write_bytecode = True
OUTD = "/home/system/scanner/tmp_gyrocheck/ate"
os.environ["OV60_OUT"] = OUTD
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-integ2/scan3d/runs/integ2_20261026")
import integ2_ate as R  # noqa: E402  (chdir MAIN, sys.path)
from pathlib import Path  # noqa: E402
OUT = Path(OUTD)
BR = Path("/home/system/scanner/tmp_dpvo/ov/bin/ov_bridge_c640f5344f35")
SHA = "c640f5344f35c974aec7ac625fa97e38b7e7153e5d88f1a589405e4ce898f03c"
RD = Path("/home/system/scanner/StereoDepthSeg-gyrocheck/scan3d/runs/gyrocheck_20261027")
CONDS = ["NL60", "GL60"]
RECS = [("loop", "hs_20261024_142148_std60"), ("loop", "hs_20261024_142610_std60"), ("loop", "hs_20261024_151646_std60"),
        ("std", "hs_20261024_165506_std60"), ("hs", None, "131326"), ("hs", None, "154320")]
KS = range(5)
# supervisor 23:31 C5 (post hoc): same NL60 / GL60 runs + --viop-udp to a local UDP listener -> the REAL VIOP POSE /
# HEARTBEAT frames (per-axis pos_std_mm) for the fc_safety arm replay. Output only (VIO poses checked identical).
VCONDS = {"NL60V": "NL60", "GL60V": "GL60"}
EXTRA = ["s10", "s20", "s30", "s50"]  # POST HOC (supervisor C3): start shifts 0.1-0.5 s for the 60 fps recordings
for _v, _b in VCONDS.items():
    R.CONDS[_v] = dict(R.CONDS[_b])


def rec_list():
    out = []
    for x in RECS:
        which = x[0]
        if which == "hs":
            rec = next(r for r, o in R.cells("hs") if r.split("_")[2] == x[2])
        else:
            rec = x[1]
        out.append((which, rec))
    return out


def key_of(which, rec):
    return R.key_of(rec, 0, which).rsplit("_", 1)[0]  # e.g. l142148 / s165506 / 131326


def jobs():
    q = []
    for k in KS:  # k outer: early results cover all recordings
        for which, rec in rec_list():
            for c in CONDS:
                q.append(f"{key_of(which, rec)}_k{k}_{c}")
    # 154320 is the slowest -> start those first within each k? keep simple order
    return q


def freeze():
    (OUT / "bin").mkdir(parents=True, exist_ok=True)
    h = hashlib.sha256(open(BR, "rb").read()).hexdigest()
    assert h == SHA, h
    dst = OUT / "bin" / BR.name
    if not dst.exists():
        shutil.copy2(BR, dst)
    json.dump(dict(src=str(BR), sha256=h, copy=str(dst), frozen=time.strftime("%F %T"),
                   note="0.6.27 PC build (integ2 build_integ2_full at 15:22 10-26, bridge.json of tmp_dpvo/ov: worktree_head 0b469ac)"),
              open(OUT / "bin" / "bridge.json", "w"), indent=1)
    for c in CONDS:
        w = R.wrapper(c)
        w.write_text(f"#!/bin/bash\nexec {dst} \"$@\" {R.CONDS[c]['args']}\n")
        w.chmod(0o755)
    print("frozen", dst, h)


def job(name):
    import argparse, tempfile as _tf, types
    import vio_bench_hs as vb
    key, pk, cond = name.split("_")
    k = int(pk[1:])
    which = R.which_of(key)
    rec = next(r for w, r in rec_list() if key_of(w, r) == key)
    start_off = k / 240.0 if which == "hs" else k / 60.0
    if pk[0] == "s":  # POST HOC (supervisor C3): larger shifts s<N> = N/100 s (60 fps recordings only)
        start_off = k / 100.0
    _, _, old = R.hs_cells()
    d = R.CONDS[cond]
    vb.BRIDGE = R.wrapper(cond)
    vb.TEMPLATE = R.MAIN / "config" / "mono" / "estimator_config.yaml"
    (OUT / "tmp").mkdir(parents=True, exist_ok=True)
    vb.tempfile = types.SimpleNamespace(mkdtemp=lambda prefix="vio_", dir=None: _tf.mkdtemp(prefix=prefix, dir=str(OUT / "tmp")))
    cfg = R.BASE_CFG + ("," + d["cfg"] if d.get("cfg") else "")
    a = argparse.Namespace(rec=f"recordings/{rec}", fps=d["fps"], size="960x540", npts=200, clones=d.get("clones", 15),
                           noise="cfg", noise_scale=1.0, imu_calib=None, lever=R.LEVER, lens_file=None, dropout="",
                           max_secs=0.0, imu_raw=rec not in old, gt="none" if which == "loop" else "auto", out=None,
                           init_cfg="strict", start_off=start_off, decode_threads=2, imu_lead=2.0, init_window=1.0,
                           init_disp=22.5, frame_cache=False, gyro_rebias=False, cfg_over=cfg, still_hold=None)
    saved = {}
    orig = vb.evaluate

    def ev(rec_, a_, poses, *rest):
        saved["poses"] = poses
        return orig(rec_, a_, poses, *rest)
    vb.evaluate = ev
    t0 = time.time()
    frames, stop = [], [False]
    if cond in VCONDS:
        import socket, threading
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.bind(("127.0.0.1", 0))
        sk.settimeout(0.2)
        os.environ["GC_VIOP_PORT"] = str(sk.getsockname()[1])

        def rx():
            while not stop[0]:
                try:
                    b = sk.recv(4096)
                except socket.timeout:
                    continue
                frames.append((time.time_ns(), b))
        th = threading.Thread(target=rx, daemon=True)
        th.start()
    try:
        r = vb.run(a)
    finally:
        vb.evaluate = orig
        if cond in VCONDS:
            time.sleep(0.5)
            stop[0] = True
            th.join(2)
            od0 = OUT / cond
            od0.mkdir(parents=True, exist_ok=True)
            with gzip.open(str(od0 / name) + ".viop.bin.gz", "wb") as f:
                for tw, b in frames:
                    f.write(len(b).to_bytes(2, "little") + tw.to_bytes(8, "little") + b)
    r.update(gc_made=time.strftime("%F %T"), gc_job_s=round(time.time() - t0, 1), gc_cond=cond, gc_cond_def=d, gc_k=k,
             gc_start_off=start_off, gc_rec=rec, gc_set=which, gc_bridge_sha256=SHA, gc_load=list(os.getloadavg()))
    od = OUT / cond
    od.mkdir(parents=True, exist_ok=True)
    P = saved.get("poses")
    if P is not None:
        with gzip.open(str(od / name) + ".poses.jsonl.gz", "wt") as f:
            for m in P:
                f.write(json.dumps(m) + "\n")
    json.dump(r, open(od / f"{name}.json", "w"), indent=1, ensure_ascii=False, default=str)


def run(par):
    q = [n for n in jobs() if not (OUT / n.split("_")[2] / f"{n}.json").exists()]
    running = {}
    (OUT / "logs").mkdir(parents=True, exist_ok=True)
    print(f"{time.strftime('%F %T')} start {len(q)} jobs PAR {par}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / n.split("_")[2] / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED rc=%s' % p.returncode}", flush=True)
        if os.path.exists("/home/system/scanner/tmp_gyrocheck/STOP"):
            if not running:
                print("STOP file, exit", flush=True); return
            q = []
        while q and len(running) < par:
            n = q.pop(0)
            lg = open(OUT / "logs" / f"{n}.log", "w")
            p = subprocess.Popen(["nice", "-n", "5", R.PY, os.path.abspath(__file__), "job", n], stdout=lg, stderr=subprocess.STDOUT,
                                 env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
            running[n] = (p, time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} ALL DONE", flush=True)


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "freeze":
        freeze()
    elif cmd == "job":
        job(sys.argv[2])
    elif cmd == "run":
        run(int(sys.argv[2]) if len(sys.argv) > 2 else 3)
    elif cmd == "runv":  # k0 hand loops with VIOP capture (sequential par from argv)
        jobs = lambda: [f"{G}_k0_{c}" for G in ("l142148", "l142610", "l151646") for c in VCONDS]
        run(int(sys.argv[2]) if len(sys.argv) > 2 else 3)
    elif cmd == "runs":  # POST HOC larger start shifts for the 60 fps recordings
        jobs = lambda: [f"{G}_{p}_{c}" for p in EXTRA for G in ("l142148", "l142610", "l151646", "s165506") for c in CONDS]
        run(int(sys.argv[2]) if len(sys.argv) > 2 else 3)
    elif cmd == "list":
        print("\n".join(jobs()))
