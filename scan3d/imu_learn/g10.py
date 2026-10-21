#!/usr/bin/env python3
"""G1.0 test, 2026-10-22 night (docs/g10_plan.md, pre-registered before any G1.0 run).

G1.0 = the i3 external IMU still gate (--still-hold 1.5:0.05:0.05:i3 decision, armed 3 s after the first tracking pose)
decides per fed frame; when it is open the frame carries flag byte 17 = 1 and the G1.0 bridge (tools/ov_bridge_g10, a copy
of tools/ov_bridge_diag + per-frame zupt_max_disparity setter, same OpenVINS library) uses zupt_max_disparity 1.0 for that
frame, else the config value 0.5. No frame is held. The bench file (vio_bench_hs.py) is NOT changed: this runner wraps it
(StillHold.step -> record decision and return False; the frame header pad byte carries the flag).

base / i3 = exactly the i3_takeoff / i3_confirm command (imu_learn/z2_isolate.py job, diag bridge, threads 2).

  (setsid nohup ../venv_ffs/bin/python imu_learn/g10.py run > runs/g10/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/g10.py list | gates
  ../venv_ffs/bin/python imu_learn/g10.py analyse   # -> runs/g10/summary.json
"""
import argparse
import csv
import json
import os
import struct as _struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
import i3_confirm as ic  # noqa: E402
import i3_takeoff as it  # noqa: E402
import z2_regress as zr  # noqa: E402

PY = "../venv_ffs/bin/python"
OUT = Path("runs/g10")
LOG = Path("runs/g10.log")
PAR = int(os.environ.get("PAR", 5))
THREADS = "2"
GATE = "1.5:0.05:0.05:i3"
ALT = 1.0
G_BRIDGE = SCAN3D / "tools" / "ov_bridge_g10" / "build" / "ov_bridge"
FILES = ("imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/imu_gt.py", "imu_learn/z2_regress.py",
         "imu_learn/z2_isolate.py", "imu_learn/i3_confirm.py", "imu_learn/i3_takeoff.py", "imu_learn/g10.py",
         "make_ov_config.py", "config/mono/estimator_config.yaml", "tools/ov_bridge_g10/main.cpp",
         "tools/ov_bridge_g10/CMakeLists.txt")
DEADLINE = os.environ.get("G10_DEADLINE", "05:50")   # no new job starts after this (HH:MM, same day)
R41 = "hs_20261021_214550"
R13 = "hs_20261021_213341"


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} g10: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def blobs():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    b = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in FILES}
    dirty = subprocess.run(["git", "status", "--porcelain", *FILES], capture_output=True, text=True).stdout.strip()
    return head, b, bool(dirty)


# ---------------------------------------------------------------- cells
def cells():
    """All cells. kind: B (takeoff B s0), BA (B'a 214550), BL (B'b 214550), N (213341 +-0.25 s), R (z2_regress).
    Each cell: name, args, and per condition either an existing result prefix ('reuse') or a new run name."""
    out = []
    for cl in it.cells():
        if cl.get("skip") or cl["si"] != 0:
            continue
        args = zr.common(cl["rec"], cl["off"], 2.0, it.MAX_SECS, True)
        out.append(dict(kind="B", key=f"B_{cl['key']}", rec=cl["rec"], off=cl["off"], args=args, dep=True,
                        still_imu_ns=cl["still_imu_ns"],
                        src={"base": str(it.OUT / f"B_{cl['key']}_base"), "i3": str(it.OUT / f"B_{cl['key']}_i3")}))
    x = it.inp(R41)
    st = x["setdown"][0]["still_imu_ns"]
    for k, off in enumerate(x["offsets_s"]):
        out.append(dict(kind="BA", key=f"BA_{R41[-6:]}_o{k}", rec=R41, off=off, args=zr.common(R41, off, 2.0, 0, True), dep=True,
                        still_imu_ns=st, src={c: str(it.SRC / f"{R41[-6:]}_{c}_o{k}") for c in ("base", "i3")}))
    for b2 in it.b2_cells():
        if b2["rec"] != R41:
            continue
        out.append(dict(kind="BL", key=f"BL_{b2['key']}", rec=R41, off=b2["off"],
                        args=zr.common(R41, b2["off"], 2.0, it.MAX_SECS, True), dep=True, still_imu_ns=st,
                        src={c: str(it.OUT / f"B2_{b2['key']}_{c}") for c in ("base", "i3")}))
    for cl in it.cells():
        if cl["rec"] != R13 or cl["si"] != 0 or cl["d"] not in (1, 5):
            continue
        for dn, dd in (("m025", -0.25), ("p025", 0.25)):
            off = round(cl["off"] + dd, 3)
            out.append(dict(kind="N", key=f"N_{cl['key']}_{dn}", rec=R13, off=off,
                            args=zr.common(R13, off, 2.0, it.MAX_SECS, True), dep=True, still_imu_ns=cl["still_imu_ns"], src={}))
    for j in zr.jobs():
        if j["cond"] != "base":
            continue
        out.append(dict(kind="R", key=f"R_{j['pair']}", rec=j["rec"], args=j["args"], dep=False, src={},
                        z2_base=str(zr.OUT / j["name"])))
    return out


def gate_possible(c):
    """Conservative (superset) check whether the i3 gate can open in this clip: any fed frame >= 3 s after the first fed
    frame whose trailing 1.5 s IMU window is still (same StillHold rule and IMU arrays as the bench). If False, G1.0 sends
    flag 0 on every frame = base bit for bit (the arming needs init + 3 s >= first frame + 3 s)."""
    import vio_bench_hs as vb
    from imu_gt import read_imu
    a = c["args"]
    rec = Path(a[0])
    off = float(a[a.index("--start-off") + 1])
    ms = float(a[a.index("--max-secs") + 1])
    raw = "--imu-raw" in a
    meta = json.load(open(rec / "meta.json"))
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    tf = np.array([int(r["t_mid_ns"]) for r in rows])
    dt = int(json.load(open(rec / "imu_gt" / "align.json"))["dt_ns"])
    t0 = tf[0]
    trim = rec / "imu_gt" / "trim.json"
    if trim.exists():
        t0 = max(t0, json.load(open(trim))["start_ns"])
    t0 += int(off * 1e9)
    t1 = min(t0 + int(ms * 1e9), tf[-1]) if ms else tf[-1]
    sel = vb.pick_by_time(tf, t0, t1, 60, 1e9 / float(meta.get("save_fps") or 240))
    acc, gyr = read_imu(rec / "imu.csv", raw=raw)
    a_i = np.stack([np.interp(gyr[:, 0], acc[:, 0], acc[:, 1 + j]) for j in range(3)], 1)
    sh = vb.StillHold("1.5:0.05:0.05", gyr[:, 0].astype(np.int64), gyr[:, 1:4].copy(), a_i)
    tci = tf[sel] - dt
    jh = np.searchsorted(sh.t_imu, tci, side="right") - 1
    still = (jh >= 0) & sh.ok[np.maximum(jh, 0)]
    late = tci >= tci[0] + 3e9 - 1
    return bool((still & late).any()), int((still & late).sum())


def jobs(cs=None):
    """Priority order (plan): identity checks, 214550 G (B, B'a, B'b), other B G, 213341 +-0.25 (base first), regression
    (base then G, gate-possible cells only; F, T, then O)."""
    cs = cs or cells()
    js = [dict(name="CHK_B_215348_s0_d1_gbridge_base", cond="base_gb", kind="CHK",
               args=next(c for c in cs if c["key"] == "B_215348_s0_d1")["args"])]
    targs = next(c for c in cs if c["key"] == "R_T_214138_S1_d0")["args"]          # gate never opens in this clip
    js += [dict(name=f"CHK_T_214138_S1_d0_{c}", cond=c, kind="CHK", args=targs) for c in ("base", "G")]
    order = {"BA": 1, "BL": 1, "B": 2, "N": 3, "R": 4}
    for c in sorted(cs, key=lambda c: (order[c["kind"]], c["rec"] != R41, c["key"].startswith("R_O"), c["key"])):
        if c["kind"] in ("B", "BA", "BL"):
            js.append(dict(name=f"{c['key']}_G", cond="G", kind=c["kind"], cell=c["key"], args=c["args"]))
        elif c["kind"] == "N":
            for cond in ("base", "i3", "G"):
                js.append(dict(name=f"{c['key']}_{cond}", cond=cond, kind="N", cell=c["key"], args=c["args"],
                               needs_base=cond != "base"))
        elif c.get("gate"):
            for cond in ("base", "G"):
                js.append(dict(name=f"{c['key']}_{cond}", cond=cond, kind="R", cell=c["key"], args=c["args"],
                               needs_base=cond != "base"))
    return js


def cmd(j):
    if j["cond"] in ("base", "i3"):
        return [PY, "imu_learn/z2_isolate.py", "job", "--cfg-over", f"num_opencv_threads={THREADS}", "--still-hold",
                GATE if j["cond"] == "i3" else "", "--", *j["args"], "--out", str(OUT / j["name"])]
    mode = "G" if j["cond"] == "G" else "base_gb"
    return [PY, "imu_learn/g10.py", "job", "--mode", mode, "--", *j["args"], "--out", str(OUT / j["name"])]


# ---------------------------------------------------------------- the G1.0 job (in-process c5_run, like z2_isolate.job)
class _Flag:
    v = 0


def job(argv):
    i = argv.index("--")
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["G", "base_gb"], required=True)
    o = ap.parse_args(argv[:i])
    import c5_run
    import vio_bench_hs as vb
    c5_run.DIAG_BRIDGE = G_BRIDGE                       # G1.0 bridge (same OpenVINS library as the diag bridge)
    real_popen = vb.subprocess.Popen
    gate_log = dict(n_open=0, n_frames=0, segs=[])

    def popen(args, **kw):                              # add --zupt-alt-disp to the bridge command (G mode only)
        if str(args[0]) == str(G_BRIDGE) and o.mode == "G":
            args = list(args) + ["--zupt-alt-disp", str(ALT)]
        return real_popen(args, **kw)
    vb.subprocess.Popen = popen

    class NoHold(vb.StillHold):
        def step(self, tci, clip_s):
            open_ = super().step(tci, clip_s)
            _Flag.v = 1 if open_ else 0
            gate_log["n_frames"] += 1
            if open_:
                gate_log["n_open"] += 1
                if not gate_log["segs"] or gate_log["segs"][-1][1] is not None:
                    gate_log["segs"].append([int(tci), None])
            elif gate_log["segs"] and gate_log["segs"][-1][1] is None:
                gate_log["segs"][-1][1] = int(tci)
            return False                                # never hold a frame

    class StructProxy:
        def __getattr__(self, k):
            return getattr(_struct, k)

        @staticmethod
        def pack(fmt, *args):
            if fmt == "<qIIB3x":                         # frame header: pad byte 17 carries the gate flag
                return _struct.pack("<qIIBB2x", *args, _Flag.v)
            return _struct.pack(fmt, *args)
    if o.mode == "G":
        vb.StillHold = NoHold
        vb.struct = StructProxy()
    orig = vb.run

    def run(a):
        a.cfg_over = f"num_opencv_threads={THREADS}"
        a.still_hold = GATE if o.mode == "G" else None
        return orig(a)
    vb.run = run
    sys.argv = ["c5_run.py"] + argv[i + 1:]
    c5_run.main()
    out = sys.argv[sys.argv.index("--out") + 1]
    d = json.load(open(out + ".json"))
    head, b, dirty = blobs()
    d.update(g10_runner="imu_learn/g10.py", g10_mode=o.mode, g10_alt_disp=ALT if o.mode == "G" else None,
             g10_gate=GATE if o.mode == "G" else None, g10_gate_open_frames=gate_log["n_open"],
             g10_gate_frames=gate_log["n_frames"], g10_gate_segments_imu_ns=gate_log["segs"],
             g10_blobs=b, g10_dirty=dirty, g10_head=head)
    if o.mode == "G":                                   # bench recorded still_hold (the gate); nothing was held
        d["g10_note"] = "still_hold field = gate spec only; frames_held is 0 by construction (no holding)"
    json.dump(d, open(out + ".json", "w"), indent=1, ensure_ascii=False)


# ---------------------------------------------------------------- scheduler
def n_running():
    r = subprocess.run(["pgrep", "-fc", f"(z2_isolate|g10).py job .*--out {OUT}/"], capture_output=True, text=True)
    return int(r.stdout.strip() or 0)


def base_ok(name):
    """Feasibility gate (plan): the cell's base run has a tracking pose (init) — and for departure cells (N) the 10 s
    window after departure is inside the clip and scored. Uses only the base run."""
    p = OUT / name
    if not Path(str(p) + ".json").exists():
        return None
    d = json.load(open(str(p) + ".json"))
    return d.get("t_init_clip_s") is not None


def past_deadline():
    hh, mm = (int(v) for v in DEADLINE.split(":"))
    lt = time.localtime()
    return (lt.tm_hour, lt.tm_min) >= (hh, mm) and lt.tm_hour < 12


def run_sched():
    OUT.mkdir(parents=True, exist_ok=True)
    head, b, dirty = blobs()
    if dirty:
        log(f"REFUSED: code files not committed (git {head})")
        sys.exit(1)
    cs = cells()
    gates = {}
    for c in cs:
        if c["kind"] == "R":
            c["gate"], c["gate_n"] = gate_possible(c)
            gates[c["key"]] = [c["gate"], c["gate_n"]]
    json.dump(dict(made=time.strftime("%F %T"), git=head, gates=gates), open(OUT / "gates.json", "w"), indent=1)
    queue = [j for j in jobs(cs) if not (OUT / f"{j['name']}.json").exists()]
    log(f"start git {head} PAR {PAR} threads {THREADS} gate {GATE} alt {ALT} jobs {len(queue)} deadline {DEADLINE} "
        f"R gate-possible {sum(v[0] for v in gates.values())}/{len(gates)} blobs {b}")
    running = {}
    while queue or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                tail = (f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} scale {d.get('scale')} gate_open "
                        f"{d.get('g10_gate_open_frames')} held {d.get('frames_held')} err {d.get('error')}")
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s, load {open('/proc/loadavg').read().split()[0]}) {tail}")
        if past_deadline() and queue:
            log(f"DEADLINE {DEADLINE}: {len(queue)} jobs not started: {[j['name'] for j in queue]}")
            queue = []
        k = 0
        while k < len(queue) and n_running() < PAR:
            j = queue[k]
            if j.get("needs_base"):
                bn = f"{j['cell']}_base"
                ok = base_ok(bn)
                if ok is None:
                    if any(q["name"] == bn for q in queue) or bn in running:
                        k += 1
                        continue                        # wait for the base run of this cell
                elif not ok:
                    log(f"skip {j['name']}: base {bn} no init -> cell not tested")
                    queue.pop(k)
                    continue
            queue.pop(k)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
            log(f"start {j['name']} ({len(queue)} queued)")
            time.sleep(0.5)
        time.sleep(10)
    log("end (queue done)")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "job":
        return job(sys.argv[2:])
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "list", "gates", "analyse"])
    o = ap.parse_args()
    if o.cmd == "run":
        return run_sched()
    if o.cmd == "gates":
        cs = cells()
        n = 0
        for c in cs:
            if c["kind"] == "R":
                g, k = gate_possible(c)
                n += g
                print(c["key"], g, k)
        print("gate-possible", n)
        return
    if o.cmd == "list":
        for j in jobs():
            print(j["name"], " ".join(cmd(j)))
        return
    import g10_analyse as ga
    res = ga.analyse()
    json.dump(res, open(OUT / "summary.json", "w"), indent=1, ensure_ascii=False, default=float)
    print(json.dumps(res["verdict"], ensure_ascii=False, default=float, indent=1))


if __name__ == "__main__":
    main()
