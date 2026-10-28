#!/usr/bin/env python3
"""appnpu error test scorer (pre-registered in README "오차 시험" before any run).
  python score_err.py err/<tag>            -> err/<tag>/score.json + one Korean summary line
  python score_err.py --abba err/r1 err/r2 err/r3 err/r4   -> comparison by the README rule (abba.json)

Inputs (run folder pulled by run_err.sh): ov_live.jsonl (VIO pose lines "p", status "tracking"; fallback poses_pred.csv px..pz),
loop.jsonl (kf / loop / opt / xf_down lines), npu.jsonl, meta.json, bridge_stderr.log, temps_pc.txt.
Metrics (OpenVINS world, z = up):
  start = mean VIO position of the first 0.5 s of tracking; end = mean of the last 2.0 s of tracking (user holds still on the mark)
  vio_end_cm = |end - start|; map_end_cm = |Rz(yaw) end + t - start| with (yaw, t) = the LAST optimiser target ("opt" tgt_yaw /
    tgt_t; the published value is a rate-limited follower of it) - no loop accepted -> map = VIO
  path_m = VIO path length from positions resampled to 10 Hz; *_pct = end error / path * 100
  z_drift_max_cm = max |z - z_start| over the run (the mark is at the same height, but the hand moves -> upper bound only)
  loops: candidates verified ok / accepted (loop lines) / rejected (verified but not accepted); optimisations
  NPU: XFeat per keyframe (npu / post_q / tot us), LighterGlue per loop candidate (npu / post us), mutual NN (post us)
  validity: meta run_valid / npu_failed / bridge_exit, xf_down lines, bridge queue drops, max SKIN / AP
All numbers are single runs (미검증); there is no ground truth here, only the return-to-mark error."""
import csv, glob, json, math, os, re, sys, time


def pct(v, q):
    if not v:
        return None
    v = sorted(v)
    return v[min(len(v) - 1, int(q * (len(v) - 1) + 0.5))]


def rz(yaw, p):
    c, s = math.cos(yaw), math.sin(yaw)
    return [c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]]


def norm(a):
    return math.sqrt(sum(x * x for x in a))


def load_poses(run):
    """[(t_s, [x, y, z])] of tracking VIO poses"""
    out = []
    f = os.path.join(run, "ov_live.jsonl")
    if os.path.isfile(f):
        for l in open(f):
            try:
                d = json.loads(l)
            except Exception:
                continue
            if d.get("src") == "ov" and d.get("status") == "tracking" and isinstance(d.get("p"), list) and d.get("t_sensor"):
                out.append((d["t_sensor"] * 1e-9, d["p"]))
    if len(out) < 10:
        f = os.path.join(run, "poses_pred.csv")
        out = []
        if os.path.isfile(f):
            seen = set()
            for r in csv.DictReader(open(f)):
                if r.get("status") != "tracking" or r["t_cam_ns"] in seen:
                    continue
                seen.add(r["t_cam_ns"])   # one prediction per camera frame (the first one = closest to the frame)
                out.append((int(r["t_cam_ns"]) * 1e-9, [float(r["px"]), float(r["py"]), float(r["pz"])]))
        src = "poses_pred.csv"
    else:
        src = "ov_live.jsonl"
    out.sort(key=lambda x: x[0])
    return out, src


def score(d):
    runs = sorted(glob.glob(os.path.join(d, "run_*")))
    if not runs:
        raise SystemExit(f"no run_* in {d}")
    run = runs[-1]
    meta = json.load(open(os.path.join(run, "meta.json"))) if os.path.isfile(os.path.join(run, "meta.json")) else {}
    em = json.load(open(os.path.join(d, "err_meta.json"))) if os.path.isfile(os.path.join(d, "err_meta.json")) else {}
    P, src = load_poses(run)
    r = dict(made=time.strftime("%F %T"), dir=d, run=run, tag=em.get("tag"), npu_loop=em.get("npu_loop", 1 if meta.get("npu_loop") else 0),
             pose_source=src, n_poses=len(P), npu_state=meta.get("npu_loop"), run_valid=meta.get("run_valid"),
             npu_failed=meta.get("npu_failed"), bridge_exit=meta.get("bridge_exit"), stop_reason=meta.get("stop_reason"),
             secs=meta.get("secs"))
    if len(P) >= 20:
        t0, t1 = P[0][0], P[-1][0]
        st = [p for t, p in P if t <= t0 + 0.5]
        en = [p for t, p in P if t >= t1 - 2.0]
        start = [sum(c) / len(st) for c in zip(*st)]
        end = [sum(c) / len(en) for c in zip(*en)]
        path, last, nxt = 0.0, P[0][1], t0
        for t, p in P:
            if t >= nxt:
                path += norm([a - b for a, b in zip(p, last)])
                last, nxt = p, t + 0.1
        yaw, tt, nopt = 0.0, [0.0, 0.0, 0.0], 0
        lj = os.path.join(run, "loop.jsonl")
        loops = cand_ok = cand_all = 0
        xf_down = []
        if os.path.isfile(lj):
            for l in open(lj):
                try:
                    j = json.loads(l)
                except Exception:
                    continue
                if "xf_down" in j:
                    xf_down.append(j)
                elif j.get("type") == "opt":
                    yaw, tt, nopt = j["tgt_yaw"], j["tgt_t"], nopt + 1
                elif j.get("type") == "loop":
                    loops += 1
                elif j.get("type") == "kf":
                    for c in j.get("cand", []):
                        cand_all += 1
                        cand_ok += c[4] == "ok"
        mend = [a + b for a, b in zip(rz(yaw, end), tt)]
        vio_e = norm([a - b for a, b in zip(end, start)]) * 100
        map_e = norm([a - b for a, b in zip(mend, start)]) * 100
        r.update(track_s=round(t1 - t0, 2), path_m=round(path, 3), vio_end_cm=round(vio_e, 2), map_end_cm=round(map_e, 2),
                 vio_end_pct=round(vio_e / max(path * 100, 1e-9) * 100, 3), map_end_pct=round(map_e / max(path * 100, 1e-9) * 100, 3),
                 z_drift_max_cm=round(max(abs(p[2] - start[2]) for _, p in P) * 100, 2),
                 loop_candidates=cand_all, loop_verified=cand_ok, loops_accepted=loops, loops_rejected=cand_ok - loops,
                 optimisations=nopt, map_final_yaw_deg=round(math.degrees(yaw), 3), map_final_t_cm=[round(x * 100, 2) for x in tt],
                 xf_down=xf_down)
    else:
        r.update(error="fewer than 20 tracking poses (VIO did not initialise / run too short)")
    # NPU
    nj = os.path.join(run, "npu.jsonl")
    F, M2, M1 = [], [], []
    if os.path.isfile(nj):
        for l in open(nj):
            try:
                j = json.loads(l)
            except Exception:
                continue
            if j.get("op") == "F" and "us" in j:
                F.append(j)
            elif j.get("op") == "M" and "us" in j:
                (M2 if j.get("mode") == 2 else M1).append(j)
    q = lambda L, k: dict(n=len(L), p50=pct([x["us"][k] for x in L], .5), p95=pct([x["us"][k] for x in L], .95)) if L else None
    r["npu"] = dict(xfeat_npu_us=q(F, "npu"), xfeat_post_us=q(F, "post_q"), xfeat_tot_us=q(F, "tot"), lg_npu_us=q(M2, "npu"),
                    lg_post_us=q(M2, "post"), lg_tot_us=q(M2, "tot"), nn_post_us=q(M1, "post"),
                    lg_matches_p50=pct([x["n"] for x in M2], .5), evicted=max([x.get("evicted", 0) for x in F], default=0))
    be = os.path.join(run, "bridge_stderr.log")
    m = re.search(r"keyframes pushed (\d+) dropped (\d+) processed (\d+)", open(be, errors="replace").read()) if os.path.isfile(be) else None
    r["keyframes"] = dict(pushed=int(m.group(1)), dropped=int(m.group(2)), processed=int(m.group(3))) if m else None
    sk, ap = [], []
    tp = os.path.join(d, "temps_pc.txt")
    if os.path.isfile(tp):
        for l in open(tp):
            a = re.search(r"SKIN ([0-9.]+) AP ([0-9.]+)", l)
            if a:
                sk.append(float(a.group(1)))
                ap.append(float(a.group(2)))
    r["temps"] = dict(skin_max=max(sk, default=None), ap_max=max(ap, default=None), n=len(sk),
                      safety_stop=os.path.isfile(tp) and "SAFETY STOP" in open(tp).read())
    r["valid"] = bool(r.get("path_m")) and r.get("bridge_exit") == "0" and (r["npu_loop"] == 0 or r.get("run_valid") is True) \
        and not r.get("xf_down") and not r["temps"]["safety_stop"]
    json.dump(r, open(os.path.join(d, "score.json"), "w"), indent=1)
    n = r["npu"]
    xs = f", XFeat NPU {n['xfeat_npu_us']['p50'] / 1000:.1f} ms + 후처리 {n['xfeat_post_us']['p50'] / 1000:.1f} ms" if n["xfeat_npu_us"] else ""
    ls = f", LG NPU {n['lg_npu_us']['p50'] / 1000:.1f} ms (n {n['lg_npu_us']['n']})" if n["lg_npu_us"] else ""
    if r.get("path_m"):
        print(f"[{r['tag']}] npu {r['npu_loop']} {'유효' if r['valid'] else '무효'}: 경로 {r['path_m']:.2f} m, 끝점 오차 지도 {r['map_end_cm']:.1f} cm "
              f"({r['map_end_pct']:.2f} %) / VIO {r['vio_end_cm']:.1f} cm, 높이 최대 {r['z_drift_max_cm']:.1f} cm, 루프 채택 {r['loops_accepted']} "
              f"(검증 {r['loop_verified']}/후보 {r['loop_candidates']}){xs}{ls}, KF 버림 {(r['keyframes'] or {}).get('dropped')}, "
              f"SKIN 최대 {r['temps']['skin_max']} AP 최대 {r['temps']['ap_max']} (1 회, 미검증)")
    else:
        print(f"[{r['tag']}] npu {r['npu_loop']} 무효: {r.get('error')} (run_valid {r.get('run_valid')}, stop {r.get('stop_reason')})")
    return r


def abba(dirs):
    """README rule: order npu1, npu0, npu0, npu1; all four valid; primary = map_end_pct."""
    R = [score(d) if not os.path.isfile(os.path.join(d, "score.json")) else json.load(open(os.path.join(d, "score.json"))) for d in dirs]
    order = [r["npu_loop"] for r in R]
    out = dict(made=time.strftime("%F %T"), dirs=dirs, order=order, rows=[{k: r.get(k) for k in ("tag", "npu_loop", "valid", "path_m",
               "map_end_cm", "map_end_pct", "vio_end_cm", "loops_accepted")} for r in R])
    if order != [1, 0, 0, 1]:
        out["verdict"] = "규칙 위반: 순서가 npu1, npu0, npu0, npu1 이 아님 -> 판정 안 함"
    elif not all(r["valid"] for r in R):
        out["verdict"] = "무효 실행 있음 -> 판정 안 함 (다시 4 회)"
    else:
        a = [r["map_end_pct"] for r in R if r["npu_loop"] == 1]
        b = [r["map_end_pct"] for r in R if r["npu_loop"] == 0]
        if max(a) < min(b):
            out["verdict"] = f"NPU 쪽이 낫다 (두 NPU 실행 모두 두 BRIEF 실행보다 작음: {a} vs {b} %), 4 회뿐 - 추정"
        elif max(b) < min(a):
            out["verdict"] = f"BRIEF 쪽이 낫다 ({b} vs {a} %), 4 회뿐 - 추정"
        else:
            out["verdict"] = f"차이 보이지 않음 (겹침: NPU {a} / BRIEF {b} %)"
    json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(dirs[0])), "abba.json"), "w"), indent=1)
    print("ABBA:", out["verdict"])


if __name__ == "__main__":
    if sys.argv[1] == "--abba":
        abba(sys.argv[2:6])
    else:
        score(sys.argv[1])
