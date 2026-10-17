#!/usr/bin/env python3
"""녹화로 위치 추적(VIO)만 빠르게 돌려 보는 도구 — 깊이·지도 없이 브리지 포즈만 (설정·브리지 인자 비교용).

    venv_ffs/bin/python scan3d/tools/vio_track_eval.py scan3d/recordings/rec_20261016_175446 \
        [--ov-config-mono 설정.yaml] [--replay-bridge-args "--auto-reset --reinit-state"] [--json out.json]

출력 (한 줄 JSON): lost/리셋 시각·종류, 추적 중 프레임 비율, 특징 0 프레임 수, |v| 최대, 리셋 직전 자이로 크기 등.
scan3d.Scanner 의 재생 경로(_feed_pair → 브리지 → on_pose/on_vio_event)를 그대로 쓰므로 실제 복원과 같은 VIO 입력이다.
"""
import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("rec")
    p.add_argument("--json", default=None)
    a, rest = p.parse_known_args(argv)
    import scan3d
    import recording
    import replay as replay_mod
    sa = scan3d.parse_args(["--mode", "idle", "--no-rec-auto", "--no-start-app"] + rest)
    sc = scan3d.Scanner(sa)
    sc.log = lambda m: None if not m.startswith("[ov]") else print(m, flush=True)
    rec_dir = sc.resolve_rec(a.rec)
    meta = json.loads((rec_dir / "meta.json").read_text())
    phys = [str(x) for x in (meta.get("physical") or ["2", "5"])]
    pair = "2+6" if "6" in phys and len(phys) == 2 else "2+5"
    sc.apply_geometry((int(meta.get("width", 640)), int(meta.get("height", 480))), meta.get("fps") or 15, None, pair)
    rec = recording.Recording(rec_dir, lambda m: None, pair=scan3d.pair_physical(pair) if len(phys) > 2 else None)
    frames_log, events = [], []
    orig_pose, orig_evt = sc.on_pose, sc.on_vio_event

    def on_pose(msg):
        t = msg.get("t")
        v = msg.get("v")
        frames_log.append((t, msg.get("status"), int(msg.get("n_slam") or 0), int(msg.get("n_msckf") or 0),
                           float(np.linalg.norm(v)) if v and all(x is not None for x in v) else None))
        orig_pose(msg)

    def on_evt(msg):
        if msg.get("type") == "reset" and msg.get("auto"):
            t_last = frames_log[-1][0] if frames_log else None
            events.append(dict(t=t_last, reason=msg.get("reason"), reinit=msg.get("reinit"), gap=msg.get("gap"),
                               v_norm=msg.get("v_norm")))
        orig_evt(msg)

    sc.on_pose, sc.on_vio_event = on_pose, on_evt
    vio = sc.make_replay_vio()
    sc.begin_replay(vio, False, 8)
    sc.mode = "replay"
    imu = rec.sensors
    gyro = [(s.timestamp_ns, float(np.linalg.norm(s.values[:3]))) for s in imu if s.type == sa.gyro_type]
    k, first = 0, True
    t_wall = time.monotonic()
    for pr in rec.iter_pairs(sc.pair_skew_ms() if len(phys) == 2 else 0.55e3 / 15, {}):
        t_hi = max(f.timestamp_ns for f in pr) + replay_mod.IMU_LEAD_NS
        while k < len(imu) and imu[k].timestamp_ns <= t_hi:
            sc.on_sensor(imu[k])
            k += 1
        t_ns = sc._feed_pair(pr[::-1] if sc.swap else pr, timing=rec.timing)
        sc.take_replay_kf()
        if t_ns is not None:
            if sc.wait_pose(t_ns, replay_mod.FIRST_POSE_TIMEOUT_S if first else replay_mod.POSE_TIMEOUT_S):
                first = False
    time.sleep(0.3)
    vio.close()
    n = len(frames_log)
    trk = [f for f in frames_log if f[1] == "tracking"]
    zero = sum(1 for f in trk if f[2] + f[3] == 0)
    first_init = next((f[0] for f in frames_log if f[1] in ("tracking", "lost")), None)
    g = np.array(gyro, float) if gyro else np.zeros((0, 2))
    t0 = sc.t0_ns or (g[0, 0] if len(g) else 0)
    for e in events:          # 리셋 직전 1 s 자이로 최대 (rad/s)
        if e["t"] is not None and len(g):
            tt = (g[:, 0] - t0) * 1e-9
            sel = (tt > e["t"] - 1.5) & (tt <= e["t"])
            e["gyro_max_1s"] = round(float(g[sel, 1].max()), 2) if sel.any() else None
    out = dict(rec=rec_dir.name, frames=n, first_init_t=first_init, tracking_frac=round(len(trk) / max(1, n), 3),
               tracking_s=round(len(trk) / 15.0, 1), zero_feat_frames=zero,
               vmax=round(max((f[4] for f in trk if f[4] is not None), default=0.0), 2),
               resets=len(events), events=events, wall_s=round(time.monotonic() - t_wall, 1),
               bridge_args=sa.replay_bridge_args, config=sa.ov_config_mono)
    s = json.dumps(out, ensure_ascii=False)
    print("RESULT " + s, flush=True)
    if a.json:
        Path(a.json).write_text(s)
    sc.stopped = True
    return 0


if __name__ == "__main__":
    sys.exit(main())
