#!/usr/bin/env python3
"""녹화로 위치 추적(VIO)만 빠르게 돌려 보는 도구 — 깊이·지도 없이 브리지 포즈만 (설정·브리지 인자 비교용).

    venv_ffs/bin/python scan3d/tools/vio_track_eval.py scan3d/recordings/rec_20261016_175446 \
        [--ov-config-mono 설정.yaml] [--replay-bridge-args "--auto-reset --reinit-state"] [--json out.json]

출력 (한 줄 JSON): lost/리셋 시각·종류, 추적 중 프레임 비율, 특징 0 프레임 수, |v| 최대·중앙·95 %, 첫 카메라·첫 초기화 시각
(첫 IMU 기준 s), 정지 구간 (자이로 직전 1 s < 0.03 rad/s) 의 |v| 중앙·최대, 리셋 직전 자이로 크기 등.
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
    p.add_argument("--series", default=None, help="프레임별 t,status,n_slam,n_msckf,|v| CSV 저장")
    p.add_argument("--poses-out", default=None,
                   help="브리지 포즈·이벤트를 replay.py vio_poses.json 형식으로 저장 (eval_aruco_scale.py rec --vio-poses 경로)")
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
    frames_log, events, static_init = [], [], [None]
    pose_ents, last_tns = [], [-1]
    orig_pose, orig_evt = sc.on_pose, sc.on_vio_event

    def on_pose(msg):
        t = msg.get("t")
        v = msg.get("v")
        frames_log.append((t, msg.get("status"), int(msg.get("n_slam") or 0), int(msg.get("n_msckf") or 0),
                           float(np.linalg.norm(v)) if v and all(x is not None for x in v) else None))
        ts = msg.get("t_state")
        if static_init[0] is None and ts is not None and ts >= 0 and not msg.get("init_gate"):
            static_init[0] = t          # 정지 초기화가 된 첫 프레임 (상태 시각이 생김, ZUPT 동안은 status init)
        if a.poses_out:
            try:
                last_tns[0] = int(msg.get("t_ns", -1))
            except (TypeError, ValueError):
                pass
            pose_ents.append(["p", last_tns[0], dict(msg)])
        orig_pose(msg)

    def on_evt(msg):
        if a.poses_out:
            pose_ents.append(["e", last_tns[0], dict(msg)])
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
    k, first, n_kf = 0, True, 0
    t_wall = time.monotonic()
    for pr in rec.iter_pairs(sc.pair_skew_ms() if len(phys) == 2 else 0.55e3 / 15, {}):
        t_hi = max(f.timestamp_ns for f in pr) + replay_mod.IMU_LEAD_NS
        while k < len(imu) and imu[k].timestamp_ns <= t_hi:
            sc.on_sensor(imu[k])
            k += 1
        t_ns = sc._feed_pair(pr[::-1] if sc.swap else pr, timing=rec.timing)
        n_kf += sc.take_replay_kf() is not None
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
    # 정지 구간 속도: 자이로가 직전 1 s 동안 0.03 rad/s 미만인 추적 프레임의 |v| (0 이어야 정상, 크면 바이어스 오인·표류)
    v_still = []
    if len(g):
        tt = (g[:, 0] - t0) * 1e-9
        for f in trk:
            if f[4] is None or f[0] is None:
                continue
            sel = (tt > f[0] - 1.0) & (tt <= f[0])
            if sel.sum() > 50 and g[sel, 1].max() < 0.03:
                v_still.append(f[4])
    vt = [f[4] for f in trk if f[4] is not None]
    first_cam = frames_log[0][0] if frames_log else None
    out = dict(rec=rec_dir.name, frames=n, first_cam_t=None if first_cam is None else round(first_cam, 3),
               static_init_t=None if static_init[0] is None else round(static_init[0], 3),
               first_init_t=None if first_init is None else round(first_init, 3),
               tracking_frac=round(len(trk) / max(1, n), 3),
               tracking_s=round(len(trk) / 15.0, 1), zero_feat_frames=zero, keyframes=n_kf,
               vmax=round(max((f[4] for f in trk if f[4] is not None), default=0.0), 2),
               v_p50=round(float(np.percentile(vt, 50)), 3) if vt else None,
               v_p95=round(float(np.percentile(vt, 95)), 3) if vt else None,
               still_frames=len(v_still), v_still_med=round(float(np.median(v_still)), 3) if v_still else None,
               v_still_max=round(float(np.max(v_still)), 3) if v_still else None,
               resets=len(events), events=events, wall_s=round(time.monotonic() - t_wall, 1),
               bridge_args=sa.replay_bridge_args, config=sa.ov_config_mono)
    s = json.dumps(out, ensure_ascii=False)
    print("RESULT " + s, flush=True)
    if a.json:
        Path(a.json).write_text(s)
    if a.poses_out:
        Path(a.poses_out).write_text(json.dumps(dict(
            format=replay_mod.POSE_LOG_FORMAT, recording=rec_dir.name, recon="vio_track_eval", pair=pair, size=list(rec.size),
            fps=int(sc.geom["fps"]), vio_mode=sa.vio_mode, vio_cam0=sc.vio_cam0, n_poses=sum(1 for e in pose_ents if e[0] == "p"),
            created=time.strftime("%Y-%m-%d %H:%M:%S"), entries=pose_ents), separators=(",", ":"), default=float))
    if a.series:
        Path(a.series).write_text("t,status,n_slam,n_msckf,v\n" + "".join(
            f"{f[0]},{f[1]},{f[2]},{f[3]},{'' if f[4] is None else round(f[4], 4)}\n" for f in frames_log))
    sc.stopped = True
    return 0


if __name__ == "__main__":
    sys.exit(main())
