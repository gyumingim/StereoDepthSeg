#!/usr/bin/env python3
"""Phone rear cameras -> USB -> PC preview/YOLO. Q quits, S saves paired PNGs.

  venv/bin/python z_phone_stereo.py --install --list
  venv/bin/python z_phone_stereo.py --physical 5 2 --yolo both
  venv/bin/python z_phone_stereo.py --physical 5 2 6 --duration 15 --headless
  venv_ffs/bin/python z_phone_stereo.py --physical 2 5 --calib-stereo calib_phone_pair.json --yolo both   # FFS depth
  venv_ffs/bin/python z_phone_stereo.py --physical 2 5 --calib-stereo calib_phone_pair.json --offline phone_stereo/runs/probe_now
  venv/bin/python z_phone_stereo.py --physical 2 5 --save-interval 2 --headless --duration 60   # 캘리용 번들 자동 저장

Camera IDs are device-specific. --list queries them using the actual Camera2 API.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import os
from pathlib import Path
import socket
import subprocess
import time

import cv2
import numpy as np

from lib_phone_stereo import PhoneReceiver

ROOT = Path(__file__).resolve().parent
ADB = ROOT / "tools/platform-tools/adb"
PACKAGE = "com.camera.dualstream"


def adb(*args, check=True):
    return subprocess.run([str(ADB), *map(str, args)], check=check, capture_output=True, text=True, timeout=30)


def start_phone(a):
    adb("shell", "am", "force-stop", PACKAGE)
    adb("shell", "run-as", PACKAGE, "rm", "-f", "files/status.txt", "files/cameras.json")
    args = ["shell", "am", "start", "-S", "-n", PACKAGE + "/.MainActivity"]
    if not a.list:
        args += ["--es", "logical", a.logical, "--es", "physical", ",".join(a.physical),
                 "--ei", "width", a.size[0], "--ei", "height", a.size[1], "--ei", "fps", a.fps,
                 "--ef", "focus_diopters", (1.0 / a.focus_m) if a.focus_m > 0 else 0.0,
                 "--ez", "raw_ts", "true" if a.raw_ts else "false"]
    print(adb(*args).stdout.strip(), flush=True)


def save_pair(out, pair, a, receiver=None):
    out.mkdir(parents=True, exist_ok=True)
    meta = dict(logical=a.logical, physical=a.physical, image_size=a.size, focus_m=a.focus_m,
                timestamp_ns=[f.timestamp_ns for f in pair], format="NV21 -> BGR PNG",
                mode="phone_stereo", calibration="required_for_metric_depth",
                imu=receiver.imu_for_pair(pair) if receiver else {})   # 프레임 시각의 중력/자이로/회전벡터/기압 + 두 노출 사이 자이로 창
    for i, f in enumerate(pair):
        if not cv2.imwrite(str(out / f"camera_{a.physical[i]}.png"), f.bgr):
            raise IOError("Cannot save paired PNG")
    (out / "pair.json").write_text(json.dumps(meta, indent=2))


def run_offline(bundle, depth_worker, a):
    """저장된 번들(camera_<id>.png) 로 depth 파이프라인만 실행. 결과는 번들 폴더의 depth_<mode>.png / depth.json."""
    from lib_phone_stereo import PhoneFrame
    bundle = Path(bundle)
    meta = json.loads((bundle / "pair.json").read_text()) if (bundle / "pair.json").exists() else {}
    ts = meta.get("timestamp_ns") or [0, 0]
    imu = dict(meta.get("imu") or {})
    source_ids = [str(p) for p in meta.get("physical", a.physical)]
    if a.rgbd_viewer:
        if (meta.get("logical") != a.logical or meta.get("image_size") != list(a.size)
                or set(source_ids[:2]) != set(a.physical[:2])
                or meta.get("focus_m") != a.focus_m or not meta.get("timestamp_ns")):
            raise ValueError(f"{bundle}: mapping requires matching lens/size/focus metadata and real timestamps")
    indices = [source_ids.index(pid) for pid in a.physical[:2]]
    if imu.get("per_frame"):
        imu["per_frame"] = [imu["per_frame"][i] for i in indices]
    elif a.rgbd_viewer and indices[0] != 0:
        # Legacy bundles have sensors at only their first lens timestamp.
        imu = {k: v for k, v in imu.items() if k == "gyro_window"}
    imu["exposure_ts_ns"] = [ts[i] if i < len(ts) else 0 for i in indices]
    pair = []
    for i, pid in enumerate(a.physical[:2]):
        img = cv2.imread(str(bundle / f"camera_{pid}.png"))
        if img is None:
            raise FileNotFoundError(bundle / f"camera_{pid}.png")
        if (img.shape[1], img.shape[0]) != tuple(a.size):
            raise ValueError(f"{bundle}: 이미지 {img.shape[1]}x{img.shape[0]} 가 --size {a.size} 와 다름")
        pair.append(PhoneFrame(index=i, timestamp_ns=int(ts[indices[i]] if indices[i] < len(ts) else 0), bgr=img))
    depth_worker.ep = None            # 라이브는 첫 쌍에서 한 번만 검사하지만, 번들마다 다른 장면이므로 매번 다시
    vis, report = depth_worker(pair, imu)
    cv2.imwrite(str(bundle / f"depth_{a.yolo}.png"), vis)
    (bundle / "depth.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    g = report.get("imu") or {}
    print(f"[offline {bundle.name}] {report['ms']:.0f}ms  epipolar dy_med {report['epipolar'].get('dy_med')} "
          f"inlier {report['epipolar'].get('inlier_frac')}  scale_status {report['scale_status']}  "
          f"tilt pitch {g.get('pitch_deg')} roll {g.get('roll_deg')}")
    for method, rows in report["objects"].items():
        for r in rows:
            print(f"   {method:4s} #{r.get('instance')} {r.get('label'):10s} {r.get('status'):12s} "
                  f"dist {r.get('dist_m')}m  z {r.get('z_cam1_med_m')}m  dims_mm {r.get('dims_mm')}  "
                  f"world {r.get('center_world_m')}  warn {r.get('warnings')}")
    return report


def detect_frame(frame, mode):
    import lib_detect as ld
    model = ld.DEFAULT_MODEL if mode == "bbox" else ld.DEFAULT_SEG_MODEL
    t0 = time.perf_counter()
    dets = ld.detect_yolo(frame.bgr, model=model, device="0")
    vis = ld.draw(frame.bgr, dets)
    return vis, dict(timestamp_ns=frame.timestamp_ns, mode=mode,
                     device=ld.detect_yolo.last_device, ms=(time.perf_counter()-t0)*1000,
                     detections=[dict(label=d["label"], conf=d["conf"], box=d["box"],
                                      mask_pixels=int(np.count_nonzero(d["mask"])) if d["mask"] is not None else None)
                                 for d in dets])


def main(a):
    a.phone_started = False
    if len(a.physical) not in (2, 3) or len(set(a.physical)) != len(a.physical):
        raise ValueError("Choose 2 or 3 distinct physical IDs")
    if min(a.size) <= 0 or any(x % 2 for x in a.size) or a.max_skew_ms < 0:
        raise ValueError("Need positive even image dimensions and nonnegative skew limit")
    if a.duration < 0 or a.fps <= 0 or not 0 < a.scale <= 1 or a.iters < 1:
        raise ValueError("Invalid duration/fps/scale/iters")
    if a.rgbd_viewer and (not a.calib_stereo or a.raw_ts):
        raise ValueError("--rgbd-viewer requires --calib-stereo and physical timestamps")
    if a.rgbd_viewer and (not np.isfinite(a.depth_range).all() or not 0 < a.depth_range[0] < a.depth_range[1]):
        raise ValueError("--depth-range requires finite 0 < near < far")
    rgbd_viewer = None
    if a.rgbd_viewer:
        from lib_rgbd_viewer import RGBDViewer
        rgbd_viewer = RGBDViewer()
    depth_worker = None
    if a.calib_stereo:
        from lib_phone_depth import PhoneDepth
        depth_worker = PhoneDepth(a)
    if a.offline:
        if depth_worker is None:
            raise ValueError("--offline 은 --calib-stereo 가 필요 (depth 만 오프라인 실행)")
        for b in a.offline:
            run_offline(b, depth_worker, a)
            if rgbd_viewer is not None:
                rgbd_viewer.set_snapshot(depth_worker.rgbd_snapshot)
                rgbd_viewer.save(Path(a.out or "phone_stereo/runs/offline_rgbd") / "rgbd" / Path(b).name)
        return
    if a.install:
        subprocess.run(["python3", str(ROOT / "phone_stereo/build_android.py")], check=True)
        print(adb("install", "-r", "-g", ROOT / "phone_stereo/android/build/phone-stereo.apk").stdout)
    out = Path(a.out or ROOT / "phone_stereo/runs" / datetime.now().strftime("%Y%m%d-%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    if not a.attach:
        a.phone_started = True
        start_phone(a)
    else:
        a.phone_started = True
    if a.list:
        # Inventory is written asynchronously by Activity.onCreate.
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            r = adb("shell", "run-as", PACKAGE, "cat", "files/cameras.json", check=False)
            if r.returncode == 0:
                inventory = json.loads(r.stdout)
                (out / "cameras.json").write_text(json.dumps(inventory, indent=2))
                for c in inventory:
                    print(f"logical {c['id']} facing={c['facing']}: "
                          + ", ".join(f"physical {p['id']} focal={p.get('focal_mm')}mm" for p in c['physical']))
                print(out / "cameras.json")
                return
            time.sleep(.1)
        raise TimeoutError("Phone inventory failed")
    adb("forward", "tcp:8765", "tcp:8765")
    deadline = time.monotonic() + 12
    sock = None
    # ADB's forwarded port accepts even before the phone starts listening, so
    # wait for the Activity's state rather than treating connect() as readiness.
    while time.monotonic() < deadline:
        state = adb("shell", "run-as", PACKAGE, "cat", "files/status.txt", check=False).stdout
        if state.startswith("STREAMING"):
            sock = socket.create_connection(("127.0.0.1", 8765), timeout=8)
            break
        if any(s in state for s in ("FAILED", "ERROR", "DISCONNECTED", "STOPPED")):
            raise RuntimeError(state)
        time.sleep(.15)
    if sock is None:
        raise TimeoutError("Camera startup: " + state)
    receiver = PhoneReceiver(sock, len(a.physical), a.size, a.max_skew_ms)
    receiver.start()
    started = time.monotonic()
    last_print = started
    worker = ThreadPoolExecutor(max_workers=1)
    future = None
    last_inferred = None
    result = None
    report = None
    inference_times = []
    last_save = 0
    last_rgbd_save = 0
    last_bundle = 0
    # --still: 연속 프레임 차이(두 스트림) + 자이로로 '정지 순간' 판정. 두 렌즈는 timestamp 가 같아도 실제 노출이
    # 어긋날 수 있어(sync APPROXIMATE) 보드/폰이 움직이는 중에 찍힌 쌍은 캘리를 망친다 (STATUS 문제점 18).
    prev_small, still_count, last_saved_small, last_pair_ts = None, 0, None, None
    board_corners, board_state = [None, None], ""          # --board-hud: 렌즈별 체커보드 코너(HUD 표시·저장 게이트)
    gate_reason, gate_color, last_saved_at, last_printed = "", (0, 220, 255), 0, ""   # 왜 저장하는지/안 하는지 HUD+터미널
    if a.board_hud:
        import lib_calib
    def small(f):
        return cv2.resize(cv2.cvtColor(f.bgr, cv2.COLOR_BGR2GRAY), (160, 120), interpolation=cv2.INTER_AREA).astype(np.float32)
    title = "Phone Stereo - USB to PC (Q: quit, S: paired PNG)"
    print(f"Receiving {a.physical}; saving to {out}", flush=True)
    try:
        while not a.duration or time.monotonic() - started < a.duration:
            frames, pair, pair_count, received = receiver.snapshot()
            if receiver.error:
                raise RuntimeError(receiver.error)
            now = time.monotonic()
            if now - started > 10 and any(t is None or now-t > 8 for t in received):
                raise TimeoutError("One or more physical streams stopped")
            if future and future.done():
                result, report = future.result()
                inference_times.append(report["ms"])
                if rgbd_viewer is not None:
                    rgbd_viewer.set_snapshot(depth_worker.rgbd_snapshot)
                    if now-last_rgbd_save > 5:
                        rgbd_viewer.save(out / "rgbd")
                        last_rgbd_save = now
                if now-last_save > 1:
                    (out / "inference.json").write_text(json.dumps(report, indent=2))
                    cv2.imwrite(str(out / "inference.png"), result)
                    last_save = now
                future = None
            if (a.yolo != "none" or depth_worker) and pair and future is None and pair[0].timestamp_ns != last_inferred:
                last_inferred = pair[0].timestamp_ns
                future = (worker.submit(depth_worker, pair, receiver.imu_for_pair(pair)) if depth_worker
                          else worker.submit(detect_frame, pair[0], a.yolo))
            if not a.headless:
                tiles = []
                show = list(pair[:2]) + list(frames[2:]) if (a.board_hud and pair) else frames   # HUD 는 코너를 찍은 그 쌍을 보여준다
                for i, f in enumerate(show):
                    img = f.bgr.copy() if f else np.zeros((a.size[1], a.size[0], 3), np.uint8)
                    stale = received[i] is None or now - received[i] > 1
                    cv2.putText(img, f"Camera {a.physical[i]} {'WAIT / STALE' if stale else 'LIVE'}", (12,25),
                                cv2.FONT_HERSHEY_SIMPLEX, .65, (0,0,255) if stale else (0,255,0), 2)
                    if a.board_hud and i < 2:
                        c = board_corners[i]
                        if c is not None:
                            cv2.drawChessboardCorners(img, tuple(lib_calib.PATTERN), c.astype(np.float32), True)
                        cv2.putText(img, "BOARD OK" if c is not None else "NO BOARD", (12, 55), cv2.FONT_HERSHEY_SIMPLEX, .8,
                                    (0, 255, 0) if c is not None else (0, 0, 255), 2)
                        if i == 0:
                            n_saved = len(list((out / "captures").iterdir())) if (out / "captures").exists() else 0
                            cv2.putText(img, f"saved {n_saved}", (12, 85), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 220, 255), 2)
                            cv2.putText(img, gate_reason[:70], (12, img.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, .7, gate_color, 2)
                    tiles.append(img)
                canvas = np.hstack(tiles)
                if rgbd_viewer is None:
                    cv2.imshow(title, canvas)
                if result is not None and rgbd_viewer is None:
                    cv2.imshow("PC inference - sampled frame (not raw live feed)", result)
                if rgbd_viewer is not None:
                    rgbd_viewer.show()
                key = cv2.waitKey(1) & 255
                if rgbd_viewer is not None:
                    rgbd_viewer.key(key)
                    if key == ord('e') and rgbd_viewer.snapshot is not None:
                        rgbd_viewer.save(out / "rgbd_captures" / str(rgbd_viewer.snapshot['timestamp_ns'][0]))
                        print("Saved RGB-D frame / scene PLY / IMU", flush=True)
                if key in (27, ord("q")):
                    break
                if key == ord("s") and pair:
                    save_pair(out / "captures" / str(pair[0].timestamp_ns), pair, a, receiver)
                    print("Saved paired PNGs", flush=True)
            if a.save_interval and pair and pair[0].timestamp_ns != last_pair_ts:
                last_pair_ts = pair[0].timestamp_ns
                ok_to_save, reasons = True, []                                     # reasons: 저장 안 되는 이유 (HUD·터미널)
                if a.board_hud:
                    # 절반 해상도 빠른 검출 (HUD·게이트용). 실제 캘리는 z_phone_calib 가 저장 PNG 에서 풀해상도로 다시 검출한다.
                    board_corners = [lib_calib.find_corners(cv2.cvtColor(f.bgr, cv2.COLOR_BGR2GRAY), fast=True) for f in pair[:2]]
                    missing = [a.physical[i] for i, c in enumerate(board_corners) if c is None]
                    if missing:
                        ok_to_save = False; reasons.append(f"NO BOARD in cam {','.join(missing)}")
                if a.still:
                    cur = [small(f) for f in pair[:2]]
                    diffs = [float(np.mean(np.abs(c - p))) for c, p in zip(cur, prev_small)] if prev_small else [99.0]
                    g = receiver.sensor_at(4, pair[0].timestamp_ns)                 # 자이로 (있으면)
                    gyro = float(np.linalg.norm(g[0])) if g else 0.0
                    still_count = still_count + 1 if max(diffs) < a.still_thr and gyro < 0.05 else 0
                    moved = last_saved_small is None or max(float(np.mean(np.abs(c - p))) for c, p in zip(cur, last_saved_small)) > 3 * a.still_thr
                    prev_small = cur
                    if still_count < 3:
                        ok_to_save = False; reasons.append(f"MOVING (frame diff {max(diffs):.1f} > {a.still_thr}" + (f", gyro {gyro:.2f}" if gyro >= 0.05 else "") + f") hold still {3-still_count} more")
                    elif not moved:
                        ok_to_save = False; reasons.append("SAME VIEW as last save - move board/phone")
                if ok_to_save and now - last_bundle < a.save_interval:
                    ok_to_save = False; reasons.append(f"WAIT {a.save_interval - (now - last_bundle):.1f}s (interval)")
                if ok_to_save:
                    save_pair(out / "captures" / str(pair[0].timestamp_ns), pair, a, receiver)
                    last_bundle = last_saved_at = now
                    if a.still:
                        last_saved_small = cur
                    n_saved = len(list((out / "captures").iterdir()))
                    gate_reason, gate_color = f"SAVED #{n_saved}", (0, 255, 0)
                    print(f"Saved bundle {pair[0].timestamp_ns}  (총 {n_saved})", flush=True)
                elif now - last_saved_at > 1.0:                                     # 저장 직후 1초는 SAVED 표시 유지
                    gate_reason = " | ".join(reasons) or "READY"
                    gate_color = (0, 220, 255) if reasons and reasons[0].startswith("WAIT") else (0, 0, 255)
                key_reason = gate_reason.split(" (")[0].split(" hold")[0]
                if key_reason != last_printed:                                       # 이유가 바뀔 때만 터미널에 (스팸 방지)
                    print(f"[save gate] {gate_reason}", flush=True); last_printed = key_reason
            if now - last_print >= 5:
                print(json.dumps(receiver.stats()), flush=True)
                last_print = now
            time.sleep(.01)
    finally:
        receiver.close()
        worker.shutdown(wait=True, cancel_futures=True)
        inference_error = None
        if future and future.done() and not future.cancelled():
            try:
                result, report = future.result()
                inference_times.append(report["ms"])
                cv2.imwrite(str(out / "inference.png"), result)
                (out / "inference.json").write_text(json.dumps(report, indent=2))
            except Exception as e:
                inference_error = str(e)
        if rgbd_viewer is not None:
            rgbd_viewer.set_snapshot(depth_worker.rgbd_snapshot)
            rgbd_viewer.save(out / "rgbd")
        _, pair, _, _ = receiver.snapshot()
        if pair:
            save_pair(out, pair, a, receiver)
        stats = receiver.stats()
        stats.update(logical=a.logical, physical=a.physical, image_size=a.size,
                     elapsed_s=time.monotonic()-started, inference=report, inference_error=inference_error,
                     inference_count=len(inference_times),
                     inference_median_ms=float(np.median(inference_times)) if inference_times else None)
        (out / "summary.json").write_text(json.dumps(stats, indent=2))
        print(json.dumps(stats, indent=2), flush=True)
        cv2.destroyAllWindows()
    if stats["pairs"] == 0:
        raise RuntimeError("No timestamp-matched frame pairs received")


if __name__ == "__main__":
    os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rgbd-viewer", action="store_true", help="RGB / depth / interactive scene point cloud / IMU dashboard")
    ap.add_argument("--install", action="store_true")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--attach", action="store_true", help="Receive from an already started Phone Stereo app")
    ap.add_argument("--logical", default="0")
    ap.add_argument("--physical", nargs="+", default=["5", "2"])
    ap.add_argument("--size", type=int, nargs=2, default=[640, 480])
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--focus-m", type=float, default=1.0,
                    help="광각 렌즈 초점 고정 거리(m). 0 = 자동초점(초점거리가 변해 스테레오에 부적합). 캘리와 같은 값으로 쓸 것")
    ap.add_argument("--max-skew-ms", type=float, default=35,
                    help="같은 촬영 결과에서 찾은 물리 SENSOR_TIMESTAMP 차 상한(ms). 첫 렌즈를 기준으로 "
                         "다른 렌즈의 앞뒤 후보 중 가까운 프레임을 선택. 하드웨어 동기화는 아님")
    ap.add_argument("--raw-ts", action="store_true", help="앱이 논리 timestamp(보정 전)로 보내게 함 (예전 동작)")
    ap.add_argument("--duration", type=float, default=0, help="0: run until Q/Ctrl-C")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--yolo", choices=["none", "seg", "bbox", "both"], default="none",
                    help="both: segment model supplies mask and bbox; inference on PC GPU")
    ap.add_argument("--out")
    ap.add_argument("--calib-stereo", help="Phone lens-pair calibration JSON; enables FFS. Run with venv_ffs.")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--iters", type=int, default=4)
    ap.add_argument("--offline", nargs="+", metavar="DIR", help="저장 번들로 depth 만 실행 (폰 불필요)")
    ap.add_argument("--depth-range", type=float, nargs=2, default=[0.3, 3.0], metavar=("ZMIN", "ZMAX"),
                    help="깊이 색상맵 범위(m): 가까움=빨강, 멂=파랑, 로그 스케일")
    ap.add_argument("--save-interval", type=float, default=0, help="초. >0 이면 쌍을 주기적으로 captures/<ts>/ 에 저장 (캘리 수집용)")
    ap.add_argument("--still", action="store_true", help="--save-interval 저장을 '정지 순간'(연속 프레임 차이·자이로 작음, 지난 저장과 다른 장면)으로 제한")
    ap.add_argument("--still-thr", type=float, default=1.5, help="정지 판정 프레임 차이 임계 (160x120 회색 평균 절대차)")
    ap.add_argument("--board-hud", action="store_true", help="체커보드 검출 표시(BOARD OK/NO BOARD, 코너) + 양쪽 다 검출될 때만 저장")
    args = ap.parse_args()
    try:
        main(args)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(args, "phone_started", False):
            adb("shell", "am", "force-stop", PACKAGE, check=False)
            adb("forward", "--remove", "tcp:8765", check=False)
