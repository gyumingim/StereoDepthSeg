#!/usr/bin/env python3
"""녹화 재생(복원): 폰 녹화 폴더 → 실시간과 같은 파이프라인 (IMU 동기 → OpenVINS 포즈 → FFS + YOLO → 복셀 맵).

녹화 형식은 scan3d/RECORDING_FORMAT.md (scan3d-phone-rec/1), 읽기는 recording.py (영상↔CSV 는 PTS 로 맞춤).
실시간과 다른 점 (느려도 정확하게):
  - 재생 전용 브리지 프로세스(pipe, 새 상태·새 시각 기준, 기본 --auto-reset). --ov none 이면 고정 포즈.
  - 프레임마다 포즈를 기다린다 (버리는 카메라 없음). IMU 는 카메라 시각 + 0.1 s 까지 먼저 넣는다.
  - 키프레임은 버리지 않고 대기열(최대 8)에 넣는다. --every-frame 이면 포즈가 있는 모든 쌍에 FFS.
  - FFS 반복 --replay-iters (기본 8, 실시간 4).
결과: <녹화>/recon_<시각>/ (또는 --replay-out) 에 voxels.ply, voxels.json, voxels.npz, objects.json, trajectory.json,
      viewer.html, summary.json (재생 통계 포함). auto-reset 이 나면 그 전 맵은 segment_NN/ 에.
웹 뷰어(http://localhost:8795)에 진행률(재생 위치 / 전체 s)이 보인다.

명령줄:
  venv_ffs/bin/python scan3d/replay.py recordings/rec_20261016_120000 [--every-frame] [--replay-iters 12]
      [--keep-web] [scan3d.py 인자...]
  = scan3d.py --mode idle --replay DIR --exit-after-replay ... (--keep-web 이면 끝난 뒤에도 뷰어 유지)
"""
import sys
import threading
import time
import traceback
from pathlib import Path

POSE_TIMEOUT_S = 5.0
FIRST_POSE_TIMEOUT_S = 15.0
IMU_LEAD_NS = 100_000_000


class Replayer:
    def __init__(self, sc, rec_dir, every_frame=False, iters=8, out_dir=None):
        self.sc, self.rec_dir = sc, Path(rec_dir)
        self.every_frame, self.iters = every_frame, iters
        self.out_dir = Path(out_dir) if out_dir else self.rec_dir / time.strftime("recon_%Y%m%d-%H%M%S")
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True, name="replay")
        self.progress = dict(state="준비", name=self.rec_dir.name, dir=str(self.rec_dir), out=str(self.out_dir),
                             every_frame=every_frame, iters=iters, pos_s=0.0, total_s=0.0, pairs=0, frames_total=0,
                             keyframes=0, pose_timeouts=0, speed=None, error=None, started=time.time(), elapsed_s=0.0)

    def start(self):
        self.thread.start()

    def stop(self):
        self._stop.set()

    def join(self, timeout=None):
        if self.thread.is_alive() and threading.current_thread() is not self.thread:
            self.thread.join(timeout)

    def _log(self, msg):
        self.sc.log(f"[replay] {msg}")

    def _enqueue(self, job):
        import queue
        while not self._stop.is_set():
            try:
                self.sc.kf_queue.put(job, timeout=0.5)
                return True
            except queue.Full:
                continue
        return False

    def _run(self):
        import recording
        sc, P = self.sc, self.progress
        vio = None
        kf0 = sc.dstats.get("keyframes", 0)
        try:
            P["state"] = "읽는 중"
            rec = recording.Recording(self.rec_dir, self._log)
            P.update(total_s=round(rec.duration_s, 2), frames_total=rec.n_frames)
            if rec.size != tuple(sc.a.size):
                raise ValueError(f"녹화 크기 {rec.size} ≠ 보정 크기 {tuple(sc.a.size)}")
            if [str(p) for p in rec.physical[:2]] != [str(p) for p in sc.a.physical]:
                self._log(f"주의: 녹화 렌즈 {rec.physical} ≠ --physical {sc.a.physical}")
            P["state"] = "브리지 시작"
            vio = sc.make_replay_vio()
            if not rec.sensors and not sc.a.ov == "none":
                raise ValueError("imu.csv 없음 → VIO 불가 (--ov none 으로 고정 포즈 재생만 가능)")
            sc.begin_replay(vio, self.every_frame, self.iters)
            P["state"] = "모델 대기"
            while not self._stop.is_set() and not (sc.dstats.get("ready") and sc.ds is not None
                                                   and sc.ds.iters == sc.want_iters):
                if sc.dstats.get("error"):
                    raise RuntimeError(f"깊이 모델 오류: {sc.dstats['error']}")
                time.sleep(0.2)
            P["state"] = "재생 중"
            self._log(f"{self.rec_dir} · {rec.n_frames} 프레임 · {rec.duration_s:.1f} s · every_frame={self.every_frame} "
                      f"iters={self.iters} → {self.out_dir}")
            imu = rec.sensors
            k = 0
            t_start = time.monotonic()
            first = True
            pstats = {}
            for pair in rec.iter_pairs(sc.a.pair_skew_ms, pstats):
                if self._stop.is_set():
                    break
                while sc.paused and not self._stop.is_set():
                    time.sleep(0.1)
                t_hi = max(f.timestamp_ns for f in pair) + IMU_LEAD_NS
                while k < len(imu) and imu[k].timestamp_ns <= t_hi:
                    sc.on_sensor(imu[k])
                    k += 1
                p2 = pair[::-1] if sc.swap else pair
                sc.take_replay_kf()
                t_ns = sc._feed_pair(p2, timing=rec.timing)
                if t_ns is not None:
                    timeout = FIRST_POSE_TIMEOUT_S if first else (POSE_TIMEOUT_S if k < len(imu) else 0.5)
                    if sc.wait_pose(t_ns, timeout):
                        first = False
                    else:
                        P["pose_timeouts"] += 1
                    kf = sc.take_replay_kf()
                    if kf is not None and not self._enqueue(kf):
                        break
                P["pairs"] += 1
                P["pos_s"] = round((pair[0].timestamp_ns - rec.t_first) * 1e-9, 2)
                el = time.monotonic() - t_start
                P["elapsed_s"] = round(el, 1)
                P["speed"] = round(P["pos_s"] / el, 3) if el > 0 else None
                P["keyframes"] = sc.dstats.get("keyframes", 0) - kf0
                P["queue"] = sc.kf_queue.qsize()
            stopped = self._stop.is_set()
            P["state"] = "깊이 마무리"
            while not stopped and not sc.queue_idle():
                if self._stop.is_set():
                    break
                time.sleep(0.1)
                P["keyframes"] = sc.dstats.get("keyframes", 0) - kf0
            P["keyframes"] = sc.dstats.get("keyframes", 0) - kf0
            P.update(reordered=pstats.get("reordered"), unmatched_dropped=pstats.get("unmatched_dropped"),
                     vio_auto_resets=sc.vio_auto_resets)
            if sc.vmap.n:
                P["state"] = "저장 중"
                sc.objects = sc.vmap.objects(force=True)
                d = sc.save(self.out_dir, extra=dict(replay=dict(P, recording=str(self.rec_dir), stopped=stopped,
                                                                 meta=rec.meta, vio=sc.vio.stats())))
                sc.last_result = str(d)
                P["out"] = str(d)
            else:
                self._log("맵이 비어 저장하지 않음 (포즈가 안 나왔으면 녹화 시작 때 폰 정지·IMU 를 확인)")
            P["state"] = "정지됨" if stopped else "완료"
            self._log(f"{P['state']}: 쌍 {P['pairs']} · 키프레임 {P['keyframes']} · 포즈 시간초과 {P['pose_timeouts']}")
        except Exception as e:
            P.update(state="오류", error=repr(e))
            self._log(f"오류: {e!r}")
            traceback.print_exc()
        finally:
            P["elapsed_s"] = round(time.time() - P["started"], 1)
            sc.end_replay(self, vio)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0].startswith("-"):
        print(__doc__)
        return 2
    rec = argv.pop(0)
    keep = "--keep-web" in argv
    argv = [x for x in argv if x != "--keep-web"]
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import scan3d
    extra = [] if keep else ["--exit-after-replay"]
    return scan3d.main(["--mode", "idle", "--replay", rec] + extra + argv)


if __name__ == "__main__":
    sys.exit(main())
