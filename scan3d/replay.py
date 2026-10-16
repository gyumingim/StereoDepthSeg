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
      리플레이용 시간 정보 timeline.json · voxels_t.bin · kf/ (replay_view.py), 녹화 폴더에 카메라 화면 그림 thumbs/.
웹 뷰어(http://localhost:8795)에 진행률(재생 위치 / 전체 s)이 보인다.

비교용 3렌즈 녹화 (meta physical 2,5,6, 웹 '3개 모두 녹화'): pair=["2","6"] 처럼 깊이 렌즈 쌍을 골라 복원한다 (결과 폴더 꼬리 _p26).
  모노 VIO 는 렌즈 2 만 쓰므로 첫 쌍 복원의 브리지 포즈·리셋 이벤트를 결과 폴더 vio_poses.json 에 저장하고,
  다른 쌍 복원은 브리지를 띄우지 않고 그 포즈를 같은 렌즈 2 프레임 시각에 그대로 넣는다 (PoseReplayVIO) → 두 쌍이 같은 궤적,
  깊이만 다름. 조건: --vio-mode mono · VIO cam0 = lens0u · 같은 크기·fps. 아니면 VIO 를 새로 돌린다 (summary vio_reused 없음).
  결과마다 비교 숫자 (summary.json compare · timeline.json compare/키프레임 q, replay_view.kf_quality).

명령줄:
  venv_ffs/bin/python scan3d/replay.py recordings/rec_20261016_120000 [--every-frame] [--replay-iters 12]
      [--keep-web] [scan3d.py 인자...]
  = scan3d.py --mode idle --replay DIR --exit-after-replay ... (--keep-web 이면 끝난 뒤에도 뷰어 유지)
"""
import json
import sys
import threading
import time
import traceback
from pathlib import Path

POSE_TIMEOUT_S = 5.0
FIRST_POSE_TIMEOUT_S = 15.0
IMU_LEAD_NS = 100_000_000
POSE_LOG_FORMAT = "scan3d-vio-poses/1"
POSE_LOG_FILE = "vio_poses.json"


def find_pose_src(rec_dir, pair, size, fps, vio_mode):
    """같은 녹화의 다른 쌍 복원이 남긴 vio_poses.json (최근 것 먼저) → (복원 이름, dict) 또는 (None, None)."""
    import replay_view
    for r in replay_view.list_recons(rec_dir):
        if not r["ready"] or r["pair"] == pair:
            continue
        f = Path(rec_dir) / r["name"] / POSE_LOG_FILE
        if not f.is_file():
            continue
        try:
            j = json.loads(f.read_text())
        except (OSError, ValueError):
            continue
        if (j.get("format") == POSE_LOG_FORMAT and j.get("recording") == Path(rec_dir).name
                and list(j.get("size") or []) == list(size) and int(j.get("fps") or 0) == int(fps)
                and j.get("vio_mode") == vio_mode == "mono" and j.get("vio_cam0") == "lens0u" and j.get("entries")):
            return r["name"], j
    return None, None


class PoseReplayVIO:
    """브리지 대신 저장해 둔 포즈를 넣는다 (다른 쌍 복원의 vio_poses.json). push_cam(t) 때 키가 t 이하인 포즈·이벤트를
    받은 순서대로 sc.on_pose / sc.on_vio_event 에 넣고, 그 시각까지 포즈를 본 것으로 표시한다 (첫 복원에 없던 프레임은 포즈 없음)."""

    def __init__(self, sc, entries, src):
        self.sc, self.E, self.src = sc, entries, src
        self.k = 0
        self.latest = None
        self.hit = self.miss = 0
        self.max_inflight = self.max_pending = 0

    def push_imu(self, t_ns, w, a):
        pass

    def push_cam(self, t_ns, grays):
        E, got = self.E, False
        while self.k < len(E) and int(E[self.k][1]) <= t_ns:
            kind, key, msg = E[self.k]
            self.k += 1
            if kind == "p":
                got = got or int(key) == int(t_ns)
                self.latest = msg
                self.sc.on_pose(dict(msg))
            else:
                self.sc.on_vio_event(dict(msg))
        if got:
            self.hit += 1
        else:
            self.miss += 1
        self.sc.mark_pose_seen(t_ns)

    def reset(self):
        pass

    def pending(self):
        return 0

    def close(self):
        pass

    def stats(self):
        L = self.latest or {}
        return dict(addr=f"reuse:{self.src}", init=bool(L.get("init")), status=L.get("status"), mono=L.get("mono"),
                    t=L.get("t"), n_slam=L.get("n_slam"), n_msckf=L.get("n_msckf"), poses=self.hit, reused_from=self.src,
                    frames_without_pose=self.miss)


class Replayer:
    def __init__(self, sc, rec_dir, every_frame=False, iters=8, out_dir=None, save_partial=True, auto=False, pair=None,
                 reuse_vio=True):
        """pair = 쓸 두 렌즈 ["2", "6"] (None = 녹화 앞 두 렌즈). reuse_vio = 다른 쌍 복원의 포즈를 다시 씀 (3렌즈 녹화만)."""
        self.sc, self.rec_dir = sc, Path(rec_dir)
        self.pair = [str(x) for x in pair] if pair else None
        self.reuse_vio = reuse_vio
        self.every_frame, self.iters = every_frame, iters
        self.save_partial = save_partial          # False (자동 복원 대기열): 중간에 멈추면 저장하지 않음 → 나중에 처음부터 다시
        self.auto = auto
        self.rec = None
        self.out_dir = Path(out_dir) if out_dir else self.rec_dir / time.strftime("recon_%Y%m%d-%H%M%S")
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True, name="replay")
        self.progress = dict(state="준비", name=self.rec_dir.name, dir=str(self.rec_dir), out=str(self.out_dir),
                             every_frame=every_frame, iters=iters, pos_s=0.0, total_s=0.0, pairs=0, frames_total=0,
                             keyframes=0, pose_timeouts=0, speed=None, error=None, started=time.time(), elapsed_s=0.0,
                             auto=auto, timeline=False, pair="+".join(self.pair) if self.pair else None,
                             vio_reused=None)

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
        pose_log = None
        reused = None
        try:
            P["state"] = "읽는 중"
            rec = recording.Recording(self.rec_dir, self._log, pair=self.pair)
            pair_str = "+".join(rec.physical[:2])
            multi = len(rec.physical_all) > 2
            self.rec = rec
            P.update(total_s=round(rec.duration_s, 2), frames_total=rec.n_frames)
            # 정류맵·OpenVINS 설정은 start_replay 가 meta.json 의 width/height/fps 로 이미 맞춰 둠 (scan3d.apply_geometry)
            if rec.size != tuple(sc.geom["size"]):
                raise ValueError(f"녹화 크기 {rec.size} ≠ 재생 정류 크기 {tuple(sc.geom['size'])}")
            P.update(size=list(rec.size), fps=sc.geom["fps"], calib_note=sc.geom["note"])
            if [str(p) for p in rec.physical[:2]] != [str(p) for p in sc.a.physical]:
                self._log(f"주의: 녹화 렌즈 {rec.physical} ≠ --physical {sc.a.physical}")
            P["state"] = "브리지 시작"
            if multi and self.reuse_vio and sc.a.ov != "none" and sc.a.vio_mode == "mono":
                reused, pj = find_pose_src(self.rec_dir, pair_str, rec.size, sc.geom["fps"], sc.a.vio_mode)
                if reused:
                    vio = PoseReplayVIO(sc, pj["entries"], reused)
                    sc.vio_cam0 = pj["vio_cam0"]
                    P["vio_reused"] = reused
                    self._log(f"위치 추적: {reused} ({pj.get('pair')}) 의 포즈 {pj.get('n_poses')} 개를 그대로 씀 "
                              f"(렌즈 2 만 쓰는 모노 VIO — 두 쌍이 같은 궤적, 깊이만 비교)")
            if vio is None:
                vio = sc.make_replay_vio()
                if multi and sc.a.ov != "none":
                    pose_log = []
                    sc.pose_log = pose_log                  # 이 복원의 포즈를 모아 다른 쌍 복원이 다시 씀
            if not rec.sensors and not sc.a.ov == "none":
                raise ValueError("imu.csv 없음 → VIO 불가 (--ov none 으로 고정 포즈 재생만 가능)")
            sc.begin_replay(vio, self.every_frame, self.iters)
            P["state"] = "모델 대기"
            while not self._stop.is_set() and not (sc.dstats.get("ready") and sc.ds is not None
                                                   and sc.ds.iters == sc.want_iters):
                if str(sc.dstats.get("error") or "").startswith("init:"):   # 모델 초기화 실패만 (지난 키프레임 오류는 무시, REVIEW5 N6)
                    raise RuntimeError(f"깊이 모델 오류: {sc.dstats['error']}")
                time.sleep(0.2)
            P["state"] = "재생 중"
            self._log(f"{self.rec_dir} · {rec.n_frames} 프레임 · {rec.duration_s:.1f} s · every_frame={self.every_frame} "
                      f"iters={self.iters} · {rec.size[0]}x{rec.size[1]} {sc.geom['fps']:g} fps · {sc.geom['note']} · 프레임 시각 {rec.ts_ref} "
                      f"→ {self.out_dir}")
            imu = rec.sensors
            k = 0
            t_start = time.monotonic()
            first = True
            pstats = {}
            skew = sc.pair_skew_ms()
            if multi and sc.a.pair_skew_ms is None:
                # 3렌즈 녹화: 렌즈 5 는 렌즈 2 와 반 주기쯤(약 31 ms) 어긋나 찍혀 0.45 주기(30 ms) 기준이면 짝이 하나도 안 될 수 있음
                # → 0.55 주기까지 (FramePairer 는 양쪽을 다 본 뒤 가장 가까운 프레임을 고르므로 겹치지 않음). 두 쌍 모두 같은 기준
                skew = max(skew, round(0.55 * 1000.0 / max(1, int(sc.geom["fps"])), 2))
            P["pair_skew_ms"] = skew
            for pair in rec.iter_pairs(skew, pstats):   # 기본 0.45 × 프레임 주기 (가장 가까운 프레임끼리)
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
            if not (stopped and not self.save_partial):
                seg = sc.restore_best_segment(self._log)
                if seg is not None:
                    P["best_segment"] = seg
            if stopped and not self.save_partial:
                self._log("중간에 멈춤 → 저장 안 함 (자동 복원은 나중에 처음부터 다시)")
            elif sc.vmap.n:
                P["state"] = "저장 중"
                import replay_view
                sc.objects = sc.vmap.objects(force=True)
                cmp = replay_view.quality_summary(list(sc.replay_tl), sc.vmap.count_shown(), int(sc.vmap.n))
                cmp.update(pair=pair_str, vio_reused=reused)
                d = sc.save(self.out_dir, extra=dict(replay=dict(P, recording=str(self.rec_dir), stopped=stopped,
                                                                 meta=rec.meta, vio=sc.vio.stats()), compare=cmp))
                # 리플레이용 시간 정보 (replay_view.py): 복셀별 처음 보인 시각, 키프레임 목록·자세, 궤적, 깊이 그림
                tl = replay_view.write_replay_extras(d, sc, rec, complete=not stopped, pair=pair_str, compare=cmp,
                                                     vio_reused=reused)
                if pose_log is not None and not stopped:
                    self._save_pose_log(d, pose_log, rec, pair_str)
                P["timeline"] = True
                self._log(f"시간 정보 저장: 키프레임 {len(tl['keyframes'])} · 복셀 {tl['voxels']} → {d / 'timeline.json'}")
                P["state"] = "카메라 화면 준비"
                try:
                    replay_view.ensure_thumbs(self.rec_dir, self._log)
                except Exception as e:
                    self._log(f"카메라 화면 그림 만들기 실패 (리플레이는 영상 없이): {e!r}")
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
            if sc.pose_log is pose_log:
                sc.pose_log = None
            P["elapsed_s"] = round(time.time() - P["started"], 1)
            sc.end_replay(self, vio)

    def _save_pose_log(self, d, log, rec, pair_str):
        """브리지 포즈·이벤트를 받은 순서대로 [종류 p|e, 키 (포즈 t_ns, 이벤트는 바로 앞 포즈 t_ns), msg] 로 저장."""
        sc = self.sc
        ents, last = [], -1
        for kind, msg in list(log):
            if kind == "p":
                try:
                    last = int(msg.get("t_ns", -1))
                except (TypeError, ValueError):
                    continue
            ents.append([kind, last, msg])
        if sc.vio_cam0 != "lens0u" or sc.a.vio_mode != "mono":
            self._log(f"포즈 저장 안 함 (VIO {sc.a.vio_mode} · cam0 {sc.vio_cam0} — 렌즈 쌍마다 달라 다시 쓸 수 없음)")
            return
        j = dict(format=POSE_LOG_FORMAT, recording=self.rec_dir.name, recon=Path(d).name, pair=pair_str,
                 size=list(rec.size), fps=int(sc.geom["fps"]), vio_mode=sc.a.vio_mode, vio_cam0=sc.vio_cam0,
                 vio_config=str(sc.vio_cfg_used) if sc.vio_cfg_used else None,
                 n_poses=sum(1 for e in ents if e[0] == "p"), created=time.strftime("%Y-%m-%d %H:%M:%S"), entries=ents)
        try:
            (Path(d) / POSE_LOG_FILE).write_text(json.dumps(j, separators=(",", ":"), default=float))
            self._log(f"위치 추적 포즈 {j['n_poses']} 개 저장 → {Path(d) / POSE_LOG_FILE} (다른 렌즈 쌍 복원이 다시 씀)")
        except (OSError, TypeError, ValueError) as e:
            self._log(f"포즈 저장 실패 {e!r}")


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
