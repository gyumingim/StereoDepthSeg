"""폰 녹화 폴더 읽기 (replay.py 용).

형식 = scan3d/RECORDING_FORMAT.md (scan3d-phone-rec/1). 이 파일이 쓰는 것:
  meta.json          physical ("2,5" 또는 ["2","5"]), width, height, complete (false 면 경고)
  cam2.mp4, cam5.mp4 렌즈별 H.264, 회전 메타 없음 (파일 이름의 숫자 = meta.physical 의 id).
  frames_camN.csv    frame_idx, physical_ts_ns (= 실시간 STER timestamp), logical_ts_ns, exposure_ns, rolling_shutter_skew_ns, pts_us
                     (-1 = 없음). 행은 인코더에 넣을 때 쓴다 (frame_idx 연속, 큐에서 버린 프레임은 행 없음).
                     노출 중앙 보정 = 0.5·(exposure + skew), 실시간 phone_rx 와 같은 식.
  imu.csv            type, t_ns, v0.. (행 길이 가변, 실시간 SENS 패킷과 같은 값, 100 = frame_timing, t_ns = logical ts)
시계: 모든 *_ns 는 elapsedRealtimeNanos (실시간 스트림과 같은 기준) → 프레임·IMU 를 바로 비교.
영상 프레임 ↔ CSV 행 맞추기 (Stream.iter_frames):
  처음 12 장으로 방식과 첫 행을 정한다: 디코드 PTS 가 순증가하면 PTS 방식 — (디코드 PTS − 첫 디코드 PTS) ≈ (pts_us − 행 r 의 pts_us),
  오차 ≤ 반 프레임, r(0..4) 는 맞은 수·오차로 고름 (드롭 간격 무늬; 간격이 일정해 못 가르면 0, meta samples_written 와 행 수가 다르면 경고).
  PTS 가 없으면(0 이 반복 등) 처음부터 순서(디코드 n 번째 = 행 r+n, B-프레임 없음 가정). PTS 가 중간에 계속 안 맞아 순서로 바꿀 때는
  마지막으로 맞은 (행, 디코드 번호) 에서 이어가므로 버린 프레임만큼 행도 나아간다.
  행 수 > 영상 프레임 수 (EOS 시간초과·강제 종료로 끝이 잘림) → 영상 끝에서 멈춤, 남은 행 버림 (로그).
  영상 프레임 수 > 행 수 (CSV 마지막 flush 전 강제 종료) → 행 없는 프레임은 시각을 모르므로 버림 (로그).
열 이름 별칭도 받는다 (t_ns/timestamp_ns, exposure_ms…), 넓은 IMU 형식 t_ns,gx,gy,gz,ax,ay,az 도 받는다.
노출 값: imu.csv frame_timing(100) 을 먼저 넣고 frames CSV 의 노출 열(있으면)로 덮어쓴다 (FrameTimingIndex, 실시간과 같은 조회).
"""
import csv
import heapq
import json
from pathlib import Path

import cv2

import lib_phone_stereo as lps
import phone_rx

NAME_TO_TYPE = {"accelerometer": 1, "accel": 1, "acc": 1, "gyroscope": 4, "gyro": 4,
                "gyroscope_uncalibrated": 16, "gyro_uncal": 16, "accelerometer_uncalibrated": 35, "accel_uncal": 35,
                "frame_timing": 100, "pressure": 6, "gravity": 9, "linear_acceleration": 10, "rotation_vector": 11,
                "game_rotation_vector": 15}
T_COLS = ("physical_ts_ns", "t_ns", "timestamp_ns", "sensor_ts_ns", "ts_ns", "timestamp")
IDX_COLS = ("frame", "frame_idx", "idx", "index", "frame_index")


def _col(header, names):
    low = [h.strip().lower() for h in header]
    for n in names:
        if n in low:
            return low.index(n)
    return None


def _read_csv(path):
    with open(path, newline="") as f:
        rows = [r for r in csv.reader(f) if r and not r[0].lstrip().startswith("#")]
    if not rows:
        return [], []
    return rows[0], rows[1:]


class Stream:
    def __init__(self, index, physical, video, frames_csv):
        self.index, self.physical, self.video, self.frames_csv = index, physical, Path(video), Path(frames_csv)
        self.samples_written = None          # meta lenses[id].samples_written (mp4 sample 수), Recording 이 채움
        header, rows = _read_csv(self.frames_csv)
        ti = _col(header, T_COLS)
        if ti is None:
            raise ValueError(f"{self.frames_csv.name}: 시각 열(t_ns) 없음, 머리줄 {header}")
        ii = _col(header, IDX_COLS)
        en, em = _col(header, ("exposure_ns", "exp_ns")), _col(header, ("exposure_ms", "exp_ms"))
        sn = _col(header, ("rolling_shutter_skew_ns", "rs_skew_ns", "skew_ns"))
        sm = _col(header, ("rs_skew_ms", "skew_ms"))
        pi = _col(header, ("pts_us",))

        def num(r, i, scale):
            if i is None:
                return None
            v = float(r[i])
            return None if v < 0 else v * scale              # -1 = 없음
        recs = []
        self.pts = {}
        for k, r in enumerate(rows):
            try:
                t = int(r[ti]) if r[ti].strip().lstrip("-").isdigit() else int(float(r[ti]))
                idx = int(r[ii]) if ii is not None else k
                exp = num(r, en, 1e-6) if en is not None else num(r, em, 1.0)
                rs = num(r, sn, 1e-6) if sn is not None else num(r, sm, 1.0)
                if pi is not None:
                    self.pts[idx] = int(float(r[pi]))
            except (ValueError, IndexError):
                continue
            if t <= 0:
                continue
            recs.append((idx, t, exp, rs))
        recs.sort()
        self.recs = recs
        self.has_exposure = any(e is not None for _, _, e, _ in recs)

    def __len__(self):
        return len(self.recs)

    PROBE = 12               # 맞추기 방식·오프셋을 정할 때 보는 처음 디코드 프레임 수
    MAX_OFF = 4              # 첫 디코드 프레임이 맞을 수 있는 CSV 행 (0..MAX_OFF, 인코더가 앞 프레임을 안 내보낸 경우)

    def _row_by_pts(self, rel_us, k0, tol_us, base_us):
        """k0 부터 (pts_us − base_us) 가 rel_us 에 가장 가까운 행 번호 (tol 안), 없으면 None."""
        best, bd = None, tol_us
        for k in range(k0, len(self.recs)):
            p = self.pts.get(self.recs[k][0])
            if p is None:
                continue
            d = (p - base_us) - rel_us
            if abs(d) <= bd:
                best, bd = k, abs(d)
            if d > tol_us:
                break
        return best

    def _decide(self, head_ms, tol_us, dt_us, log):
        """처음 몇 디코드 프레임의 PTS(ms) 로 (by_pts, 첫 디코드 프레임의 행 번호) 를 정한다 (REVIEW2 2·3).

        PTS 방식: 디코드 PTS 가 모두 ≥0 이고 순증가, CSV pts_us 가 있음. 오프셋 r (0..MAX_OFF) 마다
        (디코드 PTS − 첫 디코드 PTS) 를 (pts_us − 행 r 의 pts_us) 에 맞춰 보고 맞은 수가 가장 많고 오차가 작은 r 을 고른다
        (드롭으로 생긴 간격 무늬가 오프셋을 가른다). 간격이 일정하면 r 을 가를 수 없음 → 0 으로 두고 meta 와 개수가 다르면 경고.
        그 밖에는 순서 방식 (디코드 n 번째 = 행 n)."""
        name = self.video.name
        if not self.recs or not head_ms:
            return False, 0
        if len(head_ms) < 2 or any(p is None or p < 0 for p in head_ms) \
                or any(b <= a for a, b in zip(head_ms, head_ms[1:])):
            shown = [round(p, 1) if p is not None else None for p in head_ms[:4]]
            log(f"[rec] {name}: 디코더 PTS 를 못 씀 (처음 {len(head_ms)} 장 {shown} ms…) → 순서 맞추기")
            return False, 0
        n_off = min(self.MAX_OFF + 1, len(self.recs))
        if any(self.pts.get(self.recs[r][0]) is None for r in range(n_off)):
            log(f"[rec] {name}: CSV pts_us 없음 → 순서 맞추기")
            return False, 0
        rel = [(p - head_ms[0]) * 1000.0 for p in head_ms]
        scores = []
        for r in range(n_off):
            base = self.pts[self.recs[r][0]]
            hit, err, k = 0, 0.0, r
            for d in rel:
                m = self._row_by_pts(d, k, tol_us, base)
                if m is None:
                    continue
                hit += 1
                err += abs((self.pts[self.recs[m][0]] - base) - d)
                k = m + 1
            scores.append((hit, err, r))
        best_hit = max(s[0] for s in scores)
        if best_hit < max(2, len(rel) // 2):
            log(f"[rec] {name}: 처음 {len(rel)} 장 PTS 가 CSV 와 거의 안 맞음 (최대 {best_hit}) → 순서 맞추기")
            return False, 0
        cands = [s for s in scores if s[0] == best_hit]
        best_err = min(s[1] for s in cands)
        ties = sorted(s[2] for s in cands if s[1] <= best_err + 0.1 * dt_us)
        r0 = ties[0]
        if len(ties) > 1:
            sw = self.samples_written
            if sw is not None and sw != len(self.recs):
                log(f"[rec] {name}: 첫 디코드 프레임이 행 {ties} 중 어느 것인지 PTS 로 가를 수 없음, "
                    f"CSV {len(self.recs)} 행 ≠ meta samples_written {sw} → 행 {r0} 로 가정 "
                    f"(최대 {ties[-1] - r0} 프레임 어긋날 수 있음, 추정)")
        elif r0:
            log(f"[rec] {name}: 첫 디코드 프레임 = CSV 행 {r0} (앞 {r0} 행은 영상에 없음, PTS 간격 무늬로 판정)")
        return True, r0

    def iter_frames(self, size, log=print):
        """PhoneFrame 을 영상 순서대로. 영상 프레임 ↔ CSV 행은 PTS 로 맞추고 (못 쓰면 순서로), 짝 없는 쪽은 버린다.
        방식과 첫 행 오프셋은 처음 PROBE 장으로 정한다. 중간에 순서 방식으로 바꿔도 행 번호는 디코드 번호와 같이 움직인다."""
        cap = cv2.VideoCapture(str(self.video))
        if not cap.isOpened():
            raise IOError(f"영상 열기 실패: {self.video} (OpenCV FFMPEG 필요; meta complete=false 면 mp4 마무리 안 됨)")
        try:
            cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)        # 회전 메타가 있어도 원래 센서 방향 그대로
        except Exception:
            pass
        W, H = size
        n_rows = len(self.recs)
        if n_rows >= 2:
            dt_us = (self.recs[-1][1] - self.recs[0][1]) / 1e3 / max(1, n_rows - 1)
        else:
            dt_us = 33333.0
        tol_us = max(2000.0, 0.45 * dt_us)

        def read():
            ok, img = cap.read()
            if not ok:
                return None
            return img, cap.get(cv2.CAP_PROP_POS_MSEC)
        head = []
        while len(head) < self.PROBE:
            fr = read()
            if fr is None:
                break
            head.append(fr)
        by_pts, r0 = self._decide([p for _, p in head], tol_us, dt_us, log)
        base_us = self.pts.get(self.recs[r0][0]) if (by_pts and n_rows) else None
        dec0_us = head[0][1] * 1000.0 if (by_pts and head) else None

        def frames():
            yield from head
            while True:
                fr = read()
                if fr is None:
                    return
                yield fr
        k = r0                   # 다음에 찾을 첫 CSV 행
        n_dec = 0                # 디코드한 영상 프레임 수
        last_row = last_dec = None   # 마지막으로 짝지은 (행, 디코드 번호 0부터)
        warned = False
        skipped_rows = r0
        extra_frames = pts_miss = 0
        try:
            for dec_i, (bgr, pms) in enumerate(frames()):
                if k >= n_rows:
                    extra_frames = 1                             # 행보다 영상 프레임이 많음 (시각 모름 → 버림)
                    break
                n_dec += 1
                if by_pts:
                    m = None
                    if pms is not None and pms >= 0:
                        m = self._row_by_pts(pms * 1000.0 - dec0_us, k, tol_us, base_us)
                    if m is None:
                        pts_miss += 1
                        if pts_miss in (1, 30):
                            log(f"[rec] {self.video.name}: 디코드 PTS {pms:.1f} ms 에 맞는 CSV 행 없음 "
                                f"({pts_miss} 번) — 이 프레임 버림")
                        if pts_miss > 30 and pts_miss > n_dec // 2:
                            log(f"[rec] {self.video.name}: PTS 가 거의 안 맞음 → 순서 맞추기로 바꿈 "
                                f"(마지막 짝 행 {last_row} @ 디코드 {last_dec} 기준으로 이어감)")
                            by_pts = False
                        continue
                    kk = m
                else:
                    # 순서: 마지막 짝 이후 디코드 프레임 수만큼 행도 나아감 (버린 프레임도 행 하나씩 차지)
                    kk = (last_row + (dec_i - last_dec)) if last_row is not None else r0 + dec_i
                    if kk >= n_rows:
                        extra_frames = 1
                        break
                skipped_rows += kk - k
                idx, t, _, _ = self.recs[kk]
                last_row, last_dec = kk, dec_i
                k = kk + 1
                if bgr.shape[1] != W or bgr.shape[0] != H:
                    if (bgr.shape[1], bgr.shape[0]) == (H, W):
                        bgr = cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)   # 세로로 저장된 경우 (가정)
                    else:
                        bgr = cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA)
                    if not warned:
                        log(f"[rec] {self.video.name}: 영상 크기 {bgr.shape[1]}x{bgr.shape[0]} → {W}x{H} 로 맞춤 (확인 필요)")
                        warned = True
                yield lps.PhoneFrame(self.index, t, bgr)
            rest = max(0, n_rows - k)
            if rest or skipped_rows or pts_miss or extra_frames:
                log(f"[rec] {self.video.name}: 영상 {n_dec} 장 / CSV {n_rows} 행 — 짝 없는 행 {skipped_rows + rest} "
                    f"(앞 {r0}, 끝 {rest}), 행 없는 영상 프레임 {pts_miss}{' + 끝에 더 있음' if extra_frames else ''}")
        finally:
            cap.release()


class Recording:
    def __init__(self, path, log=print):
        self.path = Path(path)
        self.log = log
        if not self.path.is_dir():
            raise FileNotFoundError(f"녹화 폴더 없음: {self.path}")
        mp = self.path / "meta.json"
        self.meta = json.loads(mp.read_text()) if mp.is_file() else {}
        if self.meta and self.meta.get("complete") is False:
            self.log("[rec] meta.json complete=false — 정상 마무리 안 된 녹화 (mp4 가 안 열리면 .h264 백업 필요)"
                     + (" · finishing=true: 정지 시간 초과 뒤 인코더가 마무리 중이었고 끝 기록을 못 남김"
                        if self.meta.get("finishing") else ""))
        cp = self.path / "cameras.json"
        self.cameras = json.loads(cp.read_text()) if cp.is_file() else None
        phys = self.meta.get("physical") or ["2", "5"]
        if isinstance(phys, str):
            phys = [p.strip() for p in phys.split(",")]
        self.physical = [str(p) for p in phys]
        self.size = (int(self.meta.get("width", 640)), int(self.meta.get("height", 480)))
        self.streams = self._streams()
        self.sensors = self._load_imu()
        self.timing = self._timing()
        t_all = [r[1] for s in self.streams for r in s.recs]
        self.t_first = min(t_all) if t_all else 0
        self.t_last = max(t_all) if t_all else 0
        self.duration_s = (self.t_last - self.t_first) * 1e-9
        self.n_frames = min(len(s) for s in self.streams) if self.streams else 0

    def _streams(self):
        out = []
        spec = self.meta.get("streams")
        if spec:
            for i, s in enumerate(spec):
                idx = int(s.get("index", i))
                out.append(Stream(idx, str(s.get("physical", self.physical[idx] if idx < len(self.physical) else idx)),
                                  self.path / s["video"], self.path / s["frames"]))
        else:
            for i, p in enumerate(self.physical[:2]):
                v = self.path / f"cam{p}.mp4"
                fc = self.path / f"frames_cam{p}.csv"
                if not v.is_file() or not fc.is_file():
                    raise FileNotFoundError(f"렌즈 {p}: {v.name} / {fc.name} 없음")
                out.append(Stream(i, p, v, fc))
        out.sort(key=lambda s: s.index)
        if len(out) < 2:
            raise ValueError("스트림이 2개 필요 (렌즈 2, 5)")
        lens_meta = self.meta.get("lenses") or {}
        for s in out:
            lm = lens_meta.get(str(s.physical)) or {}
            note = ""
            try:
                s.samples_written = int(lm["samples_written"]) if lm.get("samples_written") is not None else None
            except (TypeError, ValueError):
                s.samples_written = None
            if lm:
                note = f" · meta frames={lm.get('frames')} dropped={lm.get('dropped_queue')}+{lm.get('dropped_encoder_busy')}"
                if lm.get("muxer_ok") is False or lm.get("error"):
                    note += f" · 경고: muxer_ok={lm.get('muxer_ok')} error={lm.get('error')}"
            self.log(f"[rec] 스트림 {s.index} 렌즈 {s.physical}: {s.video.name} CSV {len(s)} 행"
                     + (" (노출 열 있음)" if s.has_exposure else "") + note)
        return out

    def _load_imu(self):
        p = self.path / "imu.csv"
        if not p.is_file():
            self.log("[rec] imu.csv 없음 → VIO 불가 (--ov none 로만 재생 가능)")
            return []
        header, rows = _read_csv(p)
        ti = _col(header, T_COLS)
        tyi = _col(header, ("type", "sensor", "sensor_type"))
        out = []
        if ti is None:
            raise ValueError(f"imu.csv 시각 열 없음: {header}")
        if tyi is not None:
            vcols = [i for i in range(len(header)) if i not in (ti, tyi)]
            for r in rows:
                try:
                    ty = r[tyi].strip()
                    ty = int(ty) if ty.lstrip("-").isdigit() else NAME_TO_TYPE[ty.lower()]
                    t = int(r[ti])
                    vals = tuple(float(r[i]) for i in vcols if i < len(r) and r[i].strip() != "")
                except (ValueError, KeyError, IndexError):
                    continue
                out.append(lps.SensorSample(ty, t, vals))
        else:
            g = [_col(header, (n,)) for n in ("gx", "gy", "gz")]
            a = [_col(header, (n,)) for n in ("ax", "ay", "az")]
            if None in g or None in a:
                raise ValueError(f"imu.csv 형식을 모름: {header}")
            for r in rows:
                try:
                    t = int(r[ti])
                    out.append(lps.SensorSample(phone_rx.SENSOR_GYRO, t, tuple(float(r[i]) for i in g)))
                    out.append(lps.SensorSample(phone_rx.SENSOR_ACCEL, t, tuple(float(r[i]) for i in a)))
                except (ValueError, IndexError):
                    continue
        out.sort(key=lambda s: s.timestamp_ns)
        n = {}
        for s in out:
            n[s.type] = n.get(s.type, 0) + 1
        self.log(f"[rec] imu.csv {len(out)} 샘플 {n}")
        return out

    def _timing(self):
        ti = phone_rx.FrameTimingIndex(len(self.streams), keep=1 << 30)
        for smp in self.sensors:                                 # imu.csv frame_timing (실시간 SENS 100 과 같은 값)
            if smp.type == phone_rx.SENSOR_FRAME_TIMING:
                ti.add(smp.timestamp_ns, smp.values)
        if self.meta.get("raw_ts") is True:
            self.log("[rec] raw_ts=true 녹화: physical_ts_ns 열 = logical 값 (frame_timing 조회가 안 맞을 수 있음)")
        for s in self.streams:                                   # frames CSV 의 프레임별 값이 있으면 그것으로 덮어씀
            for _, t, e, r in s.recs:
                if e is not None:
                    ti.put(s.index, t, e, r or 0.0)
        return ti

    def iter_frames(self):
        """두 스트림 프레임을 시각 순서로 합쳐서 (실시간 도착 순서 근사)."""
        its = [s.iter_frames(self.size, self.log) for s in self.streams]
        heap = []
        for k, it in enumerate(its):
            f = next(it, None)
            if f is not None:
                heap.append((f.timestamp_ns, k, f))
        heapq.heapify(heap)
        while heap:
            _, k, f = heapq.heappop(heap)
            yield f
            nf = next(its[k], None)
            if nf is not None:
                heapq.heappush(heap, (nf.timestamp_ns, k, nf))

    def iter_pairs(self, max_skew_ms=10.0, stats=None):
        """실시간 StreamReceiver 와 같은 규칙(역행 폐기 + FramePairer)으로 쌍을 만든다."""
        pairer = lps.FramePairer(len(self.streams), max_skew_ms)
        last = [None] * len(self.streams)
        st = stats if stats is not None else {}
        st.setdefault("reordered", 0)
        st.setdefault("frames", 0)
        for f in self.iter_frames():
            st["frames"] += 1
            i = f.index
            if last[i] is not None and f.timestamp_ns <= last[i]:
                st["reordered"] += 1
                continue
            last[i] = f.timestamp_ns
            pair = pairer.add(f)
            if pair:
                yield pair
        st["unmatched_dropped"] = list(pairer.dropped)
