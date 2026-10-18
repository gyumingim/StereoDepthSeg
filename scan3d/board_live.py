"""매 프레임 보정판(ChArUco) 인식 표시 — IMU 학습용 녹화에서 정답 궤적이 끊기지 않는지 실시간으로 보기 위함 (IMU_PLAN.md 4).

scan3d 의 짝 프레임(on_pair)마다 광각 렌즈(쌍의 두 번째 = 렌즈 5) 영상에서 보드를 찾는다. 별도 스레드, 밀리면 최신 프레임만
처리하고 건너뛴 수를 센다. 웹 상태 'board_live': 최근 2 s 인식률, 지금 코너 수, 세션 누적, 가장 긴 끊김.
깊이 워커의 aruco_scale 관측(키프레임만, 축척용)과는 별개.
"""
import collections
import threading
import time

import cv2

import aruco_scale


class BoardLive:
    def __init__(self, board_json, lens_index=1, min_corners=6, log=print):
        self.board = aruco_scale.Board(aruco_scale.load_board_spec(board_json))
        self.lens_index = lens_index          # 쌍에서 몇 번째 렌즈 (2+5 → 1 = 렌즈 5)
        self.min_corners = min_corners        # 이만큼 코너가 보여야 '인식' (자세를 풀 수 있는 최소보다 넉넉히)
        self.log = log
        self.lock = threading.Lock()
        self.evt = threading.Event()
        self.latest = None
        self.stopped = False
        self.reset()
        threading.Thread(target=self._run, daemon=True, name="board-live").start()

    def reset(self):
        with getattr(self, "lock", threading.Lock()):
            self.recent = collections.deque()     # (t_wall, ok, n_corners)
            self.n_frames = self.n_ok = self.n_skipped = 0
            self.last_ok_t = None
            self.max_gap_s = 0.0
            self.gap_start = None
            self.last_corners = 0
            self.det_ms = 0.0
            self.t_start = time.time()

    def push(self, pair):
        """on_pair 에서 부름 (막지 않음)."""
        if self.latest is not None:
            self.n_skipped += 1
        self.latest = pair
        self.evt.set()

    def _run(self):
        while not self.stopped:
            self.evt.wait(0.5)
            self.evt.clear()
            pair, self.latest = self.latest, None
            if pair is None:
                continue
            try:
                f = pair[self.lens_index]
                gray = cv2.cvtColor(f.bgr, cv2.COLOR_BGR2GRAY) if f.bgr.ndim == 3 else f.bgr
                t0 = time.perf_counter()
                corners, ids, nm = self.board.detect(gray)
                ms = (time.perf_counter() - t0) * 1e3
                self._add(len(ids), ms)
            except Exception as e:
                self.log(f"[board-live] {e!r}")

    def _add(self, n_corners, ms):
        now = time.time()
        ok = n_corners >= self.min_corners
        with self.lock:
            self.n_frames += 1
            self.n_ok += ok
            self.last_corners = n_corners
            self.det_ms = 0.8 * self.det_ms + 0.2 * ms if self.det_ms else ms
            self.recent.append((now, ok, n_corners))
            while self.recent and now - self.recent[0][0] > 2.0:
                self.recent.popleft()
            if ok:
                if self.gap_start is not None and self.last_ok_t is not None:
                    self.max_gap_s = max(self.max_gap_s, now - self.last_ok_t)
                self.gap_start = None
                self.last_ok_t = now
            elif self.gap_start is None:
                self.gap_start = now

    def status(self):
        with self.lock:
            r = list(self.recent)
            now = time.time()
            gap_now = (now - self.last_ok_t) if self.last_ok_t is not None else None
            return dict(
                seen=bool(r and r[-1][1]),
                rate_2s=round(100.0 * sum(o for _, o, _ in r) / len(r), 1) if r else None,
                corners=self.last_corners,
                frames=self.n_frames, ok=self.n_ok, skipped=self.n_skipped,
                ok_pct=round(100.0 * self.n_ok / self.n_frames, 1) if self.n_frames else None,
                gap_now_s=round(gap_now, 2) if gap_now is not None else None,
                max_gap_s=round(max(self.max_gap_s, gap_now or 0.0) if self.gap_start is not None else self.max_gap_s, 2),
                det_ms=round(self.det_ms, 1), lens=self.lens_index, since_s=round(now - self.t_start, 1))
