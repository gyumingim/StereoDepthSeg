"""폰 녹화 자동 가져오기 + 자동 복원 대기열 (scan3d.Scanner 안의 스레드 하나).

하는 일 (4 s 마다, 또는 kick):
  1) 폰이 붙으면 (adb devices 에 device 가 새로 보이면 · 서버 시작 때 이미 붙어 있으면 · [정지] 로 녹화가 끝나면 · 30 s 마다)
     폰의 rec_* 목록(phone_link.list_recordings)과 각 녹화의 meta.json(complete)·파일 크기(stat)를 읽는다.
  2) 끝난(complete=true) 녹화 중 PC(scan3d/recordings/)에 같은 사본이 없는 것을 하나씩 가져온다.
     같은 사본 = 폰 파일이 모두 PC 에 같은 크기로 있음 (stat 을 못 읽으면 전체 크기 ±2 %). 녹화 중·덜 끝난 것은 건너뜀.
     가져오기는 recordings/.incoming/ 에 받은 뒤 바꿔 넣는다 (PC 에 있던 recon_* 폴더는 새 폴더로 옮겨 보존).
  3) 시간 정보가 있는 복원(replay_view.best_recon)이 없는 녹화를 하나씩 복원한다 (scan3d.start_replay auto=True).
     실시간·녹화 세션 중에는 멈춰 있다가 끝나면 이어서 (중간에 끊긴 자동 복원은 저장하지 않고 처음부터 다시).
     사용자가 [정지] 로 멈춘 자동 복원은 [다시 시도] 전까지 다시 안 함. 실패한 것도 [다시 시도] 전까지 안 함.
     비교용 3렌즈 녹화(2,5,6)는 두 쌍을 순서대로 복원 (2+6 → 2+5, replay_view.rec_pairs; 실패·멈춤도 쌍마다 따로 기억).
설정 runs/rec_auto.json {"pull": true, "recon": true} (웹에서 켜고 끔). 폰에서 지우기는 웹에서 물어본 뒤에만 (자동으로 안 지움).
"""
import json
import re
import shutil
import hashlib
import threading
import time
from pathlib import Path

import phone_link
import replay_view

HERE = Path(__file__).resolve().parent
CFG_FILE = HERE / "runs" / "rec_auto.json"
UPLOAD_STATE = HERE / "runs" / "upload_state.json"     # upload_server.py (폰이 HTTPS 로 올리는 녹화)
UPLOAD_HOLD_S = 180.0      # 업로드 방식이 켜져 있으면 폰에서 끝난 지 이만큼은 adb 자동 가져오기를 미룸 (앱이 올릴 시간)
# 자동 복원 실패 기록 (서버를 다시 켜도 남음): {열쇠: {reason, at, code, rec_bytes}}. 같은 녹화(크기)·같은 코드 버전이면
# 다시 시도하지 않음 → 웹 [다시 시도] (retry) 로만. 맵이 비어 실패한 것만 (모델 오류 같은 일시적 실패는 메모리에만)
FAIL_FILE = HERE / "runs" / "rec_recon_failed.json"
# 코드 버전 = 복원 결과를 바꾸는 파일 내용의 해시 (이 중 하나라도 고치면 실패했던 녹화를 한 번 다시 시도)
CODE_FILES = ("scan3d.py", "replay.py", "recording.py", "frames.py", "motion_rect.py", "depth_seg.py", "ov_client.py",
              "voxel_map.py", "tsdf_map.py", "reloc.py", "replay_view.py", "make_ov_config.py", "ov_bridge/main.cpp")


def code_version():
    h = hashlib.sha1()
    for f in CODE_FILES:
        try:
            h.update(f.encode() + b"\0" + (HERE / f).read_bytes())
        except OSError:
            h.update(f.encode() + b"\0-")
    return h.hexdigest()[:12]
POLL_S = 4.0
PHONE_RESCAN_S = 30.0
PHONE_MODES = ("live", "record", "both")
REC_RE = re.compile(r"^rec_[\w.\-]+$")


class RecSync:
    def __init__(self, sc, enabled=True):
        self.sc = sc
        self.evt = threading.Event()
        self.cfg = dict(pull=True, recon=True)
        try:
            self.cfg.update({k: bool(v) for k, v in json.loads(CFG_FILE.read_text()).items() if k in self.cfg})
        except (OSError, ValueError, AttributeError):
            pass
        if not enabled:
            self.cfg = dict(pull=False, recon=False)
        self.phone = None              # [{name, base, size_kb, complete, files}]
        self.phone_t = 0.0
        self.phone_err = None
        self.dev = False
        self.force_scan = True
        self.info_cache = {}           # (name, size_kb) → dict(complete, files)
        self.pulling = None            # dict(name, done, total, state)
        self.pull_q = []               # 웹 [가져오기] 요청 [{name, replay, every_frame, iters}]
        self.recon_first = []          # 먼저 복원할 이름 (웹 [다시 시도] 등)
        self.failed = {}               # 복원 실패 name → 이유
        self.pull_failed = {}          # 가져오기 실패 name → 이유
        self.held = set()              # 사용자가 [정지] 로 멈춘 자동 복원
        self.job = None                # dict(name, rp, t0, pair)
        self.waiting_stop = None       # [정지] 뒤 앱 마무리를 기다리는 녹화 이름
        self.thread = None
        self.code = code_version()
        self.seen_complete = {}        # 폰 녹화 이름 → 처음 complete 로 본 시각 (업로드 대기)
        self._load_failed()

    # ---------------- 실패 기록 (파일) ----------------
    def _rec_bytes(self, name):
        lf = self._local_files(name)
        return sum(lf.values()) if lf else None

    def _read_fail_file(self):
        try:
            d = json.loads(FAIL_FILE.read_text())
            return d if isinstance(d, dict) else {}
        except (OSError, ValueError):
            return {}

    def _load_failed(self):
        """서버 시작: 같은 코드 버전·같은 녹화 크기로 '맵 비었음' 실패한 녹화는 다시 시도하지 않게 self.failed 에 넣음."""
        n = 0
        for k, v in self._read_fail_file().items():
            if not isinstance(v, dict) or v.get("code") != self.code:
                continue
            if v.get("rec_bytes") != self._rec_bytes(k.split("|", 1)[0]):
                continue                                    # 녹화가 바뀜 (다시 가져옴) → 다시 시도
            self.failed[k] = f"{v.get('reason')} ({v.get('at')} 실패, 같은 코드 — [다시 시도] 로만 다시)"
            n += 1
        if n:
            self.log(f"자동 복원 실패 기록 {n} 개 불러옴 (코드 {self.code}, 다시 시도 안 함 — 웹 [다시 시도] 로만)")

    def _save_failed(self, key, reason=None):
        """reason None = 기록 지움 (retry)."""
        d = self._read_fail_file()
        if reason is None:
            if key not in d:
                return
            d.pop(key, None)
        else:
            d[key] = dict(reason=reason, at=time.strftime("%Y-%m-%d %H:%M:%S"), code=self.code,
                          rec_bytes=self._rec_bytes(key.split("|", 1)[0]))
        try:
            FAIL_FILE.parent.mkdir(parents=True, exist_ok=True)
            tmp = FAIL_FILE.with_suffix(".part")
            tmp.write_text(json.dumps(d, ensure_ascii=False, indent=1))
            tmp.replace(FAIL_FILE)
        except OSError as e:
            self.log(f"실패 기록 저장 못함 {e!r}")

    # ---------------- 바깥에서 부르는 것 ----------------
    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._loop, daemon=True, name="rec-sync")
            self.thread.start()

    def kick(self, scan=True):
        if scan:
            self.force_scan = True
        self.evt.set()

    def log(self, msg):
        self.sc.log(f"[rec] {msg}")

    def set_cfg(self, pull=None, recon=None):
        if pull is not None:
            self.cfg["pull"] = bool(pull)
        if recon is not None:
            self.cfg["recon"] = bool(recon)
        try:
            CFG_FILE.parent.mkdir(parents=True, exist_ok=True)
            CFG_FILE.write_text(json.dumps(self.cfg))
        except OSError:
            pass
        self.kick()
        return dict(ok=True, **self.cfg)

    @staticmethod
    def _key(name, pair=None):
        """실패·멈춤 기록 열쇠: 2렌즈 녹화 = 이름, 3렌즈 녹화 = 이름|쌍."""
        return name if not pair else f"{name}|{pair}"

    def retry(self, name):
        for d in (self.failed, self.pull_failed):
            for k in [k for k in d if k == name or k.startswith(name + "|")]:
                d.pop(k, None)
        for k in [k for k in self._read_fail_file() if k == name or k.startswith(name + "|")]:
            self._save_failed(k, None)
        for k in [k for k in self.held if k == name or k.startswith(name + "|")]:
            self.held.discard(k)
        if name and name not in self.recon_first:
            self.recon_first.append(name)
        self.kick()
        return dict(ok=True, name=name)

    def after_record_stop(self, rec, stopper):
        """[정지] (녹화·녹화+실시간): 앱이 mp4 를 마무리하면 바로 폰 목록을 다시 읽고 가져오기 → 복원."""
        self.waiting_stop = rec or "*"

        def w():
            try:
                if stopper is not None:
                    stopper.join(40.0)
                time.sleep(1.0)
            finally:
                self.waiting_stop = None
                if rec:
                    self.log(f"녹화 {rec} 끝남 → " + ("PC 로 가져옵니다" if self.cfg["pull"] else "자동 가져오기 꺼짐 (웹에서 [가져오기])"))
                self.kick()
        threading.Thread(target=w, daemon=True, name="rec-stop-wait").start()
        return dict(name=rec, auto_pull=self.cfg["pull"], auto_recon=self.cfg["recon"])

    def on_user_stop(self):
        """사용자가 [정지] 로 복원을 멈춤 → 그 녹화는 [다시 시도] 전까지 자동으로 다시 안 함."""
        j = self.job
        if j is not None:
            self.held.add(self._key(j["name"], j.get("pair")))

    def request_pull(self, name, replay=True, every_frame=False, iters=8):
        if not REC_RE.match(name or ""):
            return dict(ok=False, error=f"녹화 이름이 이상함: {name!r}")
        if not self.sc.link.available:
            return dict(ok=False, error="adb 없음")
        ph = next((r for r in (self.phone or []) if r["name"] == name), None)
        if ph is not None and self._same(ph, self._local_files(name)):
            if replay:
                return dict(self.sc.start_replay(str(self._local_dir(name)), every_frame, iters), note="PC 에 이미 있음 → 바로 복원")
            return dict(ok=True, note="PC 에 이미 같은 사본이 있음")
        if any(q["name"] == name for q in self.pull_q) or (self.pulling or {}).get("name") == name:
            return dict(ok=True, note="이미 가져오는 중")
        self.pull_failed.pop(name, None)
        self.pull_q.append(dict(name=name, replay=replay, every_frame=every_frame, iters=iters))
        self.kick()
        return dict(ok=True, name=name, queued=True)

    def delete_phone(self, name):
        """PC 에 같은 사본이 있는 것만 폰에서 지운다 (웹이 먼저 물어봄)."""
        link = self.sc.link
        if not REC_RE.match(name or ""):
            return dict(ok=False, error="녹화 이름이 이상함")
        if not (link.available and self.dev):
            return dict(ok=False, error="폰이 연결돼 있지 않음")
        rec = next((r for r in link.list_recordings() if r["name"] == name), None)
        if rec is None:
            return dict(ok=False, error="폰에 그 녹화가 없음")
        if not rec["base"].startswith("/"):
            return dict(ok=False, error="앱 내부 저장소 녹화는 여기서 지우지 않음")
        if self.waiting_stop or (self.sc.mode in ("record", "both")
                                 and name == (self.sc.rec_status or {}).get("rec")):
            return dict(ok=False, error="녹화 중인 폴더")
        info = self._phone_info(rec)
        if not info["files"] or not self._same(dict(rec, **info), self._local_files(name)):
            return dict(ok=False, error="PC 사본이 폰과 똑같은지 확인하지 못해 지우지 않았습니다")
        rc, out = link.run("shell", "rm", "-rf", f"{rec['base']}/{name}", timeout=30)
        if rc != 0:
            return dict(ok=False, error=f"지우기 실패: {out}")
        self.log(f"폰에서 지움: {name} (PC 사본 확인함)")
        self.kick()
        return dict(ok=True, name=name)

    # ---------------- 업로드 (upload_server.py) ----------------
    @staticmethod
    def uploads():
        """지금 폰이 올리고 있는 녹화 {이름: dict(done, total, mbps)} (upload_state.json, 2 분 넘게 조용하면 뺌)."""
        try:
            st = json.loads(UPLOAD_STATE.read_text())
        except (OSError, ValueError):
            return {}
        if time.time() - float(st.get("t") or 0) > 60:
            return {}
        return {a["name"]: a for a in st.get("active") or [] if a.get("idle_s", 999) < 120}

    def _upload_hold(self, name):
        """adb 자동 가져오기를 미룰까: 받는 중(.uploading/이름 이 30 분 안에 바뀜) 또는 업로드 방식이 켜져 있고 폰에서
        끝난 지 3 분이 안 됨. 웹 [가져오기] 는 미루지 않음."""
        d = Path(self.sc.a.rec_root) / ".uploading" / name
        try:
            # 올리다 만 것이 있으면 30 분 동안은 이어 올리기를 기다림 (터널 주소가 바뀌면 몇 분 멈췄다 이어짐, 2026-10-18)
            if time.time() - d.stat().st_mtime < (1800.0 if phone_link.upload_on() else UPLOAD_HOLD_S):
                return True
        except OSError:
            pass
        return phone_link.upload_on() and time.time() - self.seen_complete.get(name, 0) < UPLOAD_HOLD_S

    # ---------------- 상태 (웹) ----------------
    def snapshot(self):
        j = self.job
        jp = None
        if j is not None:
            P = j["rp"].progress
            jp = dict(name=j["name"], state=P.get("state"), pos_s=P.get("pos_s"), total_s=P.get("total_s"), pair=j.get("pair"))
        pl = self.pulling
        return dict(pull=self.cfg["pull"], recon=self.cfg["recon"], phone=self.dev, phone_t=self.phone_t,
                    phone_error=self.phone_err, pulling=dict(pl) if pl else None, job=jp,
                    paused=self.sc.mode in PHONE_MODES, upload=self.upload_info())

    @staticmethod
    def upload_info():
        """웹 한 줄: 업로드 방식 켜짐·터널 상태·폰 설정·최근 받은 것 (upload_state.json). 없으면 None."""
        try:
            st = json.loads(UPLOAD_STATE.read_text())
        except (OSError, ValueError):
            return None
        alive = time.time() - float(st.get("t") or 0) < 60
        return dict(alive=alive, enabled=st.get("enabled"), url=st.get("url"), tunnel=st.get("tunnel"),
                    phone_cfg_ok=st.get("phone_cfg_ok"), recent=(st.get("recent") or [])[-3:])

    def items(self):
        """녹화마다 한 줄: state = pulling | recon | ready | recon_wait | held | failed | bad | pc_only |
        recording | incomplete | pull_wait | pull_failed | phone_only | mismatch."""
        local = {r["name"]: r for r in phone_link.list_local(self.sc.a.rec_root)}
        phone = {r["name"]: r for r in (self.phone or [])}
        pl, j = self.pulling, self.job
        rp_now = self.sc.replayer
        if j is None and rp_now is not None and rp_now.thread.is_alive():                  # 웹에서 직접 누른 복원도 같은 줄에 진행률
            j = dict(name=rp_now.rec_dir.name, rp=rp_now, pair=rp_now.progress.get("pair"))
        cur = self._recording_now()
        paused = self.sc.mode in PHONE_MODES
        ups = self.uploads()
        out = []
        for n in sorted(set(local) | set(phone) | set(ups), reverse=True):
            if n in ups and n not in local:
                u = ups[n]
                out.append(dict(name=n, on_phone=n in phone, on_pc=False, same=False, recons=[], paused=paused,
                                phone_mb=round(u["total"] / 1048576, 1) if u.get("total") else None, pc_mb=None,
                                can_delete_phone=False, state="uploading", mbps=u.get("mbps"),
                                pct=round(100.0 * u["done"] / u["total"], 1) if u.get("total") else None))
                continue
            ph, lo = phone.get(n), local.get(n)
            lf = self._local_files(n) if lo else None
            same = bool(ph and lo and self._same(ph, lf))
            it = dict(name=n, on_phone=bool(ph), on_pc=bool(lo), same=same,
                      phone_mb=round(ph["size_kb"] / 1024, 1) if ph else None, pc_mb=lo["size_mb"] if lo else None,
                      recons=lo["recon"] if lo else [], paused=paused,
                      can_delete_phone=bool(same and ph and ph["base"].startswith("/") and ph.get("files")))
            ready = replay_view.best_recon(lo["path"]) if lo else None
            prs = replay_view.pair_recons(lo["path"]) if lo else None
            fail_n = self.failed.get(n)
            if prs:
                # 3렌즈 녹화: 쌍마다 결과·실패. 한 쌍이라도 되면 [보기], 모두 실패해야 실패
                it.update(pair_recons=prs, pair_failed={p: None if prs[p] else self.failed.get(self._key(n, p)) for p in prs},
                          pair_held=[p for p in prs if self._key(n, p) in self.held])
                ready = next((prs[p] for p in prs if prs[p]), None)
                fails = [self.failed.get(self._key(n, p)) for p in prs if not prs[p]]
                fail_n = fails[0] if fails and all(fails) and not ready else None
            if pl and pl.get("name") == n:
                it.update(state="pulling", pct=round(100.0 * pl["done"] / pl["total"], 1) if pl.get("total") else None)
            elif j is not None and j["name"] == n:
                P = j["rp"].progress
                tot = P.get("total_s") or 0
                it.update(state="recon", step=P.get("state"), recon_pair=j.get("pair"), recon=ready,
                          pct=round(min(100.0, 100.0 * (P.get("pos_s") or 0) / tot), 1) if tot else 0.0)
            elif ready:
                it.update(state="ready", recon=ready)
            elif lo and ph and not same and lo["recon"]:
                it.update(state="mismatch")
            elif lo:
                if fail_n:
                    it.update(state="failed", error=fail_n)
                elif n in self.held or (prs and it["pair_held"]):
                    it.update(state="held")
                elif lo.get("complete") is False or not lo.get("has_video"):
                    it.update(state="bad")
                elif self.cfg["recon"] or n in self.recon_first:
                    it.update(state="recon_wait")
                else:
                    it.update(state="pc_only")
            else:
                if not ph.get("complete"):
                    it.update(state="recording" if (cur and (cur == n or cur == "*")) else "incomplete")
                elif n in self.pull_failed:
                    it.update(state="pull_failed", error=self.pull_failed[n])
                elif self.cfg["pull"] or any(q["name"] == n for q in self.pull_q):
                    it.update(state="pull_wait")
                else:
                    it.update(state="phone_only")
            out.append(it)
        return out

    # ---------------- 루프 ----------------
    def _loop(self):
        while not self.sc.stopped:
            self.evt.wait(POLL_S)
            self.evt.clear()
            try:
                self._tick()
            except Exception as e:
                self.log(f"자동 가져오기·복원 오류: {e!r}")

    def _tick(self):
        sc, link = self.sc, self.sc.link
        dev = False
        if link.available:
            try:
                dev = bool(link.devices())
            except Exception:
                dev = False
        if dev and not self.dev:
            self.log("폰 연결됨 → 폰 녹화 목록을 읽습니다")
            self.force_scan = True
        self.dev = dev
        if dev:
            live = sc.mode in ("live", "both")
            if self.force_scan or (not live and time.time() - self.phone_t > PHONE_RESCAN_S):
                self.force_scan = False
                self._scan_phone()
        self._finish_job()
        if dev:
            self._pull_next()
        self._recon_next()

    def _recording_now(self):
        sc = self.sc
        if self.waiting_stop:
            return self.waiting_stop
        if sc.mode in ("record", "both"):
            return (sc.rec_status or {}).get("rec") or (sc.app_status or {}).get("rec") or "*"
        return None

    # ---------------- 폰 ----------------
    def _scan_phone(self):
        link = self.sc.link
        try:
            recs = link.list_recordings()
            self.phone_err = None
        except Exception as e:
            self.phone_err = repr(e)
            return
        out = []
        for r in recs:
            key = (r["name"], r["size_kb"])
            info = self.info_cache.get(key)
            if info is None or not info["complete"]:
                info = self._phone_info(r)
                self.info_cache[key] = info
            out.append(dict(r, **info))
        now = time.time()
        for r in out:
            if r.get("complete"):
                self.seen_complete.setdefault(r["name"], now)
        new = {r["name"] for r in out} - {r["name"] for r in (self.phone or [])}
        self.phone, self.phone_t = out, time.time()
        self.sc._phone_recs, self.sc._phone_recs_t = recs, self.phone_t
        if new:
            self.log(f"폰 녹화 {len(out)} 개 (새로 봄: {', '.join(sorted(new))})")

    def _phone_info(self, r):
        link = self.sc.link
        base, name = r["base"], r["name"]
        path = f"{base}/{name}"
        complete = False
        rc, txt = link._sh(base, "cat", f"{path}/meta.json", timeout=10)
        if rc == 0:
            try:
                complete = bool(json.loads(txt).get("complete"))
            except (ValueError, AttributeError):
                complete = False
        files = None
        if base.startswith("/"):
            rc, txt = link.run("shell", f"stat -c '%s %n' {path}/*", timeout=10)
            if rc == 0:
                files = {}
                for ln in txt.splitlines():
                    sz, _, fn = ln.strip().partition(" ")
                    if sz.isdigit() and fn:
                        files[fn.rsplit("/", 1)[-1]] = int(sz)
                files = files or None
        return dict(complete=complete, files=files)

    # ---------------- PC ----------------
    def _local_dir(self, name):
        return Path(self.sc.a.rec_root) / name

    def _local_files(self, name):
        d = self._local_dir(name)
        if not d.is_dir():
            return None
        return {p.name: p.stat().st_size for p in d.iterdir() if p.is_file() and not p.name.startswith(".")}

    @staticmethod
    def _same(ph, lf):
        if not lf:
            return False
        if ph.get("files"):
            return all(lf.get(k) == v for k, v in ph["files"].items())
        pb = ph["size_kb"] * 1024
        return abs(sum(lf.values()) - pb) <= 65536 + 0.02 * pb

    # ---------------- 가져오기 ----------------
    def _pull_next(self):
        sc = self.sc
        if self.pulling:
            return
        phone = {r["name"]: r for r in (self.phone or [])}
        cur = self._recording_now()
        job = None
        while self.pull_q:
            q = self.pull_q[0]
            ph = phone.get(q["name"])
            if ph is None:
                self.pull_q.pop(0)
                self.pull_failed[q["name"]] = "폰에 없음"
                continue
            if cur and (cur == q["name"] or (cur == "*" and not ph["complete"])):
                return                                      # 녹화 중 — 끝나면 다시
            job = self.pull_q.pop(0)
            break
        if job is None:
            if not self.cfg["pull"] or sc.mode in PHONE_MODES:
                return                                      # 실시간·녹화 중에는 자동 가져오기 쉼
            for name, ph in sorted(phone.items(), reverse=True):
                if not ph["complete"] or name in self.pull_failed or name == cur:
                    continue
                lf = self._local_files(name)
                if lf is not None and self._same(ph, lf):
                    continue
                d = self._local_dir(name)
                if lf is not None and any(d.glob("recon_*")):
                    continue                                # PC 사본이 다르지만 복원본이 있음 → 표시만 (mismatch)
                if self._upload_hold(name):
                    continue                                # 폰이 HTTPS 로 올리는 중 (upload_server.py)
                job = dict(name=name, replay=False)
                break
        if job is None:
            return
        name = job["name"]
        ph = phone[name]
        prog = dict(name=name, state="가져오는 중", done=0, total=ph["size_kb"] * 1024, t0=time.time())
        self.pulling = prog
        sc.pull_progress = prog

        def cb(done, total):
            prog.update(done=done, total=total or prog["total"])
        try:
            self.log(f"가져오기 시작: {name} ({ph['size_kb'] / 1024:.1f} MB)")
            path = self._pull_safe(name, cb)
            prog.update(state="완료", path=str(path), done=prog["total"])
            if not self._same(ph, self._local_files(name)):
                self.log(f"주의: 가져온 {name} 의 파일 크기가 폰과 다름 (녹화가 바뀌었나?)")
            if job.get("replay"):
                try:
                    sc.start_replay(str(path), job.get("every_frame", False), job.get("iters"))
                except Exception as e:
                    self.failed[name] = repr(e)
        except Exception as e:
            prog.update(state="오류", error=repr(e))
            self.pull_failed[name] = repr(e)
            self.log(f"가져오기 실패 {name}: {e!r}")
        finally:
            self.pulling = None
        self.evt.set()                                      # 다음 것 바로

    def _pull_safe(self, name, cb):
        root = Path(self.sc.a.rec_root)
        inc = root / ".incoming"
        inc.mkdir(parents=True, exist_ok=True)
        new = self.sc.link.pull(name, inc, progress=cb)    # inc/name (phone_link: .part 에 받은 뒤 바꿈)
        final = root / name
        if final.exists():
            for sub in final.iterdir():                     # 옛 복원본은 새 폴더로 옮겨 보존 (thumbs 는 새로 만듦)
                if sub.is_dir() and sub.name.startswith("recon_"):
                    shutil.move(str(sub), str(new / sub.name))
            shutil.rmtree(final)
        new.rename(final)
        return final

    # ---------------- 복원 대기열 ----------------
    def _finish_job(self):
        j = self.job
        if j is None or j["rp"].thread.is_alive():
            return
        self.job = None
        P = j["rp"].progress
        name, pair = j["name"], j.get("pair")
        key = self._key(name, pair)
        tag = f"{name}" + (f" (렌즈 {pair})" if pair else "")
        lo = self._local_dir(name)
        more = pair and any(not replay_view.best_recon(lo, p) and self._key(name, p) not in self.failed
                            and self._key(name, p) not in self.held and p != pair for p in replay_view.rec_pairs(lo))
        if name in self.recon_first and not more:
            self.recon_first.remove(name)
        if P.get("state") == "완료" and P.get("timeline"):
            self.log(f"복원 끝 → 리플레이 가능: {tag} ({Path(P.get('out', '')).name})")
        elif P.get("state") == "정지됨":
            if key in self.held:
                self.log(f"복원 멈춤 (사용자): {tag} — [다시 시도] 로 다시")
            else:
                self.log(f"복원 쉼: {tag} — 스캔이 끝나면 처음부터 다시")
        else:
            self.failed[key] = P.get("error") or ("맵이 비었음 — 위치 추적이 시작되지 않음 (녹화 때 폰을 움직이지 않았거나 처음에 "
                                                   "가만히 두지 않음)" if not P.get("keyframes") else "맵이 비어 결과 없음")
            self.log(f"복원 실패 {tag}: {self.failed[key]}")
            if not P.get("error"):
                self._save_failed(key, self.failed[key])    # 맵 비었음: 서버를 다시 켜도 다시 시도 안 함
        self.evt.set()

    def _recon_next(self):
        sc = self.sc
        rp = sc.replayer                                    # 끝난 복원도 sc.replayer 에 남아 있음 → 살아 있는지로 판단
        if self.job is not None or sc.mode != "idle" or (rp is not None and rp.thread.is_alive()):
            return
        cands = []
        for r in phone_link.list_local(sc.a.rec_root):
            n = r["name"]
            if r.get("complete") is False or not r.get("has_video"):
                continue
            if not (self.cfg["recon"] or n in self.recon_first):
                continue
            for pr in replay_view.rec_pairs(r["path"]):          # 2렌즈 녹화 = [None], 3렌즈 녹화 = ["2+6", "2+5"]
                k = self._key(n, pr)
                if k in self.failed or k in self.held or n in self.held:
                    continue
                if replay_view.best_recon(r["path"], pr):
                    continue
                cands.append((n, pr))
                break                                           # 쌍은 순서대로 하나씩 (첫 쌍의 포즈를 다음 쌍이 다시 씀)
        if not cands:
            return
        cands.sort(key=lambda c: c[0] not in self.recon_first)  # 웹 [다시 시도] 먼저, 그다음 최근 녹화부터 (list_local 순서)
        name, pair = cands[0]
        tag = f"{name}" + (f" (렌즈 {pair})" if pair else "")
        try:
            self.log(f"자동 복원 시작: {tag}")
            sc.start_replay(str(self._local_dir(name)), False, None, auto=True, pair=pair)
            rp = sc.replayer
            if rp is not None:
                self.job = dict(name=name, rp=rp, t0=time.time(), pair=pair)
        except Exception as e:
            self.failed[self._key(name, pair)] = repr(e)
            self.log(f"자동 복원을 시작하지 못함 {tag}: {e!r}")
