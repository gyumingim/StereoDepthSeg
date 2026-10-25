"""Automatic lens-calibration capture (calcap, 2026-10-25) - PC side. Run with WINDOWS python (adb.exe, Tailscale):

  python \\\\wsl.localhost\\Ubuntu-22.04\\home\\system\\scanner\\StereoDepthSeg-calcap\\scan3d\\tools\\calcap\\calcap_server.py
  -> open http://100.92.214.97:8789 on the laptop (full screen) and press "시작".

What it does (README.md next to this file has the user procedure):
  1. phone lock (mkdir, owner-checked release), thermal start gate (SKIN < 36, AP < 50 via adb dumpsys),
  2. starts the app (com.scan3d.arlat, mode calcap, 10 fps, VIO-identical 1080 YUV stream, focus 0.5, 2 ms, auto ISO once),
  3. asks the phone for a 960x540 gray preview ~10x/s over TCP (phone Tailscale IP, port 8811), finds the ChArUco board
     (laptop 5x3, else 6x4), computes distance / tilt / turn / lean / image position (slots.py),
  4. when the pose fills a still-missing slot and is steady (~0.4 s), tells the phone to save the next N (4) full-res
     1920x1080 raw Y frames (PGM, lossless) -> adb pull into <out>/calcap_<time>/<phone session>/ -> re-check on the
     stills (slot still filled, board found in every frame) -> the slot counts.
  5. Feedback: phone screen = live picture with the yellow TARGET outline + green detected corners + big text;
     laptop page = thin status lines above / below the board + beep per capture, panel with slot table on demand.
  6. Stops (camera off, verified with dumpsys media.camera) on: "정지" button, all slots done, phone auto-stop (idle 5 min,
     SKIN >= 40 / AP >= 50 / battery 42 C, 20 min max, PC silent 20 s), PC-side SKIN >= 40 or AP >= 50.
Then run calcap_calib.py on the set (README).
"""
import argparse
import base64
import datetime
import http.server
import json
import os
import queue
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import traceback

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import slots as SL  # noqa: E402

S3D = os.path.abspath(os.path.join(HERE, "..", ".."))
ap = argparse.ArgumentParser()
ap.add_argument("--host", default="100.92.214.97")
ap.add_argument("--port", type=int, default=8789)
ap.add_argument("--adb", default="C:/Users/a3162/platform-tools/adb.exe")
ap.add_argument("--serial", default="100.79.156.20:5555")
ap.add_argument("--phone-ip", default="100.79.156.20")
ap.add_argument("--phone-port", type=int, default=8811)
ap.add_argument("--out", default=r"\\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg\scan3d\recordings")
ap.add_argument("--set", default="", help="continue an existing set folder (its captures.json)")
ap.add_argument("--lens", default=os.path.join(S3D, "config", "lens", "std_1920x1080_f0.50_cal20261024d.json"),
                help="lens used ONLY for live pose guidance / slot checks (not the result)")
ap.add_argument("--charuco-dir", default=os.path.join(S3D, "config", "charuco"))
ap.add_argument("--frames", type=int, default=4, help="stills per slot (3-5)")
ap.add_argument("--fps", type=int, default=10)
ap.add_argument("--idle-s", type=int, default=300)
ap.add_argument("--max-s", type=int, default=1200)
ap.add_argument("--start-skin", type=float, default=36.0)
ap.add_argument("--stop-skin", type=float, default=40.0)
ap.add_argument("--stop-ap", type=float, default=50.0)
ap.add_argument("--lock-dir", default=r"C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----"
                                      r"\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\team\phone_lock.d",
                help="team phone lock (mkdir); '' = no lock")
ap.add_argument("--fake", default="", help="host:port of fake_phone.py (PC test without the phone; no adb)")
ap.add_argument("--fake-dir", default="", help="fake phone's files dir (pull = copy)")
ap.add_argument("--autostart", action="store_true", help="start a session right away (tests)")
ap.add_argument("--exit-after", action="store_true", help="exit the server when the session ends (tests)")
A = ap.parse_args()
A.frames = max(3, min(5, A.frames))

APP = "com.scan3d.arlat"
REMOTE_ROOT = "/sdcard/Android/data/com.scan3d.arlat/files/calcap"
K_FULL, DIST = SL.load_lens(A.lens)
BOARDS = SL.load_boards(A.charuco_dir)
LOCK_OWNER = "calcap-server (lens capture)"

state = dict(running=False, phase="대기", msg="", set_dir="", beep=0, beep_msg="", match="", target="", hint="",
             pose=None, n_ok=0, n_total=len(SL.SLOTS), phone_stat=None, log=[], session=None, captured_frames=0,
             last_capture="", stop_reason="", camera_off_check="")
slot_state = {s["id"]: "missing" for s in SL.SLOTS}       # missing | pending | ok
captures = []                                             # verified + rejected capture records (captures.json)
latest_prev = {"jpg": None, "t": 0}
stop_ev = threading.Event()
lock = threading.RLock()
pullq = queue.Queue()


def log(s):
    line = time.strftime("%H:%M:%S ") + s
    print(line, flush=True)
    with lock:
        state["log"] = (state["log"] + [line])[-60:]
    if state.get("set_dir"):
        try:
            with open(os.path.join(state["set_dir"], "server_log.txt"), "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception:
            pass


# ---------------------------------------------------------------- adb helpers
def adb(*a, timeout=30, binary=False):
    if A.fake:
        return b"" if binary else ""
    env = dict(os.environ, MSYS_NO_PATHCONV="1")
    r = subprocess.run([A.adb, "-s", A.serial, *a], capture_output=True, timeout=timeout, env=env)
    out = r.stdout if binary else r.stdout.decode("utf-8", "replace") + r.stderr.decode("utf-8", "replace")
    return out


def thermal():
    """SKIN / AP from the 'Current temperatures from HAL' section (not the cached first lines)."""
    if A.fake:
        return dict(SKIN=30.0, AP=33.0)
    out = adb("shell", "dumpsys thermalservice", timeout=20)
    t, inside = {}, False
    for line in out.splitlines():
        if "Current temperatures from HAL" in line:
            inside = True
            continue
        if inside:
            if line and not line[0].isspace():
                break
            if "mValue=" in line and "mName=" in line:
                try:
                    v = float(line.split("mValue=")[1].split(",")[0]); n = line.split("mName=")[1].split(",")[0].strip("} ")
                    t.setdefault(n, v)
                except ValueError:
                    pass
    return t


def camera_clients():
    if A.fake:
        return "[] (fake)"
    out = adb("shell", "dumpsys media.camera", timeout=30)
    lines = out.splitlines()
    for i, l in enumerate(lines):
        if "Active Camera Clients" in l:
            return " ".join(x.strip() for x in lines[i + 1:i + 3])
    return "?"


# ---------------------------------------------------------------- phone lock (team rule: mkdir, owner-checked release)
def lock_take():
    if not A.lock_dir or A.fake:
        return True, ""
    try:
        os.mkdir(A.lock_dir)
    except FileExistsError:
        try:
            who = open(os.path.join(A.lock_dir, "owner.txt"), encoding="utf-8", errors="replace").read().strip()
        except Exception:
            who = "?"
        return False, who
    with open(os.path.join(A.lock_dir, "owner.txt"), "w", encoding="utf-8") as f:
        f.write(f"{LOCK_OWNER} pid {os.getpid()} since {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    return True, ""


def lock_release():
    if not A.lock_dir or A.fake:
        return
    p = os.path.join(A.lock_dir, "owner.txt")
    try:
        who = open(p, encoding="utf-8").read()
    except Exception:
        return
    if LOCK_OWNER in who and f"pid {os.getpid()} " in who:
        os.remove(p)
        os.rmdir(A.lock_dir)
        log("폰 잠금 해제")
    else:
        log("잠금 주인이 내가 아님 - 그대로 둠: " + who.strip())


# ---------------------------------------------------------------- set (folder of captures)
def new_set():
    d = os.path.join(A.out, "calcap_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(d, exist_ok=True)
    with lock:
        state["set_dir"] = d
        captures.clear()
        for k in slot_state:
            slot_state[k] = "missing"
    json.dump(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), slots=SL.SLOTS, lens_for_guidance=A.lens,
                   boards=[b.name for b in BOARDS], frames_per_slot=A.frames, fps=A.fps,
                   note="stills = raw Y of the VIO stream (1920x1080 YUV, TEMPLATE_RECORD, focus 0.5, 2 ms, ISO auto once)"),
              open(os.path.join(d, "set.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    save_captures()
    log("새 세트: " + d)


def load_set(d):
    with lock:
        state["set_dir"] = d
        captures.clear()
        p = os.path.join(d, "captures.json")
        if os.path.exists(p):
            captures.extend(json.load(open(p, encoding="utf-8"))["captures"])
        for k in slot_state:
            slot_state[k] = "missing"
        for c in captures:
            if c.get("ok") and c["slot"] in slot_state:
                slot_state[c["slot"]] = "ok"
    log(f"세트 이어서: {d} (채운 칸 {sum(v == 'ok' for v in slot_state.values())})")


def save_captures():
    d = state["set_dir"]
    with lock:
        state["n_ok"] = sum(v == "ok" for v in slot_state.values())
        j = dict(updated=time.strftime("%Y-%m-%d %H:%M:%S"), n_ok=state["n_ok"], n_total=len(SL.SLOTS),
                 missing=[k for k, v in slot_state.items() if v != "ok"], captures=captures)
    tmp = os.path.join(d, "captures.json.tmp")
    json.dump(j, open(tmp, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    os.replace(tmp, os.path.join(d, "captures.json"))


# ---------------------------------------------------------------- phone link
class Phone:
    def __init__(self, host, port):
        self.s = socket.create_connection((host, port), timeout=5)
        self.s.settimeout(8)
        self.f = self.s.makefile("rb")

    def cmd(self, line):
        self.s.sendall((line.replace("\n", " ") + "\n").encode("utf-8"))
        r = self.f.readline()
        if not r:
            raise ConnectionError("phone closed the link")
        return r.decode("utf-8", "replace").strip()

    def prev(self):
        self.s.sendall(b"PREV\n")
        h = self.f.readline().decode().strip()
        if not h:
            raise ConnectionError("phone closed the link")
        if not h.startswith("JPG "):
            return None, 0
        _, n, ts = h.split()
        data = self.f.read(int(n))
        return data, int(ts)

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def connect_phone(t_max=25):
    host, port = (A.fake.split(":")[0], int(A.fake.split(":")[1])) if A.fake else (A.phone_ip, A.phone_port)
    t0 = time.time()
    while time.time() - t0 < t_max and not stop_ev.is_set():
        try:
            return Phone(host, port)
        except OSError:
            time.sleep(1.0)
    if not A.fake:          # fallback: adb forward through the adb link
        adb("forward", f"tcp:{A.phone_port}", f"tcp:{A.phone_port}")
        try:
            return Phone("127.0.0.1", A.phone_port)
        except OSError:
            pass
    return None


# ---------------------------------------------------------------- drawing
def draw(gray, scale, pose, corners, target_slot, board, matched, steady_n):
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if target_slot is not None:
        ol = SL.target_outline(target_slot, board or BOARDS[0], K_FULL, DIST) * scale
        cv2.polylines(img, [ol.astype(np.int32).reshape(-1, 1, 2)], True, (0, 220, 255), max(2, int(4 * scale)))
    if corners is not None and len(corners):
        col = (0, 200, 0) if matched else (0, 140, 255)
        for q in corners:
            cv2.circle(img, (int(q[0]), int(q[1])), max(3, int(7 * scale)), col, -1)
        hull = cv2.convexHull(corners.astype(np.float32)).astype(np.int32)
        cv2.polylines(img, [hull], True, col, max(1, int(3 * scale)))
    m = int(SL.EDGE_PX * scale)
    h, w = img.shape[:2]
    cv2.rectangle(img, (m, m), (w - m, h - m), (90, 90, 90), 1)
    if matched:
        cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 200, 0), max(4, int(12 * scale)))
        cv2.rectangle(img, (0, h - 14), (int(w * min(1.0, steady_n / 4.0)), h - 1), (0, 200, 0), -1)
    return img


# ---------------------------------------------------------------- pull + re-check on the stills
def pull_worker():
    while True:
        job = pullq.get()
        if job is None:
            return
        try:
            verify_capture(job)
        except Exception as e:
            log(f"확인 오류 {job.get('slot')}: {e}")
            traceback.print_exc()
            with lock:
                if slot_state.get(job["slot"]) == "pending":
                    slot_state[job["slot"]] = "missing"
        finally:
            pullq.task_done()


def pull_files(remote_dir, names, local_dir):
    os.makedirs(local_dir, exist_ok=True)
    if A.fake:
        for n in names:
            shutil.copy(os.path.join(A.fake_dir, os.path.basename(remote_dir), n), os.path.join(local_dir, n))
        return
    tmp = tempfile.mkdtemp(prefix="calcap_")
    try:
        for n in names:
            out = adb("pull", f"{remote_dir}/{n}", os.path.join(tmp, n), timeout=120)
            if not os.path.exists(os.path.join(tmp, n)):
                raise IOError(f"adb pull {n}: {out.strip()[-200:]}")
            shutil.copy(os.path.join(tmp, n), os.path.join(local_dir, n))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def verify_capture(job):
    slot = SL.SLOT_BY_ID[job["slot"]]
    sess = os.path.basename(job["remote_dir"])
    local = os.path.join(state["set_dir"], sess)
    t0 = time.time()
    pull_files(job["remote_dir"], job["files"], local)
    poses, reasons, cids = [], [], []
    for n in job["files"]:
        g = cv2.imread(os.path.join(local, n), cv2.IMREAD_UNCHANGED)
        if g is None or g.shape != (1080, 1920):
            reasons.append(f"{n}: 읽기 실패/크기 {None if g is None else g.shape}")
            poses.append(None); cids.append(None)
            continue
        p, c, i, b = SL.detect_pose(BOARDS, g, K_FULL, DIST, 1.0)
        poses.append(p)
        cids.append((c, i) if p is not None else None)
        if p is None:
            reasons.append(f"{n}: 보드 못 찾음")
    good = [p for p in poses if p is not None]
    ok = len(good) >= len(job["files"]) - 1 and len(good) >= 3 and SL.fills(slot, good[0], tol=1.0)
    if good and not SL.fills(slot, good[0], tol=1.0):
        reasons.append("사진에서 다시 재보니 칸 조건 밖")
    # corner motion between the first and last good still (the phone / board must have been still)
    motion = None
    cc = [x for x in cids if x is not None]
    if len(cc) >= 2:
        (c1, i1), (c2, i2) = cc[0], cc[-1]
        common = np.intersect1d(i1, i2)
        if len(common):
            a = c1[[list(i1).index(k) for k in common]]; b2 = c2[[list(i2).index(k) for k in common]]
            motion = float(np.max(np.linalg.norm(a - b2, axis=1)))
    if motion is not None and motion > 3.0:        # stills must come from one still pose (verifier finding 2026-10-25)
        ok = False
        reasons.append(f"사진 사이 움직임 {motion:.1f} px > 3 px")
    rec = dict(cap=job["cap"], slot=slot["id"], label=slot["label"], phone_dir=job["remote_dir"], dir=sess,
               files=[f"{sess}/{n}" for n in job["files"]], preview_pose=job["pose"], still_poses=poses, ok=bool(ok),
               why=reasons, motion_px=motion, t=time.strftime("%Y-%m-%d %H:%M:%S"), pull_s=round(time.time() - t0, 1))
    with lock:
        captures.append(rec)
        slot_state[slot["id"]] = "ok" if ok else "missing"
        state["captured_frames"] += len(good) if ok else 0
    save_captures()
    if ok:
        log(f"확인 OK {slot['id']} ({len(good)}장, 움직임 {motion if motion is None else round(motion, 2)} px, 받기 {rec['pull_s']} s)")
    else:
        log(f"확인 실패 {slot['id']}: {'; '.join(reasons)} -> 다시 찍어야 함")


# ---------------------------------------------------------------- session
def preflight():
    if A.fake:
        return True, "fake"
    t = thermal()
    skin, apc = t.get("SKIN"), t.get("AP")
    if skin is None:
        return False, "폰 온도(SKIN)를 못 읽음 (adb 연결 확인)"
    if skin >= A.start_skin or (apc is not None and apc >= A.stop_ap):
        return False, f"폰이 따뜻함 (SKIN {skin} °C, AP {apc} °C) - 식힌 뒤 다시 (시작 기준 SKIN < {A.start_skin})"
    return True, f"SKIN {skin} °C, AP {apc} °C"


def start_app():
    adb("shell", "am", "force-stop", APP)
    adb("shell", "input", "keyevent", "KEYCODE_WAKEUP")              # screen on first: a lock-screen resume/pause/resume
    adb("shell", "pm", "grant", APP, "android.permission.DUMP")     # app reads SKIN/AP itself (overheat stop)
    out = adb("shell", "am", "start", "-n", APP + "/.MainActivity", "--es", "mode", "calcap", "--ei", "fps", str(A.fps),
              "--ei", "port", str(A.phone_port), "--ei", "idle_s", str(A.idle_s), "--ei", "max_s", str(A.max_s),
              "--ef", "max_skin_c", str(A.stop_skin), "--ef", "max_ap_c", str(A.stop_ap))
    return out


def stop_app_and_check():
    adb("shell", "am", "force-stop", APP)
    time.sleep(1.0)
    cc = camera_clients()
    with lock:
        state["camera_off_check"] = f"{time.strftime('%H:%M:%S')} Active Camera Clients: {cc}"
    log("카메라 확인: " + cc)


def session():
    ok, who = lock_take()
    if not ok:
        with lock:
            state.update(running=False, phase="대기", msg=f"폰을 다른 작업이 쓰는 중 (잠금: {who}) - 잠시 후 다시")
        log("잠금 사용 중: " + who)
        return
    ph = None
    try:
        with lock:
            state.update(phase="온도 확인", msg="", stop_reason="", camera_off_check="")
        ok, why = preflight()
        log("시작 전 확인: " + why)
        if not ok:
            with lock:
                state.update(phase="대기", msg=why)
            return
        if not A.fake:
            log("앱 시작: " + start_app().strip()[-160:])
        with lock:
            state["phase"] = "폰 연결 중"
        for attempt in range(3):      # the app may restart once right after launch (lock screen) -> reconnect
            time.sleep(0 if A.fake else 3.0)
            ph = connect_phone()
            if ph is None:
                with lock:
                    state.update(phase="대기", msg="폰에 연결 못 함 (앱이 안 켜졌거나 네트워크)")
                return
            log("폰 연결됨")
            t_conn = time.time()
            try:
                run_loop(ph)
                break
            except ConnectionError as e:
                ph.close(); ph = None
                if time.time() - t_conn > 20 or stop_ev.is_set():
                    raise
                log(f"연결 끊김 ({e}) - 다시 연결")
    except Exception as e:
        log(f"세션 오류: {e}")
        traceback.print_exc()
        with lock:
            state["stop_reason"] = state["stop_reason"] or f"오류 {e}"
    finally:
        try:
            if ph is not None:
                try:
                    st = ph.cmd("STAT")
                    with lock:
                        state["phone_stat"] = json.loads(st[5:]) if st.startswith("STAT ") else state["phone_stat"]
                except Exception:
                    pass
                try:
                    ph.cmd("MSG 333333 끝 - 카메라 끔")
                    ph.cmd("STOP")
                except Exception:
                    pass
                ph.close()
            time.sleep(1.5)
            stop_app_and_check()
            pullq.join()
            fetch_session_files()
        finally:
            lock_release()
            with lock:
                state.update(running=False, phase="대기")
                state["beep"] += 1
                state["beep_msg"] = f"끝: {state['stop_reason'] or '정지'} · 채운 칸 {state['n_ok']}/{state['n_total']}"
            log(state["beep_msg"])
            if A.exit_after:
                threading.Timer(1.0, lambda: os._exit(0)).start()


def fetch_session_files():
    s = state.get("session")
    if not s:
        return
    local = os.path.join(state["set_dir"], os.path.basename(s))
    for n in ("frames.jsonl", "meta.json", "temps.csv"):
        try:
            pull_files(s, [n], local)
        except Exception as e:
            log(f"{n} 받기 실패: {e}")
    log("폰 기록 받음 -> " + local)
    try:
        m = json.load(open(os.path.join(local, "meta.json"), encoding="utf-8"))
        log(f"폰 쪽 정지 이유: {m.get('stop_reason')} · 측정 fps {m.get('stat_end', {}).get('fps_measured')}")
        with lock:
            if not state["stop_reason"] or state["stop_reason"].startswith("오류"):
                state["stop_reason"] = "폰: " + str(m.get("stop_reason"))
    except Exception as e:
        log(f"meta.json 읽기 실패: {e}")


def run_loop(ph):
    hist = []                 # (t, pose, match id)
    last_stat = last_th = 0.0
    last_msg = ""
    k = 0
    blue_until = 0.0
    t_start = time.time()
    while not stop_ev.is_set():
        k += 1
        jpg, ts = ph.prev()
        if jpg is None:
            time.sleep(0.05)
            continue
        gray = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
        scale = gray.shape[1] / 1920.0
        p, corners, ids, board = SL.detect_pose(BOARDS, gray, K_FULL, DIST, scale)
        with lock:
            missing = [s for s in SL.SLOTS if slot_state[s["id"]] == "missing"]
        match = next((s for s in missing if SL.fills(s, p)), None)
        now = time.time()
        hist.append((now, p, match["id"] if match else None))
        # 10-25 fix: the preview arrives at only ~3-5 Hz, so "3 frames inside 0.45 s" never happened (stuck at 2/4).
        # Now: same slot continuously for >= HOLD_S (time based, >= 2 frames), small motion.
        HOLD_S = 0.4
        hist = [h for h in hist if now - h[0] <= 1.0]
        run = []
        for h in reversed(hist):                       # newest run of frames matching this slot
            if match and h[2] == match["id"] and h[1] is not None:
                run.append(h)
            else:
                break
        run.reverse()
        hold = (now - run[0][0]) if len(run) >= 2 else 0.0
        steady = False
        if match and len(run) >= 2 and hold >= HOLD_S:
            cx = [h[1]["cx"] for h in run]; cy = [h[1]["cy"] for h in run]; tl = [h[1]["tilt"] for h in run]
            steady = max(np.ptp(cx), np.ptp(cy)) <= 20 and np.ptp(tl) <= 4.0
        target = match or SL.nearest_missing(missing, p)
        hint = "모든 칸 완료!" if target is None else ("그대로 멈추기…" if match else SL.hint(target, p))
        n_same = min(4, int(4 * hold / HOLD_S)) if match else 0     # 0..4 progress for the bar / "n/4" text
        img = draw(gray, scale, p, corners, target, board, match is not None, n_same)
        ok_, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        latest_prev.update(jpg=enc.tobytes(), t=now)
        with lock:
            state.update(pose=None if p is None else {k2: (round(v, 2) if isinstance(v, float) else v) for k2, v in p.items()
                                                      if k2 in ("board", "n", "tilt", "turn", "lean", "dist", "pos", "edge")},
                         match=match["id"] if match else "", target=target["label"] if target else "", hint=hint,
                         phase="촬영 중")
        if k % 2 == 0:       # guidance picture to the phone ~5x/s (small)
            small = cv2.resize(img, (480, 270), interpolation=cv2.INTER_AREA)
            ok_, e2 = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
            ph.cmd("IMG " + base64.b64encode(e2.tobytes()).decode())
        n_left = sum(1 for v in slot_state.values() if v != "ok")
        color = "16a34a" if match else ("c2410c" if p is not None else "444444")
        msg = (f"목표: {target['label']}\\n{hint}\\n남은 칸 {n_left}/{len(SL.SLOTS)}" if target else "모든 칸 완료! 끝내는 중")
        if p is not None:
            msg += f"\\n거리 {p['dist']:.2f} m · 기울기 {p['tilt']:.0f}°"
        if match and not steady:     # user 10-25: show the hold progress while green
            msg = f"초록: 그대로 멈추기 {min(n_same, 4)}/4\\n" + msg
        if (color, msg) != last_msg and now >= blue_until:   # keep the blue "captured" screen visible for a moment
            ph.cmd(f"MSG {color} {msg}")
            last_msg = (color, msg)
        if steady:
            info = dict(slot=match["id"], pose={k2: p[k2] for k2 in ("board", "n", "tilt", "turn", "lean", "dist", "pos", "edge", "bbox")},
                        t_pc=time.strftime("%Y-%m-%d %H:%M:%S"), preview_ts=ts)
            ph.cmd(f"MSG 2563eb 파랑: 촬영 중! {match['label']}\\n움직이지 말기…")
            last_msg = ""
            with lock:
                slot_state[match["id"]] = "pending"
            r = ph.cmd(f"CAP {A.frames} {match['id']} {json.dumps(info, ensure_ascii=True)}")
            if r.startswith("DONE "):
                d = json.loads(r[5:])
                with lock:
                    state["session"] = d["dir"]
                    state["beep"] += 1
                    state["beep_msg"] = "찰칵 " + match["label"]
                    state["last_capture"] = match["label"]
                log(f"캡처 {match['id']}: {len(d['files'])}장 (건너뜀 {d['skipped']})")
                ph.cmd(f"MSG 2563eb 파랑: 촬영 완료 ✓ {match['label']} ({len(d['files'])}장, 1920x1080)\\n다음 칸으로 옮기세요")
                blue_until = time.time() + 1.5
                pullq.put(dict(cap=d["cap"], slot=match["id"], remote_dir=d["dir"], files=d["files"], pose=info["pose"]))
            else:
                log(f"캡처 실패 {match['id']}: {r[:200]}")
                with lock:
                    slot_state[match["id"]] = "missing"
            hist = []
        if now - last_stat > 10:
            st = ph.cmd("STAT")
            if st.startswith("STAT "):
                with lock:
                    state["phone_stat"] = json.loads(st[5:])
                    state["session"] = state["session"] or state["phone_stat"].get("dir")
            last_stat = now
        if now - last_th > 20:
            t = thermal()
            last_th = now
            skin, apc = t.get("SKIN"), t.get("AP")
            with lock:
                state["pc_thermal"] = t
            if (skin is not None and skin >= A.stop_skin) or (apc is not None and apc >= A.stop_ap):
                with lock:
                    state["stop_reason"] = f"폰 온도 (SKIN {skin}, AP {apc}) - 식힌 뒤 이어서"
                break
        with lock:
            all_done = all(v == "ok" for v in slot_state.values())
            any_pending = any(v == "pending" for v in slot_state.values())
        if all_done:
            state["stop_reason"] = "모든 칸 완료"
            break
        if now - t_start > A.max_s + 30:
            state["stop_reason"] = "최대 시간"
            break
    if stop_ev.is_set() and not state["stop_reason"]:
        state["stop_reason"] = "정지 버튼"


# ---------------------------------------------------------------- web
class H(http.server.SimpleHTTPRequestHandler):
    def __init__(s, *a, **k):
        super().__init__(*a, directory=HERE, **k)

    def _j(s, o, code=200):
        b = json.dumps(o, ensure_ascii=False, default=str).encode("utf-8")
        s.send_response(code); s.send_header("Content-Type", "application/json; charset=utf-8")
        s.send_header("Content-Length", str(len(b))); s.send_header("Cache-Control", "no-store"); s.end_headers(); s.wfile.write(b)

    def do_GET(s):
        if s.path == "/" or s.path.startswith("/index"):
            s.path = "/calcap.html"
        if s.path.startswith("/api/state"):
            with lock:
                o = dict(state)
                o["slots"] = [dict(id=x["id"], label=x["label"], status=slot_state[x["id"]]) for x in SL.SLOTS]
            return s._j(o)
        if s.path.startswith("/api/prev"):
            j = latest_prev["jpg"]
            if j is None or not state["running"]:
                return s._j({"ok": False}, 404)
            s.send_response(200); s.send_header("Content-Type", "image/jpeg"); s.send_header("Content-Length", str(len(j)))
            s.send_header("Cache-Control", "no-store"); s.end_headers(); s.wfile.write(j)
            return
        if s.path.startswith("/board/"):
            name = os.path.basename(s.path.split("?")[0])
            p = os.path.join(A.charuco_dir, name)
            if name.endswith(".png") and os.path.exists(p):
                b = open(p, "rb").read()
                s.send_response(200); s.send_header("Content-Type", "image/png"); s.send_header("Content-Length", str(len(b)))
                s.end_headers(); s.wfile.write(b)
                return
        return super().do_GET()

    def do_POST(s):
        if s.path.startswith("/api/start"):
            return s._j(start_session())
        if s.path.startswith("/api/stop"):
            stop_ev.set()
            return s._j({"ok": True})
        if s.path.startswith("/api/newset"):
            if state["running"]:
                return s._j({"ok": False, "msg": "촬영 중에는 안 됨"})
            new_set()
            return s._j({"ok": True, "set_dir": state["set_dir"]})
        s.send_response(404); s.end_headers()

    def log_message(s, *a):
        pass


def start_session():
    with lock:
        if state["running"]:
            return {"ok": False, "msg": "이미 촬영 중"}
        state.update(running=True, phase="시작 중", msg="", session=None)
    stop_ev.clear()
    threading.Thread(target=session, daemon=True).start()
    return {"ok": True}


if __name__ == "__main__":
    if A.set:
        load_set(A.set)
    else:
        new_set()
    threading.Thread(target=pull_worker, daemon=True).start()
    srv = http.server.ThreadingHTTPServer((A.host, A.port), H)
    log(f"calcap 서버 http://{A.host}:{A.port}  (세트 {state['set_dir']})")
    if A.autostart:
        start_session()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        stop_ev.set()
