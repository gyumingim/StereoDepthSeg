#!/usr/bin/env python3
"""회귀 검증: 참조 녹화를 서버 API 로 하나씩 복원하고 결과 숫자를 표로 비교한다.

  python3 scan3d/tools/regress.py            # 참조 녹화 전부 (몇 분 걸림)
  python3 scan3d/tools/regress.py --only 175446 --only 165645
  python3 scan3d/tools/regress.py --dry-run  # 서버 상태 확인과 계획만 (복원 안 함)
  python3 scan3d/tools/regress.py --show     # 복원 없이 마지막 결과 표만 다시 보기

하는 일
  1) GET /api/state → session.mode 가 idle 이 아니면 (실시간 스캔·복원 중) 거절하고 끝.
  2) 녹화마다 POST /api/cmd {"cmd":"replay","name":..,"pair":..} → /api/state 를 1 s 마다 폴링해
     복원이 끝나면 결과 폴더의 summary.json 을 읽는다.
  3) 키프레임·복셀·VIO 리셋·재위치·평면 잔차 등 있는 값만 표로 출력, runs/regress_<시각>.json 저장.
  4) 바로 전 regress_*.json 과 비교해 나빠진 항목에 'WORSE' 표시. 하나라도 있으면 종료 코드 1.

표준 라이브러리만 쓴다 (WSL python3, Windows python 둘 다 됨). 서버를 시작·재시작하지 않는다.
"""
import argparse
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
RUNS = SCAN3D / "runs"
DISTRO = "Ubuntu-22.04"

# 참조 녹화 (이름, 렌즈 쌍). 쌍 None = 녹화한 쌍 그대로. 용도는 CLAUDE.md '참조 녹화'.
CASES = [
    ("rec_20261016_143702", None),    # 640x480 2+5, 26 s, 시작 정지 → VIO 초기화
    ("rec_20261016_165645", "2+5"),   # 3렌즈 녹화, 기본 쌍 2+5 로 복원
    ("rec_20261016_175446", None),    # 1440x1080 2+5 14 Mbps (지금 기본 설정), 추적 끊김 1 회
    ("rec_20261016_162204", None),    # 1440x1080 2+6, 37 s 긴 녹화
]

# (표 이름, summary 안 경로, 좋은 방향 +1=클수록 좋음 / -1=작을수록 좋음 / 0=판단 안 함)
METRICS = [
    ("keyframes", ("replay", "keyframes"), 0),
    ("voxels", ("state", "map", "voxels"), +1),
    ("shown", ("state", "map", "shown"), +1),
    ("vio_resets", ("replay", "vio_auto_resets"), -1),
    ("best_seg", ("replay", "best_segment"), 0),
    ("pose_timeouts", ("replay", "pose_timeouts"), -1),
    ("plane_mm", ("compare", "plane_res_mm"), -1),
    ("plane_inlier", ("compare", "plane_inlier"), +1),
    ("valid_med", ("compare", "valid_med"), +1),
    ("elapsed_s", ("replay", "elapsed_s"), 0),
]
# 아직 모르는 새 숫자 (다른 작업이 추가하는 재위치·TSDF·ArUco 등): 이름에 이 낱말이 있으면 자동으로 표에 넣는다.
AUTO_WORDS = ("reloc", "tsdf", "aruco", "scale", "loop")
AUTO_DIRECTION = {"reloc_ok": +1, "reloc_success": +1, "reloc_fail": -1, "reloc_failed": -1}


def http_json(url, body=None, timeout=10.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method="POST" if body is not None else "GET",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def local_path(p):
    """서버가 준 WSL 경로 → 지금 OS 에서 열 수 있는 경로 (Windows 면 \\\\wsl.localhost\\...)."""
    p = str(p)
    if os.name == "nt" and p.startswith("/"):
        return Path("\\\\wsl.localhost\\" + DISTRO + p.replace("/", "\\"))
    return Path(p)


def dig(d, path):
    for k in path:
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d if isinstance(d, (int, float)) and not isinstance(d, bool) else None


def auto_metrics(summary):
    """summary 최상위·replay·compare 안의 숫자 중 AUTO_WORDS 가 들어간 것 (한 단계 아래 dict 까지)."""
    out = {}
    roots = [("", summary), ("replay.", summary.get("replay") or {}), ("compare.", summary.get("compare") or {})]
    for pre, d in roots:
        if not isinstance(d, dict):
            continue
        for k, v in d.items():
            if k in ("args", "state", "meta"):
                continue
            if isinstance(v, dict) and any(w in k.lower() for w in AUTO_WORDS):
                for k2, v2 in v.items():
                    if isinstance(v2, (int, float)) and not isinstance(v2, bool):
                        out[f"{pre}{k}.{k2}"] = v2
            elif isinstance(v, (int, float)) and not isinstance(v, bool) and any(w in k.lower() for w in AUTO_WORDS):
                out[f"{pre}{k}"] = v
    return out


def extract(summary):
    m = {}
    for name, path, _ in METRICS:
        v = dig(summary, path)
        if v is not None:
            m[name] = v
    m.update(auto_metrics(summary))
    return m


def direction(name):
    for n, _, d in METRICS:
        if n == name:
            return d
    return AUTO_DIRECTION.get(name.split(".")[-1], 0)


def worse(name, old, new, rel):
    """좋은 방향의 반대로 rel(비율) 넘게, 그리고 개수형은 1 이상 바뀌었으면 True."""
    d = direction(name)
    if d == 0 or old is None or new is None:
        return False
    diff = (new - old) * d                    # 음수 = 나빠짐
    if diff >= 0:
        return False
    if isinstance(old, int) and isinstance(new, int) and abs(old) < 10:
        return abs(new - old) >= 1
    return abs(diff) > rel * max(abs(old), 1e-9)


def case_key(name, pair):
    return f"{name}" + (f"@{pair}" if pair else "")


def git_rev():
    try:
        return subprocess.run(["git", "-C", str(SCAN3D), "rev-parse", "--short", "HEAD"], capture_output=True,
                              text=True, timeout=10).stdout.strip() or None
    except Exception:
        return None


def previous_result(exclude=None):
    files = sorted(RUNS.glob("regress_*.json"))
    files = [f for f in files if exclude is None or f.resolve() != Path(exclude).resolve()]
    return files[-1] if files else None


def run_case(url, name, pair, timeout_s, log):
    body = {"cmd": "replay", "name": name}
    if pair:
        body["pair"] = pair
    t0 = time.time()
    try:
        r = http_json(url + "/api/cmd", body, timeout=120)   # 깊이 모델 준비를 기다릴 수 있음
    except (urllib.error.URLError, OSError, ValueError) as e:
        return dict(ok=False, error=f"replay 요청 실패: {e}")
    if not r.get("ok"):
        return dict(ok=False, error=str(r.get("error") or r))
    out = r.get("out")
    log(f"  복원 시작 → {out}")
    seen_replay = False
    last_print = 0.0
    while True:
        if time.time() - t0 > timeout_s:
            return dict(ok=False, out=out, error=f"시간 초과 {timeout_s:.0f} s")
        time.sleep(1.0)
        try:
            st = http_json(url + "/api/state", timeout=10)
        except (urllib.error.URLError, OSError, ValueError) as e:
            log(f"  상태 읽기 실패 ({e}), 다시 시도")
            continue
        ses = st.get("session") or {}
        rp = ses.get("replay") or {}
        mode = ses.get("mode")
        if mode == "replay":
            seen_replay = True
            if time.time() - last_print > 10:
                last_print = time.time()
                log(f"  ... {rp.get('pos_s')} / {rp.get('total_s')} s  kf={rp.get('keyframes')}")
            continue
        if mode not in ("idle", None):
            return dict(ok=False, out=out, error=f"복원 도중 모드가 {mode} 로 바뀜 (누가 스캔을 시작함?)")
        if rp.get("out") != out and not seen_replay:
            continue                                   # 아직 복원 스레드가 상태를 안 바꿈
        if rp.get("error"):
            return dict(ok=False, out=out, error=str(rp.get("error")))
        break
    sp = local_path(out) / "summary.json"
    for _ in range(60):                                # 저장이 끝나기를 최대 30 s
        if sp.exists():
            try:
                return dict(ok=True, out=out, wall_s=round(time.time() - t0, 1),
                            summary=json.loads(sp.read_text(encoding="utf-8")))
            except ValueError:
                pass                                   # 쓰는 중
        time.sleep(0.5)
    return dict(ok=False, out=out, error="summary.json 없음 (복원 실패 또는 맵 비었음)")


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.3g}" if abs(v) < 1000 else f"{v:.0f}"
    return str(v)


def print_table(results, prev, rel):
    """결과 표. prev 가 있으면 '이전 → 지금' 과 WORSE 표시. 나빠진 항목 개수를 돌려줌."""
    n_worse = 0
    prev_cases = (prev or {}).get("cases", {})
    for key, res in results.items():
        print(f"\n== {key}  {'OK' if res.get('ok') else 'FAIL: ' + str(res.get('error'))}")
        if res.get("out"):
            print(f"   {res['out']}")
        cur = res.get("metrics") or {}
        old = (prev_cases.get(key) or {}).get("metrics") or {}
        if not res.get("ok") and prev_cases.get(key, {}).get("ok"):
            print("   WORSE: 이전에는 성공한 녹화")
            n_worse += 1
        names = list(dict.fromkeys(list(cur) + list(old)))
        if not names:
            continue
        w = max(len(n) for n in names)
        for n in names:
            a, b = old.get(n), cur.get(n)
            bad = worse(n, a, b, rel)
            n_worse += bad
            mark = "  WORSE" if bad else ""
            if prev:
                print(f"   {n:<{w}}  {fmt(a):>9} -> {fmt(b):<9}{mark}")
            else:
                print(f"   {n:<{w}}  {fmt(b)}")
    return n_worse


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=os.environ.get("SCAN3D_URL", "http://localhost:8795"))
    ap.add_argument("--only", action="append", default=[], help="녹화 이름 일부 (여러 번 가능)")
    ap.add_argument("--timeout", type=float, default=900, help="녹화 하나 최대 초 (기본 900)")
    ap.add_argument("--rel", type=float, default=0.10, help="나빠짐 판정 비율 (기본 0.10 = 10 %%)")
    ap.add_argument("--baseline", help="비교할 regress_*.json (기본: runs/ 의 가장 최근 것)")
    ap.add_argument("--dry-run", action="store_true", help="서버 상태와 계획만 출력")
    ap.add_argument("--show", action="store_true", help="복원 없이 가장 최근 결과를 그 전 결과와 비교해 출력")
    ap.add_argument("--no-save", action="store_true")
    a = ap.parse_args()
    url = a.url.rstrip("/")
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    def log(msg):
        print(msg, flush=True)

    if a.show:
        files = sorted(RUNS.glob("regress_*.json"))
        if not files:
            log("저장된 결과 없음 (runs/regress_*.json)")
            return 2
        cur = json.loads(files[-1].read_text(encoding="utf-8"))
        prev = json.loads(files[-2].read_text(encoding="utf-8")) if len(files) > 1 else None
        log(f"지금: {files[-1].name}" + (f"   이전: {files[-2].name}" if prev else ""))
        return 1 if print_table(cur["cases"], prev, a.rel) else 0

    cases = [(n, p) for n, p in CASES if not a.only or any(o in n for o in a.only)]
    if not cases:
        log("고른 녹화 없음")
        return 2
    try:
        st = http_json(url + "/api/state", timeout=5)
    except (urllib.error.URLError, OSError, ValueError) as e:
        log(f"서버 응답 없음 ({url}): {e}\n  → scan3d/tools/restart_server.ps1 (또는 .sh) 로 켜세요")
        return 2
    mode = (st.get("session") or {}).get("mode")
    if mode != "idle":
        log(f"거절: 서버가 지금 '{mode}' 중입니다 (실시간 스캔·복원 중에는 회귀 검증을 하지 않음). 끝난 뒤 다시.")
        return 2
    log(f"서버 {url} 대기 중 (idle). 녹화 {len(cases)} 개:")
    for n, p in cases:
        exists = (SCAN3D / "recordings" / n).is_dir()
        log(f"  {case_key(n, p):<28} {'있음' if exists else '없음 (pull 필요)'}")
    if a.dry_run:
        return 0

    results = {}
    for n, p in cases:
        key = case_key(n, p)
        log(f"\n[{len(results) + 1}/{len(cases)}] {key}")
        res = run_case(url, n, p, a.timeout, log)
        summ = res.pop("summary", None)
        if summ is not None:
            res["metrics"] = extract(summ)
        log("  " + ("완료" if res.get("ok") else f"실패: {res.get('error')}"))
        results[key] = res

    prev_file = Path(a.baseline) if a.baseline else previous_result()
    prev = json.loads(prev_file.read_text(encoding="utf-8")) if prev_file and prev_file.exists() else None
    if prev:
        log(f"\n비교 기준: {prev_file.name}")
    n_worse = print_table(results, prev, a.rel)
    if not a.no_save:
        RUNS.mkdir(parents=True, exist_ok=True)
        path = RUNS / time.strftime("regress_%Y%m%d-%H%M%S.json")
        path.write_text(json.dumps(dict(created=time.strftime("%Y-%m-%d %H:%M:%S"), git=git_rev(), url=url,
                                        baseline=prev_file.name if prev else None, cases=results),
                                   ensure_ascii=False, indent=1), encoding="utf-8")
        log(f"\n저장: {path}")
    n_fail = sum(1 for r in results.values() if not r.get("ok"))
    log(f"RESULT: {'FAIL' if (n_worse or n_fail) else 'PASS'}  (나빠짐 {n_worse}, 실패 {n_fail})")
    return 1 if (n_worse or n_fail) else 0


if __name__ == "__main__":
    sys.exit(main())
