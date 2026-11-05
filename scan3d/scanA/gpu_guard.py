"""scanA GPU guard (2026-11-05, after the WSL crash at 16:56: a 1920x1080 FFS run spilled past 16 GB VRAM into shared memory).

  import gpu_guard
  with gpu_guard.job("scan_offline " + out):   # one GPU job at a time (~/scanner/gpu.lock), torch memory cap, clean OOM exit
      ...

* cap: torch.cuda.set_per_process_memory_fraction(frac) right after the torch import, frac = 0.75 (env SCAN3D_GPU_FRAC,
  clamped to <= 0.85). Allocations above the cap raise OutOfMemoryError instead of spilling into WSL shared memory.
* OOM: caught -> message + OUT/FAILED_OOM.json (if out_dir given) + exit code 3. No retry (no expandable_segments).
* lock: ~/scanner/gpu.lock = JSON {owner, pid, host, time}. Held by a live pid and younger than 2 h -> wait (poll 30 s);
  dead pid or older than 2 h -> stale, taken over (noted on stderr). Released on exit.
"""
import contextlib, json, os, socket, sys, time
from pathlib import Path

LOCK = Path.home() / "scanner" / "gpu.lock"
MAX_AGE_S = 2 * 3600


def frac():
    try:
        f = float(os.environ.get("SCAN3D_GPU_FRAC", "0.75"))
    except ValueError:
        f = 0.75
    return max(0.1, min(f, 0.85))


def cap_torch():
    import torch
    if torch.cuda.is_available():
        torch.cuda.set_per_process_memory_fraction(frac(), 0)
        tot = torch.cuda.get_device_properties(0).total_memory / 2 ** 30
        print(f"[gpu_guard] torch memory cap {frac():.2f} x {tot:.1f} GB = {frac() * tot:.1f} GB", file=sys.stderr, flush=True)
        return frac() * tot
    return 0.0


def _alive(pid):
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, ValueError, TypeError):
        return False


def acquire(owner):
    while True:
        if LOCK.exists():
            try:
                d = json.loads(LOCK.read_text())
            except Exception:
                d = {}
            age = time.time() - float(d.get("time", 0))
            if d.get("pid") != os.getpid() and _alive(d.get("pid")) and age < MAX_AGE_S and d.get("host") == socket.gethostname():
                print(f"[gpu_guard] GPU held by {d.get('owner')} (pid {d.get('pid')}, {age / 60:.0f} min) -> waiting",
                      file=sys.stderr, flush=True)
                time.sleep(float(os.environ.get("SCAN3D_GPU_POLL_S", "30")))   # featloop: env poll interval (same lock rules)
                continue
            if d:
                print(f"[gpu_guard] stale lock taken over: {d}", file=sys.stderr, flush=True)
        tmp = LOCK.with_suffix(f".{os.getpid()}")
        tmp.write_text(json.dumps(dict(owner=owner, pid=os.getpid(), host=socket.gethostname(), time=time.time(),
                                       time_str=time.strftime("%F %T"))))
        os.replace(tmp, LOCK)
        time.sleep(0.5)
        try:
            if json.loads(LOCK.read_text()).get("pid") == os.getpid():
                return
        except Exception:
            pass


def release():
    try:
        if LOCK.exists() and json.loads(LOCK.read_text()).get("pid") == os.getpid():
            LOCK.unlink()
    except Exception:
        pass


@contextlib.contextmanager
def job(owner, out_dir=None, torch_cap=True):
    acquire(owner)
    try:
        if torch_cap:
            cap_torch()
        try:
            yield
        except BaseException as e:   # OOM -> clean failure, no retry
            msg = str(e)
            oom = type(e).__name__ == "OutOfMemoryError" or "out of memory" in msg.lower()
            if not oom:
                raise
            print(f"[gpu_guard] GPU out of memory under the cap {frac():.2f} -> run failed (no retry): {msg[:200]}",
                  file=sys.stderr, flush=True)
            if out_dir:
                Path(out_dir).mkdir(parents=True, exist_ok=True)
                (Path(out_dir) / "FAILED_OOM.json").write_text(json.dumps(dict(owner=owner, made=time.strftime("%F %T"),
                                                                               cap_frac=frac(), error=msg[:2000])))
            release()
            sys.exit(3)
    finally:
        release()
