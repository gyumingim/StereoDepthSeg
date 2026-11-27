"""gsmap 2026-11-27: frame-rate and recording-profile helpers for the FFS pair planners (user decision: next recording at 30 fps).

Frame-rate: every frame-count rule in the planners is written in SECONDS and turned into frames with the recording's real rate.
  rec_fps(rec, t_ns): nominal rate from REC/meta.json (save_fps, else fps), checked against the median frame interval of t_ns
  (frame timestamps, e.g. lensonly/gt.npz t_ns = frames_cam5.csv t_mid). If they differ by > 5 % the measured rate is used (warning).
  No meta and no timestamps -> error (no silent 60 fps default). 121801: meta 60, median dt 16.67 ms -> 60 -> win 2 s = 120 frames,
  min partner gap 0.2 s = 12 frames (= the old hard-coded 60 fps numbers, so 121801 plans are unchanged).
  frames(sec, fps) = round(sec * fps) (window half-width); gap_frames(sec, fps) = max(1, round(sec * fps)) (min partner spacing).

Profile (low-angle "backdoors" = partners taken while walking forward: pairs3 low tier, gsm_ffs_relax partners, --fb-perp fallback):
  legacy : old behaviour, every backdoor as the arguments ask (121801 and every recording made before 2026-11-27).
  new    : backdoors OFF (pair diagnostic 11-27: low-angle pairs carry 1.0-1.4 px disparity error vs 0.16-0.21 px sideways).
  auto   : (default) legacy if REC/meta.json start_wall_ms is before 2026-11-27 00:00 KST, else new.
"""
import json, sys
from pathlib import Path

import numpy as np

NEW_METHOD_FROM_MS = 1795705200000      # 2026-11-27 00:00:00 KST (UTC+9) in Unix ms: recordings from here on use the new walking method


def _meta(rec):
    p = Path(rec) / "meta.json" if rec else None
    return json.loads(p.read_text()) if p and p.exists() else {}


def rec_fps(rec=None, t_ns=None, override=0.0, tol=0.05):
    if override and override > 0:
        return float(override)
    m = _meta(rec); nom = float(m.get("save_fps") or m.get("fps") or 0)
    meas = 0.0
    if t_ns is not None and len(t_ns) > 2:
        dt = np.diff(np.asarray(t_ns, np.int64)); dt = dt[dt > 0]
        if len(dt):
            meas = 1e9 / float(np.median(dt))
    if nom and meas:
        if abs(meas / nom - 1) > tol:
            print(f"[rec_fps] meta says {nom:g} fps but frame timestamps say {meas:.2f} fps -> using the timestamps", file=sys.stderr)
            return meas
        return nom
    if nom or meas:
        return nom or meas
    raise SystemExit(f"[rec_fps] no frame rate: no meta.json fps in {rec} and no frame timestamps")


def frames(sec, fps):
    return int(round(sec * fps))


def gap_frames(sec, fps):
    return max(1, int(round(sec * fps)))


def profile(rec, want="auto"):
    if want in ("legacy", "new"):
        return want
    if want != "auto":
        raise SystemExit(f"--profile must be auto|legacy|new, not {want}")
    ms_ = _meta(rec).get("start_wall_ms")
    if ms_ is None:
        print("[profile] no start_wall_ms in meta.json -> new (backdoors off)", file=sys.stderr)
        return "new"
    return "legacy" if int(ms_) < NEW_METHOD_FROM_MS else "new"


def add_args(ap):
    ap.add_argument("--fps", type=float, default=0.0, help="frame rate override (0 = from the recording: meta.json checked by timestamps)")
    ap.add_argument("--min-gap-s", type=float, default=0.2, help="min time between two partners of one keyframe (old: 12 frames @ 60 fps)")
    ap.add_argument("--profile", default="auto", help="auto|legacy|new; new = low-angle backdoors off (pairs3 low, relax, fb-perp)")
