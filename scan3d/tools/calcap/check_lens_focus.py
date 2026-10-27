"""Lens file vs app focus / resolution check (lenskit 2026-10-27). Exit 1 and WARN lines on a mismatch.

  python check_lens_focus.py LENS.json [--app-focus 0.5 | --variants .../variants.json] [--frames <set>/*/frames.jsonl ...]
         [--size 1920x1080] [--tol 0.05]

Why: the app lens c4f60 was a 2.22 dpt calibration while the app ran at 0.5 dpt (lenscal_20261027 README: r0.8
off by ~8.7 px; k1 changes with focus). A lens file must say the focus it was calibrated at and the image size.
Checks: lens json has focus_dpt and size; |lens focus - app focus| <= tol; every still's measured focus (camera result
metadata) within tol of the lens focus; size equals --size.
"""
import argparse
import json
import sys

import numpy as np


def check(lens_path, app_focus=None, variants=None, frames=(), size="1920x1080", tol=0.05, out=print):
    L = json.load(open(lens_path, encoding="utf-8"))
    bad = []
    f = L.get("focus_dpt")
    sz = L.get("size")
    if f is None:
        bad.append("렌즈 파일에 focus_dpt 가 없음 (어느 초점에서 잰 값인지 모름)")
    if sz is None:
        bad.append("렌즈 파일에 size 가 없음")
    elif f"{sz[0]}x{sz[1]}" != size:
        bad.append(f"렌즈 해상도 {sz[0]}x{sz[1]} ≠ 앱 원본 {size}")
    if variants:
        v = json.load(open(variants, encoding="utf-8"))           # make_phone_ov_main.py output (app config set)
        app_focus = v.get("run_focus_dpt", app_focus)
        if v.get("lens_focus_dpt") is not None and abs(float(v["lens_focus_dpt"]) - float(app_focus)) > tol:
            bad.append(f"앱 설정 세트의 렌즈 초점 {v['lens_focus_dpt']} ≠ 실행 초점 {app_focus} (focus_scale {v.get('focus_scale')} 로 fx 만 보정 - 왜곡은 그대로)")
    if app_focus is not None and f is not None and abs(float(f) - float(app_focus)) > tol:
        bad.append(f"렌즈 초점 {f} dpt ≠ 앱 초점 {app_focus} dpt (허용 ±{tol}) - 초점이 다르면 왜곡·fx 가 달라짐")
    meas = []
    for fp in frames:
        for ln in open(fp, encoding="utf-8"):
            try:
                r = json.loads(ln)
            except ValueError:
                continue
            x = (r.get("result") or {}).get("focus_dpt")
            if x is not None:
                meas.append(float(x))
    if frames and not meas:
        bad.append("frames.jsonl 에 측정 초점(result.focus_dpt) 이 없음")
    if meas and f is not None:
        m = np.asarray(meas)
        if np.max(np.abs(m - float(f))) > tol:
            bad.append(f"사진 측정 초점 {m.min():.3f}–{m.max():.3f} dpt 가 렌즈 초점 {f} 와 ±{tol} 넘게 다름")
    for b in bad:
        out("WARN " + b)
    if not bad:
        out(f"OK 렌즈 초점 {f} dpt · 해상도 {sz} · 앱 초점 {app_focus} · 측정 사진 {len(meas)}장")
    return not bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("lens")
    ap.add_argument("--app-focus", type=float, default=None)
    ap.add_argument("--variants", default="")
    ap.add_argument("--frames", nargs="*", default=[])
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--tol", type=float, default=0.05)
    a = ap.parse_args()
    ok = check(a.lens, a.app_focus, a.variants or None, a.frames, a.size, a.tol)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
