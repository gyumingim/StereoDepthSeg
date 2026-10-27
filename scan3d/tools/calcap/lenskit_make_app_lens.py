"""Make the app lens files from a passed printed-board calibration (lenskit 2026-10-27). NEVER installs anything.

  venv_ffs/bin/python lenskit_make_app_lens.py WORK_DIR [--app-focus 0.5] [--lens05-root ~/scanner/StereoDepthSeg-lens05]
                      [--allow-focus-mismatch]

Refuses (exit 2) unless WORK_DIR/criteria.json says C1-C8 all PASS.
Lens = OpenCV k12_notan fit on A+B (WORK_DIR/opencv_AB.json: k1 k2 free, p1 p2 k3 = 0, OpenVINS radtan compatible) -
board data only, never VIO. Focus = median of the stills' measured focus (camera result metadata); must match the app
focus within 0.05 dpt (check_lens_focus.py), else stop.
Writes WORK_DIR/app_lens/std_1920x1080_f<focus>_print<date>.json and runs the lens05 generator
  <lens05>/scan3d/ov_bridge/android/make_phone_ov_main.py WORK_DIR/app_lens/phone_ov_main --lens-file <that> --no-focus-scale
(the same script that made the app's current 0.5 dpt set, runs/lens05_20261027). Then checks the new variants.json focus.
Installing on the phone is a separate human decision (runs/lens05_20261027/phone_install.sh shows how it was done).
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_lens_focus  # noqa: E402


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("work")
    ap.add_argument("--app-focus", type=float, default=0.5)
    ap.add_argument("--lens05-root", default=os.path.expanduser("~/scanner/StereoDepthSeg-lens05"))
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--allow-focus-mismatch", action="store_true")
    a = ap.parse_args()
    W = a.work
    cj = os.path.join(W, "criteria.json")
    crit = json.load(open(cj, encoding="utf-8"))
    if not crit.get("all_pass"):
        fails = {k: v["result"] for k, v in crit["criteria"].items() if v["result"] != "PASS"}
        print(f"합격 기준 통과 못 함 {fails} → 앱 렌즈 파일 만들지 않음")
        sys.exit(2)
    oc = json.load(open(os.path.join(W, "opencv_AB.json"), encoding="utf-8"))
    r = oc["models"]["k12_notan"]
    foc = []
    for t in ("A", "B"):
        for m in json.load(open(os.path.join(W, f"meta_{t}.json")))["images"]:
            if m.get("focus_dpt") is not None:
                foc.append(float(m["focus_dpt"]))
    if not foc:
        print("WARN 사진에 측정 초점 메타데이터가 없음")
        if not a.allow_focus_mismatch:
            sys.exit(3)
    fmed = float(np.median(foc)) if foc else a.app_focus
    out = os.path.join(W, "app_lens")
    os.makedirs(out, exist_ok=True)
    day = time.strftime("%Y%m%d")
    lp = os.path.join(out, f"std_1920x1080_f{round(fmed, 2):.2f}_print{day}.json")
    d = (list(r["dist"]) + [0.0] * 5)[:5]
    lens = dict(K=r["K"], dist=d, focus_dpt=round(fmed, 3), focus_dpt_measured_range=[round(min(foc), 3), round(max(foc), 3)] if foc else None,
                focus_dpt_app=a.app_focus, size=[1920, 1080], model="k1k2p", fit="k12_notan",
                fit_note="OpenCV calibrateCamera, k1 k2 free, p1 p2 k3 fixed 0 (OpenVINS radtan compatible), sets A+B together",
                rms_px=r["rms_px"], n_views=r["n_views"], n_poses=r["n_caps"],
                criteria="C1-C8 all PASS (runs/lenscal_20261027 README section 3)", criteria_file=os.path.abspath(cj),
                criteria_sha256=sha(cj), criteria_flags=dict(accept_small=crit.get("accept_small"),
                                                             flatness_source=crit.get("flatness_source")),
mrcal_AB=crit["per_set"]["AB"]["intrinsics"], warp_mm=crit["per_set"]["AB"]["warp_mm_x_y"],
                sets=crit["sets"], board=crit["grid"]["board"], status="candidate (printed board, passed C1-C8) - NOT installed",
                status_note="Chosen from board calibration only, never from VIO error. Installing = human decision.",
                made=time.strftime("%Y-%m-%d %H:%M:%S"))
    json.dump(lens, open(lp, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    print("렌즈 파일:", lp)
    ok = check_lens_focus.check(lp, app_focus=a.app_focus)
    if not ok and not a.allow_focus_mismatch:
        print("초점 불일치 → 앱 설정 세트 만들지 않음")
        sys.exit(3)
    gen = os.path.join(a.lens05_root, "scan3d", "ov_bridge", "android", "make_phone_ov_main.py")
    po = os.path.join(out, "phone_ov_main")
    cmd = [a.python, gen, po, "--lens-file", lp, "--focus-dpt", str(a.app_focus), "--no-focus-scale"]
    print("실행:", " ".join(cmd))
    rr = subprocess.run(cmd, cwd=a.lens05_root, capture_output=True, text=True)
    open(os.path.join(out, "make_phone_ov_main.log"), "w").write(rr.stdout + "\n" + rr.stderr)
    if rr.returncode != 0:
        print("생성 실패:", rr.stderr[-1500:])
        sys.exit(4)
    vj = os.path.join(po, "variants.json")
    ok2 = check_lens_focus.check(lp, variants=vj)
    print("앱 설정 세트:", po, "(설치 안 함)", "· 초점 검사", "OK" if ok2 else "WARN")
    sys.exit(0 if ok2 else 3)


if __name__ == "__main__":
    main()
