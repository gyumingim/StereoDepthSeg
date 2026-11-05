#!/usr/bin/env python3
"""scalehunt: perturbed copies of the live c4f60 config folder (README 3). One factor per folder.

  python sh_cfg.py BASE_C4F60_DIR OUT_ROOT
Writes OUT_ROOT/<cond>/{estimator_config.yaml,kalibr_imu_chain.yaml,kalibr_imucam_chain.yaml,remap_1920x1080.bin} + cfg_made.json.
"""
import hashlib, json, re, shutil, sys, time
from pathlib import Path
import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parents[1]
sys.path.insert(0, str(SCAN3D / "ov_bridge" / "android"))
sys.path.insert(0, str(SCAN3D / "imu_learn")); sys.path.insert(0, str(SCAN3D))
import make_phone_ov_main as mk  # noqa: E402
import lens_calib  # noqa: E402

LENS = SCAN3D / "config/lens/std_1920x1080_f0.50_calcap20261026.json"


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


def rot(axis, deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def read_T(cam_y):
    rows = re.findall(r"(?m)^\s*- \[([^\]]+)\]", cam_y.split("cam_overlaps")[0])
    return np.array([[float(v) for v in r.split(",")] for r in rows[:4]])


def write_T(cam_y, T):
    head, tail = cam_y.split("cam_overlaps", 1)
    lines = head.rstrip(" ").splitlines()
    k = [i for i, l in enumerate(lines) if re.match(r"^\s*- \[", l)][:4]
    for i, r in zip(k, T):
        lines[i] = "    - [" + ", ".join(f"{v:.12g}" for v in r) + "]"
    return "\n".join(lines) + "\n  cam_overlaps" + tail


def set_intr(cam_y, f):
    return re.sub(r"(?m)^(\s*intrinsics:).*$", r"\1 [" + ", ".join(f"{v:.6f}" for v in f) + "]", cam_y)


def main():
    base, root = Path(sys.argv[1]), Path(sys.argv[2])
    cam0 = (base / "kalibr_imucam_chain.yaml").read_text()
    intr = [float(v) for v in re.search(r"intrinsics:\s*\[([^\]]+)\]", cam0).group(1).split(",")]
    T0 = read_T(cam0)
    K, dist, _ = lens_calib.load(str(LENS))
    Knew, _ = cv2.getOptimalNewCameraMatrix(K, dist, mk.RAW_SIZE, 0, (960, 540))
    made = {}

    def mkdir(name):
        d = root / name
        if d.exists():
            shutil.rmtree(d)
        shutil.copytree(base, d)
        return d

    # sanity: regenerating the base remap with the real distortion must give the phone sha
    chk = root / "_remap_check.bin"; root.mkdir(parents=True, exist_ok=True)
    mk.write_remap(chk, K, dist, mk.RAW_SIZE, Knew, (960, 540))
    made["_remap_check"] = dict(sha=sha(chk), base_sha=sha(base / "remap_1920x1080.bin"))
    assert made["_remap_check"]["sha"] == made["_remap_check"]["base_sha"], made
    chk.unlink()

    def cam_edit(name, fn, note):
        d = mkdir(name)
        y = fn((d / "kalibr_imucam_chain.yaml").read_text())
        (d / "kalibr_imucam_chain.yaml").write_text(y)
        made[name] = dict(note=note, imucam_sha=sha(d / "kalibr_imucam_chain.yaml"))

    f = intr
    cam_edit("K_f+2", lambda y: set_intr(y, [f[0] * 1.02, f[1] * 1.02, f[2], f[3]]), "fx,fy x1.02")
    cam_edit("K_f-2", lambda y: set_intr(y, [f[0] * 0.98, f[1] * 0.98, f[2], f[3]]), "fx,fy x0.98")
    cam_edit("K_fx+2", lambda y: set_intr(y, [f[0] * 1.02, f[1], f[2], f[3]]), "fx x1.02 only")
    cam_edit("K_cx+5", lambda y: set_intr(y, [f[0], f[1], f[2] + 5, f[3]]), "cx +5 px (960 grid)")
    cam_edit("K_cy+5", lambda y: set_intr(y, [f[0], f[1], f[2], f[3] + 5]), "cy +5 px (960 grid)")
    for ax in "xyz":
        def fr(y, ax=ax):
            T = T0.copy(); T[:3, :3] = rot(ax, 1.0) @ T0[:3, :3]
            return write_T(y, T)
        cam_edit(f"E_r{ax}1", fr, f"R_CtoI rotated +1 deg about IMU {ax} (p_CinI kept)")
    def fp(y, dp):
        T = T0.copy(); T[:3, 3] = dp if dp is not None else 0
        return write_T(y, T)
    cam_edit("E_p0", lambda y: fp(y, None), "p_CinI = 0")
    for i, ax in enumerate("xyz"):
        dp = T0[:3, 3].copy(); dp[i] += 0.02
        cam_edit(f"E_p{ax}2", lambda y, dp=dp: fp(y, dp), f"p_CinI {ax} +2 cm")
    # distortion off: same K and Knew, dist 0
    d = mkdir("D_off")
    mk.write_remap(d / "remap_1920x1080.bin", K, np.zeros(5), mk.RAW_SIZE, Knew, (960, 540))
    made["D_off"] = dict(note="remap with dist = 0, same K/Knew", remap_sha=sha(d / "remap_1920x1080.bin"))
    made["base"] = dict(dir=str(base), imucam_sha=sha(base / "kalibr_imucam_chain.yaml"), remap_sha=sha(base / "remap_1920x1080.bin"),
                        intrinsics=intr, T_CtoI=T0.tolist(), Knew=Knew.tolist(), lens=str(LENS))
    json.dump(dict(made=time.strftime("%F %T"), conds=made), open(root / "cfg_made.json", "w"), indent=1)
    print(json.dumps(made, indent=1)[:3000])


if __name__ == "__main__":
    main()
