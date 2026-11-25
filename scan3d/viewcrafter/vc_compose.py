#!/usr/bin/env python3
"""viewcrafter 2026-11-25: side-by-side videos + stills + numbers for the report (CPU only).

  ~/scanner/gsmap_env/venv/bin/python scan3d/viewcrafter/vc_compose.py WORK OUT_MEDIA_DIR

Per mode (sparse, single) one grid video (25 frames, 6 fps), every panel 512x320:
  row 1: condition render from DUSt3R PC | ViewCrafter on DUSt3R PC | gsA2 splat (same path, K864 crop, resized)
  row 2: condition render from our PC    | ViewCrafter on our PC    | input keyframes (static)
Red outline on the ViewCrafter panels = pixels that the point cloud did NOT cover in the condition render
(= fully generated / hallucinated there). Covered pixels are still re-drawn by the diffusion model.
Numbers (stats.json): per run hole fraction, PSNR of ViewCrafter vs its own condition render on covered pixels (frames 1..24,
how far the output drifts from the given geometry), and mean |frame_t - frame_t-1| in grey levels (motion + flicker; same
path for all, so only relative).
"""
import json, subprocess, sys
from pathlib import Path

import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX


def label(im, txt, sub=None):
    im = im.copy()
    cv2.rectangle(im, (0, 0), (512, 24), (0, 0, 0), -1)
    cv2.putText(im, txt, (6, 17), FONT, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    if sub:
        cv2.putText(im, sub, (6, 312), FONT, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    return im


def outline(im, cover):
    hole = (~cover).astype(np.uint8)
    hole = cv2.morphologyEx(hole, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cs, _ = cv2.findContours(hole, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cs = [c for c in cs if cv2.contourArea(c) > 30]
    im = im.copy(); cv2.drawContours(im, cs, -1, (0, 0, 255), 2, cv2.LINE_AA)
    return im


def psnr(a, b, m):
    if m.sum() < 100:
        return float("nan")
    e = ((a.astype(np.float32) - b.astype(np.float32)) ** 2)[m].mean()
    return float(10 * np.log10(255 ** 2 / max(e, 1e-6)))


def mp4(frames, path, fps=6):
    import imageio_ffmpeg
    h, w = frames[0].shape[:2]
    p = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}",
                          "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", "-movflags", "+faststart", str(path)],
                         stdin=subprocess.PIPE)
    for f in frames:
        p.stdin.write(np.ascontiguousarray(f).tobytes())
    p.stdin.close(); p.wait()


def main():
    wk, out = Path(sys.argv[1]), Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    tr = np.load(wk / "traj.npz"); meta = json.loads(str(tr["meta"])); off = meta["off_path_deg"]
    splat = [cv2.resize(cv2.imread(str(wk / "splat_gsA2" / "frames" / f"{j:02d}.png")), (512, 320), interpolation=cv2.INTER_AREA) for j in range(25)]
    stats = {"traj": meta, "runs": {}}
    sd = np.stack(splat).astype(np.float32).mean(-1)
    stats["splat_gsA2"] = dict(frame_diff=float(np.abs(np.diff(sd, axis=0)).mean()),
                               info=json.loads((wk / "splat_gsA2" / "info.json").read_text()))
    for mode in ("sparse", "single"):
        P = {}
        for pc in ("dust3r", "ours"):
            r = wk / f"{mode}_{pc}"
            f = np.load(r / "frames.npz"); info = json.loads((r / "info.json").read_text())
            rend = [cv2.cvtColor(x, cv2.COLOR_RGB2BGR) for x in f["render"]]; dif = [cv2.cvtColor(x, cv2.COLOR_RGB2BGR) for x in f["diffusion"]]
            cov = f["cover"]
            ps = [psnr(dif[j], rend[j], cov[j]) for j in range(1, 25)]
            dd = np.stack(dif).astype(np.float32).mean(-1)
            stats["runs"][f"{mode}_{pc}"] = dict(hole_frac=info["hole_frac"], psnr_vs_condition_covered=ps,
                                                 psnr_vs_condition_mean=float(np.nanmean(ps)), frame_diff=float(np.abs(np.diff(dd, axis=0)).mean()),
                                                 t=info["t"], peak_torch_alloc_gb=info["peak_torch_alloc_gb"],
                                                 peak_gpu_used_mib_nvsmi=info["peak_gpu_used_mib_nvsmi"], n_points=info["n_points"],
                                                 output=info["output"], sim3=info.get("sim3"), ours=info.get("ours"), input_kf=info["input_kf"])
            P[pc] = (rend, dif, cov, info)
        ins = [cv2.imread(str(wk / f"{mode}_dust3r" / "inputs" / f"{i}.png")) for i in range(len(P["dust3r"][3]["input_kf"]))]
        thumb = np.zeros((320, 512, 3), np.uint8)
        if len(ins) == 1:
            thumb = ins[0].copy()
        else:
            for k, im in enumerate(ins[:4]):
                y, x = divmod(k, 2); thumb[y * 160:(y + 1) * 160, x * 256:(x + 1) * 256] = cv2.resize(im, (256, 160))
        kfs = P["dust3r"][3]["input_kf"]
        frames = []
        for j in range(25):
            sub = f"frame {j}  off-path {off[j]:.0f} deg"
            r1 = [label(P["dust3r"][0][j], "condition: DUSt3R point cloud", sub),
                  label(outline(P["dust3r"][1][j], P["dust3r"][2][j]), "ViewCrafter (DUSt3R PC)  red = no geometry", sub),
                  label(splat[j], "gsA2 splat (ours)", sub)]
            r2 = [label(P["ours"][0][j], "condition: OUR point cloud (FFS+DA3)", sub),
                  label(outline(P["ours"][1][j], P["ours"][2][j]), "ViewCrafter (our PC)  red = no geometry", sub),
                  label(thumb, f"input keyframes {kfs} (frame 0 = {kfs[0]})")]
            frames.append(np.vstack([np.hstack(r1), np.hstack(r2)]))
        mp4(frames, out / f"grid_{mode}.mp4")
        for j in (0, 12, 24):
            cv2.imwrite(str(out / f"grid_{mode}_{j:02d}.jpg"), frames[j], [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(str(out / f"inputs_{mode}.jpg"), thumb, [cv2.IMWRITE_JPEG_QUALITY, 90])
    (out / "stats.json").write_text(json.dumps(stats, indent=1))
    for k, v in stats["runs"].items():
        print(k, "holes f12/f24 %.2f/%.2f" % (v["hole_frac"][12], v["hole_frac"][24]), "psnr_cond %.1f" % v["psnr_vs_condition_mean"],
              "fdiff %.2f" % v["frame_diff"], v["t"], v["peak_gpu_used_mib_nvsmi"])
    print("splat fdiff %.2f" % stats["splat_gsA2"]["frame_diff"])


if __name__ == "__main__":
    main()
